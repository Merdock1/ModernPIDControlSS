/**
 * @file ModernPIDControlSS.h
 * @author Oluwasegun Somefun (oasomefun@futa.edu.ng, somefuno@oregonstate.edu)
 * @brief Archivo de inclusión principal para la librería de controlador PID ModernPIDControlSS para Arduino.
 * @details Esta librería proporciona un controlador PID (`PIDNet`) con características avanzadas como
 *          control 2-DOF, anti-windup, y un auto-sintonizador opcional (`cplmfc`).
 *          Está diseñada con teoría de control moderna y algoritmos de procesamiento de señales.
 *          Para más información, visite: http://github.com/somefunagba/ModernPIDControlSS
 * 
 * @section AGRADECIMIENTOS
 *          Se ha invertido tiempo y recursos en esta librería.
 *          ¡Por favor, apoye compartiendo y marcando como favorito en GitHub!
 *
 * @section HISTORIAL
 *          (El historial/versionado original iría aquí si estuviera disponible)
 */
#pragma once

#ifndef MODERNPIDCONTROLSS_MODERNPIDCONTROLSS_H
#define MODERNPIDCONTROLSS_MODERNPIDCONTROLSS_H

#include "pidkernel/PIDNet.h"
#include "cplmfc/filterFO_pass.h" // Incluido ya que PIDNet lo usa
#include "cplmfc/cplmfc.h"
#include "dynsys/testsys_ss.h"    // Sistema dinámico de ejemplo, puede no ser necesario para todos los usuarios
#include "helpers/norm++kernel.h" // Utilidades de ayuda, pueden no ser necesarias para todos los usuarios

// --- Funciones Principales del Kernel PID ---

/**
 * @brief Ejecuta un ciclo del bucle de control PID con auto-sintonización CPLMFC.
 * @details Esta función sirve como la interfaz principal para ejecutar el controlador PID
 *          cuando se usa el auto-sintonizador `cplmfc`. Comprueba si el intervalo de cálculo del PID
 *          (`Knet.Ts`) ha transcurrido desde el último cálculo (`Knet.T_prev`). Si es así,
 *          primero ejecuta el auto-sintonizador (`Tune.run()`) para actualizar las ganancias PID, luego
 *          calcula la nueva salida de control PID (`Knet.compute()`).
 *
 * @param[in,out] Knet La instancia del controlador `PIDNet`. Su estado y parámetros serán actualizados.
 * @param[in,out] Tune La instancia del auto-sintonizador `cplmfc`, configurada para `Knet`.
 * @param[in] t El tiempo de simulación actual en segundos.
 * @return int Un indicador que señala si el cálculo PID se realizó en esta llamada.
 *             Retorna 1 si se realizó el cálculo, 0 en caso contrario (p.ej., si `t` es demasiado temprano).
 * @note Asegúrese de que `Knet.Ts` (tiempo de muestreo) esté configurado correctamente en el constructor de `PIDNet`.
 *       `Knet.T_prev` se actualiza internamente por `Knet.compute()`.
 */
inline int PID_kernelOS(PIDNet& Knet, cplmfc& Tune, const double& t) {
    int flag = 0;
    // Comprueba si ha pasado suficiente tiempo para un nuevo ciclo PID
    // La condición permite ligeras imprecisiones de temporización (tolerancia de 0.5*Knet.Ts).
    if (t >= ((Knet.T_prev + Knet.Ts) - (0.5 * Knet.Ts))) {
        /* Evolución de Auto-sintonización CPLMFC */
        Tune.run(Knet, t); // Actualiza Kp, Ki, Kd en Knet
        /* Evolución del Estado de Control PID: Arquitectura */
        Knet.compute(t);   // Calcula la nueva salida de control u
        flag = 1;
    }
    return flag;
}

/**
 * @brief Ejecuta un ciclo del bucle de control PID con ganancias Kp, Ti, Td especificadas manualmente.
 * @details Esta sobrecarga permite usar el controlador `PIDNet` con ganancias establecidas manualmente
 *          (Ganancia Proporcional Kp, Tiempo Integral Ti, Tiempo Derivativo Td).
 *          Calcula `Ki = Kp/Ti` y `Kd = Kp*Td` internamente.
 *          Similar a la versión con auto-sintonización, comprueba la condición de temporización antes de calcular.
 * 
 * @param[in,out] Knet La instancia del controlador `PIDNet`. Sus ganancias (Kp, Ki, Kd, Ti, Td) serán establecidas, y su estado actualizado.
 * @param[in] t El tiempo de simulación actual en segundos.
 * @param[in] Kp La Ganancia Proporcional.
 * @param[in] Ti La constante de Tiempo Integral en segundos.
 * @param[in] Td La constante de Tiempo Derivativo en segundos.
 * @return char Un indicador que señala si el cálculo PID se realizó.
 *              Retorna 1 si se realizó el cálculo, 0 en caso contrario.
 * @note `Ki` y `Kd` se derivan de `Kp, Ti, Td`. Un `Ti` pequeño puede llevar a un `Ki` grande.
 *       Asegúrese de que `Ti` no sea cero para prevenir la división por cero.
 */
inline char PID_kernelOS(PIDNet& Knet, const double& t, const float& Kp, const float& Ti, const float& Td) {
    char flag = 0;
    /* Establecer Ganancias Manualmente */
    Knet.Kp = Kp;
    Knet.Ti = Ti; // Almacenar Ti
    Knet.Td = Td; // Almacenar Td
    // Calcular Ki y Kd a partir de Kp, Ti, Td
    if (fabs(Ti) > pid_net_constants::DIV_BY_ZERO_EPSILON_PID) { // Usar épsilon de las constantes de PIDNet
        Knet.Ki = Kp / Ti;
    } else {
        Knet.Ki = (Kp > 0) ? std::numeric_limits<double>::max() : std::numeric_limits<double>::lowest(); // Evitar NaN, representar Ki grande
    }
    Knet.Kd = Kp * Td;

    if (t >= ((Knet.T_prev + Knet.Ts) - (0.5 * Knet.Ts))) {
        /* Evolución del Estado de Control PID: Arquitectura */
        Knet.compute(t);
        flag = 1;
    }
    return flag;
}

/**
 * @brief Ejecuta un ciclo del bucle de control PID con ganancias Kp, Ki, Kd especificadas manualmente.
 * @details Esta sobrecarga permite usar el controlador `PIDNet` con ganancias establecidas manualmente
 *          (Proporcional Kp, Integral Ki, Derivativo Kd).
 *          Calcula `Ti = Kp/Ki` y `Td = Kd/Kp` internamente.
 *          Similar a otras versiones, comprueba la condición de temporización antes de calcular.
 *
 * @param[in,out] Knet La instancia del controlador `PIDNet`. Sus ganancias (Kp, Ki, Kd, Ti, Td) serán establecidas, y su estado actualizado.
 * @param[in] t El tiempo de simulación actual en segundos.
 * @param[in] Kp La Ganancia Proporcional.
 * @param[in] Ki La Ganancia Integral.
 * @param[in] Kd La Ganancia Derivativa.
 * @param[in] dummy_flag Un argumento char ficticio para diferenciar esta sobrecarga de la versión (Kp, Ti, Td).
 *                     Su valor no se usa. Establecer a cualquier char, p.ej., `0`.
 * @return char Un indicador que señala si el cálculo PID se realizó.
 *              Retorna 1 si se realizó el cálculo, 0 en caso contrario.
 * @note `Ti` y `Td` se derivan de `Kp, Ki, Kd`. Un `Ki` pequeño puede llevar a un `Ti` grande.
 *       Asegúrese de que `Ki` y `Kp` (para el cálculo de Td) no sean cero donde se usan como divisores.
 */
inline char PID_kernelOS(PIDNet& Knet, const double& t, const float& Kp, const float& Ki, const float& Kd, char /*dummy_flag*/) {
    char flag = 0; 
    /* Establecer Ganancias Manualmente */
    Knet.Kp = Kp;
    Knet.Ki = Ki;
    Knet.Kd = Kd;
    // Calcular Ti y Td a partir de Kp, Ki, Kd
    if (fabs(Ki) > pid_net_constants::DIV_BY_ZERO_EPSILON_PID) { // Usar épsilon de las constantes de PIDNet
        Knet.Ti = Kp / Ki;
    } else {
        Knet.Ti = std::numeric_limits<double>::max(); // Representar Ti grande
    }
    if (fabs(Kp) > pid_net_constants::DIV_BY_ZERO_EPSILON_PID) { // Usar épsilon de las constantes de PIDNet
        Knet.Td = Kd / Kp;
    } else {
        Knet.Td = (Kd > 0) ? std::numeric_limits<double>::max() : std::numeric_limits<double>::lowest(); // Representar Td grande/pequeño basado en el signo de Kd
    }

    if (t >= ((Knet.T_prev + Knet.Ts) - (0.5 * Knet.Ts))) {
        /* Evolución del Estado de Control PID: Arquitectura */
        Knet.compute(t);
        flag = 1;
    }
    return flag;
}

#endif //MODERNPIDCONTROLSS_MODERNPIDCONTROLSS_H
