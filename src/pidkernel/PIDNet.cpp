/*
 * Archivo: sfunPID_kernel.c
 * Arquitectura Moderna del Controlador PID de SOMEFUN
 * oasomefun@futa.edu.ng            : 2019, 2020
 */

/* Archivos de Inclusión */
#include "PIDNet.h"
#include "helpers/norm++kernel.h" // Para normalize/denormalize si se usa, no directamente en este fragmento
#include <cmath> // Para fabs, fmax (a menudo vía Arduino.h pero es bueno ser explícito)

// Usar las constantes definidas en PIDNet.h
using namespace pid_net_constants;

/* Inicialización de Instancia */
PIDNet::PIDNet(double ref, double yout, double dt,
        int umax_lim, int umin_lim, int dead_max_val, int dead_min_val) {
    follow = 0;
    Ts = dt;
    T_prev = -Ts; // Inicializar tiempo previo para el primer cálculo de dt, asegurar que sea negativo de Ts
    countseq = 0;

    r = ref;
    y = yout;
    ym = 0; // Valor medido para PID, inicializado

    // Inicializar estados a cero
    e = 0.0;
    ei = 0.0; 
    ed = 0.0; 
    up = 0.0; 
    ui = 0.0; 
    ud = 0.0; 
    upd = 0.0;
    ua = 0.0; 
    v = 0.0;  
    u = 0.0;  
    uo = 0.0; 
    e_t = 0.0;
    uf = 0.0;
    filter_u.x = 0.0; // Resetear estado del filtro

    umax = umax_lim;
    umin = umin_lim;

    this->dead_max = dead_max_val; 
    this->dead_min = dead_min_val; 

    // Inicializar parámetros PID a valores por defecto
    Kp = KP_DEFAULT;
    Ki = KI_DEFAULT;
    Kd = KD_DEFAULT;
    lambdai = LAMBDA_I_DEFAULT;
    lambdad = LAMBDA_D_DEFAULT;
    Ti = TI_DEFAULT;
    Td = TD_DEFAULT;
    this->Tf = TF_FILTER_DEFAULT; // Inicializar miembro Tf (constante de tiempo del filtro para derivativo)
                                 // Esto se recalculará inmediatamente basado en dt.
    b = B_2DOF_DEFAULT;
    c = C_2DOF_DEFAULT;

    /* Discretización e Inicialización del Filtro para la Ruta Derivativa */
    // dt (Ts) debe ser mayor que cero para estos cálculos
    double safe_dt = (dt > DIV_BY_ZERO_EPSILON_PID) ? dt : DIV_BY_ZERO_EPSILON_PID;
    double cut_freq = (PI) / (CUT_FREQ_DIVISOR * safe_dt);

    // Constante bilineal pre-distorsionada (kpi) y constante de tiempo del filtro (Tf)
    // para el filtro paso bajo de primer orden aplicado al término derivativo (ud).
    // TAN_ST se define en filterFO_pass.h, incluido vía PIDNet.h
    kpi = cut_freq / (TAN_ST + DIV_BY_ZERO_EPSILON_PID); 
    this->Tf = Ts / (TF_CALC_DENOMINATOR_SCALING * TAN_ST + DIV_BY_ZERO_EPSILON_PID); // Actualizar miembro Tf

    // Inicializar el objeto filter_u.
    // filter_u es una instancia de filterFO_pass. Su constructor inicializa su propio
    // coeficiente Tf_kpi basado en una relación fija (implícitamente fc/fs = 1/20 vía TAN_ST).
    // Este filtro (filter_u) NO SE UTILIZA actualmente en el método PIDNet::compute()
    // para filtrar la salida final 'u' o 'uf'. La línea 'u = uf;' usa 'uf' derivado
    // de 'v - ua', no de filter_u.run().
    // Si filter_u se usara para la salida principal 'u', sus parámetros probablemente
    // necesitarían configurarse basados en PIDNet::Ts en lugar de su valor fijo por defecto.
    filter_u; 
    uf = 0;   // Inicializar salida ajustada por anti-windup y pre-saturada
}

/* Definiciones de Funciones */

void PIDNet::reset_state() {
    T_prev = -Ts; // Consistente con la lógica del constructor para el primer dt si t=0 se pasa a compute
    countseq = 0;
    ym = 0.0;   // O considerar r si follow está habilitado y r es conocido, pero 0.0 es un reseteo general
    e = 0.0;
    ei = 0.0;
    ed = 0.0;
    up = 0.0;
    ui = 0.0;
    ud = 0.0;
    upd = 0.0;
    ua = 0.0;
    v = 0.0;
    u = 0.0;
    uo = 0.0;
    e_t = 0.0;
    uf = 0.0;
    filter_u.x = 0.0; // Resetear estado interno del filtro no usado filter_u
}

void PIDNet::set_deadzone(int min_val, int max_val) {
    this->dead_min = min_val;
    this->dead_max = max_val;
}

/*
 * @brief Calcula la salida de control PID para el paso de tiempo actual.
 * @details Esta es la función principal de trabajo. Implementa el algoritmo PID 2-DOF
 *          usando una transformación bilineal para la discretización e incluye un mecanismo anti-windup.
 *
 *          La secuencia de cálculo dentro de una sola llamada a compute() es crítica:
 *          1. Determinar el punto de consigna para el cálculo (ym_calc).
 *          2. Inicialización Anti-Windup (AWU) y cálculo de la medición ficticia (yfict).
 *          3. Actualizar los estados Derivativo (ud) e Integral (ui) (usando valores del final del ciclo *previo* de compute).
 *          4. Calcular Errores Actuales (para el ciclo actual).
 *          5. Calcular Términos de Salida P, D (para el ciclo actual).
 *          6. Actualizar término I (segunda parte, usando errores actuales, ua, upd actual).
 *          7. Calcular Salida Total No Saturada y Aplicar Saturación y Zona Muerta.
 *
 * @param t Tiempo actual.
 */
void PIDNet::compute(const double& t) {

    // Variables locales para claridad en los cálculos
    double ym_calc;   // El valor de punto de consigna usado para el cálculo del error (ya sea `r` crudo o `this->ym` filtrado)
    double yfict;     // Medición ficticia, y ajustada por el término anti-windup ua
    double e_u;       // Error entre la salida saturada `u` y la salida no saturada `v` (u-v)
    double Keu;       // Ganancia anti-windup (recíproco de la constante de tiempo de seguimiento Tt)
    double ep_calc;   // Error proporcional para el cálculo del término P
    double kui_calc;  // Coeficiente anti-windup integral escalado (KUI_NUMERATOR / Ki)

    // Almacenar tiempo actual como tiempo previo para el cálculo de Ts de la siguiente iteración (si Ts puede cambiar, aunque se fija en el constructor)
    T_prev = t; 

    /* Etapa 1: Determinar Punto de Consigna para Cálculo (ym_calc) */
    // ym_calc es el punto de consigna usado para los cálculos internos del PID.
    // Si el modo 'follow' está habilitado, ym_calc se toma de this->ym (que podría ser una versión filtrada de r).
    // De lo contrario, ym_calc es el punto de consigna crudo 'r', y this->ym también se actualiza a 'r'.
    if (follow==1) {
        ym_calc = this->ym;
    } else {
        this->ym = r; 
        ym_calc = this->ym;
    }

    /* Etapa 2: Inicialización Anti-Windup (AWU) y Medición Ficticia */
    // e_u es la diferencia entre la salida de control saturada (u) del ciclo anterior
    // y la salida no saturada (v) del ciclo anterior.
    e_u = (u - v); 

    // Keu es la ganancia anti-windup, equivalente a 1/Tt (constante de tiempo de seguimiento).
    // Se calcula basándose en los parámetros PID actuales. Ts (this->Ts) debe ser > 0.
    double safe_Ts = (this->Ts > DIV_BY_ZERO_EPSILON_PID) ? this->Ts : DIV_BY_ZERO_EPSILON_PID;
    Keu = Kp + (KEU_KI_SCALING_FACTOR * safe_Ts * Ki) + ((KEU_KD_TS_SCALING_FACTOR / safe_Ts) * Kd);
    
    // ua es el término de corrección anti-windup. Este término representa cuánto necesita
    // ajustarse el estado interno del controlador debido a la saturación.
    ua = e_u / (Keu + DIV_BY_ZERO_EPSILON_PID); 
    
    // yfict es una medición "ficticia". Es la medición real 'y' ajustada por 'ua'.
    // Este yfict es lo que 'y' necesitaría ser para que la 'v' no saturada hubiera sido la
    // señal de control correcta (saturada) 'u'. Esto es una parte central del anti-windup por back-calculation/tracking.
    yfict = y + ua; 

    /* Etapa 3: Actualizar Estados D e I (usando valores del final del ciclo *previo*) */
    // Estos cálculos usan ed, ei, y upd tal como estaban al final de la llamada anterior a compute().
    
    /* Término D - Derivativo (parte de predicción basada en ed_prev) */
    // ud_prev = ud_ciclo_prev * (kpi*Tf - 1) - kpi*Td*ed_ciclo_prev
    // Nota: 'this->ud', 'this->Tf', 'this->Td', 'this->ed' se usan aquí.
    // 'this->ed' aún contiene el error derivativo del ciclo *previo* de compute.
    this->ud *= (kpi * this->Tf - 1.0); 
    this->ud -= kpi * this->Td * (this->ed);   
    
    /* Término I - Integral (primera parte de la actualización, usando ei_prev y upd_prev) */
    // ui_intermedio = ui_ciclo_prev + (1/(Ti*kpi)) * ei_ciclo_prev - (KUI_NUMERATOR/Ki) * upd_ciclo_prev
    // 'this->ui', 'this->Ti', 'this->ei', 'this->upd' se usan.
    // 'this->ei' y 'this->upd' contienen valores del ciclo *previo* de compute.
    kui_calc = KUI_NUMERATOR / (Ki + DIV_BY_ZERO_EPSILON_PID); // Coeficiente anti-windup/moldeado integral
    this->ui += (1.0 / (this->Ti * kpi + DIV_BY_ZERO_EPSILON_PID)) * (this->ei); 
    this->ui -= kui_calc * (this->upd); 

    /* Etapa 4: Calcular Errores Actuales (para el ciclo actual) */
    // ep_calc es el error proporcional, calculado usando ym_calc e yfict (y ajustado por ua).
    // 'b' es la ponderación del punto de consigna para el término proporcional (2-DOF).
    ep_calc = (static_cast<double>(b) * ym_calc) - yfict; 
    
    // e es el error principal del proceso (punto de consigna crudo - medición cruda).
    e = r - y;                                     
    
    // ei_actual (this->ei) es el error integral para el ciclo actual.
    // Se basa en la diferencia entre ym_calc e yfict (que incluye el efecto anti-windup 'ua').
    // Crucialmente, también suma de nuevo e_u (u-v, el error de saturación crudo).
    // Esta formulación específica significa que el error integral es impulsado por:
    // 1. (ym_calc - (y+ua)): Error relativo a la medición ajustada por el efecto de saturación.
    // 2. + (u-v):         Una alimentación directa adicional del error de saturación.
    // Esto podría estar destinado a hacer el anti-windup más responsivo o a incorporar
    // aspectos de diferentes estrategias anti-windup.
    this->ei = (ym_calc - yfict) + e_u;
    
    // ed_actual (this->ed) es el error derivativo para el ciclo actual.
    // 'c' es la ponderación del punto de consigna para el término derivativo (2-DOF).
    this->ed = (static_cast<double>(c) * ym_calc) - yfict; 

    /* Etapa 5: Calcular Términos de Salida P, D (para el ciclo actual) */
    /* Término P - Proporcional */
    up = (ep_calc); // Salida proporcional actual

    /* Término D - Derivativo (completar actualización para ciclo actual) */
    // ud_actual = (ud_intermedio + kpi*Td*ed_actual) / (kpi*Tf + 1)
    // 'this->ud' fue parcialmente actualizado en Etapa 3 usando ed_prev. Ahora se añade el componente ed_actual.
    this->ud += kpi * this->Td * (this->ed);
    // Aplicar filtro paso bajo de primer orden al término derivativo.
    this->ud = this->ud / (kpi * this->Tf + 1.0 + DIV_BY_ZERO_EPSILON_PID); 

    /* Suma del Término PD (para el ciclo actual) */
    // upd_actual es la suma de los términos proporcional y derivativo actuales.
    // Se usará en la segunda parte de la actualización del término integral.
    this->upd = (up + this->ud);

    /* Etapa 6: Actualizar término I (segunda parte, usando ei_actual, ua, upd_actual) */
    // ui_actual = ui_intermedio_de_etapa3 + (1/(Ti*kpi))*ei_actual - ua + (KUI_NUMERATOR/Ki)*upd_actual
    
    // Añadir efecto del error integral actual
    this->ui += (1.0 / (this->Ti * kpi + DIV_BY_ZERO_EPSILON_PID)) * (this->ei);
    
    // Restar corrección anti-windup 'ua' (derivada de e_u y Keu)
    this->ui -= ua;        
    
    // Añadir el segundo término 'kui * upd', esta vez usando el upd *actual*.
    // Los términos (KUI_NUMERATOR/Ki) * upd (uno restado con upd_prev, uno sumado con upd_actual)
    // representan una contribución proporcional al cambio en el término PD del ciclo previo al actual,
    // escalado por 1.5/Ki. Este es un mecanismo de retroalimentación específico dentro del cálculo integral,
    // potencialmente para moldear la respuesta o como parte de una interacción 2-DOF más compleja.
    // El factor 1.5 no es estándar para formas PID comunes y probablemente es específico del diseño de este algoritmo.
    this->ui += kui_calc * (this->upd); 

    /* Etapa 7: Calcular Salida Total No Saturada y Aplicar Saturación y Zona Muerta */
    // e_t es la señal de error total ponderada antes de escalar por Kp.
    // lambdai y lambdad son ponderaciones del punto de consigna para los términos I y D en el error total.
    e_t = (up + lambdai * (this->ui) + lambdad * (this->ud)); 
    v = Kp * e_t;  // Salida no saturada del controlador para el ciclo actual.
    
    // uf es la salida del controlador después de anti-windup (v-ua) pero antes de saturación.
    // Sin embargo, 'ua' ya se usó para ajustar 'yfict' y 'ui'.
    // El cálculo directo aquí es `uf = v - ua` (donde `ua` se basa en `u` y `v` del ciclo *anterior*).
    // Esto significa que `uf` es la salida no saturada actual `v` corregida por el efecto del error de saturación del ciclo *anterior*.
    uf = v - ua; 

    /* Saturación Dura */
    u = uf; // Asignar esta salida ajustada por anti-windup
    // Aplicar límites de saturación (umax, umin) a la salida de control u
    u = fmax(static_cast<double>(umin), fmin(u, static_cast<double>(umax)));
    uo = u; // uo almacena la salida de control saturada antes de la zona muerta

    // Activar funcionalidad de zona muerta
    // La función dead_zone modifica uo en el lugar.
    if (!(dead_min == 0 && dead_max == 0)) { // Aplicar zona muerta solo si los límites no son ambos cero
        dead_zone<double>(uo, this->dead_max, this->dead_min);
    }
    
    u = uo; // La salida final 'u' es el valor después de la aplicación de la zona muerta.

    countseq += 1; // Incrementar contador interno de muestras
}
/**************************************************************************/
/*!
    @brief  Establece tres parámetros en la estructura de control PID
    @param b_new Constante de ponderación del punto de consigna PID-2DOF para el término P: 0 o 1
    @param c_new Constante de ponderación del punto de consigna PID-2DOF para el término D: 0 o 1
    @param follow_new Lógica para habilitar el filtrado del punto de consigna: 0 o 1
    @returns void.
*/
/**************************************************************************/
void PIDNet::set_bc_follow(const int& b_new, const int& c_new, const char& follow_new) {
    this->b = b_new; 
    this->c = c_new; 
    this->follow = follow_new;
}

/*
 * Tráiler de archivo para sfunPID.cpp
 *
 * [EOF]
 */
