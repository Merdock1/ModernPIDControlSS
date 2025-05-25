/*
 * @file PIDNet.h
 * @author Oluwasegun Somefun (oasomefun@futa.edu.ng)
 * @brief Archivo de cabecera para la clase del controlador PID PIDNet.
 * @date 2019-2023
 * @copyright Copyright (c) 2020-2023 Oluwasegun Somefun
 */

#ifndef SFUNPID_H
#define SFUNPID_H

#include <Arduino.h> // Para tipos estándar, funciones matemáticas (fabs, fmax, PI) y copysign.
#include "cplmfc/filterFO_pass.h" // Para filtro de primer orden (filter_u) y constante TAN_ST.

/**
 * @namespace pid_net_constants
 * @brief Define constantes por defecto y valores de estabilidad numérica para el controlador PIDNet.
 */
namespace pid_net_constants {
    // Parámetros de sintonización PID por defecto
    static constexpr double KP_DEFAULT = 0.001;    /**< @brief Ganancia Proporcional (Kp) por defecto. */
    static constexpr double KI_DEFAULT = 0.0;      /**< @brief Ganancia Integral (Ki) por defecto. */
    static constexpr double KD_DEFAULT = 0.0;      /**< @brief Ganancia Derivativa (Kd) por defecto. */
    static constexpr double LAMBDA_I_DEFAULT = 1.0;/**< @brief Ponderación por defecto para el término integral en la señal de error total `e_t`. */
    static constexpr double LAMBDA_D_DEFAULT = 1.0;/**< @brief Ponderación por defecto para el término derivativo en la señal de error total `e_t`. */
    static constexpr double TI_DEFAULT = 100.0;    /**< @brief Constante de Tiempo Integral (Ti) por defecto en segundos. */
    static constexpr double TD_DEFAULT = 0.0;      /**< @brief Constante de Tiempo Derivativo (Td) por defecto en segundos. */
    static constexpr double TF_FILTER_DEFAULT = 0.5;/**< @brief Valor inicial por defecto para la constante de tiempo del filtro derivativo `Tf` (segundos). Se recalcula en el constructor. */
    static constexpr int B_2DOF_DEFAULT = 1;       /**< @brief Ponderación del punto de consigna por defecto para el término proporcional (parámetro `b` para PID 2-DOF). Típicamente 0 o 1. */
    static constexpr int C_2DOF_DEFAULT = 0;       /**< @brief Ponderación del punto de consigna por defecto para el término derivativo (parámetro `c` para PID 2-DOF). Típicamente 0 o 1. */

    // Constantes para cálculos del constructor relacionados con el filtro de la ruta derivativa
    static constexpr double CUT_FREQ_DIVISOR = 10.0;             /**< @brief Divisor usado para calcular `cut_freq` para el filtro derivativo: `cut_freq = PI / (CUT_FREQ_DIVISOR * dt)`. */
    static constexpr double TF_CALC_DENOMINATOR_SCALING = 2.0;   /**< @brief Factor de escala en el denominador para calcular `Tf`: `Tf = Ts / (TF_CALC_DENOMINATOR_SCALING * TAN_ST)`. */

    // Constantes para cálculos del método compute()
    static constexpr double KEU_KI_SCALING_FACTOR = 0.5;    /**< @brief Factor de escala para `Ts*Ki` en el cálculo de la ganancia anti-windup `Keu`. */
    static constexpr double KEU_KD_TS_SCALING_FACTOR = 2.0; /**< @brief Factor de escala para `Kd/Ts` en el cálculo de la ganancia anti-windup `Keu`. */
    static constexpr float KUI_NUMERATOR = 1.5F;           /**< @brief Numerador para el coeficiente `kui_calc` (`kui_calc = KUI_NUMERATOR / Ki`). Esto es parte de una lógica de actualización integral personalizada. */

    // Pequeño valor epsilon para prevenir división por cero en cálculos PID.
    static constexpr double DIV_BY_ZERO_EPSILON_PID = 1e-9; /**< @brief Una pequeña constante para prevenir la división por cero en cálculos sensibles. */
} // namespace pid_net_constants

/**
 * @class PIDNet
 * @brief Implementa un controlador PID (Proporcional-Integral-Derivativo) de 2-DOF con anti-windup.
 * @details Esta clase proporciona un algoritmo de control PID con características como:
 *          - Control de Dos Grados de Libertad (2-DOF) usando parámetros de ponderación de punto de consigna `b` y `c`.
 *          - Mecanismo anti-windup (back-calculation/tracking) para prevenir el "windup" integral durante la saturación.
 *          - Filtrado de primer orden para el término derivativo.
 *          - Discretización usando transformación bilineal.
 *          - Funcionalidad opcional de zona muerta en la salida final.
 *          - Capacidad de reseteo de estado.
 *          - Funciones "getter" para varias señales internas y parámetros.
 */
class PIDNet {
public:
    /**
     * @brief Constructor para el controlador PIDNet.
     * @details Inicializa parámetros PID, variables de estado, límites de saturación, límites de zona muerta,
     *          y calcula coeficientes para el filtro derivativo basados en el tiempo de muestreo `dt_init`.
     * @param ref_init Punto de consigna inicial (valor de referencia) para el controlador. Unidad: específica del proceso.
     * @param yout_init Valor inicial de la salida del proceso (medición). Unidad: específica del proceso.
     * @param dt_init Tiempo de muestreo (delta t) en segundos. Esto es crucial para cálculos en tiempo discreto.
     * @param umax_lim Límite de saturación superior para la salida de control `u`. Unidad: específica del actuador.
     * @param umin_lim Límite de saturación inferior para la salida de control `u`. Unidad: específica del actuador.
     * @param dead_max_lim Límite superior para la funcionalidad de zona muerta en la salida `uo`.
     *                     Si `uo` es positivo y `uo > dead_min` y `uo <= dead_max_lim`, `uo` se convierte en `dead_max_lim`.
     *                     Unidad: específica del actuador.
     * @param dead_min_lim Límite inferior para la funcionalidad de zona muerta en la salida `uo`.
     *                     Si `fabs(uo) <= dead_min_lim`, `uo` se convierte en 0. Unidad: específica del actuador (típicamente valor positivo).
     */
    explicit PIDNet(double ref_init,
                    double yout_init,
                    double dt_init,
                    int umax_lim,
                    int umin_lim,
                    int dead_max_lim,
                    int dead_min_lim
                    );

    /**
     * @brief Calcula la salida de control PID para el paso de tiempo actual.
     * @details Esta es la función principal de trabajo. Toma el tiempo actual, calcula errores,
     *          actualiza los términos P, I, y D, aplica anti-windup, saturación y zona muerta,
     *          y produce la salida de control final `u`.
     * @param t Tiempo de simulación actual en segundos. Usado para rastrear `T_prev`.
     */
    void compute(const double& t);

    /**
     * @brief Establece los parámetros de ponderación 2-DOF (Dos Grados de Libertad) y la habilitación del filtrado del punto de consigna.
     * @param b_new Ponderación del punto de consigna para el término proporcional (`b`). Típicamente 0 o 1.
     *              Si `b=1`, el término proporcional actúa sobre `r - yfict` (o `ym_calc - yfict`).
     *              Si `b=0`, el término proporcional actúa sobre `0 - yfict` (efectivamente sobre `-y` si `ym_calc` no se usa para el término P).
     * @param c_new Ponderación del punto de consigna para el término derivativo (`c`). Típicamente 0 o 1.
     *              Si `c=1`, el error derivativo `ed` se basa en `ym_calc - yfict`.
     *              Si `c=0`, el error derivativo `ed` se basa en `0 - yfict` (efectivamente sobre `-y`).
     * @param follow_new Indicador para habilitar el filtrado del punto de consigna para `ym`.
     *                   Si `follow_new = 1`, `ym_calc` en `compute()` usa `this->ym` (que puede ser filtrado externamente o por `cplmfc`).
     *                   Si `follow_new = 0`, `ym_calc` usa el punto de consigna crudo `r`.
     */
    void set_bc_follow(const int& b_new, const int& c_new, const char& follow_new);

    /**
     * @brief Reinicia los estados internos del controlador PID.
     * @details Establece el estado integral `ui`, estado derivativo `ud`, términos de error (`e`, `ei`, `ed`),
     *          componentes de salida (`up`, `v`, `u`, `uo`, `uf`, `ua`), y temporización/contadores internos
     *          (`T_prev`, `countseq`, `ym`) a sus estados iniciales cero o por defecto.
     *          Las ganancias PID (Kp, Ki, Kd, Ti, Td), el punto de consigna (r), la última medición (y), y los parámetros de configuración
     *          (b, c, follow, límites) NO son reiniciados por esta función.
     */
    void reset_state();

    /**
     * @brief Establece los límites de la zona muerta para la salida del controlador `uo`.
     * @details La zona muerta se aplica después de la saturación.
     * @param min_val El valor absoluto para el límite inferior de la zona muerta. Si `fabs(uo) <= min_val`, `uo` se convierte en 0.
     *                Típicamente un valor positivo.
     * @param max_val El valor absoluto para el límite superior de la zona muerta (o límite de zona muerta inversa).
     *                Si `fabs(uo) > min_val` y `fabs(uo) <= max_val`, `uo` se convierte en `copysign(max_val, uo)`.
     *                Típicamente un valor positivo mayor o igual a `min_val`.
     * @note Si tanto `min_val` como `max_val` son 0, la funcionalidad de zona muerta se deshabilita efectivamente.
     */
    void set_deadzone(int min_val, int max_val);


    // --- Funciones Getter ---
    /** @brief Obtiene la contribución proporcional a la salida de control (Kp * up). */
    double get_proportional_term_output() const { return Kp * up; }
    /** @brief Obtiene la contribución integral a la salida de control (Kp * lambdai * ui). */
    double get_integral_term_output() const { return Kp * lambdai * ui; }
    /** @brief Obtiene la contribución derivativa a la salida de control (Kp * lambdad * ud). */
    double get_derivative_term_output() const { return Kp * lambdad * ud; }
    /** @brief Obtiene la salida de control final `u` (después de saturación y zona muerta). */
    double get_control_output() const { return u; }
    /** @brief Obtiene la salida de control no saturada `v` (Kp * e_t, antes de saturación y zona muerta). */
    double get_unsaturated_control_output() const { return v; }
    /** @brief Obtiene el error principal del proceso (`e = r - y`). */
    double get_error() const { return e; }
    /** @brief Obtiene el componente de error integral actual `ei` (usado en el cálculo integral). */
    double get_integral_error_component() const { return ei; }
    /** @brief Obtiene el componente de error derivativo actual `ed` (usado en el cálculo derivativo, post-filtrado). */
    double get_derivative_error_component() const { return ed; }
    /** @brief Obtiene el componente proporcional crudo `up` (término de error para P, pre-escalado Kp). */
    double get_raw_proportional_component() const { return up; }
    /** @brief Obtiene el componente integral crudo `ui` (estado integral acumulado, pre-escalado Kp*lambdai). */
    double get_raw_integral_component() const { return ui; }
    /** @brief Obtiene el componente derivativo crudo `ud` (acción derivativa filtrada, pre-escalado Kp*lambdad). */
    double get_raw_derivative_component() const { return ud; }
    /** @brief Obtiene el tiempo de muestreo `Ts` en segundos. */
    double get_sampling_time() const { return Ts; }


    // --- Variables Miembro ---
    // Estas son públicas para acceso directo, común en algunos contextos embebidos por rendimiento o simplicidad.
    // Considere hacerlas privadas y usar getters/setters si se desea una encapsulación más estricta.
    
    char follow;    /**< @brief Indicador para habilitar (1) o deshabilitar (0) el modo de filtrado del punto de consigna. Si es 1, `ym` (potencialmente filtrado por `cplmfc`) se usa como punto de consigna para el cálculo del error. Si es 0, se usa `r` crudo. */
    double Ts;      /**< @brief Tiempo de muestreo en segundos. */
    double T_prev;  /**< @brief Tiempo de la llamada anterior a `compute()`, en segundos. Usado para cálculos internos si `dt` puede variar (aunque `Ts` se fija tras la construcción). */
    int countseq;   /**< @brief Contador interno de muestras, incrementado cada vez que se llama a `compute()`. */

    // --- Variables de Estado PID ---
    double r;       /**< @brief Punto de consigna (valor de referencia) actual. Unidad: específica del proceso. */
    double y;       /**< @brief Salida actual del proceso (valor medido). Unidad: específica del proceso. */
    double ym;      /**< @brief Valor medido usado para el cálculo PID. Puede ser una versión filtrada de `r` si `follow=1`, o `r` mismo. */
    double e;       /**< @brief Error principal del proceso (`e = r - y`). */
    double ei;      /**< @brief Estado del componente de error integral. Este es el valor que se alimenta al acumulador integral. */
    double ed;      /**< @brief Estado del componente de error derivativo. Este es el término de error usado para el cálculo derivativo, después de filtrar. */
    double e_t;     /**< @brief Señal de error total (`up + lambdai*ui + lambdad*ud`), calculada antes de escalar por `Kp`. */

    // --- Componentes de Salida PID ---
    double up;      /**< @brief Componente de salida del término proporcional (`ep_calc`), calculado antes de escalar por `Kp`. */
    double ui;      /**< @brief Componente de salida del término integral (estado integral acumulado), calculado antes de escalar por `Kp*lambdai`. */
    double ud;      /**< @brief Componente de salida del término derivativo (acción derivativa filtrada), calculado antes de escalar por `Kp*lambdad`. */
    double upd;     /**< @brief Suma de los términos proporcional y derivativo (`up + ud`). Usado en la actualización del término integral. */
    double ua;      /**< @brief Término de corrección anti-windup, calculado como `(u_saturada_prev - v_no_saturada_prev) / Keu`. */
    double v;       /**< @brief Salida no saturada del controlador (`v = Kp * e_t`), calculada antes de saturación y zona muerta. */
    double u;       /**< @brief Salida final del controlador después de aplicar saturación y zona muerta. Este es el valor a enviar al actuador. */
    double uo;      /**< @brief Salida intermedia después de saturación pero antes de la aplicación de la zona muerta. `uo` se alimenta a la función `dead_zone`. */

    // --- Límites de Saturación y Zona Muerta ---
    int umax;       /**< @brief Límite de saturación superior para la salida de control `u`. */
    int umin;       /**< @brief Límite de saturación inferior para la salida de control `u`. */
    int dead_max;   /**< @brief Límite superior para la funcionalidad de zona muerta aplicada a `uo`. Ver función `dead_zone`. */
    int dead_min;   /**< @brief Límite inferior para la funcionalidad de zona muerta aplicada a `uo`. Ver función `dead_zone`. */

    // --- Parámetros de Sintonización PID ---
    double Kp;      /**< @brief Ganancia proporcional. */
    double Ki;      /**< @brief Ganancia integral. Nota: `Ti` (Tiempo Integral) también es un parámetro; `Ki` es a menudo `Kp/Ti`. Asegurar consistencia si se ajusta manualmente. */
    double Kd;      /**< @brief Ganancia derivativa. Nota: `Td` (Tiempo Derivativo) también es un parámetro; `Kd` es a menudo `Kp*Td`. Asegurar consistencia si se ajusta manualmente. */
    double lambdai; /**< @brief Ponderación para el término integral en el cálculo de `e_t`. Típicamente 1.0. */
    double lambdad; /**< @brief Ponderación para el término derivativo en el cálculo de `e_t`. Típicamente 1.0. */
    double Ti;      /**< @brief Constante de tiempo integral en segundos. */
    double Td;      /**< @brief Constante de tiempo derivativa en segundos. */
    int b;          /**< @brief Ponderación del punto de consigna para el término proporcional (PID 2-DOF). `up` se basa en `b*ym_calc - yfict`. Valor: 0 o 1. */
    int c;          /**< @brief Ponderación del punto de consigna para el término derivativo (PID 2-DOF). `ed` se basa en `c*ym_calc - yfict`. Valor: 0 o 1. */

    // --- Parámetros del Filtro para la Ruta Derivativa ---
    double kpi;     /**< @brief Constante de transformación bilineal para el filtro paso bajo del término derivativo. Calculada en el constructor. */
    double Tf;      /**< @brief Constante de tiempo (segundos) para el filtro paso bajo de primer orden aplicado al término derivativo `ud`. Calculada en el constructor. */

    // --- Filtro de Salida (actualmente no usado en la ruta de salida final de compute) ---
    filterFO_pass filter_u; /**< @brief Un objeto de filtro de primer orden. @note Este filtro NO SE USA actualmente para procesar la salida final `u` o `uf` en el método `compute()`. Su estado se resetea con `reset_state()`. */
    double uf;              /**< @brief Señal intermedia en `compute()`: `v - ua` (salida no saturada `v` corregida por el efecto de saturación del ciclo anterior `ua`). Este `uf` luego se satura para obtener `u` (antes de la zona muerta). */
};


/**
 * @brief Aplica una zona muerta a una señal de entrada.
 * @details Esta función modifica la entrada `uin` en el lugar.
 *          - Si `fabs(uin) <= fabs(dead_min_val)`, `uin` se convierte en 0.
 *          - Si `fabs(uin) > fabs(dead_min_val)` y `fabs(uin) <= fabs(dead_max_val)`, `uin` se convierte en `copysign(static_cast<T>(dead_max_val), uin)`.
 *          - Si `fabs(uin) > fabs(dead_max_val)`, `uin` se convierte en `copysign(fabs(uin + static_cast<T>(dead_max_val)), uin)`. Este último caso implica un tipo de comportamiento de "zona muerta inversa con offset".
 * @tparam T El tipo de dato de la señal de entrada (p.ej., `double`, `float`).
 * @param[in,out] uin La señal de entrada a la que se aplica la zona muerta. Modificada en el lugar.
 * @param[in] dead_max_val El valor absoluto para el límite superior de la zona muerta (o límite de zona muerta inversa).
 * @param[in] dead_min_val El valor absoluto para el límite inferior de la zona muerta.
 * @note El comportamiento para `fabs(uin) > dead_max_val` es específico.
 *       La implementación actual es `uin = copysign(fabs(uin + static_cast<T>(dead_max_val)), uin)`.
 *       Si `uin` es positivo, p.ej., 5, y `dead_max_val` es 2: `fabs(5+2)=7`. `copysign(7, 5) = 7`.
 *       Si `uin` es negativo, p.ej., -5, y `dead_max_val` es 2: `fabs(-5+2)=3`. `copysign(3, -5) = -3`.
 *       El comentario "deadmax added as a disturbance, effect of the Coulomb friction in certain sense" (deadmax añadido como perturbación, efecto de la fricción de Coulomb en cierto sentido) sugiere un modelo específico.
 */
template<class T> // Tipo esperado es punto flotante
void dead_zone(T& uin, const int& dead_max_val, const int& dead_min_val); // Declaración


template<class T> // Definición
void dead_zone(T& uin, const int& dead_max_val, const int& dead_min_val) {
    if ((fabs(uin) <= fabs(dead_min_val))) {
        uin = 0;
    }
    else if ((fabs(uin) > fabs(dead_min_val)) && (fabs(uin) <= fabs(dead_max_val))) {
        uin = copysign(static_cast<T>(dead_max_val), uin); 
    }
    else { // fabs(uin) > fabs(dead_max_val)
        uin = copysign(fabs(uin + static_cast<T>(dead_max_val)), uin);
    }
}

#endif // SFUNPID_H

/*
  Tráiler de archivo para sfunPID.h
  [EOF]
*/