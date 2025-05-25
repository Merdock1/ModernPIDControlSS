/*
 * Archivo: CPLMFC.cpp
 *
 * <MODELO DE BUCLE PID CERRADO> <CONTROL DE SEGUIMIENTO> <MÉTODO> : 2020
 *
 * oasomefun@futa.edu.ng. Copyright.2020
 */

#include "cplmfc.h" 
#include <cmath> // Para fabs, fmax (implícitamente incluido vía Arduino.h usualmente, pero es buena práctica)

// Usando las constantes del espacio de nombres cplmfc_constants definido en cplmfc.h
using namespace cplmfc_constants;

/**************************************************************************/
/*!
    @brief Establece manualmente los hiperparámetros para el algoritmo de sintonización.
    @param Knet La instancia del controlador PID en el bucle.
    @param alpha Controla la ganancia proporcional.
    @param lambda_i Controla la contribución de salida del error integral.
    @param lambda_d Controla la contribución de salida del error derivativo.
    @returns void.
*/
/**************************************************************************/
void cplmfc::set_alpha_critics(PIDNet& Knet, const float& alpha, const float& lambda_i, const float& lambda_d) {
    this->alpha = alpha; // Almacena alpha, que influye en el cálculo de Kp
    Knet.lambdai = lambda_i; // Establece la ponderación integral en el controlador PID
    Knet.lambdad = lambda_d; // Establece la ponderación derivativa en el controlador PID
}
/**************************************************************************/
/*!
    @brief Establece el tiempo de estabilización y el ancho de banda del bucle cerrado.
    @param Knet La instancia del controlador PID en el bucle.
    @param N_ts El conteo de tiempo discreto para el tiempo de estabilización de la salida a controlar.
    @param N_taul El conteo de tiempo discreto para el retardo de entrada-salida en el bucle.
    @returns void.
*/
/**************************************************************************/
void cplmfc::begin(PIDNet& Knet, const int& N_ts, const int& N_taul) {
    ts = N_ts*Knet.Ts; // Calcula el tiempo de estabilización en segundos
    tau_l = float(N_taul*Knet.Ts); // Calcula el retardo de entrada-salida en segundos
    // Frecuencia natural = ancho de banda; tuneWn calcula 'wn' basado en 'ts' y parámetros PID.
    tuneWn(Knet);
}
/**************************************************************************/
/*!
    @brief Ejecuta el algoritmo de sintonización.
    @param Knet La instancia del controlador PID en el bucle.
    @param t El tiempo actual.
    @returns void.
*/
/**************************************************************************/
void cplmfc::run(PIDNet& Knet, const double& t) {
    // Filtra el punto de consigna 'Knet.r' usando el filtro de primer orden de coeficiente fijo 'filter_r'.
    // La salida Knet.ym es una versión suavizada de Knet.r, usada como el punto de consigna efectivo para los cálculos PID.
    // Nota: las características de filter_r están determinadas por su constructor por defecto (vía la clase filterFO_pass),
    // lo que significa que su frecuencia de corte es fija relativa a una tasa de muestreo implícita definida por TAN_ST,
    // y no se ajusta dinámicamente por el tiempo de muestreo del PIDNet (Knet.Ts).
    filter_r.run(Knet.ym, Knet.r);
    /* Cálculo de Sintonización CPLMFC */
    // Sintoniza Kp basado en el tiempo actual 't' y el modo '2' (usa NLSIG_N_KP_COMMON para nlsig)
    tuneKp(Knet, t, 2); 
    // Sintoniza Ki y Kd basado en el nuevo Kp y el 'wn' calculado
    tuneKiKd(Knet); 
}

// Calcula la frecuencia natural (wn) para la sintonización PID.
// Esta frecuencia es un parámetro clave para determinar Ki y Kd.
void cplmfc::tuneWn(PIDNet& Knet) {
    /* Funciones del algoritmo de sintonización */
    float xtsn; // Factor basado en la estructura PID (parámetros b y c)
    // La suma de Knet.b y Knet.c determina el tipo de respuesta PID
    // y por lo tanto el valor xtsn apropiado para el cálculo de wn.
    // Knet.b es la ponderación del punto de consigna para el término Proporcional
    // Knet.c es la ponderación del punto de consigna para el término Derivativo
    int xx = Knet.b+Knet.c; 
    if (xx==0) { // Corresponde a la estructura I-PD
        xtsn = XTSN_IPD;
    }
    else if (xx==1) { // Corresponde a la estructura PI-D o P-ID
        xtsn = XTSN_PID_PI_D;
    }
    else if (xx==2) { // Corresponde a la estructura PID (b=1, c=1)
        xtsn = XTSN_PID;
    }
    else { // Valor por defecto si b+c no es 0, 1, o 2
        xtsn = XTSN_DEFAULT;
    }

    /* w_n normalizado (frecuencia natural) */
    // wn se calcula usando xtsn, un factor de amortiguamiento (INV_SQRT_TWO), y el tiempo de estabilización (ts)
    // Esta fórmula se deriva de la teoría de control estándar para sistemas de segundo orden.
    wn = float(xtsn/(INV_SQRT_TWO * ts + DIV_BY_ZERO_EPSILON));

}

// Sintoniza la Ganancia Proporcional (Kp) del controlador PID.
// Utiliza una función sigmoide n-logística y una regla de actualización de Lyapunov.
void cplmfc::tuneKp(PIDNet& Knet, const double& t, const int& mode) {
    /* Actualización n-logística */
    // Función racional ajustada por curva para la programación de ganancia basada en el retardo y el tiempo de estabilización
    double k_sig1, k_sig2, xe, xu, x_lim, LL, kg, kp_lim, e_t_abs_local; // Renombrado e_t para evitar confusión con Knet.e_t
    
    // LL es una versión normalizada de tau_l (retardo de entrada-salida).
    // Las constantes KG_LL_MEAN y KG_LL_STD_DEV se usan para esta normalización y
    // probablemente se derivan empíricamente de la identificación del sistema o modelos de procesos específicos.
    LL = (tau_l - KG_LL_MEAN) / (KG_LL_STD_DEV + DIV_BY_ZERO_EPSILON);
    
    // Calcula 'kg', un factor de ganancia, usando una función racional de 'LL'.
    // Esta fórmula para 'kg':
    //   kg = (N2*LL^2 + N1*LL + N0) / (D2*LL^2 + D1*LL + D0) 
    //   (donde N2=KG_NUM_COEFF_LL2, N1=KG_NUM_COEFF_LL, N0=KG_NUM_COEFF_CONST,
    //    D2=1.0, D1=KG_DEN_COEFF_LL, D0=KG_DEN_COEFF_CONST)
    // parece ser una fórmula empírica o una derivada de un modelo/artículo específico.
    // Su origen exacto no es evidente solo a partir del código.
    // El rol de 'kg' es proporcionar un factor de escala adaptativo para el límite preliminar
    // de la ganancia proporcional (kp_lim) basado en el retardo de proceso normalizado 'LL'.
    // Esto permite que kp_lim se ajuste según las características del retardo del sistema.
    double kg_numerator = KG_NUM_COEFF_LL2 * LL * LL +
                          KG_NUM_COEFF_LL * LL +
                          KG_NUM_COEFF_CONST;
    double kg_denominator = LL * LL + // El coeficiente LL^2 del denominador es implícitamente 1.0
                            KG_DEN_COEFF_LL * LL +
                            KG_DEN_COEFF_CONST;
    kg = kg_numerator / (kg_denominator + DIV_BY_ZERO_EPSILON); // Evitar división por cero

    // kp_lim es el límite preliminar para Kp, escalado por 'alpha' (ganancia definida por el usuario),
    // 'kg' (factor dependiente del retardo), y constantes de tiempo del sistema (tau_l, ts).
    kp_lim = alpha*kg*(tau_l+ts)/(ts + DIV_BY_ZERO_EPSILON);

    // Calcular entradas para la función nlsig basadas en el error (Knet.e) y la salida de control (Knet.u)
    xe = kp_lim+Knet.e; // Límite superior para la entrada de error a nlsig
    xu = kp_lim+Knet.u; // Límite superior para la entrada de salida de control a nlsig
    
    // nlsig calcula ajustes de ganancia en forma de sigmoide (k_sig1, k_sig2)
    // basados en el error y la salida de control, limitados por +/- kp_lim.
    // Estos términos ayudan a modular Kp según el punto de operación actual.
    // Usando NLSIG_N_KP_COMMON para n, NLSIG_LAMBDA_KP_TUNING para lambda.
    // Otros parámetros (ymin=0.0, safety=0, isreverse=0) son valores por defecto de nlsig o de uso común.
    nlsig(k_sig1, Knet.e, xe, -xe, kp_lim, -kp_lim, 
          NLSIG_N_KP_COMMON, 
          NLSIG_LAMBDA_KP_TUNING, 0, 0);
    nlsig(k_sig2, Knet.u, xu, -xu, kp_lim, -kp_lim, 
          NLSIG_N_KP_COMMON, 
          NLSIG_LAMBDA_KP_TUNING, 0, 0);

    // x_lim es el límite superior efectivo para el cálculo de Kp usando nlsig.
    x_lim = kp_lim+(k_sig1+k_sig2); 
    
    // El parámetro 'mode' determina las características de la función nlsig usada para Kp.
    // Específicamente, afecta el parámetro 'n' de nlsig, que controla la pendiente/forma.
    if (mode==0) { // Modo 0 usa alpha (hiperparámetro) para influir en 'n' de nlsig
        // Si alpha es < NLSIG_N_KP_MODE_0_BASE_N, n será NLSIG_N_KP_MODE_0_BASE_N. De lo contrario, n es int(alpha).
        int n_val_mode0 = static_cast<int>(fmax(static_cast<double>(NLSIG_N_KP_MODE_0_BASE_N), static_cast<double>(alpha)));
        nlsig(Knet.Kp, (k_sig1+k_sig2), x_lim, 0.0, kp_lim,
                0.0, n_val_mode0, NLSIG_LAMBDA_KP_TUNING, 0, 0);
    }
    else if (mode==1) { // Modo 1 usa un 'n' fijo para nlsig
        nlsig(Knet.Kp, (k_sig1+k_sig2), x_lim, 0.0,
                kp_lim, 0.0, NLSIG_N_KP_MODE_1_AGGRESSIVE, 
                NLSIG_LAMBDA_KP_TUNING, 0, 0);
    }
    else if (mode==2) { // Modo 2 usa un 'n' fijo para nlsig (el más común en este contexto)
        nlsig(Knet.Kp, (k_sig1+k_sig2), x_lim, 0.0,
                kp_lim, 0.0, NLSIG_N_KP_COMMON, 
                NLSIG_LAMBDA_KP_TUNING, 0, 0);
    }
    else { // Por defecto al modo 2 si se proporciona un modo desconocido
        nlsig(Knet.Kp, (k_sig1+k_sig2), x_lim, 0.0,
                kp_lim, 0.0, NLSIG_N_KP_COMMON,
                NLSIG_LAMBDA_KP_TUNING, 0, 0);
    }

    // Opción adaptativa: regla de actualización de Lyapunov
    // Esta parte ajusta Kp adicionalmente si el tiempo actual 't' excede el retardo de entrada-salida 'tau_l'.
    // Esto sugiere un mecanismo de adaptación que se activa después de una fase inicial de respuesta del sistema.
    if ( fabs(t) > fabs(double (tau_l)) ) {
        e_t_abs_local = fabs(Knet.e_t); // Señal de error total absoluta del PID (suma de componentes P, I, D)
        
        // Limitar e_t_abs_local por Knet.umax. Esto es una heurística para prevenir actualizaciones excesivas de Kp
        // durante grandes eventos transitorios o cuando la salida del controlador ya está saturada.
        // Efectivamente reduce la sensibilidad de adaptación cuando el controlador opera
        // cerca de sus límites o experimenta grandes magnitudes de error general.
        e_t_abs_local = fmin(static_cast<double>(Knet.umax), e_t_abs_local);
        
        // Regla de Actualización Adaptativa de Kp:
        // Esta regla ajusta Kp en línea basándose en errores observados y actividad del controlador.
        // Se asemeja a una estrategia de adaptación basada en Lyapunov o una actualización simplificada basada en gradiente.
        // Componentes:
        //   - Knet.e: Error actual del proceso (r - y). Impulsa la dirección del cambio de Kp.
        //   - e_t_abs_local: Suma absoluta y limitada de los componentes PID. Actúa como una medida de la "actividad" del controlador
        //              o magnitud del error general, escalando la actualización.
        //   - alpha: Hiperparámetro definido por el usuario, escala la fuerza de adaptación general.
        //   - KP_ADAPTIVE_RULE_LEARNING_RATE: Una pequeña tasa de aprendizaje constante.
        // Propósito: Ajustar finamente Kp basándose en el rendimiento en tiempo de ejecución. Si el error persiste, Kp se ajusta.
        //
        // Nota sobre estabilidad: La adaptación continua de Kp sin límites o criterios de convergencia específicos
        // podría teóricamente llevar a que Kp crezca mucho o se vuelva inestable durante períodos prolongados,
        // especialmente con perturbaciones persistentes o dinámicas no modeladas. Esta es una observación general
        // para tales reglas adaptativas y puede requerir una sintonización cuidadosa de 'alpha' o lógica supervisora adicional
        // en sistemas prácticos de larga duración.
        Knet.Kp = Knet.Kp + (alpha * KP_ADAPTIVE_RULE_LEARNING_RATE * Knet.e) * e_t_abs_local;
    }

}

// Sintoniza el Tiempo Integral (Ti), Ganancia Integral (Ki),
// Tiempo Derivativo (Td), y Ganancia Derivativa (Kd) del controlador PID.
// Estos se calculan basándose en el Kp sintonizado y la frecuencia natural (wn).
void cplmfc::tuneKiKd(PIDNet& Knet) const {
    /* Constante de tiempo integral */
    // Ti se establece basándose en wn y un factor de amortiguamiento (INV_SQRT_TWO).
    // Ti = (TUNING_FACTOR_TI_NUM_BASE * INV_SQRT_TWO) / wn;
    Knet.Ti = (TUNING_FACTOR_TI_NUM_BASE * INV_SQRT_TWO) /
              (wn + DIV_BY_ZERO_EPSILON); // Evitar división por cero si wn es cero
    /* Ganancia integral */
    // Ki se calcula como Kp / Ti.
    Knet.Ki = Knet.Kp / (Knet.Ti + DIV_BY_ZERO_EPSILON); // Evitar división por cero si Ti es cero

    /* Constante de tiempo derivativa */
    // Td también se establece basándose en wn y un factor de amortiguamiento.
    // Td = 1.0 / (TUNING_FACTOR_TD_DEN_BASE * INV_SQRT_TWO * wn);
    Knet.Td = 1.0F / (TUNING_FACTOR_TD_DEN_BASE * INV_SQRT_TWO * 
                      (wn + DIV_BY_ZERO_EPSILON)); // Evitar división por cero si wn es cero
    /* Ganancia derivativa */
    // Kd se calcula como Kp * Td.
    Knet.Kd = Knet.Kp * Knet.Td;
}