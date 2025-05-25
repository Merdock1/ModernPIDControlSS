/*
 * Archivo: CPLMFC.h
 *
 * <MODELO DE BUCLE PID CERRADO> <CONTROL DE SEGUIMIENTO> <MÉTODO> : 2019-2020
 * Este archivo de cabecera define la clase cplmfc (Closed PID Loop Model Following Control - Control de Seguimiento de Modelo de Bucle PID Cerrado),
 * que es responsable del autoajuste de un controlador PIDNet.
 *
 * oasomefun@futa.edu.ng. Copyright.2020
 */

#ifndef CPLMFC_H
#define CPLMFC_H

/* Archivos de Inclusión */
#include <Arduino.h> // Para tipos y funciones básicas de Arduino como fmax, PI
#include "pidkernel/PIDNet.h" // Definición de la clase PIDNet (el controlador a sintonizar)
#include "cplmfc/filterFO_pass.h" // Definición para el filtro de primer orden usado para filtrar el punto de consigna
#include "nlsig/nlsig.h" // Para la función sigmoide N-logística usada en la sintonización de Kp

/**
 * @namespace cplmfc_constants
 * @brief Define constantes usadas en el algoritmo de autoajuste CPLMFC.
 */
namespace cplmfc_constants {
    // Factores para el cálculo de xtsn en tuneWn basados en la estructura PID (Knet.b + Knet.c)
    // Estos valores determinan la relación entre el tiempo de estabilización y la frecuencia natural
    // para diferentes configuraciones PID (I-PD, PI-D/P-ID, PID).
    static constexpr float XTSN_IPD = 9.98F;          /**< @brief Factor para estructura I-PD (b=0, c=0). */
    static constexpr float XTSN_PID_PI_D = 7.74F;     /**< @brief Factor para estructura PI-D o P-ID (b+c=1). */
    static constexpr float XTSN_PID = 11.07F;         /**< @brief Factor para estructura PID (b=1, c=1). */
    static constexpr float XTSN_DEFAULT = 8.0F;       /**< @brief Valor xtsn por defecto si la estructura PID no está definida. */

    // Constante para 1/sqrt(2), a menudo usada para características de respuesta críticamente amortiguada o Butterworth.
    static constexpr float INV_SQRT_TWO = 0.70710678118F; /**< @brief Valor de 1/sqrt(2), factor de amortiguamiento común. */

    // Constantes para el cálculo de 'kg' en tuneKp. Son coeficientes de una
    // función racional usada para determinar una ganancia adaptativa 'kg' basada en el retardo de bucle normalizado 'LL'.
    // LL = (tau_l - KG_LL_MEAN) / KG_LL_STD_DEV
    // kg = (KG_NUM_COEFF_LL2*LL^2 + KG_NUM_COEFF_LL*LL + KG_NUM_COEFF_CONST) /
    //      (LL^2 + KG_DEN_COEFF_LL*LL + KG_DEN_COEFF_CONST)
    static constexpr float KG_LL_MEAN = 5.936F;           /**< @brief Valor medio para normalizar tau_l en el cálculo de 'kg'. */
    static constexpr float KG_LL_STD_DEV = 8.771F;        /**< @brief Desviación estándar para normalizar tau_l en el cálculo de 'kg'. */
    static constexpr float KG_NUM_COEFF_LL2 = 0.05132F;   /**< @brief Coeficiente del numerador para el término LL^2 en el cálculo de 'kg'. */
    static constexpr float KG_NUM_COEFF_LL = 0.2041F;     /**< @brief Coeficiente del numerador para el término LL en el cálculo de 'kg'. */
    static constexpr float KG_NUM_COEFF_CONST = 0.1214F;  /**< @brief Término constante del numerador en el cálculo de 'kg'. */
    // El coeficiente LL^2 del denominador es implícitamente 1.0
    static constexpr float KG_DEN_COEFF_LL = 1.538F;      /**< @brief Coeficiente del denominador para el término LL en el cálculo de 'kg'. */
    static constexpr float KG_DEN_COEFF_CONST = 0.5864F;  /**< @brief Término constante del denominador en el cálculo de 'kg'. */

    // Tasa de aprendizaje para la regla de actualización adaptativa de Kp (adaptación basada en Lyapunov).
    static constexpr double KP_ADAPTIVE_RULE_LEARNING_RATE = 0.001; /**< @brief Tasa de aprendizaje para la regla adaptativa de Kp. */

    // Factor usado en el cálculo de Ti: Ti = (TUNING_FACTOR_TI_NUM_BASE * INV_SQRT_TWO) / wn
    static constexpr float TUNING_FACTOR_TI_NUM_BASE = 2.0F; /**< @brief Factor base del numerador para el cálculo de Ti. */
    // Factor usado en el cálculo de Td: Td = 1.0F / (TUNING_FACTOR_TD_DEN_BASE * INV_SQRT_TWO * wn)
    static constexpr float TUNING_FACTOR_TD_DEN_BASE = 2.0F; /**< @brief Factor base de escala del denominador para el cálculo de Td. */

    // Parámetros de la función nlsig específicos para su uso en el método tuneKp de cplmfc.
    static constexpr float NLSIG_LAMBDA_KP_TUNING = 0.1F; /**< @brief Lambda (tasa de crecimiento) para llamadas a nlsig en tuneKp. */
    // Parámetro 'n' (número de segmentos logísticos) para nlsig en diferentes modos de tuneKp:
    static constexpr int NLSIG_N_KP_COMMON = 1; /**< @brief 'n' común para k_sig1, k_sig2 iniciales y modo de sintonización Kp estable por defecto. */
    static constexpr int NLSIG_N_KP_MODE_0_BASE_N = 1; /**< @brief 'n' base para el modo 0 de sintonización Kp; 'n' real es fmax(this, alpha). */
    static constexpr int NLSIG_N_KP_MODE_1_AGGRESSIVE = 16; /**< @brief 'n' para el modo 1 de sintonización Kp agresivo. */
        
    static constexpr double DIV_BY_ZERO_EPSILON = 1e-9; /**< @brief Pequeño valor epsilon para prevenir la división por cero. */

} // namespace cplmfc_constants

/**
 * @class cplmfc
 * @brief Implementa el algoritmo de autoajuste CPLMFC (Closed PID Loop Model Following Control - Control de Seguimiento de Modelo de Bucle PID Cerrado).
 * @details Esta clase proporciona métodos para sintonizar automáticamente los parámetros (Kp, Ki, Kd) de un controlador `PIDNet`.
 * Utiliza características del sistema como el tiempo de estabilización y el retardo de entrada-salida, junto con
 * el error y la salida en tiempo de ejecución, para ajustar las ganancias PID. Incluye un filtro de punto de consigna y utiliza una función
 * sigmoide N-logística para la adaptación de la ganancia.
 */
class cplmfc{

public:
    /**
     * @brief Inicializa el sintonizador CPLMFC con las características del sistema.
     * @details Calcula el tiempo de estabilización `ts` y el retardo de entrada-salida `tau_l` en segundos basándose en
     * las cuentas de tiempo discreto proporcionadas y el tiempo de muestreo del controlador PID `Knet.Ts`.
     * Luego llama a `tuneWn()` para calcular la frecuencia natural inicial `wn`.
     * @param Knet Referencia a la instancia del controlador `PIDNet` a sintonizar.
     * @param N_ts El tiempo de estabilización deseado del sistema en bucle cerrado, expresado como un número de intervalos de muestreo PID (`Knet.Ts`).
     * @param N_taul Opcional. El retardo de entrada-salida (tiempo muerto) del sistema, expresado como un número de intervalos de muestreo PID. Por defecto es 0.
     */
    void begin(PIDNet& Knet, const int& N_ts, const int& N_taul = 0);

    /**
     * @brief Establece hiperparámetros para el algoritmo de sintonización CPLMFC.
     * @details Estos parámetros influyen en el comportamiento y la agresividad del auto-sintonizador.
     * @param Knet Referencia a la instancia del controlador `PIDNet`.
     * @param alpha Hiperparámetro que influye principalmente en el cálculo de la ganancia proporcional (Kp).
     *              Valores mayores generalmente conducen a una sintonización de Kp más agresiva.
     * @param lambda_i Opcional. Ponderación para la contribución integral en la señal de error total (`e_t`) del controlador `PIDNet`. Por defecto es 0.5.
     *                   Este es un parámetro de `PIDNet`, pero se establece a través del sintonizador por conveniencia.
     * @param lambda_d Opcional. Ponderación para la contribución derivativa en la señal de error total (`e_t`) del controlador `PIDNet`. Por defecto es 0.1.
     *                   Este es un parámetro de `PIDNet`, pero se establece a través del sintonizador por conveniencia.
     */
    void set_alpha_critics(PIDNet& Knet, const float& alpha, const float& lambda_i = 0.5F, const float& lambda_d = 0.1F);

    /**
     * @brief Ejecuta un paso del algoritmo de autoajuste CPLMFC.
     * @details Esta función debe llamarse periódicamente a la misma tasa que el controlador PID.
     * Primero filtra el punto de consigna `Knet.r` para producir `Knet.ym` usando `filter_r`.
     * Luego, llama a `tuneKp()` y `tuneKiKd()` para actualizar las ganancias PID en `Knet`.
     * @param Knet Referencia a la instancia del controlador `PIDNet` que se está sintonizando.
     * @param t Tiempo de simulación actual en segundos. Usado por la regla de actualización adaptativa de Kp.
     */
    void run(PIDNet& Knet, const double& t);

    // --- Funciones Auxiliares de Sintonización (públicas para posible inspección o uso avanzado, pero típicamente llamadas por run()) ---
    /**
     * @brief Calcula la frecuencia natural (`wn`) para la sintonización PID.
     * @details `wn` es un parámetro clave derivado del tiempo de estabilización deseado (`ts`) y la estructura PID
     * (determinada por `Knet.b` y `Knet.c`). Esta frecuencia natural se usa luego para calcular `Ti` y `Td`.
     * @param Knet Referencia a la instancia del controlador `PIDNet`.
     */
    void tuneWn(PIDNet& Knet);

    /**
     * @brief Sintoniza la ganancia proporcional (Kp) del controlador `PIDNet`.
     * @details Utiliza una función sigmoide N-logística basada en el error y la salida de control, y una
     * regla de actualización adaptativa basada en Lyapunov. El parámetro `mode` selecciona diferentes configuraciones
     * para la sigmoide N-logística, afectando su forma y agresividad.
     * @param Knet Referencia a la instancia del controlador `PIDNet`.
     * @param t Tiempo de simulación actual en segundos, usado para la actualización adaptativa de Lyapunov.
     * @param mode Entero que selecciona la configuración específica de nlsig para el cálculo de Kp.
     *             Típicamente, el modo 2 (usando `NLSIG_N_KP_COMMON`) es usado por `run()`.
     */
    void tuneKp(PIDNet& Knet, const double& t, const int& mode);
    
    /**
     * @brief Sintoniza la ganancia integral (Ki) y la ganancia derivativa (Kd) basadas en Kp y `wn`.
     * @details También calcula y establece la constante de tiempo integral (`Ti = Kp/Ki`) y
     * la constante de tiempo derivativa (`Td = Kd/Kp`) en la instancia `PIDNet`.
     * @param Knet Referencia a la instancia del controlador `PIDNet`.
     * @note Esta función es `const` ya que modifica `Knet` a través de su referencia pero no los miembros del objeto `cplmfc` directamente relacionados con este cálculo.
     */
    void tuneKiKd(PIDNet& Knet) const;

    // --- Variables Miembro ---
    float wn = 1.0F;    /**< @brief Frecuencia natural calculada (rad/s) de la respuesta deseada del sistema en bucle cerrado. */
    float alpha = 1.0F; /**< @brief Hiperparámetro para la sintonización de Kp, afecta la agresividad. Establecido vía `set_alpha_critics`. */
    double ts = 1.0F;   /**< @brief Tiempo de estabilización deseado (segundos) para la respuesta del sistema. Calculado en `begin`. */
    float tau_l = 0.0F; /**< @brief Retardo de entrada-salida del sistema (segundos), o retardo de bucle. Calculado en `begin`. */
    
    /**
     * @brief Filtro de primer orden para el punto de consigna.
     * @details Este filtro (`filter_r`) es una instancia de `filterFO_pass`. Se usa en `cplmfc::run()`
     * para filtrar el punto de consigna crudo `Knet.r` para producir `Knet.ym`, que luego es utilizado por el controlador `PIDNet`
     * para sus cálculos de error internos si `Knet.follow` está habilitado (o si `PIDNet` usa `ym` directamente).
     * @note Las características del filtro están fijadas por su constructor por defecto (a través de la clase `filterFO_pass`),
     * lo que significa que su frecuencia de corte está implícitamente definida por la constante `TAN_ST` y no es ajustada
     * dinámicamente por `cplmfc` o el tiempo de muestreo de `PIDNet` (`Ts`).
     */
    filterFO_pass filter_r;

};

#endif // CPLMFC_H
/*
 * Tráiler de archivo para cplmfc.h
 *
 * [EOF]
 */