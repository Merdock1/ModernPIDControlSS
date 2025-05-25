//
// Creado por oasomefun@futa.edu.ng el 16/1/2020.
// Este archivo de cabecera define una clase de filtro paso bajo de primer orden.
//
#ifndef FILTERFO_PASS_H
#define FILTERFO_PASS_H

#include "Arduino.h" // Para la constante PI, típicamente M_PI de math.h, a menudo incluido por Arduino.h

#ifndef TAN_ST_C // Asegurar que TAN_ST se defina solo una vez
#define TAN_ST_C
/**
 * @brief Constante tangente de pre-distorsión bilineal para tiempo discreto de Shannon.
 * @details Esta constante se utiliza en la transformación bilineal para mapear la respuesta
 * en frecuencia del filtro en tiempo continuo de manera más precisa al dominio del tiempo discreto.
 * Típicamente se calcula como tan(PI * frecuencia_corte / frecuencia_muestreo).
 * Para esta biblioteca, está precalculada para fc/fs = 1/20 (la frecuencia de corte es 1/20 de la frecuencia de muestreo),
 * resultando en TAN_ST = tan(PI/20) aprox 0.1583844403.
 * @note Esta constante implica una relación fija entre la frecuencia de corte y la frecuencia de muestreo para los filtros que la utilizan directamente.
 */
inline constexpr double TAN_ST = 0.1583844403;
#endif

/**
 * @class filterFO_pass
 * @brief Implementa un filtro paso bajo de primer orden.
 * @details Esta clase proporciona una funcionalidad básica de filtro de primer orden. Las características
 * del filtro (frecuencia de corte relativa al tiempo de muestreo) están determinadas por la
 * constante TAN_ST utilizada en el cálculo de su coeficiente interno Tf_kpi.
 * Esto significa que el filtro tiene una característica fija definida en tiempo de compilación mediante TAN_ST,
 * a menos que Tf_kpi se altere manualmente después de la construcción.
 */
class filterFO_pass{
public:
    /**
     * @brief Constructor para el filtro de primer orden.
     * @details Inicializa el estado del filtro `x` a 0 y calcula el coeficiente del filtro `Tf_kpi`
     * usando la constante predefinida `TAN_ST`. La fórmula utilizada es
     * `Tf_kpi = PI / (20.0 * TAN_ST * TAN_ST)`.
     */
    explicit filterFO_pass();

    // --- Variables Miembro ---
    double x;       /**< @brief Variable de estado interna del filtro. Almacena un valor del paso de tiempo anterior. */
    double Tf_kpi;  /**< @brief Coeficiente del filtro precalculado basado en `TAN_ST`. 
                         @details Esta constante se relaciona con la constante de tiempo del filtro y el período de muestreo.
                         Se calcula como `PI / (20.0 * TAN_ST * TAN_ST)`. Este coeficiente se utiliza
                         en la ecuación de diferencias que implementa el filtro. Un Tf_kpi más grande generalmente
                         corresponde a una frecuencia de corte más baja para una tasa de muestreo (implícita) dada. */

    /**
     * @brief Ejecuta un paso del filtro paso bajo de primer orden.
     * @details Actualiza el estado interno del filtro y calcula el nuevo valor de salida filtrado.
     * La ecuación de diferencias utilizada es equivalente a:
     * `x_nuevo = x_antiguo + (Tf_kpi - 1) * y_antiguo`
     * `y_nuevo = (x_nuevo + u) / (Tf_kpi + 1)`
     * `x_antiguo_para_siguiente_iteracion = u`
     * @param y Salida: El valor filtrado. Este parámetro se pasa por referencia y se actualiza con la nueva salida filtrada.
     *          También sirve como la salida anterior (`y_antiguo`) para el paso de cálculo actual.
     * @param u Entrada: El valor de entrada crudo, no filtrado, para el paso de tiempo actual.
     */
    void run(double& y, double u);
};

#endif // FILTERFO_PASS_H
