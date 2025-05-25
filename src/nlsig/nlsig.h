/*
 * Archivo: nlsig.h
 * Autor: Oluwasegun Somefun. oasomefun@futa.edu.ng : 2020
 * Descripción: Archivo de cabecera para la Función Sigmoide N-Logística (nlsig).
 * Este archivo proporciona la declaración de la función nlsig.
 * Código fuente C/C++
 */
#ifndef NLSIG_H
#define NLSIG_H

/* Archivos de Inclusión */
#include <Arduino.h> // Requerido para `min`, `max`, `exp` si no se usa <cmath> estándar
                     // y para compatibilidad general con proyectos Arduino.
#include "helpers/fast_exps.h" // Contiene `exp_by_ones` (aunque actualmente comentado en .cpp)

/* Declaraciones de Funciones */

/**
 * @brief Calcula una función sigmoide N-logística.
 * @details Esta función genera una curva sigmoide (forma de S) que puede componerse de
 * múltiples segmentos logísticos. Permite modelar de forma flexible la curva ajustando
 * los rangos de entrada/salida, el número de segmentos, la pendiente y la dirección.
 * La función mapea un valor de entrada `x` desde un rango de entrada definido `[xmin, xmax]`
 * a un valor de salida `y` en un rango de salida definido `[ymin, ymax]`.
 *
 * @param[out] y El valor sigmoide calculado. Este parámetro se pasa por referencia y es actualizado por la función.
 * @param[in] x El valor de entrada a la función sigmoide.
 * @param[in] xmax El valor de entrada en el cual la función sigmoide se aproxima a su asíntota superior (`ymax` o `ymin` si está invertida).
 * @param[in] xmin El valor de entrada en el cual la función sigmoide se aproxima a su asíntota inferior (`ymin` o `ymax` si está invertida).
 * @param[in] ymax La asíntota superior (valor máximo) del rango de salida de la sigmoide.
 * @param[in] ymin La asíntota inferior (valor mínimo) del rango de salida de la sigmoide.
 * @param[in] n Opcional. El número de segmentos logísticos utilizados para construir la sigmoide. Por defecto es 1.
 *              Un `n` mayor puede crear transiciones más abruptas o formas sigmoides por tramos. Debe ser >= 1.
 * @param[in] lambda Opcional. Un hiperparámetro que controla la tasa de crecimiento (pendiente)
 *               de los segmentos logísticos individuales. Valores mayores hacen la curva más empinada. Por defecto es 6.0.
 * @param[in] safety Opcional. Un porcentaje de margen de seguridad (entero, p.ej., 10 para 10%). Por defecto es 0.
 *               Este parámetro ajusta los `xmax`, `xmin`, `ymax`, `ymin` efectivos para crear un búfer
 *               desde los límites absolutos. Un valor positivo (p.ej., 10) reduce el rango efectivo en un 10%
 *               (p.ej., `xmax_eff = xmax * (1 - 0.1)`). Un valor negativo (p.ej., -10) lo expande en un 10%
 *               (p.ej., `xmax_eff = xmax * (1 + 0.1)`). Se limita internamente a `[-100, 100]`.
 *               @warning Un valor de `safety` de 100 hará que `xmax_eff`, `xmin_eff`, etc., se conviertan en 0,
 *                        lo que podría llevar a una división por cero si `xmin` y `xmax` no fueran iguales.
 *                        La implementación añade un pequeño épsilon a los denominadores para mitigar esto, pero el comportamiento
 *                        con `safety = 100` podría ser extremo.
 * @param[in] isreverse Opcional. Un indicador para invertir la dirección de la sigmoide. Por defecto es 0 (directa/creciente).
 *                  Establecer a 1 para una sigmoide inversa (decreciente), donde `x` mapeando a `xmin` resulta en `y` cerca de `ymax`,
 *                  y `x` mapeando a `xmax` resulta en `y` cerca de `ymin`.
 * 
 * @note La función usa `std::exp` de `<cmath>` (a menudo incluido vía `Arduino.h`). El `exp_by_ones` comentado
 *       se encontró que no era funcional en revisiones previas.
 */
void nlsig(double& y, const double& x,
        double xmax, double xmin, double ymax, double ymin,
        int n=1, double lambda=6, int safety=0, unsigned char isreverse=0);

#endif // NLSIG_H

/*
 * Tráiler de archivo para nlsig.h
 *
 * [EOF]
 */
