/*
 * Archivo: filterFO_pass.cpp (originalmente cplm_kernel.c)
 * Autor: oasomefun@futa.edu.ng
 * Descripción: Implementa un filtro paso bajo de primer orden.
 * Este archivo proporciona la implementación para un filtro de primer orden,
 * que se puede utilizar para suavizar señales o filtrar puntos de consigna.
 * Código fuente C/C++ generado el: 16-Ene-2020 06:00:56
 */

/* Archivos de Inclusión */
#include "filterFO_pass.h" // Archivo de cabecera para esta clase de filtro

/* Definiciones de Funciones */

/*
 * @brief Implementa un filtro paso bajo de primer orden usando transformación bilineal.
 * Esta función, cplm_kernel, parece ser una implementación de filtro independiente.
 * La clase filterFO_pass utiliza una estructura similar en su método run.
 * @param ym Salida: Valor filtrado. Actualizado por referencia.
 * @param xm Estado: Variable de estado interna del filtro. Actualizada por referencia.
 * @param rin Entrada: Valor de entrada crudo a ser filtrado.
 * @return void
 */
void cplm_kernel(double& ym, double& xm, double rin) {
    // Tf_kpi es una constante relacionada con la constante de tiempo del filtro y el período de muestreo.
    // Se deriva de una transformación bilineal con pre-distorsión (TAN_ST).
    // Tf_kpi = (2 * Tf / Ts) donde Tf es la constante de tiempo del filtro y Ts es el tiempo de muestreo.
    // Aquí, se expresa usando PI y TAN_ST, sugiriendo elecciones de diseño específicas.
    // TAN_ST (0.1583844403) es tan(pi * fc / fs) donde fc es la frecuencia de corte, fs es la frecuencia de muestreo.
    // Si fc/fs = 1/20, entonces TAN_ST = tan(pi/20).
    // Tf_kpi = PI / (20.0 * TAN_ST * TAN_ST) parece un coeficiente precalculado para la ecuación de diferencias del filtro.
    // Analicemos la ecuación de diferencias:
    // xm_nuevo = xm_antiguo + (coeff - 1) * ym_antiguo
    // ym_nuevo = (xm_nuevo + rin) / (coeff + 1)
    // xm_antiguo_para_siguiente_iteracion = rin (esta es una forma común de actualizar el estado en algunas formas de filtro)
    // Esta estructura corresponde a un filtro paso bajo de primer orden.
    double Tf_kpi = PI/(20.0*TAN_ST*TAN_ST); // Coeficiente del filtro

    xm = xm + ((Tf_kpi-1)*ym); // Actualizar estado interno basado en la salida anterior
    ym = (xm+rin)/(Tf_kpi+1);  // Calcular nueva salida filtrada
    xm = rin;                  // Actualizar estado para la siguiente iteración (la entrada actual se convierte en la entrada anterior para la actualización del estado)
}

// Constructor para la clase filterFO_pass.
// Inicializa el coeficiente del filtro Tf_kpi y la variable de estado x.
filterFO_pass::filterFO_pass() {
    // Tf_kpi se calcula una vez durante la construcción.
    // Ver cplm_kernel para una explicación detallada de Tf_kpi.
    Tf_kpi = PI/(20.0*TAN_ST*TAN_ST); 
    x = 0; // Inicializar el estado del filtro 'x' a cero.
}

/*
 * @brief Ejecuta un paso del filtro paso bajo de primer orden.
 * Este método es parte de la clase filterFO_pass.
 * @param y Salida: Valor filtrado. Actualizado por referencia. También se usa como y anterior en el cálculo.
 * @param u Entrada: Valor de entrada crudo a ser filtrado.
 * @return void
 */
void filterFO_pass::run(double& y, double u) {
    // La implementación del filtro es idéntica en lógica a cplm_kernel:
    // x_nuevo = x_antiguo + (Tf_kpi - 1) * y_antiguo
    // y_nuevo = (x_nuevo + u) / (Tf_kpi + 1)
    // x_antiguo_para_siguiente_iteracion = u
    // 'y' se pasa por referencia y sirve tanto como la salida anterior (y_antiguo)
    // para la primera línea, y luego se actualiza a la nueva salida (y_nuevo).
    // 'x' es el estado interno del filtro, equivalente a 'xm' en cplm_kernel.
    // 'u' es la entrada cruda actual, equivalente a 'rin' en cplm_kernel.
    x = x + ((Tf_kpi-1)*y); // Actualizar estado interno basado en la salida anterior 'y'
    y = (x+u)/(Tf_kpi+1);   // Calcular nueva salida filtrada 'y'
    x = u;                  // Actualizar estado para la siguiente iteración (la entrada actual 'u' se convierte en la entrada anterior para el estado)
}


/*
 * Tráiler de archivo para filterFO_pass.cpp
 *
 * [EOF]
 */