/*
*	nlsig.cpp
*	Autor: Oluwasegun Somefun. oasomefun@futa.edu.ng : 2020
* 	Versión: 1.0 Producción
*   Descripción: Implementación de la Función Sigmoide N-Logística.
*/

/* Archivos de Inclusión */
#include "nlsig.h" // Cabecera para esta función
#include <Arduino.h> // Para min, max, exp (asumiendo plataforma Arduino)
#include <cmath>     // Para fabs, exp (matemáticas estándar de C++)

// Pequeño valor epsilon para prevenir división por cero en cálculos de nlsig.
// Esta es una constante local para nlsig.cpp.
static constexpr double NLSIG_DIV_BY_ZERO_EPSILON = 1e-9;


/* Definiciones de Funciones */
/*
 * FUNCIÓN SIGMOIDE N-LOGÍSTICA
 * Esta función calcula una curva sigmoide que puede particionarse en 'n' segmentos logísticos.
 * Permite crear curvas en forma de S con pendientes y rangos variables,
 * y puede ser directa o invertida.
 *
 *  ARGUMENTOS:
 *			y: Salida - El valor sigmoide calculado (pasado por referencia).
 *			x: Entrada - El valor de entrada a la función sigmoide.
 *			xmax, xmin: Límites del rango de entrada. xmax es el valor de entrada donde la sigmoide se acerca a su asíntota superior.
 *			            xmin es el valor de entrada donde la sigmoide se acerca a su asíntota inferior.
 *			ymax, ymin: Límites del rango de salida. ymax es la asíntota superior de la sigmoide.
 *			            ymin es la asíntota inferior de la sigmoide.
 *          n: Entero - El número de segmentos logísticos (tipo de sigmoide). Un 'n' mayor puede crear transiciones más abruptas.
 *			lambda: Double - Hiperparámetro, controla la tasa de crecimiento (pendiente) de las curvas sigmoides.
 *			safety: Entero - Porcentaje de margen de seguridad. Ajusta los rangos efectivos de entrada/salida.
 *			isreverse: Unsigned char - Indicador para invertir la sigmoide.
 *			                       0 para sigmoide directa (creciente), 1 para sigmoide inversa (decreciente).
 *
 */
void nlsig(double& y, const double& x, double xmax, double xmin, double ymax, double ymin,
			const int n, const double lambda, int safety_param, const unsigned char isreverse) { // Se renombró safety a safety_param para evitar conflicto con var local

    double e_margin, dy_segment, dx_segment, N_segments, alpha_rate, u_exponent;
    // delta_i almacena los puntos de inflexión para cada segmento
    // v_i almacena la salida parcial de cada segmento antes de sumar
    double delta_i[n], v_i[n]; 
    int c_direction; // Coeficiente de dirección para invertir la sigmoide
	
	N_segments = static_cast<double>(n); 
    if (N_segments == 0) N_segments = NLSIG_DIV_BY_ZERO_EPSILON; // Evitar división por cero para N_segments

	// Aplicar margen de seguridad si se especifica
	if (safety_param != 0) {
        // Limitar safety_param al rango [-100, 100].
        // Esto limita e_margin a [-1.0, 1.0].
		int current_safety = min(100,max(-100,safety_param)); 
		e_margin = static_cast<double>(current_safety)/100.0; // Convertir porcentaje de seguridad a un factor

		// Ajustar límites de entrada y salida por el margen de seguridad.
        // El factor de escala es (1.0 - e_margin).
        // - Si safety_param es positivo (p.ej., 10), e_margin es positivo (0.1).
        //   El factor de escala (1.0 - 0.1 = 0.9) reduce los rangos efectivos de x e y
        //   hacia sus respectivos puntos medios, creando un margen interno o zona de "seguridad".
        //   La sigmoide operará dentro de estos rangos reducidos.
        //
        // - Si safety_param es negativo (p.ej., -10), e_margin es negativo (-0.1).
        //   El factor de escala (1.0 - (-0.1) = 1.1) expande los rangos efectivos de x e y
        //   alejándolos de sus puntos medios. Este comportamiento, aunque resulta de la matemática, es
        //   contraintuitivo para un parámetro llamado "safety", ya que hace que la función
        //   opere sobre rangos virtuales más amplios que los especificados por los xmin/xmax/ymin/ymax originales.
        //
        // - CASO CRÍTICO: Si safety_param es 100, e_margin se convierte en 1.0.
        //   El factor de escala (1.0 - 1.0 = 0.0) hace que xmin, xmax, ymin, ymax sean todos cero.
        //   Esto subsecuentemente causa que dx_segment y dy_segment se conviertan en cero.
        //   Si dx_segment es cero, el cálculo de alpha_rate resultará en división por cero.
        //   Este caso necesita ser manejado para prevenir errores en tiempo de ejecución. (Actualmente no manejado más allá de este comentario).
		ymin = (1.0 - e_margin) * ymin;
		ymax = (1.0 - e_margin) * ymax;
		xmin = (1.0 - e_margin) * xmin;
		xmax = (1.0 - e_margin) * xmax;
	}
	
	// Establecer coeficiente de dirección basado en el indicador isreverse
	c_direction = -1; // Por defecto para sigmoide directa (ymin a ymax)
	if (isreverse){ // Si isreverse es verdadero (1)
		c_direction = 1; // Cambiar a sigmoide inversa (ymax a ymin)
	}

	// Cuantizar o particionar el espacio de entrada-salida por n segmentos.
	// Un n mayor resulta en un espacio particionado más fino (potencialmente más abrupto).
	// El espacio más disperso o grueso es n = 1 (sigmoide logística estándar).

	// Calcular la altura (dy) y anchura (dx) de cada segmento
	dy_segment = (ymax - ymin) / N_segments; // N_segments está protegido contra cero arriba
	dx_segment = (xmax - xmin) / N_segments; // dx_segment aún puede ser cero si xmax=xmin post-seguridad
    
	y = ymin; // Inicializar salida y al mínimo (potencialmente escalado) de su rango
    
	// Tasa de crecimiento logístico para cada segmento
    // alpha_rate determina la pendiente de las curvas logísticas individuales.
    // El factor de 2 es característico de algunas formas de función logística.
    // CRÍTICO: Si dx_segment es 0 (p.ej., debido a safety=100 o xmin=xmax), esto dividirá por cero.
    alpha_rate = lambda * (2.0 / (dx_segment + NLSIG_DIV_BY_ZERO_EPSILON));


    // Iterar a través de cada uno de los 'n' segmentos logísticos
    for (int id = 0; id < n; id++) {
		// Calcular el punto de inflexión (centro) del segmento actual
        // Cada segmento está centrado en xmin + dx_segment * (id + 0.5)
        delta_i[id] = (xmin) + (dx_segment * (static_cast<double>(id) + 0.5));
		
		// Calcular el exponente para la función logística para este segmento
        // u_exponent = coeficiente_direccion * tasa_crecimiento * (valor_entrada - punto_inflexion)
		u_exponent = static_cast<double>(c_direction) * alpha_rate * (x - delta_i[id]);
		
        // Calcular la salida del segmento logístico actual (v_i)
        // Esta es una forma estándar de función logística: altura_segmento / (1 + exp(exponente))
		v_i[id] = dy_segment / (1.0 + exp(u_exponent));
		
        // La siguiente línea se refiere a `exp_by_ones` de `helpers/fast_exps.h`.
        // En la última revisión (Nov 2023), `exp_by_ones` en `fast_exps.h` parece estar
        // fundamentalmente defectuoso, probablemente devolviendo 1.0 para la mayoría de las entradas debido a problemas en sus
        // funciones dependientes (`exp_fast`, `expbysq`). No proporciona una aproximación
        // exponencial rápida correcta. Si se necesita una exponencial rápida verdadera, `exp_by_ones`
        // y sus dependencias requerirían una corrección o reemplazo significativo.
		// v_i[id] = dy_segment/(1+exp_by_ones<double>(u_exponent)); 
		
		// Acumular la salida parcial de este segmento a la salida total y
		y = y + v_i[id];
    }
	
	// La salida final y es la suma de ymin y todas las contribuciones de v_i[id].
	// El "+ 0.0" es probablemente una operación nula, posiblemente por consistencia de tipo o razones históricas.
	y = y + 0.0;

	// FIN
}
