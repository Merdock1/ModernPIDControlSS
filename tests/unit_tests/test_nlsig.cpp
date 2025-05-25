#include <iostream>
#include <vector>
#include <cmath>   // Para fabs, exp, M_PI
#include <cassert>
#include <limits>  // Para std::numeric_limits

// Simulación de dependencias de Arduino.h si alguna se incluye indirectamente por nlsig.h o fast_exps.h
// Para nlsig, min, max, exp, fabs son primarias. Estas están en cmath.
// PI no es usado directamente por nlsig.
// fast_exps.h usa Arduino.h pero sus funciones no se prueban directamente aquí,
// y el exp_by_ones específico que nlsig *podría* usar está defectuoso y comentado.
// Nos basaremos en std::exp de cmath.

#ifndef PI
#define PI M_PI
#endif

// Asumiendo que nlsig.h y fast_exps.h están en rutas relativas a este archivo de prueba
// Para la compilación con g++, estas rutas deben ser correctas.
#include "../../src/nlsig/nlsig.h"
// fast_exps.h es incluido por nlsig.h. No necesitamos incluirlo directamente aquí.
// Si estuviéramos probando funciones de fast_exps.cpp, necesitaríamos compilarlo.

// Ayudante para comparaciones de punto flotante
void check_close_nlsig(double val, double expected, double tolerance = 1e-5, const char* test_name = "") {
    if (fabs(val - expected) > tolerance) {
        std::cerr << "Aserción fallida [" << test_name << "]: " << val << " no está cerca de " << expected << std::endl;
        assert(false);
    }
}

void test_basic_range_mapping() {
    std::cout << "Ejecutando test_basic_range_mapping..." << std::endl;
    double y;
    double xmin = 0.0, xmax = 10.0;
    double ymin = -1.0, ymax = 1.0;
    int n = 1;
    double lambda = 1.0; // Un lambda moderado para evitar pendientes extremas

    // Probar x = xmin
    nlsig(y, xmin, xmax, xmin, ymax, ymin, n, lambda, 0, 0);
    check_close_nlsig(y, ymin, 1e-3, "x = xmin"); // Esperar cercano a ymin, no exacto debido a la naturaleza sigmoide

    // Probar x = xmax
    nlsig(y, xmax, xmax, xmin, ymax, ymin, n, lambda, 0, 0);
    check_close_nlsig(y, ymax, 1e-3, "x = xmax"); // Esperar cercano a ymax

    // Probar x = punto medio
    double x_mid = (xmin + xmax) / 2.0;
    double y_mid_expected = (ymin + ymax) / 2.0;
    nlsig(y, x_mid, xmax, xmin, ymax, ymin, n, lambda, 0, 0);
    check_close_nlsig(y, y_mid_expected, 1e-3, "x = punto medio");

    std::cout << "test_basic_range_mapping SUPERADO." << std::endl;
}

void test_is_reverse() {
    std::cout << "Ejecutando test_is_reverse..." << std::endl;
    double y;
    double xmin = 0.0, xmax = 10.0;
    double ymin = -1.0, ymax = 1.0;
    int n = 1;
    double lambda = 1.0;

    // Probar x = xmin con isreverse = 1
    nlsig(y, xmin, xmax, xmin, ymax, ymin, n, lambda, 0, 1);
    check_close_nlsig(y, ymax, 1e-3, "x = xmin, isreverse=1"); // Esperar cercano a ymax

    // Probar x = xmax con isreverse = 1
    nlsig(y, xmax, xmax, xmin, ymax, ymin, n, lambda, 0, 1);
    check_close_nlsig(y, ymin, 1e-3, "x = xmax, isreverse=1"); // Esperar cercano a ymin

    // Probar x = punto medio con isreverse = 1
    double x_mid = (xmin + xmax) / 2.0;
    double y_mid_expected = (ymin + ymax) / 2.0; // El punto medio debería seguir siendo el punto medio
    nlsig(y, x_mid, xmax, xmin, ymax, ymin, n, lambda, 0, 1);
    check_close_nlsig(y, y_mid_expected, 1e-3, "x = punto medio, isreverse=1");
    
    std::cout << "test_is_reverse SUPERADO." << std::endl;
}

void test_safety_parameter() {
    std::cout << "Ejecutando test_safety_parameter..." << std::endl;
    double y;
    double xmin_orig = 0.0, xmax_orig = 10.0;
    double ymin_orig = 0.0, ymax_orig = 1.0; // Usar rango y positivo para facilitar la lógica de seguridad
    int n = 1;
    double lambda = 1.0; // Usando un lambda moderado

    // Probar con safety = 10 (reducción del 10%)
    // Rangos efectivos: x_eff = [0.9*0, 0.9*10] = [0, 9], y_eff = [0.9*0, 0.9*1] = [0, 0.9]
    // Sin embargo, la función escala el ymin/ymax original por (1-e_margin)
    // Entonces, ymin_eff = ymin_orig * (1-0.1) = 0 * 0.9 = 0
    // ymax_eff = ymax_orig * (1-0.1) = 1 * 0.9 = 0.9
    // xmin_eff = xmin_orig * (1-0.1) = 0 * 0.9 = 0
    // xmax_eff = xmax_orig * (1-0.1) = 10 * 0.9 = 9
    nlsig(y, xmin_orig, xmax_orig, xmin_orig, ymax_orig, ymin_orig, n, lambda, 10, 0);
    check_close_nlsig(y, ymin_orig * 0.9, 1e-3, "safety=10, x=xmin_orig");

    nlsig(y, xmax_orig, xmax_orig, xmin_orig, ymax_orig, ymin_orig, n, lambda, 10, 0);
    // En xmax_orig (10), está fuera del x_eff_max efectivo (9), por lo que debería estar más allá de ymax_eff (0.9)
    // La sigmoide se calcula sobre el x *original* relativo al xmin_eff, xmax_eff *escalados*.
    // Entonces, si x=xmax_orig (10), para el cálculo interno, x se compara con xmin_eff=0, xmax_eff=9.
    // Esto significa que x=10 es efectivamente "más que el máximo" para la sigmoide interna.
    // La salida debería estar muy cerca del ymax_eff escalado.
    check_close_nlsig(y, ymax_orig * 0.9, 1e-3, "safety=10, x=xmax_orig");


    // Probar con safety = 100 (reducción del 100% -> todos los rangos se vuelven 0)
    // dx_segment y dy_segment se vuelven 0. alpha_rate implica división por dx_segment.
    // El NLSIG_DIV_BY_ZERO_EPSILON debería prevenir un crash.
    // Si dx_segment es épsilon, alpha_rate es enorme. u_exponent será enorme.
    // exp(enorme) es enorme, entonces v_i[id] = dy_segment / (1 + exp(enorme)) -> 0 (ya que dy_segment es 0)
    // y comienza en ymin_scaled (0) y los v_i son 0. Entonces y debería ser 0.
    nlsig(y, xmin_orig, xmax_orig, xmin_orig, ymax_orig, ymin_orig, n, lambda, 100, 0);
    check_close_nlsig(y, 0.0, 1e-3, "safety=100, x=xmin_orig");
    nlsig(y, (xmin_orig + xmax_orig) / 2.0, xmax_orig, xmin_orig, ymax_orig, ymin_orig, n, lambda, 100, 0);
    check_close_nlsig(y, 0.0, 1e-3, "safety=100, x=punto medio");

    // Probar con safety = -10 (expansión del 10%)
    // ymin_eff = ymin_orig * (1 - (-0.1)) = 0 * 1.1 = 0
    // ymax_eff = ymax_orig * (1 - (-0.1)) = 1 * 1.1 = 1.1
    // xmin_eff = xmin_orig * (1.1) = 0
    // xmax_eff = xmax_orig * (1.1) = 11
    nlsig(y, xmin_orig, xmax_orig, xmin_orig, ymax_orig, ymin_orig, n, lambda, -10, 0);
    check_close_nlsig(y, ymin_orig * 1.1, 1e-3, "safety=-10, x=xmin_orig");

    nlsig(y, xmax_orig, xmax_orig, xmin_orig, ymax_orig, ymin_orig, n, lambda, -10, 0);
    // x=10 está ahora dentro del rango efectivo [0, 11].
    // Ya no está al "final" del rango sigmoide escalado.
    // La salida debería ser menor que ymax_eff (1.1) pero mayor que y_mid_eff (0.55)
    // Para x=10, xmax_eff=11, xmin_eff=0. (10-0)/(11-0) = 10/11 = 0.909 del rango.
    // Esperamos que esté cerca de ymax_eff pero no exactamente allí.
    // Una prueba verdadera aquí requeriría conocer el valor exacto de la sigmoide.
    // Para lambda=1, la sigmoide no es extremadamente empinada.
    // Verifiquemos que esté dentro de los límites expandidos y no en el ymax original.
    assert(y < ymax_orig * 1.1 + 1e-3 && y > ymax_orig - 1e-2); // Debería acercarse a 1.1, bastante más allá de 1.0

    std::cout << "test_safety_parameter SUPERADO." << std::endl;
}

void test_effect_of_n() {
    std::cout << "Ejecutando test_effect_of_n..." << std::endl;
    double y1, y2;
    double xmin = 0.0, xmax = 10.0;
    double ymin = 0.0, ymax = 1.0;
    double lambda = 2.0; // Un poco más empinado
    double x = (xmin + xmax) / 2.5; // No exactamente en el punto medio, para ver diferencias

    // n = 1
    nlsig(y1, x, xmax, xmin, ymax, ymin, 1, lambda, 0, 0);
    // n = 2
    nlsig(y2, x, xmax, xmin, ymax, ymin, 2, lambda, 0, 0);
    
    std::cout << "n=1, y=" << y1 << "; n=2, y=" << y2 << std::endl;
    // Con n > 1, la curva puede volverse más empinada si lambda es suficientemente alto, o tener "mesetas"
    // Para un punto no exactamente en el centro, y1 e y2 podrían diferir.
    // Esta no es una aserción muy fuerte, solo verifica que haya diferencia.
    // La naturaleza exacta de la diferencia depende de x, lambda y n.
    assert(fabs(y1 - y2) > 1e-5 || lambda < 0.1); // Pueden ser iguales si lambda es muy pequeño o x está en puntos específicos

    // Probar punto medio para n=1 vs n=2 (debería ser igual debido a la simetría si n es impar vs par)
    x = (xmin + xmax) / 2.0;
    nlsig(y1, x, xmax, xmin, ymax, ymin, 1, lambda, 0, 0);
    nlsig(y2, x, xmax, xmin, ymax, ymin, 2, lambda, 0, 0);
    std::cout << "Punto medio: n=1, y=" << y1 << "; n=2, y=" << y2 << std::endl;
    check_close_nlsig(y1, (ymin+ymax)/2.0, 1e-3, "n=1 punto medio");
    // Para n=2, el punto medio está entre dos segmentos, también debería ser (ymin+ymax)/2
    check_close_nlsig(y2, (ymin+ymax)/2.0, 1e-3, "n=2 punto medio");


    std::cout << "test_effect_of_n SUPERADO." << std::endl;
}

void test_input_clamping_observation() {
    std::cout << "Ejecutando test_input_clamping_observation..." << std::endl;
    double y;
    double xmin = 0.0, xmax = 10.0;
    double ymin = 0.0, ymax = 1.0;
    int n = 1;
    double lambda = 1.0;

    // Probar x < xmin
    nlsig(y, xmin - 5.0, xmax, xmin, ymax, ymin, n, lambda, 0, 0);
    std::cout << "x < xmin (-5.0): y = " << y << std::endl;
    // Esperar que y esté muy cerca de ymin, posiblemente un poco menos si ymin no es 0
    check_close_nlsig(y, ymin, 1e-2, "x < xmin"); // Permitir mayor tolerancia para extrapolación

    // Probar x > xmax
    nlsig(y, xmax + 5.0, xmax, xmin, ymax, ymin, n, lambda, 0, 0);
    std::cout << "x > xmax (15.0): y = " << y << std::endl;
    // Esperar que y esté muy cerca de ymax, posiblemente un poco más si ymax no es 0
    check_close_nlsig(y, ymax, 1e-2, "x > xmax");

    std::cout << "test_input_clamping_observation SUPERADO." << std::endl;
}


int main() {
    test_basic_range_mapping();
    test_is_reverse();
    test_safety_parameter();
    test_effect_of_n();
    test_input_clamping_observation();

    std::cout << "Todas las pruebas de nlsig SUPERADAS." << std::endl;
    return 0;
}
