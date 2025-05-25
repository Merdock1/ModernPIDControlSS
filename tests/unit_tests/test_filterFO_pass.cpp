#include <iostream>
#include <vector>
#include <cmath> // Para fabs y M_PI (para PI)
#include <cassert>

// Simulación de dependencias de Arduino.h si alguna se incluye indirectamente.
// Para filterFO_pass, PI es la principal.
#ifndef PI
#define PI M_PI
#endif

// Es una buena práctica incluir el archivo fuente directamente en el archivo de prueba
// para configuraciones simples de pruebas unitarias, ya que evita sistemas de compilación complejos para esta tarea.
// Sin embargo, el prompt pide comandos g++, así que asumiré una compilación separada.
// Para que esto funcione con el comando g++, filterFO_pass.cpp necesita ser compilado y enlazado.
// Para simplificar este entorno, podría poner la definición de la clase directamente aquí o incluir .cpp
// Por ahora, asumiendo que filterFO_pass.h apunta correctamente a su implementación.
#include "../../src/cplmfc/filterFO_pass.h"


// Ayudante para comparaciones de punto flotante
void check_close(double val, double expected, double tolerance = 1e-5) {
    if (fabs(val - expected) > tolerance) {
        std::cerr << "Aserción fallida: " << val << " no está cerca de " << expected << std::endl;
        assert(false);
    }
}

void test_constructor() {
    std::cout << "Ejecutando test_constructor..." << std::endl;
    filterFO_pass filter;
    check_close(filter.x, 0.0);

    // Tf_kpi esperado = PI / (20.0 * TAN_ST * TAN_ST)
    // TAN_ST = 0.1583844403
    double expected_Tf_kpi = PI / (20.0 * TAN_ST * TAN_ST);
    check_close(filter.Tf_kpi, expected_Tf_kpi);
    std::cout << "test_constructor SUPERADO." << std::endl;
}

void test_step_response() {
    std::cout << "Ejecutando test_step_response..." << std::endl;
    filterFO_pass filter;
    double output = 0.0; // Salida inicial para el filtro
    const double input_signal = 1.0;

    // Aplicar entrada escalón
    filter.run(output, input_signal);
    std::cout << "Después de 1 paso: output = " << output << ", state x = " << filter.x << std::endl;
    assert(output > 0.0); // La salida debería empezar a aumentar
    check_close(filter.x, input_signal); // El estado x debería ser la última entrada

    double prev_output = output;
    for (int i = 0; i < 10; ++i) {
        filter.run(output, input_signal);
        std::cout << "Paso " << i + 2 << ": output = " << output << ", state x = " << filter.x << std::endl;
        assert(output > prev_output); // La salida debería continuar aumentando hacia la entrada
        check_close(filter.x, input_signal);
        prev_output = output;
    }

    // Después de muchos pasos, la salida debería estar muy cerca de input_signal
    for (int i = 0; i < 1000; ++i) {
        filter.run(output, input_signal);
    }
    std::cout << "Después de 1000+ pasos: output = " << output << std::endl;
    check_close(output, input_signal, 1e-3); // Comprobar si está muy cerca de 1.0

    std::cout << "test_step_response SUPERADO." << std::endl;
}

void test_zero_input() {
    std::cout << "Ejecutando test_zero_input..." << std::endl;
    filterFO_pass filter;
    double output = 0.0; // Salida inicial
    const double input_signal = 0.0;

    // Aplicar entrada cero
    filter.run(output, input_signal);
    std::cout << "Después de 1 paso (entrada cero): output = " << output << ", state x = " << filter.x << std::endl;
    check_close(output, 0.0);
    check_close(filter.x, 0.0);

    for (int i = 0; i < 100; ++i) {
        filter.run(output, input_signal);
    }
    std::cout << "Después de 100+ pasos (entrada cero): output = " << output << std::endl;
    check_close(output, 0.0);

    // Probar con un estado de salida inicial no cero alimentado al filtro
    output = 5.0; // Simular una salida previa no cero
    filter.x = 5.0; // Y un estado no cero correspondiente (aunque 'x' se actualiza a 'u' primero)
    // El filterFO_pass actualiza x a u al final de run(), así que para esta prueba,
    // deberíamos considerar que el estado 'x' del filtro es lo que importa de la entrada del paso anterior.
    // Si la entrada previa fue 0, entonces x debería ser 0.
    // Reiniciemos el filtro y démosle un valor de salida no cero, luego alimentemos ceros.
    
    filterFO_pass filter3; // filtro nuevo, x=0
    double output3 = 0.0;
    filter3.run(output3, 10.0); // estado x = 10.0, output3 será > 0
    filter3.run(output3, 10.0); // estado x = 10.0, output3 será mayor
    std::cout << "Después de entrada no cero: output3 = " << output3 << std::endl;
    assert(output3 > 0.0);

    double prev_output = output3;
    std::cout << "Cambiando a entrada cero para filter3..." << std::endl;
    for (int i = 0; i < 10; ++i) {
        filter3.run(output3, 0.0); // Aplicar entrada cero
         std::cout << "Paso " << i + 1 << " (entrada cero): output3 = " << output3 << std::endl;
        assert(fabs(output3) < fabs(prev_output) || fabs(output3) < 1e-5); // Debería decrecer hacia 0
        prev_output = output3;
    }
     for (int i = 0; i < 1000; ++i) {
        filter3.run(output3, 0.0);
    }
    std::cout << "Después de muchos pasos (entrada cero desde no cero): output3 = " << output3 << std::endl;
    check_close(output3, 0.0, 1e-3);


    std::cout << "test_zero_input SUPERADO." << std::endl;
}


int main() {
    test_constructor();
    test_step_response();
    test_zero_input();

    std::cout << "Todas las pruebas de filterFO_pass SUPERADAS." << std::endl;
    return 0;
}
