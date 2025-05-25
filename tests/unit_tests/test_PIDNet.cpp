#include <iostream>
#include <vector>
#include <cmath>   // Para fabs, fmax, fmin, M_PI
#include <cassert>
#include <limits>  // Para std::numeric_limits
#include <iomanip> // Para std::fixed y std::setprecision

// Simulación de dependencias de Arduino.h
#ifndef PI
#define PI M_PI
#endif

// Para la compilación con g++, asegurar que las rutas sean correctas
#include "../../src/pidkernel/PIDNet.h"
// filterFO_pass.h es incluido por PIDNet.h
// norm++kernel.h es incluido por PIDNet.cpp pero no usado directamente por la lógica central aquí.

// Ayudante para comparaciones de punto flotante
bool are_close(double v1, double v2, double epsilon = 1e-6, const char* msg = "") {
    bool check = fabs(v1 - v2) < epsilon;
    if (!check) {
        std::cerr << std::fixed << std::setprecision(10);
        std::cerr << "Aserción fallida " << msg << ": " << v1 << " no está cerca de " << v2 << " (epsilon: " << epsilon << ")" << std::endl;
    }
    return check;
}

void test_constructor_and_initialization() {
    std::cout << "Ejecutando test_constructor_and_initialization..." << std::endl;
    double ref = 5.0, yout = 1.0, dt = 0.01;
    int umax = 100, umin = -10, dead_max_val = 1, dead_min_val = -1; // Renombrado para evitar conflicto

    PIDNet pid(ref, yout, dt, umax, umin, dead_max_val, dead_min_val);

    assert(are_close(pid.Ts, dt, 1e-9, "Verificación Ts"));
    assert(pid.umax == umax && "Verificación umax");
    assert(pid.umin == umin && "Verificación umin");
    assert(pid.dead_max == dead_max_val && "Verificación dead_max");
    assert(pid.dead_min == dead_min_val && "Verificación dead_min");

    // Parámetros PID por defecto de pid_net_constants
    assert(are_close(pid.Kp, pid_net_constants::KP_DEFAULT, 1e-9, "Kp por defecto"));
    assert(are_close(pid.Ki, pid_net_constants::KI_DEFAULT, 1e-9, "Ki por defecto"));
    assert(are_close(pid.Kd, pid_net_constants::KD_DEFAULT, 1e-9, "Kd por defecto"));
    assert(are_close(pid.lambdai, pid_net_constants::LAMBDA_I_DEFAULT, 1e-9, "lambdai por defecto"));
    assert(are_close(pid.lambdad, pid_net_constants::LAMBDA_D_DEFAULT, 1e-9, "lambdad por defecto"));
    assert(are_close(pid.Ti, pid_net_constants::TI_DEFAULT, 1e-9, "Ti por defecto"));
    assert(are_close(pid.Td, pid_net_constants::TD_DEFAULT, 1e-9, "Td por defecto"));
    assert(pid.b == pid_net_constants::B_2DOF_DEFAULT && "b por defecto");
    assert(pid.c == pid_net_constants::C_2DOF_DEFAULT && "c por defecto");

    // Comprobar cálculo de kpi y Tf para el filtro derivativo
    double safe_dt_calc = (dt > pid_net_constants::DIV_BY_ZERO_EPSILON_PID) ? dt : pid_net_constants::DIV_BY_ZERO_EPSILON_PID;
    double cut_freq_calc = (PI) / (pid_net_constants::CUT_FREQ_DIVISOR * safe_dt_calc);
    double expected_kpi = cut_freq_calc / (TAN_ST + pid_net_constants::DIV_BY_ZERO_EPSILON_PID);
    double expected_Tf = pid.Ts / (pid_net_constants::TF_CALC_DENOMINATOR_SCALING * TAN_ST + pid_net_constants::DIV_BY_ZERO_EPSILON_PID);
    
    assert(are_close(pid.kpi, expected_kpi, 1e-9, "Cálculo kpi"));
    assert(are_close(pid.Tf, expected_Tf, 1e-9, "Cálculo Tf"));

    // Comprobar inicialización de r e y
    assert(are_close(pid.r, ref, 1e-9, "r init"));
    assert(are_close(pid.y, yout, 1e-9, "y init"));
    
    // Comprobar que las variables de estado iniciales son cero (según lógica del constructor)
    assert(are_close(pid.ei, 0.0, 1e-9, "ei init"));
    assert(are_close(pid.ed, 0.0, 1e-9, "ed init"));
    assert(are_close(pid.ui, 0.0, 1e-9, "ui init"));
    assert(are_close(pid.ud, 0.0, 1e-9, "ud init"));
    assert(are_close(pid.u, 0.0, 1e-9, "u init"));

    std::cout << "test_constructor_and_initialization SUPERADO." << std::endl;
}

void test_proportional_action() {
    std::cout << "Ejecutando test_proportional_action..." << std::endl;
    double dt = 0.1;
    PIDNet pid(0.0, 0.0, dt, 100, -100, 0, 0); // dead_min=0, dead_max=0 para ningún efecto de zona muerta

    pid.Kp = 1.0; 
    pid.Ki = 0.0; pid.Ti = 1e9; 
    pid.Kd = 0.0; pid.Td = 0.0;
    pid.lambdai = 0.0; 
    pid.lambdad = 0.0; 
    
    pid.c = 0; 
    pid.follow = 0; 
    
    pid.reset_state(); // Asegurar estado limpio

    // Caso 1: b=1 (P sobre error r-y)
    pid.b = 1; 
    pid.r = 10.0; pid.y = 0.0;
    pid.compute(0.0); 
    assert(are_close(pid.u, 10.0, 1e-6, "Prueba Acción-P 1 (b=1, r=10, y=0)"));

    // Caso 2: b=0 (P sobre -y)
    pid.reset_state();
    pid.b = 0;
    pid.r = 10.0; pid.y = 5.0; 
    pid.compute(0.1);
    assert(are_close(pid.u, -5.0, 1e-6, "Prueba Acción-P 2 (b=0, r=10, y=5)"));
    std::cout << "test_proportional_action SUPERADO." << std::endl;
}

void test_integral_action_steps() {
    std::cout << "Ejecutando test_integral_action_steps..." << std::endl;
    double dt = 0.1;
    PIDNet pid(0.0, 0.0, dt, 100, -100, 0, 0);

    pid.Kp = 1.0; 
    pid.Ti = 1.0; 
    pid.Ki = pid.Kp / pid.Ti; 
    pid.Kd = 0.0; pid.Td = 0.0; 
    pid.lambdai = 1.0; pid.lambdad = 0.0; 
    pid.b = 1; pid.c = 0; 
    pid.follow = 0; 
    pid.r = 1.0; pid.y = 0.0; 

    pid.reset_state();
    
    double kpi_val = pid.kpi; 

    pid.compute(0.0); // Paso 0
    double expected_ui_step0 = (1.0 / (pid.Ti * kpi_val + pid_net_constants::DIV_BY_ZERO_EPSILON_PID)) * 1.0 +
                               (pid_net_constants::KUI_NUMERATOR / (pid.Ki + pid_net_constants::DIV_BY_ZERO_EPSILON_PID)) * 1.0;
    assert(are_close(pid.ui, expected_ui_step0, 1e-6, "Acción-I Paso 0 ui"));
    double expected_u_step0 = pid.Kp * (1.0 + pid.lambdai * expected_ui_step0);
    assert(are_close(pid.u, expected_u_step0, 1e-6, "Acción-I Paso 0 u"));

    // Almacenar estados del ciclo anterior para el siguiente cálculo manual
    double ui_prev_cycle = pid.ui;
    double ei_prev_cycle = pid.ei;
    double upd_prev_cycle = pid.upd;
    double u_prev_cycle = pid.u;
    double v_prev_cycle = pid.v;

    pid.y = 0.0; // Mantener error constante
    pid.compute(0.1); // Paso 1
    // Cálculo manual esperado para ui en el Paso 1, basado en la lógica interna de compute()
    double e_u1 = u_prev_cycle - v_prev_cycle;
    double Keu1 = pid.Kp + pid_net_constants::KEU_KI_SCALING_FACTOR * pid.Ts * pid.Ki + 
                  (pid_net_constants::KEU_KD_TS_SCALING_FACTOR / (pid.Ts + pid_net_constants::DIV_BY_ZERO_EPSILON_PID)) * pid.Kd;
    double ua1 = e_u1 / (Keu1 + pid_net_constants::DIV_BY_ZERO_EPSILON_PID);
    double yfict1 = pid.y + ua1;
    double kui1 = pid_net_constants::KUI_NUMERATOR / (pid.Ki + pid_net_constants::DIV_BY_ZERO_EPSILON_PID);
    double ui_interim = ui_prev_cycle + (1.0 / (pid.Ti * kpi_val + pid_net_constants::DIV_BY_ZERO_EPSILON_PID)) * ei_prev_cycle - 
                        kui1 * upd_prev_cycle; // Primera parte de la actualización de ui
    double ep_calc1 = pid.b * 1.0 - yfict1; // ym_calc es 1.0 porque r=1.0, follow=0
    double ei_curr1 = (1.0 - yfict1) + e_u1; // ei para el ciclo actual
    double up_curr1 = ep_calc1;
    double ud_curr1 = 0; 
    double upd_curr1 = up_curr1 + ud_curr1;
    double expected_ui_step1 = ui_interim + (1.0 / (pid.Ti * kpi_val + pid_net_constants::DIV_BY_ZERO_EPSILON_PID)) * ei_curr1 -
                               ua1 + kui1 * upd_curr1; // Segunda parte de la actualización de ui
    assert(are_close(pid.ui, expected_ui_step1, 1e-5, "Acción-I Paso 1 ui")); 
    
    std::cout << "test_integral_action_steps SUPERADO." << std::endl;
}


void test_derivative_action_detailed() {
    std::cout << "Ejecutando test_derivative_action_detailed..." << std::endl;
    double dt = 0.1;
    PIDNet pid(0.0, 0.0, dt, 100, -100, 0, 0);

    pid.Kp = 1.0; 
    pid.Td = 0.1; 
    pid.Kd = pid.Kp * pid.Td; 
    pid.Ki = 0.0; pid.Ti = 1e9; 
    pid.lambdai = 0.0; pid.lambdad = 1.0; 
    pid.b = 0; 
    pid.c = 1; // Poner D en yfict (c=1), P en -yfict (b=0)
    pid.follow = 0; 

    pid.reset_state();
    pid.r = 0.0; pid.y = 0.0; // Establecer referencia y salida iniciales
    
    double kpi_val = pid.kpi;
    double Tf_val = pid.Tf;

    // Paso 0: y=0, r=0. Todo debería ser cero.
    pid.compute(0.0);
    assert(are_close(pid.u, 0.0, 1e-6, "Acción-D Paso 0 Salida"));
    assert(are_close(pid.ed, 0.0, 1e-6, "Acción-D Paso 0 ed")); // ed = c*ym_calc - yfict. ym_calc=r=0. y=0, ua=0 -> yfict=0.
    assert(are_close(pid.ud, 0.0, 1e-6, "Acción-D Paso 0 ud"));
    
    // Paso 1: Cambiar y para inducir acción derivativa. r=0, y=1.0
    // yfict debería ser 1.0 (ya que ua=0 del paso anterior).
    // ed_previo (del Paso 0) fue 0.0. ud_previo fue 0.0.
    // ed_actual = c*r - yfict = 1*0 - 1.0 = -1.0 (con c=1, ym_calc=r=0)
    // ud_actualizado = (ud_previo_actualizado_parcialmente + kpi*Td*ed_actual) / (kpi*Tf + 1)
    // ud_actualizado_parcialmente = ud_previo * (kpi*Tf - 1) - kpi*Td*ed_previo = 0 - 0 = 0
    pid.y = 1.0; 
    pid.r = 0.0; 
    pid.compute(dt); 
    
    // up = b*r - yfict = 0*0 - 1.0 = -1.0 (porque yfict será 1.0 ya que u y v del paso anterior eran 0)
    double expected_ud_s1_unfiltered = -kpi_val * pid.Td * 1.0; // ed_actual es -1.0. La fórmula es -kpi*Td*ed_prev + kpi*Td*ed_curr. Aquí ed_prev=0.
                                                                // No, la fórmula es: this->ud *= (kpi * this->Tf - 1.0); this->ud -= kpi * this->Td * (this->ed_previo);
                                                                // this->ud += kpi * this->Td * (this->ed_actual); this->ud = this->ud / (kpi * this->Tf + 1.0);
                                                                // Entonces, ud = (0 - kpi*Td*0 + kpi*Td*(-1.0))/(kpi*Tf+1) = -kpi*Td / (kpi*Tf+1)
    double expected_ud_s1_filtered = (0.0 - kpi_val * pid.Td * 0.0 + kpi_val * pid.Td * (-1.0) ) / (kpi_val * Tf_val + 1.0 + pid_net_constants::DIV_BY_ZERO_EPSILON_PID);
    assert(are_close(pid.ed, -1.0, 1e-6, "Acción-D Paso 1 ed"));
    assert(are_close(pid.ud, expected_ud_s1_filtered, 1e-6, "Acción-D Paso 1 ud"));
    double expected_u_s1 = pid.Kp * (-1.0 + pid.lambdad * expected_ud_s1_filtered); // up es -1.0 (b=0, yfict=1.0)
    assert(are_close(pid.u, expected_u_s1, 1e-6, "Acción-D Paso 1 u"));

    std::cout << "test_derivative_action_detailed SUPERADO." << std::endl;
}


void test_output_saturation() {
    std::cout << "Ejecutando test_output_saturation..." << std::endl;
    double dt = 0.1;
    PIDNet pid(0.0, 0.0, dt, 10, -10, 0, 0); // umax=10, umin=-10

    pid.Kp = 100.0; // Kp alto para forzar saturación
    pid.Ki = 0.0; pid.Kd = 0.0; pid.Ti = 1e9; pid.Td = 0.0; // Sin acción I o D
    pid.lambdai = 0.0; pid.lambdad = 0.0; 
    pid.b = 1; pid.c = 0; // P sobre error
    pid.follow = 0;

    pid.reset_state();
    pid.r = 1.0; pid.y = 0.0; // Error = 1.0. Salida P = 100.0 * 1.0 = 100.0
    pid.compute(0.0);
    assert(are_close(pid.u, 10.0, 1e-9, "Salida limitada en umax"));

    pid.reset_state();
    pid.r = -1.0; pid.y = 0.0; // Error = -1.0. Salida P = 100.0 * -1.0 = -100.0
    pid.compute(0.1); 
    assert(are_close(pid.u, -10.0, 1e-9, "Salida limitada en umin"));

    std::cout << "test_output_saturation SUPERADO." << std::endl;
}


void test_anti_windup_effect() {
    std::cout << "Ejecutando test_anti_windup_effect..." << std::endl;
    double dt = 0.1;
    // PID con anti-windup (límites de saturación estrechos)
    PIDNet pid_aw(0.0, 0.0, dt, 10, -10, 0, 0); 
    pid_aw.Kp = 2.0; pid_aw.Ti = 0.5; pid_aw.Ki = pid_aw.Kp / pid_aw.Ti; 
    pid_aw.Kd = 0.0; pid_aw.Td = 0.0;
    pid_aw.lambdai = 1.0; pid_aw.lambdad = 0.0;
    pid_aw.b = 1; pid_aw.c = 0; pid_aw.follow = 0;
    pid_aw.r = 10.0; pid_aw.y = 0.0; // Error grande para inducir saturación y windup

    // PID sin anti-windup efectivo (límites de saturación muy amplios)
    PIDNet pid_no_aw(0.0, 0.0, dt, 1000, -1000, 0, 0);
    pid_no_aw.Kp = 2.0; pid_no_aw.Ti = 0.5; pid_no_aw.Ki = pid_no_aw.Kp / pid_no_aw.Ti; 
    pid_no_aw.Kd = 0.0; pid_no_aw.Td = 0.0;
    pid_no_aw.lambdai = 1.0; pid_no_aw.lambdad = 0.0;
    pid_no_aw.b = 1; pid_no_aw.c = 0; pid_no_aw.follow = 0;
    pid_no_aw.r = 10.0; pid_no_aw.y = 0.0;

    // Ejecutar ambos PIDs durante un tiempo para acumular término integral
    for (int i = 0; i < 10; ++i) {
        pid_aw.compute(i * dt);
        pid_no_aw.compute(i * dt);
    }
    // El PID con AW debería estar saturado, el otro no (o menos)
    assert(pid_aw.u >= 10.0 - 1e-5 && "PID con AW debería estar saturado alto"); 
    assert(pid_no_aw.u > 10.0 && "Salida PID sin AW debería ser > umax de PID con AW");
    // El término integral del PID con AW debería ser menor debido al anti-windup
    assert(pid_aw.ui < pid_no_aw.ui && "ui de PID con AW debería ser menor que ui de PID sin AW");
    
    // Cambiar setpoint para observar recuperación
    pid_aw.r = 0.0; pid_aw.y = 0.0; // Error ahora es 0 (o cercano si y cambia)
    pid_no_aw.r = 0.0; pid_no_aw.y = 0.0;

    for (int i = 10; i < 20; ++i) { 
        pid_aw.compute(i * dt);
        pid_no_aw.compute(i * dt);
    }
    // El PID con AW debería recuperarse de la saturación más rápido
    assert(fabs(pid_aw.u) < fabs(pid_no_aw.u) && "PID con AW debería recuperarse más rápido de la saturación");

    std::cout << "test_anti_windup_effect SUPERADO (comparativo)." << std::endl;
}

void test_set_bc_follow_method() {
    std::cout << "Ejecutando test_set_bc_follow_method..." << std::endl;
    PIDNet pid(0.0, 0.0, 0.1, 100, -100, 0, 0);
    pid.set_bc_follow(0, 0, 1);
    assert(pid.b == 0 && "b después de set_bc_follow");
    assert(pid.c == 0 && "c después de set_bc_follow");
    assert(pid.follow == 1 && "follow después de set_bc_follow");

    pid.set_bc_follow(1, 1, 0);
    assert(pid.b == 1 && "b de nuevo después de set_bc_follow");
    assert(pid.c == 1 && "c de nuevo después de set_bc_follow");
    assert(pid.follow == 0 && "follow de nuevo después de set_bc_follow");
    std::cout << "test_set_bc_follow_method SUPERADO." << std::endl;
}

void test_reset_state() {
    std::cout << "Ejecutando test_reset_state..." << std::endl;
    PIDNet pid(10.0, 1.0, 0.1, 100, -100, 0, 0);
    pid.Kp = 2.0; pid.Ki = 1.0; pid.Kd = 0.5; // Establecer algunas ganancias no por defecto

    // Ejecutar compute varias veces para cambiar estados internos
    for(int i=0; i<5; ++i) {
        pid.y = i * 0.2; // Variar y
        pid.compute(i * 0.1);
    }
    assert(!are_close(pid.ei, 0.0, 1e-9) || !are_close(pid.u, 0.0, 1e-9)); // Los estados deberían haber cambiado

    pid.reset_state();

    assert(are_close(pid.T_prev, -pid.Ts, 1e-9, "T_prev reseteado"));
    assert(pid.countseq == 0 && "countseq reseteado");
    assert(are_close(pid.ym, 0.0, 1e-9, "ym reseteado"));
    assert(are_close(pid.e, 0.0, 1e-9, "e reseteado"));
    assert(are_close(pid.ei, 0.0, 1e-9, "ei reseteado"));
    assert(are_close(pid.ed, 0.0, 1e-9, "ed reseteado"));
    assert(are_close(pid.up, 0.0, 1e-9, "up reseteado"));
    assert(are_close(pid.ui, 0.0, 1e-9, "ui reseteado"));
    assert(are_close(pid.ud, 0.0, 1e-9, "ud reseteado"));
    assert(are_close(pid.upd, 0.0, 1e-9, "upd reseteado"));
    assert(are_close(pid.ua, 0.0, 1e-9, "ua reseteado"));
    assert(are_close(pid.v, 0.0, 1e-9, "v reseteado"));
    assert(are_close(pid.u, 0.0, 1e-9, "u reseteado"));
    assert(are_close(pid.uo, 0.0, 1e-9, "uo reseteado"));
    assert(are_close(pid.e_t, 0.0, 1e-9, "e_t reseteado"));
    assert(are_close(pid.uf, 0.0, 1e-9, "uf reseteado"));
    assert(are_close(pid.filter_u.x, 0.0, 1e-9, "filter_u.x reseteado"));
    
    // Comprobar que las ganancias NO se resetean
    assert(are_close(pid.Kp, 2.0, 1e-9, "Kp no reseteado"));
    assert(are_close(pid.Ki, 1.0, 1e-9, "Ki no reseteado"));
    assert(are_close(pid.Kd, 0.5, 1e-9, "Kd no reseteado"));
    // Comprobar que el punto de consigna y la medición actual no se resetean por reset_state()
    assert(are_close(pid.r, 10.0, 1e-9, "r no reseteado por reset_state"));
    assert(are_close(pid.y, 4 * 0.2, 1e-9, "y no reseteado por reset_state")); // y mantiene su último valor


    std::cout << "test_reset_state SUPERADO." << std::endl;
}

void test_getters() {
    std::cout << "Ejecutando test_getters..." << std::endl;
    PIDNet pid(10.0, 1.0, 0.1, 100, -100, 0, 0);
    pid.Kp = 2.0; pid.Ki = 1.0; pid.Kd = 0.5; pid.lambdai = 0.8; pid.lambdad = 0.7;
    pid.b = 1; pid.c = 1; pid.follow = 0;

    pid.compute(0.0); // Ejecutar un ciclo

    assert(are_close(pid.get_proportional_term_output(), pid.Kp * pid.up, 1e-9, "get_proportional_term_output"));
    assert(are_close(pid.get_integral_term_output(), pid.Kp * pid.lambdai * pid.ui, 1e-9, "get_integral_term_output"));
    assert(are_close(pid.get_derivative_term_output(), pid.Kp * pid.lambdad * pid.ud, 1e-9, "get_derivative_term_output"));
    assert(are_close(pid.get_control_output(), pid.u, 1e-9, "get_control_output"));
    assert(are_close(pid.get_unsaturated_control_output(), pid.v, 1e-9, "get_unsaturated_control_output"));
    assert(are_close(pid.get_error(), pid.e, 1e-9, "get_error"));
    assert(are_close(pid.get_integral_error_component(), pid.ei, 1e-9, "get_integral_error_component"));
    assert(are_close(pid.get_derivative_error_component(), pid.ed, 1e-9, "get_derivative_error_component"));
    assert(are_close(pid.get_raw_proportional_component(), pid.up, 1e-9, "get_raw_proportional_component"));
    assert(are_close(pid.get_raw_integral_component(), pid.ui, 1e-9, "get_raw_integral_component"));
    assert(are_close(pid.get_raw_derivative_component(), pid.ud, 1e-9, "get_raw_derivative_component"));
    assert(are_close(pid.get_sampling_time(), pid.Ts, 1e-9, "get_sampling_time"));

    std::cout << "test_getters SUPERADO." << std::endl;
}

void test_deadzone_functionality() {
    std::cout << "Ejecutando test_deadzone_functionality..." << std::endl;
    PIDNet pid(0.0, 0.0, 0.1, 100, -100, 0, 0); // Zona muerta inicial es 0,0

    // Deshabilitar P,I,D para control directo de u para configuración de prueba
    pid.Kp = 1.0; pid.Ki = 0.0; pid.Kd = 0.0; pid.Ti = 1e9; pid.Td = 0.0;
    pid.lambdai = 0.0; pid.lambdad = 0.0; pid.b = 1;

    // Caso 1: Sin zona muerta (dead_min = 0, dead_max = 0)
    pid.set_deadzone(0, 0);
    pid.r = 5.0; pid.y = 0.0; // up = 5, v = 5, uf = 5, u = 5, uo = 5
    pid.compute(0.0);
    assert(are_close(pid.u, 5.0, 1e-9, "Zona muerta 0,0: u sin cambios"));

    // Caso 2: Salida dentro de (-dead_min, dead_min) -> salida debería ser 0
    pid.set_deadzone(1, 2); // dead_min=1, dead_max=2
    pid.r = 0.5; pid.y = 0.0; // up = 0.5, v = 0.5, uf = 0.5, u_antes_dz = 0.5
    pid.reset_state(); // resetear ui, etc.
    pid.compute(0.1);
    // uo antes de zona muerta = 0.5. fabs(0.5) <= fabs(dead_min=1). Entonces uo se vuelve 0. Luego u se vuelve 0.
    assert(are_close(pid.u, 0.0, 1e-9, "Zona muerta 1,2: u=0.5 -> u=0"));

    // Caso 3: Salida > dead_min y <= dead_max -> salida debería ser dead_max
    pid.r = 1.5; pid.y = 0.0; // u_antes_dz = 1.5
    pid.reset_state();
    pid.compute(0.2);
    // uo antes de zona muerta = 1.5. fabs(1.5) > fabs(dead_min=1) Y fabs(1.5) <= fabs(dead_max=2).
    // Entonces uo se vuelve copysign(dead_max, 1.5) = 2.0. u se vuelve 2.0.
    assert(are_close(pid.u, 2.0, 1e-9, "Zona muerta 1,2: u=1.5 -> u=2.0"));

    // Caso 4: Salida < -dead_min y >= -dead_max -> salida debería ser -dead_max
    pid.r = -1.5; pid.y = 0.0; // u_antes_dz = -1.5
    pid.reset_state();
    pid.compute(0.3);
    // uo antes de zona muerta = -1.5. fabs(-1.5) > fabs(dead_min=1) Y fabs(-1.5) <= fabs(dead_max=2).
    // Entonces uo se vuelve copysign(dead_max, -1.5) = -2.0. u se vuelve -2.0.
    assert(are_close(pid.u, -2.0, 1e-9, "Zona muerta 1,2: u=-1.5 -> u=-2.0"));

    // Caso 5: Salida > dead_max -> salida = u_original + dead_max (para u positivo)
    pid.r = 3.0; pid.y = 0.0; // u_antes_dz = 3.0
    pid.reset_state();
    pid.compute(0.4);
    // uo antes de zona muerta = 3.0. fabs(3.0) > fabs(dead_max=2).
    // Entonces uo se vuelve copysign(fabs(3.0 + 2.0), 3.0) = 5.0. u se vuelve 5.0.
    assert(are_close(pid.u, 5.0, 1e-9, "Zona muerta 1,2: u=3.0 -> u=5.0"));
    
    // Caso 6: Salida < -dead_max -> salida = u_original - dead_max (para u negativo) -> Lógica de dead_zone: uin = copysign(fabs(uin+dead_max_val), uin);
    pid.r = -3.0; pid.y = 0.0; // u_antes_dz = -3.0
    pid.reset_state();
    pid.compute(0.5);
    // uo antes de zona muerta = -3.0. fabs(-3.0) > fabs(dead_max=2).
    // Entonces uo se vuelve copysign(fabs(-3.0 + 2.0), -3.0) = copysign(fabs(-1.0), -3.0) = copysign(1.0, -3.0) = -1.0.
    assert(are_close(pid.u, -1.0, 1e-9, "Zona muerta 1,2: u=-3.0 -> u=-1.0"));

    std::cout << "test_deadzone_functionality SUPERADO." << std::endl;
}


int main() {
    std::cout << std::fixed << std::setprecision(6); // Establecer precisión para salidas de prueba

    test_constructor_and_initialization();
    test_proportional_action();
    test_integral_action_steps(); 
    test_derivative_action_detailed(); 
    test_output_saturation();
    test_anti_windup_effect();
    test_set_bc_follow_method();
    test_reset_state();
    test_getters();
    test_deadzone_functionality();

    std::cout << "Todas las pruebas de PIDNet ejecutadas. Revise la salida para comportamiento detallado y aserciones." << std::endl;
    return 0;
}
