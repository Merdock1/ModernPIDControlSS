/**
 * @file autotune_cplmfc_example.ino
 * @brief Sketch de ejemplo para usar el auto-sintonizador CPLMFC con PIDNet.
 * @details Este sketch demuestra cómo configurar y usar el auto-sintonizador CPLMFC (Closed PID Loop Model Following Control - Control de Seguimiento de Modelo de Bucle PID Cerrado)
 * para ajustar dinámicamente las ganancias de un controlador PIDNet. Incluye una planta simulada simple
 * de primer orden para proporcionar retroalimentación al PID.
 *
 * Componentes clave:
 * 1. PIDNet: El controlador PID.
 * 2. cplmfc: El algoritmo de auto-sintonización.
 * 3. PID_kernelOS: La función que orquesta el sintonizador y el controlador PID.
 * 4. Planta Simulada: Un sistema simple de primer orden para proporcionar comportamiento dinámico.
 *
 * Nota: Para una aplicación del mundo real, `read_plant_sensor()` leería de un sensor real,
 * y `apply_plant_actuator()` controlaría un actuador real. La simulación de la planta aquí es
 * para fines ilustrativos.
 */

#include <ModernPIDControlSS.h> // Cabecera principal de la librería

// --- Configuración del Controlador PID ---
// Definir objeto PIDNet: PIDNet(ref_init, yout_init, dt_init, umax_lim, umin_lim, dead_max, dead_min)
// ref_init: Punto de consigna inicial (ej., 0.0)
// yout_init: Salida inicial de la planta (ej., 0.0)
// dt_init: Tiempo de muestreo en segundos (ej., 0.01s = 10ms)
// umax_lim: Salida de control máxima (ej., 255 para PWM de Arduino)
// umin_lim: Salida de control mínima (ej., 0 o -255 para bidireccional)
// dead_max_lim, dead_min_lim: Límites de zona muerta (ej., 0, 0 para sin zona muerta)
PIDNet Knet(0.0, 0.0, 0.01, 255, 0, 0, 0);

// --- Configuración del Auto-Sintonizador CPLMFC ---
cplmfc Tune; // Crear un objeto auto-sintonizador CPLMFC

// --- Parámetros de Simulación para la Planta ---
double plant_output = 0.0;        // Estado actual de la planta simulada
double plant_time_constant = 0.5; // Constante de tiempo de la planta de primer orden (segundos)
double plant_gain = 1.0;          // Ganancia de la planta

// --- Temporización y Punto de Consigna ---
unsigned long last_serial_print_time = 0;
const unsigned long serial_print_interval = 200; // Imprimir datos cada 200 ms
double current_time_seconds = 0.0;
unsigned long last_loop_time_micros = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ; // esperar a que el puerto serie se conecte. Necesario para USB nativo
  }
  Serial.println("ModernPIDControlSS - Ejemplo de Auto-Sintonizador CPLMFC");

  // --- Configurar Auto-Sintonizador CPLMFC ---
  // Tune.begin(Knet, N_ts, N_taul)
  // Knet: La instancia de PIDNet a sintonizar.
  // N_ts: Tiempo de estabilización deseado en términos de número de intervalos de muestreo (Knet.Ts).
  //       Si Knet.Ts = 0.01s y se desea un tiempo de estabilización de 2s, N_ts = 2s / 0.01s = 200.
  // N_taul: Retardo estimado del sistema (tiempo muerto) en términos de número de intervalos de muestreo.
  //         Si el retardo es 0.05s y Knet.Ts = 0.01s, N_taul = 0.05s / 0.01s = 5.
  // Estos valores son estimaciones/objetivos iniciales para el sintonizador.
  int N_ts_settling_time_counts = 200; // Tiempo de estabilización objetivo = 200 * 0.01s = 2.0s
  int N_taul_delay_counts = 5;         // Retardo estimado = 5 * 0.01s = 0.05s
  Tune.begin(Knet, N_ts_settling_time_counts, N_taul_delay_counts);
  Serial.println("Sintonizador CPLMFC Inicializado con begin()");
  Serial.print("Tiempo de Estabilización Objetivo (N_ts_counts * Knet.Ts): "); Serial.print(N_ts_settling_time_counts * Knet.Ts); Serial.println(" s");
  Serial.print("Retardo Estimado (N_taul_counts * Knet.Ts): "); Serial.print(N_taul_delay_counts * Knet.Ts); Serial.println(" s");


  // Opcionalmente, establecer alpha_critics: Tune.set_alpha_critics(Knet, alpha, lambda_i, lambda_d)
  // alpha: Hiperparámetro que influye en la agresividad del cálculo de Kp (por defecto 1.0).
  // lambda_i: Ponderación de PIDNet para la contribución integral al error total (por defecto 0.5 en cplmfc, pero 1.0 en PIDNet por defecto).
  // lambda_d: Ponderación de PIDNet para la contribución derivativa al error total (por defecto 0.1 en cplmfc, pero 1.0 en PIDNet por defecto).
  // float alpha = 1.5;    // Sintonización de Kp ligeramente más agresiva
  // float lambda_i = 0.8; // Ajustar ponderación integral en PIDNet
  // float lambda_d = 0.2; // Ajustar ponderación derivativa en PIDNet
  // Tune.set_alpha_critics(Knet, alpha, lambda_i, lambda_d);
  // Serial.println("CPLMFC alpha_critics establecidos.");

  // Inicializar punto de consigna del controlador PID
  Knet.r = 50.0; // Ejemplo: apuntar a un punto de consigna de 50
  Serial.print("Punto de Consigna Inicial Knet.r: "); Serial.println(Knet.r);

  // Inicializar tiempo
  last_loop_time_micros = micros();
}

/**
 * @brief Simula la lectura de la salida actual de la planta.
 * @details En un sistema real, esto leería un sensor. Aquí, simula una simple
 *          respuesta de primer orden a la última señal de control aplicada.
 * @return La salida simulada actual de la planta.
 */
double read_plant_sensor() {
  // Aquí es donde leerías tu sensor real.
  // Para este ejemplo, usamos la variable global `plant_output`.
  return plant_output;
}

/**
 * @brief Simula la aplicación de la señal de control al actuador de la planta.
 * @details En un sistema real, esto comandaría un actuador (ej., motor, calentador).
 *          Aquí, actualiza el estado de la planta simulada de primer orden.
 * @param control_signal La salida de control `Knet.u` del controlador PID.
 * @param dt El paso de tiempo (delta t) para la simulación, en segundos.
 */
void apply_plant_actuator(double control_signal, double dt) {
  // Aquí es donde aplicarías la señal de control a tu actuador.
  // Simulación de sistema de primer orden simple:
  // dy/dt = (-y + G*u) / tau
  // y_nuevo = y_antiguo + dt * dy/dt
  double plant_derivative = (-plant_output + plant_gain * control_signal) / plant_time_constant;
  plant_output += dt * plant_derivative;

  // Opcional: Añadir ruido o perturbaciones para una simulación más realista
  // plant_output += ((double)random(-10, 11) / 100.0); // Ejemplo: pequeño ruido
}

void loop() {
  // --- Calcular Delta Time (dt) ---
  unsigned long current_micros = micros();
  unsigned long delta_micros = current_micros - last_loop_time_micros;
  last_loop_time_micros = current_micros;
  double dt_seconds = static_cast<double>(delta_micros) / 1000000.0;
  current_time_seconds += dt_seconds;

  // Asegurar que Knet.Ts coincida con el dt real del bucle si es variable,
  // o asegurar que el bucle se ejecute cerca de Knet.Ts. Para este ejemplo, Knet.Ts es fijo.
  // Para dt variable, se podría actualizar Knet.Ts = dt_seconds; antes de PID_kernelOS,
  // pero esto no es estándar para esta librería que asume Ts fijo desde el constructor.
  // La comprobación de temporización de PID_kernelOS `if (t >= ((Knet.T_prev + Knet.Ts) - (0.5 * Knet.Ts)))`
  // maneja la ejecución a intervalos de Knet.Ts.

  // --- Punto de Consigna (Referencia) ---
  // Ejemplo: Cambio escalonado en el punto de consigna después de algún tiempo
  if (current_time_seconds > 10.0 && Knet.r == 50.0) {
    Serial.println("\n--- Cambiando punto de consigna a 100.0 ---");
    Knet.r = 100.0;
  }
   if (current_time_seconds > 20.0 && Knet.r == 100.0) {
    Serial.println("\n--- Cambiando punto de consigna a 20.0 ---");
    Knet.r = 20.0;
  }


  // --- Leer Variable de Proceso (Salida de la Planta) ---
  Knet.y = read_plant_sensor();

  // --- Ejecutar Controlador PID y Auto-Sintonizador ---
  // PID_kernelOS llamará a Tune.run() y luego a Knet.compute()
  // si ha pasado suficiente tiempo (Knet.Ts).
  PID_kernelOS(Knet, Tune, current_time_seconds);

  // --- Aplicar Salida de Control a la Planta ---
  // Usar el dt_seconds real para el paso de simulación de la planta para mayor precisión
  apply_plant_actuator(Knet.u, dt_seconds);

  // --- Salida para Serial Plotter ---
  // Imprimir periódicamente valores para monitorización y graficación.
  unsigned long current_millis = millis();
  if (current_millis - last_serial_print_time >= serial_print_interval) {
    last_serial_print_time = current_millis;

    Serial.print("Tiempo:"); Serial.print(current_time_seconds, 3);
    Serial.print(" Consigna:"); Serial.print(Knet.r, 2);
    Serial.print(" PV:"); Serial.print(Knet.y, 2);
    Serial.print(" Salida:"); Serial.print(Knet.u, 2);
    Serial.print(" Kp:"); Serial.print(Knet.Kp, 4);
    Serial.print(" Ki:"); Serial.print(Knet.Ki, 4);
    Serial.print(" Kd:"); Serial.print(Knet.Kd, 4);
    // Para más información sobre el sintonizador:
    // Serial.print(" wn_sintonizador:"); Serial.print(Tune.wn, 2);
    // Serial.print(" ts_sintonizador:"); Serial.print(Tune.ts, 2);
    // Serial.print(" taul_sintonizador:"); Serial.print(Tune.tau_l, 2);
    Serial.println();
  }

  // Pequeño retardo para simular un bucle de control que no se ejecuta a máxima velocidad,
  // y para dar tiempo a las impresiones seriales, etc.
  // Idealmente, esto debería alinearse con Knet.Ts para una ejecución PID consistente.
  // Si Knet.Ts = 0.01s (10ms), un delay(10) sería apropiado.
  // La comprobación de temporización de PID_kernelOS maneja si el bucle es más rápido.
  // Si el bucle es más lento que Knet.Ts, el PID podría perder ciclos de ejecución.
  if (Knet.Ts * 1000 > delta_micros / 1000.0) {
     delay( (Knet.Ts * 1000) - (delta_micros / 1000.0) );
  }

}
