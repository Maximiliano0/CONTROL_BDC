// ============================================================
//  Control PID con Anti-Windup (Back-Calculation)
//  Motor DC + Puente H + Encoder incremental
//  Implementación práctica basada en los capítulos 05/06 del repo
// ============================================================
//
//  CONEXIONES SUGERIDAS (ajustar según tu puente H):
//  ─────────────────────────────────────────────────
//  Puente H (ej: L298N / L293D / BTS7960)
//    ENA / PWM  →  PIN_PWM   (pin PWM, ej: 9)
//    IN1        →  PIN_DIR_A (ej: 7)
//    IN2        →  PIN_DIR_B (ej: 8)
//
//  Encoder incremental (canal A y B)
//    Canal A    →  PIN_ENC_A (2)  ← interrupción
//    Canal B    →  PIN_ENC_B (3)  ← interrupción (opcional para cuadratura)
//
//  Referencia de posición: se ingresa por Serial en grados
// ============================================================

#include <Arduino.h>
#include <ctype.h>
#include <stdlib.h>

// ── Perfil fijo de implementación (hardware 12V) ────────────
// Ts = 1 ms, saturación del actuador = ±12 V.
#define CTRL_TS_US 1000UL
#define CTRL_V_MAX 12.0f

#define CTRL_V_MIN (-CTRL_V_MAX)

// Modo de anti-windup por back-calculation:
// 0: Kaw = Kp/10     (heurística de clase)
// 1: Kaw = sqrt(Ki*Kd)
// 2: Kaw fijo por CTRL_KAW_FIXED
#ifndef CTRL_KAW_MODE
#define CTRL_KAW_MODE 0
#endif

#ifndef CTRL_KAW_FIXED
#define CTRL_KAW_FIXED 0.02f
#endif

// 1: reinicia estados PID al cambiar referencia | 0: mantiene memoria del lazo
#ifndef CTRL_RESET_ON_SETPOINT
#define CTRL_RESET_ON_SETPOINT 0
#endif

// ── Pines ────────────────────────────────────────────────────
const uint8_t PIN_PWM   = 9;
const uint8_t PIN_DIR_A = 7;
const uint8_t PIN_DIR_B = 8;
const uint8_t PIN_ENC_A = 2;   // INT0
const uint8_t PIN_ENC_B = 3;   // INT1

// ── Parámetros del sistema ───────────────────────────────────
const unsigned long TS_US = CTRL_TS_US;   // [us] Ts para planificador temporal
const float Ts = (float)TS_US * 1e-6f;    // [s] período de muestreo

// Caja reductora

const int   PPR          = 600;         // pulsos por revolución del encoder 


// Conversión: pulsos → radianes en eje de SALIDA
// (igual que C = [0,0,1/n] en el modelo de espacio de estados)
const float PULSOS_A_RAD = (2.0f * PI) / (float)(PPR * 4);

// ── Parámetros PID ───────────────────────────────────────────
// Obtener de pidtune en MATLAB con la planta discreta sys_planta_z
// y reemplazar aquí:
float Kp   = 0.75f;      // Valor inicial práctico para banco 12V
float Ki   = 0.25f;      // Aumentado para evitar integración excesivamente lenta
float Kd   = 0.02f;      // Derivativo suave para no amplificar cuantización del encoder
float Tf   = 0.02f;      // Filtro derivativo más rápido que el valor didáctico original

// Anti-windup: ganancia de back-calculation
// Se calcula en setup() según CTRL_KAW_MODE (Kp/10, sqrt(Ki*Kd) o fijo).
float Kaw  = 0.0f;

// ── Saturación de tensión ────────────────────────────────────
const float V_MAX = CTRL_V_MAX;
const float V_MIN = CTRL_V_MIN;

// ── Ajustes prácticos del driver L298N ───────────────────────
// Este bloque agrega compensación de no linealidades reales del banco:
// caída del puente H, umbral de arranque, PWM mínimo y anti-stiction.
const float L298N_DROP_V      = 0.5f;   // Caida aproximada medida del puente H
const float UMBRAL_ARRANQUE_V = 2.0f;  // Evita zona muerta excesiva en lazo cerrado
const int   PWM_MIN_ARRANQUE  = 76;     // ~30% de 255 para vencer friccion estatica
const float V_MIN_CONTROL     = 6.0f;   // Asistencia anti-stiction en lazo cerrado
const float ERROR_MIN_MOV_RAD = 0.05f;  // Si error supera esto, forzar esfuerzo mínimo útil

// ── Referencia de posición ───────────────────────────────────
float referencia_rad = 0;   // modificar por Serial

// ── Variables de estado del PID ─────────────────────────────
float ui         = 0.0f;   // acumulador integral
float ud         = 0.0f;   // término derivativo filtrado (estado)
float e_prev     = 0.0f;   // error en k-1
float sat_error_prev = 0.0f;  // error de saturación en k-1

// ── Encoder ─────────────────────────────────────────────────
volatile long pulsos = 0;

#if defined(ARDUINO_ARCH_AVR)
const uint8_t ENC_A_MASK = _BV(PD2); // Pin 2 en ATmega328P
const uint8_t ENC_B_MASK = _BV(PD3); // Pin 3 en ATmega328P
#endif

// ── Temporización ────────────────────────────────────────────
unsigned long t_ultimo = 0;
unsigned long t_telemetria_ms = 0;

// ── Serial no bloqueante ─────────────────────────────────────
char buffer_serial[24];
uint8_t idx_serial = 0;
const unsigned long T_TELEMETRIA_MS = 1000; // 1 Hz
bool control_habilitado = false;
bool cero_realizado = false;

// ── Prototipos ───────────────────────────────────────────────
void   ISR_encoderA();
void   ISR_encoderB();
float  saturar(float val, float vmin, float vmax);
void   aplicarPWM(float voltaje);
void   leerSerialReferencia();
void   resetControlador();
void   procesarLineaSerial(char* linea);
void   evaluarConstantes();
void   actualizarKaw();

// ============================================================
void setup() {
    Serial.begin(115200);

    pinMode(PIN_PWM,   OUTPUT);
    pinMode(PIN_DIR_A, OUTPUT);
    pinMode(PIN_DIR_B, OUTPUT);
    pinMode(PIN_ENC_A, INPUT_PULLUP);
    pinMode(PIN_ENC_B, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(PIN_ENC_A), ISR_encoderA, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_ENC_B), ISR_encoderB, CHANGE);

    t_ultimo = micros();
    t_telemetria_ms = millis();
    actualizarKaw();

    Serial.println(F("=== PID Motor DC con Anti-Windup ==="));
    Serial.println(F("Perfil fijo: HARDWARE 12V (Ts=1ms, Vsat=+/-12V)"));
    Serial.println(F("1) Coloca eje manualmente en 0"));
    Serial.println(F("2) Envia START para tomar ese cero y habilitar control"));
    Serial.println(F("Comandos: START, STOP, HELP, MOTOR <V>, o referencia en grados (ej: 90)"));
    Serial.println(F("CSV_HEADER,t_ms,ref_deg,pos_rad,error_rad,u_total_v,u_p_v,u_i_v,u_d_v"));
    evaluarConstantes();
}

// ============================================================
void loop() {

    // ── 1. Esperar el período de muestreo exacto ──────────────
    unsigned long ahora = micros();
    if ((unsigned long)(ahora - t_ultimo) < TS_US) {
        leerSerialReferencia();
        return;
    }
    t_ultimo += TS_US;
    if ((unsigned long)(ahora - t_ultimo) > TS_US) {
        // Si hubo sobretiempo grande, resincroniza para evitar deriva acumulada.
        t_ultimo = ahora;
    }

    // ── 2. Leer comandos/referencia por Serial (no bloqueante) ─
    leerSerialReferencia();

    // ── 3. Leer posición actual ────────────────────────────────
    long p;
    noInterrupts();
        p = pulsos;
    interrupts();
    float y = (float)p * PULSOS_A_RAD;   // posición eje salida [rad] , lectura del encoder

    if (!control_habilitado) {
        aplicarPWM(0.0f);

        unsigned long t_ms = millis();
        if ((unsigned long)(t_ms - t_telemetria_ms) >= T_TELEMETRIA_MS) {
            t_telemetria_ms = t_ms;
            Serial.println(F("=============================================="));
            Serial.println(F(" MODO ESPERA: coloca eje en 0 y envia START"));
            Serial.print(F("Posicion [rad]     : "));
            Serial.println(y, 4);
            Serial.print(F("Pulsos encoder     : "));
            Serial.println(p);
            Serial.println(F("=============================================="));
        }
        return;
    }

    // ── 4. Error ───────────────────────────────────────────────
    float error = referencia_rad - y; 

    // ── 5. Término Proporcional ────────────────────────────────
    float up = Kp * error;

    // ── 6. Término Derivativo con filtro (Euler hacia atrás) ───
    // Equivalente a: ud(k) = (Tf/(Tf+Ts))*ud(k-1) + (Kd/(Tf+Ts))*(e(k)-e(k-1))
    ud = (Tf / (Tf + Ts)) * ud
       + (Kd / (Tf + Ts)) * (error - e_prev);
    e_prev = error;

    // ── 7. Término Integral con Anti-Windup (Back-Calculation) ─
    // ui(k) = ui(k-1) + Ts * (Ki*e(k) + Kaw*sat_error(k-1))
    ui += Ts * (Ki * error + Kaw * sat_error_prev);

    // Clamping directo del integrador (barrera adicional)
    ui = saturar(ui, V_MIN, V_MAX);

    // ── 8. Señal de control y saturación ──────────────────────
    float v_calc = up + ui + ud;
    float v_sat  = saturar(v_calc, V_MIN, V_MAX);

    // Asistencia práctica: evita quedar "pegado" en fricción estática
    // cuando el error es claro pero el voltaje calculado es insuficiente.
    if (abs(error) > ERROR_MIN_MOV_RAD && abs(v_sat) < V_MIN_CONTROL) {
        float signo = (v_sat != 0.0f) ? v_sat : error;
        v_sat = (signo >= 0.0f) ? V_MIN_CONTROL : -V_MIN_CONTROL;
    }

    // Error de saturación para el próximo ciclo (back-calculation)
    sat_error_prev = v_sat - v_calc;

    // ── 9. Aplicar al puente H ─────────────────────────────────
    aplicarPWM(v_sat);

    // ── 10. Panel de depuración + salida CSV ───────────────────
    unsigned long t_ms = millis();
    if ((unsigned long)(t_ms - t_telemetria_ms) >= T_TELEMETRIA_MS) {
        t_telemetria_ms = t_ms;

        Serial.println(F("=============================================="));
        Serial.println(F("           DEBUG PID MOTOR DC"));
        Serial.println(F("=============================================="));

        Serial.print(F("Ref [deg]          : "));
        Serial.println(referencia_rad * (180.0f / PI), 2);

        Serial.print(F("Posicion [rad]     : "));
        Serial.println(y, 4);

        Serial.print(F("Pulsos encoder     : "));
        Serial.println(p);

        Serial.print(F("Error [rad]        : "));
        Serial.println(error, 4);

        Serial.println(F("----------------------------------------------"));
        Serial.print(F("Esfuerzo total [V] : "));
        Serial.println(v_sat, 3);

        Serial.print(F("P [V]              : "));
        Serial.println(up, 3);

        Serial.print(F("I [V]              : "));
        Serial.println(ui, 3);

        Serial.print(F("D [V]              : "));
        Serial.println(ud, 3);

        Serial.println(F("=============================================="));

        // Salida CSV para logging en PC.
        Serial.print(F("CSV,"));
        Serial.print(t_ms); Serial.print(',');
        Serial.print(referencia_rad * (180.0f / PI), 2); Serial.print(',');
        Serial.print(y, 4); Serial.print(',');
        Serial.print(error, 4); Serial.print(',');
        Serial.print(v_sat, 3); Serial.print(',');
        Serial.print(up, 3); Serial.print(',');
        Serial.print(ui, 3); Serial.print(',');
        Serial.println(ud, 3);
    }
}

// ============================================================
//  Aplica voltaje [V_MIN, V_MAX] al puente H vía PWM
//  Mapea ±V_MAX → 0–255 en duty cycle con control de dirección.
//  Incluye compensación práctica de caída y umbral de arranque.
// ============================================================
void aplicarPWM(float voltaje) {
    float v_abs = abs(voltaje);
    int pwm_val = 0;

    if (v_abs >= UMBRAL_ARRANQUE_V) {
        // Compensa caida del L298N y garantiza un duty minimo de arranque.
        float v_comp = v_abs + L298N_DROP_V;
        v_comp = constrain(v_comp, 0.0f, V_MAX);

        pwm_val = (int)(v_comp / V_MAX * 255.0f);
        if (pwm_val > 0 && pwm_val < PWM_MIN_ARRANQUE) {
            pwm_val = PWM_MIN_ARRANQUE;
        }
        pwm_val = constrain(pwm_val, 0, 255);
    }
    
    if (voltaje >= 0.0f) {
        digitalWrite(PIN_DIR_A, HIGH);
        digitalWrite(PIN_DIR_B, LOW);
    } else {
        digitalWrite(PIN_DIR_A, LOW);
        digitalWrite(PIN_DIR_B, HIGH);
    }
    analogWrite(PIN_PWM, pwm_val);
}

// ============================================================
//  Lee comandos/referencia desde Serial (no bloqueante)
//  Comandos: START, STOP, HELP, MOTOR <V>, o referencia en grados
//  Ejemplos: "START", "MOTOR 6", "90"
// ============================================================
void leerSerialReferencia() {
    while (Serial.available() > 0) {
        char c = (char)Serial.read();

        if (c == '\r') continue;

        if (c == '\n') {
            if (idx_serial > 0) {
                buffer_serial[idx_serial] = '\0';
                procesarLineaSerial(buffer_serial);
            }
            idx_serial = 0;
            continue;
        }

        if (idx_serial < sizeof(buffer_serial) - 1) {
            buffer_serial[idx_serial++] = c;
        }
    }
}

void resetControlador() {
    ui = 0.0f;
    ud = 0.0f;
    e_prev = 0.0f;
    sat_error_prev = 0.0f;
}

void procesarLineaSerial(char* linea) {
    while (*linea != '\0' && isspace(*linea)) linea++;
    if (*linea == '\0') return;

    char* fin = linea + strlen(linea) - 1;
    while (fin > linea && isspace(*fin)) {
        *fin = '\0';
        fin--;
    }

    char cmd[24];
    size_t i = 0;
    while (linea[i] != '\0' && i < sizeof(cmd) - 1) {
        cmd[i] = (char)toupper((unsigned char)linea[i]);
        i++;
    }
    cmd[i] = '\0';

    if (strcmp(cmd, "START") == 0) {
        noInterrupts();
        pulsos = 0;
        interrupts();

        referencia_rad = 0.0f;
        resetControlador();
        control_habilitado = true;
        cero_realizado = true;

        Serial.println(F(">> START recibido: cero tomado en posicion actual."));
        Serial.println(F(">> Control habilitado."));
        return;
    }

    if (strcmp(cmd, "STOP") == 0) {
        control_habilitado = false;
        aplicarPWM(0.0f);
        Serial.println(F(">> STOP recibido: control deshabilitado."));
        return;
    }

    if (strcmp(cmd, "HELP") == 0) {
        Serial.println(F("Comandos disponibles:"));
        Serial.println(F("  START -> fija posicion actual como cero y habilita control"));
        Serial.println(F("  STOP  -> deshabilita control y PWM=0"));
        Serial.println(F("  MOTOR <V> -> prueba manual en lazo abierto, ejemplo: MOTOR 6"));
        Serial.println(F("  <num> -> referencia en grados (ej: 30)"));
        return;
    }

    if (strncmp(cmd, "MOTOR ", 6) == 0) {
        char* endptr_motor = nullptr;
        float v_cmd = (float)strtod(linea + 6, &endptr_motor);
        while (endptr_motor != nullptr && *endptr_motor != '\0' && isspace(*endptr_motor)) endptr_motor++;

        if (endptr_motor != (linea + 6) && endptr_motor != nullptr && *endptr_motor == '\0') {
            control_habilitado = false;
            float v_aplicar = saturar(v_cmd, V_MIN, V_MAX);
            aplicarPWM(v_aplicar);

            Serial.print(F(">> MOTOR manual: aplicando "));
            Serial.print(v_aplicar, 2);
            Serial.println(F(" V (control deshabilitado)."));
            return;
        }

        Serial.println(F(">> Formato invalido. Usa: MOTOR <voltaje> (ej: MOTOR 6)"));
        return;
    }

    char* endptr = nullptr;
    float nueva_deg = (float)strtod(linea, &endptr);
    while (endptr != nullptr && *endptr != '\0' && isspace(*endptr)) endptr++;

    if (endptr != linea && endptr != nullptr && *endptr == '\0') {
        if (!control_habilitado) {
            Serial.println(F(">> Control en espera. Envia START tras ubicar el eje en cero."));
            return;
        }

        referencia_rad = nueva_deg * (PI / 180.0f);
    #if CTRL_RESET_ON_SETPOINT
        resetControlador();
    #endif

        Serial.print(F(">> Nueva referencia: "));
        Serial.print(nueva_deg, 2);
        Serial.print(F(" deg ("));
        Serial.print(referencia_rad, 4);
        Serial.println(F(" rad)"));
        return;
    }

    Serial.println(F(">> Entrada no valida. Envia HELP para ver comandos."));
}

void actualizarKaw() {
#if CTRL_KAW_MODE == 0
    Kaw = Kp / 10.0f;
#elif CTRL_KAW_MODE == 1
    float prod = Ki * Kd;
    Kaw = (prod > 0.0f) ? sqrt(prod) : 0.0f;
#else
    Kaw = CTRL_KAW_FIXED;
#endif
}

void evaluarConstantes() {
    bool advertencia = false;

    Serial.print(F("[CFG] Ts_us="));
    Serial.print(TS_US);
    Serial.print(F(" | Vsat=["));
    Serial.print(V_MIN, 1);
    Serial.print(F(", "));
    Serial.print(V_MAX, 1);
    Serial.print(F("] | Kaw="));
    Serial.println(Kaw, 5);

    if ((Kp < 0.0f) || (Ki < 0.0f) || (Kd < 0.0f) || (Tf <= 0.0f) || (Kaw < 0.0f)) {
        Serial.println(F("[WARN] PID: hay constantes negativas o Tf <= 0."));
        advertencia = true;
    }

    if (abs((long)(Ts * 1e6f) - (long)TS_US) > 2) {
        Serial.println(F("[WARN] Ts y TS_US no coinciden. Revisar temporizacion."));
        advertencia = true;
    }

    if ((L298N_DROP_V < 0.3f) || (L298N_DROP_V > 3.0f)) {
        Serial.println(F("[WARN] L298N_DROP_V fuera de rango tipico (0.3 a 3.0 V)."));
        advertencia = true;
    }

    if ((PWM_MIN_ARRANQUE < 20) || (PWM_MIN_ARRANQUE > 180)) {
        Serial.println(F("[WARN] PWM_MIN_ARRANQUE fuera de rango esperado (20 a 180)."));
        advertencia = true;
    }

    if (UMBRAL_ARRANQUE_V > (0.2f * V_MAX)) {
        Serial.println(F("[WARN] UMBRAL_ARRANQUE_V alto: puede bloquear movimiento fino."));
        advertencia = true;
    }

    if ((Ki * Ts) < 1e-6f) {
        Serial.println(F("[WARN] Ki*Ts muy pequeno: accion integral puede ser lenta."));
        advertencia = true;
    }

    if (!advertencia) {
        Serial.println(F("[OK] Evaluacion de constantes sin alertas criticas."));
    }
}

// ============================================================
//  Saturación genérica
// ============================================================
float saturar(float val, float vmin, float vmax) {
    if (val > vmax) return vmax;
    if (val < vmin) return vmin;
    return val;
}

// ============================================================
//  ISRs del encoder en cuadratura
// ============================================================
void ISR_encoderA() {
    #if defined(ARDUINO_ARCH_AVR)
    uint8_t portd = PIND;
    bool a = (portd & ENC_A_MASK) != 0;
    bool b = (portd & ENC_B_MASK) != 0;
    #else
    bool a = digitalRead(PIN_ENC_A);
    bool b = digitalRead(PIN_ENC_B);
    #endif
    pulsos += (a == b) ? +1 : -1;
}

void ISR_encoderB() {
    #if defined(ARDUINO_ARCH_AVR)
    uint8_t portd = PIND;
    bool a = (portd & ENC_A_MASK) != 0;
    bool b = (portd & ENC_B_MASK) != 0;
    #else
    bool a = digitalRead(PIN_ENC_A);
    bool b = digitalRead(PIN_ENC_B);
    #endif
    pulsos += (a != b) ? +1 : -1;
}
