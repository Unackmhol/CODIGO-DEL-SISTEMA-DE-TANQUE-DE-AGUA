// ══════════════════════════════════════════════════
//  Sistema Control de Nivel de Agua
//  Control PID → PWM analógico sobre MOSFET IRFZ44N
//  LEDs: Rojo parpadea <25% | Rojo fijo >25% | 
//        Amarillo >45% | Verde 70-94% | Todos parpadean >95%
// ══════════════════════════════════════════════════

// ── Pines ──────────────────────────────────────────
#define TRIG_PIN    9
#define ECHO_PIN    10
#define MOSFET_PIN  3   // PWM analógico (pin 3, 5 o 6 en Arduino UNO)
#define BUZZER_PIN  6
#define LED_ROJO    5
#define LED_AMARILLO 4
#define LED_VERDE   2

// ── Dimensiones físicas del tanque ─────────────────
#define DIST_FONDO  9.0
#define DIST_TOPE   4.0

// ── Umbrales ────────────────────────────────────────
#define NIVEL_MIN        20.0
#define NIVEL_ROJO_FIJO  25.0
#define NIVEL_AMARILLO   45.0
#define NIVEL_SETPOINT   70.0
#define NIVEL_ALARMA     95.0

// ── Ganancias PID ──────────────────────────────────
float Kp = 2.0;
float Ki = 0.1;
float Kd = 0.3;

// ── Filtro promedio móvil ───────────────────────────
#define VENTANA 10
float buffer[VENTANA];
int   bufIndex = 0;
bool  bufLleno = false;

// ── Variables PID ───────────────────────────────────
float integral       = 0;
float errorAnterior  = 0;
unsigned long tAnterior = 0;

// ── Máquina de estados ──────────────────────────────
enum Estado { REPOSO, LLENANDO, MANTENIENDO };
Estado estadoActual = REPOSO;

// ── Alarmas ─────────────────────────────────────────
bool alertaBajoActiva  = false;
bool alertaLlenoActiva = false;

// ── Parpadeo LEDs (sin delay) ───────────────────────
unsigned long tParpadeo      = 0;
bool          estadoParpadeo = false;
#define INTERVALO_PARPADEO 300  // ms

// ══════════════════════════════════════════════════
//  Funciones de sensado y filtrado
// ══════════════════════════════════════════════════
float distanciaAPorcentaje(float dist) {
  float nivel = (DIST_FONDO - dist) / (DIST_FONDO - DIST_TOPE) * 100.0;
  return constrain(nivel, 0.0, 100.0);
}

float medirDistancia() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long duracion = pulseIn(ECHO_PIN, HIGH, 30000);
  if (duracion == 0) return DIST_FONDO;
  return duracion * 0.01715;
}

float promedioMovil(float nuevo) {
  buffer[bufIndex] = nuevo;
  bufIndex = (bufIndex + 1) % VENTANA;
  if (bufIndex == 0) bufLleno = true;
  int n = bufLleno ? VENTANA : bufIndex;
  float suma = 0;
  for (int i = 0; i < n; i++) suma += buffer[i];
  return suma / n;
}

void pitarBuzzer(int veces) {
  for (int i = 0; i < veces; i++) {
    digitalWrite(BUZZER_PIN, HIGH); delay(200);
    digitalWrite(BUZZER_PIN, LOW);  delay(200);
  }
}

// ══════════════════════════════════════════════════
//  Control de LEDs según nivel
//  Sin delay() — usa millis() para parpadeo
// ══════════════════════════════════════════════════
void actualizarLEDs(float nivel) {
  unsigned long ahora = millis();

  // Actualiza el estado de parpadeo cada INTERVALO_PARPADEO ms
  if (ahora - tParpadeo >= INTERVALO_PARPADEO) {
    tParpadeo     = ahora;
    estadoParpadeo = !estadoParpadeo;
  }

  // Por defecto apaga todo
  bool rojo    = false;
  bool amarillo = false;
  bool verde   = false;

  if (nivel >= NIVEL_ALARMA) {
    // ≥ 95%: todos parpadean juntos
    rojo     = estadoParpadeo;
    amarillo = estadoParpadeo;
    verde    = estadoParpadeo;

  } else if (nivel >= NIVEL_SETPOINT) {
    // 70% – 94%: solo verde fijo
    verde = true;

  } else if (nivel >= NIVEL_AMARILLO) {
    // 45% – 69%: rojo fijo + amarillo fijo
    rojo     = true;
    amarillo = true;

  } else if (nivel >= NIVEL_ROJO_FIJO) {
    // 25% – 44%: solo rojo fijo
    rojo = true;

  } else if (nivel >= NIVEL_MIN) {
    // 20% – 24%: rojo parpadea
    rojo = estadoParpadeo;

  }
  // < 20%: todos apagados (el sistema está en reposo aún)

  digitalWrite(LED_ROJO,    rojo    ? HIGH : LOW);
  digitalWrite(LED_AMARILLO, amarillo ? HIGH : LOW);
  digitalWrite(LED_VERDE,   verde   ? HIGH : LOW);
}

// ══════════════════════════════════════════════════
//  PID → retorna valor 0-255 para analogWrite
// ══════════════════════════════════════════════════
int calcularPID(float nivelActual) {
  unsigned long ahora = millis();
  float dt = (ahora - tAnterior) / 1000.0;
  if (dt < 0.01) dt = 0.01;

  float error = NIVEL_SETPOINT - nivelActual;

  float P = Kp * error;

  integral += error * dt;
  integral  = constrain(integral, -100, 100);
  float I   = Ki * integral;

  float D = Kd * ((error - errorAnterior) / dt);

  float salida = P + I + D;
  salida = constrain(salida, 0, 100);

  errorAnterior = error;
  tAnterior     = ahora;

  Serial.print("SP:"); Serial.print(NIVEL_SETPOINT, 0);
  Serial.print("% | Nivel:"); Serial.print(nivelActual, 1);
  Serial.print("% | Err:"); Serial.print(error, 1);
  Serial.print(" | P:"); Serial.print(P, 2);
  Serial.print(" | I:"); Serial.print(I, 2);
  Serial.print(" | D:"); Serial.print(D, 2);
  Serial.print(" | PWM:"); Serial.println((int)(salida * 2.55));

  return (int)(salida * 2.55);
}

// ══════════════════════════════════════════════════
void setup() {
  Serial.begin(9600);
  pinMode(TRIG_PIN,    OUTPUT);
  pinMode(ECHO_PIN,    INPUT);
  pinMode(MOSFET_PIN,  OUTPUT);
  pinMode(BUZZER_PIN,  OUTPUT);
  pinMode(LED_ROJO,    OUTPUT);
  pinMode(LED_AMARILLO,OUTPUT);
  pinMode(LED_VERDE,   OUTPUT);

  analogWrite(MOSFET_PIN, 0);
  digitalWrite(BUZZER_PIN,  LOW);
  digitalWrite(LED_ROJO,    LOW);
  digitalWrite(LED_AMARILLO,LOW);
  digitalWrite(LED_VERDE,   LOW);

  tAnterior = millis();
  Serial.println("=== Sistema iniciado — Esperando nivel < 20% ===");
}

// ══════════════════════════════════════════════════
void loop() {

  // 1. Leer y filtrar nivel
  float distRaw  = medirDistancia();
  float distFilt = promedioMovil(distRaw);
  float nivel    = distanciaAPorcentaje(distFilt);

  // 2. Actualizar LEDs (siempre, en todo estado)
  actualizarLEDs(nivel);

  // ── ALARMA SOBRELLENADO ≥ 95% ──────────────────
  if (nivel >= NIVEL_ALARMA) {
    analogWrite(MOSFET_PIN, 0);
    if (!alertaLlenoActiva) {
      alertaLlenoActiva = true;
      pitarBuzzer(3);
    }
    estadoActual = REPOSO;
    integral = 0;
    Serial.print("⚠ EMERGENCIA SOBRELLENADO: ");
    Serial.print(nivel, 1); Serial.println("%");
    delay(200);
    return;
  } else {
    alertaLlenoActiva = false;
  }

  // ── ALARMA NIVEL BAJO < 20% ────────────────────
  if (nivel < NIVEL_MIN) {
    if (!alertaBajoActiva) {
      alertaBajoActiva = true;
      pitarBuzzer(2);
    }
  } else {
    alertaBajoActiva = false;
  }

  // ── MÁQUINA DE ESTADOS ─────────────────────────
  switch (estadoActual) {

    case REPOSO:
      analogWrite(MOSFET_PIN, 0);
      if (nivel < NIVEL_MIN) {
        estadoActual  = LLENANDO;
        integral      = 0;
        errorAnterior = 0;
        tAnterior     = millis();
        Serial.println("→ ESTADO: LLENANDO");
      }
      break;

    case LLENANDO:
      {
        int pwm = calcularPID(nivel);
        analogWrite(MOSFET_PIN, pwm);

        if (nivel >= NIVEL_SETPOINT) {
          estadoActual = MANTENIENDO;
          Serial.println("→ ESTADO: MANTENIENDO en 70%");
        }
        if (nivel < NIVEL_MIN) {
          analogWrite(MOSFET_PIN, 0);
          estadoActual = REPOSO;
          Serial.println("→ ESTADO: REPOSO");
        }
      }
      break;

    case MANTENIENDO:
      {
        int pwm = calcularPID(nivel);
        analogWrite(MOSFET_PIN, pwm);

        if (nivel < NIVEL_MIN) {
          estadoActual  = LLENANDO;
          integral      = 0;
          errorAnterior = 0;
          tAnterior     = millis();
          Serial.println("→ ESTADO: LLENANDO (nuevo ciclo)");
        }
      }
      break;
  }

  delay(100);
}