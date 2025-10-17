#include "MOTORDC.h"
#include "HOTEND.h"
#include "Pin_map.h"
#include <AccelStepper.h>
#include <Arduino.h>

// --- VARIABLES GLOBALES ---
String inputSerial = "";
String digits = "0000";  // Estado de los actuadores [Motor DC, Fan, Extrusor, Hotend]

// --- MOTOR EXTRUSOR (STEPPER) ---
AccelStepper motor2(AccelStepper::DRIVER, 26, 28);
const int enablePin2 = 24;
bool motor2Enabled = false;

// --- CONTROL PID DE MOTOR DC ---
double setpoint_Motor = 20.0; // RPM objetivo Maximo 55
double Kp_M = 6.5, Ki_M = 2.5, Kd_M = 0.6;

// --- CONTROL PID DE HOTEND ---
double setpoint_Hotend = 190.0;
float Kp_H = 2.0, Ki_H = 0.1, Kd_H = 1.0;

// --- ESTADOS Y TEMPORIZADORES ---
int moto_m = 0, fan_m = 0, heater_m = 0, extruder_m = 0;
unsigned long lastStatusUpdate = 0;

// --- CONFIGURACIÓN INICIAL ---
void setup() {
  Serial.begin(115200);
  Serial.println("Conexión establecida con control PID personalizado.");

  // Pines de salida
  pinMode(pinFan, OUTPUT);
  pinMode(enablePin2, OUTPUT);
  pinMode(pinHotend, OUTPUT);
  pinMode(pinMotor, OUTPUT);
  pinMode(C1, INPUT);
  pinMode(C2, INPUT);

  // Inicializar actuadores
  digitalWrite(enablePin2, HIGH);
  digitalWrite(pinHotend, LOW);
  analogWrite(pinMotor, 0);

  // Configuración del motor extrusor
  motor2.setMaxSpeed(2000);
  motor2.setAcceleration(1000);

  // Interrupciones del encoder
  attachInterrupt(digitalPinToInterrupt(C1), encoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(C2), encoder, CHANGE);

  prevTime = millis();
}

// --- BUCLE PRINCIPAL ---
void loop() {
  // --- Lectura de comandos seriales ---
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      processInput(inputSerial);
      inputSerial = "";
    } else {
      inputSerial += c;
    }
  }

  // --- PID cada 500 ms ---
  unsigned long now = millis();
  if (now - prevTime >= 500) {
    float dt = (now - prevTime) / 1000.0;
    prevTime = now;

    // Sensores
    float temp_actual = thermistor(analogRead(termPin));
    computeRpm(); // Actualiza N (RPM global en MOTORDC.h)

    // --- PID HOTEND ---
    if (digits.length() >= 4 && digits[3] == '1') {
      if (temp_actual == -999 || temp_actual > maxTemp || temp_actual < 10) {
        digitalWrite(pinHotend, LOW);
        integral_H = 0;
        Serial.println("¡SHUTDOWN DE SEGURIDAD!");
      } else {
        double pidPWM_Hotend = PIDHotend(temp_actual, dt, setpoint_Hotend, Kp_H, Ki_H, Kd_H);
        int pwmTemp = constrain((int)pidPWM_Hotend, 0, 255);
        analogWrite(pinHotend, pwmTemp);
      }
      heater_m = 1;
    } else {
      analogWrite(pinHotend, 0);
      heater_m = 0;
    }

    // --- PID MOTOR DC ---
    if (digits.length() >= 1 && digits[0] == '1') {
      input = N; // "input" es global de MOTORDC.h
      double pidPWM_Motor = PIDMotor(dt, setpoint_Motor, Kp_M, Ki_M, Kd_M);
      int pwmMotorOut = constrain((int)pidPWM_Motor, 10, 255);
      analogWrite(pinMotor, pwmMotorOut);
      moto_m = 1;
    } else {
      analogWrite(pinMotor, 0);
      moto_m = 0;
    }
  }

  // --- FAN ---
  if (digits.length() >= 2 && digits[1] == '1') {
    analogWrite(pinFan, 150);
    fan_m = 1;
  } else {
    analogWrite(pinFan, 0);
    fan_m = 0;
  }

  // --- EXTRUDER ---
  if (digits.length() >= 3 && digits[2] == '1') {
    if (!motor2Enabled) {
      digitalWrite(enablePin2, LOW);
      motor2Enabled = true;
    }
    motor2.runSpeed();
    extruder_m = 1;
  } else {
    if (motor2Enabled) {
      digitalWrite(enablePin2, HIGH);
      motor2Enabled = false;
    }
    extruder_m = 0;
  }

  // --- ENVÍO DE ESTADO ---
  unsigned long currentMillis = millis();
  if (currentMillis - lastStatusUpdate >= 1000) {
    lastStatusUpdate = currentMillis;
    Serial.print("Temp:");
    Serial.println(thermistor(analogRead(termPin)));
    Serial.println("--- Estado de componentes ---");
    Serial.print("Motor DC:   "); Serial.println(moto_m ? "Encendido" : "Apagado");
    Serial.print("Fan:        "); Serial.println(fan_m ? "Encendido" : "Apagado");
    Serial.print("Heater:     "); Serial.println(heater_m ? "Encendido" : "Apagado");
    Serial.print("Extruder:   "); Serial.println(extruder_m ? "Encendido" : "Apagado");
    Serial.println("-----------------------------");
  }
}

// --- COMANDOS SERIAL ---
void processInput(String command) {
  Serial.print("Recibido: ");
  Serial.println(command);

  if (command.startsWith("ACTUATE:")) {
    digits = command.substring(8);
  }
  else if (command.startsWith("SPEED:")) { // Extrusor
    int nuevaVel = command.substring(6).toInt();
    motor2.setSpeed(nuevaVel);
    Serial.print("Nueva velocidad extrusor: ");
    Serial.println(nuevaVel);
  }
  else if (command.startsWith("TEMP:")) { // Hotend
    double nuevaTemp = command.substring(5).toDouble();
    setpoint_Hotend = nuevaTemp;
    Serial.print("Nuevo Setpoint de temperatura: ");
    Serial.println(setpoint_Hotend);
  }
  else if (command.startsWith("DCSPEED:")) { // Motor DC
    double nuevaVelDC = command.substring(8).toDouble();
    setpoint_Motor = nuevaVelDC;
    Serial.print("Nuevo Setpoint de RPM para motor DC: ");
    Serial.println(setpoint_Motor);
  }
}
