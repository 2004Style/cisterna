// Sensores de proximidad
int HCSR_FrontalEcho = 5;
int HCSR_FrontalTrigger = 6;
int HCSR_TraseroEcho = 10;
int HCSR_TraseroTrigger = 11;

// Fotoresistencias
int LDRX1 = A0;
int LDRX2 = A1;
int LDRY1 = A2;
int LDRY2 = A3;

// Sensor de humedad
int SensorHumedad = A4;
bool humedadActiva = false;  // Flag para controlar el sensor de humedad

// Variables de tiempo para cada sensor
unsigned long previousMillisUltrasonidoFrontal = 0;
unsigned long previousMillisUltrasonidoTrasero = 0;
unsigned long previousMillisLDR = 0;
unsigned long previousMillisHumedad = 0;
unsigned long previousMillisEnviar = 0;
unsigned long previousMillisRecepcion = 0;

const long intervalUltrasonidoFrontal = 500;
const long intervalUltrasonidoTrasero = 500;
const long intervalLDR = 1000;
const long intervalHumedad = 500;
const long intervalRecepcion = 500;
const long intervalEnviar = 1000;

long DistanciaFrontal = 0;
long DistanciaTrasera = 0;
int LecturaHumedad = 0;
int LDRX1Value = 0;
int LDRX2Value = 0;
int LDRY1Value = 0;
int LDRY2Value = 0;

void setup() {
  Serial.begin(9600);

  pinMode(HCSR_FrontalTrigger, OUTPUT);
  pinMode(HCSR_FrontalEcho, INPUT);
  pinMode(HCSR_TraseroTrigger, OUTPUT);
  pinMode(HCSR_TraseroEcho, INPUT);

  pinMode(LDRX1, INPUT);
  pinMode(LDRX2, INPUT);
  pinMode(LDRY1, INPUT);
  pinMode(LDRY2, INPUT);

  pinMode(SensorHumedad, INPUT);
}

void loop() {
  unsigned long currentMillis = millis();
  // Leer datos recibidos desde el ESP32 de manera asíncrona cada 1 segundo
  if (currentMillis - previousMillisRecepcion >= intervalRecepcion) {
    previousMillisRecepcion = currentMillis;
    if (Serial.available()) {
      String datosRecibidos = Serial.readStringUntil('\n');  // Lee datos hasta el salto de línea
      //Serial.println(datosRecibidos);
      String Humedad_Obtenida_esp32 = datosRecibidos;
      Serial.println(Humedad_Obtenida_esp32);
    }
  }

  // Leer el sensor de ultrasonido frontal cada 1 segundo
  if (currentMillis - previousMillisUltrasonidoFrontal >= intervalUltrasonidoFrontal) {
    previousMillisUltrasonidoFrontal = currentMillis;
    DistanciaFrontal = leerUltrasonido(HCSR_FrontalTrigger, HCSR_FrontalEcho);
  }

  // Leer el sensor de ultrasonido trasero cada 1 segundo
  if (currentMillis - previousMillisUltrasonidoTrasero >= intervalUltrasonidoTrasero) {
    previousMillisUltrasonidoTrasero = currentMillis;
    DistanciaTrasera = leerUltrasonido(HCSR_TraseroTrigger, HCSR_TraseroEcho);
  }

  // Leer las fotoresistencias cada 1 segundo
  if (currentMillis - previousMillisLDR >= intervalLDR) {
    previousMillisLDR = currentMillis;
    leerLDRs();
  }

  // Leer el sensor de humedad cada 1 segundo (solo si está activo)
  if (humedadActiva && (currentMillis - previousMillisHumedad >= intervalHumedad)) {
    previousMillisHumedad = currentMillis;
    int LecturaHumedad = analogRead(SensorHumedad);
  }


  // enviar los datos al ESP32 cada 1 segundo
  if (currentMillis - previousMillisEnviar >= intervalEnviar) {
    previousMillisEnviar = currentMillis;
    Serial.println("UF:" + String(DistanciaFrontal) + " UB:" + String(DistanciaTrasera) + " LH:" + String(LecturaHumedad) + " LDRX1:" + String(LDRX1Value) + " LDRX2:" + String(LDRX2Value) + " LDRY1:" + String(LDRY1Value) + " LDRY2:" + String(LDRY2Value));
  }
}

// Función para leer la distancia de un sensor de ultrasonido
long leerUltrasonido(int triggerPin, int echoPin) {
  digitalWrite(triggerPin, LOW);
  delayMicroseconds(2);
  digitalWrite(triggerPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(triggerPin, LOW);

  long duracion = pulseIn(echoPin, HIGH);
  long distancia = duracion * 0.034 / 2;

  return distancia;
}

// Función para leer las fotoresistencias (LDRs)
void leerLDRs() {
  LDRX1Value = analogRead(LDRX1);
  LDRX2Value = analogRead(LDRX2);
  LDRY1Value = analogRead(LDRY1);
  LDRY2Value = analogRead(LDRY2);
}
