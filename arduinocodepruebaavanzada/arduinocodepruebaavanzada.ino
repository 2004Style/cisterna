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
bool humedadActiva = false; // Flag para controlar el sensor de humedad

// Variables de tiempo para cada sensor

unsigned long previousMillisUltrasonidoFrontal = 0;
unsigned long previousMillisUltrasonidoTrasero = 0;
unsigned long previousMillisLDR = 0;
unsigned long previousMillisHumedad = 0;
unsigned long previousMillisRecepcion = 0;

const long intervalUltrasonidoFrontal = 1000;
const long intervalUltrasonidoTrasero = 1000;
const long intervalLDR = 1000;
const long intervalHumedad = 1000;
const long intervalRecepcion = 1000;

void setup()
{
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

void loop()
{
  unsigned long currentMillis = millis();

  // Leer el sensor de ultrasonido frontal cada 1 segundo
  if (currentMillis - previousMillisUltrasonidoFrontal >= intervalUltrasonidoFrontal)
  {
    previousMillisUltrasonidoFrontal = currentMillis;
    long DistanciaFrontal = leerUltrasonido(HCSR_FrontalTrigger, HCSR_FrontalEcho);
    Serial.println("UF:" + String(DistanciaFrontal));
  }

  // Leer el sensor de ultrasonido trasero cada 1 segundo
  if (currentMillis - previousMillisUltrasonidoTrasero >= intervalUltrasonidoTrasero)
  {
    previousMillisUltrasonidoTrasero = currentMillis;
    long DistanciaTrasera = leerUltrasonido(HCSR_TraseroTrigger, HCSR_TraseroEcho);
    Serial.println("UB:" + String(DistanciaTrasera));
  }

  // Leer las fotoresistencias cada 1 segundo
  if (currentMillis - previousMillisLDR >= intervalLDR)
  {
    previousMillisLDR = currentMillis;
    leerLDRs();
  }

  // Leer el sensor de humedad cada 1 segundo (solo si está activo)
  if (humedadActiva && (currentMillis - previousMillisHumedad >= intervalHumedad))
  {
    previousMillisHumedad = currentMillis;
    int LecturaHumedad = analogRead(SensorHumedad);
    Serial.println("LH:" + String(LecturaHumedad));
  }

  // Leer datos recibidos desde el ESP32 de manera asíncrona
  if (currentMillis - previousMillisRecepcion >= intervalRecepcion)
  {
    previousMillisRecepcion = currentMillis;
    leerDatosESP32();
  }
}

// Función para leer la distancia de un sensor de ultrasonido
long leerUltrasonido(int triggerPin, int echoPin)
{
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
void leerLDRs()
{
  int LDRX1Value = analogRead(LDRX1);
  int LDRX2Value = analogRead(LDRX2);
  int LDRY1Value = analogRead(LDRY1);
  int LDRY2Value = analogRead(LDRY2);

  // Enviar los valores leídos
  Serial.println("LDRX1:" + String(LDRX1Value) +
                 " LDRX2:" + String(LDRX2Value) +
                 " LDRY1:" + String(LDRY1Value) +
                 " LDRY2:" + String(LDRY2Value));
}

// Función para leer datos enviados desde el ESP32
void leerDatosESP32()
{
  if (Serial.available() > 0)
  {
    String datosRecibidos = Serial.readStringUntil('\n'); // Lee datos hasta el salto de línea
    Serial.println("Datos recibidos del ESP32: " + datosRecibidos);

    // Verificar si el comando recibido es "lectorDeHumedadActivo"
    if (datosRecibidos == "lectorDeHumedadActivo")
    {
      humedadActiva = true;
      Serial.println("Sensor de humedad activado.");
    }
    // Verificar si el comando recibido es "lectorDeHumedadInactivo"
    else if (datosRecibidos == "lectorDeHumedadInactivo")
    {
      humedadActiva = false;
      Serial.println("Sensor de humedad desactivado.");
    }
  }
}
