#include <WiFi.h>
//#include <ESP32Servo360.h>
#include <ESP32Servo.h>

// Definición de variables
const char *ssid = "ConvertSystems";
// const char *password = "style1234";
const int tcpPort = 80;  // Puerto para el servidor TCP

WiFiServer server(tcpPort);

//variables para los componentes fisicos del robot
int ElectroBomba = 26;

Servo servoTorretaX;
Servo servoTorretaY;
int gradosX = 0;
int gradosY = 0;
const int MAX_GRADOS = 29;

Servo ServoPanelX;
Servo ServoPanelY;
// Umbrales para controlar el movimiento
const int UMBRAL_SUPERIOR = 100;  // Umbral superior para empezar el movimiento
const int UMBRAL_INFERIOR = 50;   // Umbral inferior para detener el movimiento
int diffX, diffY;                 // Diferencias entre los LDRs

// Posiciones del servo
const int POS_DERECHA = 180;
const int POS_IZQUIERDA = 0;
const int POS_CENTRO = 90;

int LucesFrontales = 27;
int LucesTraseras = 14;

int IntermitentesDerechos = 12;
int IntermitentesIzquierdos = 13;

int MotorDireccionDerecha1 = 23;
int MotorDireccionDerecha2 = 22;

int MotorDireccionIzquierda1 = 19;
int MotorDireccionIzquierda2 = 21;

int ServoHumedad = 4;


// Variables para controlar el robot
String Direccion_robot = "STOP";
String Direccion_torreta = "STOP";
String Luces_frontales = "LF_OFF";
String Luces_traseras = "OFF";
String Luces_intermitentes = "OFF";
String Estado_humedad = "H_OFF";

//variables de datos desde arduino
String DistanciaFront = "0";
String DistanciaBack = "0";

int LDRX1 = 0;
int LDRX2 = 0;
int LDRY1 = 0;
int LDRY2 = 0;

int Humedad = 0;

// Prototipos de las funciones de las tareas
void TaskIntermitentes(void *pvParameters);
void TaskLucesDelanteras(void *pvParameters);
void TaskTorreta(void *pvParameters);
void TaskHumedad(void *pvParameters);
void TaskTanque(void *pvParameters);
void TaskComunicacionArduino(void *pvParameters);
void TaskSeguirLuz(void *pvParameters);

void setup() {
  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1, 16, 17);

  pinMode(ElectroBomba, OUTPUT);

  pinMode(LucesFrontales, OUTPUT);
  pinMode(LucesTraseras, OUTPUT);

  pinMode(IntermitentesDerechos, OUTPUT);
  pinMode(IntermitentesIzquierdos, OUTPUT);

  pinMode(MotorDireccionDerecha1, OUTPUT);
  pinMode(MotorDireccionDerecha2, OUTPUT);

  pinMode(MotorDireccionIzquierda1, OUTPUT);
  pinMode(MotorDireccionIzquierda2, OUTPUT);

  pinMode(ServoHumedad, OUTPUT);

  servoTorretaX.attach(18);
  servoTorretaY.attach(5);
  ServoPanelX.attach(2);
  ServoPanelY.attach(15);

  // Conexión a la red Wi-Fi
  //WiFi.begin(ssid, password);
  WiFi.begin(ssid);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Conectando a WiFi...");
  }
  Serial.println("Conectado a WiFi");

  // Iniciar el servidor
  server.begin();
  Serial.println("Servidor iniciado");
  Serial.print("Dirección IP: ");
  Serial.print(WiFi.localIP());
  Serial.print(" Port: ");
  Serial.println(tcpPort);

  // Crear tareas para cada función de control

  xTaskCreatePinnedToCore(TaskIntermitentes, "Intermitentes", 1000, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(TaskLucesDelanteras, "LucesDelanteras", 1000, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(TaskTorreta, "Torreta", 1000, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(TaskHumedad, "EnviarArduino", 1000, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(TaskTanque, "Tanque", 1000, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(TaskComunicacionArduino, "ComunicacionArduino", 2048, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(TaskSeguirLuz, "TaskSeguirLuz", 1000, NULL, 1, NULL, 0);
}

void loop() {
  WiFiClient client = server.available();  // Espera por un cliente

  if (client) {
    Serial.println("Cliente conectado");
    client.println("Conectado con el robot");
    //client.println("EsMotor=" + String(Humedad) + "EsHumedad=" + String(Humedad) + "EsRieg" + String(Humedad));

    while (client.connected()) {
      if (client.available()) {
        String data = client.readStringUntil('\n');
        data.trim();

        if (data.startsWith("Direccion_robot=")) {
          Direccion_robot = data.substring(strlen("Direccion_robot="));
          Serial.print("Robot: ");
          Serial.println(Direccion_robot);
        } else if (data.startsWith("Direccion_torreta=")) {
          Direccion_torreta = data.substring(strlen("Direccion_torreta="));
          Serial.print("Torreta: ");
          Serial.println(Direccion_torreta);
        } else if (data.startsWith("Luces_frontales=")) {
          Luces_frontales = data.substring(strlen("Luces_frontales="));
          Serial.print("Luces Frontales: ");
          Serial.println(Luces_frontales);
        } else if (data.startsWith("Luces_traseras=")) {
          Luces_traseras = data.substring(strlen("Luces_traseras="));
          Serial.print("Luces Traseras: ");
          Serial.println(Luces_traseras);
        } else if (data.startsWith("Luces_intermitentes=")) {
          Luces_intermitentes = data.substring(strlen("Luces_intermitentes="));
          Serial.print("Luces Intermitentes: ");
          Serial.println(Luces_intermitentes);
        } else if (data.startsWith("Estado_humedad=")) {
          Estado_humedad = data.substring(strlen("Estado_humedad="));
          Serial.print("Sensor Humedad: ");
          Serial.println(Estado_humedad);
        }
      }
    }
    // Cierra la conexión
    client.stop();
    Serial.println("Cliente desconectado");
  }
}

// Función para manejar las luces intermitentes
void TaskIntermitentes(void *pvParameters) {
  while (true) {
    if (Luces_intermitentes == "I_right") {
      Serial.println("Las luces intermitentes derechas estan encendidas");
      IntermitentesDireccionales(IntermitentesDerechos);
    }
    if (Luces_intermitentes == "I_left") {
      Serial.println("Las luces intermitentes izquierdas estan encendidas");
      IntermitentesDireccionales(IntermitentesIzquierdos);
    }
    if (Luces_intermitentes == "I_advertencia") {
      Serial.println("Las luces intermitentes de advertencia estan encendidas");
      IntermitentesEmergencia();
    } else {
      //Serial.println("Las luces intermitentes no estan encendidas");
      IntermitentesApagados();
    }
    delay(200);
  }
}
void IntermitentesDireccionales(int intermitente) {
  Serial.println("direccion");
  digitalWrite(intermitente, HIGH);
  delay(500);
  digitalWrite(intermitente, LOW);
  delay(300);
}
void IntermitentesApagados() {
  //Serial.println("apagados");
  digitalWrite(IntermitentesDerechos, LOW);
  digitalWrite(IntermitentesIzquierdos, LOW);
}
void IntermitentesEmergencia() {
  Serial.println("Emergencia");
  digitalWrite(IntermitentesDerechos, HIGH);
  digitalWrite(IntermitentesIzquierdos, HIGH);
  delay(2000);
  digitalWrite(IntermitentesDerechos, LOW);
  digitalWrite(IntermitentesIzquierdos, LOW);
  delay(2000);
}
// Función para manejar las luces delanteras
void TaskLucesDelanteras(void *pvParameters) {
  while (true) {
    if (Luces_frontales == "LF_ON") {
      Serial.println("Las luces frontales están encendidas");
      digitalWrite(LucesFrontales, HIGH);
    } else {
      //Serial.println("Las luces frontales están apagadas");
      digitalWrite(LucesFrontales, LOW);
    }
    delay(200);
  }
}

// Función para manejar la torreta
void TaskTorreta(void *pvParameters) {
  while (true) {
    if (Direccion_torreta == "T_up") {
      Serial.println("La torreta se mueve hacia arriba");
      MoverServo(servoTorretaY, 180, gradosY, true);
    } else if (Direccion_torreta == "T_down") {
      Serial.println("La torreta se mueve hacia abajo");
      MoverServo(servoTorretaY, 0, gradosY, false);
    } else if (Direccion_torreta == "T_left") {
      Serial.println("La torreta se mueve hacia la izquierda");
      MoverServo(servoTorretaX, 0, gradosX, false);
    } else if (Direccion_torreta == "T_right") {
      Serial.println("La torreta se mueve hacia la derecha");
      MoverServo(servoTorretaX, 180, gradosX, true);
    } else if (Direccion_torreta == "T_reset") {
      Serial.println("La torreta se reinicia su posicion");
      Reset();
    } else {
      //Serial.println("La torreta no se mueve");
      DetenerServo(servoTorretaY);
      DetenerServo(servoTorretaX);
    }
    delay(200);
  }
}
// Función genérica para mover un servo
void MoverServo(Servo &servo, int direccion, int &grados, bool sentidoHorario) {
  servo.write(direccion);  // Mover el servo en la dirección especificada

  // Actualizar los grados en función del sentido de rotación
  if (sentidoHorario) {
    grados++;
  } else {
    grados--;
  }

  // Controlar los límites de grados
  ControlarGrados(grados);

  // Mostrar información por consola
  Serial.print("Grados: ");
  Serial.println(grados);

  delay(100);  // Ajusta el tiempo de delay según sea necesario
}

// Controlar si los grados están dentro del límite
void ControlarGrados(int &grados) {
  if (grados > MAX_GRADOS) {
    grados = 0;
    Serial.println("Grados superados, reiniciando a 0.");
  } else if (grados < 0) {
    grados = MAX_GRADOS;
    Serial.println("Grados negativos, ajustando a máximo.");
  }
}

// Restablecer posición de los servos
void Reset() {
  Serial.println("Restableciendo posición de servomotores...");
  Serial.print("Posición actual del eje X: ");
  Serial.println(gradosX);
  Serial.print("Posición actual del eje Y: ");
  Serial.println(gradosY);

  // Restablecer eje Y (servoY)
  while (gradosY > 0) {
    servoTorretaY.write(0);  // Mover hacia abajo (sentido antihorario)
    gradosY--;
    Serial.println(gradosY);
  }
  DetenerServo(servoTorretaY);

  // Restablecer eje X (servoX)
  while (gradosX > 0) {
    servoTorretaX.write(0);  // Mover hacia izquierda (sentido antihorario)
    gradosX--;
    Serial.println(gradosX);
  }
  DetenerServo(servoTorretaX);
}

// Detener el servo en la posición neutral
void DetenerServo(Servo &servo) {
  //Serial.println("Deteniendo servomotor...");
  servo.write(90);  // Detener el servo (posición neutral)
}

//funciona para activar el servo para abjar el sensor de humedad y tambien apra enviar datos para activar el sensor de humedad en arduino
void TaskHumedad(void *pvParameters) {
  while (true) {
    Serial2.println(Estado_humedad);
    delay(1500);
  }
}


// Función para manejar el tanque
void TaskTanque(void *pvParameters) {
  while (true) {
    if (Direccion_robot == "R_Forward" && DistanciaFront != "20") {
      Serial.println("El robot se mueve hacia adelante");
      digitalWrite(LucesTraseras, LOW);
      digitalWrite(MotorDireccionDerecha1, HIGH);
      digitalWrite(MotorDireccionDerecha2, LOW);
      digitalWrite(MotorDireccionIzquierda1, HIGH);
      digitalWrite(MotorDireccionIzquierda2, LOW);
    } else if (Direccion_robot == "R_Back" && DistanciaBack != "20") {
      Serial.println("luces encendidas");
      digitalWrite(LucesTraseras, HIGH);
      Serial.println("El robot se mueve hacia atrás");
      digitalWrite(MotorDireccionDerecha1, LOW);
      digitalWrite(MotorDireccionDerecha2, HIGH);
      digitalWrite(MotorDireccionIzquierda1, LOW);
      digitalWrite(MotorDireccionIzquierda2, HIGH);
    } else if (Direccion_robot == "R_Right") {
      Serial.println("El robot se mueve hacia la derecha");
      digitalWrite(LucesTraseras, LOW);
      digitalWrite(MotorDireccionDerecha1, HIGH);
      digitalWrite(MotorDireccionDerecha2, LOW);
      digitalWrite(MotorDireccionIzquierda1, LOW);
      digitalWrite(MotorDireccionIzquierda2, HIGH);
    } else if (Direccion_robot == "R_Left") {
      Serial.println("El robot se mueve hacia la izquierda");
      digitalWrite(LucesTraseras, LOW);
      digitalWrite(MotorDireccionDerecha1, LOW);
      digitalWrite(MotorDireccionDerecha2, HIGH);
      digitalWrite(MotorDireccionIzquierda1, HIGH);
      digitalWrite(MotorDireccionIzquierda2, LOW);
    } else {
      //Serial.println("El robot no se mueve");
      digitalWrite(LucesTraseras, LOW);
      digitalWrite(MotorDireccionDerecha1, LOW);
      digitalWrite(MotorDireccionDerecha2, LOW);
      digitalWrite(MotorDireccionIzquierda1, LOW);
      digitalWrite(MotorDireccionIzquierda2, LOW);
    }
    delay(200);
  }
}

void TaskComunicacionArduino(void *pvParameters) {
  while (true) {
    if (Serial2.available()) {
      String datos = Serial2.readStringUntil('\n');
      Serial.println(datos);

      // Dividir los datos en función de los identificadores
      LDRX1 = extraerValor(datos, "LDRX1:");
      LDRX2 = extraerValor(datos, "LDRX2:");
      LDRY1 = extraerValor(datos, "LDRY1:");
      LDRY2 = extraerValor(datos, "LDRY2:");
      DistanciaFront = String(extraerValor(datos, "UF:"));
      DistanciaBack = String(extraerValor(datos, "UB:"));
      Humedad = extraerValor(datos, "LH:");

      // Calcular diferencias
      diffX = LDRX1 - LDRX2;
      diffY = LDRY1 - LDRY2;
    }
    delay(500);
  }
}

int extraerValor(String datos, String identificador) {
  int startIndex = datos.indexOf(identificador);
  if (startIndex == -1) {
    return 0;
  }
  startIndex += identificador.length();
  int endIndex = datos.indexOf(' ', startIndex);
  if (endIndex == -1) {
    endIndex = datos.length();
  }
  return datos.substring(startIndex, endIndex).toInt();
}

/*
controlarServoPanel(ServoPanelX, diffX, "X");
controlarServoPanel(ServoPanelY, diffY, "Y");*/
// Función para controlar el movimiento de los servos
void TaskSeguirLuz(void *pvParameters) {
  while (true) {
    controlarServoPanel(ServoPanelX, diffX, "X");
    controlarServoPanel(ServoPanelY, diffY, "Y");
    delay(1000);
  }
}

void controlarServoPanel(Servo &servo, int diferencia, const char *eje) {
  if (diferencia > UMBRAL_SUPERIOR) {
    // Mover en una dirección (derecha o abajo)
    servo.write(POS_DERECHA);
    Serial.print("Moviendo servo ");
    Serial.print(eje);
    Serial.println(" hacia la derecha/abajo");
  } else if (diferencia < -UMBRAL_SUPERIOR) {
    // Mover en la dirección opuesta (izquierda o arriba)
    servo.write(POS_IZQUIERDA);
    Serial.print("Moviendo servo ");
    Serial.print(eje);
    Serial.println(" hacia la izquierda/arriba");
  } else if (abs(diferencia) < UMBRAL_INFERIOR) {
    // Detener el servo
    servo.write(POS_CENTRO);
    //Serial.print("Deteniendo servo ");
    //Serial.print(eje);
    //Serial.println(" en posición central");
  }
}