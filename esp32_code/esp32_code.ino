#include <WiFi.h>

// Definición de variables
const char *ssid = "ConvertSystems";
//const char *password = "style1234";

WiFiServer server(80);

//variables para los componentes fisicos del robot
int ElectroBomba = 26;

int ServoTorretaEjeY = 18;
int ServoTorretaEjeX = 5;

int ServoPanelEjeY = 15;
int ServoPanelEjeX = 2;

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

void setup() {
  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1, 16, 17);

  pinMode(ElectroBomba, OUTPUT);

  pinMode(ServoTorretaEjeY, OUTPUT);
  pinMode(ServoTorretaEjeX, OUTPUT);

  pinMode(ServoPanelEjeY, OUTPUT);
  pinMode(ServoPanelEjeX, OUTPUT);

  pinMode(LucesFrontales, OUTPUT);
  pinMode(LucesTraseras, OUTPUT);

  pinMode(IntermitentesDerechos, OUTPUT);
  pinMode(IntermitentesIzquierdos, OUTPUT);

  pinMode(MotorDireccionDerecha1, OUTPUT);
  pinMode(MotorDireccionDerecha2, OUTPUT);

  pinMode(MotorDireccionIzquierda1, OUTPUT);
  pinMode(MotorDireccionIzquierda2, OUTPUT);

  pinMode(ServoHumedad, OUTPUT);

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
  Serial.println(WiFi.localIP());

  // Crear tareas para cada función de control

  xTaskCreatePinnedToCore(TaskIntermitentes, "Intermitentes", 1000, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(TaskLucesDelanteras, "LucesDelanteras", 1000, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(TaskTorreta, "Torreta", 1000, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(TaskHumedad, "EnviarArduino", 1000, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(TaskTanque, "Tanque", 1000, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(TaskComunicacionArduino, "ComunicacionArduino", 2048, NULL, 1, NULL, 0);
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
  delay(500);
  digitalWrite(IntermitentesDerechos, LOW);
  digitalWrite(IntermitentesIzquierdos, LOW);
  delay(300);
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
    } else if (Direccion_torreta == "T_down") {
      Serial.println("La torreta se mueve hacia abajo");
    } else if (Direccion_torreta == "T_left") {
      Serial.println("La torreta se mueve hacia la izquierda");
    } else if (Direccion_torreta == "T_right") {
      Serial.println("La torreta se mueve hacia la derecha");
    } else {
      //Serial.println("La torreta no se mueve");
    }
    delay(200);
  }
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