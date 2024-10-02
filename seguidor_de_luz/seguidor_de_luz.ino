#include <Servo.h>

// Crear objetos Servo para los dos ejes
Servo servoX; // Eje X (izquierda-derecha)
Servo servoY; // Eje Y (arriba-abajo)

// Pines de las LDR
int LDR1 = A0; // LDR izquierda
int LDR2 = A1; // LDR derecha
int LDR3 = A2; // LDR arriba
int LDR4 = A3; // LDR abajo

// Variables para la posición de los servos
int posX = 90; // Posición inicial del servo X (centro)
int posY = 90; // Posición inicial del servo Y (centro)

int threshold = 15; // Umbral de diferencia para mover los servos

void setup()
{
    // Inicializar los servos
    servoX.attach(9);   // Pin para el servo X
    servoY.attach(10);  // Pin para el servo Y
    servoX.write(posX); // Posición inicial
    servoY.write(posY); // Posición inicial

    // Configurar pines de LDR como entradas
    pinMode(LDR1, INPUT);
    pinMode(LDR2, INPUT);
    pinMode(LDR3, INPUT);
    pinMode(LDR4, INPUT);

    Serial.begin(9600); // Para monitorizar los valores en el Monitor Serie
}

void loop()
{
    // Leer valores de las LDR
    int ldrValue1 = analogRead(LDR1); // LDR izquierda
    int ldrValue2 = analogRead(LDR2); // LDR derecha
    int ldrValue3 = analogRead(LDR3); // LDR arriba
    int ldrValue4 = analogRead(LDR4); // LDR abajo

    // Comparar valores para el eje X (izquierda-derecha)
    int diffX = ldrValue1 - ldrValue2; // Diferencia entre LDR1 y LDR2
    if (abs(diffX) > threshold)
    {
        if (ldrValue1 > ldrValue2)
        {
            posX--; // Mover hacia la derecha
        }
        else
        {
            posX++; // Mover hacia la izquierda
        }
        posX = constrain(posX, 0, 180); // Limitar el rango de movimiento
        servoX.write(posX);
    }

    // Comparar valores para el eje Y (arriba-abajo)
    int diffY = ldrValue3 - ldrValue4; // Diferencia entre LDR3 y LDR4
    if (abs(diffY) > threshold)
    {
        if (ldrValue3 > ldrValue4)
        {
            posY--; // Mover hacia abajo
        }
        else
        {
            posY++; // Mover hacia arriba
        }
        posY = constrain(posY, 0, 180); // Limitar el rango de movimiento
        servoY.write(posY);
    }

    // Monitorizar los valores en el Monitor Serie (opcional)
    Serial.print("LDR1: ");
    Serial.print(ldrValue1);
    Serial.print(" | LDR2: ");
    Serial.print(ldrValue2);
    Serial.print(" | LDR3: ");
    Serial.print(ldrValue3);
    Serial.print(" | LDR4: ");
    Serial.println(ldrValue4);

    delay(15); // Pequeña pausa para permitir que los servos se muevan
}
