/*
* Name: Control Gestual de brazo Robotico de 3 grados de libertad mas actuador final
* Autor: Ing. Gustavo David Mendoza Pinto
* Version: 1.24.06
* Descripcion: Control de brazo robotico con Arduino UNO R3 o Arduinon nano
*/

//Librerias necesarias
#include <EEPROM.h>
#include <Servo.h>

//declaracio de constantes


//Variables a utilizar

// Declaracion de variables de pines a utilizar
uint8_t pinServoEE = 11;
uint8_t pinServoJ2 = 10;
uint8_t pinServoJ1 = 9;
uint8_t pinServoBase = 8;

uint8_t pinButtomRun = 7;
uint8_t pinButtomStop = 6;
uint8_t pinButtomWrite = 5;
uint8_t pinButtomSave = 4;

uint8_t pinLedRed = 3;
uint8_t pinLedGreen = 2;

uint8_t pinPotBase = A0;
uint8_t pinPotJ1 = A1;
uint8_t pinPotJ2 = A2;
uint8_t pinPotEE = A3;

//Variables a utilizar
uint8_t angleServoBase = 0;
uint8_t angleServoJ1 = 0;
uint8_t angleServoJ2 = 0;
uint8_t angleServoEE = 0;

uint8_t state = 0;            // Variable donde guardaremos estado del control del robot
unsigned int totalMoves = 0;  // Variable donde guardamos el total de movimientos del robot
unsigned int index = 0;       // Indice de memoria del Robot

Servo servoBase;
Servo servoJ1;
Servo servoJ2;
Servo servoEE;



void setup() {
  // put your setup code here, to run once:
  //Configurando el puerto D
  // DDRD &= 0xF0;
  // DDRD |= 0x0C;
  DDRD = 0x0E; //0b00001110
  //Configurando Servomotores
  servoBase.attach(pinServoBase,1000,2000);
  servoJ1.attach(pinServoJ1,1000,2000);
  servoJ2.attach(pinServoJ2,1000,2000);
  servoEE.attach(pinServoEE,1000,2000);

  Serial.begin(9600);
  digitalWrite(pinLedRed,HIGH);
  digitalWrite(pinLedGreen,LOW);

  EEPROM.get(1022, totalMoves);

  Serial.print("Robot listo datos almacenados en la memoria: ");
  Serial.print(totalMoves);


}

void loop() {
  // put your main code here, to run repeatedly:
  
  //Lectura de estado
  if(!digitalRead(pinButtomRun)){ //
    state = 1;
    digitalWrite(pinLedRed,LOW);
    digitalWrite(pinLedGreen,HIGH);
  }else if(!digitalRead(pinButtomStop)){
    state = 0;
    digitalWrite(pinLedRed,HIGH);
    digitalWrite(pinLedGreen,LOW);
  }else if(!digitalRead(pinButtomWrite)){
    state = 2;
    digitalWrite(pinLedRed,HIGH);
    digitalWrite(pinLedGreen,LOW);
  }else if(!digitalRead(pinButtomWrite) && !digitalRead(pinButtomSave)){
    state = 3;
    digitalWrite(pinLedRed,HIGH);
    digitalWrite(pinLedGreen,HIGH);
  }
  switch(state){
    case 0: // Funsion de Paro de Emergencia
      servoBase.write(angleServoBase);
      servoJ1.write(angleServoJ1);
      servoJ2.write(angleServoJ2);
      servoEE.write(angleServoEE);
      delay(10);
      break;
    case 1:
      angleServoBase = EEPROM.read(index);
      angleServoJ1 =  EEPROM.read(index + 1);
      angleServoJ2 =  EEPROM.read(index + 2);
      angleServoEE = EEPROM.read(index + 3) ;
      servoBase.write(angleServoBase);
      servoJ1.write(angleServoJ1);
      servoJ2.write(angleServoJ2);
      servoEE.write(angleServoEE);
      delay(10);
      index += 4;
      if(index >= totalMoves){
        index = 0;
      }
      break;

      break;

    case 2:
      angleServoBase = map(analogRead(pinPotBase),0,1024,0,180);
      angleServoJ1 = map(analogRead(pinPotJ1),0,1024,0,180);
      angleServoJ2 = map(analogRead(pinPotJ2),0,1024,0,180);
      angleServoEE = map(analogRead(pinPotEE),0,1024,0,180);
      servoBase.write(angleServoBase);
      servoJ1.write(angleServoJ1);
      servoJ2.write(angleServoJ2);
      servoEE.write(angleServoEE);
      delay(10);
      break;
  }


}
