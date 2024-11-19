/*
* Name: motor_PID
* Autor: Ing. Gustavo David Mendoza Pinto
* Description: Controlador de motor PID 
* Version : 0.24.07
*/

#include<stdint.h>

uint8_t motor1VelPin = 5;
uint8_t motorVel = 0;

unsigned long int sampleTimeMillis = 100; // fijar tiempo de muestreo en milisegundos
unsigned long int sampledTimeMilis = 0;
double sampleTime = double(sampleTimeMillis)/1000;

double RPM = 0;
double setPoint = 0;

double maxMotorRevolutions = 250.0; //Cambiar por las caracteristicas de su motor

double controlValue = 0;
double controlValuePass = 0;
double error = 0;
double errorPass1 = 0;
double errorPass2 = 0;

double Kp = 1.0;
double Ki = 1.0;
double Kd = 0.001;


unsigned long int steps = 0;
unsigned long int stepPerRevolution = 20; // Cambiar por las caracteristicas de su encoder 


//Funciones globales
void interruptionOne();

void setup(){
    //connfiguracion de los pines
    DDRB |= 0x0F;

    DDRD |= 0x60;
    DDRD &= 0xF3;

    attachInterrupt(digitalPinToInterrupt(2),interruptionOne,FALLING);

    Serial.begin(115200);
    PORTB |= 0x01;
}
void loop(){
    
    if(millis() - sampledTimeMilis >= sampleTimeMillis){
        noInterrupts();
        RPM = (double(steps/stepPerRevolution))/(sampleTime) * 60.0;  // Revoluciones por minuto
        steps = 0;
        interrupts();
        sampledTimeMilis = millis();
        
    }
    setPoint = ((double(analogRead(A0)))/1023.00)*maxMotorRevolutions;

    error = setPoint - RPM;

    controlValue = controlValuePass + (Kp + Kd/sampleTime)*error + (Ki*sampleTime - Kp - 2*Kd/sampleTime)*errorPass1 + (Kd/sampleTime)*errorPass2;

    controlValuePass = controlValue;
    errorPass2 = errorPass1;
    errorPass1 = error;

    if(controlValue >= 255){
        motorVel = 255;
    }else{
        motorVel = uint8_t(controlValue);
    }
    analogWrite(motor1VelPin,motorVel);

    Serial.print("Set Point: ");
    Serial.print(setPoint);
    Serial.print(" R.P.M.: ");
    Serial.print(RPM);
    Serial.print(" CV: ");
    Serial.print(controlValue);
    Serial.print(" PWM:");
    Serial.println(motorVel);

}

void interruptionOne(){
    ++steps;
}