/*
*       Programa de control de vehiculo  por Serial 
*       Autor: Ing. Gustavo David Mendoza Pinto
*       Descripsion: Control de cuatro motores a travez de dos puentes H 298N 
*       y con interfaz serial. El diagrama de conexion corresponde al mostrado en Auto_Serial.pdf
*
*       Distribucion de los motores 
*          Frente
            ____
*         _|    |_
         |_M1  M3_|
          _|    |_
         |_M2  M4_|
           |____|
           
           Reverso


*/

// Variables para el control de velocidad de los motores
uint8_t motor_1_EN = 3;
uint8_t motor_2_EN = 5;
uint8_t motor_3_EN = 10;
uint8_t motor_4_EN = 11;

//Caracteristicas del Robot
double l = 0.16;  // longitud entre los ejes
double r = 0.06; // Radio de las llantas

//Variables para el control de velocidad
double V = 0; //velocidad del robot
double W = 0; // velocidad angular
double Vr = 0;
double Vl = 0;

//Variables para el control por serial
String message, stringW, stringV;
int pos1;
void setup() {
  
  // Configuraacion de puertos
  DDRD |= 0xFC;
  DDRB |=0x3F;
  Serial.begin(115200);

}

void loop() {
    //verificamos si hay una nueva entrada de velocidad y velocidad angular
    if(Serial.available()){
        message = Serial.readString();
        pos1 = message.indexOf(',');
        stringV = message.substring(0,pos1);
        stringW = message.substring(pos1+1);
        V = stringV.toFloat();
        W = stringV.toFloat();
        Vr = V/r + (W*l)/(2*r);
        Vl = V/r - (W*l)/(2*r);
    }

    // Verificamos la direccion de giro de los motores
    if(Vr < 0 && Vl < 0){
        // Combinacion del puerto D para encender motores hacia el Atras
        PORTD |= 0X90;
        PORTD &= 0xBB;
        // Combinacion del puerto B para encender motores hacia el Atras
        PORTB |= 0x11;
        PORTB &= 0xDD;
        // descomentar durante pruebas
        Serial.println(" Movimiento de reversa");
    }else if(Vr > 0 && Vl < 0){
        // Combinacion del puerto D para encender motores hacia el frente
        PORTD |= 0X44;
        PORTD &= 0x6F;
        // Combinacion del puerto B para encender motores hacia el Atras
        PORTB |= 0x11;
        PORTB &= 0xDD;
        // descomentar durante pruebas
        Serial.println(" Movimiento Rotacion a la izquierda");

    }else if(Vr < 0 && Vl > 0){

        // Combinacion del puerto D para encender motores hacia el Atras
        PORTD |= 0X90;
        PORTD &= 0xBB;
        // Combinacion del puerto B para encender motores hacia el frente
        PORTB |= 0x22;
        PORTB &= 0xEE;
        // descomentar durante pruebas
        Serial.println(" Movimiento Rotacion a la derecha");
        
    }else{
        // Combinacion del puerto D para encender motores hacia el frente
        PORTD |= 0X44;
        PORTD &= 0x6F;
        // Combinacion del puerto B para encender motores hacia el frente
        PORTB |= 0x22;
        PORTB &= 0xEE;
        // descomentar durante pruebas
        Serial.println(" Movimiento avanzar");
    }
    if(Vr > 255 || Vr < -255){
        Vr = 255;
    }
    if(Vr < 0){
        Vr *= (-1);
    }
    if(Vl > 255 || Vl < -255){
        Vl = 255;
    }
    if(Vl < 0){
        Vl *= (-1);
    }


    analogWrite(motor_1_EN,uint8_t(Vl));
    analogWrite(motor_4_EN,uint8_t(Vr));
    analogWrite(motor_3_EN,uint8_t(Vr));
    analogWrite(motor_2_EN,uint8_t(Vl));
    // descomentar durante pruebas
    Serial.print("PWM M1: ");
    Serial.print(Vr);
    Serial.print("PWM M2: ");
    Serial.print(Vr);
    Serial.print("PWM M3: ");
    Serial.print(Vl);
    Serial.print("PWM M4: ");
    Serial.print(Vl);
    
}
