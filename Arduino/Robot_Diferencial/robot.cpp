#include"robot.hpp"


//--------------------- Funciones y Variables Globales -----------------------

unsigned long int stepsRight = 0;
unsigned long int stepsLeft = 0;


void interruptionRight(){
    ++stepsRight;
}
void interruptionLeft(){
    ++stepsLeft;
}

//------------------ Clase Robot

robot::robot(/* args */){
    wireRatio = 0.08;
    lengDistance = 0.16;

}

robot::robot(float r, float l, float maxRev, unsigned long int stepsRev){
    wireRatio = r;
    lengDistance = l;
    maxMotorRevolutions =  maxRev;
    stepPerRevolution = stepsRev;
}


robot::~robot(){

}

void robot::setup(){
    DDRB |= 0x0F;
    DDRD = 0x72;

    attachInterrupt(digitalPinToInterrupt(2),interruptionRight,FALLING);
    attachInterrupt(digitalPinToInterrupt(3),interruptionLeft,FALLING);
    Serial.begin(115200);
}

void robot::move(float vel, float velAng){
    float velRight = vel/wireRatio + (velAng*lengDistance)/(2*wireRatio);
    float velLeft = vel/wireRatio - (velAng*lengDistance)/(2*wireRatio);
    Serial.print(" VelR: ");
    Serial.print( velRight);
    Serial.print(" VelL: ");
    Serial.println(velLeft);
    
    // Control de direciones 
    if(velRight >= 0 && velLeft >= 0){
        PORTB = 0x05;
    }else if(velRight < 0 && velLeft >= 0){
        PORTB = 0x06;
    }else if(velRight < 0 && velLeft < 0){
        PORTB = 0x0A;
    }else{
        PORTB = 0x09;
    }
    // Control de velocidad
    
    _moveMotorRight(velRight);
    _moveMotorLeft(velLeft);

}
void robot::setupControlPID(double Ts, double kp, double ki, double kd){
    
    sampleTime = Ts;                                    // tiempo de muestreo enn segundos
    sampleTimeMillis = int(sampleTime*1000); // tiempo de muestreo en milisegundos
    sampledTimeMilis = 0;                               // Inicialización de tiempo muestreado en segundos

    Kp = kp;
    Ki = ki;
    Kd = kd;                        
    

    

}
void robot::sample(){
    if(millis() - sampledTimeMilis >= sampleTimeMillis){
        
        rightRPM = (double(stepsRight/stepPerRevolution))/(sampleTime) * 60.0;  // Revoluciones por minuto
        leftRPM = (double(stepsLeft/stepPerRevolution))/(sampleTime) * 60.0;  // Revoluciones por minuto
        stepsRight = 0;
        stepsLeft = 0;
        sampledTimeMilis = millis();
    }
}

void robot::_moveMotorRight( double Phi){
    errorRight = Phi - rightRPM;

    controlValueRight = controlValuePassRight + (Kp + Kd/sampleTime)*errorRight + (Ki*sampleTime - Kp - 2*Kd/sampleTime)*errorPass1Right + (Kd/sampleTime)*errorPass2Right;

    controlValuePassRight = controlValueRight;
    errorPass2Right = errorPass1Right;
    errorPass1Right = errorRight;

    if(controlValueRight > 255.0){
        analogWrite(5,255);
    }else{
        analogWrite(5,uint8_t(controlValueRight));
    }
}

void robot::_moveMotorLeft( double Phi){
    errorLeft = Phi - leftRPM;

    controlValueLeft = controlValuePassLeft + (Kp + Kd/sampleTime)*errorLeft + (Ki*sampleTime - Kp - 2*Kd/sampleTime)*errorPass1left + (Kd/sampleTime)*errorPass2Left;

    controlValuePassLeft  = controlValueLeft;
    errorPass2Left = errorPass1left;
    errorPass1left = errorLeft;

    if(controlValueLeft > 255.0){
        analogWrite(6,255);
    }else{
        analogWrite(6,uint8_t(controlValueLeft));
    }
}