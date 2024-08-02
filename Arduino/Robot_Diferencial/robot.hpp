#ifndef ROBOT_HPP
#define ROBOT_HPP

#include<Arduino.h>

void interruptionOne();
void interruptionTwo();

class robot
{
    private:
    float wireRatio;
    float lengDistance;

    unsigned long int stepPerRevolution;

    unsigned long int sampleTimeMillis; // fijar tiempo de muestreo en milisegundos
    unsigned long int sampledTimeMilis;
    double sampleTime;


    double leftRPM;
    double rightRPM;
    double leftSetPoint;
    double rightSetPoint;

    double maxMotorRevolutions;

    double controlValueRight;
    double controlValuePassRight;
    double errorRight;
    double errorPass1Right;
    double errorPass2Right;

    double controlValueLeft;
    double controlValuePassLeft;
    double errorLeft;
    double errorPass1left;
    double errorPass2Left;

    double Kp;
    double Ki;
    double Kd;


    void _moveMotorRight(double phi); // Recibe velocidad angular en R.P.M
    void _moveMotorLeft(double phi); // Recibe velocidad angular en R.P.M

    
    public:
        robot(/* args */);
        robot(float r, float l, float maxRev, unsigned long int stepsRev);
        ~robot();
        void setup();
        void setupControlPID(double Ts, double kp, double ki, double kd);
        void sample();
        void move(float vel, float velAng);
};



#endif