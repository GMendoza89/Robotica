#include"robot.hpp"

robot myRobot(0.04,0.15,240.0,20);

float Vel = 0.1;
float AngVel = 0.0;
void setup(){
    myRobot.setup();
    myRobot.setupControlPID(0.01,1,1,0.1);
    
}
void loop(){

    myRobot.sample();
    myRobot.move(Vel,AngVel);

}