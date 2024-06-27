import sim
import time
import math as mt
import numpy as np

class PioneerP3DX:
    def __init__(self, controlKp=0.1, controlKo=0.1, controlKi = 0.1, controlKoi = 0.1, controlKd = 1, controlKod = 1):
        self.R = 0.1
        self.L = 0.335
        self.Kp = controlKp
        self.Ki = controlKi
        self.Kd = controlKd
        self.Ko = controlKo
        self.Koi = controlKoi
        self.Kod = controlKod
        self.returnCode = []
        self.robotHandle = []
        self.robotLeftMotor = []
        self.robotRightMotor = []
        self.clientID = []
        self.ultrasonicSensors = []
        self.ultrasonicSensorsValue = np.zeros((16),dtype=bool)
        self.ultrasonicSensorsMeasure = np.zeros((16))
        self.robotPosition = []
        self.robotOrientation = []
    
    def connect(self,port):
        sim.simxFinish(-1) # just in case, close all opened connections
        self.clientID=sim.simxStart('127.0.0.1',port,True,True,2000,5) # Conectarse
        if self.clientID == 0: print("conectado al puerto", port)
        else: print("No se pudo conectar verifique simulador")
        return self.clientID
    def getRobotHandle(self):
        self.returnCode, self.robotHandle = sim.simxGetObjectHandle(self.clientID,'Pioneer_p3dx',sim.simx_opmode_blocking)
        print(self.robotHandle)
    def getMotorsHandle(self):
        # Obteniendo handle del motor izquierdo
        self.returnCode, self.robotLeftMotor = sim.simxGetObjectHandle(self.clientID,'Pioneer_p3dx_leftMotor',sim.simx_opmode_blocking) 
        # Obteniedo Handle de Motor Derecho           
        self.returnCode, self.robotRightMotor = sim.simxGetObjectHandle(self.clientID,'Pioneer_p3dx_rightMotor',sim.simx_opmode_blocking)
        print(self.robotLeftMotor)
        print(self.robotRightMotor)
    def getUltrasonicSensorshandle(self):
        for i0 in range(1,17):
            self.returnCode,sensorI = sim.simxGetObjectHandle(self.clientID, "Pioneer_p3dx_ultrasonicSensor"+str(i0), sim.simx_opmode_blocking)
            self.ultrasonicSensors.append(sensorI) 
        print(self.ultrasonicSensors)
    def getMesaurament(self):
        for i0 in range(16):
            returnCode, stateCode, point, detectedObject,detectSurNormVect = sim.simxReadProximitySensor(self.clientID, self.ultrasonicSensors[i0], sim.simx_opmode_streaming)
            self.ultrasonicSensorsValue[i0] = stateCode
            self.ultrasonicSensorsMeasure[i0] = np.linalg.norm(point)
    def initUltrasonicSensors(self):
        for i0 in range(16):
            returnCode, stateCode, point, detectedObject,detectSurNormVect = sim.simxReadProximitySensor(self.clientID, self.ultrasonicSensors[i0], sim.simx_opmode_streaming)
    
    def robotInit(self):
        self.getRobotHandle()
        self.getMotorsHandle()
        self.getUltrasonicSensorshandle()

    def move(self,velocity, angularVelocity):
        velocityL = velocity/self.R - angularVelocity*self.L/(2*self.R)
        velocityR = velocity/self.R + angularVelocity*self.L/(2*self.R)
        self.returnCode = sim.simxSetJointTargetVelocity(self.clientID, self.robotRightMotor, velocityR, sim.simx_opmode_streaming)
        self.returnCode = sim.simxSetJointTargetVelocity(self.clientID, self.robotLeftMotor, velocityL, sim.simx_opmode_streaming)
    def Stop(self):
        self.returnCode = sim.simxSetJointTargetVelocity(self.clientID, self.robotRightMotor, 0, sim.simx_opmode_streaming)
        self.returnCode = sim.simxSetJointTargetVelocity(self.clientID, self.robotLeftMotor, 0, sim.simx_opmode_streaming)
    def getRobotPose(self):
        self.returnCode, self.robotPosition = sim.simxGetObjectPosition(self.clientID, self.robotHandle, -1, sim.simx_opmode_blocking)
        self.returnCode, self.robotOrientation = sim.simxGetObjectOrientation(self.clientID, self.robotHandle, -1, sim.simx_opmode_blocking)
    def getDistanceError(self,point):
        return mt.sqrt((point[0] - self.robotPosition[0])**2 + (point[1] - self.robotPosition[1])**2)
    def angleDifferential(self,phi):
        theta = mt.acos(mt.cos(self.robotOrientation[2])*mt.cos(phi)+mt.sin(self.robotOrientation[2])*mt.sin(phi))
        direction = mt.cos(self.robotOrientation[2])*mt.sin(phi) - mt.cos(phi)*mt.sin(self.robotOrientation[2])
        return mt.copysign(theta, direction)
    def goToAPoint(self,point):
        self.getRobotPose()
        print("Punto Objetivo: "+str(point))
        lastDistanceError = 0
        presentDistanceError = self.getDistanceError(point)
        print(presentDistanceError)
        lastAngleError = 0
        angle = mt.atan2(point[1] - self.robotPosition[1],point[0] - self.robotPosition[0])
        presentAngleError = self.angleDifferential(angle)
        lastTime = 0
        presentTime = sim.simxGetLastCmdTime(self.clientID)
        intErrorDistance = 0
        intErrorAngle = 0 
        while presentDistanceError > 0.05:
            presentTime = sim.simxGetLastCmdTime(self.clientID)
            dTime = presentTime - lastTime
            #print(dTime)
            self.getRobotPose()
            presentDistanceError = self.getDistanceError(point)
            angle = mt.atan2(point[1] - self.robotPosition[1],point[0] - self.robotPosition[0])
            presentAngleError = self.angleDifferential(angle)
            intErrorDistance += presentDistanceError*dTime
            intErrorAngle += presentAngleError*dTime
            # velocity = self.Kp*presentDistanceError + self.Kd*(presentDistanceError - lastDistanceError)/dTime + self.Ki*intErrorDistance
            # angularVelocity = self.Ko*presentAngleError + self.Kod*(presentAngleError - lastAngleError)/dTime + self.Koi*intErrorAngle
            velocity = self.Kp*presentDistanceError
            angularVelocity = self.Ko*presentAngleError
            self.move(velocity,angularVelocity)
            lastDistanceError = presentDistanceError
            lastAngleError = presentAngleError
            #print("Distancia al Objetivo: " + str(lastDistanceError) + " Direccion al objetivo: "+str(lastAngleError)+" Diferencia de Tiempo: " + str(dTime))
            #print("Velocidad: " +str(velocity)+ " Velocidad Angular" + str(angularVelocity) )
            lastTime = presentTime
            
        self.Stop()

robot = PioneerP3DX(controlKp=0.1,controlKo=0.1,controlKi = 0.00001, controlKoi = 0.1, controlKd = 0.1, controlKod = 0.0001)
robot.connect(19999)
print(robot.clientID)
robot.robotInit()
while True:
    robot.getMesaurament()
    print(robot.ultrasonicSensorsMeasure)
    time.sleep(1)
# robot.goToAPoint([10,10,0])
# robot.goToAPoint([10,-10,0])
# robot.goToAPoint([-10,-10,0])
# robot.goToAPoint([-10,10,0])
    

