#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Nov 22 18:55:09 2021

@author: gustavomendoza
"""

import sim
import numpy as np
import math as mt

#Variables Globales

#Variables del Agewnte
tireRadius = 0.1
axesLength = 0.335
# Constantes del control
kPosition = 0.1
kOrientation = 0.5
# Punto objetivo
setPoint = [10,10,0]

ultrasonicSensors = []
ultrasonicSensorsValue = np.zeros((16),dtype=bool)
ultrasonicSensorsMeasure = np.zeros((16))


#funciones
def connect(port):
#establece la coneccion con coppelia sim
#port debe coincidir con el puerto de conexion de copelia sim
    sim.simxFinish(-1) # just in case, close all opened connections
    clientID=sim.simxStart('127.0.0.1',port,True,True,2000,5) # Conectarse
    if clientID == 0: print("conectado al puerto", port)
    else: print("No se pudo conectar verifique simulador")
    return clientID

def velocityL(velocity, angularVelocity):
    return velocity/tireRadius - angularVelocity*axesLength/(2*tireRadius)

def velocityR(velocity, angularVelocity):
    return velocity/tireRadius + angularVelocity*axesLength/(2*tireRadius)

def angleDifferential(phiRobot,phi):
    theta = mt.acos(mt.cos(phiRobot)*mt.cos(phi)+mt.sin(phiRobot)*mt.sin(phi))
    direction = mt.cos(phiRobot)*mt.sin(phi) - mt.cos(phi)*mt.sin(phiRobot)
    return mt.copysign(theta, direction)

def getUltrasonicSensorshandle(clientID):
    for i0 in range(1,17):
        returnCode,sensorI = sim.simxGetObjectHandle(clientID, "Pioneer_p3dx_ultrasonicSensor"+str(i0), sim.simx_opmode_blocking)
        ultrasonicSensors.append(sensorI)

def initUltrasonicSensors():
    for i0 in range(16):
        returnCode, stateCode, point, detectedObject,detectSurNormVect = sim.simxReadProximitySensor(clientID, ultrasonicSensors[i0], sim.simx_opmode_streaming)
        
def getMesaurament():
    for i0 in range(16):
        returnCode, stateCode, point, detectedObject,detectSurNormVect = sim.simxReadProximitySensor(clientID, ultrasonicSensors[i0], sim.simx_opmode_streaming)
        ultrasonicSensorsValue[i0] = stateCode
        ultrasonicSensorsMeasure[i0] = np.linalg.norm(point)
def evita():
    returnCode = sim.simxSetJointTargetVelocity(clientID, robotRightMotor, -0.3, sim.simx_opmode_streaming)
    returnCode = sim.simxSetJointTargetVelocity(clientID, robotLeftMotor, 0.3, sim.simx_opmode_streaming)
    
def follow():
    returnCode = sim.simxSetJointTargetVelocity(clientID, robotRightMotor, 0.3, sim.simx_opmode_streaming)
    returnCode = sim.simxSetJointTargetVelocity(clientID, robotLeftMotor, 0.3, sim.simx_opmode_streaming)
        
        

#Iniclialización del prograsma principal
clientID = connect(19999) #conexion con el simulador
#obtención del Handle del robot
returnCode,handle=sim.simxGetObjectHandle(clientID,'Pioneer_p3dx',sim.simx_opmode_blocking)
robot = handle
print(robot)
#Obtener handle de los motores
#handle del motor izquierdo
returnCode,robotLeftMotor=sim.simxGetObjectHandle(clientID,'Pioneer_p3dx_leftMotor',sim.simx_opmode_blocking)
print(robotLeftMotor)
#handle del motor Derecho
returnCode,robotRightMotor=sim.simxGetObjectHandle(clientID,'Pioneer_p3dx_rightMotor',sim.simx_opmode_blocking)
print(robotRightMotor)

# Obtencion dde la posicion inicial del agente
returnCode,robotPosition = sim.simxGetObjectPosition(clientID, robot, -1, sim.simx_opmode_blocking)
print(robotPosition)
#obtencion de la posición de la orientación del agente
returnCode,robotOrientation = sim.simxGetObjectOrientation(clientID, robot, -1, sim.simx_opmode_blocking)
#print(robotOrientation)
getUltrasonicSensorshandle(clientID)
initUltrasonicSensors()
getMesaurament()

print(ultrasonicSensorsValue)
print(ultrasonicSensorsMeasure)


#calculamos el error inicial
distanceError = mt.sqrt((setPoint[0] - robotPosition[0])**2 + (setPoint[1] - robotPosition[1])**2 )

angle = mt.atan2(setPoint[1] - robotPosition[1], setPoint[0] - robotPosition[0])
angleError = angleDifferential(robotOrientation[2], angle)


while distanceError > 0.05:
    
    if ultrasonicSensorsValue[2] or ultrasonicSensorsValue[3] or ultrasonicSensorsValue[4] or ultrasonicSensorsValue[5] :
        evita()
    elif ultrasonicSensorsValue[0] or ultrasonicSensorsValue[15]:
        follow()
    else:
        velocity = kPosition*distanceError
        angularVelocity = kOrientation*angleError
        returnCode = sim.simxSetJointTargetVelocity(clientID, robotRightMotor, velocityR(velocity,angularVelocity), sim.simx_opmode_streaming)
        returnCode = sim.simxSetJointTargetVelocity(clientID, robotLeftMotor, velocityL(velocity,angularVelocity), sim.simx_opmode_streaming)
        
    #distanceError = mt.sqrt((setPoint[0] - robotPosition[0])**2 + (setPoint[1] - robotPosition[1])**2 )
    
    returnCode,robotPosition = sim.simxGetObjectPosition(clientID, robot, -1, sim.simx_opmode_blocking)
    angle = mt.atan2(setPoint[1] - robotPosition[1], setPoint[0] - robotPosition[0])
    angleError = angleDifferential(robotOrientation[2], angle)
    distanceError = mt.sqrt((setPoint[0] - robotPosition[0])**2 + (setPoint[1] - robotPosition[1])**2 )
    
    #print(robotPosition)
    #obtencion de la posición de la orientación del agente
    returnCode,robotOrientation = sim.simxGetObjectOrientation(clientID, robot, -1, sim.simx_opmode_blocking)
    #print(robotOrientation)
    getMesaurament()
    print(ultrasonicSensorsMeasure[3])
    
print(ultrasonicSensorsValue)
print(ultrasonicSensorsMeasure)
returnCode = sim.simxSetJointTargetVelocity(clientID, robotRightMotor, 0, sim.simx_opmode_streaming)
returnCode = sim.simxSetJointTargetVelocity(clientID, robotLeftMotor, 0, sim.simx_opmode_streaming)
    
