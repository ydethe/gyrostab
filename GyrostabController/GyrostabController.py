import requests
import time
import threading
import random
import math

# Use for Periodic Task
next_call = time.time()
bool = True

attitude = [0.0,0.0,0.0]

def periodicTask():
    global next_call
    next_call = next_call+0.5
    updateState()
    updateCommand()
    threading.Timer( next_call - time.time(), periodicTask ).start()

def updateState():
    global attitude
    time, attitude = getAttitude()
    
def updateCommand():
    global bool, attitude
    servo=[attitude[1],0,0]
    if bool == True:
        servo = [-45, random.randint(-90, 90), random.randint(-90, 90)]
        bool = False
    else:
        servo = [45, random.randint(-90, 90), random.randint(-90, 90)]
        bool = True
    # print("Command send :" + str(servo))
    putServo(servo)
    
def getIMU():
    response = requests.get('http://127.0.0.1:5000/imu')
    assert response.status_code == 200 # A quoi ça sert ??
    return response.json()["time"], response.json()["acceleration"], response.json()["angularSpeed"], response.json()["magneticField"]

def getAttitude():
    response = requests.get('http://127.0.0.1:5000/attitude')
    assert response.status_code == 200 # A quoi ça sert ??
    return response.json()["time"], response.json()["attitude"]

def getPenduleSimulationState():
    response = requests.get('http://127.0.0.1:5000/simulation/pendulum/state')
    assert response.status_code == 200 # A quoi ça sert ??
    return response.json()["time"], response.json()["state"]

def putPenduleSimulationAngle(angle):
    response = requests.put('http://127.0.0.1:5000/simulation/pendulum/state', json = {'angle': angle})
    assert response.status_code == 200

def putServo(servo):
    response = requests.put('http://127.0.0.1:5000/servo', json = {'x': servo[0], 'y': servo[1], 'z': servo[2]})
    assert response.status_code == 200

if __name__ == '__main__':
    controllerTask = threading.Thread(None, periodicTask)
    controllerTask.start()
    
    
    