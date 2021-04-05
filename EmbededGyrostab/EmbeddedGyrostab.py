
import random
import threading
import json
import math
from flask import Flask, jsonify, request, render_template

from ServoController import ServoController
from AHRS import AHRS
from Controller import Controller
from PendulumSimulator import PendulumSimulator

# Use for web server
app = Flask(__name__)

# Use for Attitude computation
AHRS = AHRS.AHRS(period=0.6)
servos = ServoController.ServoController(11, 13, 14)
pendule = PendulumSimulator.PendulumSimulator(sysAngle=4*math.pi/5)

@app.route('/display', methods=['GET'])
def get_displayRoutes():
    routes = []
    for rule in app.url_map.iter_rules():
        if "/display/" in str(rule) and "/caption" not in str(rule):
            route = {}
            route['route'] = str(rule)
            route['title'] = str(rule).split('/')[-1].capitalize()
            routes.append(route)
    response = jsonify(routes)
    response.headers.add('Access-Control-Allow-Origin', '*')
    return response

##########    AHRS services    ##########
@app.route('/imu', methods=['GET'])
def get_imu():
    global AHRS
    response = jsonify(AHRS.getIMUData())
    response.headers.add('Access-Control-Allow-Origin', '*')
    return response

@app.route('/attitude', methods=['GET'])
def get_attitude():
    global AHRS
    response = jsonify(AHRS.getAttitudeData())
    response.headers.add('Access-Control-Allow-Origin', '*')
    return response

@app.route('/display/acceleration', methods=['GET'])
def get_acceleration():
    global AHRS
    response = jsonify(AHRS.getDisplayableAcceleration())
    response.headers.add('Access-Control-Allow-Origin', '*')
    return response

@app.route('/display/acceleration/caption', methods=['GET'])
def get_accelerationCaption():
    global AHRS
    response = jsonify(AHRS.getDisplayableAccelerationCaption())
    response.headers.add('Access-Control-Allow-Origin', '*')
    return response

@app.route('/display/angularspeed', methods=['GET'])
def get_angularspeed():
    global AHRS
    response = jsonify(AHRS.getDisplayableAngularSpeed())
    response.headers.add('Access-Control-Allow-Origin', '*')
    return response

@app.route('/display/angularspeed/caption', methods=['GET'])
def get_angularspeedCaption():
    global AHRS
    response = jsonify(AHRS.getDisplayableAngularSpeedCaption())
    response.headers.add('Access-Control-Allow-Origin', '*')
    return response

@app.route('/display/magneticfield', methods=['GET'])
def get_magneticfield():
    global AHRS
    response = jsonify(AHRS.getDisplayableMagneticField())
    response.headers.add('Access-Control-Allow-Origin', '*')
    return response

@app.route('/display/magneticfield/caption', methods=['GET'])
def get_magneticfieldCaption():
    global AHRS
    response = jsonify(AHRS.getDisplayableMagneticFieldCaption())
    response.headers.add('Access-Control-Allow-Origin', '*')
    return response

@app.route('/display/euler', methods=['GET'])
def get_euler():
    global AHRS
    response = jsonify(AHRS.getDisplayableEuler())
    response.headers.add('Access-Control-Allow-Origin', '*')
    return response

@app.route('/display/euler/caption', methods=['GET'])
def get_eulerCaption():
    global AHRS
    response = jsonify(AHRS.getDisplayableEulerCaption())
    response.headers.add('Access-Control-Allow-Origin', '*')
    return response

@app.route('/display/magnetometer_tl', methods=['GET'])
def get_magnetometer_tl():
    global AHRS
    response = jsonify(AHRS.getDisplayableMagnetometerTL())
    response.headers.add('Access-Control-Allow-Origin', '*')
    return response

@app.route('/display/magnetometer_tl/caption', methods=['GET'])
def get_magnetometer_tlCaption():
    global AHRS
    response = jsonify(AHRS.getDisplayableMagnetometerTLCaption())
    response.headers.add('Access-Control-Allow-Origin', '*')
    return response

@app.route('/display/accelerometer_tl', methods=['GET'])
def get_accelerometer_tl():
    global AHRS
    response = jsonify(AHRS.getDisplayableAccelerometerTL())
    response.headers.add('Access-Control-Allow-Origin', '*')
    return response

@app.route('/display/accelerometer_tl/caption', methods=['GET'])
def get_accelerometer_tlCaption():
    global AHRS
    response = jsonify(AHRS.getDisplayableAccelerationTLCaption())
    response.headers.add('Access-Control-Allow-Origin', '*')
    return response


########## Calibration services ##########
@app.route('/calibration/gyrometer/status', methods=['GET'])
def get_gyrometer_calibration_status():
    global AHRS
    response = jsonify(AHRS.getGyrometerCalibrationStatus())
    response.headers.add('Access-Control-Allow-Origin', '*')
    return response

@app.route('/calibration/gyrometer/calibrate', methods=['PUT'])
def put_calibrate_gyrometer():
    global AHRS
    AHRS.calibrateGyrometer()
    response = jsonify({'status': 'OK'})
    response.headers.add('Access-Control-Allow-Origin', '*')
    return response

@app.route('/calibration/accelerometer/status', methods=['GET'])
def get_accelerometer_calibration_statusget_calibration_status():
    global AHRS
    response = jsonify(AHRS.getAccelerometerCalibrationStatus())
    response.headers.add('Access-Control-Allow-Origin', '*')
    return response

@app.route('/calibration/accelerometer/calibrate', methods=['PUT'])
def put_calibrate_accelerometer():
    global AHRS
    AHRS.calibrateAccelerometer()
    response = jsonify({'status': 'OK'})
    response.headers.add('Access-Control-Allow-Origin', '*')
    return response

########## Simulation services ##########
@app.route('/simulation/pendule/state', methods=['GET'])
def get_sim_pendule_state():
    global pendule
    response = jsonify(pendule.getState())
    response.headers.add('Access-Control-Allow-Origin', '*')
    return response

@app.route('/display/simulation/pendule/state', methods=['GET'])
def get_sim_displayable_pendule_state():
    global pendule
    response = jsonify(pendule.getDisplayableState())
    response.headers.add('Access-Control-Allow-Origin', '*')
    return response

@app.route('/display/simulation/pendulum/state/caption', methods=['GET'])
def get_pendulumStateCaption():
    global pendule
    response = jsonify(pendule.getDisplayableStateCaption())
    response.headers.add('Access-Control-Allow-Origin', '*')
    return response

# Expected JSON : {'angle': 'actuator angle (rad)'}
@app.route('/simulation/pendulum/angle', methods=['PUT'])
def put_simulationPendulumAngle():
    global pendule
    pendule.setAngle(request.json["angle"])
    response = jsonify({'status': 'Commands apply OK'})
    response.headers.add('Access-Control-Allow-Origin', '*')
    return response

# Expected JSON : {'x': 'x angle value', 'y': 'y angle value', 'z': 'z angle value'}
@app.route('/servo', methods=['PUT'])
def put_servos():
    global servos
    servos.setAllServoAngle(request.json["x"], request.json["y"], request.json["z"])
    response = jsonify({'status': 'Commands apply OK'})
    response.headers.add('Access-Control-Allow-Origin', '*')
    return response

##########    Servo services    ##########
# Expected JSON : {'angle': 'angle value'}
@app.route('/servo/<string:axis>', methods=['PUT'])
def put_servo(axis):
    global servos
    response = jsonify({'status': 'Command apply'})
    if axis == "x" or axis == "y" or axis == "z":
        servos.setServoAngle(axis, request.json["angle"])
    else:
        response = jsonify({'status': 'Axis incorrect'})
    response.headers.add('Access-Control-Allow-Origin', '*')
    return response

# Expected JSON : {'servoPin': 'pinNumber',
#                  'minPulse': 'value (us)', 'maxPulse': 'value (us)',
#                  'minAngle': 'value (deg)' 'maxAngle': 'value (deg)'}
@app.route('/servo/calibrate/<string:axis>', methods=['POST','OPTIONS'])
def put_servoCalibrateX(axis):
    global servos
    response = jsonify({'status': 'Calibration done'})
    if axis == "x" or axis == "y" or axis == "z":
        servos.configureServo(axis, int(request.form["servoPin"]), int(request.form["minPulse"]), int(request.form["maxPulse"]), int(request.form["minAngle"]), int(request.form["maxAngle"]))
    else:
        response = jsonify({'status': 'Axis incorrect'})
    response.headers.add('Access-Control-Allow-Origin', '*')
    return response

@app.route('/')
def index():
    return render_template('client.html')
    
@app.route('/help')
def help():
   return str(app.url_map)
   
   
if __name__ == '__main__':
    AHRS.start()
    # pendule.start()
    controller = Controller.Controller(AHRS, servos, period=0.06)
    controller.start()
    
    print('Start web server\n')
    app.run(host='0.0.0.0', threaded=True)

    controller.stop()
    AHRS.stop()
    #  pendule.stop()
    
    print('Stop web server\n')
    
