# Summer: Offput button latching to flight board
# Summer: Hack apart the controller to remove spring
import sys
import os
import threading
from time import sleep
from xboxdrv_parser import Controller

sys.path.append (os.path.abspath("../"))
from communication.zmq_communicator import communicator

SENSITIVITY = 1.0

def adjust_throttle (inputs):
    THROTTLE_C = .26
    inputs["Z"] = inputs["Z"] + (THROTTLE_C * (abs (inputs["Pitch"]) + abs (inputs["Roll"])))
    return inputs

def main ():
    com = communicator ("Controller")

    final_packet = {"Pitch":0, "Yaw":0, "Roll":0, "Z":0, "Arm":0, "Stabalize":0}

    controller =  Controller (["X1", "Y1", "X2", "Y2", "[]", "/\\", "start", "R2", "L2"], ["Yaw", "Z", "Roll", "Pitch", "Unarm", "Arm", "Calibrate", "Stabalize1", "Stabalize2"])

    IS_ARMED = 0
    IS_STABALIZED = 0
    CALIBRATE = 0

    while True:
        inputs = controller.get_values ()

        try:
            # Buttons (0 - 100)
            inputs["Arm"] = controller.map_range (inputs["Arm"], 0, 255, 0.0, 100.0)
            inputs["Unarm"] = controller.map_range (inputs["Unarm"], 0, 255, 0.0, 100.0)
            inputs["Calibrate"] = controller.map_range (inputs["Calibrate"], 0, 1, 0.0, 100.0)
            inputs["Stabalize1"] = controller.map_range (inputs["Stabalize1"], 0, 255, 0.0, 100.0)
            inputs["Stabalize2"] = controller.map_range (inputs["Stabalize2"], 0, 255, 0.0, 100.0)

            # Throttle (0 - 100)
            inputs["Z"] = controller.map_range (inputs["Z"], 0, 255, 100.0, 0.0) / (1.0 / SENSITIVITY) # Reversed

            # Directions (-100 - 100)
            inputs["Yaw"] = controller.map_range (inputs["Yaw"], 0, 255, -100.0, 100.0) / (1.0 / SENSITIVITY) 
            inputs["Pitch"] = controller.map_range (inputs["Pitch"], 0, 255, 100.0, -100.0) / (1.0 / SENSITIVITY)  # Reversed
            inputs["Roll"] = controller.map_range (inputs["Roll"], 0, 255, -100.0, 100.0) / (1.0 / SENSITIVITY) 

            # Have to press hard to prevent accidental arm/unarm/throttle helper/throttle hold
            if inputs["Unarm"] == 100.0:
                inputs["Unarm"] = 100.0
            else:
                inputs["Unarm"] = 0

            if inputs["Arm"] == 100.0:
                inputs["Arm"] = 100.0
            else:
                inputs["Arm"] = 0

            if inputs["Stabalize1"] == 100.0:
                inputs["Stabalize1"] = 100.0
            else:
                inputs["Stabalize1"] = 0
            
            if inputs["Stabalize2"] == 100.0:
                inputs["Stabalize2"] = 100.0
            else:
                inputs["Stabalize2"] = 0

            if inputs["Calibrate"] == 100.0:
                inputs["Calibrate"] = 100.0
            else:
                inputs["Calibrate"] = 0

            if inputs["Stabalize1"] and inputs["Stabalize2"]:
                IS_STABALIZED = 1
            else:
                IS_STABALIZED = 0

            if inputs["Calibrate"]:
                CALIBRATE = 1
            else:
                CALIBRATE = 0

            # Logic so you don't have to hold arm/unarm/etc
            if IS_ARMED and inputs["Unarm"]:
                inputs["Arm"] = 0
                IS_ARMED = 0
                print "UNARMED!" 
            if not IS_ARMED and inputs["Arm"]:
                inputs["Arm"] = 1
                IS_ARMED = 1
                print "ARMED!"

            for key in inputs.keys ():
                if abs (inputs[key]) < 10:
                    inputs[key] = 0.0 

            final_packet = {"Pitch":inputs["Pitch"], "Yaw":inputs["Yaw"], "Roll":inputs["Roll"], "Z":inputs["Z"], "Arm":IS_ARMED, "Stabalize":IS_STABALIZED, "Calibrate": CALIBRATE}

            # Clips small controller movements to zero. Necessary especially for throttle so copter will arm
        
            com.send_message (final_packet)

        except KeyError:
            pass

        print final_packet
    
        # Send 5 commands/sec
        sleep (.1)


main ()
