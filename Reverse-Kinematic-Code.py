import sys
import math
import tkinter as tk
import threading
import traceback

sys.path.append("C://Users/Chaniya/Desktop/Brewer/DynamixelProtocol1AXorMXseries_ReubenPython3Class")
#sys.path.append('../')

import DynamixelProtocol1AXorMXseries_ReubenPython3Class


FIRST_ARM_LENGTH = 118.5
SECOND_ARM_LENGTH = 100

moving_in_progress = False

def onclick():
	global x_entry
	global y_entry


	# Move the motors to their starting positions

	x_request = int(x_entry.get())
	y_request = int(y_entry.get())

	move_motor(x_request, y_request)


def move_motor(x, y):
	global DynamixelProtocol1AXorMXseries_Object

	#Calculate each angle
	hypotenuse = math.sqrt(x^2 + y^2)
	angle_a = math.atan(y/x)
	angle_b = math.acos(((hypotenuse^2) + (FIRST_ARM_LENGTH^2) - (SECOND_ARM_LENGTH^2))/(2*hypotenuse*FIRST_ARM_LENGTH))

	angle_first_motor = angle_a - angle_b # Assume that we are starting from the right most, 0 angle.

	angle_c = math.asin(hypotenuse * (math.sin(angle_b) / SECOND_ARM_LENGTH))
	angle_second_motor = 180 - angle_c

	#NOTE: the starting position of the motors will be on the positive x-axis

	# NOTE: First both the motors will be in their 0 position. Then the first motor will turn counterclockwise by angle_first_motor, then the second motor will turn counterclockwise by angle_second_motor.

	#Re-zero each motor
	DynamixelProtocol1AXorMXseries_Object.SetPosition_FROM_EXTERNAL_PROGRAM(0, 0, "deg")
	DynamixelProtocol1AXorMXseries_Object.SetPosition_FROM_EXTERNAL_PROGRAM(0, 0, "deg")

	#Set motors to calculated angle
	DynamixelProtocol1AXorMXseries_Object.SetPosition_FROM_EXTERNAL_PROGRAM(0, angle_first_motor, "deg")
	DynamixelProtocol1AXorMXseries_Object.SetPosition_FROM_EXTERNAL_PROGRAM(0, angle_second_motor, "deg")

def Update_GUI():
	global current_x_label
	global current_y_label
	global DynamixelProtocol1AXorMXseries_Object

	curr_first_motor_angle = DynamixelProtocol1AXorMXseries_Object.GetMostRecentDataDict()["PositionReceived_Degrees"][0]
	curr_second_motor_angle = DynamixelProtocol1AXorMXseries_Object.GetMostRecentDataDict()["PositionReceived_Degrees"][1]


	# Use Forward Kinematics to determine the x,y position based on the angles the motors are providing
	x = math.cos(curr_first_motor_angle) + math.cos(curr_second_motor_angle)
	y = math.sin(curr_first_motor_angle) + math.cos(curr_second_motor_angle)

	current_x_label.config(text="Current x-val: " + str(x))
	current_y_label.config(text="Current y-val: " + str(y))

	window.after(5, Update_GUI)


def end_program():
	key = input("Enter q to exit! \n")
	if(key == 'q'):
		sys.exit()


def Display_GUI():
	global window
	global x_entry
	global y_entry
	global start_btn
	global current_x_label
	global current_y_label

	window = tk.Tk()


	x_label = tk.Label(window, text="Enter x value (in mm)")
	x_label.pack()

	x_entry = tk.Entry(window)
	x_entry.pack()

	y_label = tk.Label(window, text="Enter y value (in mm)")
	y_label.pack()

	y_entry = tk.Entry(window)
	y_entry.pack()

	start_btn = tk.Button(window, text="Move!", command=onclick)
	start_btn.pack()

	current_x_label = tk.Label(window, text="Current x-val: -")
	current_x_label.pack()

	current_y_label = tk.Label(window, text="Current y-val: -")
	current_y_label.pack()

	window.after(5, Update_GUI)

	window.mainloop()


if __name__ == "__main__":
	end_program_thread = threading.Thread(target=end_program)
	end_program_thread.start()

	global DynamixelProtocol1AXorMXseries_Object


	global DynamixelProtocol1AXorMXseries_setup_dict
	DynamixelProtocol1AXorMXseries_setup_dict = dict([("DesiredSerialNumber_USBtoSerialConverter", "FT3M9STOA"),  #Sangeet = FT3M9STOA
													("NameToDisplay_UserSet", "My U2D2"),
													("SerialBaudRate", 1000000),
													("SerialTxBufferSize", 64),
													("SerialRxBufferSize", 64),
													("GlobalPrintByteSequencesDebuggingFlag", 0),  #INPORTANT FOR DEBUGGING
													("ENABLE_SETS", 1),
													("ENABLE_GETS", 1),
													("AskForInfrequentDataReadLoopCounterLimit", 200),
													("MainThread_TimeToSleepEachLoop", 0.010),
													("MotorType_StringList", ["AX", "AX"]), #AX, MX
													("Position_DynamixelUnits_Min_UserSet", [0.0, 0.0]),
													("Position_DynamixelUnits_Max_UserSet", [1023.0, 1023.0]), #1023 for AX-series, 4095 for MX-series
													("Position_DynamixelUnits_StartingValueList", [0.0, 0.0]),
													("Speed_DynamixelUnits_Min_UserSet", [-1023.0, -1023.0]),
													("Speed_DynamixelUnits_Max_UserSet", [1023.0, 1023.0]),
													("Speed_DynamixelUnits_StartingValueList", [1023.0, 1023.0]),
													("MaxTorque_DynamixelUnits_StartingValueList", [1023.0, 1023.0])])
													#("CWlimit_StartingValueList",  [0.0, 0.0]),
													#("CCWlimit_StartingValueList",  [1023.0, 1023.0])])

	try:
		DynamixelProtocol1AXorMXseries_Object = DynamixelProtocol1AXorMXseries_ReubenPython3Class(DynamixelProtocol1AXorMXseries_setup_dict)

	except:
		exceptions = sys.exc_info()[0]
		print("DynamixelProtocol1AXorMXseries_Object __init__: Exceptions: %s" % exceptions)
		traceback.print_exc()

	display_gui_thread = threading.Thread(target=Display_GUI)
	display_gui_thread.start()



