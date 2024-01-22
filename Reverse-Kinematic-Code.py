from logging import warn
import sys
import math
import tkinter as tk
import threading
import traceback
from tkinter import ttk
import time

sys.path.append("C:/Users/sange/OneDrive/Desktop/DynamixelProtocol1AXorMXseries_ReubenPython3Class")
#sys.path.append('../')

from DynamixelProtocol1AXorMXseries_ReubenPython3Class import DynamixelProtocol1AXorMXseries_ReubenPython3Class


FIRST_ARM_LENGTH = 118.5
SECOND_ARM_LENGTH = 100

DYNAMIXEL_OBJ_SUCCESSFULLY_CREATED = False

moving_in_progress = False

def onclick():
	global x_entry
	global y_entry


	# Move the motors to their starting positions

	x_request = int(x_entry.get())
	y_request = int(y_entry.get())

	move_motor(x_request, y_request)


def move_motor(x, y):# 90, 80
	global DynamixelProtocol1AXorMXseries_Object
	global warning_label
	global last_angles_label
	global debugging_label

	#Calculate each angle
	hypotenuse = math.sqrt(pow(x, 2) + pow(y, 2)) #sqrt(90^2 + 80^2)
	try:
		angle_a = math.atan2(y, x)
		angle_b = math.acos(((pow(hypotenuse,2)) + (pow(FIRST_ARM_LENGTH,2)) - (pow(SECOND_ARM_LENGTH,2)))/(2*hypotenuse*FIRST_ARM_LENGTH))
	except(ValueError):
		warning_label.config(text="Invalid x,y coordinate: either out of arm range or too little for the arm range")
		return
	angle_first_motor_rad = angle_a - angle_b # Assume that we are starting from the right most, 0 angle.

	angle_c = math.asin(hypotenuse * (math.sin(angle_b) / SECOND_ARM_LENGTH))
	angle_second_motor_rad = math.pi - angle_c

	debugging_label.config(text=f"angle a: {angle_a * (180.0/math.pi)} \n angle b: {angle_b * (180.0/math.pi)} \n angle c: {angle_c * (180.0/math.pi)}")

	angle_first_motor = angle_first_motor_rad * (180.0/math.pi)
	angle_second_motor = angle_second_motor_rad * (180.0/math.pi)

	last_angles_label.config(text=f"Last angle requested of first motor: {angle_first_motor} ; \n Last angle requested of second motor: {angle_second_motor}")
	#NOTE: the starting position of the motors will be on the positive x-axis
	
	# NOTE: First both the motors will be in their 0 position. Then the first motor will turn counterclockwise by angle_first_motor, then the second motor will turn counterclockwise by angle_second_motor.

	angle_first_motor+= 4.69 #"zero" angle for first motor.

	#Re-zero each motor
	DynamixelProtocol1AXorMXseries_Object.SetPosition_FROM_EXTERNAL_PROGRAM(0, 4.69, "deg")
	time.sleep(1)
	DynamixelProtocol1AXorMXseries_Object.SetPosition_FROM_EXTERNAL_PROGRAM(1, 0, "deg")

	time.sleep(1)

	if(angle_first_motor < 0):
		angle_first_motor = 360 + angle_first_motor
	if(angle_second_motor < 0):
		angle_first_motor = 360 + angle_second_motor

	#Set motors to calculated angle
	DynamixelProtocol1AXorMXseries_Object.SetPosition_FROM_EXTERNAL_PROGRAM(0, angle_first_motor, "deg")
	time.sleep(1)
	DynamixelProtocol1AXorMXseries_Object.SetPosition_FROM_EXTERNAL_PROGRAM(1, angle_second_motor, "deg")

	time.sleep(1)

def Update_GUI():
	global current_x_label
	global current_y_label
	global DynamixelProtocol1AXorMXseries_Object
	global DYNAMIXEL_OBJ_SUCCESSFULLY_CREATED

	if(DYNAMIXEL_OBJ_SUCCESSFULLY_CREATED):
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
		DynamixelProtocol1AXorMXseries_Object.ExitProgram_Callback()
		sys.exit()


def Display_GUI():
	global window
	global x_entry
	global y_entry
	global start_btn
	global current_x_label
	global current_y_label
	global TabControlObject
	global Tab_DynamixelProtocol1AXorMXseries
	global GUI_SETUP_DICT
	global warning_label
	global last_angles_label
	global debugging_label

	window = tk.Tk()

	TabControlObject = ttk.Notebook(window)

	Tab_DynamixelProtocol1AXorMXseries = ttk.Frame(TabControlObject)
	TabControlObject.add(Tab_DynamixelProtocol1AXorMXseries, text='   Dynamixel   ')
	Tab_MainControls = ttk.Frame(TabControlObject)
	TabControlObject.add(Tab_MainControls, text='   Main Controls   ')

	Tab_MyPrint = ttk.Frame(TabControlObject)
	TabControlObject.add(Tab_MyPrint, text='   MyPrint Terminal   ')

	TabControlObject.pack(expand=1, fill="both")  # CANNOT MIX PACK AND GRID IN THE SAME FRAME/TAB, SO ALL .GRID'S MUST BE CONTAINED WITHIN THEIR OWN FRAME/TAB.

	GUI_SETUP_DICT = dict([("USE_GUI_FLAG", 1),
					("root", Tab_DynamixelProtocol1AXorMXseries),
					("EnableInternal_MyPrint_Flag", 1),
					("NumberOfPrintLines", 10),
					("UseBorderAroundThisGuiObjectFlag", 0),
					("GUI_ROW", 0),
					("GUI_COLUMN", 0),
					("GUI_PADX", 1),
					("GUI_PADY", 10),
					("GUI_ROWSPAN", 1),
					("GUI_COLUMNSPAN", 1)])

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

	warning_label = tk.Label(window, text="No warnings as of now")
	warning_label.pack()

	last_angles_label = tk.Label(window, text="Last angle requested of first motor: - ; \n Last angle requested of second motor: -")
	last_angles_label.pack()

	debugging_label = tk.Label(window, text="angle a: - \n angle b: - \n angle_first_motor: - \n angle c: -")
	debugging_label.pack()

	window.after(10, Update_GUI)

	window.mainloop()


if __name__ == "__main__":
	end_program_thread = threading.Thread(target=end_program)
	end_program_thread.start()

	display_gui_thread = threading.Thread(target=Display_GUI)
	display_gui_thread.start()

	time.sleep(4) # wait for the GUI thread to run a little so that GUI_SETUP_DICT is defined

	global Tab_DynamixelProtocol1AXorMXseries

	global GUI_SETUP_DICT

	global DynamixelProtocol1AXorMXseries_setup_dict

	DynamixelProtocol1AXorMXseries_setup_dict = dict([("GUIparametersDict", GUI_SETUP_DICT),
													("DesiredSerialNumber_USBtoSerialConverter", "FT3M9STOA"),  #Sangeet = FT3M9STOA
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

		global DynamixelProtocol1AXorMXseries_Object
		DynamixelProtocol1AXorMXseries_Object = DynamixelProtocol1AXorMXseries_ReubenPython3Class(DynamixelProtocol1AXorMXseries_setup_dict)
		print("Hiiiiiii")
		DYNAMIXEL_OBJ_SUCCESSFULLY_CREATED = True
	except:
		exceptions = sys.exc_info()[0]
		print("DynamixelProtocol1AXorMXseries_Object __init__: Exceptions: %s" % exceptions)
		traceback.print_exc()





