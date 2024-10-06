from logging import warn
import sys
import math
import tkinter as tk
import threading
import traceback
from tkinter import ttk
import time

sys.path.append("C:/Users/sange/OneDrive/Desktop/DynamixelProtocol2Xseries_ReubenPython3Class-main")
#sys.path.append('../')

from DynamixelProtocol2Xseries_ReubenPython3Class import DynamixelProtocol2Xseries_ReubenPython3Class



DYNAMIXEL_OBJ_SUCCESSFULLY_CREATED = False

moving_in_progress = False

def onclick():
	global motor1_input
	global motor2_input


	# Move the motors to their starting positions

	x_request = int(motor1_input.get())
	y_request = int(motor2_input.get())



def Update_GUI():
	global current_motor1_degree
	global current_motor2_degree
	global DynamixelProtocol1AXorMXseries_Object
	global DYNAMIXEL_OBJ_SUCCESSFULLY_CREATED

	if(DYNAMIXEL_OBJ_SUCCESSFULLY_CREATED):
		curr_first_motor_angle = DynamixelProtocol1AXorMXseries_Object.GetMostRecentDataDict()["PositionReceived_Degrees"][0]
		curr_second_motor_angle = DynamixelProtocol1AXorMXseries_Object.GetMostRecentDataDict()["PositionReceived_Degrees"][1]


		# Use Forward Kinematics to determine the x,y position based on the angles the motors are providing
		x = math.cos(curr_first_motor_angle) + math.cos(curr_second_motor_angle)
		y = math.sin(curr_first_motor_angle) + math.cos(curr_second_motor_angle)

		current_motor1_degree.config(text="Current x-val: " + str(x))
		current_motor2_degree.config(text="Current y-val: " + str(y))

	window.after(5, Update_GUI)


def end_program():
	key = input("Enter q to exit! \n")
	if(key == 'q'):
		DynamixelProtocol1AXorMXseries_Object.ExitProgram_Callback()
		sys.exit()


def Display_GUI():
	global window
	global motor1_input
	global motor2_input
	global motor3_input
	global motor4_input
	global start_btn
	global current_motor1_degree
	global current_motor2_degree
	global current_motor3_degree
	global current_motor4_degree
	global TabControlObject
	global Dynamixel_Control_Tab
	global GUI_SETUP_DICT
	global warning_label
	global last_angles_label
	global debugging_label

	window = tk.Tk()

	TabControlObject = ttk.Notebook(window)

	Dynamixel_Control_Tab = ttk.Frame(TabControlObject)
	TabControlObject.add(Dynamixel_Control_Tab, text='   Dynamixel   ')
	Tab_MainControls = ttk.Frame(TabControlObject)
	TabControlObject.add(Tab_MainControls, text='   Main Controls   ')

	Tab_MyPrint = ttk.Frame(TabControlObject)
	TabControlObject.add(Tab_MyPrint, text='   MyPrint Terminal   ')

	TabControlObject.pack(expand=1, fill="both")  # CANNOT MIX PACK AND GRID IN THE SAME FRAME/TAB, SO ALL .GRID'S MUST BE CONTAINED WITHIN THEIR OWN FRAME/TAB.

	GUI_SETUP_DICT = dict([("USE_GUI_FLAG", 1),
					("root", Dynamixel_Control_Tab),
					("EnableInternal_MyPrint_Flag", 1),
					("NumberOfPrintLines", 10),
					("UseBorderAroundThisGuiObjectFlag", 0),
					("GUI_ROW", 0),
					("GUI_COLUMN", 0),
					("GUI_PADX", 1),
					("GUI_PADY", 10),
					("GUI_ROWSPAN", 1),
					("GUI_COLUMNSPAN", 1)])

	motor1_input_label = tk.Label(window, text="Enter Degrees of Motor 1")
	motor1_input_label.pack()

	motor1_input = tk.Entry(window)
	motor1_input.pack()

	motor2_input_label = tk.Label(window, text="Enter Degrees of Motor 2")
	motor2_input_label.pack()

	motor2_input = tk.Entry(window)
	motor2_input.pack()

	motor3_input_label = tk.Label(window, text="Enter Degrees of Motor 3")
	motor3_input_label.pack()

	motor3_input = tk.Entry(window)
	motor3_input.pack()

	motor4_input_label = tk.Label(window, text="Enter Degrees of Motor 4")
	motor4_input_label.pack()

	motor4_input = tk.Entry(window)
	motor4_input.pack()



	start_btn = tk.Button(window, text="Move!", command=onclick)
	start_btn.pack()

	current_motor1_degree = tk.Label(window, text="Current angle of Motor 1: -")
	current_motor1_degree.pack()

	current_motor2_degree = tk.Label(window, text="Current angle of Motor 2: -")
	current_motor2_degree.pack()

	current_motor3_degree = tk.Label(window, text="Current angle of Motor 3: -")
	current_motor3_degree.pack()

	current_motor4_degree = tk.Label(window, text="Current angle of Motor 4: -")
	current_motor4_degree.pack()

	warning_label = tk.Label(window, text="No warnings as of now")
	warning_label.pack()

	window.after(10, Update_GUI)

	window.mainloop()


if __name__ == "__main__":
	end_program_thread = threading.Thread(target=end_program)
	end_program_thread.start()

	display_gui_thread = threading.Thread(target=Display_GUI)
	display_gui_thread.start()

	time.sleep(4) # wait for the GUI thread to run a little so that GUI_SETUP_DICT is defined

	global GUI_SETUP_DICT

	global DynamixelProtocol1AXorMXseries_setup_dict

	DynamixelProtocol2Xseries_setup_dict = dict([("GUIparametersDict", GUI_SETUP_DICT),
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

		global DynamixelProtocol2Xseries_Object
		DynamixelProtocol1AXorMXseries_Object = DynamixelProtocol2Xseries_ReubenPython3Class(DynamixelProtocol2Xseries_setup_dict)
		print("Hiiiiiii")
		DYNAMIXEL_OBJ_SUCCESSFULLY_CREATED = True
	except:
		exceptions = sys.exc_info()[0]
		print("DynamixelProtocol1AXorMXseries_Object __init__: Exceptions: %s" % exceptions)
		traceback.print_exc()





