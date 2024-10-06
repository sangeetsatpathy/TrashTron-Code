from logging import warn
import sys
import math
import tkinter as tk
import threading
import traceback
import time

sys.path.append("C:/Users/sange/Desktop/DynamixelProtocol1AXorMXseries_ReubenPython3Class")
sys.path.append("C:/Users/sange/Desktop/Robotics_Playground/DynamixelProtocol2Xseries_ReubenPython3Class-main")
#sys.path.append('../')

from DynamixelProtocol2Xseries_ReubenPython3Class import DynamixelProtocol2Xseries_ReubenPython3Class
from DynamixelProtocol1AXorMXseries_ReubenPython3Class import DynamixelProtocol1AXorMXseries_ReubenPython3Class

def pick_up_bottle():
    HEIGHT_FROM_DISTANCE_SENSOR = 106.15
    X_CHANGE_TO_DISTANCE_SENSOR = 108.7
    LENGTH_OF_ARM = 190.5
    LENGTH_OF_GEARBOX = 264.2
    ANGLE_BETWEEN_ARM_AND_GEARBOX_RADIANS = 1.424
    BIG_GEAR_TEETH = 126.0
    SMALL_GEAR_TEETH = 14.0
    BEVEL_GEAR_2_TEETH = 26.0
    BEVEL_GEAR_1_TEETH = 16.0


    ##Teeth to move big gear: 10.427831871380983
    #Driver shaft rotations: 0.7448451336700702
    #Teeth to move big bevel: 19.365973475421825
    #Rotations of motor: 1.210373342213864
    #Angle to change motor: 7.6049999999999995
    

    angle_for_arm_to_turn = 0.52
    teeth_to_move_big_gear = (angle_for_arm_to_turn/(2*math.pi)) * BIG_GEAR_TEETH
    #print("Teeth to move big gear: " + str(teeth_to_move_big_gear))

    driver_shaft_rotations = teeth_to_move_big_gear / SMALL_GEAR_TEETH
    #print("Driver shaft rotations: " + str(driver_shaft_rotations))

    teeth_to_move_big_bevel = driver_shaft_rotations * BEVEL_GEAR_2_TEETH
    #print("Teeth to move big bevel: " + str(teeth_to_move_big_bevel))

    rotations_motor = teeth_to_move_big_bevel / BEVEL_GEAR_1_TEETH
    #print("Rotations of motor: " + str(rotations_motor))

    angle_to_change_motor = rotations_motor * 360
    print("Angle to change motor: " + str(angle_to_change_motor))
    #Note: counterclockwise movement of gear move the arm down, clockwise rotation moves it up
    #clockwise is a lower angle, counterclockwise is a higher number

    most_recent_dict_position = DynamixelProtocol2Xseries_Object.GetMostRecentDataDict(OverrideLengthMatchingRequirementFlag=1)["PresentPosition_Degrees"]#"PresentPosition"
    position_0 = float(most_recent_dict_position[0])

    print(position_0)

    #lowering the arm means ccw, means higher number
    goal_position_motor = position_0 + angle_to_change_motor
    print("Goal position: " + str(goal_position_motor) + " and in 3 seconds this motor should MOVE")
    time.sleep(3)
    
    #DynamixelProtocol2Xseries_Object.SetEngagedState_FROM_EXTERNAL_PROGRAM(0, 1)

    #TODO: problem -- even when the command here is being executed, it is only returning to the ORIGINAL place

    DynamixelProtocol2Xseries_Object.SetPosition_FROM_EXTERNAL_PROGRAM(0, goal_position_motor, "deg")
    time.sleep(0.01)
    time.sleep(5)

    print(DynamixelProtocol2Xseries_Object.GetMostRecentDataDict(OverrideLengthMatchingRequirementFlag=1)["PresentPosition_Degrees"][0])
    
def move_robot():
    print("Move robot function started")
    most_recent_data_dict = DynamixelProtocol1AXorMXseries_Object.GetMostRecentDataDict()
    most_recent_dict_degrees_0 = most_recent_data_dict["PositionReceived_Degrees"][0]

    ## MX 64 (right motor) has ID 0, MX-106 (left motor) has ID 1; 
    # For moving forward, left motor is POSITIVE addition, right motor is NEGATIVE (subtraction)

    goal_position_0 = most_recent_dict_degrees_0 - 30

    print("Current motor 0 angle: " + str(most_recent_dict_degrees_0) + " and GOAL angle is: " + str(goal_position_0))

    DynamixelProtocol1AXorMXseries_Object.SetPosition_FROM_EXTERNAL_PROGRAM(0, goal_position_0, "deg")
    time.sleep(2)

    most_recent_pos_v2 = DynamixelProtocol1AXorMXseries_Object.GetMostRecentDataDict()["PositionReceived_Degrees"][0]
    print("Most recent Dynamixel Position: " + str(most_recent_pos_v2))

    goal_position_0_v2 = most_recent_pos_v2 + 50
    DynamixelProtocol1AXorMXseries_Object.SetPosition_FROM_EXTERNAL_PROGRAM(0, goal_position_0_v2, "deg")
    time.sleep(2)
    print("Final position: " + str(DynamixelProtocol1AXorMXseries_Object.GetMostRecentDataDict()["PositionReceived_Degrees"][0]))

    sys.exit()
    


if __name__ == "__main__":
    root = tk.Tk()

    DynamixelProtocol2Xseries_TestChannelsList = [0]
    

    Tab_MainControls_2 = root
    Tab_DynamixelProtocol2Xseries = root
    Tab_MyPrint_2 = root

    Data_Label_2 = tk.Label(Tab_MainControls_2, text="Data_Label", width=150)
    Data_Label_2.grid(row=1, column=0, padx=1, pady=1, columnspan=1, rowspan=1)

    global DynamixelProtocol2Xseries_GUIparametersDict
    DynamixelProtocol2Xseries_GUIparametersDict = dict([("USE_GUI_FLAG", 1), # 1 and 1
                                    ("root", Tab_DynamixelProtocol2Xseries),
                                    ("EnableInternal_MyPrint_Flag", 1),
                                    ("NumberOfPrintLines", 10),
                                    ("UseBorderAroundThisGuiObjectFlag", 0),
                                    ("GUI_ROW", 0),
                                    ("GUI_COLUMN", 0),
                                    ("GUI_PADX", 1),
                                    ("GUI_PADY", 10),
                                    ("GUI_ROWSPAN", 1),
                                    ("GUI_COLUMNSPAN", 1)])
    
    SINUSOIDAL_MOTION_INPUT_MinValue_PositionControl_0 = -1800.0 
    SINUSOIDAL_MOTION_INPUT_MinValue_PositionControl_1 = -338.0 / 2.0
    SINUSOIDAL_MOTION_INPUT_MinValue_PositionControl_2 = -338.0 / 2.0
    SINUSOIDAL_MOTION_INPUT_MinValue_PositionControl_3 = -338.0 / 2.0

    SINUSOIDAL_MOTION_INPUT_MaxValue_PositionControl_0 = 1800.0  # TODO: Check these
    SINUSOIDAL_MOTION_INPUT_MaxValue_PositionControl_1 = 338.0 / 2.0
    SINUSOIDAL_MOTION_INPUT_MaxValue_PositionControl_2 = 338.0 / 2.0
    SINUSOIDAL_MOTION_INPUT_MaxValue_PositionControl_3 = 338.0 / 2.0


    global DynamixelProtocol2Xseries_setup_dict 
    DynamixelProtocol2Xseries_setup_dict = dict([("GUIparametersDict", DynamixelProtocol2Xseries_GUIparametersDict),
                                ("DesiredSerialNumber_USBtoSerialConverter", "FT5O0I2VA"), #Change to the serial number of your unique device, FT89FE7OA FT891KTUA
                                ("NameToDisplay_UserSet", "Example Name U2D2"),
                                ("SerialBaudRate", 4000000),
                                ("WatchdogTimeIntervalMilliseconds", 100.0),
                                ("EnableSafetyShutoff", 0),
                                ("GetVariablesEveryNloopsCycles", 3),
                                ("ENABLE_SETS", 1),
                                ("MainThread_TimeToSleepEachLoop", 0.002),
                                ("MotorType_StringList", ["XC330-288-T"]*len(DynamixelProtocol2Xseries_TestChannelsList)), #EACH INPUT LIST MUST BE THE SAME LENGTH AS NUMBER OF MOTORS. XC330-288-T
                                ("MotorName_StringList", ["Large", "Small"]),
                                ("ControlType_StartingValueList", ["ExtendedPositionControlMultiTurn"]*len(DynamixelProtocol2Xseries_TestChannelsList)), #MOTOR ID'S MUST BE IN ORDER FROM 0 T0 (NumberOfMotors - 1) (E.G. FOR 3 MOTORS, THE ID'S WOULD BE 0, 1, AND 2).
                                ("Position_Deg_StartingValueList", [(SINUSOIDAL_MOTION_INPUT_MinValue_PositionControl_0 + SINUSOIDAL_MOTION_INPUT_MaxValue_PositionControl_0)/2.0]),
                                ("Position_Deg_min", [SINUSOIDAL_MOTION_INPUT_MinValue_PositionControl_0] * len(DynamixelProtocol2Xseries_TestChannelsList)),
                                ("Position_Deg_max", [SINUSOIDAL_MOTION_INPUT_MaxValue_PositionControl_0] * len(DynamixelProtocol2Xseries_TestChannelsList)),
                                ("Velocity_DynamixelUnits_StartingValueList", [0.0] * len(DynamixelProtocol2Xseries_TestChannelsList)),
                                ("Velocity_DynamixelUnits_min", [0.0] * len(DynamixelProtocol2Xseries_TestChannelsList)),
                                ("Velocity_DynamixelUnits_max", [60.0] * len(DynamixelProtocol2Xseries_TestChannelsList)),
                                ("Current_DynamixelUnits_StartingValueList", [30.0] * len(DynamixelProtocol2Xseries_TestChannelsList)),
                                ("StartEngagedFlag", [0]*len(DynamixelProtocol2Xseries_TestChannelsList)),
                                ("ListOfVariableNameStringsToGet", ["PresentPosition", "PresentCurrent", "PresentInputVoltage", "PresentTemperature","CurrentLimit", "HardwareErrorStatus", "Shutdown"])])
    ################################################################################3
    ################################################################################
    GUI_SETUP_DICT_PROTOCOL_1 = dict([("USE_GUI_FLAG", 0),
					("root", Tab_DynamixelProtocol2Xseries),
					("EnableInternal_MyPrint_Flag", 1),
					("NumberOfPrintLines", 10),
					("UseBorderAroundThisGuiObjectFlag", 0),
					("GUI_ROW", 0),
					("GUI_COLUMN", 0),
					("GUI_PADX", 1),
					("GUI_PADY", 10),
					("GUI_ROWSPAN", 1),
					("GUI_COLUMNSPAN", 1)])

    MXseries_NumberOfRevolutionsPerDirectionInMultiturnMode  = 4.0
    DynamixelProtocol1AXorMXseries_setup_dict = dict([("GUIparametersDict", GUI_SETUP_DICT_PROTOCOL_1),
                                                    ("DesiredSerialNumber_USBtoSerialConverter", "FT3M9STOA"),
                                                    ("NameToDisplay_UserSet", "My U2D2"),
                                                    ("SerialBaudRate", 1000000),
                                                    ("SerialTxBufferSize", 64),
                                                    ("SerialRxBufferSize", 64),
                                                    ("GlobalPrintByteSequencesDebuggingFlag", 0),  #INPORTANT FOR DEBUGGING
                                                    ("ENABLE_SETS", 1),
                                                    ("ENABLE_GETS", 1),
                                                    ("AskForInfrequentDataReadLoopCounterLimit", 200),
                                                    ("MainThread_TimeToSleepEachLoop", 0.010),
                                                    ("MotorType_StringList", ["MX"]), #AX, MX
                                                    ("Position_DynamixelUnits_Min_UserSet", [-4095.0*MXseries_NumberOfRevolutionsPerDirectionInMultiturnMode]),
                                                    ("Position_DynamixelUnits_Max_UserSet", [4095.0*MXseries_NumberOfRevolutionsPerDirectionInMultiturnMode]), #1023 for AX-series, 4095 for MX-series
                                                    ("Position_DynamixelUnits_StartingValueList", [0.0]),
                                                    ("Speed_DynamixelUnits_Min_UserSet", [-1023.0]),
                                                    ("Speed_DynamixelUnits_Max_UserSet", [1023.0]),
                                                    ("Speed_DynamixelUnits_StartingValueList", [1023.0]),
                                                    ("MaxTorque_DynamixelUnits_StartingValueList", [1023.0]),
                                                    ("CWlimit_StartingValueList",  [4095.0*MXseries_NumberOfRevolutionsPerDirectionInMultiturnMode]),
                                                    ("CCWlimit_StartingValueList",  [-4095.0*MXseries_NumberOfRevolutionsPerDirectionInMultiturnMode]),
                                                    ("MXseries_NumberOfRevolutionsPerDirectionInMultiturnMode", MXseries_NumberOfRevolutionsPerDirectionInMultiturnMode)])



    try:
        global DynamixelProtocol2Xseries_Object
        global DynamixelProtocol1AXorMXseries_Object
        DynamixelProtocol2Xseries_Object = DynamixelProtocol2Xseries_ReubenPython3Class(DynamixelProtocol2Xseries_setup_dict)
        #DynamixelProtocol1AXorMXseries_Object = DynamixelProtocol1AXorMXseries_ReubenPython3Class(DynamixelProtocol2Xseries_setup_dict)
        time.sleep(1)
        most_recent_data_dict_2X = DynamixelProtocol2Xseries_Object.GetMostRecentDataDict(OverrideLengthMatchingRequirementFlag=1)["PresentPosition_Degrees"]
        #print("Motor should still move at this point. ") #TESTING RN: even if I do not give it the command to go to original position, will it do so/
        DynamixelProtocol2Xseries_Object.SetPosition_FROM_EXTERNAL_PROGRAM(0, most_recent_data_dict_2X[0], "deg")
        if(most_recent_data_dict_2X[0] > 1000 or most_recent_data_dict_2X[0] < -1000):
            print("Motor position is way too far out of bounds. There might be a serial communication erorr, as well.")
            sys.exit()

        print("Current position is: " + str(most_recent_data_dict_2X[0]) + ". Motor should still move now.")
        #Setting engaged state.
        time.sleep(3)
        print("LOCKED: motor shouldnt be able to move, and SHOULD NOT MOVE EITHER")
        DynamixelProtocol2Xseries_Object.SetEngagedState_FROM_EXTERNAL_PROGRAM(0, 1) # motor ID, then State 
        time.sleep(3)

        print("Hiiiiiii")
        DYNAMIXEL_OBJ_SUCCESSFULLY_CREATED = True
    except:
        exceptions = sys.exc_info()[0]
        print("DynamixelProtocol1AXorMXseries_Object __init__: Exceptions: %s" % exceptions)
        traceback.print_exc()

    pick_up_bottle() # test function for 2X motors
    #move_robot() # test function for MX motors
    
    root.mainloop()
    root.quit()
    root.destroy()