
from DynamixelProtocol1AXorMXseries_ReubenPython3Class import *


if __name__ == '__main__':
	global DynamixelProtocol1AXorMXseries_Object


	global DynamixelProtocol1AXorMXseries_setup_dict
    DynamixelProtocol1AXorMXseries_setup_dict = dict([("GUIparametersDict", DynamixelProtocol1AXorMXseries_GUIparametersDict),
                                                    ("DesiredSerialNumber_USBtoSerialConverter", "FT5O0I2VA"),  #Sangeet = FT3M9STOA
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



# while not exit program:


DynamixelProtocol1AXorMXseries_MostRecentDict = DynamixelProtocol1AXorMXseries_Object.GetMostRecentDataDict()


## TO EXAMINE::
 if USE_SINUSOIDAL_POS_CONTROL_INPUT_FLAG == 1:

            time_gain = math.pi / (2.0 * SINUSOIDAL_MOTION_INPUT_ROMtestTimeToPeakAngle)
            SINUSOIDAL_INPUT_TO_COMMAND = (SINUSOIDAL_MOTION_INPUT_MaxValue + SINUSOIDAL_MOTION_INPUT_MinValue) / 2.0 + 0.5 * abs(SINUSOIDAL_MOTION_INPUT_MaxValue - SINUSOIDAL_MOTION_INPUT_MinValue) * math.sin(time_gain * CurrentTime_MainLoopThread)
            #print("SINUSOIDAL_INPUT_TO_COMMAND: " + str(SINUSOIDAL_INPUT_TO_COMMAND))

            DynamixelProtocol1AXorMXseries_Object.SetPosition_FROM_EXTERNAL_PROGRAM(SINUSOIDAL_MOTION_INPUT_MotorID, SINUSOIDAL_INPUT_TO_COMMAND, "deg")

            LED_ToggleCounter = LED_ToggleCounter + 1
            if LED_ToggleCounter == 100:
                DynamixelProtocol1AXorMXseries_Object.ToggleLEDstate_FROM_EXTERNAL_PROGRAM(SINUSOIDAL_MOTION_INPUT_MotorID)
                LED_ToggleCounter = 0