from Phidget22.Phidget import *
from Phidget22.Devices.RCServo import *
from Phidget22.Devices.VoltageRatioInput import *
from Phidget22.Devices.DigitalInput import *
from Phidget22.Devices.DigitalOutput import *
import time

#Declare any event handlers here. These will be called every time the associated event occurs.

def main():

	
	#Create your Phidget channels
	thumbStickPhidgetVertical = VoltageRatioInput()
	thumbStickButton = DigitalInput()
	lights = DigitalOutput()

	#Set addressing parameters to specify which channel to open (if any)
	lights.setHubPort(2)
	lights.setChannel(0)

	thumbStickPhidgetVertical.setHubPort(1)
	thumbStickButton.setHubPort(1)

	thumbStickPhidgetVertical.setChannel(0)
	thumbStickButton.setChannel(0)
	#Assign any event handlers you need before calling open so that no events are missed.

	stopProgram = False

	lights.setIsHubPortDevice(True)

	#Open your Phidgets and wait for attachment
	lights.openWaitForAttachment(5000)
	thumbStickPhidgetVertical.openWaitForAttachment(5000)
	thumbStickButton.openWaitForAttachment(5000)



	#Do stuff with your Phidgets here or in your event handlers.
	while(not stopProgram):
		if(thumbStickButton.getState()):
			stopProgram = True
		else:
			voltage_reading = thumbStickPhidgetVertical.getVoltageRatio();
			if(voltage_reading >= 0.1):
				lights.setState(True)
			else:
				lights.setState(False)

	lights.setState(False)
	#Close your Phidgets once the program is done.
	lights.close()
	thumbStickPhidgetVertical.close()
	thumbStickButton.close()

main()