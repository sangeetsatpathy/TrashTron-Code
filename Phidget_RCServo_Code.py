from Phidget22.Phidget import *
from Phidget22.Devices.RCServo import *
from Phidget22.Devices.VoltageRatioInput import *
from Phidget22.Devices.DigitalInput import *
import time

#Declare any event handlers here. These will be called every time the associated event occurs.

def main():

	
	#Create your Phidget channels
	rcServo0 = RCServo()
	thumbStickPhidgetHorizontal = VoltageRatioInput()
	thumbStickPhidgetVertical = VoltageRatioInput()
	thumbStickButton = DigitalInput()

	#Set addressing parameters to specify which channel to open (if any)
	rcServo0.setHubPort(0)
	rcServo0.setChannel(0)

	thumbStickPhidgetVertical.setHubPort(1)
	thumbStickPhidgetHorizontal.setHubPort(1)
	thumbStickButton.setHubPort(1)

	thumbStickPhidgetVertical.setChannel(0)
	thumbStickPhidgetHorizontal.setChannel(1)
	thumbStickButton.setChannel(0)
	#Assign any event handlers you need before calling open so that no events are missed.

	stopProgram = False



	#Open your Phidgets and wait for attachment
	rcServo0.openWaitForAttachment(5000)
	thumbStickPhidgetVertical.openWaitForAttachment(5000)
	thumbStickPhidgetHorizontal.openWaitForAttachment(5000)
	thumbStickButton.openWaitForAttachment(5000)


	half_variable = rcServo0.getMaxPosition()/2.0

	#Do stuff with your Phidgets here or in your event handlers.
	while(not stopProgram):
		if(thumbStickButton.getState()):
			stopProgram = True
			rcServo0.setEngaged(False)
		else:
			voltage_reading = thumbStickPhidgetHorizontal.getVoltageRatio();
			rcServo0.setTargetPosition((1+voltage_reading)*half_variable)
			rcServo0.setEngaged(True)


	#Close your Phidgets once the program is done.
	rcServo0.close()
	thumbStickPhidgetVertical.close()
	thumbStickPhidgetHorizontal.close()
	thumbStickButton.close()

main()