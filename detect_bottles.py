import torch
import cv2
import tkinter
import threading
import time
import math
import sys
from Phidget22.Devices.DistanceSensor import DistanceSensor

sys.path.append("C:/Users/sange/Desktop/DynamixelProtocol1AXorMXseries_ReubenPython3Class")
sys.path.append("C:/Users/sange/Desktop/Robotics_Playground/DynamixelProtocol2Xseries_ReubenPython3Class-main")
#sys.path.append('../')

from DynamixelProtocol1AXorMXseries_ReubenPython3Class import DynamixelProtocol1AXorMXseries_ReubenPython3Class
from DynamixelProtocol2Xseries_ReubenPython3Class import DynamixelProtocol2Xseries_ReubenPython3Class

class ObjectDetection:
    def __init__(self):
        self.model = self.load_model()
        self.classes = self.model.names
        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
        print("\n\nDevice Used:",self.device)

    def load_model(self):
        model = torch.hub.load('ultralytics/yolov5', 'custom', path="C:/Users/sange/Desktop/Robotics_Playground/yolov5/runs/train/exp/weights/best.pt", force_reload=True)
        return model
    
    def create_detections(self, frame, confidence=0.4):
        self.model.to(self.device)
        frame_arr = [frame]
        results = self.model(frame_arr)

        #dataframe = results.pandas().xyxy[0]

#      xmin    ymin    xmax   ymax  confidence  class    name
# 0  749.50   43.50  1148.0  704.5    0.874023      0  person
# 1  433.50  433.50   517.5  714.5    0.687988     27     tie
# 2  114.75  195.75  1095.0  708.0    0.624512      0  person
# 3  986.00  304.00  1028.0  420.0    0.286865     27     tie

        labels, cord = results.xyxyn[0][:, -1], results.xyxyn[0][:, :-1]

        n = len(labels)
        x_shape, y_shape = frame.shape[1], frame.shape[0] #sets the x width and y width
        detections = []

        for i in range(n):
            row = cord[i]

            if(row[4] > confidence):
                x1, y1, x2, y2 = int(row[0]*x_shape), int(row[1]*y_shape), int(row[2]*x_shape), int(row[3]*y_shape)
                detections.append([x1, y1, int(x2-x1), int(y2-y1), row[4], 'bottle'])
        print(detections)
        return frame, detections
       
    



# if the bottle is to the left or the right, we need to turn the robot that respective way TILL it is centered.

def center_robot(): # NOTE: At this point, we know that the bottle is THERE. so we probably dont have to do verification detections.
    # HOWEVER: this can be a problem if we turn left too much, resulting in the bottle being out of frame. Then we do not know if the robot is there.
    KP = 0.8
    ret, raw_frame = player.read()
    frame, detections = model.create_detections(raw_frame)

    if(len(detections) != 0):
        first_bottle = detections[0]
        (bot_x1, bot_y1, bot_width, bot_height) = (first_bottle[0], first_bottle[1], first_bottle[2], first_bottle[3])

        frame_with_box = cv2.rectangle(frame, (bot_x1, bot_y1), (bot_x1 + bot_width, bot_y1 + bot_height), (36, 255, 12), 1)
        img_width = frame.shape[1]

        print("Robot centering has begun!")
        CENTER_TOLERANCE = 10



        center_x = int(img_width/2)

        bottle_center_x = bot_x1 + int(bot_width/2)
        bottle_center_y = bot_y1 + int(bot_height/2)

        frame_with_point = cv2.circle(frame_with_box, (bottle_center_x, bottle_center_y), 0, (0, 0, 255), -1)

        cv2.imshow("preview", frame_with_point)
        cv2.waitKey(1)

        #determine if it is left, right or centered
        if(abs(bottle_center_x-center_x) < CENTER_TOLERANCE): 

            #if the 
            #stop_wheels()
            btn.config(text=button_messages[2])
            return # end the function

            #Alert Tkinter to allow the robot to move forward and pickup
        elif(bottle_center_x < center_x):
            deviation = center_x - bottle_center_x

            angle = math.atan2(deviation, 432.0) # in RADIANS
            angle_degrees = angle * (180.0 / math.pi) * KP

            turn_left(angle_degrees)
        else:
            deviation_r = bottle_center_x - center_x

            #######################################
            # code for speed control
            # max_deviation_possible_r = center_x
            #percent_deviation_right = deviation_r / max_deviation_possible_r
            #speed_right = percent_deviation_right * MAX_SPEED
            ########################################

            angle_right = math.atan2(deviation_r, 432.0) # in RADIANS
            angle_degrees = angle_right * (180.0 / math.pi) * KP
            turn_right(angle_degrees)
        # Needs to keep turning until the thing is processed; but we still need to keep the frame data coming in
    else:
        cv2.imshow("preview", frame)
        cv2.waitKey(1)
    time.sleep(0.02)
    center_robot()

def pick_up_bottle():
    HEIGHT_FROM_DISTANCE_SENSOR = 106.15
    X_CHANGE_TO_DISTANCE_SENSOR = 108.7
    LENGTH_OF_ARM = 190.5
    #LENGTH_OF_GEARBOX = 264.2
    LENGTH_OF_GEARBOX = 266.7

    MIN_ANGLE_BETWEEN_ARM_AND_GEARBOX_RADIANS = 1.424

    ## The following constants are for motor 0
    BIG_GEAR_TEETH_0 = 126.0
    SMALL_GEAR_TEETH_0 = 14.0
    BEVEL_GEAR_2_TEETH_0 = 26.0
    BEVEL_GEAR_1_TEETH_0 = 16.0

    ## The following constants are for motor 1
    GEAR_ON_ARM_CIRCUMFERENCE_1 = 80.0 * math.pi
    LIFTER_GEAR_CIRCUMFERENCE_1 = 24.0 * math.pi



    print("Bottle pickup has begun!")
    # Taking the average of distance measurements
    list_distances = []
    for i in range(10):
        current_distance_detection = distance_sense.getDistance()
        if(current_distance_detection < 600):
            list_distances.append(current_distance_detection)
        time.sleep(0.7)
    print(list_distances)
    average_distance = sum(list_distances) / len(list_distances)
    print("Average distance measurement: " + str(average_distance))
    # Kinematics stuff
    alpha = math.atan2(X_CHANGE_TO_DISTANCE_SENSOR + average_distance, HEIGHT_FROM_DISTANCE_SENSOR)
    hypotenuse = math.sqrt(math.pow(HEIGHT_FROM_DISTANCE_SENSOR, 2) + math.pow(X_CHANGE_TO_DISTANCE_SENSOR + average_distance, 2))

    #intermediate_variable = math.sin(ANGLE_BETWEEN_ARM_AND_GEARBOX_RADIANS) / hypotenuse
    intermediate_var = (math.pow(LENGTH_OF_ARM, 2) + math.pow(hypotenuse, 2) - math.pow(LENGTH_OF_GEARBOX, 2))/(2*hypotenuse*LENGTH_OF_ARM)
    print("Alpha = " + str(alpha) + "; Hypotenuse = " + str(hypotenuse) + "; Intermediate_var = " + str(intermediate_var))
    angle_c = math.acos(intermediate_var)
    print("Calculated angle C: " + str(angle_c) + " Calculated alpha: " + str(alpha))

    ##############################################
    ########## Calculations for Motor 0
    ##############################################
    angle_for_arm_to_turn_0 = math.pi - alpha - angle_c
    print("Angle for arm to turn: " + str(angle_for_arm_to_turn_0))
    teeth_to_move_big_gear = (angle_for_arm_to_turn_0/(2*math.pi)) * BIG_GEAR_TEETH_0

    driver_shaft_rotations = teeth_to_move_big_gear / SMALL_GEAR_TEETH_0
    teeth_to_move_big_bevel = driver_shaft_rotations * BEVEL_GEAR_2_TEETH_0

    rotations_motor = teeth_to_move_big_bevel / BEVEL_GEAR_1_TEETH_0
    angle_to_change_motor_0 = rotations_motor * 360

    #Note: counterclockwise movement of gear move the arm down, clockwise rotation moves it up
    #clockwise is a lower angle, counterclockwise is a higher number

    print("Angle to change motor 0: " + str(angle_to_change_motor_0))
   

    ###################################
    ######## Motor 1 Calculations
    ###################################
    intermediate_var_for_theta = (hypotenuse * math.sin(angle_c))/LENGTH_OF_GEARBOX
    theta = math.pi - math.asin(intermediate_var_for_theta)  # there are 2 arcsin values. How do we know which one to use? For now, just subtract from 180

    if(theta < MIN_ANGLE_BETWEEN_ARM_AND_GEARBOX_RADIANS):
        print("Angle requested of motor 1 is too low;")
        return
    print("Theta value: " + str(theta))

    angle_change_joint_1 = theta - MIN_ANGLE_BETWEEN_ARM_AND_GEARBOX_RADIANS
    print("Angle to change in the joint (radians): " + str(angle_change_joint_1))
    distance_along_circle_1 = GEAR_ON_ARM_CIRCUMFERENCE_1 * (angle_change_joint_1 / (2*math.pi))
    print("Distance to turn the gear in mm: " + str(distance_along_circle_1))
    revolutions_lifter_gear_1  = distance_along_circle_1 / LIFTER_GEAR_CIRCUMFERENCE_1
    print("Revolutions to turn small lifter gear: " + str(revolutions_lifter_gear_1))
    # TODO: figure out if we are moving it clockwise or counterclockwise.
    degrees_to_turn_motor1 = revolutions_lifter_gear_1 * 360


    print("Angle to change motor 1: " + str(degrees_to_turn_motor1))
  
    ###############################################
    ############## Movement
    ###############################################

    most_recent_dict_position = DynamixelProtocol2Xseries_Object.GetMostRecentDataDict(OverrideLengthMatchingRequirementFlag=1)["PresentPosition_Degrees"]#"PresentPosition"
    position_0 = float(most_recent_dict_position[0])
    position_1 = float(most_recent_dict_position[1])
    position_2 = float(most_recent_dict_position[2])

    print("Position of motor 0: " + str(position_0))
    print("Position of motor 1: " + str(position_1))

    #lowering the arm means ccw, means higher number
    goal_position_0 = position_0 + angle_to_change_motor_0 
    goal_position_1 = position_1 + degrees_to_turn_motor1 # Moves counterclockwise to raise the gearbox

    print("Goal position 1: " + str(goal_position_1) + " wait 3 seconds for the gripper joint to move")
    time.sleep(3)
    DynamixelProtocol2Xseries_Object.SetPosition_FROM_EXTERNAL_PROGRAM(1, goal_position_1, "deg")
    time.sleep(5)
    print(DynamixelProtocol2Xseries_Object.GetMostRecentDataDict(OverrideLengthMatchingRequirementFlag=1)["PresentPosition_Degrees"][1])


    print("Goal position 0: " + str(goal_position_0) +" wait 3 seconds for arm to move.")
    time.sleep(3)
    DynamixelProtocol2Xseries_Object.SetPosition_FROM_EXTERNAL_PROGRAM(0, goal_position_0, "deg")
    time.sleep(5)

    print(DynamixelProtocol2Xseries_Object.GetMostRecentDataDict(OverrideLengthMatchingRequirementFlag=1)["PresentPosition_Degrees"][0])
    


    #Note: counterclockwise movement of gear move the arm down, clockwise rotation moves it up

    #clockwise is a lower angle, counterclockwise is a higher number
    
    #most_recent_dict_position = DynamixelProtocol2Xseries_Object.GetMostRecentDataDict(OverrideLengthMatchingRequirementFlag=1)["PresentPosition"]

    #position_0 = most_recent_dict_position[0] # motor 0 controls gripper
    #position_1 = most_recent_dict_position[1] # Motor 1 is arm controller
    #position_2 = most_recent_dict_position[2] # Motor 2 controls joint
    DEGREES_MAX = 63.98
    DEGREES_MIN = 40.38
    DEGREES_TO_TURN_2 = DEGREES_MAX - DEGREES_MIN
    # GRIPPER MOVEMENT NOW
    # turning the gripper gear counterclockwise CLOSES the gripper (so we need a higher number)
    DynamixelProtocol2Xseries_Object.SetEngagedState_FROM_EXTERNAL_PROGRAM(2, 1) 
    goal_position_2 = position_2 + DEGREES_TO_TURN_2
    print("Goal position 2: " + str(goal_position_2) + " wait 1 second for gripper to move.")
    time.sleep(1)
    DynamixelProtocol2Xseries_Object.SetPosition_FROM_EXTERNAL_PROGRAM(2, goal_position_2, "deg")
    time.sleep(1)

    print("Gripped bottle, now raising arm to original position in 3 seconds")
    time.sleep(3)
    DynamixelProtocol2Xseries_Object.SetPosition_FROM_EXTERNAL_PROGRAM(0, position_0, "deg")
    




def GUI_Update(): #  There should be a button if there is a bottle detected; a button that is activated when   
    text.config(text="5 s")
    window.after(5, GUI_Update)

def onclick():
    btn_text = btn.cget('text')
    if(btn_text == button_messages[0]):
        print("Button not enabled yet.")
    elif(btn_text == button_messages[1]):
        center_robot()
    elif(btn_text == button_messages[2]):
        move_towards_robot()
    elif(btn_text == button_messages[3]):
        pick_up_bottle()

def endprogram():
    #press Q to exit
    while True:
        key = input("press q to exit \n")
        if (key == 'q'):
            print("program end function called")
            cv2.destroyAllWindows()
            sys.exit()
def stop_wheels():
    # TODO: check if this is even necessary!!!
    DynamixelProtocol1AXorMXseries_Object.SetSpeed_FROM_EXTERNAL_PROGRAM(1, 0, "PERCENT")
    DynamixelProtocol1AXorMXseries_Object.SetSpeed_FROM_EXTERNAL_PROGRAM(0, 0, "PERCENT")

# for both motors, CCW is turning right; CW means turning left
def turn_left(angle_change):
    #turning left means motor 0 will turn forward and motor 1 will turn backward
    motor_angle_left = (332.811/60.0) * angle_change

   # print("turned the robot LEFT by angle : " + str(angle_change))
    print("Robot to turn left by: " + str(angle_change))

    most_recent_data_dict = DynamixelProtocol1AXorMXseries_Object.GetMostRecentDataDict()
    most_recent_dict_degrees_0 = most_recent_data_dict["PositionReceived_Degrees"][0]
    most_recent_dict_degrees_1 = most_recent_data_dict["PositionReceived_Degrees"][1]

    #####################################################
    #####################################################
    # Code from Speed Control Attempt
    #####################################################
    #self, MotorIndex, SpeedFromExternalProgram, Units = "None"
    #DynamixelProtocol1AXorMXseries_Object.SetSpeed_FROM_EXTERNAL_PROGRAM(1, -1*angle_change, "PERCENT") # moves forward (cw)
    #DynamixelProtocol1AXorMXseries_Object.SetSpeed_FROM_EXTERNAL_PROGRAM(0, -1*angle_change, "PERCENT") #this has to be backwards (cw)
    #####################################################

    # A lower angle means more Clockwise, a higher angle means more counterclockwise
    goal_position_0 = most_recent_dict_degrees_0 - motor_angle_left
    goal_position_1 = most_recent_dict_degrees_1 - motor_angle_left


    print("Motor 0 - Current Position was: " + str(most_recent_dict_degrees_0 )+ " and the goal motor position was: " + str(goal_position_0))
    print("Motor 1 - Current Position was: " + str(most_recent_dict_degrees_1 )+ " and the goal motor position was: " + str(goal_position_1))
    DynamixelProtocol1AXorMXseries_Object.SetPosition_FROM_EXTERNAL_PROGRAM(0, goal_position_0, "DEG")
    DynamixelProtocol1AXorMXseries_Object.SetPosition_FROM_EXTERNAL_PROGRAM(1, goal_position_1, "DEG")
    time.sleep(1)
    updated_data_dict = DynamixelProtocol1AXorMXseries_Object.GetMostRecentDataDict()
    updated_data_0 = updated_data_dict["PositionReceived_Degrees"][0]
    updated_data_1 = updated_data_dict["PositionReceived_Degrees"][1]
    print("Motor 0 - final motor position was: " + str(updated_data_0))
    print("Motor 1 - final motor position was: " + str(updated_data_1))
    
def turn_right(angle_change):
    #turning right means motor 0 will turn backward,  motor 1 will move forward 
    motor_angle_right = (332.811/60.0) * angle_change


    print("Robot to turn right by: " + str(angle_change))

    most_recent_data_dict = DynamixelProtocol1AXorMXseries_Object.GetMostRecentDataDict()
    most_recent_dict_degrees_0 = most_recent_data_dict["PositionReceived_Degrees"][0]
    most_recent_dict_degrees_1 = most_recent_data_dict["PositionReceived_Degrees"][1]
    ###############################################
    ###############################################
    #Code from Speed Control Attempt
    ###############################################
    #DynamixelProtocol1AXorMXseries_Object.SetSpeed_FROM_EXTERNAL_PROGRAM(1, 1*angle_change, "PERCENT") #moves backward (ccw)
    #DynamixelProtocol1AXorMXseries_Object.SetSpeed_FROM_EXTERNAL_PROGRAM(0, 1*angle_change, "PERCENT") #moves forward (ccw)
    ###############################################
    goal_position_0 = most_recent_dict_degrees_0 + motor_angle_right
    goal_position_1  = most_recent_dict_degrees_1 + motor_angle_right
    print("Motor 0 - Current Position was: " + str(most_recent_dict_degrees_0 )+ " and the goal motor position was: " + str(goal_position_0))
    print("Motor 1 - Current Position was: " + str(most_recent_dict_degrees_1 )+ " and the goal motor position was: " + str(goal_position_1))
    DynamixelProtocol1AXorMXseries_Object.SetPosition_FROM_EXTERNAL_PROGRAM(0, goal_position_0, "DEG")
    DynamixelProtocol1AXorMXseries_Object.SetPosition_FROM_EXTERNAL_PROGRAM(1, goal_position_1, "DEG")

    time.sleep(1)
    updated_data_dict = DynamixelProtocol1AXorMXseries_Object.GetMostRecentDataDict()
    updated_data_0 = updated_data_dict["PositionReceived_Degrees"][0]
    updated_data_1 = updated_data_dict["PositionReceived_Degrees"][1]
    print("Motor 0 - final motor position was: " + str(updated_data_0))
    print("Motor 1 - final motor position was: " + str(updated_data_1))


def move_towards_robot():
    IDEAL_DISTANCE_MM = 250
    WHEEL_DIAMETER = 60.0
    WHEEL_CIRCUMFERENCE = math.pi * WHEEL_DIAMETER
    print("move-towards-robot function called!")
    #move the motors forward TILL the distance sensor detects it
    #DynamixelProtocol1AXorMXseries_Object.SetSpeed_FROM_EXTERNAL_PROGRAM(1, 5, "PERCENT") #start the motors again
    #DynamixelProtocol1AXorMXseries_Object.SetSpeed_FROM_EXTERNAL_PROGRAM(0, 5, "PERCENT") 

    #DynamixelProtocol1AXorMXseries_Object.SetPosition_FROM_EXTERNAL_PROGRAM(0, 1000000000000, "DEG")
    #DynamixelProtocol1AXorMXseries_Object.SetPosition_FROM_EXTERNAL_PROGRAM(1, 1000000000000, "DEG") # Set it to a high number, so it becomes speed control

    #bottle_in_range = False
    #while(bottle_in_range == False):
    #    if(distance_sense.getDistance() - 370 < 10):
    #        bottle_in_range = True
    #        stop_wheels()
    #        continue
    #    print("moved a bit forward!")
    

    # Taking the average of distance measurements
    list_distances = []
    for i in range(10):
        current_distance_detection = distance_sense.getDistance()
        if(current_distance_detection < 1500):
            list_distances.append(current_distance_detection)
        time.sleep(0.7)

    average_distance = sum(list_distances) / len(list_distances)

    distance_to_move = average_distance - IDEAL_DISTANCE_MM

    print(list_distances)
    print("Calculated average distance: " + str(average_distance) + " Distance to move: " + str(distance_to_move))
    
    if(distance_to_move > 50): 
        num_rotations = distance_to_move / WHEEL_CIRCUMFERENCE # MAKE SURE THIS IS A FLOAT
        motor_angle_turns = num_rotations*360.0
        most_recent_data_dict = DynamixelProtocol1AXorMXseries_Object.GetMostRecentDataDict()
        most_recent_dict_degrees_0 = most_recent_data_dict["PositionReceived_Degrees"][0]
        most_recent_dict_degrees_1 = most_recent_data_dict["PositionReceived_Degrees"][1]

        ## MX 64 (right motor) has ID 0, MX-106 (left motor) has ID 1; 
        # For moving forward, left motor is POSITIVE addition, right motor is NEGATIVE (subtraction)
        print("Calculated number of rotations: " + str(num_rotations))

        goal_position_0 = most_recent_dict_degrees_0 - motor_angle_turns
        goal_position_1 = most_recent_dict_degrees_1 + motor_angle_turns

        print("Current motor 0 angle: " + str(most_recent_dict_degrees_0) + " and GOAL angle is: " + str(goal_position_0))
        print("Current motor 1 angle: " + str(most_recent_dict_degrees_1) + " and GOAL angle is: " + str(goal_position_1))

        # Motor 0 Position will DECREASE over time, while Motor 1 Position will INCREASE over time
        # Assumption for now, we will have 200 small movements.
        NUM_ITERATIONS = 200
        turn_fragments = motor_angle_turns / NUM_ITERATIONS

        step_position_0 = most_recent_dict_degrees_0 - turn_fragments
        step_position_1 = most_recent_dict_degrees_1 + turn_fragments

        for i in range(NUM_ITERATIONS):
            DynamixelProtocol1AXorMXseries_Object.SetPosition_FROM_EXTERNAL_PROGRAM(0, step_position_0, "deg") # TODO: do this movement over smaller movements.
            DynamixelProtocol1AXorMXseries_Object.SetPosition_FROM_EXTERNAL_PROGRAM(1, step_position_1, "deg")

            step_position_0 -= turn_fragments
            step_position_1 += turn_fragments
            time.sleep(0.002)


    print("Bottle in range now!")
    btn.config(text=button_messages[3])

def process_current_frame_undetected():
    #NOTE: the motors do not move even at this point.

    #print("processing current frame")
    STANDARD_DEV_THRESHOLD = 20
    
    ret, raw_frame = player.read()
    frame, detections = model.create_detections(raw_frame)

    #TODO: check if there are any detections here based on the past 5 frames
    if(len(past_five_frames) == 5):  
        #check if we count the detection
        # RULES: 4/5 of them must be actual detections
        # The x-values must have a decent standard deviation (if the std deviation is too high, print error message that the model is BAD, or low confidence)
        
        list_detected = [] # this a list of indeces of any detected instances
        sum = 0
        for i in range(len(past_five_frames)):
            if(past_five_frames[i] != -1):
                list_detected.append(i)
                sum += past_five_frames[i]
        if(len(list_detected) == 0):
            average = -1
        else:
            average = sum / len(list_detected)
        
        standard_deviation = -1
        if(average != -1):
            sum_deviations = 0

            for index in list_detected:
                deviation = past_five_frames[index] - average
                sum_deviations += pow(deviation, 2)
            
            standard_deviation = math.sqrt(sum_deviations/len(list_detected))


        if(len(list_detected) >= 4 and standard_deviation < STANDARD_DEV_THRESHOLD): # checking if there is a valid detection
            btn.config(text=button_messages[1]) ## if almost all the frames are detections, and they are not very varied from one another, then we are good.
            #If there is a bottle detected, then the list of widths must be good
            past_five_widths.sort()
            median_width = -1
            len_widths = len(past_five_widths)
            if(len_widths % 2 == 0): #even
                median_index_right = int(len_widths / 2)
                median_index_left = int(median_index_right - 1)
                median_width = (past_five_widths[median_index_left] + past_five_widths[median_index_right]) / 2.0
            else: # odd list
                median_index = int(math.floor(len_widths / 2))
                median_width = past_five_widths[median_index]
            
            #Setting up distance similarity; bottle distance in MM
            bottle_distance = (432.0 * 63.5) / median_width
            print("Bottle Distance is at: " + str(bottle_distance))
            return # end the loop of this function


    if(len(detections) == 0):
        cv2.imshow("preview", frame)
        cv2.waitKey(1)
        if(len(past_five_frames) == 5):
            past_five_frames.pop(0)
            if(len(past_five_widths) != 0):
                past_five_widths.pop(0)
        past_five_frames.append(-1)
        
    else:
        first_bottle = detections[0]## if there is a detection, then add it to the list of the past 5 detections!
       

        (bot_x1, bot_y1, bot_width, bot_height) = (first_bottle[0], first_bottle[1], first_bottle[2], first_bottle[3])
        print("Detected bottle width: " + str(bot_width))
        img_with_bb = cv2.rectangle(frame, (bot_x1, bot_y1), (bot_x1+bot_width, bot_y1+bot_height), (36, 255, 12), 1)
        cv2.imshow("preview",img_with_bb)
        cv2.waitKey(1)

        bottle_center_x = bot_x1 + int(bot_width/2)

        
        if(len(past_five_frames) == 5):
            past_five_frames.pop(0)
            if(len(past_five_widths) != 0):
                past_five_widths.pop(0)
        past_five_frames.append(bottle_center_x)
        past_five_widths.append(bot_width)

       #if(position != "centered"):
           # center_robot(x1=bot_x1, width=bot_width, img_width=frame.shape[1]) ## if there is a detection, and the robot is not already centered, then center the robot.
    #TODO: add a repetition of this function
    print(past_five_frames)

    time.sleep(0.5)
    process_current_frame_undetected()

def stow_arm():
    print("Right now, we are assuming that arm is stowed to begin with.")
    process_current_frame_undetected()

if __name__ == "__main__":
    global player
    print("Main thread")
    # we will assume there is only 1 bottle now...
    global past_five_frames
    global button_messages
    global btn
    global window
    global text
    global distance_sense
    global wheel_motors
    global past_five_widths
    global bottle_distance

    global DynamixelProtocol1AXorMXseries_Object

    button_messages = ["-", "Bottle detected! Click to center!", "Centering complete! Cick to move toward bottle.", "Bottle is within range to pickup. Click to pick up the bottle"]
    past_five_frames = []
    past_five_widths = []
    bottle_distance = -1


    distance_sense = DistanceSensor()
    distance_sense.setDeviceSerialNumber(374119)
    distance_sense.setHubPort(3)
    distance_sense.setChannel(0)
    distance_sense.openWaitForAttachment(5000)



    model = ObjectDetection()

    player = cv2.VideoCapture(1) 
    
    #so we are assuming there is 1 bottle or no bottles. All of the detections are above 0.4 confidence, so just choose the highest confidence one.

    window = tkinter.Tk()
    tkinter.Label(window, text='Hello World!').pack()

    btn = tkinter.Button(window, text=button_messages[0], command=onclick)
    btn.pack()

    text = tkinter.Label(window, text="0 s") # why is this not showing up?
    text.pack()

    #Tab_MainControls_1 = window
    Tab_DynamixelProtocol1AXorMXseries = window
    Tab_DynamixelProtocol2Xseries = window
    #Tab_MyPrint_1 = window

    #Data_Label_1 = tkinter.Label(Tab_MainControls_1, text="Data_Label", width=150)
    #Data_Label_1.grid(row=1, column=0, padx=1, pady=1, columnspan=1, rowspan=1)


    GUI_SETUP_DICT_PROTOCOL_1 = dict([("USE_GUI_FLAG", 0),
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
                                                    ("MotorType_StringList", ["MX", "MX"]), #AX, MX
                                                    ("Position_DynamixelUnits_Min_UserSet", [-4095.0*MXseries_NumberOfRevolutionsPerDirectionInMultiturnMode, -4095.0*MXseries_NumberOfRevolutionsPerDirectionInMultiturnMode]),
                                                    ("Position_DynamixelUnits_Max_UserSet", [4095.0*MXseries_NumberOfRevolutionsPerDirectionInMultiturnMode, 4095.0*MXseries_NumberOfRevolutionsPerDirectionInMultiturnMode]), #1023 for AX-series, 4095 for MX-series
                                                    ("Position_DynamixelUnits_StartingValueList", [0.0, 0.0]),
                                                    ("Speed_DynamixelUnits_Min_UserSet", [-1023.0, -1023.0]),
                                                    ("Speed_DynamixelUnits_Max_UserSet", [1023.0, 1023.0]),
                                                    ("Speed_DynamixelUnits_StartingValueList", [1023.0, 1023.0]),
                                                    ("MaxTorque_DynamixelUnits_StartingValueList", [1023.0, 1023.0]),
                                                    ("CWlimit_StartingValueList",  [4095.0*MXseries_NumberOfRevolutionsPerDirectionInMultiturnMode, 4095.0*MXseries_NumberOfRevolutionsPerDirectionInMultiturnMode]),
                                                    ("CCWlimit_StartingValueList",  [-4095.0*MXseries_NumberOfRevolutionsPerDirectionInMultiturnMode, -4095.0*MXseries_NumberOfRevolutionsPerDirectionInMultiturnMode]),
                                                    ("MXseries_NumberOfRevolutionsPerDirectionInMultiturnMode", MXseries_NumberOfRevolutionsPerDirectionInMultiturnMode)])

    try:
        DynamixelProtocol1AXorMXseries_Object = DynamixelProtocol1AXorMXseries_ReubenPython3Class(DynamixelProtocol1AXorMXseries_setup_dict)
        DynamixelProtocol1AXorMXseries_Object.SetEngagedState_FROM_EXTERNAL_PROGRAM(0, 0) # Set both motors to torque OFF so it doesnt spin
        DynamixelProtocol1AXorMXseries_Object.SetEngagedState_FROM_EXTERNAL_PROGRAM(1, 0)
        most_recent_data_dict_MX = DynamixelProtocol1AXorMXseries_Object.GetMostRecentDataDict()["PositionReceived_Degrees"]
        print("Starting to set positions for MX motors.  Motor 0: " + str(most_recent_data_dict_MX[0]) +" ; Motor 1: " + str(most_recent_data_dict_MX[1]))
        DynamixelProtocol1AXorMXseries_Object.SetPosition_FROM_EXTERNAL_PROGRAM(0, most_recent_data_dict_MX[0], "deg")
        DynamixelProtocol1AXorMXseries_Object.SetPosition_FROM_EXTERNAL_PROGRAM(1, most_recent_data_dict_MX[1], "deg")
        DynamixelProtocol1AXorMXseries_Object.SetEngagedState_FROM_EXTERNAL_PROGRAM(0, 1) # Set both motors to torque ON to keep it at current pos.
        DynamixelProtocol1AXorMXseries_Object.SetEngagedState_FROM_EXTERNAL_PROGRAM(1, 1)
        print("Dynamixel MX Motors Successfully set up!")
        DYNAMIXEL_OBJ_1_SUCCESSFULLY_CREATED = True
    except:
        exceptions = sys.exc_info()[0]
        print("DynamixelProtocol1AXorMXseries_Object __init__: Exceptions: %s" % exceptions)



    GUI_SETUP_DICT_PROTOCOL_2 = dict([("USE_GUI_FLAG", 0),
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
    #SINUSOIDAL_MOTION_INPUT_MinValue_PositionControl_1 = -338.0 / 2.0
    #SINUSOIDAL_MOTION_INPUT_MinValue_PositionControl_2 = -338.0 / 2.0
    #SINUSOIDAL_MOTION_INPUT_MinValue_PositionControl_3 = -338.0 / 2.0

    SINUSOIDAL_MOTION_INPUT_MaxValue_PositionControl_0 = 1800.0 # TODO: Check these
    #SINUSOIDAL_MOTION_INPUT_MaxValue_PositionControl_1 = 338.0 / 2.0
    #SINUSOIDAL_MOTION_INPUT_MaxValue_PositionControl_2 = 338.0 / 2.0
    #SINUSOIDAL_MOTION_INPUT_MaxValue_PositionControl_3 = 338.0 / 2.0

    DynamixelProtocol2Xseries_TestChannelsList = [0, 1, 2]


    global DynamixelProtocol2Xseries_setup_dict 
    DynamixelProtocol2Xseries_setup_dict = dict([("GUIparametersDict", GUI_SETUP_DICT_PROTOCOL_2),
                                ("DesiredSerialNumber_USBtoSerialConverter", "FT5O0I2VA"), #Change to the serial number of your unique device, FT89FE7OA FT891KTUA
                                ("NameToDisplay_UserSet", "Example Name U2D2"),
                                ("SerialBaudRate", 4000000),
                                ("WatchdogTimeIntervalMilliseconds", 100.0),
                                ("EnableSafetyShutoff", 0),
                                ("GetVariablesEveryNloopsCycles", 3),
                                ("ENABLE_SETS", 1),
                                ("MainThread_TimeToSleepEachLoop", 0.002),
                                ("MotorType_StringList", ["XC330-181-T"]*len(DynamixelProtocol2Xseries_TestChannelsList)), #EACH INPUT LIST MUST BE THE SAME LENGTH AS NUMBER OF MOTORS. XC330-288-T
                                ("MotorName_StringList", ["Large", "Small"]),
                                ("ControlType_StartingValueList", ["ExtendedPositionControlMultiTurn"]*len(DynamixelProtocol2Xseries_TestChannelsList)), #MOTOR ID'S MUST BE IN ORDER FROM 0 T0 (NumberOfMotors - 1) (E.G. FOR 3 MOTORS, THE ID'S WOULD BE 0, 1, AND 2).
                                ("Position_Deg_StartingValueList", [300]*len(DynamixelProtocol2Xseries_TestChannelsList)),
                                ("Position_Deg_min", [SINUSOIDAL_MOTION_INPUT_MinValue_PositionControl_0]*len(DynamixelProtocol2Xseries_TestChannelsList)),
                                ("Position_Deg_max", [SINUSOIDAL_MOTION_INPUT_MaxValue_PositionControl_0]*len(DynamixelProtocol2Xseries_TestChannelsList)),
                                ("Velocity_DynamixelUnits_StartingValueList", [0.0] * len(DynamixelProtocol2Xseries_TestChannelsList)),
                                ("Velocity_DynamixelUnits_min", [-1620.0, -131.0, 166.0]),
                                ("Velocity_DynamixelUnits_max", [60.0] * len(DynamixelProtocol2Xseries_TestChannelsList)),
                                ("Current_DynamixelUnits_StartingValueList", [-1500.0] * len(DynamixelProtocol2Xseries_TestChannelsList)),
                                ("StartEngagedFlag", [0]*len(DynamixelProtocol2Xseries_TestChannelsList)),
                                ("ListOfVariableNameStringsToGet", ["PresentPosition", "PresentCurrent", "PresentInputVoltage", "PresentTemperature","CurrentLimit", "HardwareErrorStatus", "Shutdown"])])


    try:
        DynamixelProtocol2Xseries_Object = DynamixelProtocol2Xseries_ReubenPython3Class(DynamixelProtocol2Xseries_setup_dict)
        print("Dynamixel XC330 Motors Successfully set up!")
        DYNAMIXEL_OBJ_2_SUCCESSFULLY_CREATED = True

        most_recent_data_dict_2X = DynamixelProtocol2Xseries_Object.GetMostRecentDataDict(OverrideLengthMatchingRequirementFlag=1)["PresentPosition_Degrees"] 
        print("Starting to set positions for XC330 Motors. Current position of Motor 0: " + str(most_recent_data_dict_2X[0]) + " and Current position of Motor 1: " + str(most_recent_data_dict_2X[1]))
        DynamixelProtocol2Xseries_Object.SetPosition_FROM_EXTERNAL_PROGRAM(0, most_recent_data_dict_2X[0], "deg")
        DynamixelProtocol2Xseries_Object.SetPosition_FROM_EXTERNAL_PROGRAM(1, most_recent_data_dict_2X[1], "deg")
        #DynamixelProtocol2Xseries_Object.SetPosition_FROM_EXTERNAL_PROGRAM(1, most_recent_data_dict_2X[2], "deg")

        #Setting engaged state.
        DynamixelProtocol2Xseries_Object.SetEngagedState_FROM_EXTERNAL_PROGRAM(0, 1) # motor ID, then State
        DynamixelProtocol2Xseries_Object.SetEngagedState_FROM_EXTERNAL_PROGRAM(1, 1)
        #DynamixelProtocol2Xseries_Object.SetEngagedState_FROM_EXTERNAL_PROGRAM(2, 1)
    except:
        exceptions = sys.exc_info()[0]
        print("DynamixelProtocol2Xseries_Object __init__: Exceptions: %s" % exceptions)


    image_thread = threading.Thread(target=stow_arm) 
    image_thread.start()
    print("image_thread started!")

    end_program_thread = threading.Thread(target=endprogram)
    end_program_thread.start()

    gui_thread = threading.Thread(target=GUI_Update)
    gui_thread.start()

    ##Check if the bottle is in the center of the screen  

    window.mainloop()
    window.quit()
    window.destroy()
