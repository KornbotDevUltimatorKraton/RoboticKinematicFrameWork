#!/user/bin/env python
# Author code: Mr.Chanapai Chuadchum 
# Describesion: Robotic Arm opensource 
import sys # import the system function
import numpy as np # Math for the analytic the matrix kinematrix
import math # The math for the calculation function of the kinematic framework
import scipy  # Function for the machinelearning learning from the csv file fun$
import csv # Record the map of the position movement
from nanpy import(ArduinoApi,SerialManager) # Robotic arm control serial functi$
from nanpy import Servo
#>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
               # Robotic Arm Sensory input and the control function
# Servo part for the robotic arm
servo3e = Servo(3)
servo3u = Servo(4)
servo1u = Servo(5)
servo1e = Servo(8)
servo2u = Servo(9)
servo2e = Servo(10)
          # Servo for the robotic wrist of the arm
servoWrist= Servo(43) # Wrist servo
servowristrotate = Servo(44) # Wrist rotate servo
#>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
try:
   connectionHand = SerialManager('/dev/ttyACM0',115200) # Serial manager funct$
   a = ArduinoApi(connection=connectionHand) # The connection hand function for$
except:
    print("Please check Serial Hand connection")
try:
   connectionBody = SerialManager('/dev/ttyACM1',115200) #Serial Mamnanger S
   b = ArduinoApi(connection=connectionBody) # The connection Body function for$
except:
   print("Please check the Serial Body connection") # Serial body function
#>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
        # Robotic Sensory and control function of the robotic arm
#Hand Part
     # Servo part for the robotic arm
servo3e = Servo(3)
servo3u = Servo(4)
servo1u = Servo(5)
servo1e = Servo(8)
servo2u = Servo(9)
servo2e = Servo(10)
#>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
          # Servo for the robotic wrist of the arm
servoWrist= Servo(43) # Wrist servo
servowristrotate = Servo(44) # Wrist rotate servo
#>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
  # Stepper motor at the finger
#Finger 1
a.pinMode(11,a.OUTPUT) #dir
a.pinMode(12,a.OUTPUT) #step
#Finger 2
a.pinMode(2,a.OUTPUT)  #dir
a.pinMode(6,a.OUTPUT)  #step
#>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   
    #Stepper motor Motor power cotrol function
a.pinMode(29,a.OUTPUT) #Stepper motor Base power on/off
a.pinMode(30,a.OUTOUT) #Stepper motor Shoulder power on/off
a.pinMode(31,a.OUTPUT) #Stepper motor Elbow power on/off
#>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
   #Body part function of the  robotic arm  control and sensory
#Base function for the robotic arm
b.pinMode(2,b.OUTPUT) # dir
b.pinMode(3,b.OUTPUT) # step
#Shoulder function for the robotic arm
b.pinMode(4,b.OUTPUT) # dir
b.pinMode(5,b.OUTPUT) # step 
#Elbow function for the robotic arm
b.pinMode(8,b.OUTPUT) #dir 
b.pinMode(9,b.OUTPUT) #step
#>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
       #Robotics Sensory function for the Sensor input option
Finger1Temp = 0 #Finger 1 Sensory for Temp1 
Finger2Temp = 1 #Finger 2 Sensory for Temp2 
Finger3Temp = 2 #Finger 3 Sensory for Temp3 
#>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
       # Force sensor for the robotics arm
Finger1Force = 3 # Finger 1 Sensory Force sense  
Finger2Force = 4 # Finger 2 Sensory Force sense 
Finger3Force = 5 # Finger 3 Sensory Force sense
#>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
       # Capasitive sensor function for the Sensitive static 
Finger1Cap = 6 # Finger 1 Sensory capasitive function 
Finger2Cap = 7 # Finger 2 Sensory capasitive function 
Finger3Cap = 8 # Finger 3 Sensory capasitive function 
#>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
       # Angle Feed back of the robotic joint recorder function 
AngleBrec = 9
AngleSrec = 10 
AngleErec = 11       # b
AngleWRrec = 12 
AngleWrec = 13 
AngleWrxrec = 14 
AngleWryrec = 15 
#>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
     # Angle Hand    #a 
AngleFinger1 = 0  # Angle Finger 1
AnglerFinger15 = 1
#>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
AngleFinger2 = 2  # Angle Finger 2 
AngleFinger25 = 3 # Angle Finger 3
#>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
AngleFinger3 = 4  # Angle Finger 4
AngleFinger35 = 5 # Angle Finger 5  
#>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
               

#>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
       #Barrett hand function for the robotic arm to control the finger and actuator 
def BarrettHandFunction(x,y,z,Anglef1,Anglef2,Anglef3):
              #Finger 1 functional positioning control 
    Finger1(dir1,AngleI1,AngleF1,Speed)
      #Finger 2 functional position control 
    Finger2(dir2,AngleI2,AngleF2,Speed)
      #Finger 3 funtional position control 
    Finger3(dir3,AngleI3,AngleF3,Speed)
      #Finger 3 rotator 
    StepperFinger3(dir5,step3,speed,timing)
      #Finger 2 rotator 
    StepperFinger2(dir4,step2,speed,timing)
     
 # Finger control for the robot barrett hand 
def Finger1(dir,AngleI,AngleF,speed):
    if dir == 1:
        for move in range(AngleF,AngleI,1): #Finger move catch 
            servo1u.write(4+move)
            servo1e.write(5+move)  #Finger 1 move catch  
            time.sleep(speed) # Speed control function for the servo motor
    else if dir == 0 :
        for move in  range(AngleI,AngleF,-1):
            servo1u.write(4+move)
            servo1e.write(5+move)
            time.sleep(speed) #Speed control function for the servo motor 
def Finger2(dir,AngleI,AngleF,speed):
     if dir == 1:
         for move in range(AngleF,AngleI,1): #Finger move catch 
            servo2u.write(170-move)
            servo2e.write(move)
            time.sleep(speed) # Speed control function for the servo motor 
     else if dir == 0 :
         for move in range(AngleI,AngleF,-1):
             servo2u.write(180-move)
             servo2e.write(move)
             time.sleep(speed)  #Speed control function for the servo motor 
def Finger3(dir,AngleI,AngleF,speed):
      if dir == 1:
         for move in range(AngleF,AngleI,1):
             servo3e.write(10-move)
             servo3u.write(move)
             time.sleep(speed) # speed control function for the servo motor 
      else if dir == 0:
         for move in range(AngleI,AngleF,-1):
             servo3e.write(move)
             servo3u.write(move)
             time.sleep(speed) #Speed control function for the  servo motor 
       #Distance for the angles s transformation  s,w is the lenght bottom to the lenght wrist 
def StepperFinger3(dir,step,speed,timing):
       if dir == 1:
          a.digitalWrite(11,a.HIGH) # Forward 
          for move in range(0,step,1):
               a.digitalWrite(12,a.HIGH)
               time.sleep(speed)
               a.digitalWrite(12,a.LOW)
               time.sleep(speed)
       time.sleep(timing) # timing for the stepper  
       if dir == 0:
          a.digitalWrite(11,a.LOW) # Backward 
          for move in range(0,step,1):
              a.digitalWrite(12,a.HIGH)
              time.sleep(speed)
              a.digitalWrite(12,a.LOW)
              time.sleep(speed)
       time.sleep(timing) # timing for the stepper 
def StepperFinger2(dir,step,speed,timing):
       if dir == 1:
          a.digitalWrite(2,a.HIGH) # Forward 
          for move in range(0,step,1):
               a.digitalWrite(6,a.HIGH)
               time.sleep(speed)
               a.digitalWrite(6,a.LOW)
               time.sleep(speed)
       time.sleep(timing) # timing for the stepper  
       if dir == 0:
          a.digitalWrite(2,a.LOW) # Backward 
          for move in range(0,step,1):
              a.digitalWrite(6,a.HIGH)
              time.sleep(speed)
              a.digitalWrite(6,a.LOW)
              time.sleep(speed)
       time.sleep(timing) # timing for the stepper 
def DistanceCalculation(x,y,z,s,w,AnglefE):   # AnglefE is the feed back control function for the angle using the 9 DOF 
     d = math.sqrt(math.pow(s,2) + math.pow(w,2) - 2*s*w*math.cos(math.radians(AnglefE))) # Elbow angle computational 
     angleZ = math.atan(z/math.sqrt(math.pow(x,2)+ math.pow(y,2)))# Angle function for the s1     
     dy = d*math.cos(angleZ) # AngleZ 
     return dy #The dy is the distance from the angle needed  
def DistanceCoordination(x,y): 
     D2d = math.sqrt(math.pow(x,2) + math.pow(y,2)) # Function of the 
     return D2d    #Distance of the 2 dimentsion output function 

def TotalAngleShoulder(x,y,z,s,w,AnglefE):
     d = math.sqrt(math.pow(s,2) + math.pow(w,2) - 2*s*w*math.cos(math.radians($
     AngleSt = math.degrees(math.acos(d/(2*s))) + math.degrees(math.atan(z/math.sqrt(math.pow(x,2) + math.pow(y,2)))) 
     return AngleSt  #return Angle of the Shoulder 
       # Kinematic calculation function
def KinematicBodyandcontrol(Catch,x,y,z,s,w,AngleS,AngleH,Anglez,AngleE,AngleB,AnglefE,Anglef1,Anglef2,Anglef3):
      #Robotic Conditioning  pickt up 
 if Catch == 1: 
      
    if DistanceCoordination(x,y) == DistanceCalculation(x,y,z,s,w,AnglefE): 
            BarrettHandFunction(x,y,z,Anglef1,Anglef2,Anglef3) #AngleFinger control option   
   
   #Robotic calculationnot pick up just move show  
  if Catch == 0: 
          

   # Record the flow angle function of the robotic arm 
  if Catch == 2:                       

def AngleRecorder(AngleB,AngleS,AngleE,AngleW,AngleF1,AngleF2,AngleF3): # Angle input for the angle recorder   
          

def Angle_Display(): # Open the CSV file to play the angle of the system 

      
def csvfilewriter(): # Wristing the CSV file 
 

def csvfilRead():  #Reading the CSV file 
    
def HandStepCutof(Logic): 
    # Logic check to turn on and off the system motor control power option function 
   if Logic == 1: 
       

   if Logic == 0: 

while(True):     
    
      

