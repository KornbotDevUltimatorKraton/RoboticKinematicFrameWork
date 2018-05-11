import numpy as np numpy  # For the math of matrix calculation 
import matplotlib.pyplot as plt  #Math modelling function for the 3D visualization of dynamic angle 
import math   # Math function for the angle transformation calculation of the kinematic framwork 
from nanpy import(ArduinApi,SerialManager) # Serial for the Hardware connection 
from nanpy import Servo # Servo controller for the Angle actuator input feed back control motor  
import Tkinter 


try: 
    connectionbody =  SerialManager('/dev/ttyACM0',115200)   # Serial connection for the system of the hardwareat the body  
    a = ArduinoApi(connection=connectionbody) # Connection body for the hardware control 
try:   
    connectionHand = SerialManager(]'/dev/ttyACM1',115200)   # Serial connection for the hardware at the hand 
    b = ArduinoApi(connection=connectionHand) # Connection Hand activate control 
except:
    print("Hand Serial disconnect please connect the hand serial")
try:
    connectionIO = SerialManager('/dev/ttyACM2',115200) # 
except: 


def Kinematic_Calpose(x,y,z,AngleS,AngleE,AngleW,AngleBarrett): 


def Base_Calculation(x,y):      # 1  
   AngleB = math.atan(y/x)
   return math.degrees(AngleB) # Return the function of angle base calculation
def  distantCalculation(x,y):   # 2D distance calculation for the distant comparation function 
     D2d = math.sqrt(math.pow(s,2) + math.pow(b,2))  # 2D distance comparation function 
def OutputdistantAngleInput(s,w,AngleE,AngleS2): #3 
    d = math.sqrt(math.pow()+math.pow() - 2*s*w*math.cos(math.radians(AngleE))) # dynamic change on the distance calculation function 
    dy = d*math.cos(math.radians(AngleS2)) 
    return dy   # otuput for the dy calculation function on the 2d distant comparation function 
def ZLenghtCal(AngleS):  #4 

def WristAngleCal(AngleW): 

def  AngleS_Orientation(z,dy):   
     
     return AngleS
   
while True: 
      
       