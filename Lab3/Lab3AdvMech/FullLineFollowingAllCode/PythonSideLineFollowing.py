#!/usr/bin/env python3
import serial
import time
import numpy as np
from sendStringScript import sendString
leftMotor=int(100)
rightMotor=int(100)
# packetDropped = False

intersection_counter = 0;
counter = 0;
yprev = 0;
"""
at_intersection = False;
rotationStateRight = False
followingState = False;
rotationStateLeft = False;
pattern = ["forward", "forward", "turn-right", "forward", "turn-left"]
# Idea of Implementation
# We have certain states that are running that set the motor power (such as forward moves both mtoros forward)
# turn right turns right
# when we are off on the line following (too right or too left) - we apply correction on top of the states to go back onto the line
# the correction should only apply at straight line following
# when it detects the intersection it switches to the next state in the pattern
   # in this part we need to implement a system when it doesn't constantly switch state when at intersection 
"""

def count():
    global counter, yprev, y
    if y==1 and yprev!=1:
        counter +=1
        yprev=y
        #print(counter) 
    else :
        yprev=y

        return counter

def assignString(x,y,intersection_counter):
    if y==0 and x>=1000:
        state="following"
    elif y==1 and intersection_counter==1:
        state="following"
    elif y==1 and intersection_counter==2:
        state="turn_right"
    elif y==1 and intersection_counter==3:
        state="turn_left"
    else:
        state="INDETERMINATE"

    print(state)
    return(state)
            
if __name__ == '__main__':
    ser=serial.Serial('/dev/ttyACM0',9600)
    ser.reset_input_buffer() #clears anything the arduino has been sending while the Rpi isnt prepared to recieve.

    while True:
        sendString('/dev/ttyACM0',9600,'<'+str(leftMotor)+','+str(rightMotor)+'>',0.0001)
        
        if ser.in_waiting > 0:  #we wait until the arduino has sent something to us before we try to read anything from the serial port.
                 
                line = ser.readline().decode('utf-8')
                line=line.split(',')
                #print(line)
                #this splits the incoming string up by commas
                try:                  
                    x=int(line[0])
                    y=int(line[1])
                    l=int(line[2]) #we dont convert this to a float becasue we went to be able to recieve the message that we are at a cross, which wont be an int. 
                    r=int(line[3])
                    print([x,y,l,r])
                    intersection_counter = count()  
                except:
                    # yprev = y;
                    print("packet dropped") #this is designed to catch when python shoves bits on top of each other. 

                #Following is my control law, we're keeping it basic for now, writing good control law is your job
                #ok so high numbers(highest 7000) on the line follwing mean I am too far to the LEFT,
                #low numbers mean I am too far on the RIGHT, 3500 means I am at the middle
                #below is a basic control law you can send to your motors, with an exeption if z is a value greater than 7000, meaning the arduino code sees that the line sensor is on a cross. Feel free to take insperation from this,
            #but you will need to impliment a state machine similar to what you made in lab 2 (including a way of counting time without blocking)
                print("Y: ",y)
                print("Interseciton Counter: ",intersection_counter)
                print("yprev: ",yprev)
                situation = assignString(x,y,intersection_counter)
                match situation:
                    case "following":
                        print("inside following state")
                        rightMotor=0.4*(100+.02*x)  
                        leftMotor=0.4*(250-.02*x)
                    case "turn_right":
                        print("inside turn right state")
                        leftMotor=125  
                        rightMotor=0
                        time.sleep(.9)
                    case "turn_left":
                        print("inside turn left state")
                        leftMotor=0  
                        rightMotor=140
                        time.sleep(.7)
                    case "INDETERMINATE":
                        print("inside indeterminate state")
                        leftMotor, rightMotor = (0, 0);
                """ 
                if int(y) == 0: #im assuming that in your arduino code you will be setting z to the int 8000 if you sense a cross, dont feel obligated to do it this way.  
                    leftMotor=100+.02*l #now that we are SURE that z isnt the string cross, we cast z to an int and recalculate leftMotor and rightMotor, 
                    rightMotor=250-.02*r
                    print('not at intersection')
                else:
                    print('at intersection')
                    #do something here like incrimenting a value you call 'lines_hit' to one higher, and writing code to make sure that some time (1 second should do it) 
                    # passes between being able to incriment lines_hit so that it wont be incrimented a bunch of times when you hit your first cross. IE give your robot time to leave a cross
                    #before allowing lines_hit to be incrimented again.
                """ 