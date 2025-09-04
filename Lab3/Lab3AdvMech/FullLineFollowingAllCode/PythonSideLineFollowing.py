#!/usr/bin/env python3
import serial
import time
import numpy as np
from sendStringScript import sendString
leftMotor=int(100)
rightMotor=int(100)

intersection_counter = 0;
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


if __name__ == '__main__':
    ser=serial.Serial('/dev/ttyACM0',115200)
    ser.reset_input_buffer() #clears anything the arduino has been sending while the Rpi isnt prepared to recieve.

    while True:
        sendString('/dev/ttyACM0',115200,'<'+str(leftMotor)+','+str(rightMotor)+'>',0.0001)
        
        if ser.in_waiting > 0:  #we wait until the arduino has sent something to us before we try to read anything from the serial port.
                 
                line = ser.readline().decode('utf-8')
                line=line.split(',')
                print(line)
                #this splits the incoming string up by commas
                try:
                    
                    x=int(line[0])
                    y=int(line[1])
                    z=int(line[2]) #we dont convert this to a float becasue we went to be able to recieve the message that we are at a cross, which wont be an int. 
                    print([x,y,z])
                except:
                    print("packet dropped") #this is designed to catch when python shoves bits on top of each other. 


            
            
                #Following is my control law, we're keeping it basic for now, writing good control law is your job
                #ok so high numbers(highest 7000) on the line follwing mean I am too far to the LEFT,
                #low numbers mean I am too far on the RIGHT, 3500 means I am at the middle
                #below is a basic control law you can send to your motors, with an exeption if z is a value greater than 7000, meaning the arduino code sees that the line sensor is on a cross. Feel free to take insperation from this,
            #but you will need to impliment a state machine similar to what you made in lab 2 (including a way of counting time without blocking)
            
                if int(y) == 0: #im assuming that in your arduino code you will be setting z to the int 8000 if you sense a cross, dont feel obligated to do it this way.  
                    leftMotor=100+.02*z #now that we are SURE that z isnt the string cross, we cast z to an int and recalculate leftMotor and rightMotor, 
                    rightMotor=250-.02*z
                    print('not at intersection')
                else:
                    print('at intersection')
                    #do something here like incrimenting a value you call 'lines_hit' to one higher, and writing code to make sure that some time (1 second should do it) 
                    # passes between being able to incriment lines_hit so that it wont be incrimented a bunch of times when you hit your first cross. IE give your robot time to leave a cross
                    #before allowing lines_hit to be incrimented again.
