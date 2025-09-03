#include <AStar32U4Motors.h>

AStar32U4Motors m; //read the documentation of this library to understand what functions to use to drive the motors and how to use them

int leftMotor;
int rightMotor;

const byte numChars = 32;
char receivedChars[numChars];
char tempChar[numChars]; // temporary array used for parsing

boolean newData = false;

int x=0;
int y=0;
int z=0;

int a=0;
int b=0;
int c=0;


//=====================================================

void setup() {
  Serial.begin(115200);
  pinMode(7, INPUT_PULLUP);
  pinMode(1, INPUT_PULLUP); //might have to reassign these to pins that the astar knows
  pinMode(0, INPUT_PULLUP);

  pinMode(5, INPUT_PULLUP);
  pinMode(4, INPUT_PULLUP);
  pinMode(8, INPUT_PULLUP);

}

//=====================================================

void loop() {


  recvWithStartEndMarkers();

  if (newData == true){
    
    strcpy(tempChar, receivedChars); //make a copy of recievedChars so we can make changes to it and not change recievedChars 
    parseData();  
    SendBumpData();
    
    //SendRecievedData(); //uncomment this (and make the neccicary changes to the Rpi code) to have the arduino send the Rpi back what it sent. Viewing the message the
                          //Rpi sends the arduino is important to make sure the Rpi isnt sending garbage
    newData = false;
    
  }

  
 // printBumpData(); //for testing the arduino code with the bump sensors on its own, comment this in to test the 
                   //bump sensors without doing any communication with the RPI
  
CommandMotors();

}

//============================================

void recvWithStartEndMarkers() {
//this function is the most important one of the whole lab, read the blog post made my Robin2
//some questions:
      //whats the purpose of the start and end markers?
      //Why bother making this code unblocking?
      //
    static boolean recvInProgress = false;
    static byte ndx = 0;
    char startMarker = '<';
    char endMarker = '>';
    char rc;
                                         
    while (Serial.available() > 0 && newData == false) {
        rc = Serial.read();
                                       

        if (recvInProgress == true) {
            if (rc != endMarker) {
                receivedChars[ndx] = rc;
                ndx++;
                if (ndx >= numChars) {
                    ndx = numChars - 1;
                }
            }
            else {
                receivedChars[ndx] = '\0'; // terminates the string, frankly unsure why I need 
                                           //this but it breaks if I remove it. Bonus points if you find out why
                recvInProgress = false;
                ndx = 0;
                newData = true;
            }
        }

        else if (rc == startMarker) {
            recvInProgress = true;
        }
    }
}

//============================================

void parseData(){ //this function takes a string and seperates it by the comma into 2 strings, then casts those strings to ints
  
strcpy(tempChar,receivedChars); //copying recievedChar into tempChar so we dont alter recievedChar

char *strIndexer = strtok(tempChar,",");

leftMotor = atoi(strIndexer);

strIndexer = strtok(NULL,",");

rightMotor = atoi(strIndexer);

}  

//============================================
void SendBumpData(){
  
  x=digitalRead(0);
  y=digitalRead(1);
  z=digitalRead(4);

  a=digitalRead(5);
  b=digitalRead(7);
  c=digitalRead(8);
  

  Serial.print(x);
  Serial.print(',');
  Serial.print(y);
  Serial.print(',');
  Serial.print(z);
  Serial.print(',');
  Serial.print(a);
  Serial.print(',');
  Serial.print(b);
  Serial.print(',');
  Serial.println(c);
}

//============================================

void CommandMotors(){  

  //read the documentation for the functions that drive the motors in the astar library

  m.setM1Speed(-rightMotor);
  m.setM2Speed(-leftMotor);
  //uncomment to drive motors
}







//===========================FUNCTIONS USED FOR TESTING, NOT NEEDED FOR LAB====================================





void SendRecievedData(){
  Serial.println(receivedChars);
}


void printBumpData(){
  
  x=digitalRead(0);
  y=digitalRead(1);
  z=digitalRead(4);

  a=digitalRead(5);
  b=digitalRead(7); //gonna have to set these also
  c=digitalRead(8);
  

  //Serial.print(receivedChars);
  Serial.print(x);
  Serial.print(',');
  Serial.print(y);
  Serial.print(',');
  Serial.print(z);
  Serial.print(',');
  Serial.print(a);
  Serial.print(',');
  Serial.print(b);
  Serial.print(',');
  Serial.println(c);
  delay(1000);
}
