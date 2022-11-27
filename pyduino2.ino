/*
 * Author: Terrance Williams
 * Date: 23 November 2022
 * Description: A program that controls the Lynxmotion Control
 * 3DoF Robot while interacting with Python Serial Communication.
 */

#include <LSS.h>

// Notes for the User:
/*
 * 2IO LED codes : 0=Off (black); 
 * 1=Red 2=Green; 3=Blue; 4=Yellow; 5=Cyan; 6=Magenta; 7=White; 
 * Change color via (SoftwareSerial_object).write("#207LED<COLORCODE>\r")
 * Leave the "<>" out when transmitting.
 */

 /*
  * LSS ANGLE RANGES (in tenths of a degree)
  * 
  * Servo 1: -850 to 1250
  * Servo 2: -750 to 200
  * Servo 3: -450 to 600
  * 
  */


// Globals
#define ARRAYSIZE (3)
#define SERIAL_DELAY (100)
  // Servo variables
#define GRIPPER_ID (209)
#define GRIPPER_OPEN (450) //Just for visual use. MANUAL SEND COMMAND: #209D450
#define GRIPPER_CLOSED (1500) //MANUAL COMMAND: (SoftwareSerial_object).write("#209D1800\r")
//#define GRIPPER_LIMP "#209L\r"

  // LSS Variables
#define LSS_ID1    (1)
#define LSS_ID2    (2)
#define LSS_ID3    (3)
#define LSS_BAUD  (LSS_DefaultBaud)  //115200
#define MAX_ANGLE_DIFF (100)
#define SLOW_MOVE_RATE (2) //ms per tenth of a degree. Ex. moving 500 units should take 1000 ms

// Create Objects
SoftwareSerial LSS_Serial(8, 9);  //makes arduino pins 8 and 9 the serial Rx and Tx for the LSS bus.
LSS myLSS1 = LSS(LSS_ID1); // Servos on the Lynxmotion Robot Arm (ID1 = Base rotation)
LSS myLSS2 = LSS(LSS_ID2); // -angle values move end effector down (CCW servo rotation); wider range of motion (-750 to 200)
LSS myLSS3 = LSS(LSS_ID3); // -angle values move end effector back (contraction)
LSS gripper = LSS(GRIPPER_ID); //interact with gripper using LSS Protocol.

/* declare an array populated with dummy values for positions
*FORM: [[RED], [YELLOW], [GREEN],[BLUE]]
* For example, if positions = [[P1],[P2],[HOME],[P3]],
* P1 is associated with red, P2:yellow, P3:Blue, 
* and HOME:Green
*/
int positions[4][3] = {{10, 10, 10},{11,11,11},{12,12,12},{13,13,13}}; 
// Define Positions
//int RESET[ARRAYSIZE] = {0, 0, 0}; // Robot reset config
int pos_1[ARRAYSIZE] = {-404, -390, 275};//(16, 13.6, 5.5) mag 21cm // (17, 8.49, 5.5) = {-265, -416, 208} angles (x, y mag is 19)
int pos_2[ARRAYSIZE] = {-205, -274, 405};  //(24, 9, 7)cm
int pos_3[ARRAYSIZE] = {0, -416, 208}; //(19, 0, 5.5)cm
int HOME[ARRAYSIZE] =  {337,-442, 191};// (15, -10, 5.5)cm Object home position
int target_color = 10; // destination
int current_color = 2; // current object pos (default to GREEN)

// Variables for Python Communication 
bool readyForInput = true;
int incomingByte = -2; // a variable to read incoming serial data
int fill_cnt = 0; 
char dumBuffer[8];

void setup() {

  /* Lynxmotion Setup
   *  1. Start Serial Bus (LSS::initBus(SoftwareSerial object, Baud Rate)
   *  2. Initialize Gripper
   *  3. Move to RESET position + Gripper close
  */
  delay(5000);
  
  LSS::initBus(LSS_Serial, LSS_BAUD);
  delay(500);
  LSS_Serial.write("#207LED5\r");  //2IO should now be cyan.
  delay(50);
    
  delay(3000);
  resetLynx();
  gripper.move(GRIPPER_CLOSED);
  delay(250);
  gripper.limp();
  delay(50);
  LSS_Serial.write("#207LED0\r");

  // ESTABLISH Python Comms
  Serial.begin(19200); // Python Communication Baudrate
  delay(30); // give time to connect with python
  while (!Serial){
    ;
  }
  Serial.print("LYNX: Associate Colors.\n");
  delay(SERIAL_DELAY*10);
  Serial.print('I'); // tell Python we're ready
 
  // populate positions array: WHAT COLOR is associated with the position?
  while(fill_cnt != 4){
    
    // listen to COM port for data
    if(Serial.available()>0){
      
      incomingByte = Serial.read();
      // get rid of excess characters
      if(Serial.peek() == '\r'){
        Serial.readBytes(dumBuffer, 2);
      }
      else if(Serial.peek() == '\n'){
        Serial.readBytes(dumBuffer, 1); 
      }
      delay(SERIAL_DELAY);
      
      // Associate Positions and Colors
      if(incomingByte>='0' and incomingByte<='3'){
        incomingByte = cvtASCII(incomingByte);
        if(fill_cnt == 0) // Set position 1's color
        {
          populateArray(positions, pos_1, incomingByte);
          Serial.print('I');
          delay(SERIAL_DELAY);
        }
        if(fill_cnt == 1){
          populateArray(positions, pos_2, incomingByte);
          Serial.print('I');
          delay(SERIAL_DELAY); 
        }
        if(fill_cnt == 2){
          populateArray(positions, pos_3, incomingByte);
          Serial.print('I');
          delay(SERIAL_DELAY);
        }
        if(fill_cnt == 3){
          populateArray(positions, HOME, incomingByte);
          Serial.print('I');
          delay(SERIAL_DELAY); 
        }
        fill_cnt++;
        incomingByte = -2; // reset the byte
      }
    }   
  }
}

// the loop function runs over and over again forever
void loop() {
  // listen to COM port for data
  if(Serial.available()>0){
    
    delay(SERIAL_DELAY); 
    
    // get the data
    incomingByte = Serial.read();
    // remove excess data from buffer
    if(Serial.peek() == '\r'){
       Serial.readBytes(dumBuffer, 2);
      }
    else if(Serial.peek() == '\n'){
      Serial.readBytes(dumBuffer, 1);
    }
    delay(SERIAL_DELAY);

    // treat the data
    if(incomingByte>='0' and incomingByte<='3') // Did we get a color?
    {
      //getPos(current_pos);
      target_color = cvtASCII(incomingByte);

      //Pick & Place operation
      if(target_color != current_color)
      {
        moveArm(positions[target_color]);  //PICK&PLACE;
        current_color = target_color;
      }
      else
      {
        ;
      }      
      // variable reset
      target_color = 10;  
      Serial.print("I"); // Open to comms
      delay(SERIAL_DELAY);
    }
    else if(incomingByte != 'q') // Not a target color but also not the 'QUIT' signal; Tell py to send new data.
    {
      Serial.print("I");
      delay(SERIAL_DELAY); 
    }
    else // RECEIVED 'QUIT' SIGNAL FROM PYTHON
    { 
      Serial.print("9");
      delay(SERIAL_DELAY);
      
        //MAKE A MOVE TO RESET POSITION + GO LIMP; PROGRAM IS DONE.
       
    }
    // reset variables
    incomingByte = -1;
  }   
}

// HELPER FUNCTIONS 
/*
 * int cvtASCII (int input_character):
 * Translates python data ['0','3'] to the integers [0,3].
 * Used to iterate position arrays.
 */
int cvtASCII(int character){
  if(character == '0'){
    return 0;
  }
  if(character == '1'){
    return 1;
  }
  if(character == '2'){
    return 2;
  }
  if(character == '3'){
    return 3;
  }
  else{
    return -1;
  }
}


/*
 * void populateArray
 * 
 * Inputs the array meant to store the servo angles of the four positions, 
 * the position array to copy, 
 * and the color position to copy it to.
 * 
 * For example, void(positions, pos_1, 3) will result in position 1 being associated with the color BLUE such that:
 * positions = [[], [], [], [pos_1]] = [[RED],[YELLOW],[GREEN],[BLUE]]
 */

void populateArray(int master[4][ARRAYSIZE], int entry[ARRAYSIZE], int place){
  for(int i=0; i<ARRAYSIZE; i++){
    master[place][i] = entry[i];
  }
}
/*
* void moveArm
* Performs the pick and place operation
*/
void moveArm(int target[3]){
  // wake up LSSs
  resetLynx();
  delay(500);
  // servo angles of target position
  int obj_target1 = target[0];
  int obj_target2 = target[1];
  int obj_target3 = target[2];
  // servo angles of current object position
  
  int obj_curr1 = positions[current_color][0];
  int obj_curr2 = positions[current_color][1];
  int obj_curr3 = positions[current_color][2];
  
  
  //current servo angles
  int32_t currS1 = myLSS1.getPosition();
  delay(SERIAL_DELAY);
  int32_t currS2 = myLSS2.getPosition();
  delay(SERIAL_DELAY);
  int32_t currS3 = myLSS3.getPosition();
  delay(SERIAL_DELAY);

 /* PICK & PLACE
  *
  * MOVE ARM TO OBJECT (Open Gripper ,Move S1, Move S3, MV S2, Close Gripper) 
  * INTERMEDIATE MOVEMENT (Move S2, Move S3): Basically, give the bot room to move
  * MOVE ARM TO TARGET (Move S1, Move S3, Move S2)
  * Release (Open Gripper (slowly))
  * Intermediate Move 
  * MOVE TO RESET
  * GRIPPER LIMP
  */
  LSS_Serial.write("#207LED6\r");  //turn on 2IO LED to indicate movement
  delay(SERIAL_DELAY);
  gripper.move(GRIPPER_OPEN);
  // move to object
    // Servo1
  if(abs(obj_curr1 - currS1) >= MAX_ANGLE_DIFF + 100){
    myLSS1.moveT(obj_curr1, SLOW_MOVE_RATE*abs(obj_curr1 - currS1));
    delay((SLOW_MOVE_RATE*abs(obj_curr1 - currS1))+150);
  }
  else{
    myLSS1.move(obj_curr1);
    delay(500);
  }
    // Servo 3
  if(abs(obj_curr3 - currS3) >= MAX_ANGLE_DIFF){
    myLSS3.moveT(obj_curr3, 3*abs(obj_curr3 - currS3)/2);
    delay((SLOW_MOVE_RATE*abs(obj_curr3 - currS3))+150);
  }
  else{
    myLSS3.move(obj_curr3);
    delay(500);
  }
    // Servo 2
  if(abs(obj_curr2 - currS2) >= MAX_ANGLE_DIFF){
    myLSS2.moveT(obj_curr2, SLOW_MOVE_RATE*abs(obj_curr2 - currS2));
    delay(2*(SLOW_MOVE_RATE*abs(obj_curr2 - currS2)));
  }
  else{
    myLSS2.move(obj_curr2);
    delay(500);
  }
  //grab object
  gripper.move(GRIPPER_CLOSED);
  delay(250);
  // move to intermediate position
  if(abs(obj_curr2) >= MAX_ANGLE_DIFF){
    myLSS2.moveT(0, SLOW_MOVE_RATE*abs(0-obj_curr2));
    delay(2*(SLOW_MOVE_RATE*abs(0-obj_curr2)));
  }
  else{
    myLSS2.move(0);
  }
  if(abs(obj_curr3) >= MAX_ANGLE_DIFF){
    myLSS3.moveT(0, SLOW_MOVE_RATE*abs(0-obj_curr3));
    delay((SLOW_MOVE_RATE*abs(0-obj_curr3))+150);
  }
  else{
    myLSS3.move(0);
  }
  // move to Target (S1->S3->S2)
    // Servo 1
  if (abs(obj_target1 - obj_curr1) >= MAX_ANGLE_DIFF){
    myLSS1.moveT(obj_target1, SLOW_MOVE_RATE*abs(obj_target1 - obj_curr1));
    delay((SLOW_MOVE_RATE*abs(obj_target1 - obj_curr1))+150); 
  }
  else{
    myLSS1.move(obj_target1);
    delay(500);
  }
  // servo 3
  currS3 = myLSS3.getPosition();
  delay(200);
  if (abs(obj_target3-currS3) >= MAX_ANGLE_DIFF){
    delay(200);
    myLSS3.moveT(obj_target3, 3*abs(obj_target3-currS3)/2);
    delay(SLOW_MOVE_RATE*abs(obj_target3));
  }
  else{
    myLSS3.move(obj_target3);
    delay(500);
  }
  // servo 2 (set object down slowly)
  currS2 = myLSS2.getPosition();
  delay(200);
  myLSS2.moveT(obj_target2, SLOW_MOVE_RATE*abs(obj_target2-currS2));
  delay(2*SLOW_MOVE_RATE*abs(obj_target2));
  //
  gripper.move(GRIPPER_OPEN);
  delay(500);
  resetLynx();
  delay(250);
  gripper.move(GRIPPER_CLOSED);
  delay(200);
  // gripper limp
  gripper.limp();
  delay(200);
  LSS_Serial.write("#207LED0\r");
  delay(200);
  myLSS1.limp();
  myLSS2.limp();
  myLSS3.limp();
}

/*
 * Move Robot to RESET position (all servo angles 0) and close gripper.
 */
void resetLynx(){
  // get current servo positions & max angle
  int32_t currS1 = myLSS1.getPosition();
  delay(SERIAL_DELAY);
  int32_t currS2 = myLSS2.getPosition();
  delay(SERIAL_DELAY);
  int32_t currS3 = myLSS3.getPosition();
  delay(SERIAL_DELAY);
  currS1 = abs(currS1);
  currS2 = abs(currS2);
  currS3 = abs(currS3);
  
  // make moves  
  if (abs(currS2) >= MAX_ANGLE_DIFF){
    myLSS2.moveT(0, SLOW_MOVE_RATE*currS2);
    delay(2*SLOW_MOVE_RATE*currS2);
  }
  if (abs(currS3) >= MAX_ANGLE_DIFF){
    myLSS3.moveT(0, (3*currS3)/2);
    delay(2*SLOW_MOVE_RATE*currS3);
  }
  if (abs(currS1) >= MAX_ANGLE_DIFF){
    myLSS1.moveT(0, SLOW_MOVE_RATE*currS1);
    delay(2*SLOW_MOVE_RATE*currS1);
  }
  else{
    myLSS2.move(0);
    myLSS3.move(0);
    myLSS1.move(0);
    delay(1000);
  }
}
