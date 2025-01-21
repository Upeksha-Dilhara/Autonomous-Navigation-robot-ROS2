#include <VL6180X.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <SoftwareSerial.h>
#include "RoboClaw.h"
#include <ros.h>
#include <std_msgs/Int8.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>

// ROS
ros::NodeHandle nh;

std_msgs::Int8 irMsg;
ros::Publisher irPub("/ir", &irMsg);
std_msgs::UInt8 left_distance_msg;
ros::Publisher leftDisPub("left_distance", &left_distance_msg);
std_msgs::UInt8 right_distance_msg;
ros::Publisher rightDisPub("right_distance", &right_distance_msg);
std_msgs::Float32 imuMsg;
ros::Publisher imuPub("imu", &imuMsg);


//See limitations of Arduino SoftwareSerial
SoftwareSerial serial(10,11);	
//RoboClaw Motor Controller
RoboClaw roboclaw(&serial,10000);  
#define address 0x80

// Time of Flight address and enable pins
#define LOX1_ADDRESS 0x30
#define LOX2_ADDRESS 0x31

#define lox_sensor1_en 4
#define lox_sensor2_en 5

#define ENABLE 1
#define DISABLE 0

// Motor A connections
#define enA 30
#define in1 6
#define in2 7

// Motor B connections
#define enB 31
#define in3 8
#define in4 9

#define encoderPinMotorA 3
#define encoderPinMotorB 2

#define ForwardMotor 'F'
#define BackwardMotor 'B'
#define stopMotor 'S'

char LeftReadTemp = 'N';
char RightReadTemp = 'N';
char CenterReadTemp ='N';

char readRight ='.';
char readLeft ='.';
char readCenter ='.';

int ir_read_position = 10; // IR sensor reading

uint8_t motorAEncoderRead;
uint8_t motorBEncoderRead;

uint8_t encoderAState = 0;
uint8_t encoderBState = 0;

int encoderCountMotorA = 0;
int encoderCountMotorB = 0;

VL6180X lox_sensor1 = VL6180X(); // Left Distance sensor
VL6180X lox_sensor2 = VL6180X(); // Right Distance sensor
static uint16_t data[2];
int paraller_observer = 0;
uint16_t *read_Left_Right_Loxdata;

Adafruit_BNO055 oriantionBN = Adafruit_BNO055(55); // Oriantion Sensor
sensors_event_t event;  // Oriantion Sensor event
float oriantion_X_Current_read =0.0;

bool enableTurnFunction = false;
bool enableCenterTask = false;

bool run = true;
bool turned = false;

void turnAround(){
  int imu_start = oriantion_X_Aixs();
  int start_time = millis();

  int imu_current;
  while (true){
    imu_current = oriantion_X_Aixs();
    if ((millis()-start_time)>2000){
      if (abs(imu_start-imu_current)>5){
        roboclaw.ForwardM1(address,25);
        roboclaw.BackwardM2(address,25);
      }
      else{
        roboclaw.ForwardM1(address,0);
        roboclaw.BackwardM2(address,0);
        turned = true;
        break;
      }
    }
  }
}

void turnLeft(int angle){
  int imu_start = oriantion_X_Aixs();
  // Serial.print("start value: ");
  // Serial.println(imu_start);

  int imu_current;
  while (true){
    //Serial.println(imu_current);
    int imu_current = oriantion_X_Aixs();
    if (abs(imu_start-imu_current)<angle || (360-abs(imu_start-imu_current)<angle)){
        roboclaw.ForwardM1(address,25);
        roboclaw.BackwardM2(address,25);
    }
    else{
        roboclaw.ForwardM1(address,0);
        roboclaw.BackwardM2(address,0);
        turned = true;
        break;      
    }
  }
}

void turnRight(int angle){
  int imu_start = oriantion_X_Aixs();
  // Serial.print("start value: ");
  // Serial.println(imu_start);

  int imu_current;
  while (true){
    //Serial.println(imu_current);
    int imu_current = oriantion_X_Aixs();
    if (abs(imu_start-imu_current)<angle || (360-abs(imu_start-imu_current)<angle)){
        roboclaw.ForwardM2(address,25);
        roboclaw.BackwardM1(address,25);
    }
    else{
        roboclaw.ForwardM2(address,0);
        roboclaw.BackwardM1(address,0);
        turned = true;
        break;      
    }
  }
}

double degreeCalculation(double readDegree , double deltaDegree , char operators){ // operator '+' , '-'  readDegree is before calcution of will need value
  if(operators == '-'){
    if(deltaDegree <=  readDegree){
        return readDegree - deltaDegree;
    }else if(readDegree < deltaDegree){
        return (360.0 + (readDegree - deltaDegree));
    }
  }else if(operators == '+'){
    if(readDegree <= (360.0 - deltaDegree)){
        return readDegree + deltaDegree;
    }else if( (360.0 - deltaDegree) < readDegree){
        return (deltaDegree -(360.0 - readDegree ));
    }
  }
}

void motor_Init(){
  // Set all the motor control pins to outputs
	pinMode(enA, OUTPUT);
	pinMode(enB, OUTPUT);
	pinMode(in1, OUTPUT);
	pinMode(in2, OUTPUT);
	pinMode(in3, OUTPUT);
	pinMode(in4, OUTPUT);
	
	// Turn off motors - Initial state
	digitalWrite(in1, LOW);
	digitalWrite(in2, LOW);
	digitalWrite(in3, LOW);
	digitalWrite(in4, LOW);
  // Motor Encoder read pins
  pinMode(encoderPinMotorA, INPUT_PULLUP);
	pinMode(encoderPinMotorB, INPUT_PULLUP);
  //RoboClaw Motor Controller
  roboclaw.begin(38400);
}
void oriantion_Init(){
  /* Initialise the sensor */
  if(!oriantionBN.begin()){
    /* There was a problem detecting the BNO055 ... check your connections */
    // Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
  }
  delay(1000);
  oriantionBN.setExtCrystalUse(true);
  
}

void lox_sensors_Init(){
  // time of Flight Initial state and Address set
  pinMode(lox_sensor1_en, OUTPUT);
  pinMode(lox_sensor2_en, OUTPUT);
  digitalWrite(lox_sensor1_en, LOW);
  digitalWrite(lox_sensor2_en, LOW);

  digitalWrite(lox_sensor1_en, HIGH);
  delay(50);
  lox_sensor1.init();
  lox_sensor1.configureDefault();
  lox_sensor1.setTimeout(500);
  lox_sensor1.setAddress(0x54);
  

  digitalWrite(lox_sensor2_en, HIGH);
  delay(50);
  lox_sensor2.init();
  lox_sensor2.configureDefault();
  lox_sensor2.setTimeout(500);
  lox_sensor2.setAddress(0x56);
}


void setup() {
  Serial.begin(57600);
  Serial1.begin(1200);
  Serial2.begin(1200);
  Serial3.begin(1200);
  Wire.begin();
  /* Time of Flight Distance IR sensor Initilize*/
  lox_sensors_Init();
  /* Oriantion Sensor*/
  oriantion_Init();
  /* Motors Initilizei*/
  motor_Init();
  
  //sensors_Run();
  enableTurnFunction = true;
  enableCenterTask = true;

  // Initialize ROS publishers
  nh.initNode();
  nh.getHardware()->setBaud(57600);
  nh.advertise(irPub);
  nh.advertise(leftDisPub);
  nh.advertise(rightDisPub);
  nh.advertise(imuPub);
  
  
}
void loop() {

  if (run){  
    sensors_Run(); 
    // if (ir_read_position!=10) Serial.println(ir_read_position);
    // // Serial.print(data[0]);
    // Serial.println("loop");
    // // Serial.println(data[1]);
    // roboclaw.ForwardM1(address,25);
    // roboclaw.ForwardM2(address,25);
    //IR_Task_Process();

  }
  else{
    roboclaw.ForwardM1(address,0);
    roboclaw.ForwardM2(address,0);
  
  //if (ir_read_position != 10){
    
  //}
  }

  // spin the ROS node
  nh.spinOnce();

}

void freeDrive(char mode, int encoderCount=0, char direction='F'){   // mode - encoders/distance
  encoderCountMotorA = 0;
  encoderCountMotorB = 0;
  while (true){
    if (mode == 'E'){ 
      if (data[0]<80 || data[1]<80){
        freeDrive('D');
        break;
      }
      if (direction == 'F'){
        roboclaw.BackwardM1(address,25);
        roboclaw.BackwardM2(address,25);
      }
      else{
        roboclaw.ForwardM1(address,25);
        roboclaw.ForwardM2(address,25);    
      }
      encoderCountMotorA = encoderCountM1(true, 'F');
      encoderCountMotorB = encoderCountM2(true, 'F');
      if (abs(encoderCountMotorA) > encoderCount || abs(encoderCountMotorB) > encoderCount){
        break;
      }
    }
    else if (mode == 'D'){
      Serial.println("distance mode");
      lox_Distance_sensorRead();
      if (data[0]==255 && data[1]==255){
        break;
      }
      else{
        roboclaw.BackwardM1(address,25);
        roboclaw.BackwardM2(address,25);        
      }
    }
    delayMicroseconds(100);
  }
}

void setCenter(){
  //run = false;
  //Serial.print("ir : ");
  //Serial.println(ir_read_position);
  stopMotors();  
  //Serial.println("motors stopped");
  delay(2000);
  
  // Serial.println("Moving");
  // freeDrive('D');
  // stopMotors();
  // Serial.println("motors stopped");
  // delay(2000);

  // Serial.println("turning left");
  // int imu_start = oriantion_X_Aixs();
  // turnRight(30);
  // delay(2000);

  // Serial.println("Moving");
  // freeDrive('E', 200, 'F');
  // stopMotors();
  // Serial.println("motors stopped");
  // delay(2000);

  // Serial.println("turning right");
  // turnLeft(30);
  delay(2000);
}

void IR_Task_Process(){
  if(ir_read_position!=10){
    
    if(ir_read_position == 0){
      robotTaskStepCenter();
    }
    else if(ir_read_position == 1){
      //setCenter();
    }else if(ir_read_position == 2){
      //setCenter();
    }else if(ir_read_position == 3){
      //setCenter();
    }else if(ir_read_position == 4){
      //setCenter();
    }else if(ir_read_position == 5){

    }else if(ir_read_position == 6){

    }else if(ir_read_position == 7){

    }else if(ir_read_position == -1){

    }else if(ir_read_position == -2){

    }else if(ir_read_position == -3){

    }else if(ir_read_position == -4){

    }else if(ir_read_position == -5){

    }else if(ir_read_position == -6){

    }else if(ir_read_position == -7){

    }
    
  }
}

void robotTaskStriptOne(bool enable){
  if(data[0]>255 && data[1]>255){
    while(encoderCountM1(true,'B')<10 && encoderCountM2(true,'B') <10){
        roboclaw.BackwardM2(address,25);
        roboclaw.BackwardM1(address,25);
    }
    roboclaw.BackwardM2(address,0);
    roboclaw.BackwardM1(address,0);
    delay(400);

  }
}
void turnLeftRight(char turnDirection,double oriantion_X_start, double degree){
  if(enableTurnFunction){
      double oriantion_Temp = oriantion_X_start;
      if(turnDirection == 'L'){
        double position_degree = degreeCalculation(oriantion_Temp,degree,'-');
        while(position_degree < oriantion_Temp){
          oriantion_Temp = oriantion_X_Aixs();
          roboclaw.ForwardM1(address,20);
          roboclaw.BackwardM2(address,20);
        }

        roboclaw.ForwardM1(address,0);
        roboclaw.BackwardM2(address,0);
        delay(2000);
        
      }else if(turnDirection == 'R'){
      roboclaw.ForwardM2(address,20);
      roboclaw.BackwardM1(address,20);
      }
  }
}
void robotTaskStepCenter(){
    if(enableCenterTask){
      if(data[0]<255||data[1]<255){
            

          while((data[0] != data[1]) || ((data[0]-data[1]) > 20) || ((data[1]-data[0]) > 20) ){ // not parallel
              distance_Sensor_sub_task(); // Check sensor data avalability

            if(data[0] > data[1]){
                  roboclaw.ForwardM2(address,25);
                  roboclaw.ForwardM1(address,0);
                }else if(data[1] > data[0]){
                  roboclaw.ForwardM2(address,0);
                  roboclaw.ForwardM1(address,25);
                }
            }
            // is parallel
              roboclaw.ForwardM1(address,0); // stop forward
              roboclaw.ForwardM2(address,0); // stop forward
              if(data[0]<50 && data[1]<50){
                enableCenterTask = false;
              }
          }else{
            roboclaw.ForwardM1(address,25); // go forward
            roboclaw.ForwardM2(address,25); // go forward
          }
    }
}

void sensors_Run(){
  ir_read_position = 10; // null
  longLRSensor(); //Read Sensor reading
  ir_read_position = IRLogicProcess(LeftReadTemp,RightReadTemp,CenterReadTemp); // Process Left and Right Sensor Logic

  // Serial.println(ir_read_position);
  read_Left_Right_Loxdata = lox_Distance_sensorRead(); // Distance sensor process

  imuMsg.data = oriantion_X_Aixs(); // oriantion sensor process
  irMsg.data = ir_read_position;
  left_distance_msg.data = data[0];
  right_distance_msg.data = data[1];

  // Publish ROS messages
  imuPub.publish(&imuMsg);  
  irPub.publish(&irMsg);
  leftDisPub.publish(&left_distance_msg);
  rightDisPub.publish(&right_distance_msg);
}

void distance_Sensor_sub_task(){
  read_Left_Right_Loxdata = lox_Distance_sensorRead(); // Distance sensor process
}
double oriantion_X_Aixs(){
  oriantionBN.getEvent(&event);
  oriantion_X_Current_read = event.orientation.x;
  return oriantion_X_Current_read;
}
void longLRSensor(){
      if(Serial2.available()>0 ){
        readRight = char(Serial2.read());
        if(readRight == 'R'|| readRight =='L' || readRight == 'C' || readRight == 'P' || readRight == 'Q'|| readRight == 'X' || readRight == 'Y'){
          RightReadTemp = readRight;
        }else{
          readRight ='.';
          RightReadTemp ='N';
        }  
      }

      if(Serial1.available()>0 ){
        readLeft = char(Serial1.read());
        if(readLeft == 'R'|| readLeft =='L' || readLeft == 'C' || readLeft == 'P' || readLeft == 'Q'|| readLeft == 'X' || readLeft == 'Y'){
          LeftReadTemp = readLeft;
        }else{
          readLeft ='.';
          LeftReadTemp ='N';
        }  
      }

      if(Serial3.available()>0 ){   
        readCenter = char(Serial3.read());
        if(readCenter == 'R'|| readCenter =='L' || readCenter == 'C' || readCenter == 'P' || readCenter == 'Q'|| readCenter == 'X' || readCenter == 'Y'){
          CenterReadTemp = readCenter;
        }else{
          readCenter ='.';
          CenterReadTemp ='N';
        }
      }
}

int IRLogicProcess(char left , char right ,char center){

  if((center == 'C' && right == 'R' && left == 'L') ||(center == 'C' && right == 'N' && left == 'N')||(center == 'C' && right == 'C' && left == 'C')){
    return 0;
  }else if(left == 'C' && right == 'R'){
    return -1;       
  }else if((center == 'R' && right == 'Q' && left == 'C')||(center == 'R' && right == 'N' && left == 'N')||(center == 'R' && right == 'R' && left == 'R')){
    return -2;
  }else if(left == 'R' && right == 'Q'){
    return -3;       
  }else if((center == 'Q' && right == 'Y' && left == 'R')||(center == 'Q' && right == 'N' && left == 'N')||(center == 'Q' && right == 'Q' && left == 'Q')){
    return -4;
  }else if(left == 'Q' && right == 'Y'){
    return -5;       
  }else if((center == 'Y' && right == 'N' && left == 'Q')||(center == 'Y' && right == 'N' && left == 'N')||(center == 'Y' && right == 'Y' && left == 'Y')){
    return -6;
  }else if(center == 'N' && left == 'Y' && right == 'N'){
    return -7;       
  }else if(left == 'L' && right == 'C'){
    return 1;       
  }else if((center == 'L' && right == 'C' && left == 'P')||(center == 'L' && right == 'N' && left == 'N')||(center == 'L' && right == 'L' && left == 'L')){
    return 2;
  }else if(left == 'P' && right == 'L'){
    return 3;       
  }else if((center == 'P' && right == 'L' && left == 'X')||(center == 'P' && right == 'N' && left == 'N')||(center == 'P' && right == 'P' && left == 'P')){
    return 4;
  }else if(left == 'X' && right == 'P'){
    return 5;       
  }else if((center == 'X' && right == 'P' && left == 'N')||(center == 'X' && right == 'N' && left == 'N')||(center == 'X' && right == 'X' && left == 'X')){
    return 6;
  }else if(center == 'N' && left == 'N' && right == 'X'){
    return 7;       
  }else{
    return 10;
  }
}

uint16_t * lox_Distance_sensorRead(){
  data[0] = lox_sensor1.readRangeSingleMillimeters();
  data[1] = lox_sensor2.readRangeSingleMillimeters();
  if(data[0] > 250 && data[1] > 250){
    paraller_observer = (data[0] + data[1]);
  }else{
    paraller_observer = ((data[0] - data[1]));
  }
  if (lox_sensor1.timeoutOccurred() || lox_sensor2.timeoutOccurred()) { /*Serial.print(" TIMEOUT");*/ }
  return (data);
}

void stopMotors(){
  roboclaw.ForwardM1(address,0);
  roboclaw.ForwardM2(address,0);
}

void controllMotorA(uint16_t speed , char direction){  // 0 to 255 speed
  analogWrite(enA, speed);
  switch(direction){
    case 'F':digitalWrite(in1, LOW);
           digitalWrite(in2, HIGH);
     break;
    case 'B':digitalWrite(in1, HIGH);
	         digitalWrite(in2, LOW);
            break;
    case 'S':digitalWrite(in1, LOW);
	         digitalWrite(in2, LOW);
  }
}

void controllMotorB(uint16_t speed , char direction){ // 0 to 255 speed
  analogWrite(enB, speed);
  switch(direction){
    case 'F':digitalWrite(in3, LOW);
           digitalWrite(in4, HIGH);
      break;
    case 'B':digitalWrite(in3, HIGH);
	         digitalWrite(in4, LOW);
      break;
    case 'S':digitalWrite(in3, LOW);
	         digitalWrite(in4, LOW);       
  }
}

int encoderReadMotorA(){
  motorAEncoderRead = digitalRead(encoderPinMotorA);
  if(encoderAState != motorAEncoderRead){
      encoderAState = motorAEncoderRead;
      return 1; 
  }else{
      return 0;
  }
}

int encoderReadMotorB(){
  motorBEncoderRead = digitalRead(encoderPinMotorB);
  if(encoderBState != motorBEncoderRead){
      encoderBState = motorBEncoderRead;
      return 1; 
  }else{
      return 0;
  }
}

int encoderCountM1(bool enable, char direction){
  if(enable){
    if(ENABLE == encoderReadMotorA() && direction == 'F'){encoderCountMotorA++;}
    if(ENABLE == encoderReadMotorA() && direction == 'B'){encoderCountMotorA--;}
  }else{
      encoderCountMotorA = 0;
  }
  return encoderCountMotorA;
}

int encoderCountM2(bool enable, char direction){
  if(enable){
    if(ENABLE == encoderReadMotorB() && direction == 'F'){encoderCountMotorB++;}
    if(ENABLE == encoderReadMotorB() && direction == 'B'){encoderCountMotorB--;}
  }else{
      encoderCountMotorB = 0;
  }
  return encoderCountMotorB;
}


