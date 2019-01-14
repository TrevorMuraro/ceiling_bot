#include <ros.h>
#include <Arduino.h>
#include <stdio.h>

#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Vector3.h>

ros::NodeHandle nh;

char log_buffer[50];

#ifndef LED_BUILTIN
#define LED_BUILTIN 13
#endif

#define L_MOTOR_PWM_PIN 9
#define L_MOTOR_DIR_PIN 7
#define R_MOTOR_PWM_PIN 5
#define R_MOTOR_DIR_PIN 4
#define L_ENCODER_PIN 3
#define R_ENCODER_PIN 2

#define WHEELBASE 0.135 // in meters
#define WHEEL_CIRCUMFERENCE 0.2042 // in meters
#define ENCODER_TICKS_PER_ROTATION 20 // 20 holes in encoder disc, only counting rising edges
#define WHEEL_DISTANCE_PER_ENCODER_TICK 0.01021 // in meters

#define L_MOTOR_FWD_OR_STOP LOW
#define L_MOTOR_BKWD HIGH
#define R_MOTOR_FWD_OR_STOP LOW
#define R_MOTOR_BKWD HIGH
#define MAX_PWM 255 //Arduino analogWrite (PWM) always on is 255

#define ENCODER_DEBOUNCE_TIME 1500 // in us

#define LOOP_TICK 1000 // in ms






unsigned int l_motor_dir;
unsigned int r_motor_dir;




/***************************** Encoders ******************************************/

volatile unsigned long l_encoder_last_change_time;
volatile unsigned long r_encoder_last_change_time;
volatile float l_encoder_count;
volatile float r_encoder_count;
unsigned long l_encoder_timeOfLastPublish;
unsigned long r_encoder_timeOfLastPublish;

//x: Signed left ticks, y: Signed right ticks, z: Time elapsed
//geometry_msgs::Vector3 encoder_message;
//ros::Publisher encoder_pub("raw_encoder_data", &encoder_message);

std_msgs::Float32 leftEncoderMessage;
std_msgs::Float32 rightEncoderMessage;

ros::Publisher leftEncoderPub("left_encoder_vel", &leftEncoderMessage);
ros::Publisher rightEncoderPub("right_encoder_vel", &rightEncoderMessage);

void leftEncoderCallback() {
  unsigned long currentMicros = micros();
  if(currentMicros - l_encoder_last_change_time > ENCODER_DEBOUNCE_TIME) {
    l_encoder_count++;
    l_encoder_last_change_time = currentMicros;
  }
}

void rightEncoderCallback() {
  unsigned long currentMicros = micros();
  if(currentMicros - r_encoder_last_change_time > ENCODER_DEBOUNCE_TIME) {
    r_encoder_count++;
    r_encoder_last_change_time = currentMicros;
  }
}




void publishLeftEncoder() {
  unsigned long elapsedTime = (millis() - l_encoder_timeOfLastPublish);

  float leftEncoderDistance = (float) l_encoder_count * WHEEL_DISTANCE_PER_ENCODER_TICK;
  float leftEncoderVelocity = leftEncoderDistance / ((float) elapsedTime / 1000);

  l_encoder_count = 0;
  l_encoder_timeOfLastPublish = millis();

  //snprintf(log_buffer, sizeof(log_buffer) - 1, "Left wheel traveled %f meters", leftEncoderDistance);
  //nh.loginfo(log_buffer);

  if(l_motor_dir != L_MOTOR_FWD_OR_STOP) {
    leftEncoderVelocity = -leftEncoderVelocity;
  }

  leftEncoderMessage.data = leftEncoderVelocity;
  leftEncoderPub.publish(&leftEncoderMessage);
}

void publishRightEncoder() {
  unsigned long elapsedTime = (millis() - r_encoder_timeOfLastPublish);

  float rightEncoderDistance = (float) r_encoder_count * WHEEL_DISTANCE_PER_ENCODER_TICK;
  float rightEncoderVelocity = rightEncoderDistance / ((float) elapsedTime / 1000);


  r_encoder_count = 0;
  r_encoder_timeOfLastPublish = millis();

  if(r_motor_dir != R_MOTOR_FWD_OR_STOP) {
    rightEncoderVelocity = -rightEncoderVelocity;
  }

  rightEncoderMessage.data = rightEncoderVelocity;
  rightEncoderPub.publish(&rightEncoderMessage);
}

/*
  post: publishes encoder ticks and milliseconds
*/
/*
void publishEncoders() {
  encoder_message.x = l_encoder_count;
  encoder_message.y = r_encoder_count;
  encoder_message.z = (double) (millis() - timeOfLastPublish);

  l_encoder_count = 0;
  r_encoder_count = 0;
  timeOfLastPublish = millis();

  if(l_motor_dir != L_MOTOR_FWD_OR_STOP) {
    encoder_message.x = -encoder_message.x;
  }
  if(r_motor_dir != R_MOTOR_FWD_OR_STOP) {
    encoder_message.y = -encoder_message.y;
  }  

  encoder_pub.publish(&encoder_message);
}
*/




/***************************** Motors ******************************************/


/*
  pre:
  post: Motors driving at new configuration, l_motor_dir and r_motor_dir updated to
        reflect new drive directions
*/
/*
void driveWheelCallback(const geometry_msgs::Vector3 &wheel_cmd) {
  analogWrite(LED_BUILTIN, HIGH);

  int leftWheelCmd = (int) wheel_cmd.x;
  int rightWheelCmd = (int) wheel_cmd.y;
  
  //snprintf(log_buffer, sizeof(log_buffer) - 1, "Received wheel commands %i, %i", leftWheelCmd, rightWheelCmd);
  //nh.loginfo(log_buffer);

  publishEncoders();

  int leftWheelSpeed = abs(leftWheelCmd);
  if (leftWheelSpeed > MAX_PWM) {
    leftWheelSpeed = MAX_PWM;
    nh.logwarn("Arduino: Motor command above max PWM value, adjusting down");
  }

  int rightWheelSpeed = abs(rightWheelCmd);
  if (rightWheelSpeed > MAX_PWM) {
    rightWheelSpeed = MAX_PWM;
    nh.logwarn("Arduino: Motor command above max PWM value, adjusting down");
  }

  if(leftWheelCmd >= 0) {
    l_motor_dir = L_MOTOR_FWD_OR_STOP;
  } else {
    l_motor_dir = L_MOTOR_BKWD;
    leftWheelSpeed = MAX_PWM - leftWheelSpeed;
  }

  if(rightWheelCmd >= 0) {
    r_motor_dir = R_MOTOR_FWD_OR_STOP;
  } else {
    r_motor_dir = R_MOTOR_BKWD;
    rightWheelSpeed = MAX_PWM - rightWheelSpeed;
  }

  //snprintf(log_buffer, sizeof(log_buffer) - 1, "Wheel speeds %i, %i", leftWheelSpeed, rightWheelSpeed);
  //nh.loginfo(log_buffer);

  analogWrite(L_MOTOR_PWM_PIN, leftWheelSpeed);
  digitalWrite(L_MOTOR_DIR_PIN, l_motor_dir);
  analogWrite(R_MOTOR_PWM_PIN, rightWheelSpeed);
  digitalWrite(R_MOTOR_DIR_PIN, r_motor_dir);
}


ros::Subscriber<geometry_msgs::Vector3> motorSub("motor_cmd", &driveWheelCallback);
*/


void turnWheel(int wheelCmdSpd,
      unsigned int pwmPin,
      unsigned int dirPin,
      unsigned int &motorDirVariable) {
  
  int wheelSpeed = abs(wheelCmdSpd);
  if (wheelSpeed > MAX_PWM) {
    wheelSpeed = MAX_PWM;
    nh.logwarn("Arduino: Motor command above max PWM value, adjusting down");
  }

  if(wheelCmdSpd >= 0) {
    motorDirVariable = L_MOTOR_FWD_OR_STOP;
  } else {
    motorDirVariable = L_MOTOR_BKWD;
    wheelSpeed = MAX_PWM - wheelSpeed;
  }

  //snprintf(log_buffer, sizeof(log_buffer) - 1, "Wheel speed %i", wheelSpeed);
  //nh.loginfo(log_buffer);

  analogWrite(pwmPin, wheelSpeed);
  digitalWrite(dirPin, motorDirVariable);
}





void leftWheelCallback(const std_msgs::Int16 &wheel_cmd) {
  //publishLeftEncoder();
  int wheelCmdSpd = wheel_cmd.data;
  turnWheel(wheelCmdSpd, L_MOTOR_PWM_PIN, L_MOTOR_DIR_PIN, l_motor_dir);
}

void rightWheelCallback(const std_msgs::Int16 &wheel_cmd) {
  //publishRightEncoder();
  int wheelCmdSpd = wheel_cmd.data;
  turnWheel(wheelCmdSpd, R_MOTOR_PWM_PIN, R_MOTOR_DIR_PIN, r_motor_dir);
}



ros::Subscriber<std_msgs::Int16> lMotorSub("l_motor_cmd", &leftWheelCallback);
ros::Subscriber<std_msgs::Int16> rMotorSub("r_motor_cmd", &rightWheelCallback);

//ros::Subscriber<geometry_msgs::Vector3> motorSub("motor_cmd", &driveWheelCallback);






/***************************** Setup and Loop ******************************************/

void setup() {
  // put your setup code here, to run once:

  //Configure pins
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(L_MOTOR_PWM_PIN, OUTPUT);
  pinMode(L_MOTOR_DIR_PIN, OUTPUT);
  pinMode(R_MOTOR_PWM_PIN, OUTPUT);
  pinMode(R_MOTOR_DIR_PIN, OUTPUT);
  pinMode(L_ENCODER_PIN, INPUT);
  pinMode(R_ENCODER_PIN, INPUT);

  //Stop motors
  analogWrite(L_MOTOR_PWM_PIN, 0);
  digitalWrite(L_MOTOR_DIR_PIN, LOW);
  analogWrite(R_MOTOR_PWM_PIN, 0);
  digitalWrite(R_MOTOR_DIR_PIN, LOW);

  //Initialize encoder logic
  attachInterrupt(digitalPinToInterrupt(L_ENCODER_PIN), leftEncoderCallback, RISING);
  attachInterrupt(digitalPinToInterrupt(R_ENCODER_PIN), rightEncoderCallback, RISING);
  l_encoder_count = 0;
  r_encoder_count = 0;


  nh.initNode();
  nh.advertise(leftEncoderPub);
  nh.advertise(rightEncoderPub);
  nh.subscribe(lMotorSub);
  nh.subscribe(rMotorSub);

  //Delay for encoder interrupts to initialize
  delay(500);
}

void loop() {
  // put your main code here, to run repeatedly:

  publishLeftEncoder();
  publishRightEncoder();
  analogWrite(LED_BUILTIN, LOW);

  nh.spinOnce();
  delay(LOOP_TICK);
}
