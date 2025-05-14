/*To understand the context and purpose of this code, visit:
 * https://www.instructables.com/How-to-Make-a-Robot-Car-Drive-Straight-and-Turn-Ex/
 * This code makes references to steps on this Instructables website
 * written by square1a on 7th July 2022
 * 
 * Acknowledgement:
 * some of the MPU6050-raw-data-extraction code in void loop() and most in calculateError() are written by Dejan from:
 * https://howtomechatronics.com/tutorials/arduino/arduino-and-mpu6050-accelerometer-and-gyroscope-tutorial/
*/
#include <Arduino.h>
#include <Wire.h>
#define IR_SUPPORT 1
#ifdef IR_SUPPORT
#include <IRremote.hpp>
int IR_RECEIVE_PIN = 7; 
#endif

//control pins for left and right motors
const int leftSpeed = 9; //means pin 9 on the Arduino controls the speed of left motor
const int rightSpeed = 10;
const int left1 = 3; //left 1 and left 2 control the direction of rotation of left motor
const int left2 = 2;

const int right1 = 5;
const int right2 = 4;

const int MPU = 0x68; // MPU6050 I2C address
float AccX, AccY, AccZ; //linear acceleration
float GyroX, GyroY, GyroZ; //angular velocity
float accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ; //used in void loop()
float roll, pitch, yaw;
float AccErrorX, AccErrorY, GyroErrorX, GyroErrorY, GyroErrorZ;
float elapsedTime, currentTime, previousTime;
int c = 0;

const int maxSpeed = 120; //max PWM value written to motor speed pin. It is typically 255.
const int minSpeed = 50; //min PWM value at which motor moves (160)
float angle; //due to how I orientated my MPU6050 on my car, angle = roll
float targetAngle = 0;
int equilibriumSpeed = 120; //rough estimate of PWM at the speed pin of the stronger motor, while driving straight 
//and weaker motor at maxSpeed
int leftSpeedVal;
int rightSpeedVal;
bool isDriving = false; //it the car driving forward OR rotate/stationary
bool prevIsDriving = true; //equals isDriving in the previous iteration of void loop()
bool paused = false; //is the program paused

void readAcceleration();
void calculateError();
int changeSpeed (int motorSpeed, int increment);
void readGyro();
void stopCar();
void forward();
void left();
void right();
void controlSpeed ();
void driving();
void rotate();
void ir_loop();

void setup() {
#ifdef IR_SUPPORT
  IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK); // Start the receiver
#endif
  Serial.begin(115200);
  Wire.begin();                      // Initialize comunication
  Wire.beginTransmission(MPU);       // Start communication with MPU6050 // MPU=0x68
  Wire.write(0x6B);                  // Talk to the register 6B
  Wire.write(0x00);                  // Make reset - place a 0 into the 6B register
  Wire.endTransmission(true);        //end the transmission
  // Call this function if you need to get the IMU error values for your module
  calculateError();
  delay(20);
  pinMode(left1, OUTPUT);
  pinMode(left2, OUTPUT);
  pinMode(right1, OUTPUT);
  pinMode(right2, OUTPUT);
  pinMode(leftSpeed, OUTPUT);
  pinMode(rightSpeed, OUTPUT);
  currentTime = micros();

  stopCar();

}
#ifdef IR_SUPPORT
void dumpIR() {
    Serial.println("Infrared  dump ~~~~~~~~~~~~~~~~ ");
    Serial.println(IrReceiver.decodedIRData.decodedRawData, HEX); // Print "old" raw data
    IrReceiver.printIRResultShort(&Serial); // Print complete received data in one line
    IrReceiver.printIRSendUsage(&Serial);   // Print the statement required to send this data
    
}
#endif 

long t0 = micros();

void loop() {
  long now = micros();
  if(( now - t0) > 5000000) {
    //Serial.print("in loop ");
    //wSerial.println(now);
    t0 = now;

    Serial.print(t0);
    Serial.print(" angle: ");
    Serial.print(angle);
    Serial.print(", targetAngle: ");
    Serial.print(targetAngle);
    Serial.print(", GyroX: "); Serial.print(GyroX);
    Serial.print(", roll/pitch/yaw: "); Serial.print(roll); Serial.print("/");Serial.print(pitch); Serial.print("/");Serial.print(yaw);
    Serial.print(",  elapsedTime (in ms): "); //estimates time to run void loop() once
    Serial.print(elapsedTime * pow(10, 3));
    Serial.print(", equilibriumSpeed: ");
    Serial.print(equilibriumSpeed);
    Serial.print(", Left/Right ");Serial.print(leftSpeedVal); Serial.print("/");Serial.print(rightSpeedVal);
    Serial.println("");
  }

  // === Read accelerometer (on the MPU6050) data === //
  readAcceleration();
  // Calculating Roll and Pitch from the accelerometer data
  accAngleX = (atan(AccY / sqrt(pow(AccX, 2) + pow(AccZ, 2))) * 180 / PI) - AccErrorX; //AccErrorX is calculated in the calculateError() function
  accAngleY = (atan(-1 * AccX / sqrt(pow(AccY, 2) + pow(AccZ, 2))) * 180 / PI) - AccErrorY;
  
  // === Read gyroscope (on the MPU6050) data === //
  previousTime = currentTime;
  currentTime = micros();
  elapsedTime = (currentTime - previousTime) / 1000000; // Divide by 1000 to get seconds
  readGyro();
  // Correct the outputs with the calculated error values
  GyroX -= GyroErrorX; //GyroErrorX is calculated in the calculateError() function
  GyroY -= GyroErrorY;
  GyroZ -= GyroErrorZ;
  // Currently the raw values are in degrees per seconds, deg/s, so we need to multiply by sendonds (s) to get the angle in degrees
  gyroAngleX += GyroX * elapsedTime; // deg/s * s = deg
  gyroAngleY += GyroY * elapsedTime;
  yaw += GyroZ * elapsedTime;
  //combine accelerometer- and gyro-estimated angle values. 0.96 and 0.04 values are determined through trial and error by other people
  roll = 0.96 * gyroAngleX + 0.04 * accAngleX;
  pitch = 0.96 * gyroAngleY + 0.04 * accAngleY;
  // do a roll/pitch/yaw test.
  angle = yaw; //if you mounted MPU6050 in a different orientation to me, angle may not = roll. It can roll, pitch, yaw or minus version of the three
  //for me, turning right reduces angle. Turning left increases angle.
#ifdef IR_SUPPORT
    ir_loop();
#endif
  // Print the values on the serial monitor
  if(Serial.available()){
    char c = Serial.read ();
    //c = 'w';
    if (c == 'w') { //drive forward
      Serial.println("forward");
      isDriving = true;
    } else if (c == 'a') { //turn left
      Serial.println("left");
      targetAngle += 90;
      if (targetAngle > 180){
        targetAngle -= 360;
      }
      isDriving = false;
    } else if (c == 'd') { //turn right
      Serial.println("right");
      targetAngle -= 90;
      if (targetAngle <= -180){
        targetAngle += 360;
      }
      isDriving = false;
    } else if (c == 'q') { //stop or brake
      Serial.println("stop");
      Serial.print(" isdriving ");Serial.println(isDriving);
      isDriving = false;
    } else if (c == 'i') { //print information. When car is stationary, GyroX should approx. = 0. 
      Serial.print("angle: ");
      Serial.println(angle);
      Serial.print("targetAngle: ");
      Serial.println(targetAngle);
      Serial.print("GyroX: ");
      Serial.println(GyroX);
      Serial.print("elapsedTime (in ms): "); //estimates time to run void loop() once
      Serial.println(elapsedTime * pow(10, 3));
      Serial.print("equilibriumSpeed: ");
      Serial.println(equilibriumSpeed);
    } else if (c == 'p') { //pause the program
      paused = !paused;
      stopCar();
      isDriving = false;
      Serial.println("key p was pressed, which pauses/unpauses the program");
    }
  }

  static int count;
  static int countStraight;
  if (count < 6){  
    count ++;
  } else { //runs once after void loop() runs 7 times. void loop runs about every 2.8ms, so this else condition runs every 19.6ms or 50 times/second
    count = 0;
    if (!paused){
      if (isDriving != prevIsDriving){
          leftSpeedVal = equilibriumSpeed;
          countStraight = 0;
          Serial.print("mode changed, isDriving: ");
          Serial.println(isDriving);
      }
      if (isDriving) {
        if (abs(targetAngle - angle) < 3){
          if (countStraight < 20){
            countStraight ++;
          } else {
            countStraight = 0;
            equilibriumSpeed = leftSpeedVal; //to find equilibrium speed, 20 consecutive readings need to indicate car is going straight
            Serial.print("EQUILIBRIUM reached, equilibriumSpeed: ");
            Serial.println(equilibriumSpeed);
          }
        } else {
          countStraight = 0;
        }
        driving();
      } else {
        rotate();
      }
      prevIsDriving = isDriving;
    }
  }
}


double pid_integral = 0;
double pid_last_error = 0;

//the 'k' values are the ones you need to fine tune before your program will work. Note that these are arbitrary values that you just need to experiment with one at a time.
double Kp = 11;
double Ki = 0.09;
double Kd = 10;
long pid_t0  =0;
  double angle_yaw_output = 0;

void driving (){//called by void loop(), which isDriving = true
  long now = millis();

  int deltaAngle = round(targetAngle - angle); //rounding is neccessary, since you never get exact values in reality

  angle_yaw_output = angle_yaw_output * 0.9 + angle * 0.1; 

  //PID::
  double pid_error = targetAngle - angle_yaw_output;// proportional
 //if(abs(angle) > 3) {
    pid_integral = pid_integral + pid_error; //integral
    double pid_derivative = pid_error - pid_last_error; //derivative
    double adjust_angle_speed = (pid_error * Kp) + (pid_integral * Ki) + (pid_derivative * Kd);

    adjust_angle_speed = min(adjust_angle_speed, maxSpeed);
    adjust_angle_speed = max(adjust_angle_speed, minSpeed);
    pid_last_error = pid_error;

    if(( now - pid_t0) > 5000) {
      pid_t0 = now;
      Serial.print("PID speed adust ");
      Serial.print(deltaAngle);Serial.print("/");
      Serial.print(angle_yaw_output);Serial.print("/");
      Serial.print(pid_error); Serial.print("/");
      Serial.println(adjust_angle_speed);
    }

    /* from PID:
    UR->setSpeed(mtrSpd);
    UL->setSpeed(mtrSpd);
    UR->run(FORWARD);
    UL->run(FORWARD);

      // for our purplse adjust_angle_speed might be too big make sure to scale
       
    
    //setting the steering command if it is veering to the right
    if(angle_pitch_output > 0){
    UL->setSpeed(mtrSpd-abs(adjust_angle_speed));
    UR->setSpeed(mtrSpd+abs(adjust_angle_speed));
    UR->run(FORWARD);
    UL->run(FORWARD);
    
    
    }
    
    else if(angle_pitch_output < 0){ //setting the steering command if it is veering to the left
      UL->setSpeed(mtrSpd+abs(adjust_angle_speed));
    UR->setSpeed(mtrSpd-abs(adjust_angle_speed));
    UR->run(FORWARD);
    UL->run(FORWARD);
    }

    
    */
 // } else {
  //  pid_integral = 0;
  //}


  forward();
  if (deltaAngle != 0){
    controlSpeed ();
    //rightSpeedVal = maxSpeed;
    analogWrite(rightSpeed, rightSpeedVal);
    analogWrite(leftSpeed, leftSpeedVal);
  }
}

void controlSpeed (){//this function is called by driving ()
  int deltaAngle = round(targetAngle - angle);
  int targetGyroX;
  
  //setting up propoertional control, see Step 3 on the website
  if (deltaAngle > 30){
      targetGyroX = 60;
  } else if (deltaAngle < -30){
    targetGyroX = -60;
  } else {
    targetGyroX = 2 * deltaAngle;
  }
  
  if (round(targetGyroX - GyroX) == 0){
    ;
  } else if (targetGyroX > GyroX){
    leftSpeedVal = changeSpeed(leftSpeedVal, -1); //would increase GyroX
  } else {
    leftSpeedVal = changeSpeed(leftSpeedVal, +1);
  }
}

void rotate (){//called by void loop(), which isDriving = false
  int deltaAngle = round(targetAngle - angle);
  int targetGyroX;
  if (abs(deltaAngle) <= 1){
    stopCar();
  } else {
    if (angle > targetAngle) { //turn left
      left();
    } else if (angle < targetAngle) {//turn right
      right();
    }

    //setting up propoertional control, see Step 3 on the website
    if (abs(deltaAngle) > 30){
      targetGyroX = 60;
    } else {
      targetGyroX = 2 * abs(deltaAngle);
    }
    
    if (round(targetGyroX - abs(GyroX)) == 0){
      ;
    } else if (targetGyroX > abs(GyroX)){
      leftSpeedVal = changeSpeed(leftSpeedVal, +1); //would increase abs(GyroX)
    } else {
      leftSpeedVal = changeSpeed(leftSpeedVal, -1);
    }
    rightSpeedVal = leftSpeedVal;
    analogWrite(rightSpeed, rightSpeedVal);
    analogWrite(leftSpeed, leftSpeedVal);
  }
}   

int changeSpeed (int motorSpeed, int increment){
  motorSpeed += increment;
  if (motorSpeed > maxSpeed){ //to prevent motorSpeed from exceeding 255, which is a problem when using analogWrite
    motorSpeed = maxSpeed;
  } else if (motorSpeed < minSpeed){
    motorSpeed = minSpeed;
  }
  return motorSpeed;
}

void calculateError() {
  //When this function is called, ensure the car is stationary. See Step 2 for more info
  
  // Read accelerometer values 200 times
  c = 0;
  while (c < 200) {
    readAcceleration();
    // Sum all readings
    AccErrorX += (atan((AccY) / sqrt(pow((AccX), 2) + pow((AccZ), 2))) * 180 / PI);
    AccErrorY += (atan(-1 * (AccX) / sqrt(pow((AccY), 2) + pow((AccZ), 2))) * 180 / PI);
    c++;
  }
  //Divide the sum by 200 to get the error value, since expected value of reading is zero
  AccErrorX = AccErrorX / 200;
  AccErrorY = AccErrorY / 200;
  c = 0;
  
  // Read gyro values 200 times
  while (c < 200) {
    readGyro();
    // Sum all readings
    GyroErrorX += GyroX;
    GyroErrorY += GyroY;
    GyroErrorZ += GyroZ;
    c++;
  }
  //Divide the sum by 200 to get the error value
  GyroErrorX = GyroErrorX / 200;
  GyroErrorY = GyroErrorY / 200;
  GyroErrorZ = GyroErrorZ / 200;
  Serial.println("The the gryoscope setting in MPU6050 has been calibrated");
}

void readAcceleration() {
  Wire.beginTransmission(MPU);
  Wire.write(0x3B); // Start with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); // Read 6 registers total, each axis value is stored in 2 registers
  //For a range of +-2g, we need to divide the raw values by 16384, according to the MPU6050 datasheet
  AccX = (Wire.read() << 8 | Wire.read()) / 16384.0; // X-axis value
  AccY = (Wire.read() << 8 | Wire.read()) / 16384.0; // Y-axis value
  AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0; // Z-axis value 14 bit shift
}

void readGyro() {
  Wire.beginTransmission(MPU);
  Wire.write(0x43);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true);
  GyroX = (Wire.read() << 8 | Wire.read()) / 131.0;
  GyroY = (Wire.read() << 8 | Wire.read()) / 131.0;
  GyroZ = (Wire.read() << 8 | Wire.read()) / 131.0;
}

void stopCar(){
  digitalWrite(right1, LOW);
  digitalWrite(right2, LOW);
  digitalWrite(left1, LOW);
  digitalWrite(left2, LOW);
  analogWrite(rightSpeed, 0);
  analogWrite(leftSpeed, 0);
}

void forward(){ //drives the car forward, assuming leftSpeedVal and rightSpeedVal are set high enough
  digitalWrite(right1, HIGH); //the right motor rotates FORWARDS when right1 is HIGH and right2 is LOW
  digitalWrite(right2, LOW);
  digitalWrite(left1, HIGH);
  digitalWrite(left2, LOW);
}

void left(){ //rotates the car left, assuming speed leftSpeedVal and rightSpeedVal are set high enough
  digitalWrite(right1, LOW);
  digitalWrite(right2, HIGH);
  digitalWrite(left1, HIGH);
  digitalWrite(left2, LOW);
}

void right(){
  digitalWrite(right1, HIGH);
  digitalWrite(right2, LOW);
  digitalWrite(left1, LOW);
  digitalWrite(left2, HIGH);
}


void backward(){ //drives the car forward, assuming leftSpeedVal and rightSpeedVal are set high enough
  digitalWrite(right1, LOW); //the right motor rotates FORWARDS when right1 is HIGH and right2 is LOW
  digitalWrite(right2, HIGH);
  digitalWrite(left1, LOW);
  digitalWrite(left2, HIGH);
}

#ifdef IR_SUPPORT
long t0_ir = 0;
void ir_loop() {

    long now = millis();
     bool read_ir = true;
    if(( now - t0_ir) > 5000) {
      t0_ir = now;
      read_ir = true;
    }

   if ( read_ir && IrReceiver.decode()) {
      //Serial.println(IrReceiver.decodedIRData.decodedRawData, HEX); // Print "old" raw data
      //IrReceiver.printIRResultShort(&Serial); // Print complete received data in one line
      //IrReceiver.printIRSendUsage(&Serial);   // Print the statement required to send this data

      if (IrReceiver.decodedIRData.protocol == NEC) {
        uint16_t received = IrReceiver.decodedIRData.command;
        // Serial.print("Command: 0x");
        // Serial.println(received, HEX);

        if(received == 0x42 ) { /* * */
           Serial.print("angle: ");
          Serial.println(angle);
          Serial.print("targetAngle: ");
          Serial.println(targetAngle);
          Serial.print("GyroX: ");
          Serial.println(GyroX);
          Serial.print("elapsedTime (in ms): "); //estimates time to run void loop() once
          Serial.println(elapsedTime * pow(10, 3));
          Serial.print("equilibriumSpeed: ");
          Serial.println(equilibriumSpeed);
        } else if (received == 0x4A) { //pause the program cmd = #
          paused = !paused;
          stopCar();
          isDriving = false;
          Serial.println("key p was pressed, which pauses/unpauses the program");
        } else if (received == 0x44 ) { //turn left
          Serial.println("left");
          targetAngle += 90;
          if (targetAngle > 180){
            targetAngle -= 360;
          }
          isDriving = false;
        } else if (received == 0x43) { //turn right
          Serial.println("right");
          targetAngle -= 90;
          if (targetAngle <= -180){
            targetAngle += 360;
          }
          isDriving = false;
        } else if (received == 0x46) { //drive forward
           Serial.println("forward");
          isDriving = true;
        }else if (received == 0x40) { //stop or brake OK
          Serial.println("stop");
          Serial.print(" isdriving ");Serial.println(isDriving);
          isDriving = false;
        } else {
          dumpIR();
        }
      }else {
          dumpIR();
      }
      bool reset = IrReceiver.decodedIRData.protocol ==UNKNOWN;
      IrReceiver.resume(); // Enable receiving of the next value
      if( reset ) {
      // Serial.println("Reset IR");
      // IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK);
    }
  }
}
#endif