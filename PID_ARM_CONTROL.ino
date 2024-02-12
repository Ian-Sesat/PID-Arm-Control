#include <Wire.h> //I2C lib
#include <Servo.h>

//Declaring some global variables
int gyro_x, gyro_y, gyro_z;
long acc_x, acc_y, acc_z, acc_total_vector;
int temperature;
long gyro_x_cal, gyro_y_cal, gyro_z_cal;
long loop_timer;
float angle_pitch, angle_roll;
int angle_pitch_buffer, angle_roll_buffer;
boolean set_gyro_angles;
float angle_roll_acc, angle_pitch_acc;
float angle_pitch_output, angle_roll_output, ArmAngle;

//#include "ESC.h"
#define ESC_CALIBRATION_LED 13       // Pin for ESC CALIBRATION LED 
#define GYRO_CALIBRATION_LED 13      // Pin for GYRO CALIBRATION LED

//The following are the limits that our system should operate within..
#define ESC_lower_limit 1100         //The minimum value given to ESC so that the motors keep running (1000 is too low. The motors will have stopped completely!)
#define ESC_upper_limit 1300         //The maximum value given to ESC so as to operate our system at safe speed. (2000 is too much! It might cause an accident)

//The ESC needs a minimum value of 1000us and a maximum value of 2000us
//ESC myESC (7, 1000, 2000, 500);      // ESC_Name (ESC PIN, Minimum Value, Maximum Value, Arm Value)

unsigned long lastTime;
double ProcessVariable, Output;
double P_term, I_term, D_term, lastErr;
double Setpoint;                                      //This is where we want our system to be
double error;                                         //Difference between setpoint and process variable
double dErr;                                          //Change in error
int SampleTime = 10; //0.01 sec                       //The sample time for our plant (discrete PID)
double offset = 88;
double SampleTimeInSec = ((double)SampleTime)/1000;   //Converting the sample time into seconds

//The following 3 variables kp, ki and kd should be calculated as per Zigler Nichol's method
double  kp = 4.0;                                    // Ku = 1.75 Tu = 1.24
double  ki = 1.2 * SampleTimeInSec;                  //3.62
double  kd = 0.7 / SampleTimeInSec;                 //0.325
double  ue = 0;
unsigned long current_millis; 
unsigned long previous_millis = 0; 
unsigned long interval = 50; 

Servo motor;

void setup() 
{
  pinMode(4, INPUT);
  pinMode(GYRO_CALIBRATION_LED, OUTPUT);
  motor.attach(7);  //BLDC signal pin
  Wire.begin();                                                        //Start I2C as master
  Serial.begin(57600);                                                 //Use only for debugging

  motor.writeMicroseconds(1000); 
  setup_mpu_6050_registers();                                          //Setup the registers of the MPU-6050 (500dfs / +/-8g) and start the gyro

  Serial.print("Calibrating the IMU");
  //Take multiple gyro data samples and determine the average gyro offset (calibration).
  for (int cal_int = 0; cal_int < 2000 ; cal_int ++)                   //Run this code 2000 times for 2000 samples of gyro data
  { 
    if(cal_int % 80 == 0)digitalWrite(GYRO_CALIBRATION_LED, !digitalRead(GYRO_CALIBRATION_LED)); //Change the led status to indicate calibration. 
    //if(cal_int % 100 == 0)Serial.print(".");
    
    read_mpu_6050_data();                                              //Read the raw acc and gyro data from the MPU-6050
    gyro_x_cal += gyro_x;                                              //Add the gyro x-axis offset to the gyro_x_cal variable
    gyro_y_cal += gyro_y;                                              //Add the gyro y-axis offset to the gyro_y_cal variable
    gyro_z_cal += gyro_z;                                              //Add the gyro z-axis offset to the gyro_z_cal variable
    //Prevent the esc's from beeping annoyingly by giving them a 1000us pulse while calibrating the gyro.
    PORTD |= B10000000;                                                //Set digital pin 7 high.
    delayMicroseconds(1000);                                           //Wait 1000us.
    PORTD &= B01111111;                                                //Set digital pin 7 low.
    delay(3);                                                          //Delay 3us to simulate the 250Hz program loop
  }
  Serial.println("");
  Serial.println("");
  delay(500);
  
  gyro_x_cal /= 2000;                                                  //Divide the gyro_x_cal variable by 2000 to get the avarage offset
  gyro_y_cal /= 2000;                                                  //Divide the gyro_y_cal variable by 2000 to get the avarage offset
  gyro_z_cal /= 2000;                                                  //Divide the gyro_z_cal variable by 2000 to get the avarage offset

  loop_timer = micros();                                               //Reset the loop timer
}

void loop()
{  
  current_millis = millis();

  read_mpu_6050_data();                                                //Read the raw acc and gyro data from the MPU-6050

  gyro_x -= gyro_x_cal;                                                //Subtract the offset calibration value from the raw gyro_x value
  gyro_y -= gyro_y_cal;                                                //Subtract the offset calibration value from the raw gyro_y value
  gyro_z -= gyro_z_cal;                                                //Subtract the offset calibration value from the raw gyro_z value
  
  //Gyro angle calculations
  //0.0000611 = 1 / (250Hz / 65.5)
  angle_pitch += gyro_x * 0.0000611;                                   //Calculate the traveled pitch angle and add this to the angle_pitch variable
  angle_roll += gyro_y * 0.0000611;                                    //Calculate the traveled roll angle and add this to the angle_roll variable
  
  //0.000001066 = 0.0000611 * (3.142(PI) / 180degr) The Arduino sin function is in radians
  angle_pitch += angle_roll * sin(gyro_z * 0.000001066);               //If the IMU has yawed transfer the roll angle to the pitch angel
  angle_roll -= angle_pitch * sin(gyro_z * 0.000001066);               //If the IMU has yawed transfer the pitch angle to the roll angel
  
  //Accelerometer angle calculations
  acc_total_vector = sqrt((acc_x*acc_x)+(acc_y*acc_y)+(acc_z*acc_z));  //Calculate the total accelerometer vector
  //57.296 = 1 / (3.142 / 180) The Arduino asin function is in radians
  angle_pitch_acc = asin((float)acc_y/acc_total_vector)* 57.296;       //Calculate the pitch angle
  angle_roll_acc = asin((float)acc_x/acc_total_vector)* -57.296;       //Calculate the roll angle
  
  //Place the MPU-6050 spirit level and note the values in the following two lines for calibration
  angle_pitch_acc -= 0.0;                                              //Accelerometer calibration value for pitch
  angle_roll_acc -= 0.0;                                               //Accelerometer calibration value for roll

  if(set_gyro_angles)                                                  //If the IMU is already started
  { 
    angle_pitch = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004;     //Correct the drift of the gyro pitch angle with the accelerometer pitch angle
    angle_roll = angle_roll * 0.9996 + angle_roll_acc * 0.0004;        //Correct the drift of the gyro roll angle with the accelerometer roll angle
  }
  else                                                                 //At first start
  {  
    angle_pitch = angle_pitch_acc;                                     //Set the gyro pitch angle equal to the accelerometer pitch angle 
    angle_roll = angle_roll_acc;                                       //Set the gyro roll angle equal to the accelerometer roll angle 
    set_gyro_angles = true;                                            //Set the IMU started flag
  }
  //To dampen the pitch and roll angles a complementary filter is used
  angle_pitch_output = angle_pitch_output * 0.9 + angle_pitch * 0.1;   //Take 90% of the output pitch value and add 10% of the raw pitch value
  angle_roll_output = angle_roll_output * 0.9 + angle_roll * 0.1;      //Take 90% of the output roll value and add 10% of the raw roll value

  ArmAngle = map(angle_roll_output, 0, 90, 90, 0);
  
  while(micros() - loop_timer < 4000);                                 //Wait until the loop_timer reaches 4000us (250Hz) before starting the next loop
  loop_timer = micros();                                               //Reset the loop timer

  PID_magic();
  Command_Actuator();
}

//The following function gets the error (difference btwn Setpoint and ProcessVariable)
//and uses the error value in calculating the controller output so as to minimize the error.
//The P term, I term and the D term are calculated and summed up to give the controller output. 
//This is where all the magic in the system happens, hence the name of the function "PID magic"

void PID_magic()
{ 
   unsigned long now = millis();
   int timeChange = (now - lastTime);
   if(timeChange>=SampleTime)                                            //If the time change is greater than or equal to our plant's sample time, execute the following code
   {
      ProcessVariable = ArmAngle;                     //The actual position of the system. This value is obtained from sensor data
      Setpoint = 40;                                                     //The angle that we want our system to stay at.        
      error = Setpoint - ProcessVariable;                                //The error. How far we are from the setpoint..
      P_term = kp * error;                                               //Calculating the Proportional term
      I_term += ki*error;                                                //The Integral term.. Summing up the error. It helps reduce steady state error
      if(I_term > ESC_upper_limit)I_term = ESC_upper_limit;               //Applying the limits. Helps prevent the Output from pushing our system to saturation (Preventing integral windup)
      dErr  = (error - lastErr);                                         //Calculating the change in error
      D_term = (kd * dErr);                                              //Calculating the Derivative term
      Output = ESC_lower_limit + ue + P_term + I_term + D_term;          //Calculating the control signal
      if(Output > ESC_upper_limit)Output = ESC_upper_limit;              //Control signal's upper limit (Preventing integral windup)
      else if(Output < ESC_lower_limit) Output = ESC_lower_limit;        //Control signal's lower limit (Preventing integral windup)
      lastErr = error;                                                   //Saving the last error(The system should remember its last error)
      lastTime = now;                                                    //Saving the last time for the loop
   }
   if ((current_millis - previous_millis) >= interval)
   {
      //Plotting the setpoint and process variable in the serial plotter. 
      //Kindly use the serial plotter for easier analysis of the data being plotted.
      
      Serial.print(0);
      Serial.print(" ");
      Serial.print(90);
      Serial.print(" ");
      Serial.print(Setpoint);                                            //Plot the setpoint of our plant
      Serial.print(",");
      Serial.println(ProcessVariable);                                   //Plot our plant's process variable
   
      previous_millis = millis();
   }
} 

//The following function gives the command to the ESC (Electronic Speed Controller) 
//so that it can drive the Actuator.

void Command_Actuator()
{
    motor.writeMicroseconds(Output); 
}

//The following function reads the raw gyroscope and accelerometer data
//from the MPU-6050 IMU (Inertial Measurement Unit)

void read_mpu_6050_data()                                              //Subroutine for reading the raw gyro and accelerometer data
{
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x3B);                                                    //Send the requested starting register
  Wire.endTransmission();                                              //End the transmission
  Wire.requestFrom(0x68,14);                                           //Request 14 bytes from the MPU-6050
  while(Wire.available() < 14);                                        //Wait until all the bytes are received
  acc_x = Wire.read()<<8|Wire.read();                                  //Add the low and high byte to the acc_x variable
  acc_y = Wire.read()<<8|Wire.read();                                  //Add the low and high byte to the acc_y variable
  acc_z = Wire.read()<<8|Wire.read();                                  //Add the low and high byte to the acc_z variable
  temperature = Wire.read()<<8|Wire.read();                            //Add the low and high byte to the temperature variable
  gyro_x = Wire.read()<<8|Wire.read();                                 //Add the low and high byte to the gyro_x variable
  gyro_y = Wire.read()<<8|Wire.read();                                 //Add the low and high byte to the gyro_y variable
  gyro_z = Wire.read()<<8|Wire.read();                                 //Add the low and high byte to the gyro_z variable
}

//The following function sets up the MPU-6050 registers that are used in the I2C 
//communication between the sensor and the ATMEGA 328 (The microcontroller in arduino uno)

void setup_mpu_6050_registers()
{
  //Activate the MPU-6050
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x6B);                                                    //Send the requested starting register
  Wire.write(0x00);                                                    //Set the requested starting register
  Wire.endTransmission();                                              //End the transmission
  //Configure the accelerometer (+/-8g)
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x1C);                                                    //Send the requested starting register
  Wire.write(0x00);                                                    //Set the requested starting register
  Wire.endTransmission();                                              //End the transmission
  //Configure the gyro (500dps full scale)500 degrees per second
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x1B);                                                    //Send the requested starting register
  Wire.write(0x08);                                                    //Set the requested starting register
  Wire.endTransmission();                                              //End the transmission
  Wire.beginTransmission(0x68);
  Wire.write(0x1A);                                                    // the config address
  Wire.write(0x03);                                                    // the config value
  Wire.endTransmission(true);
}
