// Libraries to include
#include <Wire.h>       // Wire library for reading from gyro
#include <Servo.h>      // Servo library for writing to esc's

// Setup four ESC's as instances of the servo class
Servo esc1;
Servo esc2;
Servo esc3;
Servo esc4;

// Declaration of variables
// Receiver Input Variables
unsigned long current_time, timer_1, timer_2, timer_3, timer_4, timer_5, timer_6;
byte last_channel_1, last_channel_2, last_channel_3, last_channel_4, last_channel_5, last_channel_6;
volatile int receiver_input_channel_1, receiver_input_channel_2, receiver_input_channel_3;
volatile int receiver_input_channel_4, receiver_input_channel_5, receiver_input_channel_6;

// Inertial Measurement Unit Variables
int cal_int, gyro_address;
float acc_x, acc_y, acc_z, cal_acc_x, cal_acc_y, cal_acc_z, temperature;
float gyro_roll, gyro_pitch, gyro_yaw, cal_gyro_roll, cal_gyro_pitch, cal_gyro_yaw;
float gyro_roll_input, gyro_pitch_input;

// PID control and ESC Output Variables
unsigned long loop_timer;
int start, throttle, esc_1, esc_2, esc_3, esc_4;
// Rate Mode
float pitch_set_point, pitch_error, pitch_error_last, pitch_proportional, pitch_integral, pitch_derivative, pitch_PID;
float roll_set_point, roll_error, roll_error_last, roll_proportional, roll_integral, roll_derivative, roll_PID;
float yaw_set_point, yaw_error, yaw_error_last, yaw_proportional, yaw_integral, yaw_derivative, yaw_PID;
float roll_k_integral, pitch_k_integral, yaw_k_integral;
// Angle Mode
float acc_roll_angle, acc_pitch_angle, gyro_roll_angle, gyro_pitch_angle;
float roll_angle, pitch_angle;
float roll_error_angle, roll_proportional_angle, roll_integral_angle, roll_PID_angle;
float pitch_error_angle, pitch_proportional_angle, pitch_integral_angle, pitch_PID_angle;

// Tuning Parameter Variables
int Roll_Rate, Roll_Rate_1, Roll_Rate_2, Roll_Rate_3;
int Pitch_Rate, Pitch_Rate_1, Pitch_Rate_2, Pitch_Rate_3;
int Yaw_Rate, Yaw_Rate_1, Yaw_Rate_2, Yaw_Rate_3;
int Loop_Rate, Loop_Time, Roll_Angle, Pitch_Angle, Roll_PID_Max, Pitch_PID_Max, Yaw_PID_Max;
float Roll_K_Proportional, Roll_K_Integral, Roll_K_Derivative;
float Pitch_K_Proportional, Pitch_K_Integral, Pitch_K_Derivative;
float Yaw_K_Proportional, Yaw_K_Integral, Yaw_K_Derivative;
float Angle_K_Proportional, Angle_K_Integral;

// Expo and Anti Gravity
float Rate_Expo, Throttle_Expo, Throttle_Mid;
int Receiver_Mid, Throttle_Centre, Range_Upper, Range_Lower;
float Anti_Gravity_Gain, Threshold_High, Threshold_Low;
int anti_gravity_count, gravity_timer, input_last, input_start;
bool anti_gravity_flag, flag_last, anti_gravity_on;




////////////////////////////////////////////////////////////////////
////                      SET UP PROCEDURE                      ////
////////////////////////////////////////////////////////////////////
void setup() {
  //Serial.begin(115200);

  // Configure pins as inputs and outputs
  DDRD |= B00001000;          // Port D Data Direction Register - set pin 3 as output
  DDRD &= B00001111;          // Port D Data Direction Register - set pin 4, 5, 6, 7 as input
  DDRB |= B00101110;          // Port B Data Direction Register - set pins 9, 10, 11 as outputs
  DDRB &= B11101110;          // Port B Data Direction Register - set pins 8, 12 as input

  // Set up interrupt routine for 6 receiver channels
  PCICR |= (1 << PCIE2);      // Pin Change Interrupt Enable 2 on Pin Change Interrupt Control Register
  PCICR |= (1 << PCIE0);      // Pin Change Interrupt Enable 0 on Pin Change Interrupt Control Register
  PCMSK2 |= (1 << PCINT20);   // Set bit for PCINT20 (pin 4) in Pin Change Mask Register 2
  PCMSK2 |= (1 << PCINT21);   // Set bit for PCINT21 (pin 5) in Pin Change Mask Register 2
  PCMSK2 |= (1 << PCINT22);   // Set bit for PCINT20 (pin 6) in Pin Change Mask Register 2
  PCMSK2 |= (1 << PCINT23);   // Set bit for PCINT20 (pin 7) in Pin Change Mask Register 2
  PCMSK0 |= (1 << PCINT0);    // Set bit for PCINT0 (pin 8) in Pin Change Mask Register 0
  PCMSK0 |= (1 << PCINT4);    // Set bit for PCINT0 (pin 12) in Pin Change Mask Register 0

  // Set gyro location and open communication
  gyro_address = 0x68;
  Wire.begin();
  TWBR = 12;

  // Establish connection with gyro
  set_gyro_registers();

  // Calibrate gyro offsets
  digitalWrite(13, HIGH);                           // Turn on LED and wait 2 seconds
  delay(2000);
  for (cal_int = 0; cal_int < 10500; cal_int++) {
    if (cal_int % 1000 == 0) {                      // Toggle LED on and off every 1000 readings
      digitalWrite(13, !digitalRead(13));
    }
    if (cal_int < 500) gyro_read();                 // Discard first 500 readings
    else {
      gyro_read();                                  // Read raw gyro and accelerometer values

      cal_acc_x += acc_x;                           // Sum raw x accel to calibration variable
      cal_acc_y += acc_y;                           // Sum raw y accel to calibration variable
      cal_acc_z += acc_z;                           // Sum raw z accel to calibration variable
      cal_gyro_roll += gyro_roll;                   // Sum raw x gyro to calibration variable
      cal_gyro_pitch += gyro_pitch;                 // Sum raw y gyro to calibration variable
      cal_gyro_yaw += gyro_yaw;                     // Sum raw z gyro to calibration variable
    }
  }

  cal_acc_x /= 10000;                               // Find x accel offset (Approx. -54)
  cal_acc_y /= 10000;                               // Find y accel offset (Approx. -5)
  cal_acc_z = (cal_acc_z/10000) - 4096;             // Find z accel offset (Approx. -830)
  cal_gyro_roll /= 10000;                           // Find x gyro offset (Approx. 178)
  cal_gyro_pitch /= 10000;                          // Find y gyro offset (Approx. -342)
  cal_gyro_yaw /= 10000;                            // Find z gyro offset (Approx. 77)

  // Toggle LED on to show finished claibration and wait 2 seconds
  digitalWrite(13, HIGH);
  delay(2000);

  // Wait until the receiver is active and the throttle is set to the lower position.
  while(receiver_input_channel_2 < 990 || receiver_input_channel_2 > 1020 || receiver_input_channel_1 < 1400) {
    start ++;                                                               //While waiting increment start whith every loop.

    //Give the escs a 900us pulse to initialise them while waiting for the receiver to be switch on
    PORTD |= B00001000;                                                     //Set digital poort 4, 5, 6 and 7 high.
    PORTB |= B00001110;
    delayMicroseconds(700);                                                 //Wait 900us.
    PORTD &= B11110111;                                                     //Set digital poort 4, 5, 6 and 7 low.
    PORTB &= B11110001;
    delay(3);                                                               //Wait 3 milliseconds before the next loop.
    if(start == 20){                                                        //Every 20 loops (18ms).
      digitalWrite(13, !digitalRead(13));                                   //Change the led status.
      start = 0;                                                            //Start again at 0.
    }
  }

  // Attach esc instances to corresponding output pins
  esc1.attach(9);                   // Front Left CW
  esc2.attach(11);                  // Front Right CCW
  esc3.attach(3);                   // Back Left CW
  esc4.attach(10);                  // Back Right CCW

  start = 0;                        // Reset start variable for arming esc's
  anti_gravity_count = 0;           // Set antigravity timing condition
  loop_timer = micros();            // Start loop timer to check for loop overflow
  digitalWrite(13, LOW);            // Toggle LED of to indicate setup complete


  // Tuning Parameters
  Loop_Rate = 400;                  // Refresh rate (Hz)
  Loop_Time = 1000000 / Loop_Rate;  // Corresponding loop time (us)

  Roll_Angle = 20;                  // Maximum roll angle in stabilize mode (deg)
  Pitch_Angle = Roll_Angle;         // Maximum pitch_angle in stabilize mode (deg)

  Roll_Rate_1 = 60;                 // Maximum roll rate option 1 (deg/s)
  Roll_Rate_2 = 180;                // Maximum roll rate option 2 (deg/s)
  Roll_Rate_3 = 360;                // Maximum roll rate option 3 (deg/s)
  Pitch_Rate_1 = Roll_Rate_1;       // Maximum pitch rate option 1 (deg/s)
  Pitch_Rate_2 = Roll_Rate_2;       // Maximum pitch rate option 2 (deg/s)
  Pitch_Rate_3 = Roll_Rate_3;       // Maximum pitch rate option 3 (deg/s)
  Yaw_Rate_1 = 90;                  // Maximum yaw rate option 1 (deg/s)
  Yaw_Rate_2 = 270;                 // Maximum yaw rate option 2 (deg/s)
  Yaw_Rate_3 = 540;                 // Maximum yaw rate option 3 (deg/s)

  Rate_Expo = 0.4;                  // Rate exponential setting
  Throttle_Expo = 0.05;             // Throttle exponential setting
  Throttle_Mid = 0.3;               // Mid point for throttle exponential
  Anti_Gravity_Gain = 5.0;          // I gain multiplier in throttle punch and chop

                                    // (Defaults for 400Hz refresh rate)
  Angle_K_Proportional = 1.9;       // (Default 1.9) Proportional term for angle control
  Angle_K_Integral = 0.025;         // (Default ???) Integral term for angle control

  Roll_K_Proportional = 0.7;        // 0.8(Default 0.7) Proportional term for roll rate PID
  Roll_K_Integral = 0.0105;           // 0.01(Default 0.009) Integral term for roll rate PID
  Roll_K_Derivative = 7.5;            // 8(Default 8) Derivative term for roll rate PID
  Roll_PID_Max = 200;               // Maximum allowed correction pulse (us)

  Pitch_K_Proportional = 0.8;      // 0.95(Default 0.75) Proportional term for pitch rate PID
  Pitch_K_Integral = 0.0115;         // 0.011(Default 0.01) Integral term for pitch rate PID
  Pitch_K_Derivative = 8;           // 8(Default 8) Derivative term for pitch rate PID
  Pitch_PID_Max = Roll_PID_Max;     // Maximum allowed correction pulse (us)

  Yaw_K_Proportional = 3;           // (Default 3) Proportial term for yaw rate PID
  Yaw_K_Integral = 0.015;           // (Default 0.012) Integral term for yaw rate PID
  Yaw_K_Derivative = 0;             // (Default 0) Derivative term for yaw rate PID
  Yaw_PID_Max = Roll_PID_Max;       // Maximum allowed correction pulse (us)

  // Preliminary Calculations
  Receiver_Mid = 1000 * (1 + Throttle_Mid);
  Throttle_Centre = 800 + 700 * Throttle_Mid;
  Range_Upper = 1500 - Throttle_Centre;
  Range_Lower = Throttle_Centre - 800;

  Threshold_High = 1 + (2.5 / Loop_Rate);
  Threshold_Low = 1 - (2.5 / Loop_Rate);

  roll_k_integral = Roll_K_Integral;
  pitch_k_integral = Pitch_K_Integral;
  yaw_k_integral = Yaw_K_Integral;
}


////////////////////////////////////////////////////////////////////
////                     MAIN PROGRAM LOOP                      ////
////////////////////////////////////////////////////////////////////
void loop() {
  // Motor arming routine - Step 1, throttle low and yaw left
  if(receiver_input_channel_2 < 1050 && receiver_input_channel_1 < 1050) start = 1;

  // Motor arming routine - Step 2, yaw back to centre
  if(start == 1 && receiver_input_channel_2 < 1050 && receiver_input_channel_1 > 1450){
    start = 2;

    // Reset the PID controllers for a bumpless start
    roll_integral = 0;
    roll_integral_angle = 0;
    roll_error_last = 0;
    pitch_integral = 0;
    pitch_integral_angle = 0;
    pitch_error_last = 0;
    yaw_integral = 0;
    yaw_error_last = 0;
  }

  // Motor disarming routine - Throttle low and yaw right
  if(start == 2 && receiver_input_channel_2 < 1050 && receiver_input_channel_1 > 1950) {
    start = 0;
  }

  // When motors are armed execute flight control code
  if (start == 2) {
    // Check for current rate sensitivity - toggled by switch SwC on receiver
    if (receiver_input_channel_5 < 1250) {            // Rates set to low sensitivity
      Roll_Rate = Roll_Rate_1;
      Pitch_Rate = Pitch_Rate_1;
      Yaw_Rate = Yaw_Rate_1;
    } else if (receiver_input_channel_5 < 1750) {     // Rates set to mid sensitivity
      Roll_Rate = Roll_Rate_2;
      Pitch_Rate = Pitch_Rate_2;
      Yaw_Rate = Yaw_Rate_2;
    } else {                                          // Rates set to high sensitivity
      Roll_Rate = Roll_Rate_3;
      Pitch_Rate = Pitch_Rate_3;
      Yaw_Rate = Yaw_Rate_3;
    }

    // Read gyro and perform necessary manipulations
    gyro_read();

    // Check for current flight mode - toggled by switch SwD on receiver
    if (receiver_input_channel_6 < 1500) {            // Mode set to rate (acro) mode
      rate_mode();
    } else {                                          // Mode set to angle (stabilize) mode
      angle_mode();
    }

    if (receiver_input_channel_2 > 1050) {
      if (receiver_input_channel_1 > 1508) {
        yaw_set_point = -(Yaw_Rate / (exp(Rate_Expo) - 1)) * (exp(Rate_Expo * (receiver_input_channel_1 - 1508) / 492) - 1);
      } else if (receiver_input_channel_1 < 1492) {
        yaw_set_point = (Yaw_Rate / (exp(Rate_Expo) - 1)) * (exp(Rate_Expo * (1492 - receiver_input_channel_1) / 492) - 1);
      } else yaw_set_point = 0;
    } else yaw_set_point = 0;
    yaw_error = yaw_set_point - gyro_yaw;

    if (receiver_input_channel_2 > Receiver_Mid) {
      throttle = Throttle_Centre + (Range_Upper / (exp(Throttle_Expo) - 1)) * (exp(Throttle_Expo * (receiver_input_channel_2 - Receiver_Mid) / (2000 - Receiver_Mid)) - 1);
    } else if (receiver_input_channel_2 < Receiver_Mid) {
      throttle = Throttle_Centre - (Range_Lower / (exp(Throttle_Expo) - 1)) * (exp(Throttle_Expo * (Receiver_Mid - receiver_input_channel_2) / (Receiver_Mid - 1000)) - 1);
    }

    anti_gravity();
    PID_calc();

    esc_1 = throttle + pitch_PID + roll_PID + yaw_PID;
    esc_2 = throttle + pitch_PID - roll_PID - yaw_PID;
    esc_3 = throttle - pitch_PID + roll_PID - yaw_PID;
    esc_4 = throttle - pitch_PID - roll_PID + yaw_PID;

    if (esc_1 < 800) esc_1 = 800;
    if (esc_2 < 800) esc_2 = 800;
    if (esc_3 < 800) esc_3 = 800;
    if (esc_4 < 800) esc_4 = 800;

    if(esc_1 > 1500) esc_1 = 1500;
    if(esc_2 > 1500) esc_2 = 1500;
    if(esc_3 > 1500) esc_3 = 1500;
    if(esc_4 > 1500) esc_4 = 1500;

  } else {
    esc_1 = 700;
    esc_2 = 700;
    esc_3 = 700;
    esc_4 = 700;
  }

  esc1.write(esc_1);
  esc2.write(esc_2);
  esc3.write(esc_3);
  esc4.write(esc_4);

  if(micros() - loop_timer > Loop_Time + 50) digitalWrite(13, HIGH);
  //Serial.println(micros() - loop_timer);
  while(micros() - loop_timer < Loop_Time);
  loop_timer = micros();
}


void gyro_read() {
  //Read the MPU-6050
    Wire.beginTransmission(gyro_address);                     // Start communication with the gyro
    Wire.write(0x3B);                                         // Start reading at register 43 and auto increment with every read
    Wire.endTransmission();                                   // End the transmission
    Wire.requestFrom(gyro_address,14);                        // Request 14 bytes from the gyro
    while(Wire.available() < 14);                             // Wait until the 14 bytes are received
    acc_x = Wire.read() << 8 | Wire.read();                   // Add the low and high byte to the acc_x variable
    acc_y = Wire.read() << 8 | Wire.read();                   // Add the low and high byte to the acc_y variable
    acc_z = Wire.read() << 8 | Wire.read();                   // Add the low and high byte to the acc_z variable
    temperature = Wire.read() << 8 | Wire.read();             // Add the low and high byte to the temperature variable
    gyro_roll = Wire.read() << 8 | Wire.read();               // Read high and low part of the angular data
    gyro_pitch = Wire.read() << 8 | Wire.read();              // Read high and low part of the angular data
    gyro_yaw = Wire.read() << 8 | Wire.read();                // Read high and low part of the angular data

    if (cal_int == 10500) {
      acc_x = (acc_x - cal_acc_x);                            // Apply offset to x and convert 2's compliment data to m/s^2
      acc_y = (acc_y - cal_acc_y);                            // Apply offset to y and convert 2's compliment data to m/s^2
      acc_z = (acc_z - cal_acc_z);                            // Apply offset to z and convert 2's compliment data to m/s^2

      gyro_roll = (gyro_roll - cal_gyro_roll) / 65.536;       // Apply offset and convert 2's compliment roll data to dps
      gyro_pitch = (gyro_pitch - cal_gyro_pitch) / 65.536;    // Apply offset and convert 2's compliment pitch data to dps
      gyro_yaw = (gyro_yaw - cal_gyro_yaw) / 65.536;          // Apply offset and convert 2's compliment yaw data to dps

      // Gyro rates filter
      gyro_roll_input = 0.7 * gyro_roll_input + 0.3 * gyro_roll;      // Filter new roll rate data to smooth noise sources
      gyro_pitch_input = 0.7 * gyro_pitch_input + 0.3 * gyro_pitch;   // Filter new pitch rate data to smooth noise sources
    }
}


void rate_mode() {
  if (receiver_input_channel_4 > 1508) {
    roll_set_point = (Roll_Rate / (exp(Rate_Expo) - 1)) * (exp(Rate_Expo * (receiver_input_channel_4 - 1508) / 492) - 1);
  } else if (receiver_input_channel_4 < 1492) {
    roll_set_point = -(Roll_Rate / (exp(Rate_Expo) - 1)) * (exp(Rate_Expo * (1492 - receiver_input_channel_4) / 492) - 1);
  } else roll_set_point = 0;
  roll_error = roll_set_point - gyro_roll_input;

  if (receiver_input_channel_3 > 1508) {
    pitch_set_point = (Pitch_Rate / (exp(Rate_Expo) - 1)) * (exp(Rate_Expo * (receiver_input_channel_3 - 1508) / 492) - 1);
  } else if (receiver_input_channel_3 < 1492) {
    pitch_set_point = -(Pitch_Rate / (exp(Rate_Expo) - 1)) * (exp(Rate_Expo * (1492 - receiver_input_channel_3) / 492) - 1);
  } else pitch_set_point = 0;
  pitch_error = gyro_pitch_input - pitch_set_point;
}


void angle_mode() {
  // Accelerometer angle calculations
  acc_roll_angle = atan2((float) acc_y, sqrt((acc_x*acc_x) + (acc_z*acc_z))) * RAD_TO_DEG;    // Calculate the pitch angle
  acc_pitch_angle = atan2((float) -acc_x, sqrt((acc_y*acc_y) + (acc_z*acc_z))) * RAD_TO_DEG;  // Calculate the roll angle

  // Complementary filter
  roll_angle = 0.995 * (roll_angle + gyro_roll / Loop_Rate) + 0.005 * acc_roll_angle;         // Correct the drift of the gyro roll angle with the accelerometer roll angle
  pitch_angle = 0.995 * (pitch_angle + gyro_pitch / Loop_Rate) + 0.005 * acc_pitch_angle;     // Correct the drift of the gyro pitch angle with the accelerometer pitch angle

  if (receiver_input_channel_4 > 1508) {
    roll_set_point = Roll_Angle * (receiver_input_channel_4 - 1508) / (float)492;
  } else if (receiver_input_channel_4 < 1492) {
    roll_set_point = -Roll_Angle * (1492 - receiver_input_channel_4) / (float)492;
  } else roll_set_point = 0;
  roll_error_angle = roll_set_point - roll_angle;
  roll_proportional_angle = Angle_K_Proportional * roll_error_angle;
  roll_integral_angle += Angle_K_Integral * roll_error_angle;
  roll_PID_angle = roll_proportional_angle + roll_integral_angle;
  roll_error = roll_PID_angle - gyro_roll_input;

  if (receiver_input_channel_3 > 1508) {
    pitch_set_point = Pitch_Angle * (receiver_input_channel_3 - 1508) / (float)492;
  } else if (receiver_input_channel_3 < 1492) {
    pitch_set_point = -Pitch_Angle * (1492 - receiver_input_channel_3) / (float)492;
  } else pitch_set_point = 0;
  pitch_error_angle = pitch_set_point - pitch_angle;
  pitch_proportional_angle = Angle_K_Proportional * pitch_error_angle;
  pitch_integral_angle += Angle_K_Integral * pitch_error_angle;
  pitch_PID_angle = pitch_proportional_angle + pitch_integral_angle;
  pitch_error = gyro_pitch_input - pitch_PID_angle;
}


void anti_gravity() {
  if ((receiver_input_channel_2 > Threshold_High * input_last) ||
      (receiver_input_channel_2 < Threshold_Low * input_last)) anti_gravity_flag = true;

  if (anti_gravity_flag == true) {
    if (flag_last == false && anti_gravity_flag == true) input_start = input_last;
    anti_gravity_count += 1;

    if (anti_gravity_count == Loop_Rate / 10) {
      if (receiver_input_channel_2 > input_start * 1.25 || receiver_input_channel_2 < 0.75 * input_start) {
        roll_k_integral = Anti_Gravity_Gain * Roll_K_Integral;
        pitch_k_integral = Anti_Gravity_Gain * Pitch_K_Integral;
        yaw_k_integral = Anti_Gravity_Gain * Yaw_K_Integral;
        anti_gravity_on = true;
        gravity_timer = 0;
      }
      anti_gravity_flag = false;
      anti_gravity_count = 0;
    }
  }
  flag_last = anti_gravity_flag;
  input_last = receiver_input_channel_2;

  if (anti_gravity_on == true) {
    gravity_timer += 1;
    if (gravity_timer == Loop_Rate / 5) {
      roll_k_integral = Roll_K_Integral;
      pitch_k_integral = Pitch_K_Integral;
      yaw_k_integral = Yaw_K_Integral;
      gravity_timer = 0;
      anti_gravity_on = false;
    }
  }
}


void PID_calc() {
  roll_proportional = Roll_K_Proportional * roll_error;
  roll_integral += roll_k_integral * roll_error;
  //if (roll_integral > Roll_PID_Max) roll_integral = Roll_PID_Max;
  //else if (roll_integral < -1 * Roll_PID_Max) roll_integral = -1 * Roll_PID_Max;
  roll_derivative = Roll_K_Derivative * (roll_error - roll_error_last);
  roll_PID = roll_proportional + roll_integral + roll_derivative;
  if (roll_PID > Roll_PID_Max) roll_PID = Roll_PID_Max;
  else if (roll_PID < -1 * Roll_PID_Max) roll_PID = -1 * Roll_PID_Max;
  roll_error_last = roll_error;

  pitch_proportional = Pitch_K_Proportional * pitch_error;
  pitch_integral += pitch_k_integral * pitch_error;
  //if (pitch_integral > Pitch_PID_Max) pitch_integral = Pitch_PID_Max;
  //else if (pitch_integral < -1 * Pitch_PID_Max) pitch_integral = -1 * Pitch_PID_Max;
  pitch_derivative = Pitch_K_Derivative * (pitch_error - pitch_error_last);
  pitch_PID = pitch_proportional + pitch_integral + pitch_derivative;
  if (pitch_PID > Pitch_PID_Max) pitch_PID = Pitch_PID_Max;
  else if (pitch_PID < -1 * Pitch_PID_Max) pitch_PID = -1 * Pitch_PID_Max;
  pitch_error_last = pitch_error;

  yaw_proportional = Yaw_K_Proportional * yaw_error;
  yaw_integral += yaw_k_integral * yaw_error;
  //if (yaw_integral > Yaw_PID_Max) yaw_integral = Yaw_PID_Max;
  //else if (yaw_integral < -1 * Yaw_PID_Max) yaw_integral = -1 * Yaw_PID_Max;
  yaw_derivative = Yaw_K_Derivative * (yaw_error - yaw_error_last);
  yaw_PID = yaw_proportional + yaw_integral + yaw_derivative;
  if (yaw_PID > Yaw_PID_Max) yaw_PID = Yaw_PID_Max;
  else if (yaw_PID < -1 * Yaw_PID_Max) yaw_PID = -1 * Yaw_PID_Max;
  yaw_error_last = yaw_error;
}


ISR(PCINT0_vect, ISR_ALIASOF(PCINT2_vect));
ISR(PCINT2_vect) {
  current_time = micros();                                // Store current time with each loop
  // Channel 1 ========================================
  if (PIND & B00010000) {                                 // Is pin 4 high?
    if (last_channel_1 == 0) {                            // Has it just switched?
      last_channel_1 = 1;                                 // If so, store new value
      timer_1 = current_time;                             // Start timer
    }
  } else if (last_channel_1 == 1) {                       // Otherwise has pin 4 just switched low?
    last_channel_1 = 0;                                   // If so, store new value
    receiver_input_channel_1 = current_time - timer_1;    // Stop timer - input pulse length
  }

  // Channel 2 ========================================
  if (PIND & B00100000) {                                 // Is pin 5 high?
    if (last_channel_2 == 0) {                            // Has it just switched?
      last_channel_2 = 1;                                 // If so, store new value
      timer_2 = current_time;                             // Start timer
    }
  } else if (last_channel_2 == 1) {                       // Otherwise has pin 5 just switched low?
    last_channel_2 = 0;                                   // If so, store new value
    receiver_input_channel_2 = current_time - timer_2;    // Stop timer - input pulse length
  }

  // Channel 3 ========================================
  if (PIND & B01000000) {                                 // Is pin 6 high?
    if (last_channel_3 == 0) {                            // Has it just switched?
      last_channel_3 = 1;                                 // If so, store new value
      timer_3 = current_time;                             // Start timer
    }
  } else if (last_channel_3 == 1) {                       // Otherwise has pin 6 just switched low?
    last_channel_3 = 0;                                   // If so, store new value
    receiver_input_channel_3 = current_time - timer_3;    // Stop timer - input pulse length
  }

  // Channel 4 ========================================
  if (PIND & B10000000) {                                 // Is pin 7 high?
    if (last_channel_4 == 0) {                            // Has it just switched?
      last_channel_4 = 1;                                 // If so, store new value
      timer_4 = current_time;                             // Start timer
    }
  } else if (last_channel_4 == 1) {                       // Otherwise has pin 7 just switched low?
    last_channel_4 = 0;                                   // If so, store new value
    receiver_input_channel_4 = current_time - timer_4;    // Stop timer - input pulse length
  }

  // Channel 5 ========================================
  if (PINB & B00010000) {                                 // Is pin 12 high?
    if (last_channel_5 == 0) {                            // Has it just switched?
      last_channel_5 = 1;                                 // If so, store new value
      timer_5 = current_time;                             // Start timer
    }
  } else if (last_channel_5 == 1) {                       // Otherwise has pin 12 just switched low?
    last_channel_5 = 0;                                   // If so, store new value
    receiver_input_channel_5 = current_time - timer_5;    // Stop timer - input pulse length
  }

  // Channel 6 ========================================
  if (PINB & B00000001) {                                 // Is pin 8 high?
    if (last_channel_6 == 0) {                            // Has it just switched?
      last_channel_6 = 1;                                 // If so, store new value
      timer_6 = current_time;                             // Start timer
    }
  } else if (last_channel_6 == 1) {                       // Otherwise has pin 8 just switched low?
    last_channel_6 = 0;                                   // If so, store new value
    receiver_input_channel_6 = current_time - timer_6;    // Stop timer - input pulse length
  }
}


void set_gyro_registers() {
  //Setup the MPU-6050
  Wire.beginTransmission(gyro_address);              // Start communication with the gyro
  Wire.write(0x6B);                                  // We want to write to the PWR_MGMT_1 register (6B hex)
  Wire.write(B00000000);                             // Set the register bits as 00000000 to activate the gyro
  Wire.endTransmission();                            // End the transmission with the gyro.

  Wire.beginTransmission(gyro_address);              // Start communication with the gyro
  Wire.write(0x1B);                                  // We want to write to the GYRO_CONFIG register (1B hex)
  Wire.write(B00001000);                             // Set the register bits as 00001000 (500dps full scale)
  Wire.endTransmission();                            // End the transmission with the gyro

  Wire.beginTransmission(gyro_address);              // Start communication with the gyro
  Wire.write(0x1C);                                  // We want to write to the ACCEL_CONFIG register (1A hex)
  Wire.write(B00010000);                             // Set the register bits as 00010000 (+/- 8g full scale range)
  Wire.endTransmission();                            // End the transmission with the gyro

  Wire.beginTransmission(gyro_address);              // Start communication with the gyro
  Wire.write(0x1A);                                  // We want to write to the CONFIG register (1A hex)
  Wire.write(B00000011);                             // Set the register bits as 00000011 (Set Digital Low Pass Filter to ~43Hz)
  Wire.endTransmission();                            // End the transmission with the gyro

  // Perform a register check to see if the values are written correct
  Wire.beginTransmission(gyro_address);              // Start communication with the gyro
  Wire.write(0x1B);                                  // Start reading at register 0x1B
  Wire.endTransmission();                            // End the transmission
  Wire.requestFrom(gyro_address, 1);                 // Request 1 bytes from the gyro

  while(Wire.available() < 1);                       // Wait until the 6 bytes are received
  if(Wire.read() != 0x08){                           // Check if the value is 0x08
    digitalWrite(13,HIGH);                           // If not, turn on the warning LED
    while(1)delay(10);                               // Stay in this loop for ever
  }
}
