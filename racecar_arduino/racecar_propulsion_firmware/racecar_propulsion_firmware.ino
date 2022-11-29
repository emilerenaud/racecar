//=========================HEADER=============================================================
// Firmware for the Arduino managing the propulsion of the slash platform (UdeS Racecar)
//============================================================================================

/////////////////////////////////////////////////////////////////
// Includes
///////////////////////////////////////////////////////////////////

#include "Arduino.h"
#include <SPI.h>
#include <Servo.h> 
#define USB_USBCON
#include <ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>

// For version including 
#include "MPU9250.h" 

///////////////////////////////////////////////////////////////////
// Init I/O
///////////////////////////////////////////////////////////////////

// Servo objects for PWM control of
// Sterring servo-motor
Servo steeringServo;

// IMU
MPU9250 imu(Wire, 0x68);

// ROS
ros::NodeHandle  nodeHandle;

//Publisher 
const int prop_sensors_msg_length = 19;
float prop_sensors_data[ prop_sensors_msg_length ];
std_msgs::Float32MultiArray prop_sensors_msg;
ros::Publisher prop_sensors_pub("prop_sensors", &prop_sensors_msg);

// Serial Communication
const unsigned long baud_rate = 115200;

// Slave Select pins for the encoder
const int slaveSelectEnc = 45;

// Pins for outputs PWM
const int ser_pin = 9;      // Servo 

// Custom drive
const int dri_pwm_pin     = 6 ;  // H bridge drive pwm
const int dri_dir_pin     = 42; //

///////////////////////////////////////////////////////////////////
// Parameters
///////////////////////////////////////////////////////////////////

// Controller

//TODO: VOUS DEVEZ DETERMINEZ DES BONS PARAMETRES SUIVANTS
const float filter_rc  =  0.1;
const float vel_kp     =  100.0;
const float vel_ki     =  5.0;
//const float vel_kd     =  50.0;
const float pos_kp     =  160.0;
const float pos_kd     =  70.0;
const float pos_ki     =  4.0;
const float pos_ei_sat =  10.0;
const float prct_acpt_err = 0.01; // Percent of acceptable error (when to reset error integral)

// Loop period 
const unsigned long time_period_low   = 2;    // 500 Hz for internal PID loop
const unsigned long time_period_high  = 10;   // 100 Hz  for ROS communication
const unsigned long time_period_com   = 1000; // 1000 ms = max com delay (watchdog)

// Hardware min-zero-max range for the steering servo and the drive
const int pwm_min_ser = 30  ;
const int pwm_zer_ser = 90  ;
const int pwm_max_ser = 150 ;
const int pwm_min_dri = -511;
const int pwm_zer_dri = 0;
const int pwm_max_dri = 511;

const int dri_wakeup_time = 20; // micro second

// Units Conversion
const double batteryV  = 8;
const double maxAngle  = 40*(2*3.1416)/360;    //max steering angle in rad
const double rad2pwm   = (pwm_zer_ser-pwm_min_ser)/maxAngle;
const double volt2pwm  = (pwm_zer_dri-pwm_min_dri)/batteryV;
const double tick2m    = 0.000002600; // To confirm

///////////////////////////////////////////////////////////////////
// Memory
///////////////////////////////////////////////////////////////////

// Inputs
float ser_ref    = 0; //rad
float dri_ref    = 0; //volt
int ctl_mode     = 0; // discrete control mode
int dri_standby  = 0; 

// Ouputs
int ser_pwm   = 0;
int   dri_pwm = 0;
float dri_cmd = 0;

// Controller memory (differentiation, filters and integral actions)
signed long enc_now   = 0;
signed long enc_old   = 0;

float pos_now   = 0;
float vel_now   = 0;
float vel_old   = 0;

float vel_error_int = 0 ;
float pos_error_int = 0;
float pos_error_old = 0;

// Loop timing
unsigned long time_now       = 0;
unsigned long time_last_low  = 0;
unsigned long time_last_high = 0;
unsigned long time_last_com  = 0; //com watchdog

unsigned long dt_real = 0;
signed long ticks_dt = 0;

float pos_error_ddt_prev = 0.0;
float pos_ref, pos_error, pos_error_ddt;

// For odometry
signed long enc_last_high = 0;

///////////////////////////////////////////////////////////////////
// Encoder init/read/reset functions
///////////////////////////////////////////////////////////////////

//////////////////////////////////////
void initEncoder() {
  
  // Set slave selects as outputs
  pinMode(slaveSelectEnc, OUTPUT);
  
  // Raise select pin
  // Communication begins when you drop the individual select signsl
  digitalWrite(slaveSelectEnc,HIGH);
  
  SPI.begin();
  
  // Initialize encoder 
  //    Clock division factor: 0
  //    Negative index input
  //    free-running count mode
  //    x4 quatrature count mode (four counts per quadrature cycle)
  // NOTE: For more information on commands, see datasheet
  digitalWrite(slaveSelectEnc,LOW);        // Begin SPI conversation
  SPI.transfer(0x88);                       // Write to MDR0
  SPI.transfer(0x03);                       // Configure to 4 byte mode
  digitalWrite(slaveSelectEnc,HIGH);       // Terminate SPI conversation 
}

//////////////////////////////////////
long readEncoder() {
  
  // Initialize temporary variables for SPI read
  unsigned int count_1, count_2, count_3, count_4;
  long count_value;  
  
  // Read encoder
  digitalWrite(slaveSelectEnc,LOW);       // Begin SPI conversation
  SPI.transfer(0x60);                     // Request count
  count_1 = SPI.transfer(0x00);           // Read highest order byte
  count_2 = SPI.transfer(0x00);           
  count_3 = SPI.transfer(0x00);           
  count_4 = SPI.transfer(0x00);           // Read lowest order byte
  digitalWrite(slaveSelectEnc,HIGH);      // Terminate SPI conversation 
  
  // Calculate encoder count
  count_value = (count_1 << 8) + count_2;
  count_value = (count_value << 8) + count_3;
  count_value = (count_value << 8) + count_4;
  
  return count_value;
}

//////////////////////////////////////
void clearEncoderCount() {
    
  // Set encoder1's data register to 0
  digitalWrite(slaveSelectEnc,LOW);      // Begin SPI conversation  
  // Write to DTR
  SPI.transfer(0x98);    
  // Load data
  SPI.transfer(0x00);  // Highest order byte
  SPI.transfer(0x00);           
  SPI.transfer(0x00);           
  SPI.transfer(0x00);  // lowest order byte
  digitalWrite(slaveSelectEnc,HIGH);     // Terminate SPI conversation 
  
  delayMicroseconds(100);  // provides some breathing room between SPI conversations
  
  // Set encoder1's current data register to center
  digitalWrite(slaveSelectEnc,LOW);      // Begin SPI conversation  
  SPI.transfer(0xE0);    
  digitalWrite(slaveSelectEnc,HIGH);     // Terminate SPI conversation   
}


///////////////////////////////////////////////////////////////////
// Convertion functions
///////////////////////////////////////////////////////////////////

// Convertion function : Servo Angle --> PWM
double ser2pwm (double cmd) {
  
  // Scale and offset
  double pwm_d = cmd * rad2pwm + (double) pwm_zer_ser;
  
  // Rounding and conversion
  int pwm = (int) ( pwm_d + 0.5 );
  
  // Saturations
  if (pwm > pwm_max_ser) {
    pwm = pwm_max_ser;
  }
  if (pwm < pwm_min_ser) { 
    pwm = pwm_min_ser;
  }
  
  return pwm;
}

// Convertion function : Volt Command --> PWM
double cmd2pwm (double cmd) {

  int pwm = (int) ( cmd / batteryV * pwm_max_dri  + 0.5 );
  
  // Saturations
  if (pwm < pwm_min_dri) { 
    pwm = pwm_min_dri;
  }
  if (pwm > pwm_max_dri) {
    pwm = pwm_max_dri;
  }  
  
  return pwm;
}


///////////////////////////////////////////////////////////////////
// Set PWM value
///////////////////////////////////////////////////////////////////
void set_pwm( int pwm ){
  
  // Zero cmd
  if ( pwm == 0 ){
    digitalWrite(dri_pwm_pin, LOW);
    dri_standby = 1;
  }
  
  // Non-zero PWM
  else{
    
    // Wake-up PWM if if needed
    if (dri_standby == 1) {
      digitalWrite(dri_pwm_pin, HIGH);
      delayMicroseconds(dri_wakeup_time);
      dri_standby = 0;
    }
    
    // PWM direction
    if ( pwm < 0 ){
      digitalWrite(dri_dir_pin, HIGH);
    }
    else {
      digitalWrite(dri_dir_pin, LOW);
    }
    
    // Registery-based pwm duty cycle adjustement
    
    //Fast PWM, 9-bit, prescaler divider = 1
    TCCR4A = _BV(COM2A1) | _BV(COM2B1) | _BV(WGM41);
    TCCR4B = _BV(CS20) | _BV(WGM42);
  
    OCR4A = abs( pwm ) - 1; //set the duty cycle of pin 6
  }
  
}


///////////////////////////////////////////////////////////////////
// Read propulsion command from ROS
///////////////////////////////////////////////////////////////////
void cmdCallback ( const geometry_msgs::Twist&  twistMsg ){
  
  ser_ref  = -twistMsg.angular.z; //rad
  dri_ref  = twistMsg.linear.x;  // volt or m/s or m
  ctl_mode = twistMsg.linear.z;  // 1    or 2   or 3

  time_last_com = millis(); // for watchdog
}

///////////////////////////////////////////////////////////////////
// Controller One tick
///////////////////////////////////////////////////////////////////
void ctl(){
  
  ///////////////////////////////////////////////
  // STEERING CONTROL
  /////////////////////////////////////////////// 
  
  // Servo Open-Loop fonction
  ser_pwm      = ser2pwm( ser_ref ) ;
  steeringServo.write(ser_pwm) ; 
  
  ///////////////////////////////////////////////
  // PROPULSION CONTROL
  /////////////////////////////////////////////// 
  
  // Retrieve current encoder counters
  enc_now = readEncoder();
  
  // Position computation
  pos_now = (float) enc_now * tick2m;
  
  // Velocity computation
  double vel_raw = (enc_now - enc_old) * tick2m / (dt_real / 1000.0);
  float alpha   = dt_real / (filter_rc + dt_real);
  float vel_fil = vel_raw * alpha + (1 - alpha) * vel_old;
  
  // Propulsion Controllers
  
  //////////////////////////////////////////////////////
  if (ctl_mode == 0 ){
    // Zero output
    dri_pwm    = pwm_zer_dri ;
    
    // reset integral actions
    vel_error_int = 0;
    pos_error_int = 0 ;
    
  }
  //////////////////////////////////////////////////////
  else if (ctl_mode == 1 ){
    // Fully Open-Loop
    // Commands received in [Volts] directly
    dri_cmd    = dri_ref;
    dri_pwm    = cmd2pwm( dri_cmd ) ;
    
    // reset integral actions
    vel_error_int = 0;
    pos_error_int = 0;
  }
  //////////////////////////////////////////////////////
  else if (ctl_mode == 2 ){
    // Low-level Velocity control
    // Commands received in [m/sec] setpoints
    
    float vel_ref, vel_error;

    vel_ref       = dri_ref; 
    vel_error     = vel_ref - vel_fil;
    vel_error_int += vel_error * (dt_real / 1000.0);
    if (vel_error < vel_ref * prct_acpt_err)
    {
      vel_error_int = 0;
    }

    // Calculate the command
    dri_cmd = vel_kp * vel_error + vel_ki * vel_error_int;
    
    dri_pwm = cmd2pwm( dri_cmd ) ;

  }
  ///////////////////////////////////////////////////////
  else if (ctl_mode == 3){
    // Low-level Position control
    // Commands received in [m] setpoints

    pos_ref       = dri_ref; 
    pos_error     = pos_ref - pos_now; //remaining distance
    pos_error_int += pos_error * (dt_real / 1000.0);
    pos_error_ddt = (pos_error - pos_error_old) / (dt_real / 1000.0);

    // Filter the derivative
    float pos_error_ddt_fil = pos_error_ddt * alpha + (1 - alpha) * pos_error_ddt_prev;
    pos_error_ddt_prev = pos_error_ddt_fil;
    
    /*
    if (pos_error < pos_ref * prct_acpt_err)
    {
      pos_error_int = 0;
    }
    */

    pos_error_old = pos_error; // we may need to reset pos_error_old at some point
    
    // Anti wind-up
    if ( pos_error_int > pos_ei_sat )
    {
      pos_error_int = pos_ei_sat;
    }
    
    // Calculate the command
    dri_cmd = pos_kp * pos_error + pos_ki * pos_error_int + pos_kd * pos_error_ddt_fil;
    
    dri_pwm = cmd2pwm( dri_cmd ) ;
  }
  ///////////////////////////////////////////////////////
  else if (ctl_mode == 4){
    // Reset encoder counts
    
    clearEncoderCount();
    
    // reset integral actions
    vel_error_int = 0 ;
    pos_error_int = 0 ;
    
    dri_pwm    = pwm_zer_dri ;
  }
  ////////////////////////////////////////////////////////
  else if (ctl_mode == 5){
    // APP4 Closed-loop speed
    // Instead of using V calculated on the Pi, we calculate it here

    // V = N11 * vref - K12 * v;
    dri_cmd = 10.1874 * dri_ref - 8.2422 * vel_fil;
    dri_pwm = cmd2pwm(dri_cmd);
  }
  ////////////////////////////////////////////////////////
  else {
    // reset integral actions
    vel_error_int = 0 ;
    pos_error_int = 0 ;
    
    dri_pwm    = pwm_zer_dri ;
  }
  ///////////////////////////////////////////////////////
  
  // H-bridge pwm update
  set_pwm(dri_pwm);
  
  //Update memory variable
  enc_old = enc_now;
  vel_old = vel_fil;
}


// ROS suscriber
ros::Subscriber<geometry_msgs::Twist> cmdSubscriber("prop_cmd", &cmdCallback) ;


///////////////////////////////////////////////////////////////////
// Arduino Initialization
///////////////////////////////////////////////////////////////////
void setup(){
  
  // Init PWM output Pins
  steeringServo.attach(ser_pin); 
  pinMode(dri_dir_pin, OUTPUT);
  pinMode(dri_pwm_pin, OUTPUT);
  
  // Init Communication
  nodeHandle.getHardware()->setBaud(baud_rate);

  // Init and Clear Encoders
  initEncoder();    
  clearEncoderCount(); 
  
  // Init ROS
  nodeHandle.initNode();
  nodeHandle.subscribe(cmdSubscriber) ; // Subscribe to the steering and throttle messages
  nodeHandle.advertise(prop_sensors_pub);
  
  // Initialize Steering and drive cmd to neutral
  steeringServo.write(pwm_zer_ser) ;
  set_pwm(0);

  // Initialize IMU
  imu.begin();
  imu.setAccelRange(MPU9250::ACCEL_RANGE_2G);
  imu.setGyroRange(MPU9250::GYRO_RANGE_250DPS);
  imu.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_41HZ);
  imu.setSrd(9); //100 Hz update rate
  
  //
  delay(3000) ;
  
  nodeHandle.spinOnce();

}


////////////////////////////////////////////////////////////////////
//  Main Control Loop
////////////////////////////////////////////////////////////////////
void loop(){
  
  time_now = millis();

  /////////////////////////////////////////////////////////////
  // Watchdog: stop the car if no recent communication from ROS
  //////////////////////////////////////////////////////////////

  if (( time_now - time_last_com ) > time_period_com ) {
    
    // All-stop
    dri_ref  = 0;  // velocity set-point
    ctl_mode = 2;  // closed-loop velocity mode
    
  }

  ////////////////////////////////////////
  // Low-level controller
  ///////////////////////////////////////
  dt_real = time_now-time_last_low;
  if ((dt_real) > time_period_low ) {
    
    ctl(); // one control tick

    time_last_low = time_now ;
  }

  ////////////////////////////////////////
  // Sync with ROS high-level controller
  ///////////////////////////////////////

  unsigned long dt = time_now - time_last_high;
  if (dt > time_period_high ) {

    // Feedback loop
    prop_sensors_data[0] = pos_now; // wheel position in m
    prop_sensors_data[1] = vel_old; // wheel velocity in m/sec
    
    // For DEBUG
    prop_sensors_data[2] = dri_ref; // set point received by arduino
    prop_sensors_data[3] = dri_cmd; // drive set point in volts
    prop_sensors_data[4] = dri_pwm; // drive set point in pwm
    prop_sensors_data[5] = pos_error; // raw encoder counts
    prop_sensors_data[6] = ser_ref; // steering angle
    prop_sensors_data[7] = (float)( time_now - time_last_com ); // for com debug
    prop_sensors_data[8] = (float)dt; // time elapsed since last publish
    prop_sensors_data[9] = (enc_now - enc_last_high) * tick2m; // distance travelled since last publish

    // Read IMU
    imu.readSensor();
    prop_sensors_data[10] = imu.getAccelX_mss();
    prop_sensors_data[11] = imu.getAccelY_mss();
    prop_sensors_data[12] = imu.getAccelZ_mss();
    prop_sensors_data[13] = imu.getGyroX_rads();
    prop_sensors_data[14] = imu.getGyroY_rads();
    prop_sensors_data[15] = imu.getGyroZ_rads();
    prop_sensors_data[16] = imu.getMagX_uT();
    prop_sensors_data[17] = imu.getMagY_uT();
    prop_sensors_data[18] = imu.getMagZ_uT();
    
    prop_sensors_msg.data        = &prop_sensors_data[0];
    prop_sensors_msg.data_length = prop_sensors_msg_length;
    prop_sensors_pub.publish( &prop_sensors_msg );
    
    // Process ROS Events
    nodeHandle.spinOnce();

    time_last_high = time_now ;
    enc_last_high = enc_now ;

  }
}
