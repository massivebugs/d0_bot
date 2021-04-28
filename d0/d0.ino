/* I2Cdev library license goes here */

/*
 * Command format
 * size: 5 bytes
 * 1st byte - travel direction (f, r, s)
 * 2nd byte - travel speed ()
 * 3rd byte - turn direction (c, x)
 * 4th byte - turn speed (1~255)
 * 5th byte - imu recv ('0', '1') for now!
 * 
 * eg. 0x66(f)0xB4(180)0x78(x)0x14(20) 
 */

/*
 * Sensor Data format
 * size: ??
 * 1st byte - 
 * 
 */

// IMU library
/*
    MPU6050 Triple Axis Gyroscope & Accelerometer. Pitch & Roll & Yaw Gyroscope Example.
    Read more: http://www.jarzebski.pl/arduino/czujniki-i-sensory/3-osiowy-zyroskop-i-akcelerometr-mpu6050.html
    GIT: https://github.com/jarzebski/Arduino-MPU6050
    Web: http://www.jarzebski.pl
    (c) 2014 by Korneliusz Jarzebski
*/

// Servo
#include <Servo.h>
#define L_SERVO 5
#define R_SERVO 6

// IMU library
#include <Wire.h>
#include <MPU6050.h>

// Software Serial library
#include <SoftwareSerial.h>

// Servo setup
Servo l_servo;
Servo r_servo;

// IMU setup
MPU6050 mpu;

// Timers
unsigned long timer = 0;
float timeStep = 0.01;

// Pitch, Roll and Yaw values
float pitch = 0;
float roll = 0;
float yaw = 0;

// HC-05 (BT2.0) Software Serial setup
#define RX_PIN 8
#define TX_PIN 7
#define COMMAND_LENGTH 5
byte in_buffer[COMMAND_LENGTH];
SoftwareSerial BT_Serial(RX_PIN, TX_PIN);

// Commands
#define FORWARD 'f'
#define REVERSE 'r'
#define    STOP 's'
#define      CW 'c'
#define     CCW 'x'
#define     IMU '1'

// Sending Data
bool send_IMU_data = false;
unsigned long serial_timer = 0; // for sending data every 100ms

// Debug LED
#define LED_PIN 13

void setup() 
{
    BT_Serial.begin(9600);
    Serial.begin(9600);

    // Initialize MPU6050
    while(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
    {
        BT_Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
        delay(500);
    }
  
    // Calibrate gyroscope. The calibration must be at rest.
    // If you don't want calibrate, comment this line.
    mpu.calibrateGyro();

    // Set threshold sensivty. Default 3.
    // If you don't want use threshold, comment this line or set 0.
    mpu.setThreshold(3);

    pinMode(LED_PIN, OUTPUT);
    timer = millis();
    BT_Serial.println(F("Dicynet D0 (T3) is ready!"));
}

void loop()
{   
    timer = millis();
      
    if (BT_Serial.available() > 0)  
    {
        // Receive command from processor
        BT_Serial.readBytes(in_buffer, COMMAND_LENGTH + 2); // for crlf
        // Parse command
        parseCommand(in_buffer);
    }

    // Read normalized values
    Vector norm = mpu.readNormalizeGyro();

    //  Calculate Pitch, Roll and Yaw
    pitch = pitch + norm.YAxis * timeStep;
    roll = roll + norm.XAxis * timeStep;
    yaw = yaw + norm.ZAxis * timeStep;

    // Send data every 100ms
    if ((millis() - serial_timer) > 100)
    {
        Serial.print(yaw);
        Serial.print(" | ");
        Serial.print(pitch);
        Serial.print(" | ");
        Serial.println(roll);
        serial_timer = millis();
    }

    // Send data every 100ms
    // send_data();

    // Wait to full timeStep period
    delay((timeStep*1000) - (millis() - timer));
}

void send_data() {        
    if ((millis() - serial_timer) > 100)
    {
      // IMU data?
      if (send_IMU_data) 
      {
        BT_Serial.print(yaw);
        BT_Serial.print(" | ");
        BT_Serial.print(pitch);
        BT_Serial.print(" | ");
        BT_Serial.println(roll);
      }
      
      serial_timer = millis();
    }
}

void parseCommand(byte in_buffer[COMMAND_LENGTH])
{
    bool stopParse = false;
  
     // Travel
     switch (in_buffer[0])
    {
         case FORWARD:
             BT_Serial.println("FORWARD");
             l_servo.write(120);
             r_servo.write(120);
             break;
         case REVERSE:
             BT_Serial.println("REVERSE");
             l_servo.write(60);
             r_servo.write(60);
             break;
         case STOP:
             BT_Serial.println("STOP");
             l_servo.write(90);
             r_servo.write(90);
             stopParse = true;
             break;
         default:
             // no matching command
             errorWrite("Error in Travel Direction");
             stopParse = true;
    }

    if (stopParse) return;
    
     // Travel Speed
    if (in_buffer[1] < 0 || in_buffer[1] > 255) 
    {
        errorWrite("Error in Travel Speed");
        stopParse = true;
    }
    else
    {
        BT_Serial.println(in_buffer[1], DEC);
    }

    if (stopParse) return;

    // Turn
    switch (in_buffer[2])
    {
        case CW:
            BT_Serial.println("CC");
            break;
        case CCW:
            BT_Serial.println("CCW");
            break;
        default:
            // no matching command
           errorWrite("Error in Turn Direction");
           stopParse = true;
    }

    if (stopParse) return;

    // Turn speed
    if (in_buffer[3] < 0 || in_buffer[3] > 255) 
    {
        errorWrite("Error in Turn Speed");
        stopParse = true;
    }
    else
    {
        BT_Serial.println(in_buffer[3], DEC);
    }

    send_IMU_data = in_buffer[4] == IMU ? true : false;
}

void errorWrite(char* error_msg)
{
    BT_Serial.println(error_msg);    
}

void debugLED(bool toggle)
{
    digitalWrite(LED_PIN, toggle ? HIGH : LOW);
}
