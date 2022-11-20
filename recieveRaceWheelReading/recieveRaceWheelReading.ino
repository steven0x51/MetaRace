#include <Servo.h>
#include <Wire.h>

// DAC variables
#define MCP4725_ADDR 0x60  // DAC I2C address 
#define MCP4725_ADDR_0 0x60  // DAC I2C address 
#define MCP4725_ADDR_1 0x61  // DAC I2C address 

// Pin define
#define REVERSE_INDICATOR 2
#define NEUTRAL_INDICATOR 3
#define DRIVE_INDICATOR 4

#define THROTTLE_INDICATOR 11

// Serial input variables
#define SERIAL_BUFF_SIZE 4
byte buff[SERIAL_BUFF_SIZE];
char wheel;
unsigned char throttle;
unsigned char brake;
byte button;

// Contral variables
#define REVERSE 0
#define NEUTRAL 1
#define DRIVE 2
#define SPORT 3
Servo steeringServo;
int steering;
unsigned int motor;
int gear = NEUTRAL;
int preGearCommand = 0;


// temp debugs
// ...
void setup() {
  pinMode(REVERSE_INDICATOR, OUTPUT);
  pinMode(NEUTRAL_INDICATOR, OUTPUT);
  pinMode(DRIVE_INDICATOR, OUTPUT);
  pinMode(THROTTLE_INDICATOR, OUTPUT);
  
//  steeringServo.attach(9);
  Serial.begin(9600);
}

void loop() {  
  if(Serial.available() >= SERIAL_BUFF_SIZE){
    Serial.readBytes(buff, SERIAL_BUFF_SIZE);
  
    wheel = buff[0];
    throttle = buff[1];
    brake = buff[2];
    button = buff[3];

    if ((preGearCommand != (button & 0x03)) && (throttle < 5) ){
      if((button & 0x03) == 0x03 && brake ) {gear = SPORT;}
      else if((button & 0x03) == 0x01 && gear > REVERSE ) {gear--;}
      else if((button & 0x03) == 0x02 && gear < DRIVE) {gear++;}
      preGearCommand = button & 0x03;
    }
    
    //digitalWrite(REVERSE_INDICATOR, reversing);
    analogWrite(THROTTLE_INDICATOR, throttle);
    
    steering = map(wheel, 50, -50,0,4095);  // wheel
//    steering = map(wheel, 125, -125,0,4095);  // joysticker
    steering = constrain(steering, 0, 4095);
    Wire.beginTransmission(MCP4725_ADDR_0);
    Wire.write(64);                     // cmd to update the DAC
    Wire.write(steering >> 4);        // the 8 most significant bits...
    Wire.write(steering << 4); // the 4 least significant bits...
    Wire.endTransmission();

    if (gear == REVERSE){
      digitalWrite(REVERSE_INDICATOR, HIGH);
      digitalWrite(NEUTRAL_INDICATOR, LOW);
      digitalWrite(DRIVE_INDICATOR, LOW);
      motor = map(throttle, 0, 255, 2048, 1750); // range 0.5-0.14 (1750, 600)
      motor = constrain(motor, 1750, 2048);
    }
    if (gear == NEUTRAL){
      digitalWrite(REVERSE_INDICATOR, LOW);
      digitalWrite(NEUTRAL_INDICATOR, HIGH);
      digitalWrite(DRIVE_INDICATOR, LOW);
      motor = 2048;
    }
    if(gear == DRIVE){
      digitalWrite(REVERSE_INDICATOR, LOW);
      digitalWrite(NEUTRAL_INDICATOR, LOW);
      digitalWrite(DRIVE_INDICATOR, HIGH);
      motor = map(throttle, 0, 255, 2500, 3500); // range 0.5-1.0
      motor = constrain(motor, 2500, 2800);
    }
    if(gear == SPORT){
      digitalWrite(REVERSE_INDICATOR, HIGH);
      digitalWrite(NEUTRAL_INDICATOR, HIGH);
      digitalWrite(DRIVE_INDICATOR, HIGH);
      motor = map(throttle, 0, 255, 2500, 3500); // range 0.2-0.4
      motor = constrain(motor, 2460, 4095);
    }

    if(brake){
      motor = 2048;
    }
    Wire.beginTransmission(MCP4725_ADDR_1);
    Wire.write(64);                     // cmd to update the DAC
    Wire.write(motor >> 4);        // the 8 most significant bits...
    Wire.write(motor << 4); // the 4 least significant bits...
    Wire.endTransmission();
    
  }
}
