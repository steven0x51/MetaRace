#include "MPU9250.h" // Bolder Flight System library
#include <math.h>

// DAC variables
#define MCP4725_ADDR 0x60  // DAC I2C address 

MPU9250 imu(Wire,0x68);;

float lowPassWeight = 0.80;
float gyroWeight = 0.95;
float magWeight = 0.95;

float rawAccX, rawAccY, rawAccZ;
float rawGyroX, rawGyroY, rawGyroZ;
float rawMagX, rawMagY, rawMagZ;
float filteredAccX, filteredAccY, filteredAccZ;
float filteredMagX, filteredMagY, filteredMagZ;
float compMagX, compMagY;
float pitch, roll, yaw = 0.0;
float magHeading;
int cameraPosition;

uint32_t prevTime;
float dt;
int MAGCAL_INDICAT_PIN = 4;
int toggle = 0;
void setup() {
//    Serial.begin(9600);
    pinMode(MAGCAL_INDICAT_PIN, OUTPUT);
    Wire.begin();
    delay(100);
    
    if (imu.begin() < 0) {  // change to your own address
      Serial.println("IMU initialization unsuccessful");
      Serial.println("Check IMU wiring or try cycling power");
      while(1) {}
    }
  imu.calibrateMag();
//  imu.setMagCalX(imu.getMagBiasX_uT(),imu.getMagScaleFactorX());
//  imu.setMagCalY(imu.getMagBiasY_uT(),imu.getMagScaleFactorY());
//  imu.setMagCalZ(imu.getMagBiasZ_uT(),imu.getMagScaleFactorZ());

  for(int i = 0; i < 10; i++){
    digitalWrite(MAGCAL_INDICAT_PIN, toggle);
    toggle = !toggle;
    delay(500);
  }
  imu.readSensor();
  magHeading = atan2(imu.getMagX_uT(), imu.getMagY_uT());
  prevTime = millis();
}

void loop() {
  
  imu.readSensor();
  rawAccX =  imu.getAccelY_mss();
  rawAccY =  imu.getAccelX_mss();
  rawAccZ =  -imu.getAccelZ_mss();

  rawGyroX = imu.getGyroY_rads();
  rawGyroY = -imu.getGyroX_rads();
  rawGyroZ = imu.getGyroZ_rads();

  rawMagX = imu.getMagX_uT();
  rawMagY = imu.getMagY_uT();
  rawMagZ = imu.getMagZ_uT();

  // low pass filter
  filteredAccX = lowPassWeight * filteredAccX + (1 - lowPassWeight) * rawAccX;
  filteredAccY = lowPassWeight * filteredAccY + (1 - lowPassWeight) * rawAccY;
  filteredAccZ = lowPassWeight * filteredAccZ + (1 - lowPassWeight) * rawAccZ;

  filteredMagX = lowPassWeight * filteredMagX + (1 - lowPassWeight) * rawMagX;
  filteredMagY = lowPassWeight * filteredMagY + (1 - lowPassWeight) * rawMagY;
  filteredMagZ = lowPassWeight * filteredMagZ + (1 - lowPassWeight) * rawMagZ;

  // complementary filter in (rad)
  dt = (millis() - prevTime) / 1000.0;

  roll = gyroWeight * (roll + rawGyroY * dt) + (1 - gyroWeight) * atan2(filteredAccX, filteredAccZ);
  pitch = gyroWeight * (pitch + rawGyroX * dt) + (1 - gyroWeight) * atan2(filteredAccY, filteredAccZ);
  yaw = gyroWeight * (yaw + rawGyroZ *dt) +  (1 - gyroWeight) * (atan2(filteredMagX, filteredMagY) - magHeading);
  //yaw = (yaw + rawGyroZ *dt);
  // tilt compensation for magnetometer on X and Y axis
//  compMagX = rawMagX * cos(pitch) - rawMagY * sin(roll) * sin(pitch) + rawMagZ * cos(roll) * sin(pitch);
//  compMagY = rawMagY * cos(roll) + rawMagZ * sin(roll);
//  yaw = gyroWeight * (yaw + rawGyroZ *dt) +  (1 - gyroWeight) * atan2(compMagY, compMagX);
//  yaw =  atan2(compMagY, compMagX);
//  yaw =  atan2(rawMagY, rawMagX);
  
  prevTime = millis();

  cameraPosition = map(int(yaw*180/PI), 90, -90, 0, 4095);
  cameraPosition = constrain(cameraPosition, 0, 4095);
  // sending to DAC
  Wire.beginTransmission(MCP4725_ADDR);
  Wire.write(64);                     // cmd to update the DAC
  Wire.write(cameraPosition >> 4);        // the 8 most significant bits...
  Wire.write(cameraPosition << 4); // the 4 least significant bits...
  Wire.endTransmission();
//  Serial.print(cameraPosition);
//  Serial.print(" ");
//  printRoll();
//  printPitch();
  printYaw();
//  printAll();
//  printMag();
}




void printAll(){
  Serial.print(roll *180/PI);
  Serial.print(" ");
  Serial.print(pitch *180/PI);
  Serial.print(" ");
  Serial.println(yaw*180/PI);
}

void printRoll(){
  Serial.print(atan2(rawAccX,rawAccZ)*180/PI);
  Serial.print(" ");
  Serial.print((roll + rawGyroY * dt)*180/PI);
  Serial.print(" ");
  Serial.println(roll *180/PI);
 
 }

 void printPitch(){

  Serial.print(filteredAccY);
  Serial.print(" ");
  Serial.print(rawGyroX);
  Serial.print(" ");
  Serial.println(pitch *180/PI);
 
 }


void printYaw(){
  Serial.print(yaw*180/PI);
  Serial.print(" ");
  Serial.println(magHeading);
}

void printMag(){
  Serial.print(rawMagX);
  Serial.print(" ");
  Serial.print(rawMagY);
  Serial.print(" ");
  Serial.print(rawMagZ);
  Serial.print(" ");
  Serial.println(yaw*180/PI);
}
