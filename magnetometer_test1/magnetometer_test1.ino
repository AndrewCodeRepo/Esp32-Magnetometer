#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include <MPU9250_WE.h>

#define MPU9250_1_ADDR 0x68
//#define MPU9250_2_ADDR 0x68
#define MPU9250_2_ADDR 0x69

#define MPU9250_3_ADDR 0x68
#define MPU9250_4_ADDR 0x69


#define I2C1_SDA_PIN 21
#define I2C1_SCL_PIN 22

//Set pins for I2C2
#define I2C2_SDA_PIN 18
#define I2C2_SCL_PIN 19

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

//TwoWire I2C1 = TwoWire(0); //I2C1 bus
//TwoWire I2C2 = TwoWire(1); //I2C2 bus

struct magCalibrateData {
  boolean calibrateFlag;
  const uint32_t magAvePeriod = 10;
  uint32_t magAveCounter;
  
  float magXSum;
  float magYSum;
  float magZSum;
  float magXAverage;
  float magYAverage;
  float magZAverage;
};

void initializeMag(struct magCalibrateData *magCalibrateDataIn);
void calibrateMag(struct magCalibrateData *magCalibrateDataIn, xyzFloat magCurIn);


//MPU9250_WE myMPU9250 = MPU9250_WE(MPU9250_ADDR);
//MPU9250_WE myMPU9250_1 = MPU9250_WE(&I2C1, MPU9250_1_ADDR);
MPU9250_WE myMPU9250_1 = MPU9250_WE(&Wire, MPU9250_1_ADDR);

//MPU9250_WE myMPU9250_2 = MPU9250_WE(&I2C2, MPU9250_2_ADDR);
//MPU9250_WE myMPU9250_2 = MPU9250_WE(&I2C1, MPU9250_2_ADDR);
MPU9250_WE myMPU9250_2 = MPU9250_WE(&Wire, MPU9250_2_ADDR);

MPU9250_WE myMPU9250_3 = MPU9250_WE(&Wire1, MPU9250_3_ADDR);

MPU9250_WE myMPU9250_4 = MPU9250_WE(&Wire1, MPU9250_4_ADDR);


// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
//Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &I2C1, -1);


struct magCalibrateData mag1Filter;
struct magCalibrateData mag2Filter;
struct magCalibrateData mag3Filter;

const int calibrateSwitchPin = 32;
int calibrateSwitchState = 0;

void setup() {
  Serial.begin(115200);
  
  Wire.begin();
  //I2C1.begin();

  Wire1.begin(I2C2_SDA_PIN, I2C2_SCL_PIN);     // I don't know the default pins !
  
  if(!myMPU9250_1.init()){
    Serial.println("MPU9250 1 does not respond");
  }
  else{
    Serial.println("MPU9250 1 is connected");
  }
  if(!myMPU9250_1.initMagnetometer()){
    Serial.println("Magnetometer 1 does not respond");
  }
  else{
    Serial.println("Magnetometer 1 is connected");
  }

  delay(100);

  //I2C2.begin();
  
  if(!myMPU9250_2.init()){
    Serial.println("MPU9250 2 does not respond");
  }
  else{
    Serial.println("MPU9250 2 is connected");
  }
  if(!myMPU9250_2.initMagnetometer()){
    Serial.println("Magnetometer 2 does not respond");
  }
  else{
    Serial.println("Magnetometer 2 is connected");
  }

  delay(100);
  
  if(!myMPU9250_3.init()){
    Serial.println("MPU9250 3 does not respond");
  }
  else{
    Serial.println("MPU9250 3 is connected");
  }
  if(!myMPU9250_3.initMagnetometer()){
    Serial.println("Magnetometer 3 does not respond");
  }
  else{
    Serial.println("Magnetometer 3 is connected");
  }

  delay(100);

  /*
  if(!myMPU9250_4.init()){
    Serial.println("MPU9250 4 does not respond");
  }
  else{
    Serial.println("MPU9250 4 is connected");
  }
  if(!myMPU9250_4.initMagnetometer()){
    Serial.println("Magnetometer 4 does not respond");
  }
  else{
    Serial.println("Magnetometer 4 is connected");
  }

  delay(100);
  */

  /* You can choose the following operational modes
   * AK8963_PWR_DOWN            power down (default)
   * AK8963_CONT_MODE_8HZ       continuous at 8Hz sample rate
   * AK8963_CONT_MODE_100HZ     continuous at 100Hz sample rate 
   * 
   * In trigger mode the AK8963 goes into power down after the measurement
   */
  myMPU9250_1.setMagOpMode(AK8963_CONT_MODE_100HZ);
  
  /* In continuous mode you need to wait for the first data to be available. If you 
   * comment the line below you will probably obtain zero. 
   */
  delay(200);

  myMPU9250_2.setMagOpMode(AK8963_CONT_MODE_100HZ);

  delay(200);

  myMPU9250_3.setMagOpMode(AK8963_CONT_MODE_100HZ);

  delay(200);

  /*
  myMPU9250_4.setMagOpMode(AK8963_CONT_MODE_100HZ);

  delay(200);
  */

  pinMode(calibrateSwitchPin, INPUT);

  initializeMag(&mag1Filter);
  initializeMag(&mag2Filter);
  initializeMag(&mag3Filter);
}

void loop() {
  xyzFloat magValue1 = myMPU9250_1.getMagValues(); // returns magnetic flux density [µT] 

  delay(100);

  xyzFloat magValue2 = myMPU9250_2.getMagValues(); // returns magnetic flux density [µT] 

  delay(100);

  xyzFloat magValue3 = myMPU9250_3.getMagValues(); // returns magnetic flux density [µT] 

  delay(100);

  //xyzFloat magValue4 = myMPU9250_4.getMagValues(); // returns magnetic flux density [µT] 

  //delay(100);

  calibrateSwitchState = digitalRead(calibrateSwitchPin);

  if(calibrateSwitchState == 1){
    
    Serial.println("The calibrate switch state is high");
    Serial.println("");
    
    mag1Filter.calibrateFlag = true;
    mag2Filter.calibrateFlag = true;
    mag3Filter.calibrateFlag = true;
  }
  else{
    Serial.println("The calibrate switch state is low");
    Serial.println("");
  }

  if(mag1Filter.calibrateFlag == true){
    calibrateMag(&mag1Filter, magValue1);
  }

  if(mag2Filter.calibrateFlag == true){
    calibrateMag(&mag2Filter, magValue2);
  }

  if(mag3Filter.calibrateFlag == true){
    calibrateMag(&mag3Filter, magValue3);
  }

  Serial.println("Magnetometer 1 Data in µTesla: ");
  Serial.print(magValue1.x);
  Serial.print("   ");
  Serial.print(magValue1.y);
  Serial.print("   ");
  Serial.println(magValue1.z);

  Serial.println("Magnetometer 1 Data calibrated in µTesla: ");
  Serial.print(magValue1.x - mag1Filter.magXAverage);
  Serial.print("   ");
  Serial.print(magValue1.y - mag1Filter.magYAverage);
  Serial.print("   ");
  Serial.println(magValue1.z - mag1Filter.magZAverage);
  Serial.println("");


  Serial.println("Magnetometer 2 Data in µTesla: ");
  Serial.print(magValue2.x);
  Serial.print("   ");
  Serial.print(magValue2.y);
  Serial.print("   ");
  Serial.println(magValue2.z);

  Serial.println("Magnetometer 2 Data calibrated in µTesla: ");
  Serial.print(magValue2.x - mag2Filter.magXAverage);
  Serial.print("   ");
  Serial.print(magValue2.y - mag2Filter.magYAverage);
  Serial.print("   ");
  Serial.println(magValue2.z - mag2Filter.magZAverage);
  Serial.println("");


  Serial.println("Magnetometer 3 Data in µTesla: ");
  Serial.print(magValue3.x);
  Serial.print("   ");
  Serial.print(magValue3.y);
  Serial.print("   ");
  Serial.println(magValue3.z);

  Serial.println("Magnetometer 3 Data calibrated in µTesla: ");
  Serial.print(magValue3.x - mag3Filter.magXAverage);
  Serial.print("   ");
  Serial.print(magValue3.y - mag3Filter.magYAverage);
  Serial.print("   ");
  Serial.println(magValue3.z - mag3Filter.magZAverage);
  Serial.println("");

  /*
  Serial.println("Magnetometer 4 Data in µTesla: ");
  Serial.print(magValue4.x);
  Serial.print("   ");
  Serial.print(magValue4.y);
  Serial.print("   ");
  Serial.println(magValue4.z);

  Serial.println("Magnetometer 4 Data calibrated in µTesla: ");
  Serial.print(magValue4.x - mag4XAverage);
  Serial.print("   ");
  Serial.print(magValue4.y - mag4YAverage);
  Serial.print("   ");
  Serial.println(magValue4.z - mag4ZAverage);
  Serial.println("");
  */


  delay(2000);
}


void initializeMag(struct magCalibrateData *magCalibrateDataIn) {
  magCalibrateDataIn->calibrateFlag = false;
  magCalibrateDataIn->magAveCounter = 0;
  
  magCalibrateDataIn->magXSum = 0.0;
  magCalibrateDataIn->magYSum = 0.0;
  magCalibrateDataIn->magZSum = 0.0;

  magCalibrateDataIn->magXAverage = 0.0;
  magCalibrateDataIn->magYAverage = 0.0;
  magCalibrateDataIn->magZAverage = 0.0;
}

void calibrateMag(struct magCalibrateData *magCalibrateDataIn, xyzFloat magCurIn) {
  magCalibrateDataIn->magAveCounter += 1;
  
  magCalibrateDataIn->magXSum += magCurIn.x;
  magCalibrateDataIn->magYSum += magCurIn.y;
  magCalibrateDataIn->magZSum += magCurIn.z; 
   
  if(magCalibrateDataIn->magAveCounter >= magCalibrateDataIn->magAvePeriod){

    magCalibrateDataIn->magXAverage = magCalibrateDataIn->magXSum;
    magCalibrateDataIn->magYAverage = magCalibrateDataIn->magYSum;
    magCalibrateDataIn->magZAverage = magCalibrateDataIn->magZSum;
    
    magCalibrateDataIn->magXAverage /= float(magCalibrateDataIn->magAveCounter);
    magCalibrateDataIn->magYAverage /= float(magCalibrateDataIn->magAveCounter);
    magCalibrateDataIn->magZAverage /= float(magCalibrateDataIn->magAveCounter);

    magCalibrateDataIn->calibrateFlag = false;

    magCalibrateDataIn->magAveCounter = 0;
    
    magCalibrateDataIn->magXSum = 0.0;
    magCalibrateDataIn->magYSum = 0.0;
    magCalibrateDataIn->magZSum = 0.0;
    
  }
}
