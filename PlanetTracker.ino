#include <ESP32Servo.h>
#include <WiFi.h>
#include <LiquidCrystal_I2C.h>
#include <CheapStepper.h>
#include <time.h>
#include <Wire.h>
#include <credentials.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#define LED_BUILTIN 2
#define WHO_AM_I_MPU9250 0x75 // Should return 0x71
#define WHO_AM_I_AK8963  0x00 // should return 0x48
#define MPU9250_ADDRESS 0x68  // Device address when ADO = 1
#define AK8963_ADDRESS 0x0C   //  Address of magnetometer
#define AK8963_ST1       0x02  // data ready status bit 0
#define AK8963_XOUT_L   0x03  // data
#define AK8963_CNTL      0x0A  // Power down (0000), single-measurement (0001), self-test (1000) and Fuse ROM (1111) modes on bits 3:0
#define AK8963_ASAX      0x10  // Fuse ROM x-axis sensitivity adjustment value
#define PWR_MGMT_1       0x6B // Device defaults to the SLEEP mode
#define PWR_MGMT_2       0x6C
#define AHRS true         // set to false for basic data read
#define USER_CTRL        0x6A  // Bit 7 enable DMP, bit 3 reset DMP
#define FIFO_COUNTH      0x72
#define FIFO_R_W         0x74
#define FIFO_EN          0x23
#define XG_OFFSET_H      0x13  // User-defined trim values for gyroscope
#define XG_OFFSET_L      0x14
#define YG_OFFSET_H      0x15
#define YG_OFFSET_L      0x16
#define ZG_OFFSET_H      0x17
#define ZG_OFFSET_L      0x18
#define XA_OFFSET_H      0x77
#define XA_OFFSET_L      0x78
#define YA_OFFSET_H      0x7A
#define YA_OFFSET_L      0x7B
#define ZA_OFFSET_H      0x7D
#define ZA_OFFSET_L      0x7E
#define I2C_MST_CTRL     0x24
#define SMPLRT_DIV       0x19
#define CONFIG           0x1A
#define GYRO_CONFIG      0x1B
#define ACCEL_CONFIG     0x1C
#define ACCEL_CONFIG2    0x1D
#define ACCEL_XOUT_H     0x3B
#define ACCEL_XOUT_L     0x3C
#define ACCEL_YOUT_H     0x3D
#define ACCEL_YOUT_L     0x3E
#define ACCEL_ZOUT_H     0x3F
#define ACCEL_ZOUT_L     0x40
#define TEMP_OUT_H       0x41
#define TEMP_OUT_L       0x42
#define GYRO_XOUT_H      0x43
#define GYRO_XOUT_L      0x44
#define GYRO_YOUT_H      0x45
#define GYRO_YOUT_L      0x46
#define GYRO_ZOUT_H      0x47
#define GYRO_ZOUT_L      0x48
#define INT_STATUS       0x3A
#define INT_ENABLE       0x38
#define INT_PIN_CFG      0x37
#define SELF_TEST_X_GYRO 0x00
#define SELF_TEST_Y_GYRO 0x01
#define SELF_TEST_Z_GYRO 0x02
#define SELF_TEST_X_ACCEL 0x0D
#define SELF_TEST_Y_ACCEL 0x0E
#define SELF_TEST_Z_ACCEL 0x0F
#define SerialDebug true   // set to true to get Serial output for debugging

// MPU
float pitch, yaw, roll;

enum Ascale {
  AFS_2G = 0,
  AFS_4G,
  AFS_8G,
  AFS_16G
};

enum Gscale {
  GFS_250DPS = 0,
  GFS_500DPS,
  GFS_1000DPS,
  GFS_2000DPS
};

enum Mscale {
  MFS_14BITS = 0, // 0.6 mG per LSB
  MFS_16BITS      // 0.15 mG per LSB
};

// Specify sensor full scale
uint8_t Gscale = GFS_250DPS;
uint8_t Ascale = AFS_2G;
uint8_t Mscale = MFS_16BITS; // Choose either 14-bit or 16-bit magnetometer resolution
uint8_t Mmode = 0x02;        // 2 for 8 Hz, 6 for 100 Hz continuous magnetometer data read
float aRes, gRes, mRes;      // scale resolutions per LSB for the sensors

// Pin definitions
int intPin = 12;  // These can be changed, 2 and 3 are the Arduinos ext int pins
int myLed = 13; // Set up pin 13 led for toggling

int16_t accelCount[3];  // Stores the 16-bit signed accelerometer sensor output
int16_t gyroCount[3];   // Stores the 16-bit signed gyro sensor output
int16_t magCount[3];    // Stores the 16-bit signed magnetometer sensor output
float magCalibration[3] = {0, 0, 0}, magbias[3] = {0, 0, 0};  // Factory mag calibration and mag bias
float gyroBias[3] = {0, 0, 0}, accelBias[3] = {0, 0, 0};      // Bias corrections for gyro and accelerometer
int16_t tempCount;      // temperature raw count output
float   temperature;    // Stores the real internal chip temperature in degrees Celsius
float   SelfTest[6];    // holds results of gyro and accelerometer self test

// global constants for 9 DoF fusion and AHRS (Attitude and Heading Reference System)
float GyroMeasError = PI * (40.0f / 180.0f);   // gyroscope measurement error in rads/s (start at 40 deg/s)
float GyroMeasDrift = PI * (0.0f  / 180.0f);   // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
// There is a tradeoff in the beta parameter between accuracy and response speed.
// In the original Madgwick study, beta of 0.041 (corresponding to GyroMeasError of 2.7 degrees/s) was found to give optimal accuracy.
// However, with this value, the LSM9SD0 response time is about 10 seconds to a stable initial quaternion.
// Subsequent changes also require a longish lag time to a stable output, not fast enough for a quadcopter or robot car!
// By increasing beta (GyroMeasError) by about a factor of fifteen, the response time constant is reduced to ~2 sec
// I haven't noticed any reduction in solution accuracy. This is essentially the I coefficient in a PID control sense;
// the bigger the feedback coefficient, the faster the solution converges, usually at the expense of accuracy.
// In any case, this is the free parameter in the Madgwick filtering and fusion scheme.
float beta = sqrt(3.0f / 4.0f) * GyroMeasError;   // compute beta
float zeta = sqrt(3.0f / 4.0f) * GyroMeasDrift;   // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value
#define Kp 2.0f * 5.0f // these are the free parameters in the Mahony filter and fusion scheme, Kp for proportional feedback, Ki for integral
#define Ki 0.0f

uint32_t delt_t = 0; // used to control display output rate
uint32_t count = 0, sumCount = 0; // used to control display output rate
//float pitch, yaw, roll;
float deltat = 0.0f, sum = 0.0f;        // integration interval for both filter schemes
uint32_t lastUpdate = 0, firstUpdate = 0; // used to calculate integration interval
uint32_t Now = 0;        // used to calculate integration interval

float ax, ay, az, gx, gy, gz, mx, my, mz; // variables to hold latest sensor data values
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};    // vector to hold quaternion
float eInt[3] = {0.0f, 0.0f, 0.0f};       // vector to hold integral error for Mahony method

float magBias[3], magScale[3];

// ble lcd icon
byte btIcon[8]      = {0x04, 0x06, 0x15, 0x0E, 0x06, 0x0D, 0x16, 0x04};
// target lcd icon
byte targetIcon[8]  = {0x01, 0x02, 0x14, 0x08, 0x06, 0x0F, 0x0F, 0x06};
// app lcd icon
byte appIcon[8]     = {0x01, 0x02, 0x14, 0x08, 0x07, 0x05, 0x05, 0x07};

// set LCD address, number of columns and rows
int lcdColumns = 16;
int lcdRows = 2;
LiquidCrystal_I2C lcd(0x27, lcdColumns, lcdRows);

// set Stepper motor adress [in1, in2, in3, in4]
int ELStepperPins[4] = {13, 12, 14, 27};
int AZStepperPins[4] = {26, 25, 17, 16};
int stepperFullRotation = 4096;
int ELTargetSteps;
int AZTargetSteps;
bool ELClockWise = true;
bool AZClockWise = true;
CheapStepper ELStepper (ELStepperPins[0], ELStepperPins[1], ELStepperPins[2], ELStepperPins[3]);
CheapStepper AZStepper (AZStepperPins[0], AZStepperPins[1], AZStepperPins[2], AZStepperPins[3]);

// Touch Controls
const int numOfTouchInputs = 3;
const int touchInputPins[numOfTouchInputs] = {T0, T3, T9};
const int touchInputPinsTreshold[3] = {40, 40, 30};
const int touchInputPinsLowTreshold = 5;
int touchInputState[numOfTouchInputs];
int lastTouchInputState[numOfTouchInputs] = {0, 0, 0};
bool touchInputFlags[numOfTouchInputs]    = {0, 0, 0};
long lastDebounceTime[numOfTouchInputs]   = {0 ,0 ,0};
long debounceDelay = 3;
int buzzerPin = 18;
long beepStart;
long beepTime = 30;

int target = 99;

const int numOfPlanets = 10;
int currentPlanet = 0;
String planets[numOfPlanets][8];
bool backLight = false;
long backLightTime;

// Wifi
long lastApiCall;
<<<<<<< HEAD
=======

const char* ssid     = "G11-R1";
const char* password = "";

>>>>>>> 36fd102b2efad204095f4a8456404b1ff025bc4b
const char* apiCertificate= \
"-----BEGIN CERTIFICATE-----\n" \
"MIIGSjCCBTKgAwIBAgISA8imQX2QkjcTGPGhmu/JWfXZMA0GCSqGSIb3DQEBCwUA\n" \
"MDIxCzAJBgNVBAYTAlVTMRYwFAYDVQQKEw1MZXQncyBFbmNyeXB0MQswCQYDVQQD\n" \
"EwJSMzAeFw0yMTAxMDYxNjM4MTJaFw0yMTA0MDYxNjM4MTJaMBQxEjAQBgNVBAMT\n" \
"CWh5ZHI0ei5ubDCCAiIwDQYJKoZIhvcNAQEBBQADggIPADCCAgoCggIBANQZVU5a\n" \
"fXLGRv7tfbNXcOG1p1LBTtWNc+CXoF4noOCHd442vErnELPpbnWHzYeSWsBD3oFn\n" \
"BLIf76BcsgTx8pntsV0rFcEx8Sn2XYKs3L2S9j16kVrcp5dmnVF5DUrQTP9a4qCN\n" \
"+BCMKYIxy92oQGiwvwfX6SjdrnXHRQKY6SlBvfmoNCp/fXvXDDchx2fmm1AiHsNW\n" \
"m/z9zsMTyCcxom2Wyr5v9vxWJSOAQ79P8yOOkIYl2vIwaEyFFSFNZYLrPOHyBlsf\n" \
"c+4cOBeiXr20Ovb8hqRvUKY2LNNn8EP7bqHShma8e271ikOiTsmrPQ1zJosbXaw4\n" \
"o7IFkfyQtyiOHx4KeKWKwQEziAyMTRhvBNqHahEjyC8T9RWJcCFDDRIX+gqyaOBs\n" \
"IauB7JiKn/tPbkyfhO2RKiB/h6Wkx23MMCHfLnTe7Ho3NJAbAfj/Ogc4gbb7lXfE\n" \
"hmkDRSLMfSouPmwUS+yjDyUQm60ErVwm7HcvrOkg5yH5kNIrNEyHaDzxhH6UzLAg\n" \
"LI5v3R6jf1NIZFrXND0ld69ulw4KRKNV7+yBpcClaKm4yvCcd6v1cF4CZcmAeX8o\n" \
"q/tthBeSijsan/YzYqi9YDlxvgtlJ2LtwCAmVoBU8a3r3Bath+8Vnyvs2sJ3gy2k\n" \
"jLkjQ8ure97rDi4/iMs0ylQ4vHyju+nNyVD5AgMBAAGjggJ2MIICcjAOBgNVHQ8B\n" \
"Af8EBAMCBaAwHQYDVR0lBBYwFAYIKwYBBQUHAwEGCCsGAQUFBwMCMAwGA1UdEwEB\n" \
"/wQCMAAwHQYDVR0OBBYEFDj4HVnFkQKNALHecmirtgWZfvnhMB8GA1UdIwQYMBaA\n" \
"FBQusxe3WFbLrlAJQOYfr52LFMLGMFUGCCsGAQUFBwEBBEkwRzAhBggrBgEFBQcw\n" \
"AYYVaHR0cDovL3IzLm8ubGVuY3Iub3JnMCIGCCsGAQUFBzAChhZodHRwOi8vcjMu\n" \
"aS5sZW5jci5vcmcvMEUGA1UdEQQ+MDyCDWFwaS5oeWRyNHoubmyCCWh5ZHI0ei5u\n" \
"bIIRd3d3LmFwaS5oeWRyNHoubmyCDXd3dy5oeWRyNHoubmwwTAYDVR0gBEUwQzAI\n" \
"BgZngQwBAgEwNwYLKwYBBAGC3xMBAQEwKDAmBggrBgEFBQcCARYaaHR0cDovL2Nw\n" \
"cy5sZXRzZW5jcnlwdC5vcmcwggEFBgorBgEEAdZ5AgQCBIH2BIHzAPEAdgCUILwe\n" \
"jtWNbIhzH4KLIiwN0dpNXmxPlD1h204vWE2iwgAAAXbYxw45AAAEAwBHMEUCIQDf\n" \
"48gCMF92BtMSFTYPocOBi2rCNTLvqPyQrf4RbevCjQIgQ2Pls4ipJnZ2wE2USICj\n" \
"XVhKdOo/FUCyEX5qEMqBRaIAdwB9PvL4j/+IVWgkwsDKnlKJeSvFDngJfy5ql2iZ\n" \
"fiLw1wAAAXbYxw66AAAEAwBIMEYCIQCpyEdqN37Xot7cUttGDRchFljHfQEKC2HT\n" \
"uK2KQRMqYwIhAKY5E28i2K9UNl9lbT+rJZDU7pXdggAq41tD93LGOoo1MA0GCSqG\n" \
"SIb3DQEBCwUAA4IBAQC0Fkld7trqvewzSfUl9LPAIBIUsbXyFbP/2uPI7UalM+NW\n" \
"++bDgEqHcA+RFDOKdstPQzNN4oebHedekPlBbMrbiYN+zdYg2tNvisj4Pt0MEMZY\n" \
"vJ+o3C4jdNNCHm/ltAGV+9yVO+gSk3J3CoeBlbA2TLDOrNEPhc2m0D6/PDfRoRMG\n" \
"lBXtaTvcavhlv6RyJAGFEii62AA3ZHenKitOH37k70z2Kyh0teOTHCHBH1ghg2q9\n" \
"aXCOH4nAOhrWdq8EbmOwmp3TcNk4Gp+FW9dNKcXfW45rFpMp1LEVoQuZ38YpR8EF\n" \
"yRYmp3cl6qMWU2Xf/Bisg4t5Z13BGU6XcT+6t+H+\n" \
"-----END CERTIFICATE-----\n";

// BLE
BLECharacteristic *pCharacteristic;
bool BLEConnected = false;

#define SERVICE_UUID           "06cd0a01-f2af-4739-83ac-2be012508cd6"
#define CHARACTERISTIC_UUID_RX "4a59aa02-2178-427b-926a-ff86cfb87571"
#define CHARACTERISTIC_UUID_TX "068e8403-583a-41f2-882f-8b0a218ab77b"

class CServerCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    BLEConnected = true;
    digitalWrite(LED_BUILTIN, HIGH);
  };
  void onDisconnect(BLEServer* pServer) {
    BLEConnected = false;
    digitalWrite(LED_BUILTIN, LOW);
  }
};

// NTP server time
const char* ntpServer          = "pool.ntp.org";
const long  gmtOffset_sec      = 3600;
const int   daylightOffset_sec = 3600;
long lastDisplayTimeUpdate;
char displayTime[9];


void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  Wire.setClock(10000);
  for (int p=0; p<4; p++) { pinMode(AZStepperPins[p], OUTPUT); pinMode(ELStepperPins[p], OUTPUT); }
  pinMode(buzzerPin, OUTPUT);
  Serial.begin(115200);

  
//  byte c = readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);  // Read WHO_AM_I register for MPU-9250
//  Serial.print("MPU9250 "); Serial.print("I AM "); Serial.print(c, HEX); Serial.print(" I should be "); Serial.println(0x71, HEX);
//  if (c == 0x71) // WHO_AM_I should always be 0x68
//  {
//    Serial.println("MPU9250 is online...");
//    MPU9250SelfTest(SelfTest); // Start by performing self test and reporting values
//    calibrateMPU9250(gyroBias, accelBias); // Calibrate gyro and accelerometers, load biases in bias registers
//    initMPU9250();
//    byte d = readByte(AK8963_ADDRESS, WHO_AM_I_AK8963);
//    // Read WHO_AM_I register for AK8963
//    Serial.print("AK8963 "); Serial.print("I AM "); Serial.print(d, HEX); Serial.print(" I should be "); Serial.println(0x48, HEX);
//    delay(1000);
//    initAK8963(magCalibration);
//    ; Serial.println("AK8963 initialized for active data mode...."); // Initialize device for active mode read of magnetometer
////    magcalMPU9250(magBias, magScale);
//  } else {
//    Serial.print("Could not connect to MPU9250: 0x");
//    Serial.println(c, HEX);
//    while (1) ; // Loop forever if communication doesn't happen
//  }
  
  
  // lcd initialisation
  lcd.init();                
  toggleBackLight(true);
  lcd.createChar(1, btIcon);
  lcd.createChar(2, targetIcon);
  lcd.createChar(3, appIcon);

  // Stepper and Servo initialisation
  AZStepper.setRpm(30);
  ELStepper.setRpm(12);

  // Connect to internet
  lcd.setCursor(0,0);
  lcd.print("PlanetTracker   ");
  lcd.setCursor(0,1);
  lcd.print("Connecting      ");
  WiFi.begin(mySSID, myPASSWORD);
  while (WiFi.status() != WL_CONNECTED) { delay(250); }
  //init and get the time
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);

  //init and get api data
  lcd.setCursor(0,1);
  lcd.print("Fetching API    ");
  fetchAPIdata();
  // Clear lcd after setup
  lcd.setCursor(0, 0);
  lcd.print("                ");
  lcd.setCursor(0, 1);
  lcd.print("                ");
  // Print actual text
  refreshLCD();

  //init bluetooth
  class CCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      String stringValue = pCharacteristic->getValue().c_str();
      if (stringValue.length() > 2) {
        target = 99;
        float az = splitString(stringValue, '|', 0).toFloat();
        float el = splitString(stringValue, '|', 1).toFloat();
        pointAt(az, el);
      } else {
        currentPlanet = stringValue.toInt();
        target = stringValue.toInt();
        pointAt(planets[target][2].toFloat(), planets[target][1].toFloat());
      }
      // Update lcd
      toggleBackLight(true);
    };
  };
  
  BLEDevice::init("PlanetTracker");
  // Create server
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new CServerCallbacks());
  // Create service
  BLEService *pService = pServer->createService(SERVICE_UUID);
  // Create characteristic
  pCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID_TX,
                      BLECharacteristic::PROPERTY_NOTIFY
                    );
  // BLE2902 needed to notify
  pCharacteristic->addDescriptor(new BLE2902());
  // Receiving characteristic
  BLECharacteristic *pCharacteristic = pService->createCharacteristic(
                                        CHARACTERISTIC_UUID_RX,
                                        BLECharacteristic::PROPERTY_WRITE
                                       );
  pCharacteristic->setCallbacks(new CCallbacks());
  pService->start();
  pServer->getAdvertising()->start();
}

// Split a string at the passed character
String splitString(String data, char separator, int index) {
  int found = 0;
  int strIndex[] = {0, -1};
  int maxIndex = data.length()-1;
  for(int i=0; i<=maxIndex && found<=index; i++){
    if(data.charAt(i)==separator || i==maxIndex){
        found++;
        strIndex[0] = strIndex[1]+1;
        strIndex[1] = (i == maxIndex) ? i+1 : i;
    }
  }
  return found>index ? data.substring(strIndex[0], strIndex[1]) : "";
}

// Update the time string to be displayed on the lcd
void updateDisplayTime(){
  struct tm timeinfo;
  if(!getLocalTime(&timeinfo)){ return; }
  strftime(displayTime, 9, "%H:%M:%S", &timeinfo);
  refreshLCD();
  lastDisplayTimeUpdate = millis();
}

// Fetch target data from the api
void fetchAPIdata() {
  esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);
  if (SerialDebug) {
    Serial.printf("\nAfter heap clearing %u\n",ESP.getFreeHeap());
    Serial.println("FetchingAPI");
  }
  HTTPClient http;
  http.begin(apiADRESS, apiCertificate);
  int httpCode = http.GET();
  if (httpCode > 0) {
    String json = http.getString();
    parseJson(json);
  }
  http.end();
  lastApiCall = millis();
}

// Parse the passed JSON string and update the target array
void parseJson(String json) {
  StaticJsonDocument<1800> doc;
  DeserializationError error  = deserializeJson(doc, json);
  JsonArray root = doc["data"].as<JsonArray>();
  for(int i = 0; i < numOfPlanets; i++) {
      JsonObject planetJsonObj = root[i];
      planets[i][0] = planetJsonObj["name"].as<String>();
      planets[i][1] = planetJsonObj["el"].as<String>();
      planets[i][2] = planetJsonObj["az"].as<String>();
      planets[i][3] = planetJsonObj["dist"].as<String>();
      planets[i][4] = planetJsonObj["diam"].as<String>();
      planets[i][5] = planetJsonObj["const"].as<String>();
      planets[i][6] = planetJsonObj["above"].as<String>();
      planets[i][7] = planetJsonObj["below"].as<String>();
  }
  if (target != 99) {
    pointAt(planets[target][2].toFloat(), planets[target][1].toFloat());
  }
}

// Send a BLE notification if BLE is enabled
void sendBLENotification(char* msg) {
  if (BLEConnected) {
    pCharacteristic->setValue(msg);
    pCharacteristic->notify();
  }
}

void setInputFlags() {
  for(int i = 0; i < numOfTouchInputs; i++) { // loop over all touch inputs
    int inputVal = (touchRead(touchInputPins[i]) < touchInputPinsTreshold[i]); // record the touch input value and turn it into either 1 for on or 0 for off
    if (inputVal != lastTouchInputState[i]) { // if the input has been pressed / changed from the state before press
      lastDebounceTime[i] = millis(); // save the time at which it was first pressed in
    }
    if ((millis() - lastDebounceTime[i]) > debounceDelay) { // if the pressed duration is or is greater than the debounceDelay (5ms)
      if (inputVal != touchInputState[i]) { // if the inputvalue is still the same as during the press
        touchInputState[i] = inputVal; // record the input value
        if (touchInputState[i] == 1) { // if the input is 1/true
          touchInputFlags[i] = 1; // set the input flag for that input to 1/true
        }
      }
    }
    lastTouchInputState[i] = inputVal; // archive the input value as the 'latest' input time
  }
}

void resolveInputFlags() {
  for(int i = 0; i < numOfTouchInputs; i++) { // loop over all touch inputs
    if (touchInputFlags[i] == 1) {
      handleInput(i);
      touchInputFlags[i] = 0;
    }
  }
}

// handle input signals sent via the touch buttons
void handleInput(int input) {
  if(SerialDebug) {
    Serial.print("Input pin: ");
    Serial.print(input);
    Serial.print(" Backlight: ");
    Serial.println(backLight);
  }
  if (backLight) { // only handle input if backlight is on, otherwise turn backlight on
    if (input == 0) { // move page down
      if (currentPlanet == 0) {
        currentPlanet = numOfPlanets - 1;
      } else {
      currentPlanet--;
      }
    } else if (input == 1) { // move page up
      if (currentPlanet == numOfPlanets -1) {
        currentPlanet = 0;
      } else {
        currentPlanet++;
      }
    } else { // set current page as target
      target = currentPlanet;
      pointAt(planets[target][2].toFloat(), planets[target][1].toFloat());
    }
    refreshLCD();
    digitalWrite(buzzerPin, HIGH);
    beepStart = millis();
  }
  toggleBackLight(true);
}

void toggleBackLight(bool state) {
  if (state) {
    lcd.setBacklight(HIGH);
    backLightTime = millis();
  } else {
    lcd.setBacklight(LOW);
  }
  backLight = state;
}

// Refresh the passed lcd segment
void refreshLCD() {
  // Time
  lcd.setCursor(0,0);
  lcd.print(displayTime);
  // Planet name
  lcd.setCursor(0, 1);
  lcd.print("                ");
  lcd.setCursor(0, 1);
  lcd.print(planets[currentPlanet][0]);
  // Planet distance
  lcd.setCursor(8, 1);
  lcd.print(planets[currentPlanet][3]);
  lcd.print("au");
  // Tracking icon
  lcd.setCursor(15, 0);
  if (currentPlanet == target) {
    lcd.write(2);
  } else {
    lcd.print(" ");
  }
  // BLE icon
  lcd.setCursor(14, 0);
  if (BLEConnected) {
    lcd.write(1);
  } else {
    lcd.print(" ");
  }
}

// Point stepper motors at the azimuth and elevation
void pointAt(float az, float el) {
  sendBLENotification("Moving");
  // EL STEPPER
  ELTargetSteps = (int)(stepperFullRotation / 360 * el);
  if (ELStepper.getStep() < ELTargetSteps && ELTargetSteps - ELStepper.getStep() < stepperFullRotation / 2) {
    ELClockWise = true;
  } else if (ELStepper.getStep() < ELTargetSteps && ELTargetSteps - ELStepper.getStep() > stepperFullRotation / 2) {
    ELClockWise = false;
  } else if (ELStepper.getStep() > ELTargetSteps && ELTargetSteps - ELStepper.getStep() < stepperFullRotation / 2) {
    ELClockWise = false;
  } else if (ELStepper.getStep() > ELTargetSteps && ELTargetSteps - ELStepper.getStep() > stepperFullRotation / 2) {
    ELClockWise = true;
  }
  
  // AZ STEPPER
  AZTargetSteps = (int)(stepperFullRotation / 360 * az);
  if (AZStepper.getStep() < AZTargetSteps && AZTargetSteps - AZStepper.getStep() < stepperFullRotation / 2) {
    AZClockWise = true;
  } else if (AZStepper.getStep() < AZTargetSteps && AZTargetSteps - AZStepper.getStep() > stepperFullRotation / 2) {
    AZClockWise = false;
  } else if (AZStepper.getStep() > AZTargetSteps && AZTargetSteps - AZStepper.getStep() < stepperFullRotation / 2) {
    AZClockWise = false;
  } else if (AZStepper.getStep() > AZTargetSteps && AZTargetSteps - AZStepper.getStep() > stepperFullRotation / 2) {
    AZClockWise = true;
  }
}

// Turn all stepper phases off to prevent heeating issues
void turnStepperOff() {
  for (int p=0; p<4; p++) { digitalWrite(ELStepperPins[p], 0); }
  for (int p=0; p<4; p++) { digitalWrite(AZStepperPins[p], 0); }
  if (target != 99) {
    sendBLENotification("Tracking");
  } else {
    sendBLENotification("Ready");
  }
}

void loop() {
  MPUloop();
  if (SerialDebug) {
    Serial.print("Yaw:");
    Serial.print(yaw);
    Serial.print(" Pitch:");
    Serial.print(pitch);
    Serial.print(" Roll:");
    Serial.println(roll);
  }

  // Time check to update displayed time and toggle the backlight
  
  if (millis() - lastDisplayTimeUpdate > 1000) { updateDisplayTime(); } 
  if (backLight) {
    if (millis() - backLightTime > 30000) { // turn off after 15 seconds
      toggleBackLight(false);
    }
  }
  
  // Handle pointer movement
  if (AZStepper.getStep() != AZTargetSteps || ELStepper.getStep() != ELTargetSteps) {
    // AZ Stepper
    if (AZStepper.getStep() != AZTargetSteps) {
      if (SerialDebug) {
        Serial.print("AZTarget:");
        Serial.print(AZTargetSteps);
        Serial.print(" AZCurrent:");
        Serial.println(AZStepper.getStep());
      }
      AZStepper.step(AZClockWise);
    }
    // EL Stepper
    if (ELStepper.getStep() != ELTargetSteps) {
      if (SerialDebug) {
        Serial.print("ELTarget:");
        Serial.print(ELTargetSteps);
        Serial.print(" ELCurrent:");
        Serial.println(ELStepper.getStep());
      }
      ELStepper.step(ELClockWise);
    }
  } else {
    // Refetch api data
    if (millis() - lastApiCall > 30000) { fetchAPIdata(); }
    turnStepperOff();
  }
  
  // Check physical input
  setInputFlags();
  resolveInputFlags();
  if (millis() - beepStart > beepTime) {
    digitalWrite(buzzerPin, LOW);
  }

  // Delay to slow loop when inactive
  if (!backLight && ((AZStepper.getStep() == AZTargetSteps && ELStepper.getStep() == ELTargetSteps) || target == 99)) {
    delay(125);
  }
}
