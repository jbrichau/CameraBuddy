#include "ArduCAM.h"
#include "memorysaver.h"
#include "SdFat.h"

#define SD_CS D6
#define ARDUCAM_CS A5

SYSTEM_THREAD(ENABLED);
SYSTEM_MODE(SEMI_AUTOMATIC);

SystemSleepConfiguration sleepconfig;
SystemSleepResult sleepresult;
SystemPowerConfiguration powerconfig;

PMIC pmic;
char batteryString[64] = "Null";

int active = 1;

ArduCAM myCAM(OV5642, ARDUCAM_CS);
SdFat SD(&SPI1);
String errorstate = "NONE";
double freeSpace;
long lastupdatemillis;

void setup() {
  Wire.begin();
  Serial.begin(921600);
  setPowerConfiguration("solarcharge");

  pinMode(ARDUCAM_CS, OUTPUT);
  pinMode(SD_CS, OUTPUT);
  digitalWrite(ARDUCAM_CS, HIGH);
  digitalWrite(SD_CS, HIGH);
  
  Particle.variable("errorstate", errorstate);
  Particle.variable("freeSpace", freeSpace);
  Particle.variable("active", active);
  Particle.function("command", command);

  SPI.begin();
  initializeCam();
  initializeSD();
  //autofocus();

  Time.zone(2); // should actually be 1 and set DST too, but taking a shortcut for now
  sleepconfig.mode(SystemSleepMode::STOP);
  Particle.connect();
  lastupdatemillis = 0;
}

void loop() {
  long currentmillis;

  if(active==1) {
    myCAM.clear_bit(ARDUCHIP_GPIO,GPIO_PWDN_MASK);
    delay(800);
    takePicture();
    savePicture();
    myCAM.set_bit(ARDUCHIP_GPIO,GPIO_PWDN_MASK);
  }

  int currentHour = Time.hour();
  if (Time.isValid() & (currentHour > 8 | currentHour < 3)) {
    if(active==1)
      activate("off");
    delay(10000);
  } else {
    if(active==0)
      activate("on");
    currentmillis = millis();
    if (lastupdatemillis + (10 * 60000) < currentmillis) {
      lastupdatemillis = currentmillis;
      freeSpace = 0.000512 * SD.vol()->freeClusterCount() * SD.vol()->blocksPerCluster();
    }
  }
}

void initializeCam() {
  uint8_t vid, pid;
  uint8_t temp;
  myCAM.clear_bit(ARDUCHIP_GPIO,GPIO_PWDN_MASK);
  delay(800);

  while (1)
  {
    Serial.println("Checking for camera...");
    myCAM.write_reg(ARDUCHIP_TEST1, 0x55);
    temp = myCAM.read_reg(ARDUCHIP_TEST1);
    if (temp != 0x55) {
      registerError("Arducam SPI interface Error!");
      Serial.println("myCam.read_reg said " + String(temp));
      delay(5000);
    } else {
      break;
    }
    Particle.process();
  }

  while (1)
  {
    Serial.println("Checking if the camera module type is OV5642");
    myCAM.rdSensorReg16_8(OV5642_CHIPID_HIGH, &vid);
    myCAM.rdSensorReg16_8(OV5642_CHIPID_LOW, &pid);
    if ((vid != 0x56) || (pid != 0x42)) {
      registerError("Can't find OV5642 module!");
      delay(5000);
      continue;
    } else {
      Serial.println("OV5642 detected.");
      break;
    }
  }

  Serial.println("Camera found, initializing...");
  myCAM.set_format(JPEG);
  delay(100);
  myCAM.InitCAM();
  delay(100);
  myCAM.set_format(JPEG);
  delay(100);
  myCAM.set_bit(ARDUCHIP_TIM, VSYNC_LEVEL_MASK);
  //myCAM.write_reg(ARDUCHIP_TIM, VSYNC_LEVEL_MASK);
  delay(100);
  myCAM.clear_fifo_flag();
  delay(100);
  myCAM.write_reg(ARDUCHIP_FRAMES, 0x00);
  delay(100);
  //myCAM.OV5642_set_JPEG_size(OV5642_1600x1200);
  myCAM.OV5642_set_JPEG_size(OV5642_2592x1944);
  delay(1000);
}

void initializeSD() {
  if (!SD.begin(SD_CS))
    registerError("Failed to open SD card!");
}

void autofocus()
{
  uint8_t regX;
  myCAM.wrSensorReg16_8(0x3022, 0x03);
  while (1) {
    // check status
    myCAM.rdSensorReg16_8(0x3029, &regX);
    if (regX == 0x10) {
      Serial.println("ACK CMD Auto focus completed END");
      break;
    } // focus completed
    else {
      Serial.println("ACK CMD Auto focusing ...END");
      delay(1000);
    }
  }
}

void takePicture() {
  unsigned long start_time, last_publish;
  //Serial.println("Taking a picture...");

  myCAM.flush_fifo();
  delay(100);
  myCAM.clear_fifo_flag();
  delay(100);
  myCAM.start_capture();
  delay(100);

  start_time = millis();
  last_publish = millis();
  while (!myCAM.get_bit(ARDUCHIP_TRIG, CAP_DONE_MASK)) {
    delay(10);
    unsigned long now = millis();
    if ((now - last_publish) > 10000) {
      Particle.process();
      last_publish = now;
    }
    if ((now - start_time) > 30000)
      break;
  }
  Serial.println("IMAGE TAKEN");
}

uint8_t savePicture() {
  uint8_t temp = 0, temp_last = 0;
  uint32_t length = 0;
  static int i = 0;
  static int k = 0;
  String fileName;
  File outFile;
  byte buf[256];
  bool is_header = false;

  length = myCAM.read_fifo_length();
  Serial.print(F("The fifo length is :"));
  Serial.println(length, DEC);
  if (length >= MAX_FIFO_SIZE) {
    Serial.println("Over size.");
    return 0;
  }
  if (length == 0) {
    Serial.println(F("Size is 0."));
    return 0;
  }
  myCAM.CS_LOW();
  myCAM.set_fifo_burst(); //Set fifo burst mode
  i = 0;
  while (length--)
  {
    temp_last = temp;
    temp = SPI.transfer(0x00);
    //Read JPEG data from FIFO
    if ((temp == 0xD9) && (temp_last == 0xFF)) //If find the end ,break while,
    {
      buf[i++] = temp; //save the last  0XD9
      //Write the remain bytes in the buffer
      myCAM.CS_HIGH();
      outFile.write(buf, i);
      //Close the file
      outFile.close();
      //Serial.println(F("OK"));
      is_header = false;
      myCAM.CS_LOW();
      myCAM.set_fifo_burst();
      i = 0;
    }
    if (is_header == true)
    {
      //Write image data to buffer if not full
      if (i < 256)
        buf[i++] = temp;
      else
      {
        //Write 256 bytes image data to file
        myCAM.CS_HIGH();
        outFile.write(buf, 256);
        i = 0;
        buf[i++] = temp;
        myCAM.CS_LOW();
        myCAM.set_fifo_burst();
      }
    }
    // Start of jpeg
    else if ((temp == 0xD8) & (temp_last == 0xFF))
    {
      is_header = true;
      myCAM.CS_HIGH();
      k = k + 1;
      fileName = Time.format(Time.now(), "%Y%m%d-%H%M%S");
      Serial.println(fileName);
      outFile = SD.open(String(fileName + ".jpg"), O_WRITE | O_CREAT | O_TRUNC);
      if (!outFile) {
        registerError("File open failed");
        initializeSD();
        return 0;
      }
      myCAM.CS_LOW();
      myCAM.set_fifo_burst();
      buf[i++] = temp_last;
      buf[i++] = temp;
    }
  }
  myCAM.CS_HIGH();
  return 1;
}

uint8_t writeToSerial()
{
  uint8_t temp = 0, temp_last = 0;
  uint32_t length = 0;
  bool is_header = false;

  length = myCAM.read_fifo_length();
  Serial.println(length, DEC);
  if (length >= MAX_FIFO_SIZE) //512 kb
  {
    Serial.println(F("ACK CMD Over size. END"));
    return 0;
  }
  if (length == 0) //0 kb
  {
    Serial.println(F("ACK CMD Size is 0."));
    return 0;
  }
  myCAM.CS_LOW();
  myCAM.set_fifo_burst(); //Set fifo burst mode
  while (length--)
  {
    temp_last = temp;
    temp = SPI.transfer(0x00);
    if (is_header == true)
    {
      Serial.write(temp);
    }
    else if ((temp == 0xD8) & (temp_last == 0xFF))
    {
      is_header = true;
      Serial.println(F("ACK IMG END"));
      Serial.write(temp_last);
      Serial.write(temp);
    }
    if ((temp == 0xD9) && (temp_last == 0xFF)) //If find the end ,break while,
      break;
    delayMicroseconds(15);
  }
  myCAM.CS_HIGH();
  is_header = false;
  return 1;
}

void registerError(String error) {
  Serial.println(error);
  errorstate = error;
}

void clearError() {
  errorstate = "NONE";
}

int activate(String arg) {
  if(arg=="on") {
    active = 1;
    myCAM.clear_bit(ARDUCHIP_GPIO,GPIO_PWDN_MASK);
  }
  if(arg=="off") {
    active = 0;
    myCAM.set_bit(ARDUCHIP_GPIO,GPIO_PWDN_MASK);
  }
  return 0;
}

void setPowerConfiguration(String arg) {
  powerconfig = SystemPowerConfiguration();
  if(arg=="solarcharge") {
    powerconfig
      .batteryChargeCurrent(1024)
      .batteryChargeVoltage(4112)
      .powerSourceMinVoltage(5080)
      .powerSourceMaxCurrent(900);
  }
  if(arg=="usbcharge") {
    powerconfig
      .batteryChargeCurrent(896)
      .batteryChargeVoltage(4112)
      .powerSourceMinVoltage(3880)
      .powerSourceMaxCurrent(900);
  }
  System.setPowerConfiguration(powerconfig);
}

void setLed(String arg) {
  if(arg=="on")
    LEDSystemTheme::restoreDefault();
  if(arg=="off") {
    LEDSystemTheme theme;
    theme.setColor(LED_SIGNAL_CLOUD_CONNECTED, 0x00000000);
    theme.apply();
  }
}

int command(String arg) {
  if((arg=="solarcharge") | (arg=="usbcharge")) {
    setPowerConfiguration(arg);
    return 1; 
  }
  if(arg=="godark") {
    setLed("off");
    return 1;
  }
  if(arg=="ledon") {
    setLed("on");
    return 1;
  }
  if(arg=="cameraon") {
    activate("on");
    return 1;
  }
  if(arg=="cameraoff") {
    activate("off");
    return 1;
  }
  return -1;
}