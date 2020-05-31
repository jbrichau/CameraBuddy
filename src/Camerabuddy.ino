#include "ArduCAM.h"
#include "memorysaver.h"
#include "SdFat.h"

SYSTEM_THREAD(ENABLED);
SYSTEM_MODE(AUTOMATIC);

#define SD_CS D6
#define ARDUCAM_CS A5

ArduCAM myCAM(OV5642, ARDUCAM_CS);
SdFat SD(&SPI1);

void setup()
{
  // Wire.setSpeed(CLOCK_SPEED_100KHZ);
  Wire.begin();
  Serial.begin();

  pinMode(ARDUCAM_CS, OUTPUT);
  pinMode(SD_CS, OUTPUT);
  digitalWrite(ARDUCAM_CS, HIGH);
  digitalWrite(SD_CS, HIGH);

  SPI.begin();
  initializeCam();
  initializeSD();
}

void loop()
{
  takePicture();
  savePicture();
  delay(1000);
}

void initializeCam()
{
  uint8_t vid, pid;
  uint8_t temp;

  while (1)
  {
    Serial.println("Checking for camera...");

    //Check if the ArduCAM SPI bus is OK
    myCAM.write_reg(ARDUCHIP_TEST1, 0x55);
    temp = myCAM.read_reg(ARDUCHIP_TEST1);
    if (temp != 0x55)
    {
      Serial.println("SPI interface Error!");
      Serial.println("myCam.read_reg said " + String(temp));
      delay(5000);
    }
    else
    {
      break;
    }
    Particle.process();
  }

  while (1)
  {
    //Check if the camera module type is OV5642
    myCAM.rdSensorReg16_8(OV5642_CHIPID_HIGH, &vid);
    myCAM.rdSensorReg16_8(OV5642_CHIPID_LOW, &pid);
    if ((vid != 0x56) || (pid != 0x42))
    {
      Serial.println(F("Can't find OV5642 module!"));
      delay(5000);
      continue;
    }
    else
    {
      Serial.println(F("OV5642 detected."));
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

  //myCAM.OV5642_set_JPEG_size(OV5642_320x240);
  myCAM.OV5642_set_JPEG_size(OV5642_1600x1200);
  //myCAM.OV5642_set_JPEG_size(OV5642_640x480);    // ?

  // wait a sec`
  delay(1000);
}

void initializeSD()
{
  if (!SD.begin(SD_CS))
  {
    Serial.println("failed to open card");
    return;
  }
}

void takePicture()
{
  Serial.println("Taking a picture...");

  myCAM.flush_fifo();
  delay(100);

  myCAM.clear_fifo_flag();
  delay(100);

  myCAM.start_capture();
  delay(100);

  unsigned long start_time = millis(),
                last_publish = millis();

  //  wait for the photo to be done
  while (!myCAM.get_bit(ARDUCHIP_TRIG, CAP_DONE_MASK))
  {
    delay(10);
    unsigned long now = millis();
    if ((now - last_publish) > 1000)
    {
      last_publish = now;
    }

    if ((now - start_time) > 30000)
    {
      break;
    }
  }
  delay(100);

  int length = myCAM.read_fifo_length();
  Serial.println("Image size is " + String(length));
}

uint8_t savePicture()
{
  uint8_t temp = 0, temp_last = 0;
  uint32_t length = 0;
  static int i = 0;
  static int k = 0;
  char str[8];
  File outFile;
  byte buf[256];
  bool is_header = false;

  length = myCAM.read_fifo_length();
  Serial.print(F("The fifo length is :"));
  Serial.println(length, DEC);
  if (length >= MAX_FIFO_SIZE) //8M
  {
    Serial.println("Over size.");
    return 0;
  }
  if (length == 0) //0 kb
  {
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
      Serial.println(F("OK"));
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
      itoa(k, str, 10);
      strcat(str, ".jpg");
      outFile = SD.open(str, O_WRITE | O_CREAT | O_TRUNC);
      if (!outFile)
      {
        Serial.println(F("File open failed"));
        while (1)
          ;
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