#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <SPI.h>
#include "SparkFun_Qwiic_KX13X.h"

//represent kx134 interruption physical wiring
#define INT1 12  //pin at high when data available for reading
#define INT2 13 //interruption source at high when free fall is detected

#define KX134_CS_PIN 21

//Data Acquisition related variables
const uint16_t acquisitionFrequencyHz = 12800 ;
const float acquisitionTimeSecond = 1;
const uint16_t nbOfSamples = (uint16_t) acquisitionFrequencyHz*acquisitionTimeSecond + 80 ; // +80 because in practice, the acquisition rate is closer to 12900 Hz than 12800 hz

typedef struct accelSampleStorage{
  int16_t xRawData;
  int16_t yRawData;
  int16_t zRawData;
  uint16_t sampleRelativeTimeMicroSecond; //relative time with the previous measure in µs
} accelSampleStorage;
accelSampleStorage *acquisitionData;
bool freefallInterruptTrigered = false;
bool accelDataAvailableForReading = false;
uint8_t interruptState;
ulong previousSampleAbsoluteDateMicrosecond;
ulong currentSampleAbsoluteDateMicrosecond = 0;

QwiicKX134 accel; //Initialization of accelerometers, an object from the QwiickKX134 class. this object represents our sensor


//ESP NOW related variables
uint8_t broadcastAddress[] = {0x40, 0x91, 0x51, 0xB2, 0x7E, 0xAC};
esp_now_peer_info receiverInfo;
esp_err_t communicationStatus;
const uint8_t maxSamplePerMessage = (uint8_t) ESP_NOW_MAX_DATA_LEN/sizeof(accelSampleStorage);
accelSampleStorage dataMessage[maxSamplePerMessage];

// Callback function called when data is sent by ESP NOW protocol
void dataSentCallbackFunction(const uint8_t *mac_addr, esp_now_send_status_t status) {
  if(status == ESP_NOW_SEND_FAIL){
    Serial.println("Failed to send message");
  }
}

void getRawData(accelSampleStorage *rawDataAcquisition,uint16_t sampleRelativeTimeMicroSecond);
void freefallTrigeredInterrupt();
void accelDataAvailableForReadingInterrupt();
void InterruptsSetting();
void rearmFreefallInterrupt();

void setup() {
  acquisitionData = (accelSampleStorage *) malloc(nbOfSamples*sizeof(accelSampleStorage));
  // protection in case there is not enough memory space for acquisition data
  if(acquisitionData == NULL){Serial.print("Error trying to allocate memory space to store the acceleration data");while(1){}}

  Serial.begin(115200); //starting serial communication with computer
  pinMode(INT1, INPUT);
  pinMode(INT2, INPUT);
  pinMode(KX134_CS_PIN, OUTPUT);

  //Initialisation of ESPNOW protocol
  WiFi.mode(WIFI_STA);
  if(esp_now_init() != ESP_OK){
    Serial.println("Error initializing ESPNOW protocol");
  }
  esp_now_register_send_cb(dataSentCallbackFunction);

  //register peer informations
  memcpy(receiverInfo.peer_addr, broadcastAddress, 6);
  receiverInfo.channel = 0;
  receiverInfo.encrypt = false;
  //add peer to the peer book
  if(esp_now_add_peer(&receiverInfo) != ESP_OK){
    Serial.println("Failed to add receiver in the peer book");
  }

  //initialisation of SPI communication with kx134
  SPI.begin(); //initialisation of SPI bus
  uint SPIbusFrequencyHz = 10000000;
  if( !accel.beginSPI(KX134_CS_PIN, SPIbusFrequencyHz, SPI) ){   // the clock speed of the SPI is set at 10MHz because it's the maximum speed of the kx13x (esp32 can reach 50MHz)
    Serial.println("Could not communicate with the the KX134. Freezing.");
    while(1);
  }
  else{ Serial.println("Ready.");}

  //initialisation of the kx134 component and setting up freefall and data ready interrupts
  InterruptsSetting();

  Serial.print("KX134 Output data rate : ");
  Serial.println(accel.readOutputDataRate());
  rearmFreefallInterrupt(); // just in case freefall interrupt was latched during last shutdown
  accel.QwiicKX13xCore::readRegister( &interruptState,KX13X_INS2); //read wich source caused the interruption on INT2 (for debug)
  Serial.println(interruptState, BIN);
}

void loop() {
  if(freefallInterruptTrigered){
    for(uint16_t i=0; i<nbOfSamples; i++){
      while (!accelDataAvailableForReading){ delayMicroseconds(15);/*delay to avoid CPU Crash. Must be inferior to 78 µs (1/acquisition frequency)*/ }
      accelDataAvailableForReading = false;
      getRawData(&acquisitionData[i], (uint16_t) currentSampleAbsoluteDateMicrosecond - previousSampleAbsoluteDateMicrosecond); //sample time is the relative time in microsecond between two measures
      previousSampleAbsoluteDateMicrosecond = currentSampleAbsoluteDateMicrosecond;
    }

    for(uint16_t sampleIndex=0; sampleIndex<nbOfSamples; sampleIndex+=maxSamplePerMessage){
      do {
        memcpy(dataMessage,acquisitionData+sampleIndex,sizeof(dataMessage));
        communicationStatus = esp_now_send(broadcastAddress, (uint8_t *) dataMessage, sizeof(dataMessage));
        delay(10); //let the time for the message to be fully sent and received before trying to send another
        if (communicationStatus != ESP_OK)
        {
          Serial.print("error sending the ");Serial.print(sampleIndex);Serial.println("th sample, we try again in 5 seconds");
          delay(5000);
        }
        else if(sampleIndex%(maxSamplePerMessage*10) == 0){ //just for debug
          Serial.print(sampleIndex); Serial.println("th sample sent");
        }
      }while (communicationStatus != ESP_OK);
    }

    //Send data to serial port to compare them with the received data (for transmission validation test)
    Serial.print("[");
    for(uint16_t i=0; i<nbOfSamples; i++){
        Serial.print(acquisitionData[i].xRawData);Serial.print(",");
    }
    Serial.println("]");
    delay(5000);
    //Rearm the freefall interruption pin (INT2)
    accel.QwiicKX13xCore::readRegister( &interruptState,KX13X_INT_REL); //reininitialisation of the interrupt pin in latched mode
    freefallInterruptTrigered = false;
  }
}

void getRawData(accelSampleStorage *rawDataAcquisition,uint16_t sampleRelativeTimeMicroSecond){
  rawOutputData rawAccelData;
  accel.QwiicKX13xCore::getRawAccelData(&rawAccelData);
  rawDataAcquisition->xRawData = rawAccelData.xData ;
  rawDataAcquisition->yRawData = rawAccelData.yData ;
  rawDataAcquisition->zRawData = rawAccelData.zData ;
  rawDataAcquisition->sampleRelativeTimeMicroSecond = sampleRelativeTimeMicroSecond ;
}

void freefallTrigeredInterrupt(){
  freefallInterruptTrigered = true;
}

void accelDataAvailableForReadingInterrupt(){
  previousSampleAbsoluteDateMicrosecond = currentSampleAbsoluteDateMicrosecond;
  currentSampleAbsoluteDateMicrosecond = micros();
  accelDataAvailableForReading = true;
}

void InterruptsSetting(){
  //definition of param constant
  const uint8_t STAND_BY_MODE = 0b00000000;
  const uint8_t OUTPUT_DATA_RATE_12800Hz = 0b00001110; //1111 for 25600, 1110 for 12800Hz, 1101 for 6400Hz, 1100 for 3200Hz, etc (show page26 of technical reference manual)
  const uint8_t ACTIVATE_INT1 = 0b00100000;
  const uint8_t INT_ACTIVE_LEVEL_HIGH = 0b00010000;
  const uint8_t ATTACH_DATA_READY_ENGINE = 0b00010000;
  const uint8_t ENABLE_FREEFALL_ENGINE = 0b10000000;
  const uint8_t LATCHED_FREEFALL_ENGINE = 0b00000000;
  const uint8_t UP_RESET_COUNT_METHODOLOGY = 0b00001000;
  const uint8_t FREEFALL_ENGINE_FREQUENCY = 0b00000111; //1600Hz
  const uint8_t FREEFALL_TRESHOLD = 0b00000010;//set the freefall threshold count (4count per g. ex : 0b00000010=3 correspond to 0.75g) see technical reference manual page 64 for more detail
  const uint8_t FREEFALL_TRIGGER_TIME = 0b00010000; //set the freefall threshold time (time = trigger time/freefall ODR) see technical reference manual page 64 for more detail
  const uint8_t ACTIVATE_INT2 = 0b00100000;
  const uint8_t ATTACH_FREEFALL_ENGINE = 0b10000000;
  const uint8_t HIGH_PERFORMANCE_MODE = 0b01000000;
  const uint8_t ACTIVATE_KX134 = 0b10000000;
  const uint8_t ENABLE_DATA_READY_ENGINE = 0b00100000;
  const uint8_t ACCELERATION_RANGE_64G = 0b00011000;

  // Put the kx134 in setting mode
  accel.QwiicKX13xCore::writeRegister(KX13X_CNTL1, 0, STAND_BY_MODE, 0);

  // Set up acquisition frequency to 12800 Hz
  accel.QwiicKX13xCore::writeRegister(KX13X_ODCNTL, 0, OUTPUT_DATA_RATE_12800Hz, 0);

  //INT1 setup
  accel.QwiicKX13xCore::writeRegister(KX13X_INC1, 0, ACTIVATE_INT1|INT_ACTIVE_LEVEL_HIGH, 0);
  accel.QwiicKX13xCore::writeRegister(KX13X_INC4, 0, ATTACH_DATA_READY_ENGINE, 0);
  attachInterrupt(digitalPinToInterrupt(INT1), accelDataAvailableForReadingInterrupt, RISING);

  // Start the freefall engine in latched mode with count up/reset debounce methodology and 1600 Hz ODR for the freefall engine (see Technical reference page 37 for more info)
  accel.QwiicKX13xCore::writeRegister(KX13X_FFCNTL, 0, ENABLE_FREEFALL_ENGINE|LATCHED_FREEFALL_ENGINE|UP_RESET_COUNT_METHODOLOGY|FREEFALL_ENGINE_FREQUENCY, 0);
  accel.QwiicKX13xCore::writeRegister(KX13X_FFTH, 0, FREEFALL_TRESHOLD, 0);

  accel.QwiicKX13xCore::writeRegister(KX13X_FFC, 0, FREEFALL_TRIGGER_TIME , 0);

  //INT2 setup
  accel.writeRegister(KX13X_INC5, 0, ACTIVATE_INT2|INT_ACTIVE_LEVEL_HIGH, 0);
  accel.writeRegister(KX13X_INC6, 0, ATTACH_FREEFALL_ENGINE, 0);
  attachInterrupt(digitalPinToInterrupt(INT2), freefallTrigeredInterrupt, RISING);

  //Activate the KX134 in High performance mode, with data ready engine enabled, with +- 64g range.
  accel.QwiicKX13xCore::writeRegister(KX13X_CNTL1, 0, ACTIVATE_KX134|HIGH_PERFORMANCE_MODE|ENABLE_DATA_READY_ENGINE|ACCELERATION_RANGE_64G, 0);
  delay(1500); //let time for the kx134 to fully wake up
}

void rearmFreefallInterrupt(){
    accel.QwiicKX13xCore::readRegister( &interruptState,KX13X_INT_REL); //reininitialisation of all the interrupt pin in latched mode
}