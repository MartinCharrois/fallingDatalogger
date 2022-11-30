#include <arduino.h>
#include <esp_now.h>
#include <WiFi.h>

//DAta Acquisition related variables
const uint16_t acquisitionFrequencyHz = 12800 ;
const float acquisitionTimeSecond = 1;
const uint16_t nbOfSamples = (uint16_t) acquisitionFrequencyHz*acquisitionTimeSecond + 80 ; // +80 because in practice, the acquisition rate is closer to 12900 Hz than 12800 hz

typedef struct accelSampleStorage{
  int16_t xRawData;
  int16_t yRawData;
  int16_t zRawData;
  uint16_t sampleRelativeTimeMicroSecond;
} accelSampleStorage;
accelSampleStorage *acquisitionData;

uint16_t sampleIndex = 0;
//ESP NOw related variables
const uint8_t maxSamplePerMessage = (uint8_t) ESP_NOW_MAX_DATA_LEN/sizeof(accelSampleStorage);
bool allDataReceived = false;

// Callback function executed when data is received
void DataReceptionCallbackFunction(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(acquisitionData+sampleIndex, incomingData, len);
  if(sampleIndex%(maxSamplePerMessage*10) == 0){  //for debug only
    Serial.print(sampleIndex); Serial.println("th sample successfully received");
  }
  if(sampleIndex>=nbOfSamples){ //if we received all the data
      allDataReceived = true;
  }
  sampleIndex+=maxSamplePerMessage;
}

void setup() {
  acquisitionData = (accelSampleStorage *) malloc(nbOfSamples*sizeof(accelSampleStorage));
  // protection in case there is not enough memory space for acquisition data
  if(acquisitionData == NULL){Serial.print("Error trying to allocate memory space to store the acceleration data");while(1){}}

  Serial.begin(115200);

  // Set ESP32 as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
  }

  // Register callback function
  esp_now_register_recv_cb(DataReceptionCallbackFunction);
  Serial.println("ready to receive data");
}

void loop() {
  if(allDataReceived){
   //Send data to serial port to compare them with the received data
    Serial.print("[");
    for(uint16_t i=0; i<nbOfSamples; i++){
        Serial.print(acquisitionData[i].xRawData);Serial.print(",");
    }
    Serial.println("]");
    allDataReceived = false;
  }
}

typedef struct convertedAccelSampleStorage{
  double xData;
  double yData;
  double zData;
  uint16_t sampleRelativeTimeMicroSecond;
} convertedAccelSampleStorage;

void accelDataConversion(accelSampleStorage* rawSample, convertedAccelSampleStorage* convertedSample, uint8_t accelRange){
  const double convRange2G =  .00006103518784142582;
  const double convRange4G =  .0001220703756828516;
  const double convRange8G =  .0002441407513657033;
  const double convRange16G = .0004882811975463118;
  double rangeConverter;
  switch(accelRange){
    case 2 : rangeConverter = convRange2G;
            break;
    case 4 : rangeConverter = convRange4G;
            break;
    case 8 : rangeConverter = convRange8G;
            break;
    case 16 : rangeConverter = convRange16G;
            break;
    default : rangeConverter = 1 ; // When the end user will see all his data a thousand times too high, he will understand there is an issue and he will not try to interprate false values
              Serial.println("Errors converting raw accelerometer data: specified range is not valid");
              Serial.println("returning raw data");
  }
  convertedSample->xData = rangeConverter * rawSample->xRawData;
  convertedSample->yData = rangeConverter * rawSample->yRawData;
  convertedSample->zData = rangeConverter * rawSample->zRawData;
  convertedSample->sampleRelativeTimeMicroSecond = rawSample->sampleRelativeTimeMicroSecond;
}
