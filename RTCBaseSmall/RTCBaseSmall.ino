#include <Wire.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
SFE_UBLOX_GNSS myGNSS;

//#define USE_SERIAL1 // Uncomment this line to push the RTCM data to Serial1

void setup() {
  Serial.begin(115200);
  while (!Serial);

#ifdef USE_SERIAL1
  Serial1.begin(115200);
#endif

  Wire.begin();
  Wire.setClock(400000);

  if (!myGNSS.begin()) {
    Serial.println(F("GNSS not detected. Check wiring."));
    while (1);
  }

  // Ensure RTCM3 is enabled and save settings
  myGNSS.setI2COutput(COM_TYPE_UBX | COM_TYPE_RTCM3);
  myGNSS.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT);

  // Enable RTCM messages
  if (!enableRTCM()) {
    Serial.println(F("Failed to enable RTCM"));
    while (1);
  }

  if (!myGNSS.getSurveyStatus(2000)) {
    Serial.println(F("Failed to get Survey-In status"));
    while (1);
  }

  if (!myGNSS.getSurveyInActive()) {
    if (!myGNSS.enableSurveyMode(60, 5.0)) {
      Serial.println(F("Survey start failed."));
      while (1);
    }
    Serial.println(F("Survey started for 60s, 5m accuracy."));
  }

  while (myGNSS.getSurveyInValid() == false) {
    delay(1000);
    if (Serial.available() && Serial.read() == 'x') {
      myGNSS.disableSurveyMode();
      Serial.println(F("Survey stopped."));
      break;
    }

    if (myGNSS.getSurveyStatus(2000)) {
      Serial.print(F("Survey time: "));
      Serial.print(myGNSS.getSurveyInObservationTimeFull());
      Serial.print(F(" Accuracy: "));
      Serial.println(myGNSS.getSurveyInMeanAccuracy());
    }
  }

  Serial.println(F("Survey complete. RTCM now broadcasting."));
  myGNSS.setI2COutput(COM_TYPE_UBX | COM_TYPE_RTCM3);
}

void loop() {
  myGNSS.checkUblox();
  delay(250);
}

bool enableRTCM() {
  bool response = true;
  response &= myGNSS.enableRTCMmessage(UBX_RTCM_1005, COM_PORT_I2C, 1);
  response &= myGNSS.enableRTCMmessage(UBX_RTCM_1074, COM_PORT_I2C, 1);
  response &= myGNSS.enableRTCMmessage(UBX_RTCM_1084, COM_PORT_I2C, 1);
  response &= myGNSS.enableRTCMmessage(UBX_RTCM_1094, COM_PORT_I2C, 1);
  response &= myGNSS.enableRTCMmessage(UBX_RTCM_1124, COM_PORT_I2C, 1);
  response &= myGNSS.enableRTCMmessage(UBX_RTCM_1230, COM_PORT_I2C, 1); // message every 1 second
  return response;
}

// RTCM processing function
void SFE_UBLOX_GNSS::processRTCM(uint8_t incoming) {
#ifdef USE_SERIAL1
  Serial1.write(incoming);
#endif
  if (myGNSS.rtcmFrameCounter % 16 == 0) Serial.println();
  Serial.print(F(" "));
  if (incoming < 0x10) Serial.print(F("0"));
  Serial.print(incoming, HEX);
}
