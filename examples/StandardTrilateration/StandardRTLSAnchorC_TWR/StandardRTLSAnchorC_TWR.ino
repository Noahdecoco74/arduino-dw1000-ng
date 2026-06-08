/*
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

/* 
 * StandardRTLSAnchorB_TWR.ino
 * 
 * This is an example slave anchor in a RTLS using two way ranging ISO/IEC 24730-62_2013 messages
 */

#include <DW1000Ng.hpp>
#include <DW1000NgUtils.hpp>
#include <DW1000NgRanging.hpp>
#include <DW1000NgRTLS.hpp>

// connection pins
#if defined(ESP32)
const uint8_t PIN_RST = 27; // ESP32 makerfabs
const uint8_t PIN_SS = 4;
//const uint8_t PIN_RST = 4; // ESP32
//const uint8_t PIN_SS = SS; // spi select pin
#else
const uint8_t PIN_RST = 9;
const uint8_t PIN_SS = A10; // spi select pin
#endif

// Extended Unique Identifier register. 64-bit device identifier. Register file: 0x01
const char EUI[] = "AA:BB:CC:DD:EE:FF:00:03";

uint16_t personal_short_address = 3;
byte personal_short_address_byte[2];
uint16_t net_id = RTLS_APP_ID;
byte net_id_byte[2];

device_configuration_t DEFAULT_CONFIG = {
    false,
    true,
    false, // smartTX
    true,
    false,
    SFDMode::STANDARD_SFD,
    Channel::CHANNEL_7,
    DataRate::RATE_6800KBPS,
    PulseFrequency::FREQ_64MHZ,
    PreambleLength::LEN_256,
    PreambleCode::CODE_9
};

frame_filtering_configuration_t ANCHOR_FRAME_FILTER_CONFIG = {
    false,
    false,
    true,
    true,
    false,
    false,
    false,
    false
};

void setup() {
    // DEBUG monitoring
    Serial.begin(115200);
    delay(1000);
    Serial.println(F("### arduino-DW1000Ng-ranging-anchor-B ###"));
    // initialize the driver
    #if defined(ESP8266)
    DW1000Ng::initializeNoInterrupt(PIN_SS);
    #else
    DW1000Ng::initializeNoInterrupt(PIN_SS, PIN_RST);
    #endif
    Serial.println(F("DW1000Ng initialized ..."));
    // general configuration
    DW1000Ng::applyConfiguration(DEFAULT_CONFIG);
    DW1000Ng::enableFrameFilteringACK(ANCHOR_FRAME_FILTER_CONFIG);
    //DW1000Ng::disableFrameFiltering();
    
    DW1000Ng::setEUI(EUI);

    DW1000Ng::setPreambleDetectionTimeout(170);
    DW1000Ng::setSfdDetectionTimeout(273);
    DW1000Ng::setReceiveFrameWaitTimeoutPeriod(5000);

    DW1000Ng::setNetworkId(net_id);
    DW1000Ng::setDeviceAddress(personal_short_address);
    DW1000Ng::setAntennaDelay(16411);
    DW1000Ng::setTXPower(0x1F1F1F1F);
    //DW1000Ng::setTXPower(0x3F3F3F3F);
    //DW1000Ng::setTXPower(0x85858585);

    DW1000NgUtils::writeValueToBytes(net_id_byte, (uint16_t)net_id, 2);
    DW1000NgUtils::writeValueToBytes(personal_short_address_byte, (uint16_t)personal_short_address, 2);

    
    Serial.println(F("Committed configuration ..."));
    // DEBUG chip info and registers pretty printed
    char msg[128];
    DW1000Ng::getPrintableDeviceIdentifier(msg);
    Serial.print("Device ID: "); Serial.println(msg);
    DW1000Ng::getPrintableExtendedUniqueIdentifier(msg);
    Serial.print("Unique ID: "); Serial.println(msg);
    DW1000Ng::getPrintableNetworkIdAndShortAddress(msg);
    Serial.print("Network ID & Device Address: "); Serial.println(msg);
    DW1000Ng::getPrintableDeviceMode(msg);
    Serial.print("Device mode: "); Serial.println(msg);
    //DW1000Ng::setDoubleBuffering(true);
    DW1000Ng::setWait4Response(20);
}

void loop() {
    //while (true) {
    bool result = ranging_fct();
    //}
    //if(!result) {
    //    Serial.println("Ranging failed");
        //return;
    //};
}

uint32_t timePollReceived;
uint32_t timeResponseToPoll;
bool ranging_fct() {
    // Receive POLL
    if(!DW1000NgRTLS::receiveFrame()) {
        //Serial.println("Error 1");
        return false;
    }

    size_t init_len = DW1000Ng::getReceivedDataLength();
    byte recv_data[init_len];
    DW1000Ng::getReceivedData(recv_data, init_len);

    if (!(init_len >= 9 && recv_data[9] == RANGING_TAG_POLL)){// && DW1000NgUtils::bytesAsValue(&recv_data[5], 2)==personal_short_address)){
        //Serial.print("Error 2");
        String toprint = "Received Frame : ";
        toprint += (uint16_t)DW1000NgUtils::bytesAsValue(&recv_data[5], 2);
        toprint += " : ";
        toprint += (uint16_t)DW1000NgUtils::bytesAsValue(&recv_data[7], 2);
        toprint += " : ";
        toprint += recv_data[9];
        //Serial.println(toprint);
        return false;
    }
    else {
        String toprint = "Received Frame : ";
        toprint += (uint16_t)DW1000NgUtils::bytesAsValue(&recv_data[5], 2);
        toprint += " : ";
        toprint += (uint16_t)DW1000NgUtils::bytesAsValue(&recv_data[7], 2);
        toprint += " : ";
        toprint += recv_data[9];
        //Serial.println(toprint);
    }

    // Send ACTIVITY_CONTROL (response to poll)
    
    DW1000NgRTLS::transmitResponseToPollACK(&recv_data[7], &recv_data[3], &recv_data[5]);
    timePollReceived = DW1000Ng::getReceiveTimestampShort();
    DW1000NgRTLS::waitForTransmission();
    
    timeResponseToPoll = DW1000Ng::getTransmitTimestampShort();
    // Receive final message
    if(!DW1000NgRTLS::receiveFrameACK()) {
        Serial.println("Error 3");
        return false;
    }
    
    //size_t rfinal_len = DW1000Ng::getReceivedDataLength();
    //byte ack_data[rfinal_len];
    //DW1000Ng::getReceivedData(ack_data, rfinal_len);

    if(!(DW1000Ng::getReceivedDataLength() == 3)){// && ack_data[0] & 0x07)) {
        Serial.println("Error 4");
        return false;
    }

    // Send modified ranging confirm
    DW1000NgRTLS::transmitRangingConfirmExtended(
        &recv_data[7],
        timePollReceived,
        timeResponseToPoll,
        DW1000Ng::getReceiveTimestampShort(), 
        &recv_data[3], 
        &recv_data[5]);
    //Serial.println("Final success");
    return true;
}