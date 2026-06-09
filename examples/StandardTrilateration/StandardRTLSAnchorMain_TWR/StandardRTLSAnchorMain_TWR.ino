/*
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

/* 
 * StandardRTLSAnchorMain_TWR.ino
 * 
 * This is an example master anchor in a RTLS using two way ranging ISO/IEC 24730-62_2013 messages
 */

#include <DW1000Ng.hpp>
#include <DW1000NgUtils.hpp>
#include <DW1000NgConstants.hpp>
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

bool debug = false;

// Extended Unique Identifier register. 64-bit device identifier. Register file: 0x01
const char EUI[] = "AA:BB:CC:DD:EE:FF:00:01";

uint16_t personal_short_address = 1;
byte personal_short_address_byte[2];
uint16_t net_id = RTLS_APP_ID;
byte net_id_byte[2];

// Tags to schedule (add up to N tags here)
uint16_t tags[2] = {0, 11}; // expand as needed
byte tags_byte[4];
const uint8_t TAG_COUNT = 2;
uint8_t current_tag = TAG_COUNT - 1;
unsigned long last_slot_ms = 0;
const unsigned long SLOT_PERIOD_MS = 10; // durée pratique maximale du positionnement d'un drone


AnchorList anchors_ids = {
    .Anchor_main_short_id = 1,
    .Anchor_B_short_id = 2,
    .Anchor_C_short_id = 3
};

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
    true, // true
    false,
    false,
    false,
    false,
    true    // true/* This allows blink frames */
};

void setup() {
    // DEBUG monitoring
    Serial.begin(115200);
    delay(1000);
    Serial.println(F("### DW1000Ng-arduino-ranging-anchorMain ###"));
    // initialize the driver
    #if defined(ESP8266)
    DW1000Ng::initializeNoInterrupt(PIN_SS);
    #else
    DW1000Ng::initializeNoInterrupt(PIN_SS, PIN_RST);
    #endif
    Serial.println(F("DW1000Ng initialized ..."));
    // general configuration
    DW1000Ng::applyConfiguration(DEFAULT_CONFIG);
    //DW1000Ng::enableFrameFiltering(ANCHOR_FRAME_FILTER_CONFIG);
    DW1000Ng::disableFrameFiltering();
    
    DW1000Ng::setEUI(EUI);


    DW1000Ng::setPreambleDetectionTimeout(170);
    DW1000Ng::setSfdDetectionTimeout(273);
    DW1000Ng::setReceiveFrameWaitTimeoutPeriod(1500);

    DW1000Ng::setNetworkId(net_id);
    DW1000Ng::setDeviceAddress(personal_short_address);
    DW1000Ng::setAntennaDelay(16415);
    DW1000Ng::setTXPower(0x1F1F1F1F);
    //DW1000Ng::setTXPower(0x85858585);

    // Precalculate values to avoid useless clock cycles later on
    DW1000NgUtils::writeValueToBytes(net_id_byte, (uint16_t)net_id, 2);
    DW1000NgUtils::writeValueToBytes(personal_short_address_byte, (uint16_t)personal_short_address, 2);
    
    DW1000NgUtils::writeValueToBytes(tags_byte, (uint16_t)tags[0], 2);

    
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
    DW1000Ng::setWait4Response(20);
    //DW1000Ng::setDoubleBuffering(true);
    //DW1000Ng::startReceive();
}

uint32_t tt = 0;
bool failed_last_ranging = false;
void loop() {
    bool sniffed_last_message_from_anchor = false;
    // coordinator: periodically tell next tag to start ranging
    if(DW1000NgRTLS::receiveFrame()) {
        size_t init_len = DW1000Ng::getReceivedDataLength();
        byte recv_data[init_len];
        DW1000Ng::getReceivedData(recv_data, init_len);


        if(init_len > 18 && recv_data[9] == ACTIVITY_CONTROL) {
            String toprint = "Received ACTIVITY_CONTROL : ";
            toprint += (uint16_t)DW1000NgUtils::bytesAsValue(&recv_data[5], 2);
            toprint += " : ";
            toprint += (uint16_t)DW1000NgUtils::bytesAsValue(&recv_data[7], 2);
            if (debug) Serial.println(toprint);

            if (DW1000NgUtils::bytesAsValue(&recv_data[5], 2) == tags[current_tag] && 
                DW1000NgUtils::bytesAsValue(&recv_data[7], 2) == anchors_ids.Anchor_C_short_id) {

                //Serial.println((String)"Succès, tstp: " + );
                Serial.println((String)"Succès, tstp:" + (uint32_t)(micros() - tt));
                tt = micros();
                sniffed_last_message_from_anchor = true;
            }
        
            else {
                String toprint = "Received Frame : ";
                toprint += (uint16_t)DW1000NgUtils::bytesAsValue(&recv_data[5], 2);
                toprint += " : ";
                toprint += (uint16_t)DW1000NgUtils::bytesAsValue(&recv_data[7], 2);
                toprint += " : ";
                toprint += recv_data[9];
                if (debug) Serial.println(toprint);
            }
        }
    }

    if(millis() - last_slot_ms >= SLOT_PERIOD_MS | sniffed_last_message_from_anchor | failed_last_ranging) {
        failed_last_ranging = false;
        current_tag = (current_tag + 1) % TAG_COUNT;
        byte target_tag[2];
        DW1000NgUtils::writeValueToBytes(target_tag, tags[current_tag], 2);
        transmitStartRanging(target_tag);
        last_slot_ms = millis();
        Serial.print("Strt tag: "); Serial.print(current_tag); Serial.print("id_byt"); Serial.println(tags[current_tag]);
        //tt = micros();
        
        // Si on n'a pas réussi le dernier ranging : 
            // On part du principe que le message qu'on devait recevoir est passé
            // Donc on passe directement au tag suivant
        failed_last_ranging = !ranging_fct(target_tag);

    }
}

void transmitStartRanging(byte dst[2]) {
    // small control frame: opcode 0xA0 means START_RANGING
    byte ctrl[] = {DATA, SHORT_SRC_AND_DEST, DW1000NgRTLS::increaseSequenceNumber(), 0,0, 0,0, 0,0, 0xA0};
    memcpy(&ctrl[3], net_id_byte, 2);
    memcpy(&ctrl[5], dst, 2); // destination: tag
    //ctrl[5] = 0x00; ctrl[6] = 0x00;     // broadcast to all 
    memcpy(&ctrl[7], personal_short_address_byte, 2);

    DW1000Ng::setTransmitData(ctrl, sizeof(ctrl));
    DW1000Ng::startTransmit();
    DW1000NgRTLS::waitForTransmission(); // utile ? -> essentiel
}

uint32_t timePollReceived;
uint32_t timeResponseToPoll;
bool writenResponseToPollACK = false;
bool ranging_fct(byte dst[2]) {
    // Receive POLL
    if (writenResponseToPollACK){
        writenResponseToPollACK = true; //(uint16_t)(current_tag + current_tag) // tags_byte[0]
        byte target_tag[2];
        DW1000NgUtils::writeValueToBytes(target_tag, tags[current_tag], 2);
        //byte local_address[2] = {0, 0};
        //DW1000NgUtils::writeValueToBytes(local_address, address, 2);
        DW1000NgRTLS::writeResponseToPollACK(target_tag, net_id_byte, personal_short_address_byte);
    }
    if(!DW1000NgRTLS::receiveFrameACK()) {
        Serial.println("Error 1");
        return false;
    }

    size_t init_len = DW1000Ng::getReceivedDataLength();
    byte recv_data[init_len];
    DW1000Ng::getReceivedData(recv_data, init_len);

    if (!(init_len > 9 and recv_data[9] == RANGING_TAG_POLL)){
        Serial.println("Error 2");
        return false;
    }

    // Send ACTIVITY_CONTROL (response to poll)
    if (writenResponseToPollACK){
        DW1000Ng::startTransmit();
        writenResponseToPollACK = false;
    }
    else {
        DW1000NgRTLS::transmitResponseToPollACK(&recv_data[7], &recv_data[3], &recv_data[5]);
    }
    
    timePollReceived = DW1000Ng::getReceiveTimestampShort();// L'accès au bus SPI est long, donc on le fait pdt le temps mort
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
    DW1000NgRTLS::waitForTransmission();
    //DW1000Ng::startReceive();
    if (debug) Serial.println("Final succes");
    return true;
}