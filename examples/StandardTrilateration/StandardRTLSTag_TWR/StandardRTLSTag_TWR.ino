/*
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

/* 
 * StandardRTLSTag_TWR.ino
 * 
 * This is an example tag in a RTLS using two way ranging ISO/IEC 24730-62_2013 messages
 */

#include <DW1000Ng.hpp>
#include <DW1000NgUtils.hpp>
#include <DW1000NgTime.hpp>
#include <DW1000NgConstants.hpp>
#include <DW1000NgRanging.hpp>
#include <DW1000NgRTLS.hpp>

#include <MAVLink_ardupilotmega.h>


// connection pins
#if defined(ESP32)
const uint8_t PIN_RST = 4; // ESP32
const uint8_t PIN_SS = SS; // spi select pin

//const uint8_t PIN_RST = 27; // ESP32
//const uint8_t PIN_SS = 4; // spi select pin
#else
const uint8_t PIN_RST = 9;
const uint8_t PIN_SS = A10; // spi select pin
#endif

// Extended Unique Identifier register. 64-bit device identifier. Register file: 0x01
const char EUI[] = "AA:BB:CC:DD:EE:FF:00:00";

uint16_t personal_short_address = 0;
byte personal_short_address_byte[2];
uint16_t net_id = RTLS_APP_ID;
byte net_id_byte[2];

typedef struct Position {
    double x;
    double y;
} Position;


double x = 0;
double y = 0;
// known anchor positions (meters) - set according to your deployment
/*Position position_main = {0,0};
Position position_B = {4.4,0};
Position position_C = {0,4};*/

Position position_main = {3,2};
Position position_B = {0,0};
Position position_C = {3,-2.5};

// known anchor short anchors_ids
AnchorList anchors_ids = {
    .Anchor_main_short_id = 1,
    .Anchor_B_short_id = 2,
    .Anchor_C_short_id = 3
};

int nbr_anchor = 3;

uint16_t anchor_ids[3] = {anchors_ids.Anchor_main_short_id, anchors_ids.Anchor_B_short_id, anchors_ids.Anchor_C_short_id};

byte target_anchor[] = {0,0, 0,0, 0,0};



void calculatePosition(double &x, double &y, double r_main, double r_B, double r_C) {
    // Convert from cm to meters    
    double A = ( (-2*position_main.x) + (2*position_B.x) );
    double B = ( (-2*position_main.y) + (2*position_B.y) );
    double C = (r_main*r_main) - (r_B*r_B) - (position_main.x*position_main.x) + (position_B.x*position_B.x) - (position_main.y*position_main.y) + (position_B.y*position_B.y);
    double D = ( (-2*position_B.x) + (2*position_C.x) );
    double E = ( (-2*position_B.y) + (2*position_C.y) );
    double F = (r_B*r_B) - (r_C*r_C) - (position_B.x*position_B.x) + (position_C.x*position_C.x) - (position_B.y*position_B.y) + (position_C.y*position_C.y);

    x = (C*E-F*B) / (E*A-B*D);
    y = (C*D-A*F) / (B*D-A*E);
}


// MAVLINK

// CS : 
#define LAT_ORIGINE  48.709108
#define LON_ORIGINE  2.167724
#define ALT_ORIGINE  126


// Australie : 
//#define LAT_ORIGINE  -31.9508512
//#define LON_ORIGINE  115.863278
//#define ALT_ORIGINE  10

uint64_t boot_time_us;
unsigned long last_slot_ms = 0;
const unsigned long SEND_POS_TIMEOUT = 90;
volatile float roll = 0.0, pitch = 0.0, yaw = 0.0;
bool heartbeat_received = false;
uint8_t fc_sysid = 0;

mavlink_message_t msg;
mavlink_status_t status;


void sendVisionPosition() {
  //static unsigned long last_send = 0;
  //if (millis() - last_send >= 100) {  // 10 Hz
    //last_send = millis();

    float covariance[21];
    for (int i = 0; i < 21; i++) covariance[i] = 0;
    //for (int i = 0; i < 21; i++) covariance[i] = NAN;
    //uint64_t usec = micros() - boot_time_us;
    uint64_t usec = 0;

    mavlink_message_t vision_msg;
    mavlink_msg_vision_position_estimate_pack(
      fc_sysid, 197, &vision_msg,
      usec,
      y, x, 0, // N, E, D donc y=N, x=E, z=imagine le baro marche
      roll, pitch, yaw,
      covariance,
      0
    );
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buffer, &vision_msg);
    Serial.write(buffer, len);
  //}
}

void taskReceiveMAVLink() {
    while (Serial.available() > 0) {
        uint8_t byte = Serial.read();
        if (mavlink_parse_char(MAVLINK_COMM_0, byte, &msg, &status)) {
            if (msg.msgid == MAVLINK_MSG_ID_HEARTBEAT) {
                if (!heartbeat_received && msg.compid == 1) {
                    fc_sysid = msg.sysid;
                    heartbeat_received = true;
                    initEKFHome();
                }
            }

            if (msg.msgid == MAVLINK_MSG_ID_ATTITUDE) {
                mavlink_attitude_t attitude;
                mavlink_msg_attitude_decode(&msg, &attitude);
                roll = attitude.roll;
                pitch = attitude.pitch;
                yaw = attitude.yaw;
            }
        }
    }
}

void initEKFHome (){
    // ==== COMMAND_INT ====
    mavlink_message_t cmd_msg;
    mavlink_msg_command_int_pack( // Home
      fc_sysid, 197, &cmd_msg,
      fc_sysid, 1,
      0,
      179,
      0, 0,
      0, 0, 0, 0,
      (int32_t)(LAT_ORIGINE * 1e7),
      (int32_t)(LON_ORIGINE * 1e7),
      ALT_ORIGINE
    );
    uint8_t buf1[MAVLINK_MAX_PACKET_LEN];
    uint16_t len1 = mavlink_msg_to_send_buffer(buf1, &cmd_msg);
    Serial.write(buf1, len1);

    // ==== SET_GPS_GLOBAL_ORIGIN ====
    mavlink_message_t origin_msg;
    uint64_t usec = micros() - boot_time_us;
    mavlink_msg_set_gps_global_origin_pack(
      fc_sysid, 197, &origin_msg,
      fc_sysid,
      (int32_t)(LAT_ORIGINE * 1e7),
      (int32_t)(LON_ORIGINE * 1e7),
      (int32_t)(ALT_ORIGINE * 1000),
      usec
    );
    uint8_t buf2[MAVLINK_MAX_PACKET_LEN];
    uint16_t len2 = mavlink_msg_to_send_buffer(buf2, &origin_msg);
    Serial.write(buf2, len2);
}

device_configuration_t DEFAULT_CONFIG = {
    false,
    true,
    false,
    true,
    false,
    SFDMode::STANDARD_SFD,
    Channel::CHANNEL_7,
    DataRate::RATE_6800KBPS,
    PulseFrequency::FREQ_64MHZ,
    PreambleLength::LEN_256,
    PreambleCode::CODE_9
};

frame_filtering_configuration_t TAG_FRAME_FILTER_CONFIG = {
    false,
    false,
    true, // true
    false,
    false,
    false,
    false,
    false
};

void setup() {
    // DEBUG monitoring
    Serial.begin(115200);
    boot_time_us = micros();
    delay(1000);
    Serial.println(F("### DW1000Ng-arduino-ranging-tag ###"));
    // initialize the driver
    #if defined(ESP8266)
    DW1000Ng::initializeNoInterrupt(PIN_SS);
    #else
    DW1000Ng::initializeNoInterrupt(PIN_SS, PIN_RST);
    #endif
    Serial.println("DW1000Ng initialized ...");
    // general configuration
    DW1000Ng::applyConfiguration(DEFAULT_CONFIG);
    DW1000Ng::enableFrameFilteringACK(TAG_FRAME_FILTER_CONFIG);
    
    DW1000Ng::setEUI(EUI);

    DW1000Ng::setPreambleDetectionTimeout(170);
    DW1000Ng::setSfdDetectionTimeout(273);
    //DW1000Ng::setSfdDetectionTimeout(350);
    DW1000Ng::setReceiveFrameWaitTimeoutPeriod(1500);

    DW1000Ng::setNetworkId(net_id);
    DW1000Ng::setDeviceAddress(personal_short_address);
    DW1000Ng::setAntennaDelay(16415);
    DW1000Ng::setTXPower(0x1F1F1F1F);
    //DW1000Ng::setTXPower(0x3F3F3F3F);
    //DW1000Ng::setTXPower(0x85858585);
    
    // Precalculate values to avoid useless clock cycles later on
    DW1000NgUtils::writeValueToBytes(net_id_byte, (uint16_t)net_id, 2);
    DW1000NgUtils::writeValueToBytes(personal_short_address_byte, (uint16_t)personal_short_address, 2);

    DW1000NgUtils::writeValueToBytes(target_anchor, anchor_ids[0], 2);
    DW1000NgUtils::writeValueToBytes(target_anchor+2, anchor_ids[1], 2);
    DW1000NgUtils::writeValueToBytes(target_anchor+4, anchor_ids[2], 2);
    
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
    
}

bool debug = false;
bool is_write_register_set = false;
void loop() {
    if (!is_write_register_set) {
        DW1000NgRTLS::writePoll(&target_anchor[0], net_id_byte, personal_short_address_byte);
        is_write_register_set = true;
    }
    if(DW1000NgRTLS::receiveFrame()){
        size_t recv_len = DW1000Ng::getReceivedDataLength();
        byte recv_data[recv_len];
        DW1000Ng::getReceivedData(recv_data, recv_len);

        if(recv_data[9] == 0xA0) {
            uint32_t tt = micros();
            RangeInfrastructureResultDouble result = RangeMultipleAnchors();
            
            if(result.success) {
                //double x,y;
                calculatePosition(x, y, result.Range_main, result.Range_B, result.Range_C);
                if (debug) {Serial.print((String)"t:" + (uint32_t)(micros() - tt));}
                String positioning = "fndpos:";
                positioning += x; positioning += ":";
                positioning += y; positioning += ":a"; //Serial.println(positioning);
            }
            else if (debug){
                Serial.println((String)"Fail global:" + (uint32_t)(micros() - tt));
            }
        }
    }
    taskReceiveMAVLink();

    if (millis() - last_slot_ms >= SEND_POS_TIMEOUT) {
        last_slot_ms = millis();
        String positioning = "POS:";
        positioning += x; positioning += ":";
        positioning += y; positioning += ":a"; //Serial.println(positioning);
        sendVisionPosition();
    }
}


RangeInfrastructureResultDouble RangeMultipleAnchors(){
    RangeInfrastructureResultDouble returnValues;
    returnValues = {true, 0, 0, 0};
    
    for(uint16_t anchor_index = 0; anchor_index < 3; anchor_index++) {
        //String toprint = "ranging with :"; toprint += anchor_ids[anchor_index]; Serial.println(toprint);
        uint32_t ttt = micros();
        RangeAcceptResult result = start_ranging_fct(anchor_index);//anchor_ids[anchor_index]);
        
        if(!result.success) {
            returnValues.success = false;
            if (debug) {Serial.println((String)"Error:" + (uint16_t)result.range);}
            return returnValues;
        }
        else {
            if (debug) {Serial.println((String)"indx:" + anchor_index + " range:" + result.range);}
            if (anchor_index == 0) {
                //Serial.println((String)"t:" + (uint32_t)(micros() - ttt));
                //Serial.print("r:"); Serial.println(result.range);
                returnValues.Range_main = result.range;
            } else if (anchor_index == 1) {
                //Serial.println((String)"t:" + (uint32_t)(micros() - ttt));
                //Serial.print("r:"); Serial.println(result.range);
                returnValues.Range_B = result.range;
            } else if (anchor_index == 2) {
                returnValues.Range_C = result.range;
            }
        }
    }
    return returnValues;
}

byte anchor_eui[] = {0xAA,0xBB,0xCC,0xDD,0xEE,0xFF,0x00,0x01};

uint64_t timeStartRangingReceived;
uint64_t timePollSent;
uint64_t timeACTReceived;
uint64_t timeFinalMessageSent;

RangeAcceptResult start_ranging_fct(uint16_t index) {
    //if (debug) {Serial.println((String)"Starting ranging:" + address);}
    RangeAcceptResult returnValue;
    //timeStartRangingReceived = DW1000Ng::getReceiveTimestamp();
    // Sending POLL

    // To convert uint16_t to byte
    //byte target_anchor[2];
    //DW1000NgUtils::writeValueToBytes(target_anchor, address, 2);

    if (!is_write_register_set) {
        DW1000NgRTLS::transmitPoll(&target_anchor[index+index], net_id_byte, personal_short_address_byte);
    }
    else {
        DW1000Ng::startTransmit();
        is_write_register_set = false;
    }
    DW1000NgRTLS::waitForTransmission();

    timePollSent = DW1000Ng::getTransmitTimestamp();


    // Receive ACTIVITY_CONTROL
    if(!DW1000NgRTLS::receiveFrame()) { //Serial.println((String)"Error 1" + ", tstp: " + micros());
        return {false, 1};}

    size_t act_ctrl_len = DW1000Ng::getReceivedDataLength();
    byte act_ctrl_data[act_ctrl_len];
    DW1000Ng::getReceivedData(act_ctrl_data, act_ctrl_len);

    if (!(act_ctrl_len > 9 && act_ctrl_data[9] == ACTIVITY_CONTROL)) { //Serial.println("Error 2");
        return {false, 2};}

    // Transmit final message
    
    timeACTReceived = DW1000Ng::getReceiveTimestamp();
    //DW1000NgRTLS::transmitFinalMessageEmpty(&act_ctrl_data[7], net_id_byte, personal_short_address_byte);
    DW1000NgRTLS::waitForTransmission();
    timeFinalMessageSent = DW1000Ng::getTransmitTimestamp();


    // Receive modified ranging confirm
    //if (index == 0){
        uint16_t index_tmp = (index+1)%nbr_anchor;
        DW1000NgRTLS::writePoll(&target_anchor[index_tmp+index_tmp], net_id_byte, personal_short_address_byte);
        is_write_register_set = true;
    //}
    if(!DW1000NgRTLS::receiveFrame()) { //Serial.println("Error 3");
        return {false, 3};}
    
    size_t rfinal_len = DW1000Ng::getReceivedDataLength();
    byte rfinal_data[rfinal_len];
    DW1000Ng::getReceivedData(rfinal_data, rfinal_len);


    if(!(rfinal_len > 18 && rfinal_data[9] == ACTIVITY_CONTROL)) { //Serial.println("Error 4");
        return {false, 4};}
    
    //uint64_t timeFinalDataReceived = DW1000Ng::getReceiveTimestamp();
    //uint64_t timeBeforeCalcRange = micros();
    //returnValue.range = 1.11;
    returnValue.range = DW1000NgRanging::computeRangeAsymmetric(
        timePollSent, // Poll send time
        DW1000NgUtils::bytesAsValue(rfinal_data + 10, LENGTH_TIMESTAMP),  // timePollReceived
        DW1000NgUtils::bytesAsValue(rfinal_data + 14, LENGTH_TIMESTAMP),  // timeResponseToPoll // Response to poll sent time
        timeACTReceived, // Response to Poll Received
        timeFinalMessageSent, // Final Message send time
        DW1000NgUtils::bytesAsValue(rfinal_data + 18, LENGTH_TIMESTAMP)   // timeFinalMessageReceive // Final message receive time
    );

    //uint64_t timeAfterCalcRange = micros();

    returnValue.range = DW1000NgRanging::correctRange(returnValue.range, DEFAULT_CONFIG.channel, DEFAULT_CONFIG.pulseFreq);
    
    //uint64_t timeCalculRange = (uint64_t)(timeAfterCalcRange - timeBeforeCalcRange);
    //uint64_t timeCorrectRange = (uint64_t)(micros() - timeAfterCalcRange);DEFAULT_CONFIG


    /*String toprint; 
    toprint = "btwn1:"; toprint += (uint64_t)(timeACTReceived-timeStartRangingReceived);
    toprint += "\nround1:"; toprint += (uint64_t)(timeACTReceived-timePollSent);
    toprint += "\nreply1:"; toprint += (uint64_t)(DW1000NgUtils::bytesAsValue(rfinal_data + 14, 4)-DW1000NgUtils::bytesAsValue(rfinal_data + 10, 4));
    toprint += "\nround2:"; toprint += (uint64_t)(DW1000NgUtils::bytesAsValue(rfinal_data + 18, 4)-DW1000NgUtils::bytesAsValue(rfinal_data + 14, 4));
    toprint += "\nreply2:"; toprint += (uint64_t)(timeFinalMessageSent-timeACTReceived);
    toprint += "\nbtwn2:"; toprint += (uint64_t)(timeFinalDataReceived-timeACTReceived);
    //toprint += "\ntimeCalculRange:"; toprint += (uint64_t)(timeCalculRange);
    //toprint += "\ntimeCorrectRange:"; toprint += (uint64_t)(timeCorrectRange);
    Serial.println(toprint);*/


    if (returnValue.range < -1 | returnValue.range > 150) {
        return {false, 5};
    }
    returnValue.success = true;

    //Serial.println((String)"range: " + returnValue.range);// + " tstp:" + micros());
    //returnValue = {true, range};
    return returnValue;
}
