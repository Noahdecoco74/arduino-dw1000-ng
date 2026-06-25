#include <DW1000Ng.hpp>

bool debug = false;

// connection pins
const uint8_t PIN_RST = 27; // ESP32
const uint8_t PIN_SS = 4; // spi select pin
uint16_t net_id = RTLS_APP_ID;
byte net_id_byte[2];
byte personal_short_address_byte[2];


int input;
int next_ranging_id;

const int round_trip = 1000;
int n = round_trip;
float list_dist[round_trip];




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

uint32_t delay_tx = 0;
uint32_t delay_rx = 0;

inline void calSetup(){
    // DEBUG monitoring
    Serial.begin(115200);
    delay(1000);
    Serial.println(F("### DW1000Ng-arduino-ranging-tag ###"));
    // initialize the driver
    DW1000Ng::initializeNoInterrupt(PIN_SS, PIN_RST);
    Serial.println("DW1000Ng initialized ...");
    // general configuration
    DW1000Ng::applyConfiguration(DEFAULT_CONFIG);
    DW1000Ng::disableFrameFiltering();
    
    DW1000Ng::setEUI(EUI);

    DW1000Ng::setPreambleDetectionTimeout(170);
    DW1000Ng::setSfdDetectionTimeout(273);
    DW1000Ng::setReceiveFrameWaitTimeoutPeriod(15000);

    DW1000Ng::setNetworkId(net_id);
    DW1000Ng::setDeviceAddress(personal_short_address);
    

    #ifndef CALIBRATE_DELAYS
    delay_tx = (uint16_t)(0.44*antennaDelay);
    delay_rx = (uint16_t)(0.56*antennaDelay);
    DW1000Ng::setTxAntennaDelay(delay_tx);
    DW1000Ng::setRxAntennaDelay(delay_rx);
    #else
    DW1000Ng::setAntennaDelay(0);
    #endif
    //DW1000Ng::setTXPower(0x1F1F1F1F);
    DW1000Ng::setTXPower(0xC0C0C0C0);
    //DW1000Ng::setTXPower(0x85858585);
    
    // Precalculate values to avoid useless clock cycles later on
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
    DW1000Ng::setWait4Response(200);
}




boolean receiveFrameACK() {
    DW1000Ng::startReceive();
    uint32_t time_start = micros();
    while(!DW1000Ng::isReceiveDone()) {
        if(micros() > time_start + 15000) {
            DW1000Ng::clearReceiveTimeoutStatus();
            return false;
        }
    }
    DW1000Ng::clearReceiveStatus();
    return true;
}

/**
 * Clock Corrected Asynmetric Double Sided Two Way Ranging (CC-ADS-TWR)
 * This work is based on the work done in: doi: 10.1109/IMCCC.2018.00238
 * and improved to support asymetry
 */
double computeRangeAsymmetric(  uint64_t timePollSent, 
                                uint64_t timePollReceived, 
                                uint64_t timePollAckSent, 
                                uint64_t timePollAckReceived,
                                uint64_t timeRangeSent,
                                uint64_t timeRangeReceived
                            )
{
    uint32_t timePollSent_32 = static_cast<uint32_t>(timePollSent);
    uint32_t timePollReceived_32 = static_cast<uint32_t>(timePollReceived);
    uint32_t timePollAckSent_32 = static_cast<uint32_t>(timePollAckSent);
    uint32_t timePollAckReceived_32 = static_cast<uint32_t>(timePollAckReceived);
    uint32_t timeRangeSent_32 = static_cast<uint32_t>(timeRangeSent);
    uint32_t timeRangeReceived_32 = static_cast<uint32_t>(timeRangeReceived);
    
    double round1 = static_cast<double>(timePollAckReceived_32 - timePollSent_32);
    double reply1 = static_cast<double>(timePollAckSent_32 - timePollReceived_32);
    double round2 = static_cast<double>(timeRangeReceived_32 - timePollAckSent_32);
    double reply2 = static_cast<double>(timeRangeSent_32 - timePollAckReceived_32);

    double k_AB = (round1+reply2)/(reply1+round2);

    int64_t tof_uwb = static_cast<int64_t>(k_AB*(round1 * round2 - reply1 * reply2) / (k_AB*(round1 + reply2) + round2 + reply1));
    double distance = tof_uwb * DISTANCE_OF_RADIO;

    return distance;
}

double computeDelays(  uint64_t timePollSent, 
                                uint64_t timePollReceived, 
                                uint64_t timePollAckSent, 
                                uint64_t timePollAckReceived,
                                uint64_t timeRangeSent,
                                uint64_t timeRangeReceived
                            )
{
    uint32_t timePollSent_32 = static_cast<uint32_t>(timePollSent);
    uint32_t timePollReceived_32 = static_cast<uint32_t>(timePollReceived);
    uint32_t timePollAckSent_32 = static_cast<uint32_t>(timePollAckSent);
    uint32_t timePollAckReceived_32 = static_cast<uint32_t>(timePollAckReceived);
    uint32_t timeRangeSent_32 = static_cast<uint32_t>(timeRangeSent);
    uint32_t timeRangeReceived_32 = static_cast<uint32_t>(timeRangeReceived);
    
    double round1 = static_cast<double>(timePollAckReceived_32 - timePollSent_32);
    double reply1 = static_cast<double>(timePollAckSent_32 - timePollReceived_32);
    double round2 = static_cast<double>(timeRangeReceived_32 - timePollAckSent_32);
    double reply2 = static_cast<double>(timeRangeSent_32 - timePollAckReceived_32);

    double k_AB = (round1+reply2)/(reply1+round2);

    //int64_t tof_uwb = static_cast<int64_t>(k_AB*(round1 * round2 - reply1 * reply2) / (k_AB*(round1 + reply2) + round2 + reply1));
    //double distance = tof_uwb * DISTANCE_OF_RADIO;

    int64_t tof = 0.32 * DISTANCE_OF_RADIO_INV;

    double A = -tof*(k_AB + 1/k_AB) + (round1*round2 - reply1*reply2) / (round2+reply1);
    //Serial.println("A:" + String(A));
    //Serial.println("B:" + String(-k_AB));
    //Serial.print("RX power is [dBm] ... "); Serial.println(DW1000Ng::getReceivePower());

    
    double distance = A;

    return distance;
}



uint64_t getTransmitTimestamp() {
    return DW1000Ng::getTransmitTimestamp();
}
uint32_t getTransmitTimestampShort() {
    return DW1000Ng::getTransmitTimestampShort();
}
uint64_t getReceiveTimestamp() {
    return DW1000Ng::getReceiveTimestamp();
}
uint32_t getReceiveTimestampShort() {
    return DW1000Ng::getReceiveTimestampShort();
}



uint64_t timeStartRangingReceived;
uint64_t timePollSent;
uint64_t timeACTReceived;
uint64_t timeFinalMessageSent;

RangeAcceptResult start_ranging_fct(uint16_t index) {
    byte target_tag[2];
    DW1000NgUtils::writeValueToBytes(target_tag, index, 2);
    RangeAcceptResult returnValue;
    // Sending POLL
    DW1000NgRTLS::transmitPoll(target_tag, net_id_byte, personal_short_address_byte);

    DW1000NgRTLS::waitForTransmission(); // Transformer en IRQ

    timePollSent = getTransmitTimestamp();


    // Receive ResponseToPoll : ACTIVITY_CONTROL
    if(!receiveFrameACK()) { //Serial.println((String)"Error 1" + ", tstp: " + micros());
        return {false, 1};}

    size_t act_ctrl_len = DW1000Ng::getReceivedDataLength();
    byte act_ctrl_data[act_ctrl_len];
    DW1000Ng::getReceivedData(act_ctrl_data, act_ctrl_len);

    if (!(act_ctrl_len > 9 && act_ctrl_data[9] == ACTIVITY_CONTROL)) { //Serial.println("Error 2");
        return {false, 2};}

    // Transmit final message
    timeACTReceived = getReceiveTimestamp();
    DW1000NgRTLS::transmitFinalMessageEmpty(&act_ctrl_data[7], net_id_byte, personal_short_address_byte);
    DW1000NgRTLS::waitForTransmission();
    timeFinalMessageSent = getTransmitTimestamp();


    // Receive modified ranging confirm
    if(!receiveFrameACK()) { //Serial.println("Error 3");
        return {false, 3};}
    
    size_t rfinal_len = DW1000Ng::getReceivedDataLength();
    byte rfinal_data[rfinal_len];
    DW1000Ng::getReceivedData(rfinal_data, rfinal_len);


    if(!(rfinal_len > 18 && rfinal_data[9] == ACTIVITY_CONTROL)) { //Serial.println("Error 4");
        return {false, 4};}
        
    #ifdef CALIBRATE_DELAYS
    returnValue.range = computeDelays(
        timePollSent, // Poll send time
        DW1000NgUtils::bytesAsValue(rfinal_data + 10, LENGTH_TIMESTAMP),  // timePollReceived
        DW1000NgUtils::bytesAsValue(rfinal_data + 14, LENGTH_TIMESTAMP),  // timeResponseToPoll // Response to poll sent time
        timeACTReceived, // Response to Poll Received
        timeFinalMessageSent, // Final Message send time
        DW1000NgUtils::bytesAsValue(rfinal_data + 18, LENGTH_TIMESTAMP)   // timeFinalMessageReceive // Final message receive time
    );
    #else
    returnValue.range = computeRangeAsymmetric(
        timePollSent, // Poll send time
        DW1000NgUtils::bytesAsValue(rfinal_data + 10, LENGTH_TIMESTAMP),  // timePollReceived
        DW1000NgUtils::bytesAsValue(rfinal_data + 14, LENGTH_TIMESTAMP),  // timeResponseToPoll // Response to poll sent time
        timeACTReceived, // Response to Poll Received
        timeFinalMessageSent, // Final Message send time
        DW1000NgUtils::bytesAsValue(rfinal_data + 18, LENGTH_TIMESTAMP)   // timeFinalMessageReceive // Final message receive time
    );
    #endif

    //uint64_t timeAfterCalcRange = micros();

    returnValue.range = DW1000NgRanging::correctRange(returnValue.range, DEFAULT_CONFIG.channel, DEFAULT_CONFIG.pulseFreq);

    //if (returnValue.range < -1 | returnValue.range > 150) {return {false, 5};}
    returnValue.success = true;

    //Serial.println((String)"range:" + returnValue.range);

    //returnValue = {true, range};
    return returnValue;
}



uint32_t timePollReceived;
uint32_t timeResponseToPoll;
bool ranging_fct() {
    // Receive POLL
    if(!receiveFrameACK()) {
        //Serial.println("Error 1");
        return false;
    }

    size_t init_len = DW1000Ng::getReceivedDataLength();
    byte recv_data[init_len];
    DW1000Ng::getReceivedData(recv_data, init_len);

    if (!(init_len > 9 and recv_data[9] == RANGING_TAG_POLL and recv_data[5] == personal_short_address)){
        if (debug) Serial.println("Error 2:" + String(recv_data[5]));
        return false;
    }

    // Send ACTIVITY_CONTROL (response to poll)
    DW1000NgRTLS::transmitResponseToPoll(&recv_data[7], &recv_data[3], &recv_data[5]);
    //DW1000NgRTLS::transmitResponseToPollACK(&recv_data[7], &recv_data[3], &recv_data[5]);

    
    timePollReceived = getReceiveTimestampShort();// L'accès au bus SPI est long, donc on le fait pdt le temps mort
    DW1000NgRTLS::waitForTransmission();
    
    timeResponseToPoll = getTransmitTimestampShort();
    // Receive final message
    if(!receiveFrameACK()) {
        if (debug) Serial.println("Error 3");
        return false;
    }

    //size_t rfinal_len = DW1000Ng::getReceivedDataLength();
    //byte ack_data[rfinal_len];
    //DW1000Ng::getReceivedData(ack_data, rfinal_len);

    if(!(DW1000Ng::getReceivedDataLength() > 1)){// && ack_data[0] & 0x07)) { // CHECK Long du message "vide"
        if (debug) Serial.println("Error 4");
        return false;
    }

    // Send modified ranging confirm
    DW1000NgRTLS::transmitRangingConfirmExtended(
        &recv_data[7],
        timePollReceived,
        timeResponseToPoll,
        getReceiveTimestampShort(), 
        &recv_data[3], 
        &recv_data[5]);
    DW1000NgRTLS::waitForTransmission();
    DW1000Ng::startReceive();
    if (debug) Serial.println("Fs");
    return true;
}


float average(float *array, int len) {
    long double sum = 0.0;
    for (int i = 0; i < len; i++)
        sum += array[i];
    return (1000*(float)sum) / len;
}


bool local_debug = false;
inline void calLoop(int a, int b) {
    if (local_debug) Serial.print("r0");
    if (Serial.available()) {
        input = Serial.parseInt();
        if (input == 100) {
            Serial.println(personal_short_address);
        }
        else if (input == a || input == b) {
            n = 0;
            next_ranging_id = input;
        }
        else {
            n = round_trip;
        }
    }
    if (local_debug) Serial.print("r1");
    if (n == 0) {
        uint32_t time_start = millis();
        while (n < round_trip && millis() - time_start < round_trip*4) {
            RangeAcceptResult result = start_ranging_fct(next_ranging_id);
            if (result.success) {
                list_dist[n] = result.range;
                n++;
            }
            else Serial.println("ranging failed:" + String(result.range));
        }
        if (n != round_trip) {
            Serial.println("timeout");
        }
        else {
            Serial.println("average:" + String(personal_short_address) + ":" + String(next_ranging_id) + ":" + String(average(list_dist, round_trip)));
        }
    }
    if (local_debug) Serial.print("r2");
    ranging_fct();
    if (local_debug) Serial.print("r3");
}

