#include <DW1000Ng.hpp>
#include <DW1000NgUtils.hpp>
#include <DW1000NgTime.hpp>
#include <DW1000NgConstants.hpp>
#include <DW1000NgRanging.hpp>
#include <DW1000NgRTLS.hpp>

// Extended Unique Identifier register. 64-bit device identifier. Register file: 0x01
static const char EUI[] = "AA:BB:CC:DD:EE:FF:00:03";
static const uint16_t personal_short_address = 3;

//#define CALIBRATE_DELAYS true
#define antennaDelay 32903//32850


#include "../AntennaCalibrationCommon.hpp"


void setup() {
    calSetup();
}


void loop() {
    calLoop(1, 2);
}
