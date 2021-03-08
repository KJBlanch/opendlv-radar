#ifndef RADAR_DECODER
#define RADAR_DECODER

#include "opendlv-standard-message-set.hpp"

#include <string>
#include <utility>

int decode (opendlv::proxy::RadarDetectionReading msg, std::unique_ptr<cluon::SharedMemory> &shmArgb, uint16_t addBk[2048][512*2], bool verbose, uint16_t origin, uint16_t c_height, uint16_t c_width);

#endif