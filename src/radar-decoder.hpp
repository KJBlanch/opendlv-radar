/*
 * Copyright (C) 2021 Krister Blanch
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef RADAR_DECODER
#define RADAR_DECODER

#include "opendlv-standard-message-set.hpp"

#include <string>
#include <utility>

int decode (opendlv::proxy::RadarDetectionReading msg, std::unique_ptr<cluon::SharedMemory> &shmArgb, uint16_t addBk[2048][512*2], bool verbose, uint16_t origin, uint16_t c_height, uint16_t c_width);

#endif