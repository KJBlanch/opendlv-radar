/*
 * Copyright (C) 2021  Krister Blanch
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

#include <iostream>
#include <thread>
#include <chrono>
#include <math.h>
#include <cmath>

#include <iostream>
#include <fstream>

#include "cluon-complete.hpp"
#include "opendlv-standard-message-set.hpp"
#include "radar-decoder.hpp"


#define radians(a) (((a)*M_PI)/180)

using namespace std::chrono_literals;



int decode (opendlv::proxy::RadarDetectionReading msg, std::unique_ptr<cluon::SharedMemory> &shmArgb, uint16_t addBk[2048][512*2], bool verbose, uint16_t origin, uint16_t c_height, uint16_t c_width) {

  ///Error handling

  //msg
  if (msg.data().size() == 0) {
      //empty packet
      if (verbose) std::cout << "Error: Empty Packet" << std::endl; 
      return(-1);
  }


  //shmArgb
  if (!shmArgb->valid() ) {
      //invalid memory
      if (verbose) std::cout << "Error: Invalid Memory" << std::endl; 
      return(-2);
  }
  
  //c_height
  if (c_height == NULL) {
    if (verbose) std::cout << "Error: Canvas height is empty" << std::endl; 
    return (-3);
  }

  //c_width
  if (c_width == NULL) {
    if (verbose) std::cout << "Error: Canvas width is empty" << std::endl;
    return (-4);
  }

  //origin
  if (origin == NULL) {
    if (verbose) std::cout << "Error: Canvas origin point is empty" << std::endl;
    return (-5);
  } else if (origin > c_width) {
    if (verbose) std::cout << "Error: Canvas origin exceeds width" << std::endl;
    return (-6);
  } else if (origin > c_height) {
    if (verbose) std::cout << "Error: Canvas origin exceeds height" << std::endl;
    return (-7);
  }

  if (msg.azimuth() == NULL) {
    if (verbose) std::cout << "Error: Azimuth point is empty" << std::endl;
    return (-12);
  } else if (msg.azimuth() > 4096) {
    if (verbose) std::cout << "Error: Azimuth point is corrupted" << std::endl;
    return (-13); 
  }
  
  uint8_t current_strength = 0;

  //Retrieve current angle, and set empty values for pixels and radians. 


  double angle = msg.azimuth()/4096*360;
  double angle_rad;
  uint16_t x, y;
  
  //Extract the packet
  std::string packet = msg.data();
  
  //If packet is empty, break lambda. 
  if (packet.size() == 0) {
    return (-8);
  } 

  if (verbose) std::cout << "Packet size: " << packet.size() << std::endl;
  int k = 1;
  //Process the packet. 
  for (int i = 0; i < packet.size(); i = i+k) {
    if (i > (packet.size()/3*2)) k = 2;
    current_strength = packet[i];
  
    uint16_t distance = i;

    //Retrieve x and y location values from the pixel map based on azimuth and distance.
    x = addBk[int(msg.azimuth())/2][distance*2];
    y = addBk[int(msg.azimuth())/2][(distance*2)+1];
      
    if (verbose) std::cout << "Angle: " << msg.azimuth() << " " << angle << " ";
    if (verbose) std::cout << "Points: " << x << " " << y << " ";
    
    //Ensure x and y are valid
    if (x > (c_width)) {
      if (verbose) std::cout << "Error: X value exceeds canvas width" << std::endl;
      return (-9);
    }
    if (y > (c_height)) {
      if (verbose) std::cout << "Error: Y value exceeds canvas height" << std::endl;
      return (-10);
    }

    if (verbose) std::cout << "Strength: " << std::to_string(current_strength) << " " << "Distance: " << distance << std::endl;
    
    //Use values from pixel map to get the image memory address for that pixel. 4 Bytes per pixel. X is *4, Y is *1024 (For the row) 
  
    uint32_t index = ((4*x)+(1024*y)*4);
    shmArgb->lock();

    //Write the new values. White strength pixel, so all strengths are the same value. Set R, G or B to 255 for PPI color. 
    //4th value is Alpha. Set as 0 for no transparency. 

    if (index > shmArgb->size()) {
        if (verbose) std::cout << "Critical Error: Lookup index exceeds memory bounds. This will lead to a SEGFAULT" << std::endl;
        return(-11);
    }

    shmArgb->data()[index+2] = (current_strength);

    if (current_strength == uint8_t(255)) { 
      current_strength = uint8_t(0);
    } else if (current_strength == 0) {
      current_strength = 255;
    }

    shmArgb->data()[index+1] = (current_strength);              
    shmArgb->data()[index] = (current_strength);
    shmArgb->data()[index+3] = char(0);
    
    if (verbose) std::cout << "Index: " << index << ". Value " << std::to_string(shmArgb->data()[index]) << std::endl;

    //Revalidate shared memory. 
    shmArgb->unlock();
    if (verbose) std::cout << shmArgb->valid() << std::endl;
    shmArgb->notifyAll();

  }
  //Spoke unpacked. Final validation
  if (verbose) std::cout << "Packet Validated: " << shmArgb->valid() << std::endl;
  return (msg.azimuth());
}