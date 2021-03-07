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



uint8_t decode (opendlv::proxy::RadarDetectionReading msg, std::unique_ptr<cluon::SharedMemory> &shmArgb, uint16_t addBk[2048][512*2], bool verbose, uint16_t origin, uint16_t c_height, uint16_t c_width) {

  ///Error handling

  //msg


  //shmArgb
  
  
  //addBk


  //verbose - Uhhh, it's a bool. Probably should look at how to check for errors with this one. Maybe null/void?

  //c_height
  if (c_height <= 0) {
    return (-1);
  }
  //c_width
  if (c_width <= 0) {
    return (-1);
  }

  //origin
  if (origin <= 0) {
    return (-1);
  } else if (origin > (2*c_width)) {
    return (-1);
  } else if (origin > (2*c_height)) {
    return (-1);
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
    return -2;
  }

  if (verbose) std::cout << "Packet: " << packet.size() << std::endl;
  int k = 1;
  //Process the packet. 
  for (int i = 0; i < packet.size(); i = i+k) {
    if (i > (packet.size()/3*2)) k = 2;
    current_strength = packet[i];
  
    uint16_t distance = i;

    //Retrieve x and y location values from the pixel map based on azimuth and distance.
    x = addBk[int(msg.azimuth())/2][distance*2];
    y = addBk[int(msg.azimuth())/2][(distance*2)+1];
      
    if (verbose) std::cout << "    Angle: " << msg.azimuth() << " " << angle << " ";
    if (verbose) std::cout << "Points: " << x << " " << y << " ";
    
    //Ensure x and y are valid
    if (x > (origin*2)) {
      return -3;
    }
    if (y > (origin*2)) {
      return -3;
    }

    if (verbose) std::cout << "Strength: " << std::to_string(current_strength) << " " << "Distance: " << distance << std::endl;
    
    //Use values from pixel map to get the image memory address for that pixel. 4 Bytes per pixel. X is *4, Y is *1024 (For the row) 
  
    uint32_t index = ((4*x)+(1024*y)*4);
    shmArgb->lock();

    //Write the new values. White strength pixel, so all strengths are the same value. Set R, G or B to 255 for PPI color. 
    //4th value is Alpha. Set as 0 for no transparency. 

    shmArgb->data()[index+2] = (current_strength);

    if (current_strength == uint8_t(255)) { 
      current_strength = uint8_t(0);
    } else if (current_strength == 0) {
      current_strength = 255;
    }

    shmArgb->data()[index+1] = (current_strength);              
    shmArgb->data()[index] = (current_strength);
    shmArgb->data()[index+3] = char(0);
    
    if (verbose) std::cout << "Index: " << index << ". Value " << shmArgb->data()[index] << std::endl;

    //Revalidate shared memory. 
    shmArgb->unlock();
    if (verbose) std::cout << shmArgb->valid() << std::endl;
    shmArgb->notifyAll();

  }
  //Spoke unpacked. Final validation
  if (verbose) std::cout << shmArgb->valid() << std::endl;
  return (msg.azimuth());
}