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

#define CATCH_CONFIG_MAIN  // This tells Catch to provide a main() - only do this in one cpp file

#include "catch.hpp"

#include "cluon-complete.hpp"
#include "opendlv-standard-message-set.hpp"

#include "radar-decoder.hpp"

#include <iostream>
#include <string>
#include <vector>

#define radians(a) (((a)*M_PI)/180)

using namespace std::chrono_literals;

/////// Tests for handling payload ///////

TEST_CASE("Test decoder with empty payload.") {
    opendlv::proxy::RadarDetectionReading msg;

    uint16_t addBk[2048][512*2]; 
    bool verbose;
    uint16_t origin; 
    uint16_t c_height;
    uint16_t c_width;
    
    std::unique_ptr<cluon::SharedMemory> shmArgb{
      new cluon::SharedMemory{"Test_0", c_width * c_height * 4}};

    auto retVal = decode(msg, shmArgb, addBk,  verbose, origin, c_height, c_width);

    REQUIRE(retVal != -1);
}

TEST_CASE("Test decoder with sample payload.") {
    opendlv::proxy::RadarDetectionReading msg;
    
    //Payload
    msg.azimuth(1024);
    std::string payload = "255, 255, 255, 255, 255";
    msg.data(payload);
    msg.range(1500);
    
    uint16_t addBk[2048][512*2]; 

    bool verbose = true;
    uint16_t origin = 512; 
    uint16_t c_height = 1024;
    uint16_t c_width = 1024;

    std::unique_ptr<cluon::SharedMemory> shmArgb{
      new cluon::SharedMemory{"Test_1", c_width * c_height * 4}};

    for (int i = 0; i <= 4095; i++){
      for (int j = 0; j<= 512; j++) {
        uint16_t distance = j;
        float angle = (float(i)/4096*360);
        float angle_rad;
        uint16_t x, y;

        if (angle < 90) {
          angle_rad = radians(angle);
          x = round(origin+((sin(angle_rad)*distance)));
          y = round(origin-((cos(angle_rad)*distance)));
        }
        
        else if (angle < 180) {
          angle_rad = radians(angle-90);
          x = round(origin+((cos(angle_rad)*distance)));
          y = round(origin+((sin(angle_rad)*distance)));
        }
        

        else if (angle < 270) {
          angle_rad = radians(angle-180);
          x = round(origin-((sin(angle_rad)*distance)));
          y = round(origin+((cos(angle_rad)*distance)));
        }
        
        else if (angle < 360){
          angle_rad = radians(angle-270);
          x = round(origin-((cos(angle_rad)*distance)));
          y = round(origin-((sin(angle_rad)*distance)));
        }
        
        addBk[i/2][j*2] = x;
        addBk[i/2][(j*2)+1] = y;
      }
    }

    auto retVal = decode(msg, shmArgb, addBk, verbose, origin, c_height, c_width);

    REQUIRE(retVal == msg.azimuth());
}

TEST_CASE("Test decoder with faulty payload.") {
   
}

//I SWEAR TO FUCKING GOD, IF THIS ERROR IS CAUSED BY NAMING TWO TESTS WITH THE SAME TEST NAME, THEN I AM GONNA PUNT A SQUIRREL

TEST_CASE("Test decoder with sample payload and channel 0 for time stamp.") {
    
}




/////// Tests for handling memory ///////
TEST_CASE("Test decoder with faulty memory") {}

TEST_CASE("Test decoder with empty memory") {}

TEST_CASE("Test decoder with correct memory") {}



/////// Tests for handling local variables ///////
TEST_CASE("Test decoder with faulty variables") {}

TEST_CASE("Test decoder with empty variables") {}

TEST_CASE("Test decoder with correct variables") {}