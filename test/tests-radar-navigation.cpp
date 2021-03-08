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

/////// Tests for verifying Test lib /////

TEST_CASE("Return test") {
  std::cout << "Validating Test Lib. Asserting 1 == 1" << std::endl;
  
  auto retVal = 1;
  REQUIRE (retVal == 1);
}


/////// Tests for handling payload ///////

TEST_CASE("Test decoder with empty payload.") {
    
    opendlv::proxy::RadarDetectionReading msg;
    msg.azimuth(5);

    uint16_t addBk[2048][512*2]; 
    bool verbose = true;
    uint16_t origin = 512; 
    uint16_t c_height = 1024;
    uint16_t c_width = 1024;
    
    std::unique_ptr<cluon::SharedMemory> shmArgb_0{
      new cluon::SharedMemory{"/Test_0.argb", c_width * c_height * 4}};

    auto retVal = decode(msg, shmArgb_0, addBk,  verbose, origin, c_height, c_width);
    if (verbose) std::cout << "Test Case 0. Expected: -1" << ". Outcome: " << retVal << std::endl;
    
    //Return value of -1 on empty message
  
    REQUIRE(retVal == -1);  
}

TEST_CASE("Test decoder with sample payload.") {
    
    opendlv::proxy::RadarDetectionReading msg;
    
    //Payload
    msg.azimuth(1024);
    
    std::vector<uint8_t> sample{
      0xe7, 0x9c, 0x95, 0x95, 0x08, 0x00, 0x7c, 0x0e,
      0x00, 0x06, 0x81, 0xfe, 0x45, 0x00, 0x00, 0xf4,
      0x00, 0x00, 0xaa, 0xff, 0xff, 0x04, 0xc2, 0x92
    };

/*
    const size_t bytesAvailable{sizeof(line->data)};
    uint8_t buffer[bytesAvailable];
    std::memcpy(buffer, line->data, bytesAvailable);
    std::string payload(reinterpret_cast<char*>(buffer), bytesAvailable);
*/
    const std::string payload(reinterpret_cast<char*>(sample.data()), sample.size());

    
    msg.data(payload);
    msg.range(1500);
    
    uint16_t addBk[2048][512*2]; 

    bool verbose = true;
    uint16_t origin = 512; 
    uint16_t c_height = 1024;
    uint16_t c_width = 1024;

    std::unique_ptr<cluon::SharedMemory> shmArgb_1{
      new cluon::SharedMemory{"/Test_1.argb", c_width * c_height * 4}};

    for (int i = 0; i <= 4095; i++){
      for (int j = 0; j<= 511; j++) {
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

    auto retVal = decode(msg, shmArgb_1, addBk, verbose, origin, c_height, c_width);
    if (verbose) std::cout << "Test Case 1. Expected: 1024" << ". Outcome: " << retVal << std::endl;
    REQUIRE(retVal == msg.azimuth());
     
}


TEST_CASE("Test decoder with non-openDLV message payload.") {
       
    opendlv::proxy::RadarDetectionReading msg;
    
    //Payload
    msg.azimuth(1024);
    
    const std::string payload {"Hello!"};

    //const std::string payload(reinterpret_cast<char*>(sample.data()), sample.size());

    
    msg.data(payload);
    msg.range(1500);
    
    uint16_t addBk[2048][512*2]; 

    bool verbose = true;
    uint16_t origin = 512; 
    uint16_t c_height = 1024;
    uint16_t c_width = 1024;

    std::unique_ptr<cluon::SharedMemory> shmArgb_2{
      new cluon::SharedMemory{"/Test_2.argb", c_width * c_height * 4}};

    for (int i = 0; i <= 4095; i++){
      for (int j = 0; j<= 511; j++) {
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

    auto retVal = decode(msg, shmArgb_2, addBk, verbose, origin, c_height, c_width);
    if (verbose) std::cout << "Test Case 2. Expected: 1024" << ". Outcome: " << retVal << std::endl;
    REQUIRE(retVal == msg.azimuth());
     
}


/*
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

*/