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

TEST_CASE("Test the tester - Validate the Test Oracle") {
//Expected outcome is 1
  std::cout << std::endl;
  std::cout << "Validating the testing framework and validating the test oracle. Asserting 1 == 1" << std::endl;
  std::cout << std::endl;
  std::cout << std::endl;
  
  std::cout << "The following test library uses the 'decode' function. Error handling is contained within the function." << std::endl; 
  std::cout << "In the event the function performs as desired, it will return the message envelopes azimuth value." << std::endl; 
  std::cout << "This is a positve integer in the range 0:4096." << std::endl;
  std::cout << "Specific error responses are thereby negative integers." << std::endl; 
  std::cout << "Test Cases are named after the specific event." << std::endl; 
  std::cout << "Expected outcome based upon error handling is commented underneath." << std::endl;
  std::cout << std::endl;

  auto retVal = 1;
  REQUIRE (retVal == 1);
}


/////// Tests for handling payload ///////

TEST_CASE("Test 0 - Test decoder with empty payload.") {
  std::cout << "Test Case 0 (Nominal Case). Decoder test with an empty payload" << std::endl; 
//Expected outcome is -1

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
  
  std::cout << "Test Case 0. Expected: -1" << ". Outcome: " << retVal << std::endl;
  std::cout << std::endl;
  //Return value of -1 on empty message

  REQUIRE(retVal == -1);  
}

TEST_CASE("Test 1 - Test decoder with sample payload.") {
  std::cout << "Test Case 1 (Nominal Case). Basic payload test with correct parameters." << std::endl;;
//Expected outcome is the msg.azimuth() value. Set as 1024    
    
  opendlv::proxy::RadarDetectionReading msg;
  
  //Payload
  msg.azimuth(1024);
  
  std::vector<uint8_t> sample{
    0xe7, 0x9c, 0x95, 0x95, 0x08, 0x00, 0x7c, 0x0e,
    0x00, 0x06, 0x81, 0xfe, 0x45, 0x00, 0x00, 0xf4,
    0x00, 0x00, 0xaa, 0xff, 0xff, 0x04, 0xc2, 0x92
  };

  const std::string payload(reinterpret_cast<char*>(sample.data()), sample.size());

  
  msg.data(payload);
  msg.range(1500);
  
  uint16_t addBk[2048][512*2]; 

  bool verbose = false;
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
  std::cout << "Test Case 1. Expected: 1024" << ". Outcome: " << retVal << std::endl;
  std::cout << std::endl;
  REQUIRE(retVal == msg.azimuth());
    
}


TEST_CASE("Test 2 - Test decoder with non-openDLV message payload.") {
//Expected outcome is msg.azimuth(). Set as 1024.       
  std::cout << "Test Case 2 (Nominal Case). Payload holds 'Hello!' which does not conform to openDLV message template but is translatable." << std::endl;
  opendlv::proxy::RadarDetectionReading msg;
  
  //Payload
  msg.azimuth(1024);
  
  const std::string payload {"Hello!"};
  
  msg.data(payload);
  msg.range(1500);
  
  uint16_t addBk[2048][512*2]; 

  bool verbose = false;
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
  std::cout << "Test Case 2. Expected: 1024" << ". Outcome: " << retVal << std::endl;
  std::cout << std::endl;
  REQUIRE(retVal == msg.azimuth());
     
}



/////// Tests for handling memory ///////

TEST_CASE("Test 3 - decoder with faulty memory.") {
std::cout << "Test Case 3 (Exceptional Case). Memory is initialised with incorrect size." << std::endl;
//Expected outcome is -11  
  
  opendlv::proxy::RadarDetectionReading msg;
  msg.azimuth(5);
  
  std::vector<uint8_t> sample{
    0xe7, 0x9c, 0x95, 0x95, 0x08, 0x00, 0x7c, 0x0e,
    0x00, 0x06, 0x81, 0xfe, 0x45, 0x00, 0x00, 0xf4,
    0x00, 0x00, 0xaa, 0xff, 0xff, 0x04, 0xc2, 0x92
  };

  const std::string payload(reinterpret_cast<char*>(sample.data()), sample.size());

  
  msg.data(payload);
  msg.range(1500);

  uint16_t addBk[2048][512*2]; 
  bool verbose = true;
  uint16_t origin = 512; 
  uint16_t c_height = 1024;
  uint16_t c_width = 1024;
  
  std::unique_ptr<cluon::SharedMemory> shmArgb_0{
    new cluon::SharedMemory{"/Test_3.argb", 1}};

  auto retVal = decode(msg, shmArgb_0, addBk,  verbose, origin, c_height, c_width);
  std::cout << "Test Case 3. Expected: -11" << ". Outcome: " << retVal << std::endl;
  std::cout << std::endl;
  //Return value of -11 on faulty memory

  REQUIRE(retVal == -11); 

}

TEST_CASE("Test 4 - decoder with empty memory.") {
  std::cout << "Test Case 4 (Exceptional Case). Memory will attempt to build with a size of 0." << std::endl; 
  //Expected outcome is -2 

  opendlv::proxy::RadarDetectionReading msg;
  msg.azimuth(5);
  
  std::vector<uint8_t> sample{
    0xe7, 0x9c, 0x95, 0x95, 0x08, 0x00, 0x7c, 0x0e,
    0x00, 0x06, 0x81, 0xfe, 0x45, 0x00, 0x00, 0xf4,
    0x00, 0x00, 0xaa, 0xff, 0xff, 0x04, 0xc2, 0x92
  };

  const std::string payload(reinterpret_cast<char*>(sample.data()), sample.size());

  msg.data(payload);
  msg.range(1500);

  uint16_t addBk[2048][512*2]; 
  bool verbose = true;
  uint16_t origin = 512; 
  uint16_t c_height = 1024;
  uint16_t c_width = 1024;
  
  std::unique_ptr<cluon::SharedMemory> shmArgb_0{
    new cluon::SharedMemory{"/Test_4.argb"}};

  auto retVal = decode(msg, shmArgb_0, addBk,  verbose, origin, c_height, c_width);
  std::cout << "Test Case 4. Expected: -2" << ". Outcome: " << retVal << std::endl;
  std::cout << std::endl;
  //Return value of -2 on empty memory

  REQUIRE(retVal == -2); 

}

TEST_CASE("Test 5 - decoder with correct memory.") {
  std::cout << "Test Case 5 (Nominal Case). Memory will attempt to build correctly." << std::endl; 
  //Expected outcome is msg.azimuth(). Set as 5 

  opendlv::proxy::RadarDetectionReading msg;
  msg.azimuth(5);
  
  std::vector<uint8_t> sample{
    0xe7, 0x9c, 0x95, 0x95, 0x08, 0x00, 0x7c, 0x0e,
    0x00, 0x06, 0x81, 0xfe, 0x45, 0x00, 0x00, 0xf4,
    0x00, 0x00, 0xaa, 0xff, 0xff, 0x04, 0xc2, 0x92
  };

  const std::string payload(reinterpret_cast<char*>(sample.data()), sample.size());

  msg.data(payload);
  msg.range(1500);

  uint16_t addBk[2048][512*2]; 
  bool verbose = false;
  uint16_t origin = 512; 
  uint16_t c_height = 1024;
  uint16_t c_width = 1024;
  
  std::unique_ptr<cluon::SharedMemory> shmArgb_0{
    new cluon::SharedMemory{"/Test_5.argb", c_height * c_width * 4}};

  auto retVal = decode(msg, shmArgb_0, addBk,  verbose, origin, c_height, c_width);
  std::cout << "Test Case 5. Expected: 5" << ". Outcome: " << retVal << std::endl;
  std::cout << std::endl;
  //Return value of 5 on faulty memory

  REQUIRE(retVal == msg.azimuth()); 

}



/////// Tests for handling local variables ///////
TEST_CASE("Test 6 - decoder with faulty variables.") {
  std::cout << "Test Case 6 (Exceptional Case). Injection of faulty variables." << std::endl; 
  //Expected outcome is -6.

  opendlv::proxy::RadarDetectionReading msg;
  msg.azimuth(5);
  
  std::vector<uint8_t> sample{
    0xe7, 0x9c, 0x95, 0x95, 0x08, 0x00, 0x7c, 0x0e,
    0x00, 0x06, 0x81, 0xfe, 0x45, 0x00, 0x00, 0xf4,
    0x00, 0x00, 0xaa, 0xff, 0xff, 0x04, 0xc2, 0x92
  };

  const std::string payload(reinterpret_cast<char*>(sample.data()), sample.size());

  msg.data(payload);
  msg.range(1500);

  uint16_t addBk[2048][512*2]; 
  bool verbose = true;

  //Origin bigger than height or width of the image. 
  uint16_t origin = 2000; 
  uint16_t c_height = 1024;
  uint16_t c_width = 1024;
  
  std::unique_ptr<cluon::SharedMemory> shmArgb_0{
    new cluon::SharedMemory{"/Test_6.argb", c_height * c_width * 4}};

  auto retVal = decode(msg, shmArgb_0, addBk,  verbose, origin, c_height, c_width);
  std::cout << "Test Case 6. Expected: -6" << ". Outcome: " << retVal << std::endl;
  std::cout << std::endl;

  REQUIRE(retVal == -6); 

}

TEST_CASE("Test 7 - decoder with empty variables.") {
std::cout << "Test Case 7 (Exceptional Case). Injection of empty variables." << std::endl; 
//Expected outcome is -3

  opendlv::proxy::RadarDetectionReading msg;
  msg.azimuth(5);
  
  std::vector<uint8_t> sample{
    0xe7, 0x9c, 0x95, 0x95, 0x08, 0x00, 0x7c, 0x0e,
    0x00, 0x06, 0x81, 0xfe, 0x45, 0x00, 0x00, 0xf4,
    0x00, 0x00, 0xaa, 0xff, 0xff, 0x04, 0xc2, 0x92
  };

  const std::string payload(reinterpret_cast<char*>(sample.data()), sample.size());

  msg.data(payload);
  msg.range(1500);

  uint16_t addBk[2048][512*2]; 
  bool verbose = true;

  //origin, height and width are empty;
  uint16_t origin; 
  uint16_t c_height;
  uint16_t c_width;
  
  std::unique_ptr<cluon::SharedMemory> shmArgb_0{
    new cluon::SharedMemory{"/Test_7.argb", 16}};

  auto retVal = decode(msg, shmArgb_0, addBk,  verbose, origin, c_height, c_width);
  std::cout << "Test Case 7. Expected: -3" << ". Outcome: " << retVal << std::endl;
  std::cout << std::endl;
  
  REQUIRE(retVal == -3); 
  
}

TEST_CASE("Test 8 - decoder with correct variables") {
  std::cout << "Test Case 8 (Nominal Case). Injection of correct variables." << std::endl; 
  //Expected outcome is msg.azimuth(). Set as 42.

  opendlv::proxy::RadarDetectionReading msg;
  msg.azimuth(42);
  
  std::vector<uint8_t> sample{
    0xe7, 0x9c, 0x95, 0x95, 0x08, 0x00, 0x7c, 0x0e,
    0x00, 0x06, 0x81, 0xfe, 0x45, 0x00, 0x00, 0xf4,
    0x00, 0x00, 0xaa, 0xff, 0xff, 0x04, 0xc2, 0x92
  };

  const std::string payload(reinterpret_cast<char*>(sample.data()), sample.size());

  msg.data(payload);
  msg.range(1500);

  uint16_t addBk[2048][512*2]; 
  bool verbose = true;

  uint16_t origin = 512; 
  uint16_t c_height = 1024;
  uint16_t c_width = 1024;
  
  std::unique_ptr<cluon::SharedMemory> shmArgb_0{
    new cluon::SharedMemory{"/Test_8.argb", c_height * c_width * 4}};

  auto retVal = decode(msg, shmArgb_0, addBk,  verbose, origin, c_height, c_width);
  std::cout << "Test Case 8. Expected: 42" << ". Outcome: " << retVal << std::endl;
  std::cout << std::endl;

  REQUIRE(retVal == 42); 

}

TEST_CASE("Test 9 - decoder injected with corrupted azimuth.") {
  std::cout << "Test Case 9 (Nominal Case). Injection of corrupted azimuth." << std::endl; 
  //Expected outcome is -13
  opendlv::proxy::RadarDetectionReading msg;
  msg.azimuth(5500);
  msg.data(std::string{"Hello!"});
  
  uint16_t addBk[2048][512*2]; 
  bool verbose = true;

  uint16_t origin = 512; 
  uint16_t c_height = 1024;
  uint16_t c_width = 1024;

  std::unique_ptr<cluon::SharedMemory> shmArgb_0{
    new cluon::SharedMemory{"/Test_9.argb", c_height * c_width * 4}};
    
  auto retVal = decode(msg, shmArgb_0, addBk,  verbose, origin, c_height, c_width);
  std::cout << "Test Case 9. Expected: -13" << ". Outcome: " << retVal << std::endl;
  std::cout << std::endl;
 
  REQUIRE (retVal == -13);

}

TEST_CASE("Test 10 - decoder injected with empty azimuth.") {
  std::cout << "Test Case 10 (Nominal Case). Injection of empty azimuth." << std::endl;
  //Expected outcome is -12
  opendlv::proxy::RadarDetectionReading msg;
  msg.azimuth();
  msg.data(std::string{"Hello!"});
  
  uint16_t addBk[2048][512*2]; 
  bool verbose = true;

  uint16_t origin = 512; 
  uint16_t c_height = 1024;
  uint16_t c_width = 1024;

  std::unique_ptr<cluon::SharedMemory> shmArgb_0{
    new cluon::SharedMemory{"/Test_10.argb", c_height * c_width * 4}};
    
  auto retVal = decode(msg, shmArgb_0, addBk,  verbose, origin, c_height, c_width);
  std::cout << "Test Case 10. Expected: -12" << ". Outcome: " << retVal << std::endl;
  std::cout << std::endl;
 
  REQUIRE (retVal == -12);

}