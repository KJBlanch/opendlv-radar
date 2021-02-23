/*
 * Copyright (C) 2021 Kris Blanch
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
#include "cluon-complete.hpp"
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include "opendlv-standard-message-set.hpp"
#include <iostream>
#include <fstream>

#include <X11/Xlib.h>



#define radians(a) (((a)*M_PI)/180)



///////////////////////////////////////////////////////////


int32_t main(int32_t argc, char **argv) {

  int32_t retCode{1};
  
  std::cerr << "WARNING!" << std::endl;
  std::cerr << std::endl;
  std::cerr << "Radar transmitters are dangerous! Ensure that a safe working environment is maintained. Check unit manual before use!" << std::endl;
  std::cerr << "Radarsändare är farliga! Se till att en säker arbetsmiljö upprätthålls. Kontrollera bruksanvisningen före användning!" << std::endl;
  std::cerr << std::endl;

  auto commandlineArguments = cluon::getCommandlineArguments(argc, argv);
  if (   (0 == commandlineArguments.count("cid"))
      ) {

    std::cerr << argv[0] << " Provides ppi and navigation for Navico Radar Units."<< std::endl;
    std::cerr << "Requires a cluon id to capture from. Typical usage with other openDLV services is <-cid=111> "<< std::endl;
    
  } else {


    //Set variables from CLI inputs
    uint32_t const id{(commandlineArguments["id"].size() != 0) 
      ? static_cast<uint32_t>(std::stoi(commandlineArguments["id"])) : 0};
    bool const verbose{commandlineArguments.count("verbose") != 0};
    bool const timings{commandlineArguments.count("timings") != 0};
  
    std::string const name{(commandlineArguments["name"].size() != 0) 
      ? commandlineArguments["name"] : "/polar0"};

    //Set Model

    //Used to build the image if on first frame capture
    bool initial = true;

    //Used to update the image upon 360 degrees worth of frame capture
    bool model_update = false;
    
    
    //For navigation display used with Optic Flow
    uint8_t track_len = 5;
    uint8_t detect_interval = 2;  
    uint8_t frame_idx = 0;


    //Fixed values for use with a Navico Radar (Spoke Length is 512 values)
    uint16_t origin, c_width, c_height;
    origin = 512;
    c_width = origin*2;
    c_height = origin*2;
    float current_angle;

    if (verbose) std::cout << c_width << " "  << c_height << std::endl;
    

    //Set image name
    std::string const nameArgb{name + ".argb"};


    //Address for image
    std::unique_ptr<cluon::SharedMemory> shmArgb{
      new cluon::SharedMemory{nameArgb, c_width * c_height * 4}};

    //Address for lookup_cache
    uint16_t addBk [2048][512*2];

    //Build Timing Variables
    cluon::data::TimeStamp pT1, pT2;

    //Build pixelmap

    //The radar spoke data comprises of an azimuth, an index (distance) and a strength. Instead of cranking out some square root functions each time
    //spoke data is received, we can create a lookup table instead. The 2d array is 2048*(512*2). 2048 spokes per circle, with 512 values per spoke.
    //Each value needs both an x and y, hence the *2. 

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

    if (verbose) std::cout << "Address Book Init with size: " << sizeof(addBk) << std::endl; //This could be a test. 
    if (verbose) std::cout << shmArgb->size() << std::endl; //Also this

    //Set X11 Paramaters
    Display* display{nullptr};
    Visual* visual{nullptr};
    Window window{0};
    XImage* ximage{nullptr};

    //If the shared memory is valid, build the image out of the shared memory values
    if (shmArgb->valid()) {
      display = XOpenDisplay(NULL);
      visual = DefaultVisual(display, 0);
      window = XCreateSimpleWindow(display, RootWindow(display, 0), 0, 0, c_width, c_height, 1, 0, 0);
      shmArgb->lock();
      {
        ximage = XCreateImage(display, visual, 24, ZPixmap, 0, shmArgb->data(), c_width, c_height, 32, c_width*4);
      }
      shmArgb->unlock();

    } else {
      if (verbose) {
        std::cerr << "Invalid Memory Allocation" << std::endl;
      }
    }

    

    if (verbose) std::cout << "Model and paramaters built. Begining listener" << std::endl;

    //Start Lambda function that fires on recieving a RadarDetectionReading envelope on the cluon id. 
    cluon::OD4Session od4{static_cast<uint16_t>(
        std::stoi(commandlineArguments["cid"])), [&pT1, &pT2, timings, &addBk, &shmArgb, c_width, c_height, &display, &visual, &window, &ximage, &current_angle, origin, &model_update, &initial, &frame_idx, verbose](cluon::data::Envelope &&env){
            cluon::data::TimeStamp cT_now = cluon::time::now();
            //Now, we unpack the cluon::data::Envelope to get our message.
            opendlv::proxy::RadarDetectionReading msg = cluon::extractMessage<opendlv::proxy::RadarDetectionReading>(std::move(env));
            
            //If invalid strength, ensure that a valid number is written to the memory.
            uint8_t current_strength = 0;

            //Retrieve current angle, and set empty values for pixels and radians. 
            float angle = msg.azimuth()/4096*360;
            float angle_rad;
            uint16_t x, y;
            
            //Extract the packet
            std::string packet = msg.data();
            if (verbose) std::cout << "Packet: " << packet.size() << std::endl;
           
            //Process the packet. 
            for (int i = 0; i < packet.size(); i = i+2) {
              current_strength = std::stoi(std::to_string(packet[i]));
              uint16_t distance = i;

              //Retrieve x and y location values from the pixel map based on azimuth and distance.
              x = std::stoi(std::to_string(addBk[int(msg.azimuth())/2][distance*2]));
              y = std::stoi(std::to_string(addBk[int(msg.azimuth())/2][(distance*2)+1]));
                
              if (verbose) std::cout << "    Angle: " << msg.azimuth() << " " << angle << " ";
              if (verbose) std::cout << "Points: " << x << " " << y << " ";
              
              //Ensure x and y are valid
              if (0 > x || x > (origin*2)) {
                return 0;
              }
              if (0 > y || y > (origin*2)) {
                return 0;
              }

      

              if (verbose) std::cout << "Strength: " << current_strength << " " << "Distance: " << distance << std::endl;
              
              //Current strength is writing from -128 to 127. Add 128 to get correct 255 RGB Value. 
              

              //Use values from pixel map to get the image memory address for that pixel. 4 Bytes per pixel. X is *4, Y is *1024 (For the row) 
              uint32_t index = ((4*x)+(1024*y)*4);
              shmArgb->lock();

              //Write the new values. White strength pixel, so all strengths are the same value. Set R, G or B to 255 for PPI color. 
              //4th value is Alpha. Set as 0 for no transparency. 
              shmArgb->data()[index] = (current_strength);
              shmArgb->data()[index+1] = (current_strength);
              shmArgb->data()[index+2] = (current_strength);
              shmArgb->data()[index+3] = char(0);
              
              if (verbose) std::cout << "Index: " << index << ". Value " << std::to_string(shmArgb->data()[index]) << std::endl;

              //Revalidate shared memory. 
              shmArgb->unlock();
              if (verbose) std::cout << shmArgb->valid() << std::endl;
              shmArgb->notifyAll();

            }

            //Spoke unpacked. Final validation
            if (verbose) std::cout << shmArgb->valid() << std::endl;
            
            //If the azimuth has completed a circle
            //Use 2 instead of 0 as dropped packets hold a 0 val for azimuth and will trigger this. 
            if (msg.azimuth() == 2 || remainder(msg.azimuth(), 256) == 0) {
              
              //Build PPI
              if (verbose) std::cout << "Updating window" << std::endl;
              
              XMapWindow(display, window);

              shmArgb->lock();

              XPutImage(display, window, DefaultGC(display, 0), ximage, 0, 0, 0, 0,
              c_width, c_height);
      
              shmArgb->unlock();
              shmArgb->notifyAll();

            }

            //For Timing Diagnostics
            if (msg.azimuth() == 2 && timings) {
              //std::cout << "Timings" << std::endl;
              cluon::data::TimeStamp cT1, cT2;
              

              cT1 = cT_now;//current message time msg.
              cT2 = cluon::time::now();//current Systime

              float T1 = cluon::time::deltaInMicroseconds(cT1, pT1);
              float T2 = cluon::time::deltaInMicroseconds(cT2, pT2);
              std::cout << float((T2-T1)/1000) << std::endl;

              pT1 = cT1;
              pT2 = cT2;

            }
            
        }
    
    };

    //Set listening to true and open thread to wait for incoming messages. 
    //od4.dataTrigger(MyMessage1::ID(), onMyMessage1); //This is in the tutorial. Seems to work without it?
    
    bool listening = true;
    
    using namespace std::chrono_literals;
    while (listening) {
      std::this_thread::sleep_for(1s);
    }
    return retCode;
  }
};