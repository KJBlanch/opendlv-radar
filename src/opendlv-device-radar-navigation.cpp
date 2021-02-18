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
    
  } else {

    uint32_t const id{(commandlineArguments["id"].size() != 0) 
      ? static_cast<uint32_t>(std::stoi(commandlineArguments["id"])) : 0};
    bool const verbose{commandlineArguments.count("verbose") != 0};
    bool const gui{commandlineArguments.count("gui") != 0};
    uint32_t const simulated_angle_rot{(commandlineArguments["testing_sar"].size() != 0) 
      ? static_cast<uint32_t>(std::stoi(commandlineArguments["testing_sar"])) : 0};
  
    std::string const name{(commandlineArguments["name"].size() != 0) 
      ? commandlineArguments["name"] : "/polar0"};

    std::string const book_name{(commandlineArguments["book_name"].size() != 0) 
      ? commandlineArguments["book_name"] : "/book0"};
    //Set Model

    bool initial = true;
    bool model_update = false;
    
    uint8_t track_len = 5;
    uint8_t detect_interval = 2;
    
    uint8_t frame_idx = 0;
  
    int origin, c_width, c_height;
    origin = 512;
    c_width = origin*2;
    c_height = origin*2;

    std::cout << c_width << " "  << c_height << std::endl;
    

    //Set picture

    std::string const nameArgb{name + ".argb"};


    //Address for image
    
    std::unique_ptr<cluon::SharedMemory> shmArgb{
      new cluon::SharedMemory{nameArgb, c_width * c_height * 4}};

    //Address for lookup_cache

    uint16_t addBk [2048][512*2];

    for (int i = 0; i <= 4095; i++){
      for (int j = 0; j<= 512; j++) {
        int distance = j;
        float angle = (float(i)/4096*360);
        float angle_rad;
        int x, y;

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

    if (verbose) std::cout << "Address Book Init with size: " << sizeof(addBk) << std::endl;
    if (verbose) std::cout << shmArgb->size() << std::endl;

    Display* display{nullptr};
    Visual* visual{nullptr};
    Window window{0};
    XImage* ximage{nullptr};

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

    
    float current_angle;
    if (verbose) std::cout << "Model and paramaters built. Begining listener" << std::endl;

 
    cluon::OD4Session od4{static_cast<uint16_t>(
        std::stoi(commandlineArguments["cid"])), [&addBk, &shmArgb, c_width, c_height, &display, &visual, &window, &ximage, &current_angle, origin, &model_update, &initial, &frame_idx, verbose](cluon::data::Envelope &&env){
            // Now, we unpack the cluon::data::Envelope to get our message.
            opendlv::proxy::RadarDetectionReading msg = cluon::extractMessage<opendlv::proxy::RadarDetectionReading>(std::move(env));

            int current_strength = 0;
            float angle = msg.azimuth()/4096*360;
            float angle_rad;
            int x, y;
            
            std::string packet = msg.data();
            if (verbose) std::cout << "Packet: " << packet.size() << std::endl;
           
          
            for (int i = 0; i < packet.size(); i = i+2) {
              current_strength = std::stoi(std::to_string(packet[i]));
              int distance = i;

              
              x = std::stoi(std::to_string(addBk[int(msg.azimuth())/2][distance*2]));
              y = std::stoi(std::to_string(addBk[int(msg.azimuth())/2][(distance*2)+1]));
                
              if (verbose) std::cout << "Angle: " << msg.azimuth() << " " << angle << " ";
              if (verbose) std::cout << "Points: " << x << " " << y << " ";
              
              if (0 > x || x > (origin*2)) {
                return 0;
              }
              if (0 > y || y > (origin*2)) {
                return 0;
              }

              if (verbose) std::cout << "Strength: " << current_strength << " " << "Distance: " << distance << std::endl;
              
              //construct-update-deconstruct

              int index = ((4*x)+(1024*y)*4);
              shmArgb->lock();

              shmArgb->data()[index] = (current_strength);
              shmArgb->data()[index+1] = (current_strength);
              shmArgb->data()[index+2] = (current_strength);
              shmArgb->data()[index+3] = char(0);
              
              if (verbose) std::cout << "Index: " << index << ". Value " << std::to_string(shmArgb->data()[index]) << std::endl;

              
             
              shmArgb->unlock();
              if (verbose) std::cout << shmArgb->valid() << std::endl;

              shmArgb->notifyAll();

            }

            
            if (verbose) std::cout << shmArgb->valid() << std::endl;

            if (msg.azimuth() == 2 ) {

              //Build PPI
              if (verbose) std::cout << "Updating window" << std::endl;
              
              XMapWindow(display, window);

              shmArgb->lock();

              XPutImage(display, window, DefaultGC(display, 0), ximage, 0, 0, 0, 0,
              c_width, c_height);
      
              shmArgb->unlock();
              shmArgb->notifyAll();
              
              model_update = false;
              return(0);
            }

            if (remainder(msg.azimuth()-1, 512) == 0 && model_update == false) {
              

              //Reset
              model_update = true;
            }
            
        }
    
    };

    //od4.dataTrigger(opendlv::proxy::RadarDetectionReading::ID(), onMyMessage1);  
    bool listening = true;
    
    using namespace std::chrono_literals;
    while (listening) {
      std::this_thread::sleep_for(1s);
    }
    return retCode;
  }
};