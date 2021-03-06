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
#include <X11/Xlib.h>



#define radians(a) (((a)*M_PI)/180)

using namespace std::chrono_literals;


//function draw
  


//function optic

//function timing


///////////////////////////////////////////////////////////


int32_t main(int32_t argc, char **argv) {

  int32_t retCode{0};
  
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
    bool const demo{commandlineArguments.count("demo") != 0};
  
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

    //Set image name
    std::string const nameArgb{name + ".argb"};


    //Address for image
    std::unique_ptr<cluon::SharedMemory> shmArgb{
      new cluon::SharedMemory{nameArgb, c_width * c_height * 4}};

    //Address for prior image
    //std::unique_ptr<cluon::SharedMemory> priorArgb{
      //new cluon::SharedMemory{nameArgb, c_width * c_height * 4}};

    //Address for lookup_cache
    uint16_t addBk [2048][512*2];

    //Build Timing Variables
    cluon::data::TimeStamp pT1, pT2;

    //Build pixelmap

    //The radar spoke data comprises of an azimuth, an index (distance) and a strength. Instead of cranking out some square root functions each time
    //spoke data is received, we can create a lookup table instead. The 2d array is 2048*(512*2). 2048 spokes per circle, with 512 values per spoke.
    //Each value needs both an x and y, hence the *2. 

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

    if (verbose) std::cout << "Address Book Init with size: " << sizeof(addBk) << ". Should match size of alloc mem: " << shmArgb->size() << std::endl; //This could be a test. 
    //Set X11 Paramaters
    Display* display{nullptr};
    Visual* visual{nullptr};
    Window window{0};
    XImage* ximage{nullptr};
    if (verbose) std::cout << "X11 Params set" << std::endl;

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
      if (verbose) {
        std::cerr << "Memory Allocated and X11 Image Initialised" << std::endl;
      }

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
            uint16_t test_c = 0;


            //Now, we unpack the cluon::data::Envelope to get our message.
            opendlv::proxy::RadarDetectionReading msg = cluon::extractMessage<opendlv::proxy::RadarDetectionReading>(std::move(env));
            uint16_t current_azimuth = decode(msg, shmArgb, addBk, verbose, origin, c_height, c_width);
            

            //If the azimuth has completed a circle
            //Use 2 instead of 0 as dropped packets hold a 0 val for azimuth and will trigger this. 
            if (remainder(current_azimuth, 170) == float(0)) {
              
              //Build PPI
              if (verbose) std::cout << "Updating window: " << msg.azimuth() << std::endl;
              
              XMapWindow(display, window);

              shmArgb->lock();

              XPutImage(display, window, DefaultGC(display, 0), ximage, 0, 0, 0, 0,
              c_width, c_height);
      
              shmArgb->unlock();
              shmArgb->notifyAll();

            }


            /*
            //For optic flow
            //Check initial rotation is complete
            if (msg.azimuth() == 2 && init) {
              std::vector<uint16_t[4]> pixel_list;
              //Retrieve current index list. 

              //Define pixel block. 
                //Xi neighbour Xii is +-4
                //Yi neighbour Yii is +-(512*4)
                //Pixel 0 is 0,0 [0:3], index 0. Pixel 1 is 1,0 is [4:7], index 4. 
                //Pixel 512 is 1,0 [2048:2051], index 2048. Pixel 513 is 1,1 [2052:2055], index 2052.
                //Resolve for y first. Add x. 

                //To compare 1,1 to surroundings.

              //for  
              //for for
              uint32_t current_index = ((4*current_x)+(1024*current_y)*4);
              uint16_t closest_array[4];
              uint16_t low_val = shmArgb->data()[current_index]-priorArgb->data()[current_index];


              for (uint8_t y_r = comp_low; y_r <= comp_high; x++) {
                uint16_t y_index = current_y + y_r; 
                if (y_index > 1024 || y_index < 0){
                  continue;
                }

                for (uint8_t x_r = comp_low; x_r <= comp_high; y++){
                  if (y_index > 1024 || y_index < 0){
                    continue;
                  }
                  uint16_t x_index = current_x + x_r;
                  uint32_t lookup_index = ((4*x_index)+(1024*y_index)*4);
                  uint16_t pixel_dif = shmArgb->data()[current_index]-priorArgb->data()[lookup_index];
                  if (pixel_dif < low_val) {

                    low_val = pixel_dif;
                    closest_array[0] = current_x;
                    closest_array[1] = current_y;
                    closest_array[2] = x_r;
                    closest_array[3] = y_r;
                  }
                }
              }

              pixel_list.push_back(closest_array);



              //close close
              //
              
              
            
              
              

              shmArgb->lock();
              priorArgb->lock();
                
              //Copy contents of current image to old image. 
              memcpy(&priorArgb, &shmArgb, sizeof(priorArgb));
                
                
              shmArgb->unlock();
              priorArgb->unlock();

              
              
            } else {
              //Ensure that an entire circle is complete. 
              
                shmArgb->lock();
                priorArgb->lock();
                
                //Copy contents of current image to old image. 
                memcpy(&priorArgb, &shmArgb, sizeof(priorArgb));
                
                
                shmArgb->unlock();
                priorArgb->unlock();
                init = true;

            }

            */
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
          //subfuntion draw
          //subfunction optic
          //subfunction timings


        }
    
    };

    //Set listening to true and open thread to wait for incoming messages. 
   // od4.dataTrigger(opendlv::proxy::RadarDetectionReading::ID(), onMyMessage1); //This is in the tutorial. Seems to work without it?
    
    bool listening = true;
    int dummy_azimuth = 0;

    while(demo) {

        if (verbose) std::cout << "...DEMO..." << std::endl;
        dummy_azimuth++;
        if (dummy_azimuth >= 4096) dummy_azimuth = 0;
        
        opendlv::proxy::RadarDetectionReading dummy_msg;
    
    //Payload
        dummy_msg.azimuth(dummy_azimuth);
    
        std::vector<uint8_t> sample{

            0xe7, 0x9c, 0x95, 0x95, 0x08, 0x00, 0x7c, 0x0e,
            0x00, 0x06, 0x81, 0xfe, 0x45, 0x00, 0x00, 0xf4,
            0x00, 0x00, 0xaa, 0xff, 0xff, 0x04, 0xc2, 0x92,
            0xe7, 0x9c, 0x95, 0x95, 0x08, 0x00, 0x7c, 0x0e,
            0x00, 0x06, 0x81, 0xfe, 0x45, 0x00, 0x00, 0xf4,
            0x00, 0x00, 0xaa, 0xff, 0xff, 0x04, 0xc2, 0x92,
            0xe7, 0x9c, 0x95, 0x95, 0x08, 0x00, 0x7c, 0x0e,
            0x00, 0x06, 0x81, 0xfe, 0x45, 0x00, 0x00, 0xf4,
            0x00, 0x00, 0xaa, 0xff, 0xff, 0x04, 0xc2, 0x92,
            0xe7, 0x9c, 0x95, 0x95, 0x08, 0x00, 0x7c, 0x0e,
            0x00, 0x06, 0x81, 0xfe, 0x45, 0x00, 0x00, 0xf4,
            0x00, 0x00, 0xaa, 0xff, 0xff, 0x04, 0xc2, 0x92,
            0xe7, 0x9c, 0x95, 0x95, 0x08, 0x00, 0x7c, 0x0e,
            0x00, 0x06, 0x81, 0xfe, 0x45, 0x00, 0x00, 0xf4,
            0x00, 0x00, 0xaa, 0xff, 0xff, 0x04, 0xc2, 0x92,
            0xe7, 0x9c, 0x95, 0x95, 0x08, 0x00, 0x7c, 0x0e,
            0x00, 0x06, 0x81, 0xfe, 0x45, 0x00, 0x00, 0xf4,
            0x00, 0x00, 0xaa, 0xff, 0xff, 0x04, 0xc2, 0x92,
            0xe7, 0x9c, 0x95, 0x95, 0x08, 0x00, 0x7c, 0x0e,
            0x00, 0x06, 0x81, 0xfe, 0x45, 0x00, 0x00, 0xf4,
            0x00, 0x00, 0xaa, 0xff, 0xff, 0x04, 0xc2, 0x92,
            0xe7, 0x9c, 0x95, 0x95, 0x08, 0x00, 0x7c, 0x0e,
            0x00, 0x06, 0x81, 0xfe, 0x45, 0x00, 0x00, 0xf4,
            0x00, 0x00, 0xaa, 0xff, 0xff, 0x04, 0xc2, 0x92,
            0xe7, 0x9c, 0x95, 0x95, 0x08, 0x00, 0x7c, 0x0e,
            0x00, 0x06, 0x81, 0xfe, 0x45, 0x00, 0x00, 0xf4,
            0x00, 0x00, 0xaa, 0xff, 0xff, 0x04, 0xc2, 0x92,
            0xe7, 0x9c, 0x95, 0x95, 0x08, 0x00, 0x7c, 0x0e,
            0x00, 0x06, 0x81, 0xfe, 0x45, 0x00, 0x00, 0xf4,
            0x00, 0x00, 0xaa, 0xff, 0xff, 0x04, 0xc2, 0x92,
            0xe7, 0x9c, 0x95, 0x95, 0x08, 0x00, 0x7c, 0x0e,
            0x00, 0x06, 0x81, 0xfe, 0x45, 0x00, 0x00, 0xf4,
            0x00, 0x00, 0xaa, 0xff, 0xff, 0x04, 0xc2, 0x92,
            0xe7, 0x9c, 0x95, 0x95, 0x08, 0x00, 0x7c, 0x0e,
            0x00, 0x06, 0x81, 0xfe, 0x45, 0x00, 0x00, 0xf4,
            0x00, 0x00, 0xaa, 0xff, 0xff, 0x04, 0xc2, 0x92

          };


          const std::string payload(reinterpret_cast<char*>(sample.data()), sample.size());
          dummy_msg.data(payload);
          dummy_msg.range(1500);
          od4.send(dummy_msg);

    }

    while (listening) {
      
      if (!demo) std::this_thread::sleep_for(1s);
    }
    return retCode;
  }
};