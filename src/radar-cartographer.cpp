
#include <iostream>
#include <string>


//Import OpenCV
#include "opencv4/opencv2/core.hpp"
#include "opencv4/opencv2/core/ocl.hpp"
#include "opencv4/opencv2/core/utility.hpp"
#include "opencv4/opencv2/highgui.hpp"
#include "opencv4/opencv2/imgcodecs.hpp"
#include "opencv4/opencv2/optflow.hpp"
#include "opencv4/opencv2/video.hpp"

int32_t optic(uint8_t preset, std::unique_ptr<cluon::SharedMemory> &shmArgb, std::unique_ptr<cluon::SharedMemory> &shmArgbt1, std::unique_ptr<cluon::SharedMemory> &optic) {
  //May have to reinterpret cast. Try this first. 


  cv::UMat uImageBefore = cv::imread(&shmArgb, cv::IMREAD_GRAYSCALE)
                              .getUMat(cv::ACCESS_WRITE);
  cv::UMat uImageAfter = cv::imread(&shmArgbt1, cv::IMREAD_GRAYSCALE)
                             .getUMat(cv::ACCESS_WRITE);

  cv::UMat flowMat;
  cv::Ptr<cv::DISOpticalFlow> dis;
  if (preset == 0) {
    dis = cv::DISOpticalFlow::create(cv::DISOpticalFlow::PRESET_ULTRAFAST);
  } else if (preset == 1) {
    dis = cv::DISOpticalFlow::create(cv::DISOpticalFlow::PRESET_FAST);
  } else if (preset == 2) {
    dis = cv::DISOpticalFlow::create(cv::DISOpticalFlow::PRESET_MEDIUM);
  } else {
    std::cerr << "Invalid preset option" << std::endl;
    return -1;
  }

  dis->calc(uImageBefore, uImageAfter, flowMat);
  cv::writeOpticalFlow(&optic, flowMat);
  
  if (&shmArgbt1->isvalid() && &shmArgb->isvalid()) {
    memcpy(&shmArgbt1, shmArgb, sizeof(shmArgb));
  } else {
    return -2;
  }
  
  return 0;

}