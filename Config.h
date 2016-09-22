#ifndef CONFIG_H

#define CONFIG_H
#include <string>
#include "logging/log.h"
namespace cmt
{

class Config
{
public:

   Config() : str_detector("FAST"), str_descriptor("BRISK"), str_estimation("CONSENSUS"), ratio_threshold(0.3) {};

   Config(std::string detector, std::string descriptor, std::string estimation, double ratio)
   {
    // TODO VALIDATE "FAST" ,"STAR" ,"ORB" ,"BRISK" , "MSER" ,"GFTT" ,"HARRIS", "Dense" ,"SimpleBlob"
    // TODO Can we use SIFT and SURF Modules.
   str_detector = detector;
   str_descriptor = descriptor;
   str_estimation = estimation;
   ratio_threshold = ratio;
   };

   std::string str_detector;
   std::string str_descriptor;
   std::string str_estimation;

   double ratio_threshold;

   void set_ratio(double);



   //INCLUDE MAY BE THE FOLLOWING IN THIS
//
//        {"challenge", no_argument, &challenge_flag, 1},
//        {"loop", no_argument, &loop_flag, 1},
//        {"verbose", no_argument, &verbose_flag, 1},
//        {"no-scale", no_argument, 0, no_scale_cmd},
//        {"with-rotation", no_argument, 0, with_rotation_cmd},
//        //Argument options
//        {"bbox", required_argument, 0, bbox_cmd},
//        {"detector", required_argument, 0, detector_cmd},
//        {"descriptor", required_argument, 0, descriptor_cmd},
//        {"output-file", required_argument, 0, output_file_cmd},
//        {"skip", required_argument, 0, skip_cmd},
//        {"skip-msecs", required_argument, 0, skip_msecs_cmd},
//        {0, 0, 0, 0}
};
}
#endif