#ifndef CMT_H

#define CMT_H

#include "common.h"
#include "Consensus.h"
#include "Fusion.h"
#include "Matcher.h"
#include "Tracker.h"
#include <iostream>
#include <opencv2/features2d/features2d.hpp>

using cv::FeatureDetector;
using cv::DescriptorExtractor;
using cv::Ptr;
using cv::RotatedRect;
using cv::Size2f;

namespace cmt
{

class CMT
{
public:
    CMT() : str_detector("FAST"), str_descriptor("BRISK"), initialized(false), name("unset") ,
             identified(false), tracker_lost(false),validated(false),counter(5),decreasing_validate(500){};
    void initialize(const Mat im_gray, const Rect rect, string tracker_name, int threshold=50);
    void processFrame(const Mat im_gray,int threshold=30);
    void set_name(string recognized);

    //Calls the intialize with the existing values. But maintains the previous values.
    void updateArea(const Mat im_gray, const Rect rect);


    Fusion fusion;
    Matcher matcher;
    Tracker tracker;
    Consensus consensus;

    string str_detector;
    string str_descriptor;

    vector<Point2f> points_active; //public for visualization purposes
    RotatedRect bb_rot;
	bool initialized;
	string name;
	//To get the same kind of ratio going in the system. 
	int num_initial_keypoints; 
	int num_active_keypoints; 
    int threshold; 

	//Removing the optical flow if elements are stopped. 
	bool opticalflow_results;
	bool identified;
	bool tracker_lost;


    Mat imArchive;
    vector<Point2f>pointsArchive;
    vector<int>classesArchive;
    Rect initialRect; 

    //This one holds how much frames we need to wait to discard a tracker from any state.


    //TODO this is to enforce tracking the elements.
    //This decreasing counter that resets to a initial counter when a ever a face is detected in the cmt track location.
    string recognized_as;
    void reset_decreasing_validate(int value);
    bool validated;
    int counter;
    int decreasing_validate;
    int initial_default;
    int ratio_frames;
private:
    Ptr<FeatureDetector> detector;
    Ptr<DescriptorExtractor> descriptor;

    Size2f size_initial;

    vector<int> classes_active;

    float theta;

    Mat im_prev;
};

} /* namespace CMT */

#endif /* end of include guard: CMT_H */
