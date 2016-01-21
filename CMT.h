#ifndef CMT_H

#define CMT_H

#include "common.h"
#include "Consensus.h"
#include "Fusion.h"
#include "Matcher.h"
#include "Tracker.h"

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
    CMT() : str_detector("FAST"), str_descriptor("BRISK"), initialized(false), name("unset") , threshold(50), tracker_lost(false){};
    void initialize(const Mat im_gray, const Rect rect, string tracker_name, int threshold=50);
    void processFrame(const Mat im_gray,int threshold=50);

    Fusion fusion;
    Matcher matcher;
    Tracker tracker;
    Consensus consensus;

    string str_detector;
    string str_descriptor;

    vector<Point2f> points_active; //public for visualization purposes
    RotatedRect bb_rot;
	bool initialized; 
	//To get the same kind of ratio going in the system. 
	int num_initial_keypoints; 
	int num_active_keypoints; 
    int threshold; 

	//Removing the optical flow if elements are stopped. 
	bool opticalflow_results; 
	bool tracker_lost; 
	string name; 

    Mat imArchive;
    vector<Point2f>pointsArchive;
    vector<int>classesArchive;
    Rect initialRect; 

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
