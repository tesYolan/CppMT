#ifndef CMT_H

#define CMT_H

#include "common.h"
#include "Consensus.h"
#include "Config.h"
#include "Fusion.h"
#include "Matcher.h"
#include "Tracker.h"


#include "GRANSAC.hpp"
#include "RANSAC_model.h"

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
    CMT() : str_detector("FAST"), str_descriptor("BRISK") , str_estimator("CONSENSUS"){};
    CMT(Config config)
    {
        str_descriptor = config.str_descriptor;
        str_detector = config.str_detector;
        str_estimator = config.str_estimation;
    }
    void initialize(const Mat im_gray, const Rect rect);
    void processFrame(const Mat im_gray);
    Mat getOriginalImage();
    int getInitialActivePoints();
    int getCurrentActivePoints();

    Fusion fusion;
    Matcher matcher;
    Tracker tracker;
    Consensus consensus;

    string str_detector;
    string str_descriptor;
    string str_estimator;

    vector<Point2f> points_active; //public for visualization purposes
    std::vector<std::shared_ptr<GRANSAC::AbstractParameter>> CandPoints;
    RotatedRect bb_rot;

private:
    GRANSAC::RANSAC<HomographyModel, 4> Estimator;
    Ptr<FeatureDetector> detector;
    Ptr<DescriptorExtractor> descriptor;

    int initialActivePoints;
    int currentActivePoints;
    Size2f size_initial;

    vector<int> classes_active;

    float theta;

    Mat im_prev;
    Mat im_Archive;
};

} /* namespace CMT */

#endif /* end of include guard: CMT_H */
