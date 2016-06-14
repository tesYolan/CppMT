#include "CMT.h"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace cmt {

void CMT::initialize(const Mat im_gray, const Rect rect, string tracker_name, int threshold_value)
{
    initialized = false;
    name = tracker_name;
    threshold = threshold_value;
    //FILE_LOG(logDEBUG) << "CMT::initialize() call";

    //Remember initial size
    size_initial = rect.size();

    //Remember initial image
    im_prev = im_gray;

    //Compute center of rect
    Point2f center = Point2f(rect.x + rect.width / 2.0, rect.y + rect.height / 2.0);

    //Initialize rotated bounding box
    bb_rot = RotatedRect(center, size_initial, 0.0);

    //Initialize detector and descriptor
#if CV_MAJOR_VERSION > 2
    detector = cv::FastFeatureDetector::create();
    descriptor = cv::BRISK::create();
#else
    detector = FeatureDetector::create(str_detector);
    descriptor = DescriptorExtractor::create(str_descriptor);
#endif

    //Get initial keypoints in whole image and compute their descriptors
    vector<KeyPoint> keypoints;
    detector->detect(im_gray, keypoints);

    //Divide keypoints into foreground and background keypoints according to selection
    vector<KeyPoint> keypoints_fg;
    vector<KeyPoint> keypoints_bg;

    for (size_t i = 0; i < keypoints.size(); i++)
    {
        KeyPoint k = keypoints[i];
        Point2f pt = k.pt;
//This is adding whatever is in the rect to be tracked by the system. So the most interesting points are this.
        if (pt.x > rect.x && pt.y > rect.y && pt.x < rect.br().x && pt.y < rect.br().y)
        {
            keypoints_fg.push_back(k);
        }

        else
        {
            keypoints_bg.push_back(k);
        }

    }

    //Create foreground classes
    vector<int> classes_fg;
    classes_fg.reserve(keypoints_fg.size());
    for (size_t i = 0; i < keypoints_fg.size(); i++)
    {
        classes_fg.push_back(i);
    }

    //Compute foreground/background features
    Mat descs_fg;
    Mat descs_bg;
    descriptor->compute(im_gray, keypoints_fg, descs_fg);
    descriptor->compute(im_gray, keypoints_bg, descs_bg);

    //Only now is the right time to convert keypoints to points, as compute() might remove some keypoints
    vector<Point2f> points_fg;
    vector<Point2f> points_bg;

    for (size_t i = 0; i < keypoints_fg.size(); i++)
    {
        points_fg.push_back(keypoints_fg[i].pt);
    }

    //FILE_LOG(logDEBUG) << points_fg.size() << " foreground points.";

    for (size_t i = 0; i < keypoints_bg.size(); i++)
    {
        points_bg.push_back(keypoints_bg[i].pt);
    }

    //Create normalized points
    vector<Point2f> points_normalized;
    for (size_t i = 0; i < points_fg.size(); i++)
    {
        points_normalized.push_back(points_fg[i] - center);
    }

    //Initialize matcher
    matcher.initialize(points_normalized, descs_fg, classes_fg, descs_bg, center);

    //Initialize consensus
    consensus.initialize(points_normalized);

    //Create initial set of active keypoints
    for (size_t i = 0; i < keypoints_fg.size(); i++)
    {
        points_active.push_back(keypoints_fg[i].pt);
        classes_active = classes_fg;
    }

    //Now set the number of active points;
    num_initial_keypoints = points_active.size();

    //Now lets store the values of the images.
    //Now we are using it to detect faces in the system;
    //So let's store the inital face results and the inital face.

    //That way the face recognition can be done in another function.

    initialRect = rect;
    imArchive = im_gray(rect);
    pointsArchive.assign(points_fg.begin(), points_fg.end());
    classesArchive.assign(classes_fg.begin(), classes_fg.end());

    ////FILE_LOG(logDEBUG) << "CMT::initialize() return";
    initialized = true;
    counter = 3;
    ratio_frames = 5;
}

void CMT::updateArea(const Mat im_gray, const Rect rect)
{
    initialize(im_gray, rect, name, threshold);
}

void CMT::set_name(string tracker_name)
{
    name = tracker_name;
    identified = true;
}

void CMT::reset_decreasing_validate(int value)
{
    decreasing_validate = value;
    initial_default = -value;
}

void CMT::processFrame(Mat im_gray, int threshold) {

    ////FILE_LOG(logDEBUG) << "CMT::processFrame() call";
    decreasing_validate--;

    if(decreasing_validate == 0)
    {
            validated = false;
    }
    else if(decreasing_validate == initial_default)
    {
            tracker_lost = true;
    }

    //Track keypoints
    vector<Point2f> points_tracked;
    vector<unsigned char> status;

    //TODO To avoid deleting faces that generated small number of thresholds.
    if(num_initial_keypoints < threshold * 2)
    {
        threshold = num_initial_keypoints / 2 ;
    }

    opticalflow_results = tracker.track(im_prev, im_gray, points_active, points_tracked, status, threshold);

    //If the optical flow results are below the threshold then go to the tracker_lost. That is decrease counter. For ten instance.
//    std::cout<<"OpticalFlow Results: "<<opticalflow_results<<std::endl;
    if (!opticalflow_results)
    {
        if (threshold != 0)
        {
            //Now let's do some processing here.
            if (counter == 0)
            {
                tracker_lost = true;
                return;
            }
            else
            {
                tracker_lost = false;
                counter--;
            }

        }
    }

    //FILE_LOG(logDEBUG) << points_tracked.size() << " tracked points.";

    //keep only successful classes
    vector<int> classes_tracked;
    for (size_t i = 0; i < classes_active.size(); i++)
    {
        if (status[i])
        {
            classes_tracked.push_back(classes_active[i]);
        }

    }

    //Detect keypoints, compute descriptors
    vector<KeyPoint> keypoints;
    detector->detect(im_gray, keypoints);

    //FILE_LOG(logDEBUG) << keypoints.size() << " keypoints found.";

    Mat descriptors;
    descriptor->compute(im_gray, keypoints, descriptors);

    //Match keypoints globally
    vector<Point2f> points_matched_global;
    vector<int> classes_matched_global;
    matcher.matchGlobal(keypoints, descriptors, points_matched_global, classes_matched_global);

    //FILE_LOG(logDEBUG) << points_matched_global.size() << " points matched globally.";

    //Fuse tracked and globally matched points
    vector<Point2f> points_fused;
    vector<int> classes_fused;
    fusion.preferFirst(points_tracked, classes_tracked, points_matched_global, classes_matched_global,
                       points_fused, classes_fused);

    //FILE_LOG(logDEBUG) << points_fused.size() << " points fused.";

    //Estimate scale and rotation from the fused points
    float scale;
    float rotation;
    consensus.estimateScaleRotation(points_fused, classes_fused, scale, rotation);

    //FILE_LOG(logDEBUG) << "scale " << scale << ", " << "rotation " << rotation;

    //Find inliers and the center of their votes
    Point2f center;
    vector<Point2f> points_inlier;
    vector<int> classes_inlier;
    consensus.findConsensus(points_fused, classes_fused, scale, rotation,
                            center, points_inlier, classes_inlier);

    //FILE_LOG(logDEBUG) << points_inlier.size() << " inlier points.";
    //FILE_LOG(logDEBUG) << "center " << center;

    //Match keypoints locally
    vector<Point2f> points_matched_local;
    vector<int> classes_matched_local;
    matcher.matchLocal(keypoints, descriptors, center, scale, rotation, points_matched_local, classes_matched_local);

    //FILE_LOG(logDEBUG) << points_matched_local.size() << " points matched locally.";

    //Assing the active points in the space.

    //Clear active points
    points_active.clear();
    classes_active.clear();

    //Fuse locally matched points and inliers

    fusion.preferFirst(points_matched_local, classes_matched_local, points_inlier, classes_inlier, points_active, classes_active);
//    points_active = points_fused;
//    classes_active = classes_fused;
    num_active_keypoints = points_active.size();
    //FILE_LOG(logDEBUG) << points_active.size() << " final fused points.";

    //TODO: Use theta to suppress result
    bb_rot = RotatedRect(center,  size_initial * scale, rotation / CV_PI * 180);

    //Remember current image
    im_prev = im_gray;


    //FILE_LOG(logDEBUG) << "CMT::processFrame() return";
}

} /* namespace CMT */
