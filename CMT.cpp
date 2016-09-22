#include "CMT.h"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace cmt {

void CMT::initialize(const Mat im_gray, const Rect rect)
{
    FILE_LOG(logDEBUG) << "CMT::initialize() call";

    //Remember initial size
    size_initial = rect.size();

    //Remember initial image
    im_prev = im_gray;

    //Compute center of rect
    Point2f center = Point2f(rect.x + rect.width/2.0, rect.y + rect.height/2.0);

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

    FILE_LOG(logDEBUG) << points_fg.size() << " foreground points.";

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
    if (str_estimator == "CONSENSUS")
    {
        consensus.initialize(points_normalized);

    }
    else
    {
        //Just for cases where there are smaller points for the RANSAC to build on
        consensus.initialize(points_normalized);
        Estimator.set_initial_keypoints(points_normalized);
        Estimator.Initialize(20,100); //threshold and maximum number of points
    }


    //Create initial set of active keypoints
    for (size_t i = 0; i < keypoints_fg.size(); i++)
    {
        points_active.push_back(keypoints_fg[i].pt);
        classes_active = classes_fg;
    }
    im_Archive = im_gray(rect);
    initialActivePoints = points_active.size();
    FILE_LOG(logDEBUG) << "CMT::initialize() return";
}

Mat CMT::getOriginalImage()
{
    return im_Archive;
}

int CMT::getInitialActivePoints()
{
    return initialActivePoints;
}

int CMT::getCurrentActivePoints()
{
    return currentActivePoints;
}
void CMT::processFrame(Mat im_gray) {

    FILE_LOG(logDEBUG) << "CMT::processFrame() call";

    //Track keypoints
    vector<Point2f> points_tracked;
    vector<unsigned char> status;
    tracker.track(im_prev, im_gray, points_active, points_tracked, status);

    FILE_LOG(logDEBUG) << points_tracked.size() << " tracked points.";

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

    FILE_LOG(logDEBUG) << keypoints.size() << " keypoints found.";

    Mat descriptors;
    descriptor->compute(im_gray, keypoints, descriptors);

    //Match keypoints globally
    vector<Point2f> points_matched_global;
    vector<int> classes_matched_global;
    matcher.matchGlobal(keypoints, descriptors, points_matched_global, classes_matched_global);

    FILE_LOG(logDEBUG) << points_matched_global.size() << " points matched globally.";

    //Fuse tracked and globally matched points
    vector<Point2f> points_fused;
    vector<int> classes_fused;
    fusion.preferFirst(points_tracked, classes_tracked, points_matched_global, classes_matched_global,
            points_fused, classes_fused);

    FILE_LOG(logDEBUG) << points_fused.size() << " points fused.";

    //Estimate scale and rotation from the fused points
    float scale;
    float rotation;
    Point2f center;//This are the points that i need to replace to incoporate it
    vector<Point2f> points_inlier;
    vector<int> classes_inlier;

    if (str_estimator == "CONSENSUS" || points_fused.size() < 4)
    {
        int start = cv::getTickCount();
        consensus.estimateScaleRotation(points_fused, classes_fused, scale, rotation);
        FILE_LOG(logDEBUG) << "scale " << scale << ", " << "rotation " << rotation;
        consensus.findConsensus(points_fused, classes_fused, scale, rotation,
            center, points_inlier, classes_inlier);
        int end = cv::getTickCount();
        FILE_LOG(logINFO)<< "CONSENSUS took: " << GRANSAC::VPFloat(end-start) / GRANSAC::VPFloat(cv::getTickFrequency()) * 1000.0 << " ms." << std::endl;
    }
    else
    {
        int start = cv::getTickCount();
        CandPoints.clear();

        for (size_t i = 0; i < points_fused.size(); i++)
        {
        std::shared_ptr<GRANSAC::AbstractParameter> CandPt = std::make_shared<Point2D>(points_fused[i],classes_fused[i]);
        CandPoints.push_back(CandPt);
        }

        Estimator.Estimate(CandPoints);

        auto BestInliers = Estimator.GetBestInliers();
        if(BestInliers.size() > 0)
        {
            for(auto& Inlier : BestInliers)
            {
                auto RPt = std::dynamic_pointer_cast<Point2D>(Inlier);
                //Get the center, scale, rotation and fused points here.
                points_inlier.push_back(RPt->m_normalized);
                classes_inlier.push_back(RPt->m_class);
            }
        }
        auto BestModel = Estimator.GetBestModel();

        if (BestModel)
        {
        std::vector<GRANSAC::VPFloat> p = BestModel->m_model.first;

        scale = p[0];
        rotation = p[1];
        center = BestModel->m_model.second;

        }

        int end = cv::getTickCount();
        FILE_LOG(logINFO)<< "RANSAC took: " << GRANSAC::VPFloat(end-start) / GRANSAC::VPFloat(cv::getTickFrequency()) * 1000.0 << " ms." ;

    }
    FILE_LOG(logINFO) <<points_fused.size()<<std::endl;

    //The above should give the scale, rotation;
    FILE_LOG(logINFO) << points_inlier.size() << " inlier points.";
    FILE_LOG(logINFO) << "center " << center;
    FILE_LOG(logINFO) << "scale " << scale;
    FILE_LOG(logINFO) << "rotation " << rotation;

    //Match keypoints locally
    vector<Point2f> points_matched_local;
    vector<int> classes_matched_local;
    matcher.matchLocal(keypoints, descriptors, center, scale, rotation, points_matched_local, classes_matched_local);

    FILE_LOG(logDEBUG) << points_matched_local.size() << " points matched locally.";

    //Clear active points
    points_active.clear();
    classes_active.clear();

    //Fuse locally matched points and inliers
    fusion.preferFirst(points_matched_local, classes_matched_local, points_inlier, classes_inlier, points_active, classes_active);
//    points_active = points_fused;
//    classes_active = classes_fused;

    FILE_LOG(logDEBUG) << points_active.size() << " final fused points.";

    //TODO: Use theta to suppress result
    bb_rot = RotatedRect(center,  size_initial * scale, rotation/CV_PI * 180);

    //Remember current image
    im_prev = im_gray;
    currentActivePoints = points_active.size();

    FILE_LOG(logDEBUG) << "CMT::processFrame() return";
}

} /* namespace CMT */
