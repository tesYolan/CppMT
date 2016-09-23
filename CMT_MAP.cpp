#include "CMT_MAP.h"

namespace cmt{
bool CMT_MAP::addTracker(const Mat im_gray, const Rect rect, string tracker_name, Config config)
{
    cmt_[tracker_name] = CMT(config);
    cmt_[tracker_name].consensus.estimate_rotation = true;
    cmt_[tracker_name].initialize(im_gray, rect);

    return true;
}
bool CMT_MAP::removeTracker(string tracker_name)
{
    if (cmt_.find(tracker_name)!=cmt_.end())
    {
        cmt_.erase(tracker_name);
        return true;
    }
    else
    {
        return false;
    }
}
bool CMT_MAP::clear()
{
    cmt_.clear();
    return true;
}
std::map<string, Mat> CMT_MAP::getTrackedImages()
{
 std::map<string, Mat> returnImages;
 for(std::map<std::string, cmt::CMT>::iterator v = cmt_.begin(); v!= cmt_.end(); v++)
 {
    returnImages[v->first] = v->second.getOriginalImage();
 }
 return returnImages;
}
std::vector<string> CMT_MAP::getTrackerNames()
{
 std::vector<string> names;
 for(std::map<std::string, cmt::CMT>::iterator v = cmt_.begin(); v!= cmt_.end(); v++)
 {
    names.push_back(v->first);
 }
 return names;
}
void CMT_MAP::process(const Mat im_gray)
{
for(std::map<std::string, cmt::CMT>::iterator v = cmt_.begin(); v!= cmt_.end(); v++)
{
    cmt_[v->first].processFrame(im_gray);
}
}

const std::map<string, cmt::CMT > CMT_MAP::get_map()
{
    const std::map<string, cmt::CMT > ptr = cmt_;
    return ptr;
}

}