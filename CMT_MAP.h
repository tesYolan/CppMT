#ifndef CMT_MAP_H

#define CMT_MAP_H

#include "CMT.h"
namespace cmt
{

class CMT_MAP {
public:
    bool addTracker(const Mat img, const Rect rect,string tracker_name);
    bool removeTracker(string id );
    bool clear();

    void process(const Mat im_gray);

    //Returns the image that CppMT is tracking.
    std::map<string, Mat> getTrackedImages();
    std::vector<string> getTrackerNames();
    const std::map<string, cmt::CMT > get_map();

private:
    std::map <string, cmt::CMT> cmt_;
};
}
#endif //end of CMT_MAP class