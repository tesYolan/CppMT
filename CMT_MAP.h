#ifndef CMTMAP_H

#define CMTMAP_H

// CMT libraryies
#include "CMT.h"
#include "gui.h"
#ifdef __GNUC__
#include <getopt.h>
#else
#include "getopt/getopt.h"
#endif

#include <time.h>
#include <sstream>
#define SSTR( x ) dynamic_cast< std::ostringstream & >(( std::ostringstream() << std::dec << x ) ).str()
namespace cmt
{
struct cmt_message{
    int initial_active_points;
    int active_points;
    string tracker_name;
    bool tracker_lost;
    bool recognized;
    Rect rect;
};
class CMTMAP
{
private:
std::map <string, cmt::CMT> cmt_;
string tempname_generator();
std::vector<string> queue_tracker;
public:
//This can be threaded in the futhre and join here without affecting functionality
std::vector<cmt_message> process_map(const Mat im_gray, const int factor);
//Let this one create the names and values.
//This one creates temp name values.
string addtomap(const Mat img,const Rect rect);
std::map<string, Mat> getImages();

std::vector<string> removeLost();

void getMapInfo();

bool updatemapname(string tempname, int index);

void updateThreshold();

void clear();
};
}



#endif /* end of include guard: CMTMAP_H */