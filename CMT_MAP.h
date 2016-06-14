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

#include <boost/thread.hpp>
#include <time.h>
#include <sstream>
#define SSTR( x ) dynamic_cast< std::ostringstream & >(( std::ostringstream() << std::dec << x ) ).str()
namespace cmt
{
struct cmt_message{
    int initial_active_points;
    int active_points;
    string tracker_name;
    // This is to indicate a lost value.  This shouldn't be visable.
    bool tracker_lost;
    bool recognized;
    //This two are the opposite.
    bool validated;
    int before_being_demoted;

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
std::vector<cmt_message> process_map(const Mat im_gray, const int factor,std::map<string, string> map_merge,double ratio=0.3);
//Let this one create the names and values.
//This one creates temp name values.
string addtomap(const Mat img,const Rect rect);
std::map<string, Mat> getImages();

std::vector<string> removeLost();

std::vector<string> string_1;
std::vector<string> string_2;
std::vector<string> string_3;
std::vector<string> string_4;

void getMapInfo();

bool updatemapname(string tempname, int index);

void updateThreshold();

void clear();

void process(const Mat im_gray, const int factor,std::vector<string> string_);

void separate();

bool validate(string name);
bool reinforce(string name, int value);
};
}



#endif /* end of include guard: CMTMAP_H */