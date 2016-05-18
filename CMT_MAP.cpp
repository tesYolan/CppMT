#include "CMT_MAP.h"

namespace cmt {
std::vector<cmt_message> CMTMAP::process_map(const Mat im_gray, const int factor)
{
std::vector<cmt_message> cmt_messages;
queue_tracker.clear();
for(std::map<std::string, cmt::CMT>::iterator v = cmt_.begin(); v!= cmt_.end(); v++)
{

  v->second.processFrame(im_gray, factor);
  cmt_message message;

  message.initial_active_points = v->second.num_initial_keypoints;
  message.active_points = v->second.num_active_keypoints;
  message.tracker_name = v->second.name;
  cv::Rect rect = v->second.bb_rot.boundingRect();

  message.rect = rect & cv::Rect(0, 0, im_gray.size().width, im_gray.size().height);
  message.tracker_lost = v->second.tracker_lost;

  if(message.tracker_lost)
  {
  //ADD TO delete equeue.
  queue_tracker.push_back(message.tracker_name);
  }
  message.recognized = v->second.identified;

  cmt_messages.push_back(message);

}

return cmt_messages;
}

std::vector<string> CMTMAP::removeLost()
{
  for(std::vector<string>::iterator v = queue_tracker.begin(); v!= queue_tracker.end(); v++)
  {
    cmt_.erase(*v);
  }
  return queue_tracker;
}

std::map<string, Mat> CMTMAP::getImages()
{
std::map<string, Mat> returnImages;
 for(std::map<std::string, cmt::CMT>::iterator v = cmt_.begin(); v!= cmt_.end(); v++)
 {
    returnImages[v->first] = v->second.imArchive;
 }
 return returnImages;
}
string CMTMAP::addtomap(const Mat im_gray,const Rect rect)
{
  int tracker_num;
  srand(time(NULL));
  //TODO Fix this to not contain conflicts going forward in previously saved faces
  //and in the files that exist here.
  tracker_num = rand() % 100000;
  std::string tracker_name = "temp_" + SSTR(tracker_num);
  //TODO Here we need to do a check to remove unresolved names
  cmt_[tracker_name] = cmt::CMT();
  cmt_[tracker_name].consensus.estimate_rotation = true;
  cmt_[tracker_name].initialize(im_gray, rect, tracker_name);
  return tracker_name;
}

bool CMTMAP::updatemapname(string tempname, int index)
{
if (cmt_.find(SSTR(index)) == cmt_.end())
{
  cmt_[tempname].set_name(SSTR(index));
  return true;
}
else
{
  //TODO handle if there is a name where there is an index of a name.
return false;
}
}

void CMTMAP::clear()
{
cmt_.clear();
}



}