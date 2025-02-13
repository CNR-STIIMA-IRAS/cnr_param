#include <cnr_param/cnr_param.h>

namespace cnr
{
namespace param
{

const bool& is_ros1_avaliable() { static bool ret = ROS1_MODULE==1; return ret; }
const bool& is_ros2_avaliable()  { static bool ret = ROS2_MODULE==1; return ret; }
const bool& is_mapped_file_avaliable()  { static bool ret = MAPPED_FILE_MODULE==1; return ret; }

}  // namespace param
}  // namespace cnr
