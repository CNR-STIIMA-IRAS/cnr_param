#include <cnr_param/utils/colors.hpp>

namespace cnr { namespace param { namespace utils {

std::string RESET()
{
  return "\033[0m";
}
std::string BLACK()
{
  return "\033[30m";
}
std::string RED()
{
  return "\033[31m";
}
std::string GREEN()
{
  return "\033[32m";
}
std::string YELLOW()
{
  return "\033[33m";
}
std::string BLUE()
{
  return "\033[34m";
}
std::string MAGENTA()
{
  return "\033[35m";
}
std::string CYAN()
{
  return "\033[36m";
}
std::string WHITE()
{
  return "\033[37m";
}
std::string BOLDBLACK()
{
  return "\033[1m\033[30m";
}
std::string BOLDRED()
{
  return "\033[1m\033[31m";
}
std::string BOLDGREEN()
{
  return "\033[1m\033[32m";
}
std::string BOLDYELLOW()
{
  return "\033[1m\033[33m";
}
std::string BOLDBLUE()
{
  return "\033[1m\033[34m";
}
std::string BOLDMAGENTA()
{
  return "\033[1m\033[35m";
}
std::string BOLDCYAN()
{
  return "\033[1m\033[36m";
}
std::string BOLDWHITE()
{
  return "\033[1m\033[37m";
}

}}}