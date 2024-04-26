#include <string>
#include <boost/filesystem.hpp>

#include <cnr_param/utils/filesystem.h>

namespace cnr
{
namespace param
{
namespace utils 
{

bool checkfilepath(const boost::filesystem::path& fp, std::string& what)
{
  what.clear();

  std::string prefix;
  prefix += "Path Error:\n";
  prefix += "\t Filseystem path representation     : " + fp.string() + "\n";
  prefix += "\t Absolute representation of the path: " + boost::filesystem::absolute(fp).string() + "\n";

  try
  {
    boost::filesystem::path _fp = boost::filesystem::canonical(fp);
    if(!boost::filesystem::exists(_fp))
    {
      what = prefix + "\t The file does not exist!\n";
    }
// #if defined(__cpp_lib_filesystem) || defined(__cpp_lib_experimental_filesystem)
//     else if(boost::filesystem::is_block_file(_fp))
//     {
//       ww << "\t The file is blocked!\n";
//     }
// #endif
    else if(boost::filesystem::is_directory(_fp))
    {
      what = prefix + "\t The path is a directory!\n";
    }
    else if(!boost::filesystem::is_regular_file(_fp))
    {
      what = prefix + "\t The path is broken..\n";
    }
  }
  catch(const std::exception& e)
  {
    what = prefix + "\t Exception Caught: " + std::string( e.what() ) + '\n';
  }
  return !what.size();
}


/**
 * @brief 
 * 
 * @param fn 
 * @param out 
 * @param what 
 * @return true 
 * @return false 
 */
bool filepath(const std::string& fn, boost::filesystem::path& out, std::string& what)
{
  what.clear();

  std::string prefix;
  auto pwd = boost::filesystem::current_path();
  auto fp = boost::filesystem::path(fn);

  prefix += "Path Error:\n";
  prefix += "\t Current directory is               : " + pwd.string() + "\n";
  prefix += "\t Input path                         : " + fn + "\n";
  prefix += "\t Filseystem path representation     : " + fp.string() + "\n";
  prefix += "\t Absolute representation of the path: " + boost::filesystem::absolute(fn).string() + "\n";

  try
  {
    boost::filesystem::path _fp = boost::filesystem::canonical(fp);
    if(!boost::filesystem::exists(_fp))
    {
      what = prefix + "\t The file does not exist!\n";
    }
// #if defined(__cpp_lib_filesystem) || defined(__cpp_lib_experimental_filesystem)
//     else if(boost::filesystem::is_block_file(_fp))
//     {
//       ww << "\t The file is blocked!\n";
//     }
// #endif
    else if(boost::filesystem::is_directory(_fp))
    {
      what = prefix + "\t The path is a directory!\n";
    }
    else if(!boost::filesystem::is_regular_file(_fp))
    {
      what = prefix + "\t The path is broken..\n";
    }
    else
    {
      out = _fp;
    }
  }
  catch(const std::exception& e)
  {
    what = prefix + "\t Exception Caught: " + std::string( e.what() ) + '\n';
  }
  return !what.size();
}


/**
 * @brief 
 * 
 * @param fn 
 * @param out 
 * @param what 
 * @return true 
 * @return false 
 */
bool dirpath(const std::string& fn, boost::filesystem::path& out, std::string& what)
{
  what.clear();

  std::string prefix;
  auto pwd = boost::filesystem::current_path();
  auto fp = boost::filesystem::path(fn);

  prefix += "Path Error:\n";
  prefix += "\t Current directory is               : " + pwd.string() + "\n";
  prefix += "\t Input path                         : " + fn + "\n";
  prefix += "\t Filseystem path representation     : " + fp.string() + "\n";
  prefix += "\t Absolute representation of the path: " + boost::filesystem::absolute(fn).string() + "\n";

  try
  {
    boost::filesystem::path _fp = boost::filesystem::canonical(fp);
    if(!boost::filesystem::exists(_fp))
    {
      what = prefix + "\t The file does not exist!\n";
    }
// #if defined(__cpp_lib_filesystem) || defined(__cpp_lib_experimental_filesystem)
//     else if(boost::filesystem::is_block_file(_fp))
//     {
//       ww << "\t The file is blocked!\n";
//     }
// #endif
    else if(!boost::filesystem::is_directory(_fp))
    {
      what = prefix + "\t The path is not a directory!\n";
    }
    else
    {
      out = _fp;
    }
  }
  catch(const std::exception& e)
  {
    what = prefix + "\t Exception Caught: " + std::string( e.what() ) + '\n';
  }
  return !what.size();
}


}
}
}