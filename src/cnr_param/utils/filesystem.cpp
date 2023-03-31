#include <string>
#include <vector>


#include <cnr_param/utils/filesystem.hpp>

namespace cnr { namespace param { namespace utils {

bool checkfilepath(const fs::path& fp, std::string& what)
{
  what.clear();

  std::string prefix;
  prefix += "Path Error:\n";
  prefix += "\t Filseystem path representation     : " + fp.string() + "\n";
  prefix += "\t Absolute representation of the path: " + fs::absolute(fp).string() + "\n";

  try
  {
    fs::path _fp = fs::canonical(fp);
    if(!fs::exists(_fp))
    {
      what = prefix + "\t The file does not exist!\n";
    }
// #if defined(__cpp_lib_filesystem) || defined(__cpp_lib_experimental_filesystem)
//     else if(fs::is_block_file(_fp))
//     {
//       ww << "\t The file is blocked!\n";
//     }
// #endif
    else if(fs::is_directory(_fp))
    {
      what = prefix + "\t The path is a directory!\n";
    }
    else if(!fs::is_regular_file(_fp))
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



bool filepath(const std::string& fn, fs::path& out, std::string& what)
{
  what.clear();

  std::string prefix;
  auto pwd = fs::current_path();
  auto fp = fs::path(fn);

  prefix += "Path Error:\n";
  prefix += "\t Current directory is               : " + pwd.string() + "\n";
  prefix += "\t Input path                         : " + fn + "\n";
  prefix += "\t Filseystem path representation     : " + fp.string() + "\n";
  prefix += "\t Absolute representation of the path: " + fs::absolute(fn).string() + "\n";

  try
  {
    fs::path _fp = fs::canonical(fp);
    if(!fs::exists(_fp))
    {
      what = prefix + "\t The file does not exist!\n";
    }
// #if defined(__cpp_lib_filesystem) || defined(__cpp_lib_experimental_filesystem)
//     else if(fs::is_block_file(_fp))
//     {
//       ww << "\t The file is blocked!\n";
//     }
// #endif
    else if(fs::is_directory(_fp))
    {
      what = prefix + "\t The path is a directory!\n";
    }
    else if(!fs::is_regular_file(_fp))
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




bool dirpath(const std::string& fn, fs::path& out, std::string& what)
{
  what.clear();

  std::string prefix;
  auto pwd = fs::current_path();
  auto fp = fs::path(fn);

  prefix += "Path Error:\n";
  prefix += "\t Current directory is               : " + pwd.string() + "\n";
  prefix += "\t Input path                         : " + fn + "\n";
  prefix += "\t Filseystem path representation     : " + fp.string() + "\n";
  prefix += "\t Absolute representation of the path: " + fs::absolute(fn).string() + "\n";

  try
  {
    fs::path _fp = fs::canonical(fp);
    if(!fs::exists(_fp))
    {
      what = prefix + "\t The file does not exist!\n";
    }
// #if defined(__cpp_lib_filesystem) || defined(__cpp_lib_experimental_filesystem)
//     else if(fs::is_block_file(_fp))
//     {
//       ww << "\t The file is blocked!\n";
//     }
// #endif
    else if(!fs::is_directory(_fp))
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


}}}