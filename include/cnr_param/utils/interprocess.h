#ifndef CNR_PARAM_INCLUDE_CNR_PARAM_UTILS_INTERPROCESS
#define CNR_PARAM_INCLUDE_CNR_PARAM_UTILS_INTERPROCESS

#include <algorithm>

#include <string>
#include <istream>
#include <fstream>
#include <yaml-cpp/yaml.h>

#define BOOST_DATE_TIME_NO_LIB

#include <boost/interprocess/file_mapping.hpp>
#include <boost/interprocess/containers/vector.hpp>
#include <boost/interprocess/allocators/allocator.hpp>
#include <boost/interprocess/containers/string.hpp>
#include <boost/interprocess/managed_mapped_file.hpp>
#include <iostream>
#include <iterator>
#include <sstream>
#include <string>

namespace cnr 
{
namespace param
{
namespace utils
{


void printMemoryContent(const std::string& header, void* addr, bool check_node);

/**
 * @brief Create a File Mapping object
 * 
 * @param absolute_path 
 * @return boost::interprocess::mapped_region* 
 */
boost::interprocess::mapped_region* createFileMapping(const std::string& absolute_path, const std::size_t& file_size);

}  // namespace utils
}  // namespace param
}  // namespace cnr 

#endif  /* CNR_PARAM_INCLUDE_CNR_PARAM_UTILS_INTERPROCESS */
