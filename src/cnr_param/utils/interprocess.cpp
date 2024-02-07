#include <functional>
#include <string>

#include <cnr_param/utils/string.h>
#include <cnr_param/utils/filesystem.h>
#include <cnr_param/utils/interprocess.h>

namespace cnr 
{
namespace param
{
namespace utils
{

void printMemoryContent(const std::string& header, void* addr, bool check_node)
{
  //Check that memory was initialized to 1
  const char *mem = static_cast<char*>(addr);
  std::string strmem(mem);

  std::cout << "^^^^^^^^^^^^^^^^^^^^" << std::endl;
  std::cout << header << std::endl;
  std::cout << strmem << std::endl;
  std::cout << "^^^^^^^^^^^^^^^^^^^^" << std::endl;

  if(check_node)
  {
    auto n = YAML::Load(strmem);
    std::cout << "key: " << n.begin()->first << std::endl;
    std::cout << "value: " << n.begin()->second << std::endl;
    std::cout << "^^^^^^^^^^^^^^^^^^^^" << std::endl;
  }
}

boost::interprocess::mapped_region* createFileMapping(const std::string& absolute_path, const std::size_t& file_size)
{
  auto l = __LINE__;
  try
  {
    std::function<void(const char* ap)> filebuf_create = [&file_size](const char* ap) 
    {
      auto tree = cnr::param::utils::tokenize(std::string(ap),"/");
      std::string root_dir = "/";
      for(std::size_t i=0;i<tree.size()-1;i++)
        root_dir += tree.at(i) +"/";

      boost::filesystem::create_directories(root_dir);
      
      boost::interprocess::file_mapping::remove(ap);
      std::filebuf fbuf;
      auto res = fbuf.open(ap, std::ios_base::in | std::ios_base::out 
                              | std::ios_base::trunc | std::ios_base::binary); 
      if(!res)
      { 
        throw std::runtime_error("Failed in opening the filebuffer");
      }
      //Create a file mapping  
      fbuf.pubseekoff(file_size-1, std::ios_base::beg);
      fbuf.sputc(0);
    };

    filebuf_create(absolute_path.c_str());

    //Create a file mapping
    boost::interprocess::file_mapping file(absolute_path.c_str(), boost::interprocess::read_write);

    l = __LINE__;
    //Map the whole file with read-write permissions in this process
    boost::interprocess::mapped_region*  region = 
      new boost::interprocess::mapped_region(file, boost::interprocess::read_write);

    l = __LINE__;
    //Get the address of the mapped region
    void* addr        = region->get_address();

    l = __LINE__;
    std::size_t size  = region->get_size();
   
    l = __LINE__;
    //Write all the memory to 1
    std::memset(addr, 0, size );

    return region;
  }
  catch(boost::interprocess::lock_exception& e)
  {
    std::cerr << __PRETTY_FUNCTION__ << ":" << __LINE__ << ": Aboslute path: " << absolute_path << std::endl;
    std::cerr << __PRETTY_FUNCTION__ << ":" << __LINE__ << ": last executed line " << l << ":" << e.what() << std::endl;
  }
  catch(boost::interprocess::bad_alloc& e)
  {
    std::cerr << __PRETTY_FUNCTION__ << ":" << __LINE__ << ": Aboslute path: " << absolute_path << std::endl;
    std::cerr << __PRETTY_FUNCTION__ << ":" << __LINE__ << ": last executed line " << l << ":" << e.what() << std::endl;
  }
  catch(boost::interprocess::interprocess_exception& e)
  {
    std::cerr << __PRETTY_FUNCTION__ << ":" << __LINE__ << ": Aboslute path: " << absolute_path << std::endl;
    std::cerr << __PRETTY_FUNCTION__ << ":" << __LINE__ << ": last executed line " << l << ":" << e.what() << std::endl;
  }
  catch(std::exception& e)
  {
    std::cerr << __PRETTY_FUNCTION__ << ":" << __LINE__ << ": Aboslute path: " << absolute_path << std::endl;
    std::cerr << __PRETTY_FUNCTION__ << ":" << __LINE__ << ": last executed line " << l << ":" << e.what() << std::endl;
  }
  return nullptr;
}

}  // namespace utils
}  // namespace param
}  // namespace cnr 

