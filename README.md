# cnr_param

A package to read and write parameters for your C++ application.

## The Design

### User Functions

The pacakge provide a unique interface to get and set param that are stored in different databases.

The core is the file [cnr_param.h](./include/cnr_param/cnr_param.h), that should be the only file needed to be included by the user.

The file provide four main functions:

```cpp
bool has(const std::string& key, std::string& what, const std::vector<ModulesID>& modules = {ModulesID::ROS2, ModulesID::MAPPED_FILE});

template<typename T>
bool get(const std::string& key, T& value, std::string& what, const T& default_val, const std::vector<ModulesID>& modules = {ModulesID::ROS2, ModulesID::MAPPED_FILE});

template<typename T>
bool get(const std::string& key, T& value, std::string& what, const std::vector<ModulesID>& modules = {ModulesID::ROS2, ModulesID::MAPPED_FILE});

template<typename T>
bool set(const std::string& key, const T& value, std::string& what, const std::vector<ModulesID>& modules = {ModulesID::ROS2, ModulesID::MAPPED_FILE});
```

The `key` is the name of the parameter that follows both the `ROS1` and `ROS2` convention. In other words, 

```cpp
std::string what;
AnyType value;
bool ok = cnr::param::get("/[namespaces / if present]/node_name/a/b/c/d", value, what )
if(!ok)
{
    std::cout << "The parameter is not available: " << what << std::endl;
}
```

The already supported supported types are:

* All the simpe types and all the type of the `rclcpp::Parameter` 
* Vectors of vectors of simple types and all the type of the `rclcpp::Parameter`  (e.g. `std::vector<std::vector<double>>`, `std::vector<std::vector<string>>`  etc. )

* `Eigen` matrixes (both with static and dynamic dimensions). For example: 
```cpp
std::string what;
Eigen::VectorXd value;
bool ok = cnr::param::get("/[namespaces / if present]/node_name/a/b/c/d", value, what )

```

* `YAML::Node`: this is useful if you want to extract part of your parameters as a whole. For example:
```cpp
std::string what;
YAML::Node value;
bool ok = cnr::param::get("/[namespaces / if present]/node_name/a/b/c/d", value, what )
```

* complex type, like a dictionary or sequences of dictionary. To extract properly a complex type, you must specilize your `template<typename T> get_map(const YAML::Node& node, ComplexType& ret, std::string& what)` ([that is in this file](./include/cnr_param/core/param.h)). Here an example:
```cpp
struct ComplexType
{
  std::string name;
  double value;
};

namespace cnr { namespace param { namespace core {

template <>
bool get_map(const YAML::Node& node, ComplexType& ret, std::string& what)
{
  try
  {
    if (node["name"] && node["value"])
    {
      ret.name = node["name"].as<std::string>();
      ret.value = node["value"].as<double>();
      return true;
    }
  }
  catch (std::exception& e)
  {
    ...
  }
  return false;
}
} } }

```

Once you have defined your specialization you can call:
```cpp
std::string what;
ComplexType value;
bool ok = cnr::param::get("/[namespaces / if present]/node_name/a/b/c/d", value, what )
...
std::vector<ComplexType> values;
bool ok = cnr::param::get("/[namespaces / if present]/node_name/a/b/c/d", values, what )
...
```

### Supported database (modules)

The package is monolitic, and the modules are compiled using proper `CMake` options. Maybe in the future the plugin will be made availble.  

By now, there are three modules for three different databases:

* `ROS1` param server database (to be finished)

* `ROS2` param server database (tested OK)

* `boost mapped files` see below for furhter information

### How to select the database(s)

The last input of the function `set` and `get` is an ordered vector that stores what are the database you want to use. It stars checking the first in the vector, and if it does not found the value there, it looks for the second and repeats.

## ROS 1 Module

TBD

## ROS 2 Module

The module needs that the User call the function

```cpp
#include <cnr_param/ros2/param.h>

rclcpp::Node my_node;
cnr::param::ros2::CNR_PARAM_INIT_RO2_MODULE(my_node);
```

Once the `my_node` is configured, the module is able to look at the node parameters, and it implements the service to ask the parameters to the other nodes under the network.

### ROS 2 yaml formatter

As well know, ROS 2 does not support all the yaml types-Specifically, it does not allow to have sequence of sequences or dictionaries. 
To turn yaml file into ROS 2 yaml file, the package just provide a formatter.

```bash
Usage: ./ros2_yaml_converter [-t [ --to-ros2-format ] str] [-f [ --from-ros2-format ] str] [-o [ --output-filenames ] str] [-v [ --version ] ] [-h [ --help ] ]
Full List of the Options:

Namespaces (shared memories):
  -t [ --to-ros2-format ] str    Convert the yaml files to the ROS2 format. The path is relative to the executable, or absolute if it starts with '/'
  -f [ --from-ros2-format ] str  Convert the yaml files from the ROS2 format.  The path is relative to the executable, or absolute if it starts with '/'
  -o [ --output-filenames ] str  The name of the output file. If not specified, it is the input concat to '_autogererated.yaml'

Generic options:
  -v [ --version ]               print version std::string
  -h [ --help ]                  produce help message
```

For example
```yaml
n1:
  - ["s11","s12","s13"]
  - ["s21","s22","s23"]
  - ["s31","s32","s33"]
```

is formatted as 

```yaml
/**:
  ros__parameters:
    n1_autogenerated_from_sequence_of_Sequence:
      size: 3
      record_0: 
      - s11
      - s12
      - s13
      record_1:
      - s21
      - s22
      - s23
      record_2:
      - s31
      - s32
      - s33
```

In the case you load the autogenerated yaml file into a node, then you can easily access the value using the original keys:

```cpp
std::string what;
std::vector<std::vector<std::string>> value;
bool ok = cnr::param::get("/node_name/n1", value, what )
```

## Mapped File Module

This module creates a param server using the file mapping as implemented in the `boost` `interprocess` library [here](https://www.boost.org/doc/libs/1_85_0/doc/html/interprocess.html#:~:text=Using%20these%20mechanisms%2C%20Boost.Interprocess%20offers%20useful%20tools%20to,implementing%20%20several%20memory%20allocation%20patterns%20%28like%20pooling%29.)

The mapping of the file creates a shared memory mapped on the hard drive under a temporary folder (see bleow for usage). If you want to access the database from remote, you make your temporary folder sharable with other user over the network ([here a tutorial](https://askubuntu.com/questions/15782/how-do-i-share-a-folder-with-another-linux-machine-on-the-same-home-network)).

### Adding parameters to the database

The basic way to load parameters is using this command from terminal:
```
cnr_param_server -p path-to-file
```
To get help and see available options, type:
```
cnr_param_server -h
```
By default, the parameters are saved in the 'cnr_param' folder located within the operating system's temporary folder (e.g., '/tmp' on Linux/Unix/Mac, or a designated temp folder on Windows). 
You can choose another directory for storing your parameters by setting the `CNR_PARAM_ROOT_DIRECTORY` environment variable.

##### Linux/Unix/Mac
Open your terminal and set the `CNR_PARAM_ROOT_DIRECTORY` environment variable by executing:
```bash
export CNR_PARAM_ROOT_DIRECTORY="your_directory_path"
```
Add this line to your .bashrc, .zshrc, or equivalent shell configuration file to make the change permanent.

##### Windows
Open Command Prompt and set the `CNR_PARAM_ROOT_DIRECTORY` environment variable by executing:
```bash
set CNR_PARAM_ROOT_DIRECTORY="your_directory_path"
```
