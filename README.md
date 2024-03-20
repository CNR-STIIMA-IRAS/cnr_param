# cnr_param

A package to read and write parameters for your C++ application.

## Getting Started
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

#### Linux/Unix/Mac
Open your terminal and set the `CNR_PARAM_ROOT_DIRECTORY` environment variable by executing:
```bash
export CNR_PARAM_ROOT_DIRECTORY="your_directory_path"
```
Add this line to your .bashrc, .zshrc, or equivalent shell configuration file to make the change permanent.

#### Windows
Open Command Prompt and set the `CNR_PARAM_ROOT_DIRECTORY` environment variable by executing:
```bash
set CNR_PARAM_ROOT_DIRECTORY="your_directory_path"
```