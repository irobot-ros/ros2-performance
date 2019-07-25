#include <cstddef>
#include <cstdio>
#include <cstring>

#include <list>
#include <string>

#include <fstream>
#include <iostream>
#include <sstream>

#include "Poco/SharedLibrary.h"


std::string get_env_var(const char * env_var)
{
  char * value = nullptr;
  value = getenv(env_var);
  std::string value_str = "";
  if (value) {
    value_str = value;
  }
  // printf("get_env_var(%s) = %s\n", env_var, value_str.c_str());
  return value_str;
}

std::list<std::string> split(const std::string & value, const char delimiter)
{
  std::list<std::string> list;
  std::istringstream ss(value);
  std::string s;
  while (std::getline(ss, s, delimiter)) {
    list.push_back(s);
  }

  return list;
}

bool is_file_exist(const char * filename)
{
  std::ifstream h(filename);
  return h.good();
}

std::string find_library_path(const std::string & library_name)
{
  const char * env_var = "LD_LIBRARY_PATH";
  char separator = ':';
  const char * filename_prefix = "lib";
  const char * filename_extension = ".so";

  std::string search_path = get_env_var(env_var);
  std::list<std::string> search_paths = split(search_path, separator);

  std::string filename = filename_prefix;
  filename += library_name + filename_extension;

  for (auto it : search_paths) {
    std::string path = it + "/" + filename;
    if (is_file_exist(path.c_str())) {
      return path;
    }
  }
  return "";
}


std::string
get_library_name(std::string msg_type)
{
  const std::string DEFAULT_LIBRARY_NAME = "irobot_interfaces_plugin";
  std::string namespace_delimiter = "::";

  std::string library_name;
  //std::string msg_name;
  size_t pos = msg_type.find(namespace_delimiter);
  if (pos == std::string::npos) {
    library_name = DEFAULT_LIBRARY_NAME + "_implementation";
    //msg_name = msg_type;
  } else {
    library_name = msg_type.substr(0, pos) + "_implementation";
    //msg_name = msg_type.substr(pos + namespace_delimiter.length(), msg_type.length());
  }

  return library_name;
}


Poco::SharedLibrary *
get_library(std::string msg_type)
{
  Poco::SharedLibrary * lib = nullptr;

  std::string library_name = get_library_name(msg_type);

  std::string library_path = find_library_path(library_name);
  if (library_path.empty()) {
    std::cout<<"Error! library path is empty "<<std::endl;
    return nullptr;
  }

  try {
    lib = new Poco::SharedLibrary(library_path);
  } catch (...) {
    std::cout<<"got exception loading library"<<std::endl;
    return nullptr;
  }

  return lib;
}