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
#ifndef _WIN32
  value = getenv(env_var);
#else
  size_t value_size;
  _dupenv_s(&value, &value_size, env_var);
#endif
  std::string value_str = "";
  if (value) {
    value_str = value;
#ifdef _WIN32
    free(value);
#endif
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
  // printf("split(%s) = %zu\n", value.c_str(), list.size());
  return list;
}

bool is_file_exist(const char * filename)
{
  std::ifstream h(filename);
  // printf("is_file_exist(%s) = %s\n", filename, h.good() ? "true" : "false");
  return h.good();
}

std::string find_library_path(const std::string & library_name)
{
  const char * env_var;
  char separator;
  const char * filename_prefix;
  const char * filename_extension;
#ifdef _WIN32
  env_var = "PATH";
  separator = ';';
  filename_prefix = "";
  filename_extension = ".dll";
#elif __APPLE__
  env_var = "DYLD_LIBRARY_PATH";
  separator = ':';
  filename_prefix = "lib";
  filename_extension = ".dylib";
#else
  env_var = "LD_LIBRARY_PATH";
  separator = ':';
  filename_prefix = "lib";
  filename_extension = ".so";
#endif
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

  std::string library_delimiter = "::";
  std::string library_name;
  //std::string msg_name;
  size_t pos = msg_type.find(library_delimiter);
  if (pos == std::string::npos) {
    library_name = DEFAULT_LIBRARY_NAME + "_implementation";
    //msg_name = msg_type;
  } else {
    library_name = msg_type.substr(0, pos) + "_implementation";
    //msg_name = msg_type.substr(pos + library_delimiter.length(), msg_type.length());
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