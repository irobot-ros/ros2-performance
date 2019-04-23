#include "performance_test/ros2/names_utilities.hpp"

#include <sstream>
#include <regex>

std::string performance_test::id_to_node_name(int id)
{
  std::stringstream ss;
  ss << "node_";
  ss << id;
  return ss.str();
}

std::string performance_test::id_to_service_name(int id)
{
  std::stringstream ss;
  ss << "service_";
  ss << id;
  return ss.str();
}

std::string performance_test::id_to_topic_name(int id)
{
  std::stringstream ss;
  ss << "topic_";
  ss << id;
  return ss.str();
}


int performance_test::item_name_to_id(std::string name)
{

  // if the name is in the format "node_7", "topic_1", "service_12" extract the number from it
  // otherwise create an hash code from the string

  //TODO: use only one regex with positive lookahead and start string delimiter
  //TODO: directly call replace and check if length is changed

  std::regex e ("(node|topic|service)_[[:digit:]]+");

  if (std::regex_match (name, e)){
    std::regex e2 ("(node|topic|service)_");
    std::stringstream number_string;
    std::regex_replace(std::ostream_iterator<char>(number_string), name.begin(), name.end(), e2, "");
    // This will FAIL if the provided number starts with 0 e.g. "topic_01" but it should not happen
    return std::stoi(number_string.str());
  }

  // fallback if regex condition is not satisfied
  return std::hash<std::string>()(name);

}
