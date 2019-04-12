#ifndef __PERFORMANCE_TESTS_OPTIONS_HPP__
#define __PERFORMANCE_TESTS_OPTIONS_HPP__

#include <string>
#include <vector>


/// Find "option" in the argument vector.
/**
 * \param[in] args The argument vector
 * \param[in] option The option to search for
 * \return True if option was found in args, false otherwise.
 */
bool find_command_option(
  const std::vector<std::string> & args, const std::string & option);


/// Get the value corresponding to option.
/**
 * \param[in] args The argument vector to search in
 * \param[in] option The option to search for
 * \return The value that comes after "option"
 */
std::string get_command_option(
  const std::vector<std::string> & args, const std::string & option);


/// Parse the C-style argument vector and return experiment-specific parameters.
/// Input params are default values of command line arguments or
/// nullptr if the argument is not used by the experiment
bool parse_command_options(
  int argc, char ** argv,
  int* n_publishers = nullptr, int* n_subscribers = nullptr,
  int* n_clients = nullptr, int* n_services = nullptr,
  std::string* msg_type = nullptr,
  int* msg_size = nullptr,
  int* executors = nullptr,
  float* frequency = nullptr,
  int* experiment_duration = nullptr,
  std::string *filename = nullptr,
  std::string *ros_namespace = nullptr
  );


#endif //__PERFORMANCE_TESTS_OPTIONS_HPP__