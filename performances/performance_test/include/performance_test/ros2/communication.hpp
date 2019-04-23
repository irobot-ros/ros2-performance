#pragma once

#include <string>

namespace performance_test {

template <typename Interface>
class CommunicationAbstraction
{
public:
  CommunicationAbstraction() = delete;
  CommunicationAbstraction(const std::string n)
    : name(n){};
  const std::string name;
};

// create alias templates for Topic and Service from common class CommunicationAbstraction

template <typename Msg>
using Topic = CommunicationAbstraction<Msg>;

template <typename Srv>
using Service = CommunicationAbstraction<Srv>;

}