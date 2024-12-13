#pragma once

#include <string>
#include <iostream>
#include <iomanip>

#include <sys/resource.h>
#include <sys/types.h>

// Globals
struct rusage usage;
long last_rss = 0;

void print_header()
{
  using namespace std;

  // Print header in CSV format
  cout << "Entity,N,RSS,DELTA" << endl;
}

void print_rss(std::string description, size_t number)
{
  using namespace std;

  getrusage(RUSAGE_SELF, &usage);

  long current_rss = usage.ru_maxrss;
  long delta_rss = current_rss - last_rss;

  // Print values in CSV format
  cout << description << ","
       << number << ","
       << current_rss << ","
       << delta_rss << endl;

  last_rss = usage.ru_maxrss;
}