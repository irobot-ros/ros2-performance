#pragma once

#include <iomanip>
#include <malloc.h>

void print_memory_stats()
{
  const char separator = ' ';
  const int name_with = 12;
  const int num_with = 3;

  auto mem_info = mallinfo();

  std::cout << "Memory stats:" << std::endl;
  std::cout << std::left << std::setw(name_with) << std::setfill(separator) << "arena:";
  std::cout << std::right << std::setw(num_with) << std::setfill(separator) << (mem_info.arena >> 20) << "MB" << std::endl;
  std::cout << std::left << std::setw(name_with) << std::setfill(separator) << "in use:";
  std::cout << std::right << std::setw(num_with) << std::setfill(separator) << (mem_info.uordblks >> 20) << "MB" << std::endl;
  std::cout << std::left << std::setw(name_with) << std::setfill(separator) << "mmap:";
  std::cout << std::right << std::setw(num_with) << std::setfill(separator) << (mem_info.hblkhd >> 20) << "MB" << std::endl;

}