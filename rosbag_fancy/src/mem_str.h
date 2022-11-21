// Conversions of memory amounts and strings
// Author: Max Schwarz <max.schwarz@ais.uni-bonn.de>

#ifndef ROSBAG_FANCY_MEM_STR_H
#define ROSBAG_FANCY_MEM_STR_H

#include <string>
#include <cstdint>

namespace rosbag_fancy
{
namespace mem_str
{

std::string memoryToString(uint64_t memory);
uint64_t stringToMemory(std::string humanSize);

}
}

#endif
