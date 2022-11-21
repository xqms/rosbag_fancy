// Conversions of memory amounts and strings
// Author: Max Schwarz <max.schwarz@ais.uni-bonn.de>

#include "mem_str.h"

#include <fmt/format.h>

#include <regex>

namespace rosbag_fancy
{
namespace mem_str
{

std::string memoryToString(uint64_t memory)
{
	if(memory < static_cast<uint64_t>(1<<10))
		return fmt::format("{}.0   B", memory);
	else if(memory < static_cast<uint64_t>(1<<20))
		return fmt::format("{:.1f} KiB", static_cast<double>(memory) / static_cast<uint64_t>(1<<10));
	else if(memory < static_cast<uint64_t>(1<<30))
		return fmt::format("{:.1f} MiB", static_cast<double>(memory) / static_cast<uint64_t>(1<<20));
	else if(memory < static_cast<uint64_t>(1ull<<40))
		return fmt::format("{:.1f} GiB", static_cast<double>(memory) / static_cast<uint64_t>(1ull<<30));
	else
		return fmt::format("{:.1f} TiB", static_cast<double>(memory) / static_cast<uint64_t>(1ull<<40));
}

uint64_t stringToMemory(std::string humanSize)
{
	// format should be "10MB" or "3 GB" or "10 B" or "10GiB"
	static std::regex memoryRegex{R"EOS((\d+)\s*(K|M|G|T|E|)i?B?)EOS"};

	std::smatch match;
	if(!std::regex_match(humanSize, match, memoryRegex))
	{
		fmt::print(stderr, "Could not parse memory string '{}' - it should be something like '120B' or '10GB'\n",
			humanSize
		);
		std::exit(1);
	}

	std::string number = match[1].str();
	std::string unit = match[2].str();

	std::uint64_t multiplier = [&]() -> std::uint64_t {
		if(unit.empty())
			return 1;

		switch(unit[0])
		{
			case 'K': return 1ULL << 10;
			case 'M': return 1ULL << 20;
			case 'G': return 1ULL << 30;
			case 'T': return 1ULL << 40;
			case 'E': return 1ULL << 50;
		}

		throw std::logic_error{"I got regexes wrong :("};
	}();

	return std::stoull(humanSize) * multiplier;
}

}
}
