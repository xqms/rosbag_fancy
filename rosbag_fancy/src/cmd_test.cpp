// test command
// Author: Max Schwarz <max.schwarz@ais.uni-bonn.de>

#include "doctest.h"

#include <vector>
#include <string>

int test(const std::vector<std::string>& options)
{
	doctest::Context context;

	std::vector<const char*> values;
	values.push_back("dummy");
	for(auto& str : options)
		values.push_back(str.c_str());

	context.applyCommandLine(values.size(), values.data());

	return context.run();
}
