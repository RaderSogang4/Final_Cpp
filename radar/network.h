#pragma once

#include <string>

namespace network
{
	struct LocalNetworkInfo
	{
		std::string Address;
		std::string SubnetMask;

		bool isValid() const
		{
			return !Address.empty() && !SubnetMask.empty();
		}
	};

	bool getLocalNetworkInfo(LocalNetworkInfo& out_info);
}
