#include "network.h"

#include <cstring>
#include <vector>

#ifdef _WIN32
#define NOMINMAX
#include <winsock2.h>
#include <ws2tcpip.h>
#include <iphlpapi.h>
#pragma comment(lib, "iphlpapi.lib")
#else
#include <arpa/inet.h>
#include <ifaddrs.h>
#include <net/if.h>
#include <netinet/in.h>
#endif

namespace network
{
	bool getLocalNetworkInfo(LocalNetworkInfo& out_info)
	{
		out_info = {};

#ifdef _WIN32
		ULONG buffer_size = 0;
		if (GetAdaptersAddresses(AF_INET, GAA_FLAG_SKIP_ANYCAST | GAA_FLAG_SKIP_MULTICAST | GAA_FLAG_SKIP_DNS_SERVER, nullptr, nullptr, &buffer_size) != ERROR_BUFFER_OVERFLOW)
			return false;

		std::vector<unsigned char> buffer(buffer_size);
		IP_ADAPTER_ADDRESSES* adapters = reinterpret_cast<IP_ADAPTER_ADDRESSES*>(buffer.data());
		if (GetAdaptersAddresses(AF_INET, GAA_FLAG_SKIP_ANYCAST | GAA_FLAG_SKIP_MULTICAST | GAA_FLAG_SKIP_DNS_SERVER, nullptr, adapters, &buffer_size) != NO_ERROR)
			return false;

		for (IP_ADAPTER_ADDRESSES* adapter = adapters; adapter != nullptr; adapter = adapter->Next)
		{
			if (adapter->OperStatus != IfOperStatusUp)
				continue;
			for (IP_ADAPTER_UNICAST_ADDRESS* unicast = adapter->FirstUnicastAddress; unicast != nullptr; unicast = unicast->Next)
			{
				if (unicast->Address.lpSockaddr == nullptr || unicast->Address.lpSockaddr->sa_family != AF_INET)
					continue;

				const sockaddr_in* address = reinterpret_cast<const sockaddr_in*>(unicast->Address.lpSockaddr);
				const unsigned long address_host = ntohl(address->sin_addr.s_addr);
				if ((address_host >> 24) == 127)
					continue;

				char ip_buffer[INET_ADDRSTRLEN] = {};
				if (inet_ntop(AF_INET, &address->sin_addr, ip_buffer, sizeof(ip_buffer)) == nullptr)
					continue;

				unsigned long prefix_mask = unicast->OnLinkPrefixLength == 0 ? 0 : (0xFFFFFFFFul << (32 - unicast->OnLinkPrefixLength));
				prefix_mask = htonl(prefix_mask);
				in_addr mask_addr {};
				mask_addr.s_addr = prefix_mask;
				char mask_buffer[INET_ADDRSTRLEN] = {};
				if (inet_ntop(AF_INET, &mask_addr, mask_buffer, sizeof(mask_buffer)) == nullptr)
					continue;

				out_info.Address = ip_buffer;
				out_info.SubnetMask = mask_buffer;
				return true;
			}
		}
#else
		ifaddrs* interfaces = nullptr;
		if (getifaddrs(&interfaces) != 0)
			return false;

		for (ifaddrs* iface = interfaces; iface != nullptr; iface = iface->ifa_next)
		{
			if (iface->ifa_addr == nullptr || iface->ifa_netmask == nullptr)
				continue;
			if (iface->ifa_addr->sa_family != AF_INET)
				continue;
			if ((iface->ifa_flags & IFF_UP) == 0 || (iface->ifa_flags & IFF_LOOPBACK) != 0)
				continue;

			char ip_buffer[INET_ADDRSTRLEN] = {};
			char mask_buffer[INET_ADDRSTRLEN] = {};
			const sockaddr_in* address = reinterpret_cast<const sockaddr_in*>(iface->ifa_addr);
			const sockaddr_in* mask = reinterpret_cast<const sockaddr_in*>(iface->ifa_netmask);
			if (inet_ntop(AF_INET, &address->sin_addr, ip_buffer, sizeof(ip_buffer)) == nullptr)
				continue;
			if (inet_ntop(AF_INET, &mask->sin_addr, mask_buffer, sizeof(mask_buffer)) == nullptr)
				continue;

			out_info.Address = ip_buffer;
			out_info.SubnetMask = mask_buffer;
			freeifaddrs(interfaces);
			return true;
		}

		freeifaddrs(interfaces);
#endif
		return false;
	}
}
