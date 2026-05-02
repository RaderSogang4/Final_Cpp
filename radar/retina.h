#pragma once

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <mutex>
#include <ostream>
#include <string>
#include <vector>

#define ASIO_STANDALONE
#include <asio.hpp>

namespace retina
{
	using asio::ip::tcp;

	enum TargetStatus : uint32_t
	{
		Standing = 0,
		Sitting = 1,
		Lying = 2,
		Walking = 4,
	};

	struct Point
	{
		float x = 0.0f;
		float y = 0.0f;
		float z = 0.0f;
		float doppler = 0.0f;
		float power = 0.0f;
		int32_t targetId = -1;
	};

	struct Target
	{
		float x = 0.0f;
		float y = 0.0f;
		TargetStatus status = TargetStatus::Standing;
		uint32_t targetId = 0;

		float minx = 0.0f;
		float maxx = 0.0f;
		float miny = 0.0f;
		float maxy = 0.0f;
		float minz = 0.0f;
		float maxz = 0.0f;
	};

	struct Frame
	{
		uint32_t packetSize = 0;
		uint32_t frameCount = 0;
		std::vector<Point> points;
		std::vector<Target> targets;
	};

	static const char* to_string(retina::TargetStatus status)
	{
		switch (status)
		{
			case retina::TargetStatus::Standing: return "Standing";
			case retina::TargetStatus::Sitting: return "Sitting";
			case retina::TargetStatus::Lying: return "Lying";
			case retina::TargetStatus::Walking: return "Walking";
			default: return "Unknown";
		}
	}

	class DeviceFinder
	{
	public:
		DeviceFinder(
			asio::io_context& io,
			std::ostream& log_stream,
			const std::atomic_bool* cancel_flag = nullptr);

		bool find(
			const std::string& local_ip,
			const std::string& subnet_mask,
			std::string& out_host);

	private:
		bool isCancelled() const;

		bool buildScanRange(
			const std::string& local_ip,
			const std::string& subnet_mask,
			uint32_t& out_network,
			uint32_t& out_first,
			uint32_t& out_last);

		bool probeHost(const std::string& ip);
		bool probeTcpPort(const std::string& ip, uint16_t port, std::chrono::milliseconds timeout);
		bool probeHttpPort(const std::string& ip, std::chrono::milliseconds timeout);
		std::string formatAddress(uint32_t address) const;

	private:
		asio::io_context& m_io;
		std::ostream& m_log;
		const std::atomic_bool* m_cancel_flag = nullptr;
	};

	class DeviceClient
	{
	public:
		static constexpr uint32_t frame_count_limit_default = 50;

		DeviceClient(asio::io_context& io, std::string host, std::ostream& log_stream);
		~DeviceClient();

		void close();

		void setFrameCountLimit(uint32_t limit);

		void readFrames(std::function<void(const std::vector<Frame>&)> callback);
		bool readLatestFrame(Frame& out_frame);

	private:
		void doRead();
		bool tryExtractPacket(std::vector<uint8_t>& stream_buf, std::vector<uint8_t>& out_packet_buf);
		bool parseSinglePacket(const std::vector<uint8_t>& packet_buf, Frame& out_frame);
		bool findPacketMagic(const std::vector<uint8_t>& buffer, size_t& out_offset);

	private:
		asio::io_context& m_io;
		std::ostream& m_log;

		tcp::resolver m_resolver;
		tcp::socket m_socket;
		uint32_t m_frame_count_limit;

		std::vector<Frame> m_frames;

		std::array<uint8_t, 4096> m_read_buf {};
		std::vector<uint8_t> m_stream_buf;
		std::vector<uint8_t> m_packet_buf;

		std::mutex m_mutex;
		std::atomic_bool m_closed = false;
	};
}