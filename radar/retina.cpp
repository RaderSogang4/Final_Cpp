#include "retina.h"

#include <algorithm>
#include <array>
#include <cassert>
#include <atomic>
#include <cstring>
#include <limits>
#include <sstream>
#include <string_view>
#include <thread>

#define CHECK_READ(exp) \
	do \
	{ \
		if (!(exp)) \
		{ \
			m_log << "[error] [parse] failed to read data\n"; \
			return false; \
		} \
	} while (false)

namespace retina
{
	enum
	{
		HTTP_PORT = 8000,
		DEVICE_PORT = 29172,
		PROBE_TIMEOUT_MS = 100
	};

	enum : uint32_t
	{
		PACKET_MAGIC = 0xABCD4321,
	};

	enum : uint64_t
	{
		FRAME_HEADER_SIZE = 8,
		MAX_FRAME_SIZE = 16 * 1024 * 1024,
		FRAME_MAGIC = 0x0807060504030201
	};

	struct RetinaPacketHeaderRaw
	{
		uint32_t _reserved0;
		uint32_t magic;
		uint32_t _reserved1;
		uint32_t _reserved2;
		uint32_t packageSize;
		uint32_t _reserved3;
		uint32_t _reserved4;
		uint32_t _reserved5;
		uint32_t _reserved6;
	};

	struct RetinaFrameHeaderRaw
	{
		uint64_t magic;
		uint32_t frameCount;
		uint32_t targetNumber;
	};

	struct RetinaTargetRaw
	{
		float x;
		float y;
		TargetStatus status;
		uint32_t targetId;
		float reserved0;
		float reserved1;
		float reserved2;
	};

	namespace detail
	{
		uint32_t parseIpv4(const std::string& ip)
		{
			asio::error_code ec;
			const auto address = asio::ip::make_address_v4(ip, ec);
			return ec ? 0u : address.to_uint();
		}

		class ByteReader
		{
		public:
			ByteReader(const uint8_t* data, size_t size) :
				m_data(data),
				m_size(size),
				m_offset(0)
			{
			}

			size_t remaining() const
			{
				return m_size - m_offset;
			}

			size_t offset() const
			{
				return m_offset;
			}

			bool seek(size_t offset)
			{
				if (offset > m_size)
					return false;

				m_offset = offset;
				return true;
			}

			template <typename T>
			bool read(T& out)
			{
				if (m_offset + sizeof(T) > m_size)
					return false;

				std::memcpy(&out, m_data + m_offset, sizeof(T));
				m_offset += sizeof(T);
				return true;
			}

			template <typename T>
			bool peek(T& out)
			{
				if (m_offset + sizeof(T) > m_size)
					return false;

				std::memcpy(&out, m_data + m_offset, sizeof(T));
				return true;
			}

		private:
			const uint8_t* m_data;
			size_t m_size;
			size_t m_offset;
		};
	}

	///////////////////////////////////////////////////////////////////////////////////////////////
	// DeviceFinder

	DeviceFinder::DeviceFinder(
		asio::io_context& io,
		std::ostream& log_stream,
		const std::atomic_bool* cancel_flag) :
		m_io(io),
		m_log(log_stream),
		m_cancel_flag(cancel_flag)
	{
	}

	bool DeviceFinder::isCancelled() const
	{
		return m_cancel_flag != nullptr && m_cancel_flag->load(std::memory_order_acquire);
	}

	bool DeviceFinder::find(
		const std::string& local_ip,
		const std::string& subnet_mask,
		std::string& out_host)
	{
		if (isCancelled())
			return false;

		uint32_t network = 0;
		uint32_t first = 0;
		uint32_t last = 0;

		if (!buildScanRange(local_ip, subnet_mask, network, first, last))
			return false;

		m_log << "[net] local ip: " << local_ip << ", mask: " << subnet_mask << '\n';
		m_log << "[net] scanning range: " << formatAddress(first) << " - " << formatAddress(last) << '\n';

		const uint32_t total_threads = std::thread::hardware_concurrency();
		const uint32_t worker_count = std::clamp(total_threads == 0 ? 8u : total_threads, 1u, 32u);

		std::atomic<uint32_t> next_ip(first);
		std::atomic<bool> found(false);
		std::mutex result_mutex;
		std::vector<std::thread> workers;

		workers.reserve(worker_count);

		for (uint32_t i = 0; i < worker_count; ++i)
		{
			workers.emplace_back([&, first, last]()
				{
					while (!found.load(std::memory_order_acquire) && !isCancelled())
					{
						const uint32_t current = next_ip.fetch_add(1, std::memory_order_relaxed);
						if (current < first || current > last)
							break;

						const std::string ip = formatAddress(current);
						if (!probeHost(ip))
							continue;

						bool expected = false;
						if (!found.compare_exchange_strong(expected, true, std::memory_order_acq_rel))
							break;

						std::lock_guard<std::mutex> lock(result_mutex);
						out_host = ip;
					}
				});
		}

		for (std::thread& worker : workers)
			worker.join();

		if (!out_host.empty())
		{
			m_log << "[net] found candidate device at " << out_host << '\n';
			return true;
		}

		if (!isCancelled())
			m_log << "[error] [net] no retina device found on local subnet\n";

		return false;
	}

	bool DeviceFinder::buildScanRange(
		const std::string& local_ip,
		const std::string& subnet_mask,
		uint32_t& out_network,
		uint32_t& out_first,
		uint32_t& out_last)
	{
		const uint32_t ip = detail::parseIpv4(local_ip);
		const uint32_t mask = detail::parseIpv4(subnet_mask);

		if (ip == 0 || mask == 0)
			return false;

		out_network = ip & mask;

		const uint32_t broadcast = out_network | ~mask;
		out_first = out_network + 1;
		out_last = broadcast > 0 ? broadcast - 1 : broadcast;

		if (out_first > out_last)
			return false;

		return true;
	}

	bool DeviceFinder::probeHost(const std::string& ip)
	{
		if (isCancelled())
			return false;

		const auto timeout = std::chrono::milliseconds(PROBE_TIMEOUT_MS);

		if (!probeTcpPort(ip, DEVICE_PORT, timeout))
			return false;

		if (isCancelled())
			return false;

		if (!probeHttpPort(ip, timeout))
			return false;

		return true;
	}

	bool DeviceFinder::probeTcpPort(
		const std::string& ip,
		uint16_t port,
		std::chrono::milliseconds timeout)
	{
		if (isCancelled())
			return false;

		asio::io_context io;
		asio::error_code ec;
		tcp::endpoint endpoint(asio::ip::make_address(ip, ec), port);

		if (ec)
			return false;

		tcp::socket socket(io);
		asio::steady_timer timer(io);

		bool connected = false;
		bool completed = false;

		timer.expires_after(timeout);
		timer.async_wait([&](const std::error_code& timer_ec)
			{
				if (!timer_ec && !completed)
					socket.cancel();
			});

		socket.async_connect(endpoint, [&](const std::error_code& connect_ec)
			{
				completed = true;
				connected = !connect_ec;
				timer.cancel();
			});

		io.run();

		asio::error_code close_ec;
		socket.close(close_ec);

		return connected && !isCancelled();
	}

	bool DeviceFinder::probeHttpPort(
		const std::string& ip,
		std::chrono::milliseconds timeout)
	{
		if (isCancelled())
			return false;

		asio::io_context io;
		asio::error_code ec;
		tcp::endpoint endpoint(asio::ip::make_address(ip, ec), HTTP_PORT);

		if (ec)
			return false;

		tcp::socket socket(io);
		asio::steady_timer timer(io);
		std::array<char, 256> response {};

		bool saw_http = false;
		bool completed = false;

		timer.expires_after(timeout);
		timer.async_wait([&](const std::error_code& timer_ec)
			{
				if (!timer_ec && !completed)
					socket.cancel();
			});

		socket.async_connect(endpoint, [&](const std::error_code& connect_ec)
			{
				if (connect_ec)
				{
					completed = true;
					timer.cancel();
					return;
				}

				static const std::string request =
					"GET / HTTP/1.1\r\n"
					"Host: radar\r\n"
					"Connection: close\r\n\r\n";

				asio::async_write(socket, asio::buffer(request), [&](const std::error_code& write_ec, std::size_t)
					{
						if (write_ec)
						{
							completed = true;
							timer.cancel();
							return;
						}

						socket.async_read_some(asio::buffer(response), [&](const std::error_code& read_ec, std::size_t bytes)
							{
								completed = true;
								timer.cancel();

								if (read_ec && read_ec != asio::error::eof)
									return;

								const std::string_view view(response.data(), bytes);
								saw_http = view.find("HTTP/") != std::string_view::npos;
							});
					});
			});

		io.run();

		asio::error_code close_ec;
		socket.close(close_ec);

		return saw_http && !isCancelled();
	}

	std::string DeviceFinder::formatAddress(uint32_t address) const
	{
		return asio::ip::address_v4(address).to_string();
	}

	///////////////////////////////////////////////////////////////////////////////////////////////
	// DeviceClient

	DeviceClient::DeviceClient(
		asio::io_context& io,
		std::string host,
		std::ostream& log_stream) :
		m_io(io),
		m_log(log_stream),
		m_resolver(io),
		m_socket(io),
		m_frame_count_limit(frame_count_limit_default)
	{
		m_resolver.async_resolve(
			host,
			std::to_string(DEVICE_PORT),
			[this, host](const std::error_code& ec, tcp::resolver::results_type endpoints)
			{
				if (m_closed.load(std::memory_order_acquire))
					return;

				m_log << "[net] try resolving " << host << ':' << DEVICE_PORT << "...\n";

				if (ec)
				{
					m_log << "[error] [net] resolve failed: " << ec.message() << '\n';
					return;
				}

				asio::async_connect(
					m_socket,
					endpoints,
					[this](const std::error_code& ec, const tcp::endpoint& endpoint)
					{
						if (m_closed.load(std::memory_order_acquire))
							return;

						if (ec)
						{
							m_log << "[error] [net] connect failed: " << ec.message() << '\n';
							return;
						}

						m_log << "[net] connected to "
							<< endpoint.address().to_string()
							<< ':' << endpoint.port() << '\n';

						doRead();
					});
			});
	}

	DeviceClient::~DeviceClient()
	{
		close();
	}

	void DeviceClient::close()
	{
		m_closed.store(true, std::memory_order_release);

		asio::error_code ec;
		m_resolver.cancel();

		if (m_socket.is_open())
		{
			m_socket.cancel(ec);
			m_socket.close(ec);
		}
	}

	void DeviceClient::setFrameCountLimit(uint32_t limit)
	{
		m_frame_count_limit = limit == 0 ? 1 : limit;
	}

	void DeviceClient::readFrames(std::function<void(const std::vector<Frame>&)> callback)
	{
		if (!callback)
			return;

		std::lock_guard<std::mutex> lock(m_mutex);

		const uint32_t limit = std::max(1u, m_frame_count_limit);
		if (m_frames.size() > limit)
			m_frames.erase(m_frames.begin(), m_frames.end() - limit);

		callback(m_frames);
	}

	bool DeviceClient::readLatestFrame(Frame& out_frame)
	{
		std::lock_guard<std::mutex> lock(m_mutex);

		if (m_frames.empty())
			return false;

		const uint32_t limit = std::max(1u, m_frame_count_limit);
		if (m_frames.size() > limit)
			m_frames.erase(m_frames.begin(), m_frames.end() - limit);

		out_frame = m_frames.back();
		return true;
	}

	void DeviceClient::doRead()
	{
		if (m_closed.load(std::memory_order_acquire))
			return;

		m_socket.async_read_some(
			asio::buffer(m_read_buf),
			[this](const std::error_code& ec, std::size_t bytes)
			{
				if (m_closed.load(std::memory_order_acquire))
					return;

				if (ec)
				{
					m_log << "[error] [net] read failed: " << ec.message() << '\n';
					return;
				}

				auto first = m_read_buf.begin();
				auto last = m_read_buf.begin() + bytes;

				m_stream_buf.insert(m_stream_buf.end(), first, last);

				while (true)
				{
					if (!tryExtractPacket(m_stream_buf, m_packet_buf))
						break;

					Frame frame;
					if (parseSinglePacket(m_packet_buf, frame))
					{
						std::lock_guard<std::mutex> lock(m_mutex);
						m_frames.push_back(std::move(frame));
					}
					else
					{
						m_log << "[error] [parse] failed to parse frame\n";
					}
				}

				doRead();
			});
	}

	bool DeviceClient::tryExtractPacket(
		std::vector<uint8_t>& stream_buf,
		std::vector<uint8_t>& out_packet_buf)
	{
		size_t packet_magic_offset = 0;

		if (!findPacketMagic(stream_buf, packet_magic_offset))
		{
			if (stream_buf.size() > 7)
				stream_buf.erase(stream_buf.begin(), stream_buf.end() - 7);

			return false;
		}

		if (packet_magic_offset < 4)
		{
			stream_buf.erase(stream_buf.begin(), stream_buf.begin() + packet_magic_offset + 4);
			return false;
		}

		if (packet_magic_offset > 4)
			stream_buf.erase(stream_buf.begin(), stream_buf.begin() + packet_magic_offset - 4);

		if (stream_buf.size() < sizeof(RetinaPacketHeaderRaw))
			return false;

		RetinaPacketHeaderRaw packet_header {};
		std::memcpy(&packet_header, stream_buf.data(), sizeof(RetinaPacketHeaderRaw));

		if (packet_header.magic != PACKET_MAGIC)
		{
			stream_buf.erase(stream_buf.begin());
			return false;
		}

		const size_t packet_size =
			sizeof(RetinaPacketHeaderRaw) + static_cast<size_t>(packet_header.packageSize);

		if (packet_header.packageSize == 0 || packet_size > MAX_FRAME_SIZE)
		{
			m_log << "[error] [parse] invalid packet size: " << packet_header.packageSize << '\n';
			stream_buf.erase(stream_buf.begin());
			return false;
		}

		if (stream_buf.size() >= packet_size)
		{
			out_packet_buf.assign(stream_buf.begin(), stream_buf.begin() + packet_size);
			stream_buf.erase(stream_buf.begin(), stream_buf.begin() + packet_size);
			return true;
		}

		return false;
	}

	bool DeviceClient::parseSinglePacket(const std::vector<uint8_t>& packet_buf, Frame& out_frame)
	{
		out_frame = {};

		detail::ByteReader reader(packet_buf.data(), packet_buf.size());

		RetinaPacketHeaderRaw packet_header {};
		RetinaFrameHeaderRaw frame_header {};

		CHECK_READ(reader.read(packet_header));

		if (packet_header.magic != PACKET_MAGIC)
		{
			m_log << "[error] [parse] invalid packet magic\n";
			return false;
		}

		if (packet_header.packageSize != packet_buf.size() - sizeof(RetinaPacketHeaderRaw))
		{
			m_log << "[error] [parse] packet size mismatch\n";
			return false;
		}

		CHECK_READ(reader.read(frame_header));

		if (frame_header.magic != FRAME_MAGIC)
		{
			m_log << "[error] [parse] invalid frame magic: "
				<< std::hex << frame_header.magic << std::dec << '\n';
			return false;
		}

		const uint32_t point_number = frame_header.targetNumber;
		uint32_t target_number = 0;

		out_frame.packetSize = packet_header.packageSize;
		out_frame.frameCount = frame_header.frameCount;

		out_frame.points.reserve(point_number);

		for (uint32_t i = 0; i < point_number; ++i)
		{
			Point p {};

			CHECK_READ(reader.read(p.x));
			CHECK_READ(reader.read(p.y));
			CHECK_READ(reader.read(p.z));
			CHECK_READ(reader.read(p.doppler));
			CHECK_READ(reader.read(p.power));

			p.targetId = -1;
			out_frame.points.push_back(p);
		}

		for (uint32_t i = 0; i < point_number; ++i)
			CHECK_READ(reader.read(out_frame.points[i].targetId));

		if (reader.remaining() == 0)
			return true;

		CHECK_READ(reader.peek(frame_header));

		const bool has_target = frame_header.magic != FRAME_MAGIC;

		if (has_target)
		{
			CHECK_READ(reader.seek(48056));
			CHECK_READ(reader.read(frame_header));

			if (frame_header.magic != FRAME_MAGIC)
			{
				m_log << "[error] [parse] invalid frame magic for target section: "
					<< std::hex << frame_header.magic << std::dec << '\n';
				return false;
			}

			if (frame_header.frameCount != out_frame.frameCount)
			{
				m_log << "[error] [parse] target frame count mismatch\n";
				return false;
			}

			target_number = frame_header.targetNumber;
			out_frame.targets.reserve(target_number);

			for (uint32_t i = 0; i < target_number; ++i)
			{
				RetinaTargetRaw target_raw {};
				CHECK_READ(reader.read(target_raw));

				Target target {};
				target.minx = target.miny = target.minz = std::numeric_limits<float>::max();
				target.maxx = target.maxy = target.maxz = std::numeric_limits<float>::lowest();

				target.x = target_raw.x;
				target.y = target_raw.y;
				target.status = target_raw.status;
				target.targetId = target_raw.targetId;

				bool has_points = false;

				for (const auto& p : out_frame.points)
				{
					if (p.targetId == static_cast<int32_t>(target.targetId))
					{
						has_points = true;
						target.minx = std::min(target.minx, p.x);
						target.maxx = std::max(target.maxx, p.x);
						target.miny = std::min(target.miny, p.y);
						target.maxy = std::max(target.maxy, p.y);
						target.minz = std::min(target.minz, p.z);
						target.maxz = std::max(target.maxz, p.z);
					}
				}

				if (!has_points)
				{
					target.minx = target.maxx = target.x;
					target.miny = target.maxy = target.y;
					target.minz = target.maxz = 0.0f;
				}

				out_frame.targets.push_back(target);
			}
		}

		return true;
	}

	bool DeviceClient::findPacketMagic(const std::vector<uint8_t>& buffer, size_t& out_offset)
	{
		if (buffer.size() < 4)
			return false;

		for (size_t i = 0; i + 4 <= buffer.size(); ++i)
		{
			uint32_t value = 0;
			std::memcpy(&value, buffer.data() + i, sizeof(value));

			if (value == PACKET_MAGIC)
			{
				out_offset = i;
				return true;
			}
		}

		return false;
	}
}