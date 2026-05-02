#include "radar_shared_memory.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstring>

RadarSharedMemory::RadarSharedMemory()
{
}

RadarSharedMemory::~RadarSharedMemory()
{
	close();
}

bool RadarSharedMemory::create(std::string* out_error)
{
	std::lock_guard<std::mutex> lock(m_mutex);

	close();

	m_mapping = CreateFileMappingW(
		INVALID_HANDLE_VALUE,
		nullptr,
		PAGE_READWRITE,
		0,
		radaripc::kTotalSize,
		radaripc::kSharedMemoryNameW);

	if (m_mapping == nullptr)
	{
		if (out_error)
			*out_error = "CreateFileMappingW failed. GetLastError=" + std::to_string(GetLastError());
		return false;
	}

	m_view = static_cast<unsigned char*>(
		MapViewOfFile(
			m_mapping,
			FILE_MAP_ALL_ACCESS,
			0,
			0,
			radaripc::kTotalSize));

	if (m_view == nullptr)
	{
		if (out_error)
			*out_error = "MapViewOfFile failed. GetLastError=" + std::to_string(GetLastError());

		CloseHandle(m_mapping);
		m_mapping = nullptr;
		return false;
	}

	initializeHeader();
	return true;
}

void RadarSharedMemory::close()
{
	if (m_view != nullptr)
	{
		UnmapViewOfFile(m_view);
		m_view = nullptr;
	}

	if (m_mapping != nullptr)
	{
		CloseHandle(m_mapping);
		m_mapping = nullptr;
	}
}

bool RadarSharedMemory::isOpen() const
{
	return m_view != nullptr;
}

radaripc::SharedHeader* RadarSharedMemory::header()
{
	return reinterpret_cast<radaripc::SharedHeader*>(m_view);
}

radaripc::SharedPoint* RadarSharedMemory::points()
{
	return reinterpret_cast<radaripc::SharedPoint*>(m_view + radaripc::kPointsOffset);
}

radaripc::SharedTarget* RadarSharedMemory::targets()
{
	return reinterpret_cast<radaripc::SharedTarget*>(m_view + radaripc::kTargetsOffset);
}

void RadarSharedMemory::initializeHeader()
{
	std::memset(m_view, 0, radaripc::kTotalSize);

	radaripc::SharedHeader* h = header();
	h->Magic = radaripc::kMagic;
	h->Version = radaripc::kVersion;
	h->HeaderSize = radaripc::kHeaderSize;
	h->TotalSize = radaripc::kTotalSize;
	h->Sequence = 0;
	h->State = radaripc::StreamState_Stopped;
	h->FrameCount = 0;
	h->PacketSize = 0;
	h->PointCount = 0;
	h->TargetCount = 0;
	h->MaxPoints = radaripc::kMaxPoints;
	h->MaxTargets = radaripc::kMaxTargets;
	h->TimestampNs = 0;
	h->PointsOffset = radaripc::kPointsOffset;
	h->TargetsOffset = radaripc::kTargetsOffset;
}

void RadarSharedMemory::publishEmpty(radaripc::StreamState state)
{
	retina::Frame empty{};
	publish(empty, state);
}

void RadarSharedMemory::publish(const retina::Frame& frame, radaripc::StreamState state)
{
	std::lock_guard<std::mutex> lock(m_mutex);

	if (!isOpen())
		return;

	radaripc::SharedHeader* h = header();
	radaripc::SharedPoint* out_points = points();
	radaripc::SharedTarget* out_targets = targets();

	std::atomic_ref<uint32_t> sequence(h->Sequence);

	uint32_t seq = sequence.load(std::memory_order_relaxed);
	if ((seq & 1u) != 0u)
		++seq;

	sequence.store(seq + 1u, std::memory_order_release);

	const uint32_t point_count = static_cast<uint32_t>(
		std::min<size_t>(frame.points.size(), radaripc::kMaxPoints));

	const uint32_t target_count = static_cast<uint32_t>(
		std::min<size_t>(frame.targets.size(), radaripc::kMaxTargets));

	for (uint32_t i = 0; i < point_count; ++i)
	{
		const retina::Point& p = frame.points[i];

		out_points[i].x = p.x;
		out_points[i].y = p.y;
		out_points[i].z = p.z;
		out_points[i].doppler = p.doppler;
		out_points[i].power = p.power;
		out_points[i].targetId = p.targetId;
	}

	for (uint32_t i = 0; i < target_count; ++i)
	{
		const retina::Target& t = frame.targets[i];

		out_targets[i].x = t.x;
		out_targets[i].y = t.y;
		out_targets[i].status = static_cast<uint32_t>(t.status);
		out_targets[i].targetId = t.targetId;
		out_targets[i].minx = t.minx;
		out_targets[i].maxx = t.maxx;
		out_targets[i].miny = t.miny;
		out_targets[i].maxy = t.maxy;
		out_targets[i].minz = t.minz;
		out_targets[i].maxz = t.maxz;
	}

	const auto now = std::chrono::system_clock::now().time_since_epoch();
	const uint64_t timestamp_ns =
		static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(now).count());

	h->Magic = radaripc::kMagic;
	h->Version = radaripc::kVersion;
	h->HeaderSize = radaripc::kHeaderSize;
	h->TotalSize = radaripc::kTotalSize;
	h->State = static_cast<uint32_t>(state);
	h->FrameCount = frame.frameCount;
	h->PacketSize = frame.packetSize;
	h->PointCount = point_count;
	h->TargetCount = target_count;
	h->MaxPoints = radaripc::kMaxPoints;
	h->MaxTargets = radaripc::kMaxTargets;
	h->TimestampNs = timestamp_ns;
	h->PointsOffset = radaripc::kPointsOffset;
	h->TargetsOffset = radaripc::kTargetsOffset;

	std::atomic_thread_fence(std::memory_order_release);
	sequence.store(seq + 2u, std::memory_order_release);
}