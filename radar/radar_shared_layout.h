#pragma once

#include <cstdint>

namespace radaripc
{
	inline constexpr wchar_t kSharedMemoryNameW[] = L"Local\\RadarPointCloudSharedMemory";
	inline constexpr char kSharedMemoryNameA[] = "Local\\RadarPointCloudSharedMemory";

	inline constexpr uint32_t kMagic = 0x52444350u;
	inline constexpr uint32_t kVersion = 1;

	inline constexpr uint32_t kMaxPoints = 65536;
	inline constexpr uint32_t kMaxTargets = 256;

	enum StreamState : uint32_t
	{
		StreamState_Stopped = 0,
		StreamState_Receiving = 1,
		StreamState_Paused = 2,
		StreamState_Error = 3,
	};

#pragma pack(push, 1)

	struct SharedHeader
	{
		uint32_t Magic;
		uint32_t Version;
		uint32_t HeaderSize;
		uint32_t TotalSize;

		// Seqlock:
		// odd  = writer is writing
		// even = stable frame
		uint32_t Sequence;

		uint32_t State;
		uint32_t FrameCount;
		uint32_t PacketSize;

		uint32_t PointCount;
		uint32_t TargetCount;
		uint32_t MaxPoints;
		uint32_t MaxTargets;

		uint64_t TimestampNs;

		uint32_t PointsOffset;
		uint32_t TargetsOffset;

		uint32_t Reserved[16];
	};

	struct SharedPoint
	{
		float x;
		float y;
		float z;
		float doppler;
		float power;
		int32_t targetId;
	};

	struct SharedTarget
	{
		float x;
		float y;
		uint32_t status;
		uint32_t targetId;

		float minx;
		float maxx;
		float miny;
		float maxy;
		float minz;
		float maxz;
	};

#pragma pack(pop)

	inline constexpr uint32_t kHeaderSize = static_cast<uint32_t>(sizeof(SharedHeader));
	inline constexpr uint32_t kPointSize = static_cast<uint32_t>(sizeof(SharedPoint));
	inline constexpr uint32_t kTargetSize = static_cast<uint32_t>(sizeof(SharedTarget));

	inline constexpr uint32_t kPointsOffset = kHeaderSize;
	inline constexpr uint32_t kTargetsOffset = kPointsOffset + kMaxPoints * kPointSize;
	inline constexpr uint32_t kTotalSize = kTargetsOffset + kMaxTargets * kTargetSize;

	static_assert(sizeof(SharedHeader) == 128, "SharedHeader size must be 128 bytes");
	static_assert(sizeof(SharedPoint) == 24, "SharedPoint size must be 24 bytes");
	static_assert(sizeof(SharedTarget) == 40, "SharedTarget size must be 40 bytes");
}