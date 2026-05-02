#pragma once

#include "radar_shared_layout.h"
#include "retina.h"

#define NOMINMAX
#include <Windows.h>

#include <mutex>
#include <string>

class RadarSharedMemory
{
public:
	RadarSharedMemory();
	~RadarSharedMemory();

	RadarSharedMemory(const RadarSharedMemory&) = delete;
	RadarSharedMemory& operator=(const RadarSharedMemory&) = delete;

	bool create(std::string* out_error = nullptr);
	void close();

	bool isOpen() const;

	void publish(const retina::Frame& frame, radaripc::StreamState state);
	void publishEmpty(radaripc::StreamState state);

private:
	radaripc::SharedHeader* header();
	radaripc::SharedPoint* points();
	radaripc::SharedTarget* targets();

	void initializeHeader();

private:
	HANDLE m_mapping = nullptr;
	unsigned char* m_view = nullptr;
	mutable std::mutex m_mutex;
};