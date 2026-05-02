#pragma once

#include <implot3d.h>

// Getter callback for custom scatter data stored in arbitrary user memory.
// #user points to caller context, #point points to one element in the data buffer.
typedef void (*ImPlot3DScatterGetter)(void* user, const void* point, ImPlot3DPoint* out_point, ImU32* out_color);

typedef int ImPlot3DViewGizmoFlags;

enum ImPlot3DViewGizmoFlags_
{
	ImPlot3DViewGizmoFlags_None = 0,
	ImPlot3DViewGizmoFlags_RotationChanged = 1 << 0,
	ImPlot3DViewGizmoFlags_Reset = 1 << 1,
	ImPlot3DViewGizmoFlags_TranslationChanged = 1 << 2,
	ImPlot3DViewGizmoFlags_ScaleChanged = 1 << 3,
};

namespace ImPlot3D
{
	IMPLOT3D_API void PlotScatter(
		const char* label_id,
		void* user,
		const void* data,
		int count,
		int stride,
		ImPlot3DScatterGetter getter,
		const ImPlot3DSpec& spec = ImPlot3DSpec());

	IMPLOT3D_API void PlotBox(const char* label_id, float minx, float maxx, float miny, float maxy, float minz, float maxz, const ImPlot3DSpec& spec);

	IMPLOT3D_API ImPlot3DViewGizmoFlags ShowViewGizmo(const char* str_id, float size = 96.0f, ImPlot3DQuat* rotation = nullptr);
}
