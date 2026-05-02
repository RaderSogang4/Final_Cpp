#include "implot3d_custom.h"

#include <imgui.h>
#include <imgui_internal.h>
#include <algorithm>
#include <array>
#include <cmath>

#include "..\thirdparty\implot3d\implot3d_internal.h"

namespace ImPlot3D
{
	namespace
	{
		constexpr float kViewGizmoMargin = 24.0f;
		constexpr float kViewGizmoArrowLength = 60.0f;
		constexpr float kViewGizmoRingRadius = 65.0f;
		constexpr float kViewGizmoBackgroundRadius = 65.0f;
		constexpr float kViewGizmoDiscRadius = 15.f;
		constexpr float kViewGizmoArrowHoverDistance = 12.0f;
		constexpr float kViewGizmoRingHoverDistance = 10.0f;
		constexpr float kViewGizmoTranslateSensitivity = 0.01f;
		constexpr float kViewGizmoRotateSensitivity = 0.01f;
		constexpr float kViewGizmoScaleSensitivity = 0.08f;
		constexpr float kViewGizmoZoomSensitivity = 0.08f;

		enum GizmoPart
		{
			GizmoPart_None = -1,
			GizmoPart_XArrow = 0,
			GizmoPart_YArrow = 1,
			GizmoPart_ZArrow = 2,
			GizmoPart_XRing = 3,
			GizmoPart_YRing = 4,
			GizmoPart_ZRing = 5,
			GizmoPart_Center = 6,
			GizmoPart_Orbit = 7,
		};

		struct GizmoState
		{
			int ActivePart = GizmoPart_None;
			int HoveredPart = GizmoPart_None;
			ImVec2 LastMousePos = ImVec2(0.0f, 0.0f);
			bool HasLastMousePos = false;
			bool CenterPressed = false;

			// Axis Limits
			ImPlot3DPoint Center = ImPlot3DPoint(0.0, 0.0, 0.0);
			bool HasPendingTranslation = false;

			// Rotation
			ImPlot3DQuat Rotation = ImPlot3DQuat(-0.513269, -0.212596, -0.318184, 0.76819);
			bool HasPendingRotation = false;
			bool NeedAnimateRotation = false;

			float Zoom = 1.0f;
			bool HasPendingZoom = false;
		};

		GizmoState& GetGizmoState(ImGuiID id)
		{
			ImGuiStorage* storage = ImGui::GetStateStorage();
			void* ptr = storage->GetVoidPtr(id);
			if (ptr == nullptr)
			{
				GizmoState* state = IM_NEW(GizmoState)();
				storage->SetVoidPtr(id, state);
				return *state;
			}
			return *static_cast<GizmoState*>(ptr);
		}

		ImVec4 AxisColor(int axis)
		{
			switch (axis)
			{
				case 0: return ImVec4(0.93f, 0.26f, 0.30f, 1.0f);
				case 1: return ImVec4(0.40f, 0.82f, 0.19f, 1.0f);
				default: return ImVec4(0.23f, 0.60f, 0.96f, 1.0f);
			}
		}

		ImVec2 toScreen(const ImPlot3DPlot& plot, const ImVec2& center, const ImPlot3DPoint& point)
		{
			ImPlot3DPoint point_pix = plot.Rotation * point;
			point_pix.y *= -1.0f; // Invert y-axis
			point_pix.x += center.x;
			point_pix.y += center.y;

			return ImVec2((float)point_pix.x, (float)point_pix.y);
		}

		ImVec4 AxisColorHovered(const ImVec4& color)
		{
			return ImVec4(
				std::min(color.x + 0.18f, 1.0f),
				std::min(color.y + 0.18f, 1.0f),
				std::min(color.z + 0.18f, 1.0f),
				1.0f);
		}

		float DistanceToSegment(const ImVec2& point, const ImVec2& a, const ImVec2& b)
		{
			const ImVec2 ab(b.x - a.x, b.y - a.y);
			const float length_sq = ab.x * ab.x + ab.y * ab.y;
			
			if (length_sq <= 1.0e-6f)
			{
				const ImVec2 diff(point.x - a.x, point.y - a.y);
				return ImSqrt(diff.x * diff.x + diff.y * diff.y);
			}
			
			const ImVec2 ap(point.x - a.x, point.y - a.y);
			const float t = std::clamp((ap.x * ab.x + ap.y * ab.y) / length_sq, 0.0f, 1.0f);
			const ImVec2 closest(a.x + ab.x * t, a.y + ab.y * t);
			const ImVec2 diff(point.x - closest.x, point.y - closest.y);
			
			return ImSqrt(diff.x * diff.x + diff.y * diff.y);
		}

		ImPlot3DPoint AxisUnit(int axis)
		{
			switch (axis)
			{
				case 0: return ImPlot3DPoint(1.0, 0.0, 0.0);
				case 1: return ImPlot3DPoint(0.0, 1.0, 0.0);
				default: return ImPlot3DPoint(0.0, 0.0, 1.0);
			}
		}

		ImVec2 DrawAxisArrow(const ImPlot3DPlot& plot, ImDrawList* draw_list, const ImVec2& center, const ImPlot3DPoint& dir, const ImVec4& color, float thickness)
		{
			const ImVec2 end = toScreen(plot, center, dir * kViewGizmoArrowLength);
			const ImU32 col = ImGui::ColorConvertFloat4ToU32(color);
			draw_list->AddLine(center, end, col, thickness);
			const ImVec2 delta = ImVec2(end.x - center.x, end.y - center.y);
			const float len = ImLengthSqr(delta) > 0.0f ? ImSqrt(ImLengthSqr(delta)) : 1.0f;
			const ImVec2 tangent(delta.x / len, delta.y / len);
			const ImVec2 normal(-tangent.y, tangent.x);
			const ImVec2 tip0(end.x - tangent.x * 7.0f + normal.x * 3.5f, end.y - tangent.y * 7.0f + normal.y * 3.5f);
			const ImVec2 tip1(end.x - tangent.x * 7.0f - normal.x * 3.5f, end.y - tangent.y * 7.0f - normal.y * 3.5f);
			draw_list->AddTriangleFilled(end, tip0, tip1, col);
			return end;
		}

		void BuildAxisRingPoints(const ImPlot3DPlot& plot, const ImVec2& center, const ImAxis3D& axis, ImVec2* points)
		{
			for (int i = 0; i < 49; ++i)
			{
				const float t = (float)i / 48.0f * 2.0f * IM_PI;
				ImPlot3DPoint local;

				if (axis == ImAxis3D_X)
					local = ImPlot3DPoint(0.0, cos(t), sin(t));
				else if (axis == ImAxis3D_Y)
					local = ImPlot3DPoint(cos(t), 0.0, sin(t));
				else if (axis == ImAxis3D_Z)
					local = ImPlot3DPoint(cos(t), sin(t), 0.0);

				points[i] = toScreen(plot, center, local * kViewGizmoRingRadius);
			}
		}

		void DrawAxisRing(const ImPlot3DPlot& plot, ImDrawList* draw_list, const ImVec2& center, const ImAxis3D& axis, const ImVec4& color, float thickness)
		{
			ImVec2 points[49];
			BuildAxisRingPoints(plot, center, axis, points);
			draw_list->AddPolyline(points, 49, ImGui::ColorConvertFloat4ToU32(color), ImDrawFlags_None, thickness);
		}

		float DistanceToRing(const ImPlot3DPlot& plot, const ImVec2& center, const ImAxis3D& axis, const ImVec2& mouse_pos)
		{
			ImVec2 points[49];
			BuildAxisRingPoints(plot, center, axis, points);
			float best = FLT_MAX;
			for (int i = 1; i < 49; ++i)
				best = std::min(best, DistanceToSegment(mouse_pos, points[i - 1], points[i]));
			return best;
		}

		float SignedAxisDelta(const ImVec2& mouse_delta, const ImVec2& axis_screen)
		{
			const float axis_length_sq = axis_screen.x * axis_screen.x + axis_screen.y * axis_screen.y;
			if (axis_length_sq <= 1.0e-6f)
				return 0.0f;
			return (mouse_delta.x * axis_screen.x + mouse_delta.y * axis_screen.y) / std::sqrt(axis_length_sq);
		}

		ImPlot3DQuat AxisDragRotation(const ImPlot3DQuat& rotation, int axis, float delta)
		{
			return ImPlot3DQuat((double)delta * kViewGizmoRotateSensitivity, rotation * AxisUnit(axis));
		}

		float SignedRingDelta(const ImVec2& center, const ImVec2& mouse_pos, const ImVec2& mouse_delta)
		{
			const ImVec2 radial(mouse_pos.x - center.x, mouse_pos.y - center.y);
			const float radial_len_sq = radial.x * radial.x + radial.y * radial.y;

			if (radial_len_sq <= 1.0e-6f)
				return 0.0f;
			
			const ImVec2 tangent(-radial.y, radial.x);
			const float tangent_len = std::sqrt(tangent.x * tangent.x + tangent.y * tangent.y);
			
			if (tangent_len <= 1.0e-6f)
				return 0.0f;
			
			return (mouse_delta.x * tangent.x + mouse_delta.y * tangent.y) / tangent_len;
		}

		GizmoPart getHoveredGizmoPart(const ImPlot3DPlot& plot, const ImVec2& center, const ImVec2& mouse_pos, bool hovered)
		{
			const ImVec2 center_delta(mouse_pos.x - center.x, mouse_pos.y - center.y);
			const float center_distance = std::sqrt(center_delta.x * center_delta.x + center_delta.y * center_delta.y);
			
			GizmoPart part = GizmoPart_None;
			float best_distance = FLT_MAX;

			if (center_distance <= kViewGizmoDiscRadius)
			{
				best_distance = 0.0f;
				part = GizmoPart_Center;
			}

			for (int axis = 0; axis < 3; ++axis)
			{
				const ImPlot3DPoint axis_dir = AxisUnit(axis);
				const ImVec2 axis_end = toScreen(plot, center, axis_dir * kViewGizmoArrowLength);
				const float arrow_distance = DistanceToSegment(mouse_pos, center, axis_end);
				const float ring_distance = DistanceToRing(plot, center, (ImAxis3D)axis, mouse_pos);

				if (arrow_distance < kViewGizmoArrowHoverDistance && arrow_distance < best_distance)
				{
					best_distance = arrow_distance;
					part = static_cast<GizmoPart>(GizmoPart_XArrow + axis);
				}

				if (ring_distance < kViewGizmoRingHoverDistance && ring_distance < best_distance)
				{
					best_distance = ring_distance;
					part = static_cast<GizmoPart>(GizmoPart_XRing + axis);
				}
			}

			if (hovered && part == GizmoPart_None)
			{
				part = GizmoPart_Orbit;
			}

			return part;
		}
	}

	void PlotScatter(const char* label_id, void* user, const void* data, int count, int stride, ImPlot3DScatterGetter getter,
		const ImPlot3DSpec& spec)
	{
		if (count < 1)
			return;
	
		IM_ASSERT(data != nullptr && "PlotScatter(custom) requires valid data!");
		IM_ASSERT(getter != nullptr && "PlotScatter(custom) requires a valid getter callback!");
		IM_ASSERT(stride > 0 && "PlotScatter(custom) requires stride > 0!");
	
		const unsigned char* bytes = static_cast<const unsigned char*>(data);

		for (int i = 0; i < count; ++i)
		{
			ImPlot3DPoint point;
			ImU32 color = IM_COL32_WHITE;
			getter(user, bytes + (size_t)(spec.Offset + i) * stride, &point, &color);
	
			float xs = static_cast<float>(point.x);
			float ys = static_cast<float>(point.y);
			float zs = static_cast<float>(point.z);
	
			ImPlot3DSpec point_spec = spec;
			point_spec.Offset = 0;
			point_spec.Stride = sizeof(float);
			point_spec.MarkerLineColor = ImGui::ColorConvertU32ToFloat4(color);
			point_spec.MarkerFillColor = ImGui::ColorConvertU32ToFloat4(color);
	
			PlotScatter(label_id, &xs, &ys, &zs, 1, point_spec);
		}
	}

	void PlotBox(const char* label_id, float minx, float maxx, float miny, float maxy, float minz, float maxz, const ImPlot3DSpec& spec)
	{
		float xs[24] = {
			minx, maxx, maxx, minx,
			minx, minx, maxx, maxx,
			minx, minx, minx, minx,
			maxx, maxx, maxx, maxx,
			minx, maxx, maxx, minx,
			minx, minx, maxx, maxx 
		};
		float ys[24] = {
			miny, miny, miny, miny,
			maxy, maxy, maxy, maxy,
			miny, miny, maxy, maxy,
			miny, maxy, maxy, miny,
			miny, miny, maxy, maxy,
			miny, maxy, maxy, miny 
		};
		float zs[24] = {
			minz, minz, maxz, maxz,
			minz, maxz, maxz, minz,
			minz, maxz, maxz, minz,
			minz, minz, maxz, maxz,
			minz, minz, minz, minz,
			maxz, maxz, maxz, maxz 
		};

		ImPlot3D::PlotQuad(label_id, xs, ys, zs, 24, spec);
	}

	ImPlot3DViewGizmoFlags ShowViewGizmo(const char* str_id, float size, ImPlot3DQuat* rotation)
	{
		ImPlot3DContext& gp = *GImPlot3D;

		IM_ASSERT_USER_ERROR(gp.CurrentPlot != nullptr, "ShowViewGizmo() needs to be called between BeginPlot() and EndPlot()!");
		
		const ImGuiID id = ImGui::GetID(str_id);
		ImPlot3DPlot& plot = *gp.CurrentPlot;
		GizmoState& state = GetGizmoState(id);

		ImGui::KeepAliveID(id);

		if (!state.HasPendingRotation && rotation && state.Rotation != *rotation)
		{
			state.Rotation = *rotation;
			state.HasPendingRotation = true;
			state.NeedAnimateRotation = true;
		}

		if (!plot.Initialized)
		{
			state.Center = plot.RangeMin() + (plot.RangeMax() - plot.RangeMin()) * 0.5;
			// state.Scale = ImPlot3DPoint(1.0, 1.0, 1.0);
			state.Rotation = plot.InitialRotation;
		}

		if (state.HasPendingTranslation)
		{
			const double half_x = 0.5 * plot.Axes[0].Range.Size();
			const double half_y = 0.5 * plot.Axes[1].Range.Size();
			const double half_z = 0.5 * plot.Axes[2].Range.Size();

			SetupAxesLimits(
				state.Center.x - half_x,
				state.Center.x + half_x,
				state.Center.y - half_y,
				state.Center.y + half_y,
				state.Center.z - half_z,
				state.Center.z + half_z,
				ImPlot3DCond_Always);

			state.HasPendingTranslation = false;
		}

		if (state.HasPendingRotation)
		{
			SetupBoxRotation(state.Rotation, state.NeedAnimateRotation, ImPlot3DCond_Always);
			state.HasPendingRotation = false;
			state.NeedAnimateRotation = false;

			if (rotation)
			{
				*rotation = state.Rotation;
			}
		}

		if (state.HasPendingZoom)
		{
			const double x_size = plot.Axes[0].Range.Size();
			const double xy_size = plot.Axes[1].Range.Size() / x_size;
			const double xz_size = plot.Axes[2].Range.Size() / x_size;

			SetupBoxScale(state.Zoom, xy_size * state.Zoom, xz_size * state.Zoom);
		}	

		ImDrawList* draw_list = ImGui::GetWindowDrawList();
		const ImGuiIO& io = ImGui::GetIO();

		const ImVec2 plot_pos = ImPlot3D::GetPlotRectPos();
		const ImVec2 plot_size = ImPlot3D::GetPlotRectSize();
		const ImVec2 gizmo_size(size, size);
		const ImVec2 gizmo_min(plot_pos.x + plot_size.x - gizmo_size.x - kViewGizmoMargin, plot_pos.y + kViewGizmoMargin);
		const ImVec2 gizmo_max(gizmo_min.x + gizmo_size.x, gizmo_min.y + gizmo_size.y);
		const ImRect rect(gizmo_min, gizmo_max);
		const ImVec2 center((gizmo_min.x + gizmo_max.x) * 0.5f, (gizmo_min.y + gizmo_max.y) * 0.5f);

		const bool hovered = rect.Contains(io.MousePos);
		const bool was_active = ImGui::GetActiveID() == id;
		const bool active = was_active || (hovered && (ImGui::IsMouseDown(ImGuiMouseButton_Left) || ImGui::IsMouseDown(ImGuiMouseButton_Right)));

		ImPlot3DViewGizmoFlags result_flags = ImPlot3DViewGizmoFlags_None;

		state.HoveredPart = getHoveredGizmoPart(plot, center, io.MousePos, hovered);

		const int visible_hovered_part = state.ActivePart != GizmoPart_None ? state.ActivePart : state.HoveredPart;

		if (hovered && (ImGui::IsMouseClicked(ImGuiMouseButton_Left) || ImGui::IsMouseClicked(ImGuiMouseButton_Right)))
		{
			ImGui::SetActiveID(id, ImGui::GetCurrentWindow());
			state.ActivePart = ImGui::IsMouseClicked(ImGuiMouseButton_Right) ? GizmoPart_Orbit : state.HoveredPart;
			state.LastMousePos = io.MousePos;
			state.HasLastMousePos = true;
			state.CenterPressed = state.ActivePart == GizmoPart_Center;
		}
		if (was_active && !ImGui::IsMouseDown(ImGuiMouseButton_Left) && !ImGui::IsMouseDown(ImGuiMouseButton_Right))
		{
			if (state.ActivePart == GizmoPart_Center && state.CenterPressed && state.HoveredPart == GizmoPart_Center)
			{
				state.Center = ImPlot3DPoint(0.0, 4.75, -0.05);
				// state.Scale = ImPlot3DPoint(1.0, 1.0, 1.0);
				state.HasPendingTranslation = true;
				state.Rotation = ImPlot3DQuat(-0.513269, -0.212596, -0.318184, 0.76819);
				state.HasPendingRotation = true;
				state.Zoom = 1.0f;
				state.HasPendingZoom = true;
				result_flags = (ImPlot3DViewGizmoFlags)(result_flags | ImPlot3DViewGizmoFlags_Reset);
			}

			ImGui::ClearActiveID();
			state.ActivePart = GizmoPart_None;
			state.HasLastMousePos = false;
			state.CenterPressed = false;
		}

		const bool center_hovered = visible_hovered_part == GizmoPart_Center;
		const bool center_active = state.ActivePart == GizmoPart_Center;
		if ((hovered || active) && !center_hovered && !center_active)
		{
			draw_list->AddCircleFilled(center, kViewGizmoBackgroundRadius, ImGui::ColorConvertFloat4ToU32(ImVec4(1.0f, 1.0f, 1.0f, 0.2f)), 64);
			draw_list->AddCircle(center, kViewGizmoBackgroundRadius, ImGui::ColorConvertFloat4ToU32(ImVec4(1.0f, 1.0f, 1.0f, 0.30f)), 64, 1.5f);
		}

		for (int axis = 0; axis < 3; ++axis)
		{
			const bool ring_hovered = visible_hovered_part == GizmoPart_XRing + axis;
			const bool ring_active = state.ActivePart == GizmoPart_XRing + axis;
			const bool arrow_hovered = visible_hovered_part == GizmoPart_XArrow + axis;
			const bool arrow_active = state.ActivePart == GizmoPart_XArrow + axis;
			const ImVec4 base = AxisColor(axis);
			const ImVec4 ring_color = (ring_hovered || ring_active) ? AxisColorHovered(base) : base;
			const ImVec4 arrow_color = (arrow_hovered || arrow_active) ? AxisColorHovered(base) : base;
			DrawAxisRing(plot, draw_list, center, (ImAxis3D)axis, ring_color, ring_active ? 4.0f : ring_hovered ? 3.2f : 2.0f);
			DrawAxisArrow(plot, draw_list, center, AxisUnit(axis), arrow_color, arrow_active ? 4.0f : arrow_hovered ? 3.2f : 2.0f);
		}

		if (center_hovered || center_active)
		{
			draw_list->AddCircleFilled(center, kViewGizmoDiscRadius, IM_COL32(255, 255, 255, 64), 0);
			draw_list->AddCircle(center, kViewGizmoDiscRadius, IM_COL32(255, 255, 255, 110), 0, 1.5f);
		}

		draw_list->AddCircle(center, kViewGizmoDiscRadius, IM_COL32(255, 255, 255, center_hovered || center_active ? 255 : 220), 0, center_active ? 3.0f : center_hovered ? 2.5f : 2.0f);

		// Handle mouse drag
		if (state.ActivePart != GizmoPart_None && state.ActivePart != GizmoPart_Center && state.HasLastMousePos)
		{
			const ImVec2 delta(io.MousePos.x - state.LastMousePos.x, io.MousePos.y - state.LastMousePos.y);
			state.LastMousePos = io.MousePos;

			if (ImLengthSqr(delta) > 0.0f)
			{
				if (state.ActivePart >= GizmoPart_XArrow && state.ActivePart <= GizmoPart_ZArrow)
				{
					const int axis = state.ActivePart - GizmoPart_XArrow;
					const ImVec2 axis_end = toScreen(plot, center, AxisUnit(axis) * kViewGizmoArrowLength);
					const ImVec2 axis_screen(axis_end.x - center.x, axis_end.y - center.y);
					const float signed_delta = SignedAxisDelta(delta, axis_screen);
					state.Center[axis] += signed_delta * kViewGizmoTranslateSensitivity;
					state.HasPendingTranslation = true;
					result_flags = (ImPlot3DViewGizmoFlags)(result_flags | ImPlot3DViewGizmoFlags_TranslationChanged);
				}
				else if (state.ActivePart >= GizmoPart_XRing && state.ActivePart <= GizmoPart_ZRing)
				{
					const int axis = state.ActivePart - GizmoPart_XRing;
					const float signed_delta = SignedRingDelta(center, io.MousePos, delta);
					const ImPlot3DQuat axis_rotation = AxisDragRotation(state.Rotation, axis, signed_delta);
					state.Rotation = axis_rotation * state.Rotation;
					state.Rotation.Normalize();
					state.HasPendingRotation = true;
					result_flags = (ImPlot3DViewGizmoFlags)(result_flags | ImPlot3DViewGizmoFlags_RotationChanged);
				}
				else if (state.ActivePart == GizmoPart_Orbit)
				{
					const ImPlot3DQuat drag_x((double)delta.y * kViewGizmoRotateSensitivity, ImPlot3DPoint(1.0, 0.0, 0.0));
					const ImPlot3DQuat drag_z((double)-delta.x * kViewGizmoRotateSensitivity, ImPlot3DPoint(0.0, 0.0, 1.0));
					state.Rotation = drag_x * state.Rotation * drag_z;
					state.Rotation.Normalize();
					state.HasPendingRotation = true;
					result_flags = (ImPlot3DViewGizmoFlags)(result_flags | ImPlot3DViewGizmoFlags_RotationChanged);
				}
			}
		}

		// Handle mouse wheel
		if ((hovered || was_active) && io.MouseWheel != 0.0f)
		{
			const int wheel_part = state.ActivePart != GizmoPart_None ? state.ActivePart : state.HoveredPart;

			if (wheel_part >= GizmoPart_XArrow && wheel_part <= GizmoPart_ZArrow)
			{
				//const int axis = wheel_part - GizmoPart_XArrow;
				//state.Scale[axis] *= std::max(0.2, 1.0 + io.MouseWheel * kViewGizmoScaleSensitivity);
				//state.Scale[axis] = std::clamp(state.Scale[axis], 0.2, 8.0);
				//state.HasPendingTranslation = true;
				//result_flags = (ImPlot3DViewGizmoFlags)(result_flags | ImPlot3DViewGizmoFlags_ScaleChanged);
			}
			else if (wheel_part >= GizmoPart_XRing && wheel_part <= GizmoPart_ZRing)
			{
				const int axis = wheel_part - GizmoPart_XRing;
				const ImPlot3DQuat axis_rotation = AxisDragRotation(state.Rotation, axis, io.MouseWheel * 16.0f);
				state.Rotation = axis_rotation * state.Rotation;
				state.Rotation.Normalize();
				state.HasPendingRotation = true;
				result_flags = (ImPlot3DViewGizmoFlags)(result_flags | ImPlot3DViewGizmoFlags_RotationChanged);
			}
			else if (wheel_part == GizmoPart_Center)
			{
				const float mag = 1.0f + io.MouseWheel * kViewGizmoZoomSensitivity;
				state.Zoom *= std::max(0.2f, mag);
				state.Zoom = std::clamp(state.Zoom, 0.2f, 8.0f);
				state.HasPendingZoom = true;
				result_flags = (ImPlot3DViewGizmoFlags)(result_flags | ImPlot3DViewGizmoFlags_ScaleChanged);
			}
		}

		return result_flags;
	}
} // namespace ImPlot3D
