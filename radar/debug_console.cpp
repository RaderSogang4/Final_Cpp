#include "debug_console.h"

#include <algorithm>
#include <cctype>
#include <cstdio>

namespace
{
	int Stricmp(const char* left, const char* right)
	{
		int diff = 0;
		while ((diff = std::toupper(*right) - std::toupper(*left)) == 0 && *left)
		{
			++left;
			++right;
		}
		return diff;
	}

	void Strtrim(char* text)
	{
		char* end = text + std::strlen(text);
		while (end > text && end[-1] == ' ')
			--end;
		*end = 0;
	}
}

DebugConsole::ConsoleStreamBuf::ConsoleStreamBuf(DebugConsole& owner) :
	m_owner(owner)
{
}

DebugConsole::ConsoleInputBuf::ConsoleInputBuf(DebugConsole& owner) :
	m_owner(owner)
{
	setg(nullptr, nullptr, nullptr);
}

void DebugConsole::ConsoleInputBuf::pushLine(std::string line)
{
	if (!line.empty() && line.back() != '\n')
		line.push_back('\n');
	m_owner.pushInputLine(line);
	setg(nullptr, nullptr, nullptr);
}

bool DebugConsole::ConsoleInputBuf::ensureBuffer()
{
	if (gptr() != nullptr && gptr() < egptr())
		return true;

	std::lock_guard<std::mutex> lock(m_owner.m_mutex);
	if (m_owner.m_input_lines.empty())
		return false;

	m_buffer = std::move(m_owner.m_input_lines.front());
	m_owner.m_input_lines.pop_front();
	m_offset = 0;
	char* begin = const_cast<char*>(m_buffer.data());
	setg(begin, begin, begin + m_buffer.size());
	return true;
}

int DebugConsole::ConsoleInputBuf::underflow()
{
	return ensureBuffer() ? traits_type::to_int_type(*gptr()) : traits_type::eof();
}

std::streamsize DebugConsole::ConsoleInputBuf::showmanyc()
{
	return ensureBuffer() ? (egptr() - gptr()) : 0;
}

int DebugConsole::ConsoleInputBuf::uflow()
{
	if (!ensureBuffer())
		return traits_type::eof();
	const int ch = traits_type::to_int_type(*gptr());
	gbump(1);
	return ch;
}

std::streamsize DebugConsole::ConsoleInputBuf::xsgetn(char* text, std::streamsize count)
{
	std::streamsize copied = 0;
	while (copied < count)
	{
		if (!ensureBuffer())
			break;
		const std::streamsize available = egptr() - gptr();
		const std::streamsize chunk = std::min(count - copied, available);
		std::memcpy(text + copied, gptr(), (size_t)chunk);
		gbump((int)chunk);
		copied += chunk;
	}
	return copied;
}

std::streamsize DebugConsole::ConsoleStreamBuf::xsputn(const char* text, std::streamsize count)
{
	m_owner.append(text, (size_t)count);
	return count;
}

int DebugConsole::ConsoleStreamBuf::overflow(int ch)
{
	if (ch != traits_type::eof())
	{
		const char value = (char)ch;
		m_owner.append(&value, 1);
	}
	return ch;
}

int DebugConsole::ConsoleStreamBuf::sync()
{
	m_owner.flushPendingLine();
	return 0;
}

DebugConsole::DebugConsole(ImFont* font) :
	m_stream_buf(*this),
	m_input_buf_stream(*this),
	m_input_stream(&m_input_buf_stream),
	m_stream(&m_stream_buf),
	m_font(font)
{
	std::memset(m_input_buf, 0, sizeof(m_input_buf));
}

std::istream& DebugConsole::getInput()
{
	return m_input_stream;
}

std::ostream& DebugConsole::getOutput()
{
	return m_stream;
}

void DebugConsole::log(const std::string& text)
{
	std::lock_guard<std::mutex> lock(m_mutex);
	appendLineUnlocked(text);
}

void DebugConsole::draw(const char* title, const ImVec2& size)
{
	ImGui::BeginChild(title, size, true);
	ImGui::TextUnformatted(title);
	ImGui::Separator();

	if (ImGui::BeginPopupContextItem("DebugConsoleContext"))
	{
		if (ImGui::MenuItem("Clear"))
			clear();
		ImGui::EndPopup();
	}

	if (m_show_clear && ImGui::Button("Clear"))
		clear();
	if (m_show_clear || m_show_copy || m_show_options)
		ImGui::SameLine();
	if (m_show_copy && ImGui::Button("Copy"))
		m_copy_to_clipboard = true;
	if (m_show_copy || m_show_options)
		ImGui::SameLine();
	if (m_show_options && ImGui::Button("Options"))
		ImGui::OpenPopup("DebugConsoleOptions");

	if (ImGui::BeginPopup("DebugConsoleOptions"))
	{
		ImGui::Checkbox("Auto-scroll", &m_auto_scroll);
		ImGui::EndPopup();
	}

	ImGui::Separator();

	if (m_font != nullptr)
		ImGui::PushFont(m_font);

	const ImGuiStyle& style = ImGui::GetStyle();
	const float input_height = ImGui::GetFrameHeightWithSpacing();
	const float filter_height = ImGui::GetFrameHeightWithSpacing();
	const float footer_height = style.SeparatorSize + style.ItemSpacing.y + input_height + style.ItemSpacing.y + filter_height + style.ItemSpacing.y;

	if (ImGui::BeginChild("DebugConsoleLog", ImVec2(0.0f, -footer_height), ImGuiChildFlags_NavFlattened, ImGuiWindowFlags_NoScrollbar))
	{
		if (ImGui::BeginPopupContextWindow())
		{
			if (ImGui::Selectable("Clear"))
				clear();
			ImGui::EndPopup();
		}

		ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(4.0f, 1.0f));
		if (m_copy_to_clipboard)
			ImGui::LogToClipboard();

		std::vector<std::string> items;
		{
			std::lock_guard<std::mutex> lock(m_mutex);
			items = m_items;
		}

		for (const std::string& item : items)
		{
			if (!m_filter.PassFilter(item.c_str()))
				continue;

			ImVec4 color;
			bool has_color = false;
			if (item.find("[error]") != std::string::npos)
			{
				color = ImVec4(1.0f, 0.4f, 0.4f, 1.0f);
				has_color = true;
			}
			else if (item.rfind("# ", 0) == 0)
			{
				color = ImVec4(1.0f, 0.8f, 0.6f, 1.0f);
				has_color = true;
			}

			if (has_color)
				ImGui::PushStyleColor(ImGuiCol_Text, color);
			ImGui::TextUnformatted(item.c_str());
			if (has_color)
				ImGui::PopStyleColor();
		}

		if (m_copy_to_clipboard)
		{
			ImGui::LogFinish();
			m_copy_to_clipboard = false;
		}

		if (m_scroll_to_bottom || (m_auto_scroll && ImGui::GetScrollY() >= ImGui::GetScrollMaxY()))
			ImGui::SetScrollHereY(1.0f);
		m_scroll_to_bottom = false;
		ImGui::PopStyleVar();
	}

	ImGui::EndChild();
	ImGui::Separator();

	if (m_font != nullptr)
		ImGui::PopFont();

	ImGui::TextUnformatted("> ");
	ImGui::SameLine();
	ImGui::SetNextItemWidth(-FLT_MIN);
	if (ImGui::InputText("##Input", m_input_buf, IM_ARRAYSIZE(m_input_buf), ImGuiInputTextFlags_EnterReturnsTrue))
	{
		char* command = m_input_buf;
		Strtrim(command);
		if (command[0] != 0)
			pushInputLine(command);
		std::strcpy(command, "");
	}

	ImGui::TextUnformatted("Filter:");
	ImGui::SameLine();
	ImGui::SetNextItemWidth(-FLT_MIN);
	m_filter.Draw("##Filter", 0.0f);

	ImGui::SetItemDefaultFocus();

	ImGui::EndChild();
}

void DebugConsole::showCopy(bool show)
{
	m_show_copy = show;
}

void DebugConsole::showOptions(bool show)
{
	m_show_options = show;
}

void DebugConsole::showClear(bool show)
{
	m_show_clear = show;
}

void DebugConsole::setFont(ImFont* font)
{
	m_font = font;
}

void DebugConsole::append(const char* text, size_t length)
{
	std::lock_guard<std::mutex> lock(m_mutex);
	for (size_t i = 0; i < length; ++i)
	{
		const char ch = text[i];
		if (ch == '\r')
			continue;
		if (ch == '\n')
		{
			flushPendingLineUnlocked();
			continue;
		}
		m_pending_line.push_back(ch);
	}
}

void DebugConsole::flushPendingLine()
{
	std::lock_guard<std::mutex> lock(m_mutex);
	flushPendingLineUnlocked();
}

void DebugConsole::flushPendingLineUnlocked()
{
	if (!m_pending_line.empty())
	{
		appendLineUnlocked(m_pending_line);
		m_pending_line.clear();
	}
}

void DebugConsole::appendLine(const std::string& text)
{
	std::lock_guard<std::mutex> lock(m_mutex);
	appendLineUnlocked(text);
}

void DebugConsole::appendLineUnlocked(const std::string& text)
{
	m_items.push_back(text);
	m_scroll_to_bottom = true;
}

void DebugConsole::pushInputLine(const std::string& text)
{
	std::lock_guard<std::mutex> lock(m_mutex);
	m_input_lines.push_back(text);
}

void DebugConsole::clear()
{
	std::lock_guard<std::mutex> lock(m_mutex);
	m_items.clear();
	m_pending_line.clear();
	m_input_lines.clear();
}
