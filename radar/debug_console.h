#pragma once

#include <imgui.h>

#include <deque>
#include <istream>
#include <ostream>
#include <streambuf>
#include <string>
#include <mutex>
#include <vector>

class DebugConsole
{
public:
	DebugConsole(ImFont* font = nullptr);

	std::istream& getInput();
	std::ostream& getOutput();

	void log(const std::string& text);
	void draw(const char* title, const ImVec2& size);

	void showCopy(bool show);
	void showOptions(bool show);
	void showClear(bool show);

	void setFont(ImFont* font);

private:
	class ConsoleStreamBuf : public std::streambuf
	{
	public:
		explicit ConsoleStreamBuf(DebugConsole& owner);

	protected:
		std::streamsize xsputn(const char* text, std::streamsize count) override;
		int overflow(int ch) override;
		int sync() override;

	private:
		DebugConsole& m_owner;
	};

	class ConsoleInputBuf : public std::streambuf
	{
	public:
		explicit ConsoleInputBuf(DebugConsole& owner);

		void pushLine(std::string line);

	protected:
		int underflow() override;
		std::streamsize showmanyc() override;
		int uflow() override;
		std::streamsize xsgetn(char* text, std::streamsize count) override;

	private:
		bool ensureBuffer();

	private:
		DebugConsole& m_owner;
		std::string m_buffer;
		std::size_t m_offset = 0;
	};

	void append(const char* text, size_t length);
	void flushPendingLine();
	void flushPendingLineUnlocked();
	void appendLine(const std::string& text);
	void appendLineUnlocked(const std::string& text);
	void pushInputLine(const std::string& text);
	void clear();

private:
	char m_input_buf[256] = {};
	std::vector<std::string> m_items;
	ImGuiTextFilter m_filter;
	bool m_auto_scroll = true;
	bool m_scroll_to_bottom = false;
	bool m_copy_to_clipboard = false;
	std::string m_pending_line;
	std::deque<std::string> m_input_lines;
	std::mutex m_mutex;
	ConsoleStreamBuf m_stream_buf;
	ConsoleInputBuf m_input_buf_stream;
	std::istream m_input_stream;
	std::ostream m_stream;
	ImFont* m_font;

	bool m_show_copy = false;
	bool m_show_options = false;
	bool m_show_clear = false;
};
