// Encapsulates terminal control (colors, cursor, ...)
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "terminal.h"

#include <termios.h>

#include <term.h>
#include <curses.h>

#include <cstdio>


#include <sys/ioctl.h>
#include <sys/types.h>

#include <boost/lexical_cast.hpp>
#include <boost/regex.hpp>
#include <boost/tokenizer.hpp>

#include <fmt/format.h>

namespace rosbag_fancy
{

Terminal::Parser::Parser()
 : m_state(STATE_ESCAPE)
 , m_fgColor(-1)
 , m_bgColor(-1)
 , m_bold(false)
{
}

void Terminal::Parser::parseSetAttributes(const std::string& str)
{
	using Tokenizer = boost::tokenizer<boost::char_separator<char>>;

	boost::char_separator<char> sep(";");
	Tokenizer tok(str, sep);

	for(Tokenizer::iterator it = tok.begin(); it != tok.end(); ++it)
	{
		errno = 0;
		auto endptr = const_cast<char*>(it->c_str());
		int code = strtoul(it->c_str(), &endptr, 10);

		if(errno != 0 || *endptr != 0)
		{
			// Error in specification, break out of here
			m_fgColor = -1;
			m_bgColor = -1;
			return;
		}

		if(code == 0)
		{
			m_fgColor = -1;
			m_bgColor = -1;
		}
		else if(code >= 30 && code <= 37)
			m_fgColor = code - 30;
		else if(code >= 40 && code <= 47)
			m_bgColor = code - 40;
		else if(code == 1)
			m_bold = true;
	}
}

void Terminal::Parser::parse(char c)
{
	switch(m_state)
	{
		case STATE_ESCAPE:
			if(c == '\033')
				m_state = STATE_TYPE;
			break;
		case STATE_TYPE:
			if(c == '[')
			{
				m_state = STATE_CSI;
				m_buf.clear();
			}
			else
				m_state = STATE_ESCAPE;
			break;
		case STATE_CSI:
			if(c == 'm')
			{
				parseSetAttributes(m_buf);
				m_state = STATE_ESCAPE;
			}
			else
			{
				m_buf.push_back(c);
				if(m_buf.length() >= 16)
					m_state = STATE_ESCAPE;
			}
			break;
	}
}

void Terminal::Parser::parse(const std::string& str)
{
	for(char c : str)
		parse(c);
}

void Terminal::Parser::apply(Terminal* term)
{
	if(m_fgColor >= 0 && m_bgColor >= 0)
		term->setSimplePair((SimpleColor)m_fgColor, (SimpleColor)m_bgColor);
	else
	{
		term->setStandardColors();
		if(m_fgColor >= 0)
			term->setSimpleForeground((SimpleColor)m_fgColor);
		else if(m_bgColor >= 0)
			term->setSimpleBackground((SimpleColor)m_fgColor);
	}
}

Terminal::Terminal()
 : m_valid(false)
 , m_256colors(false)
 , m_truecolor(false)
{
	// Override using environment variable
	char* overrideMode = getenv("ROSBAG_FANCY_COLOR_MODE");
	const char* termOverride = nullptr;
	if(overrideMode)
	{
		if(strcasecmp(overrideMode, "truecolor") == 0)
		{
			termOverride = "xterm-256color";
			m_256colors = true;
			m_truecolor = true;
		}
		else if(strcasecmp(overrideMode, "256colors") == 0)
		{
			termOverride = "xterm-256color";
			m_256colors = true;
			m_truecolor = false;
		}
		else if(strcasecmp(overrideMode, "ansi") == 0)
		{
			m_256colors = false;
			m_truecolor = false;
		}
		else
		{
			fmt::print(stderr, "Warning: Unknown ROSBAG_FANCY_COLOR_MODE value: '{}'\n", overrideMode);
		}
	}
	else
	{
		// Detect truecolor-capable terminals
		if(getenv("KONSOLE_DBUS_SESSION"))
		{
			// Sadly, there is no way to determine the Konsole version. Since
			// any reasonably recent version supports true colors, just assume
			// true color support
			m_truecolor = true;
			m_256colors = true;
		}

		char* vte_version = getenv("VTE_VERSION");
		if(vte_version && boost::lexical_cast<unsigned int>(vte_version) >= 3600)
		{
			m_256colors = true;
			m_truecolor = true;
		}
	}

	int ret;
	if(setupterm(termOverride, STDOUT_FILENO, &ret) != OK)
	{
		fmt::print("Could not setup the terminal. Disabling all colors...\n");
		return;
	}

	m_valid = true;

	if(!m_256colors && !overrideMode)
	{
		// Detect 256 color terminals
		int num_colors = tigetnum("colors");
		m_256colors = num_colors >= 256;
	}

	{
		const char* bgColor = tigetstr("setab");
		if(bgColor && bgColor != (char*)-1)
			m_bgColorStr = bgColor;
		else
			fmt::print("Your terminal does not support ANSI background!\n");
	}
	{
		const char* fgColor = tigetstr("setaf");
		if(fgColor && fgColor != (char*)-1)
			m_fgColorStr = fgColor;
		else
			fmt::print("Your terminal does not support ANSI foreground!\n");
	}

	m_opStr = tigetstr("op");
	m_sgr0Str = tigetstr("sgr0");
	m_elStr = tigetstr("el");
	m_upStr = tigetstr("cuu");

	m_boldStr = tigetstr("bold");

	auto registerKey = [&](const char* name, SpecialKey key, const std::string& fallback = ""){
		char* code = tigetstr(name);

		// Who comes up with these return codes?
		if(code && code != reinterpret_cast<char*>(-1))
			m_specialKeys[code] = key;
		else if(!fallback.empty())
			m_specialKeys[fallback] = key;
	};

	// Map function keys
	for(int i = 0; i < 12; ++i)
	{
		registerKey(
			fmt::format("kf{}", i+1).c_str(),
			static_cast<SpecialKey>(SK_F1 + i)
		);
	}

	// Backspace
	registerKey(key_backspace, SK_Backspace);

	// Arrow keys
	registerKey(key_up, SK_Up, "\033[A");
	registerKey(key_down, SK_Down, "\033[B");
	registerKey(key_right, SK_Right, "\033[C");
	registerKey(key_left, SK_Left, "\033[D");
}

bool Terminal::has256Colors() const
{
	return m_256colors;
}

void Terminal::setCursorInvisible()
{
	if(!m_valid)
		return;

	putp(tigetstr("civis"));
}

void Terminal::setCursorVisible()
{
	if(!m_valid)
		return;

	putp(tigetstr("cnorm"));
}

static int ansiColor(uint32_t rgb)
{
	int r = (rgb & 0xFF);
	int g = (rgb >> 8) & 0xFF;
	int b = (rgb >> 16) & 0xFF;

	r = r * 6 / 256;
	g = g * 6 / 256;
	b = b * 6 / 256;

	return 16 + 36 * r + 6 * g + b;
}

void Terminal::setBackgroundColor(uint32_t color)
{
	if(!m_valid)
		return;

	if(m_truecolor)
	{
		char buf[256];
		snprintf(buf, sizeof(buf), "\033[48;2;%d;%d;%dm", color & 0xFF, (color >> 8) & 0xFF, (color >> 16) & 0xFF);
		fputs(buf, stdout);
	}
	else
	{
		char* out = tiparm(m_bgColorStr.c_str(), ansiColor(color));
		putp(out);
	}
}

void Terminal::setForegroundColor(uint32_t color)
{
	if(!m_valid)
		return;

	if(m_truecolor)
	{
		char buf[256];
		snprintf(buf, sizeof(buf), "\033[38;2;%d;%d;%dm", color & 0xFF, (color >> 8) & 0xFF, (color >> 16) & 0xFF);
		fputs(buf, stdout);
	}
	else
	{
		char* out = tiparm(m_fgColorStr.c_str(), ansiColor(color));
		putp(out);
	}
}

void Terminal::setEcho(bool on)
{
	// Switch character echo on
	termios ios;
	if(tcgetattr(STDIN_FILENO, &ios) == 0)
	{
		if(on)
		{
			ios.c_lflag |= ECHO;
			ios.c_lflag |= ICANON;
		}
		else
		{
			ios.c_lflag &= ~ECHO;
			ios.c_lflag &= ~ICANON;
		}

		tcsetattr(STDIN_FILENO, TCSANOW, &ios);
	}
}

void Terminal::setBold(bool on)
{
	if(!m_valid)
		return;

	if(on)
		putp(m_boldStr.c_str());
}

void Terminal::setSimpleForeground(SimpleColor color)
{
	if(!m_valid)
		return;

	char* out = tiparm(m_fgColorStr.c_str(), color);
	putp(out);
}

void Terminal::setSimpleBackground(SimpleColor color)
{
	if(!m_valid)
		return;

	char* out = tiparm(m_bgColorStr.c_str(), color);
	putp(out);
}

void Terminal::setSimplePair(SimpleColor fg, SimpleColor bg)
{
	if(!m_valid)
		return;

	setSimpleForeground(fg);
	setSimpleBackground(bg);
}

void Terminal::setStandardColors()
{
	if(!m_valid)
		return;

	putp(m_opStr.c_str());
	putp(m_sgr0Str.c_str());
}

void Terminal::clearToEndOfLine()
{
	if(!m_valid)
		return;

	putp(m_elStr.c_str());
}

void Terminal::moveCursorUp(int numLines)
{
	if(!m_valid)
		return;

	putp(tparm(m_upStr.c_str(), numLines));
}

void Terminal::moveCursorToStartOfLine()
{
	putchar('\r');
}

bool Terminal::getSize(int* outColumns, int* outRows)
{
	struct winsize w;
	if(ioctl(STDIN_FILENO, TIOCGWINSZ, &w) == 0)
	{
		*outColumns = w.ws_col;
		*outRows = w.ws_row;
		return true;
	}
	else
		return false;
}

void Terminal::setWindowTitle(const std::string& title)
{
	char buf[256];

	// Konsole style
	snprintf(buf, sizeof(buf), "\033]30;%s\007", title.c_str());
	fputs(buf, stdout);

	// screen/tmux style
	snprintf(buf, sizeof(buf), "\033k%s\033\\", title.c_str());
	fputs(buf, stdout);
}

void Terminal::clearWindowTitle(const std::string& backup)
{
	fputs("\033]30;%d : %n\007", stdout);

	// screen/tmux style
	fmt::print("\033k{}\033\\", backup);
}

int Terminal::readKey()
{
	// Are we currently aborting an escape string that we did not recognize?
	if(m_currentEscapeAborted)
	{
		char c = m_currentEscapeStr[m_currentEscapeAbortIdx++];
		if(m_currentEscapeAbortIdx >= m_currentEscapeStr.size())
		{
			m_currentEscapeAborted = false;
			m_currentEscapeStr.clear();
		}

		return c;
	}

	char c;
	if(read(STDIN_FILENO, &c, 1) != 1)
		return -1;

	if(m_currentEscapeStr.empty() && c == '\E')
	{
		m_currentEscapeStr.push_back(c);
		m_escapeStartTime = std::chrono::steady_clock::now();
		return -1;
	}
	else if(!m_currentEscapeStr.empty())
	{
		m_currentEscapeStr.push_back(c);
		m_escapeStartTime = std::chrono::steady_clock::now();

		std::size_t matches = 0;
		int lastMatch = -1;
		bool completeMatch = false;

		for(auto& pair : m_specialKeys)
		{
			if(m_currentEscapeStr.length() > pair.first.length())
				continue;

			if(pair.first.substr(0, m_currentEscapeStr.length()) == m_currentEscapeStr)
			{
				matches++;
				lastMatch = pair.second;

				if(m_currentEscapeStr.length() == pair.first.length())
				{
					completeMatch = true;
					break;
				}
			}
		}

		if(matches == 0)
		{
			// We don't understand this code, just switch back to normal mode
			m_currentEscapeStr.clear();
		}
		else if(completeMatch)
		{
			m_currentEscapeStr.clear();
			return lastMatch;
		}
		else
			return -1;
	}

	if(c == 0x7f) // ASCII delete
		return SK_Backspace;

	return c;
}

}
