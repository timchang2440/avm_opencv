#pragma once

#include <iostream>
#include <chrono>

using namespace std;
using namespace std::chrono;

class LogProcessTime
{
public:
	LogProcessTime(string preFixTitle)
	{
		_preFixTitle = preFixTitle;
		_start = steady_clock::now();
	}

	~LogProcessTime()
	{
		_end = steady_clock::now();
		auto elapsed = duration_cast<std::chrono::milliseconds>(_end - _start).count();

		char buffer[100];
#ifdef WINDOWS
		sprintf_s(buffer,100,  "%s %llu ms", _preFixTitle.c_str(), elapsed);
#else
		sprintf(buffer, "%s %lu ms", _preFixTitle.c_str(), elapsed);
#endif // WINDOWS

		cout << buffer << endl;
	}

private:
	std::chrono::steady_clock::time_point _start;
	std::chrono::steady_clock::time_point _end;
	
	string _preFixTitle = "";
};