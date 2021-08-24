#ifndef TIMESTAMP_H
#define TIMESTAMP_H
/*
Szymon Rusinkiewicz
Princeton University

timestamp.h
Wrapper around system-specific timestamps.

Usage:
	timestamp t1 = now();
	// Do stuff
	timestamp t2 = now();
	float elapsed_sec = t2 - t1;
*/

#include "mathutil.h"

#ifdef _WIN32
 #ifndef WIN32_LEAN_AND_MEAN
  #define WIN32_LEAN_AND_MEAN
 #endif
 #include <limits.h>
 #include <windows.h>
#else
 #include <sys/time.h>
 #include <unistd.h>
#endif


namespace trimesh {


#ifdef _WIN32

struct timestamp { LARGE_INTEGER t; };

static inline double LI2d(const LARGE_INTEGER &li)
{
	// Work around random compiler bugs...
	double d = li.HighPart;
	d *= 65536.0 * 65536.0;
	d += *(unsigned long *)(&(li.LowPart));
	return d;
}

static inline float operator - (const timestamp &t1, const timestamp &t2)
{
	static LARGE_INTEGER PerformanceFrequency;
	static int status = QueryPerformanceFrequency(&PerformanceFrequency);
	if (status == 0) return 1.0f;

	return float((LI2d(t1.t) - LI2d(t2.t)) / LI2d(PerformanceFrequency));
}

static inline timestamp now()
{
	timestamp t;
	QueryPerformanceCounter(&t.t);
	return t;
}

static inline void sleep(unsigned x)
{
	Sleep(x * 1000);
}

static inline void usleep(unsigned x)
{
	Sleep(x / 1000);
}

#else

struct timestamp { timeval t; };
static inline float operator - (const timestamp &t1, const timestamp &t2)
{
	return (float)(t1.t.tv_sec  - t2.t.tv_sec) +
	       1.0e-6f * (t1.t.tv_usec - t2.t.tv_usec);
}

static inline timestamp now()
{
	timestamp t;
	gettimeofday(&t.t, 0);
	return t;
}

#endif


} // namespace trimesh

#endif
