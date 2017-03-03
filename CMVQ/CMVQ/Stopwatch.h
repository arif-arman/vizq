#ifndef STOPWATCH_H
#define STOPWATCH_H

#include <time.h>

class Stopwatch {

public:
	void start() { c1 = clock(); t1 = time(0); };
	void stop() { c2 = clock(); t2 = time(0); };
	int getDiff() {
		return (c2 - c1);
	};

private:
	time_t t1, t2;
	clock_t c1, c2;
};

#endif