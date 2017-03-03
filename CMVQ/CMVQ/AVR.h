#ifndef AVR_H

#define AVR_H
#include "gpc.h"

#include <vector>
using namespace std;

class AVR{

public:
	gpc_polygon *avrPolygon;
	vector<int> PVQS;

	AVR(gpc_polygon *poly){
		avrPolygon = poly;
	}


};

#endif // !AVR_H
