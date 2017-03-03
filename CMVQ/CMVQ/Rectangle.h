#ifndef FOURSIDEPOLYGON_H
#define FOURSIDEPOLYGON_H
#include "Coordinate.h"

struct SomeRandomRectangle{
public:
	Coordinate upper_left;
	Coordinate lower_right;

	SomeRandomRectangle(){}
	SomeRandomRectangle(Coordinate a, Coordinate b){
		upper_left = a;
		lower_right = b;
	}

};
#endif // !FOURSIDEPOLYGON_H
