#ifndef COORDINATE_H
#define COORDINATE_H

#include "Vector3d.h"

class Coordinate{
	
public:

	double x, y, z;

	Coordinate(){
		x = 0;
		y = 0;
		z = 0;
	
	}
	Coordinate(double x,double y,double z=0){
		this->x = x;
		this->y = y;
		this->z = z;
	}

	void scan(){
		scanf("%lf %lf %lf", &x, &y, &z);
	}
	Coordinate operator+(const Vector3d v)
	{
		Coordinate newCoord;
		newCoord.x = this->x + v.x;
		newCoord.y = this->y + v.y;
		newCoord.z = this->z + v.z;

		return newCoord;
	}
};

#endif