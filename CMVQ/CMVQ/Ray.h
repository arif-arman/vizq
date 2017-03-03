#ifndef RAY_H
#define RAY_H

#include "Vector3d.h"
#include "Coordinate.h"

class Ray{

public:
	
	Vector3d direction;
	Coordinate origin;
	int intesects;

	Ray(){
		intesects = -1;
	}

	Ray(Coordinate or, Vector3d dir)
	{
		origin = or; 
		direction = dir;
		intesects = -1;
	}

	Coordinate getPointAtDistance(double distance)
	{
		Coordinate endPoint = this->origin;
		endPoint =endPoint + direction*distance;
		return endPoint;
	}

};

#endif