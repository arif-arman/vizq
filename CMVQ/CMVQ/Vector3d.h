#ifndef VECTOR3D_H
#define VECTOR3D_H

#define CRT_SECURE_NO_WARNINGS
#include <stdio.h>
#include <math.h>


class Vector3d{


public:
	double x, y, z;

	Vector3d(double x,double y, double z){
		this->x = x;
		this->y = y;
		this->z = z;
	}

	Vector3d(Vector3d point1, Vector3d point2)
	{
		x = point2.x - point1.x;
		y = point2.y - point1.y;
		z = point2.z - point1.z;
	}

	Vector3d(){
	}

	Vector3d(Vector3d origin,double t,Vector3d dir){
		x = origin.x + t*dir.x;
		y = origin.y + t*dir.y;
		z = origin.z + t*dir.z;
	}

	void print(){
		printf("Vector\n");
		printf("%lf,%lf,%lf\n",x,y,z);
	}
	
	double dot(Vector3d a)
	{
		return this->x*a.x + this->y*a.y + this->z*a.z;
	}

	Vector3d cross(Vector3d a)
	{
		Vector3d v;
		v.x = this->y*a.z - this->z*a.y;
		v.y = this->z*a.x - this->x*a.z;
		v.z = this->x*a.y - this->y*a.x;
		return v;
	}

	void input()
	{
		scanf_s("%lf %lf %lf",&x,&y,&z);
	}
	
	Vector3d normalize()
	{
		Vector3d v;
		double modV = x*x + y*y + z*z;
		modV = sqrt(modV);

		v.x = x / modV;
		v.y = y / modV;
		v.z = z / modV;
		return v;
	}
	Vector3d operator+(const Vector3d v)
	{
		Vector3d newv;
		newv.x = this->x + v.x;
		newv.y = this->y + v.y;
		newv.z = this->z + v.z;

		return newv;
	}

	Vector3d operator-(const Vector3d v)
	{
		Vector3d newv;
		newv.x = this->x - v.x;
		newv.y = this->y - v.y;
		newv.z = this->z - v.z;
		return newv;
	}

	Vector3d operator=(const Vector3d v)
	{
		//Vector3d newv;
		x = v.x;
		y = v.y;
		z = v.z;
		return (*this);
	}
	
	Vector3d operator*(double d)
	{
		Vector3d newv;
		newv.x = x*d;
		newv.y = y*d;
		newv.z = z*d;
		return newv;
	}

	double distance(Vector3d v)
	{
		double d = (x - v.x)*(x - v.x) + (y - v.y)*(y - v.y) + (z - v.z)*(z - v.z);
		return sqrt(d);
	}
};

#endif