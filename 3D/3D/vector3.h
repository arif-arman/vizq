#ifndef VECTOR3_H
#define VECTOR3_H

#include<stdio.h>
#include "gpc.h"

using namespace std;
class Vector3
{
public:
	double x;
	double y;
	double z;
    Vector3();
    Vector3(double,double,double);
    Vector3(Vector3&);
	void print();
	void scan();
	void fscan(FILE* fp);
    void set(Vector3);
    void set(double,double,double);
    void setX(double);
    void setY(double);
    void setZ(double);
    double getX();
    double getY();
    double getZ();
    double dotProd(Vector3);
    Vector3 crossProd(Vector3);
	Vector3 add(Vector3);
	Vector3 sub(Vector3);
	Vector3 mul(double scale);
	Vector3 div(double scale);
    void normalize();            
	double mag2();
	bool operator<( const Vector3& v );
	void negate();
};



class Plane
{
public:
	double n1,n2,n3,d0; //plane equation form
	Vector3 n;	// plane normal
	Vector3 v0,v1; //rectangular plane lo up (lowest highest point)

	Plane();
	Plane( double n1,double n2,double n3,double d0 );
	Plane (Vector3 p1,Vector3 p2,Vector3 p3);

	bool caclProjectionOfPoint( Vector3 C, Vector3 P,Vector3& Q  );
	bool rayPlaneIntersction(Vector3 &u,Vector3 &v,Vector3 &I);

};

//class Box;

class Box
{
public:
	Box(){}

	double x[2],y[2],z[2];
	Vector3 v0,v1;

	Vector3 color;

	// 6 planes we are treating them like a box
	Box *bx,*by,*bz;
	Plane px[2],py[2],pz[2];
	

	// 8 corner point
	Vector3 a,b,c,d;
	Vector3 a1,b1,c1,d1;

	//center of the box
	Vector3 o;

	//length of the side
	double lx,ly,lz;
	
	void init(Vector3 &v0,Vector3 &v1,Vector3 &origin);

	Box(Vector3 v0,Vector3 v1);


	void scan();

	void fscan(FILE* fp,Vector3 origin);

	void setPlanes();

	void print();

	void draw();

	bool checkInside(double x, double y, double z);


};


bool commonBox( Box &a,Box &b,Box &c );

#endif