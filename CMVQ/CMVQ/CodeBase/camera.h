#include <windows.h>
#include <gl/gl.h>
#include <gl/glu.h>
//#include <gl/glut.h>
#include <cmath>
#include "Vector3.h"

class Camera
{
public:

	// eye is position of camera
	// look position we are looking  at 
	// up is up direction of camera - z axis initially
	// n is direction of we are looking at
	// u is cross of up and n
	// v is cross of n and u actually unit vector in up direction


	Vector3 eye, look;
	Vector3 u, v, n, up;
	
	double viewAngle, aspect, nearDist, farDist;


	double cameraAngle;			//in radian
	double cameraAngleDelta;

	double cameraHeight;	
	double cameraRadius;

	Camera(); // constructor

	void set(Vector3, Vector3, Vector3); // like gluLookAt()
	void set(double, double, double, double, double, double, double, double, double);
	void rotate(Vector3, double);
	void roll(double); // roll it
	void pitch(double); // increase pitch
	void yaw(double); // yaw it
	void slide(double,double,double); // slide it
	
	void rotateclock(double);
	void increaseheight(double);
	void increaseradius(double);

};
