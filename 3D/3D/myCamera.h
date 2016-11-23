#ifndef MY_CAMERA_H
#define MY_CAMERA_H

#include <windows.h>
#include <gl/gl.h>
#include <gl/glu.h>
#include <gl/glut.h>
#include <cmath>
#include "Vector3.h"

class myCamera
{
public:
	Vector3 camPosition, look, upDir;
	Vector3 dir;	//new added
	Vector3 normAlong;
	Vector3 normForward;
	Vector3 normUp;
	Vector3 newcamPosition;
	Vector3 newLook;
	Vector3 newUp;
	Vector3 along;

	double dirMag;
	double angle;

	myCamera(); // constructor

	void pitch(double p);
	void yaw(double p);
	void roll(double p);
	void slide(double speed, int axis);

};

#endif
