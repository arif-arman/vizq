
#ifndef VISIBLEREGION_H
#define VISIBLEREGION_H


#include "Coordinate.h"
#include "MBR.h"

class VisibleRegion{

public:

	Coordinate a, b, c;
	//c is the querypoint

	VisibleRegion(){}
	VisibleRegion(Coordinate a,Coordinate b,Coordinate c){
		this->a = a;
		this->b = b;
		this->c = c;
	}

	void draw(){
		glPushMatrix(); {

			glBegin(GL_LINES); {
				glVertex3f(a.x, a.y, a.z);
				glVertex3f(b.x, b.y, b.z);

				glVertex3f(c.x, c.y, c.z);
				glVertex3f(b.x, b.y, b.z);

				glVertex3f(c.x, c.y, c.z);
				glVertex3f(a.x, a.y, a.z);
			}glEnd();

		}glPopMatrix();
	}

	MBR getMBR(){
	
		double minX = (a.x < b.x) ? a.x : b.x;
		minX = (minX < c.x) ? minX : c.x;
		
		double minY = (a.y < b.y) ? a.y : b.y;
		minY = (minY < c.y) ? minY : c.y;

		double maxX = (a.x > b.x) ? a.x : b.x;
		maxX = (maxX > c.x) ? maxX : c.x;

		double maxY = (a.y > b.y) ? a.y : b.y;
		maxY = (maxY > c.y) ? maxY : c.y;

		Coordinate bottomLeft(minX, minY,0);
		Coordinate topRight(maxX, maxY,0);

		MBR mbrOfVR(bottomLeft, topRight);

		return mbrOfVR;
	}

	void print(){
		printf("%g %g 0, %g %g 0, %g %g 0\n",a.x,a.y,b.x,b.y,c.x,c.y);
	}

};
#endif