#ifndef MBR_H
#define MBR_H

#include "Coordinate.h"
#include "Rectangle.h"
#include <GL\glut.h>
class MBR{

public:

	Coordinate bottomLeft, topRight;
	Coordinate topLeft, bottomRight;
	MBR(){}
	MBR(Coordinate bottomLeft,Coordinate topRight){
		this->bottomLeft = bottomLeft;
		this->topRight = topRight;

		computeOtherPoints();

	}
	MBR(float *mbr){
		bottomLeft.x = mbr[0];
		topRight.x= mbr[1];
		bottomLeft.y= mbr[2];
		topRight.y = mbr[3];
		bottomLeft.z = mbr[4];
		topRight.z = mbr[5];

		computeOtherPoints();
	}


	void draw(){
		glPushMatrix(); {
			glBegin(GL_LINES); {
				glVertex3f(bottomLeft.x,bottomLeft.y,bottomLeft.z);
				glVertex3f(topLeft.x, topLeft.y, topLeft.z);

				glVertex3f(topRight.x, topRight.y, topRight.z);
				glVertex3f(topLeft.x, topLeft.y, topLeft.z);

				glVertex3f(topRight.x, topRight.y, topRight.z);
				glVertex3f(bottomRight.x, bottomRight.y, bottomRight.z);

				glVertex3f(bottomLeft.x, bottomLeft.y, bottomLeft.z);
				glVertex3f(bottomRight.x, bottomRight.y, bottomRight.z);
			}glEnd();
		}glPopMatrix();
	}

	void readMBR(){
		
		scanf("%lf %lf %lf %lf %lf %lf", &bottomLeft.x, &bottomLeft.y, &bottomLeft.z, &topRight.x, &topRight.y, &topRight.z);
		computeOtherPoints();
	}

	void computeOtherPoints(){
		topLeft.x = bottomLeft.x;
		topLeft.y = topRight.y;
		topLeft.z = 0;

		bottomRight.x = topRight.x;
		bottomRight.y = bottomLeft.y;
		bottomRight.z = 0;
	}

	SomeRandomRectangle getRectFromMBR(){
		
		SomeRandomRectangle rect(topLeft,bottomRight);
		return rect;
	}

};

#endif