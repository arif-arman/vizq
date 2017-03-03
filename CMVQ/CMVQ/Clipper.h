#ifndef CLIPPER_H
#define CLIPPER_H

#include<iostream>
#include<cstdio>
#include<cstdlib>
#include<cmath>
#include<fstream>
#include "Coordinate.h"
#include "VisibleRegion.h"
#include "Vector3d.h"
#include "QueryPoint.h"

using namespace std;

#include<GL/glut.h>

#define BLACK 0, 0, 0
#define round(a) ((int)(a+0.5))

int V, Pv;
double *Px = 0, *Py = 0, *Pz = 0;
double *X1 = 0, *Y1 = 0, *X2 = 0, *Y2 = 0;
double scaleX = 0, scaleY = 0;
double xMin, xMax, yMin, yMax;
double *poly, *arr;
int k = 0;
double m;
double intX = -1, intY = -1;
QueryPoint *q;

double ccw(Coordinate p1, Coordinate p2, Coordinate p3) {
	return ((p2.x - p1.x) * (p3.y - p1.y) - (p2.y - p1.y) * (p3.x - p1.x));
}

bool intersecting(Coordinate a, Coordinate b, Coordinate c, Coordinate d) {
	Coordinate *i;
	double m1 = (b.y - a.y) / (b.x - a.x);
	double c1 = a.y - m1*a.x;
	double m2 = (d.y - c.y) / (d.x - c.x);
	double c2 = c.y - m2*c.x;
	if (m1 - m2 == 0) {
		return false;
	}
	else {
		intX = (c2 - c1) / (m1 - m2);
		intY = m1*intX + c1;
	}
	cout << "Intersction " << intX << " " << intY << endl;
	return true;

}

void clipRandomEdge(float x1, float y1, float x2, float y2, Coordinate a, Coordinate b) {
	Coordinate *c1 = new Coordinate(x1, y1, 0);
	Coordinate *c2 = new Coordinate(x2, y2, 0);
	double t1 = ccw(a, b, *c1);
	double t2 = ccw(a, b, *c2);
	cout << a.x << " " << a.y << " " << a.z << endl;
	cout << b.x << " " << b.y << " " << b.z << endl;
	cout << x1 << " " << y1 << " " << x2 << " " << y2 << endl;
	cout << t1 << " " << t2 << endl;

	if (x2 - x1)
		m = (y2 - y1) / (x2 - x1);
	else
		m = 100000;
	if (t1 < 0 && t2 >= 0) {
		// keep intersection point only
		bool inters = intersecting(a, b, *c1, *c2);
		if (inters) {
			arr[k] = intX;
			arr[k + 1] = intY;
			k += 2;
		}
	}
	else if (t1 >= 0 && t2 < 0) {
		bool inters = intersecting(a, b, *c1, *c2);
		if (inters) {
			arr[k] = intX;
			arr[k + 1] = intY;
			arr[k + 2] = x2;
			arr[k + 3] = y2;
			k += 4;
		}
	}
	else if (t1 <= 0 && t2 <= 0) {
		arr[k] = x2;
		arr[k + 1] = y2;
		k += 2;
	}
}

void clipl(float x1, float y1, float x2, float y2)
{
	if (x2 - x1)
		m = (y2 - y1) / (x2 - x1);
	else
		m = 100000;
	if (x1 >= xMin && x2 >= xMin)
	{
		arr[k] = x2;
		arr[k + 1] = y2;
		k += 2;
	}
	if (x1 < xMin && x2 >= xMin)
	{
		arr[k] = xMin;
		arr[k + 1] = y1 + m*(xMin - x1);
		arr[k + 2] = x2;
		arr[k + 3] = y2;
		k += 4;
	}
	if (x1 >= xMin  && x2 < xMin)
	{
		arr[k] = xMin;
		arr[k + 1] = y1 + m*(xMin - x1);
		k += 2;
	}
}

void clipt(float x1, float y1, float x2, float y2)
{
	if (y2 - y1)
		m = (x2 - x1) / (y2 - y1);
	else
		m = 100000;
	if (y1 <= yMax && y2 <= yMax)
	{
		arr[k] = x2;
		arr[k + 1] = y2;
		k += 2;
	}
	if (y1 > yMax && y2 <= yMax)
	{
		arr[k] = x1 + m*(yMax - y1);
		arr[k + 1] = yMax;
		arr[k + 2] = x2;
		arr[k + 3] = y2;
		k += 4;
	}
	if (y1 <= yMax  && y2 > yMax)
	{
		arr[k] = x1 + m*(yMax - y1);
		arr[k + 1] = yMax;
		k += 2;
	}
}

void clipr(float x1, float y1, float x2, float y2)
{
	if (x2 - x1)
		m = (y2 - y1) / (x2 - x1);
	else
		m = 100000;
	if (x1 <= xMax && x2 <= xMax)
	{
		arr[k] = x2;
		arr[k + 1] = y2;
		k += 2;
	}
	if (x1 > xMax && x2 <= xMax)
	{
		arr[k] = xMax;
		arr[k + 1] = y1 + m*(xMax - x1);
		arr[k + 2] = x2;
		arr[k + 3] = y2;
		k += 4;
	}
	if (x1 <= xMax  && x2 > xMax)
	{
		arr[k] = xMax;
		arr[k + 1] = y1 + m*(xMax - x1);
		k += 2;
	}
}

void clipb(float x1, float y1, float x2, float y2)
{
	if (y2 - y1)
		m = (x2 - x1) / (y2 - y1);
	else
		m = 100000;
	if (y1 >= yMin && y2 >= yMin)
	{
		arr[k] = x2;
		arr[k + 1] = y2;
		k += 2;
	}
	if (y1 < yMin && y2 >= yMin)
	{
		arr[k] = x1 + m*(yMin - y1);
		arr[k + 1] = yMin;
		arr[k + 2] = x2;
		arr[k + 3] = y2;
		k += 4;
	}
	if (y1 >= yMin  && y2 < yMin)
	{
		arr[k] = x1 + m*(yMin - y1);
		arr[k + 1] = yMin;
		k += 2;
	}
}

void clip() {
	/*
	for (int i = 0; i < 2*Pv;i+=2) {
	//cout << poly[i++] << " " << poly[i++] << endl;
	cout << poly[i] << " " << poly[i + 1] << endl;
	}
	*/
	int i, n;
	int s = 2 * Pv;
	for (i = 0; i < 2 * Pv; i += 2) {
		clipRandomEdge(poly[i], poly[i + 1], poly[(i + 2) % s], poly[(i + 3) % s], q->vr.c, q->vr.a);
	}
	n = k / 2;
	for (i = 0; i < k; i++) poly[i] = arr[i];
	poly[i] = poly[0];
	poly[i + 1] = poly[1];
	k = 0;
	for (i = 0; i < 2 * n; i += 2)
		clipRandomEdge(poly[i], poly[i + 1], poly[i + 2], poly[i + 3], q->vr.a, q->vr.b);
	n = k / 2;
	for (i = 0; i < k; i++)
		poly[i] = arr[i];
	poly[i] = poly[0];
	poly[i + 1] = poly[1];
	k = 0;
	for (i = 0; i < 2 * n; i += 2)
		clipRandomEdge(poly[i], poly[i + 1], poly[i + 2], poly[i + 3], q->vr.b, q->vr.c);
	for (i = 0; i < k; i++)
		poly[i] = round(arr[i]);

}

void display(){

	//clear the display
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glClearColor(BLACK, 0);	//color black
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	/********************
	/ set-up camera here
	********************/
	//load the correct matrix -- MODEL-VIEW matrix
	glMatrixMode(GL_MODELVIEW);

	//initialize the matrix
	glLoadIdentity();

	//now give three info
	//1. where is the camera (viewer)?
	//2. where is the camera is looking?
	//3. Which direction is the camera's UP direction?

	gluLookAt(0, -1, 600, 0, 0, 0, 0, 0, 1);


	//again select MODEL-VIEW
	glMatrixMode(GL_MODELVIEW);


	/****************************
	/ Add your objects from here
	****************************/
	//add objects

	//a simple rectangles

	/*	//FORGET THE FIELD
	//an square field
	glColor3f(0.4, 1, 0.4);
	glBegin(GL_QUADS);{
	glVertex3f(-100,-100,0);
	glVertex3f(100,-100,0);
	glVertex3f(100,100,0);
	glVertex3f(-100,100,0);
	}glEnd();
	*/

	//some gridlines along the field
	int i;
	// draw points
	/*
	glColor3f(0, 1, 0);
	for (int i = 0; i < Pv; i++)
	{
	glBegin(GL_POINTS); {
	glVertex3f(Px[i], Py[i], 0.1);
	} glEnd();
	}
	*/
	glColor3f(0, 0, 1);
	q->vr.draw();

	// draw edges
	glColor3f(1, 0, 0);
	for (int i = 0; i < Pv; i++)
	{
		int j = (i + 1) % Pv;
		glBegin(GL_LINES); {
			glVertex3f(Px[i], Py[i], Pz[i]);
			glVertex3f(Px[j], Py[j], Pz[j]);
		} glEnd();
	}

	glColor3f(0.5, 0.5, 0.5);	//grey
	glBegin(GL_LINES); {
		for (i = -100; i <= 100; i++){

			if (i == 0)
				continue;	//SKIP the MAIN axes

			//lines parallel to Y-axis
			glVertex3f(i * 100, -1000, -1);
			glVertex3f(i * 100, 1000, -1);

			//lines parallel to X-axis
			glVertex3f(-1000, i * 100, -1);
			glVertex3f(1000, i * 100, -1);
		}
	}glEnd();


	//cout << xMin << xMax << yMin << yMax;
	/*
	glColor3f(1, 1, 0);
	glBegin(GL_LINES); {
	glVertex3f(xMin, yMin, 0.1);
	glVertex3f(xMin, yMax, 0.1);
	} glEnd();

	glBegin(GL_LINES); {
	glVertex3f(xMin, yMax, 0.1);
	glVertex3f(xMax, yMax, 0.1);
	} glEnd();

	glBegin(GL_LINES); {
	glVertex3f(xMax, yMax, 0.1);
	glVertex3f(xMax, yMin, 0.1);
	} glEnd();

	glBegin(GL_LINES); {
	glVertex3f(xMax, yMin, 0.1);
	glVertex3f(xMin, yMin, 0.1);
	} glEnd();
	*/
	// draw the two AXES
	glColor3f(0.5, 0.5, 0.5);	//100% white
	glBegin(GL_LINES); {
		//Y axis
		glVertex3f(0, -1500, 0);	// intentionally extended to -150 to 150, no big deal
		glVertex3f(0, 1500, 0);

		//X axis
		glVertex3f(-1500, 0, 0);
		glVertex3f(1500, 0, 0);
	}glEnd();

	for (int i = 0; i < k; i += 2) {
		glColor3f(0, 0, 1);
		glBegin(GL_POINTS); {
			glVertex3f(poly[i], poly[i + 1], 0.1);
		} glEnd();
		glColor3f(0, 1, 1);
		glBegin(GL_LINES); {
			glVertex3f(poly[i], poly[i + 1], 2);
			glVertex3f(poly[i + 2], poly[i + 3], 2);
		}
		glEnd();
	}
	glColor3f(0, 1, 1);
	glBegin(GL_LINES); {
		glVertex3f(poly[0], poly[1], 2);
		glVertex3f(poly[k - 2], poly[k - 1], 2);
	}
	glEnd();


	//ADD this line in the end --- if you use double buffer (i.e. GL_DOUBLE)
	glutSwapBuffers();
}

void animate(){
	//codes for any changes in Models, Camera
}

void init(){

	//clip polygon
	clip();

	glEnable(GL_POINT_SMOOTH);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glPointSize(6.0);

	//codes for initialization
	//clear the screen
	glClearColor(BLACK, 0);

	/************************
	/ set-up projection here
	************************/
	//load the PROJECTION matrix
	glMatrixMode(GL_PROJECTION);

	//initialize the matrix
	glLoadIdentity();

	//give PERSPECTIVE parameters
	gluPerspective(70, 1, 0.1, 10000.0);
	//field of view in the Y (vertically)
	//aspect ratio that determines the field of view in the X direction (horizontally)
	//near distance
	//far distance
}

int main(int argc, char **argv){
	ifstream myf("input.txt");
	int iter = 0, index = 0;
	if (myf.is_open()) {
		do
		{
			if (!iter++) {
				myf >> Pv;
				Px = new double[Pv];
				Py = new double[Pv];
				Pz = new double[Pv];
				poly = new double[2 * Pv];
				arr = new double[2 * Pv];
			}
			else {
				myf >> Px[index];
				myf >> Py[index];
				myf >> Pz[index++];

			}
		} while (myf.peek() != EOF);
	}
	myf.close();
	/*
	ifstream myf1("input2.txt");
	if (myf1.is_open()) {
	do
	{
	myf1 >> xMin;
	myf1 >> xMax;
	myf1 >> yMin;
	myf1 >> yMax;

	} while (myf1.peek() != EOF);
	}
	myf1.close();
	*/
	ifstream myf1("input2.txt");
	if (myf1.is_open()) {
		do
		{
			int x, y, z, i, j, k;
			myf1 >> x; myf1 >> y; myf1 >> z; myf1 >> i; myf1 >> j; myf1 >> k;
			q = new QueryPoint(*(new Coordinate(x, y, z)), *(new Vector3d(i, j, k)));
		} while (myf1.peek() != EOF);
	}
	myf1.close();

	for (int i = 0, j = 0; j < Pv;) {
		poly[i++] = Px[j];
		poly[i++] = Py[j++];
	}

	/*
	ifstream myf1("output.txt");
	iter = 0, index = 0;
	if (myf1.is_open()) {
	do
	{
	if (!iter++) {
	myf1 >> V;
	X1 = new double[V];
	Y1 = new double[V];
	X2 = new double[V];
	Y2 = new double[V];
	}
	else {
	myf1 >> X1[index];
	myf1 >> Y1[index];
	myf1 >> X2[index];
	myf1 >> Y2[index++];
	}
	} while (myf1.peek() != EOF);
	}
	myf1.close();

	cout << V << endl;

	double maxX = 0;
	for (int i = 0; i < Pv; i++) {
	double x = Px[i];
	if (x < 0) x *= -1;
	if (x > maxX) maxX = x;
	}
	scaleX = 100 / maxX;

	double maxY = 0;
	for (int i = 0; i < Pv; i++) {
	double y = Py[i];
	if (y < 0) y *= -1;
	if (y > maxY) maxY = y;
	}
	scaleY = 100 / maxY;
	cout << scaleX << scaleY;
	for (int i = 0; i < Pv; i++) {
	Px[i] *= scaleX;
	Py[i] *= scaleY;
	}

	for (int i = 0; i < V; i++) {
	X1[i] *= scaleX;
	Y1[i] *= scaleY;
	X2[i] *= scaleX;
	Y2[i] *= scaleY;
	}
	*/
	glutInit(&argc, argv);
	glutInitWindowSize(700, 700);
	glutInitWindowPosition(0, 0);
	glutInitDisplayMode(GLUT_DEPTH | GLUT_DOUBLE | GLUT_RGB);	//Depth, Double buffer, RGB color

	glutCreateWindow("My OpenGL Program");

	init();

	glEnable(GL_DEPTH_TEST);	//enable Depth Testing

	glutDisplayFunc(display);	//display callback function
	glutIdleFunc(animate);		//what you want to do in the idle time (when no drawing is occuring)

	glutMainLoop();		//The main loop of OpenGL

	return 0;
}

#endif
