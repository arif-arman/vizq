#ifndef UTILITY_H
#define UTILITY_H

#include "Rectangle.h"
#include "VisibleRegionOverT.h"

double FOV = 30.0;
int D = 100;
#define PI 3.1416
#define min(a, b) (((a) < (b))? (a) : (b)  )
#define max(a, b) (((a) > (b))? (a) : (b)  )




char c[] = "D:\\VizQ\ February\ 2017\\CMVQ\\CMVQ\\VizQ\\";
//char c[] = "H:\\UIU\\Drive\\Research\\VizQ\ February\ 2017\\CMVQ\\CMVQ\\VizQ\\";
char dataFilePath[] = "\\british.txt";
char dataFile[] = "british";
char qpFilePathBase[] = "\\VaryPath\\British\\";
char qpFilePathTail[]= "querypoint";
char qpFilePathOutputTail[] = "";// "Range";

char outputPath[] = "D:\\VizQ\ February\ 2017\\CMVQ\\CMVQ\\VizQ\\";
//char outputPath[] = "H:\\UIU\\Drive\\Research\\VizQ\ February\ 2017\\CMVQ\\CMVQ\\VizQ\\";
//char prefix[] = "500";
char prefix[500];

//remember to change infront of fov
char choice[20] = "British\\";
char prefixOutput[500] = "";// "Range";// "FOV";
char varyWhat[] = "\\VaryPath\\British\\";
char varyWhatOutputBase[] = "\\VaryPath\\British\\";// 500\\";

//char varyAlgo[50] = "";
char varyAlgo[50] = "MBRBS\\";
char targetChoice[20] = "";
char isPathVaried[10] = "path_";
char pathCountString[10]="";

char debugString[10] = "debug";
int isDebug = 0;

int Nth =3;






double intX = -1, intY = -1;
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
	//cout << "Intersction " << intX << " " << intY << endl;
	return true;

}

double slope(double x1, double y1, double x2, double y2)//2 points of a line is given
{
	double s = (double)(y1 - y2) / (x1 - x2);
	return s;
}
double angle_between_2_lines(double x1, double y1, double x2, double y2, double x11, double y11, double x21, double y21)
{
	double m1, m2;
	int flag1 = 0; int flag2 = 0;
	if (x1 != x2)
	{
		flag1 = 1;
		m1 = slope(x1, y1, x2, y2);
	}
	if (x11 != x21)
	{
		flag2 = 1;
		m2 = slope(x11, y11, x21, y21);
	}
	if ((x1 == x2) && (x11 == x21))
	{
		return 0.0;
	}

	else if (x1 == x2)
	{
		return 90 - (atan((float)m2) * (180 / PI));
	}
	else if (x11 == x21)
	{
		return 90 - (atan((float)m1) * (180 / PI));
	}
	else
	{
		double tan_theta = ((double)(m1 - m2) / (1 + m1*m2));
		if (tan_theta<0.0)tan_theta = (tan_theta*-1);//acute angle only
		return (atan((float)tan_theta) * (180 / PI));
	}

}


bool IsPointInsideTriangle(Coordinate a, Coordinate b, Coordinate c, Coordinate x)
{
	double angle1 = angle_between_2_lines(x.x, x.y, a.x, a.y, x.x, x.y, c.x, c.y);
	double angle2 = angle_between_2_lines(x.x, x.y, a.x, a.y, x.x, x.y, b.x, b.y);
	double angle3 = angle_between_2_lines(x.x, x.y, b.x, b.y, x.x, x.y, c.x, c.y);

	if (abs(angle1 + angle2 + angle3 - 180) <= 0.0001)
	{
		return true;
	}
}

Coordinate mid_of_line_segment(Coordinate a, Coordinate b)
{
	Coordinate mid;
	mid.x = (a.x + b.x) / 2.0;
	mid.y = (a.y + b.y) / 2.0;
	return mid;
}



double reduced_visibility_factor4angle(double angle)
{
	return angle / 90.0;
}

double Distance(Coordinate p1, Coordinate p2)
{
	return sqrt(((p1.x - p2.x)*(p1.x - p2.x)) + ((p1.y - p2.y)*(p1.y - p2.y)));
}

Coordinate intersection_point(Coordinate p1, Coordinate p2, Coordinate p3, Coordinate p4)
{
	Coordinate intersect;
	intersect.x = (((p1.x*p2.y - p2.x*p1.y)*(p3.x - p4.x)) - ((p1.x - p2.x)*(p3.x*p4.y - p3.y*p4.x))) / (((p1.x - p2.x)*(p3.y - p4.y)) - ((p1.y - p2.y)*(p3.x - p4.x)));

	intersect.y = (((p1.x*p2.y - p2.x*p1.y)*(p3.y - p4.y)) - ((p1.y - p2.y)*(p3.x*p4.y - p3.y*p4.x))) / (((p1.x - p2.x)*(p3.y - p4.y)) - ((p1.y - p2.y)*(p3.x - p4.x)));
	return intersect;
}

bool ccw(Coordinate A, Coordinate B, Coordinate C)
{
	return (C.y - A.y)*(B.x - A.x) > (B.y - A.y)*(C.x - A.x);
}

bool intersect(Coordinate a, Coordinate b, Coordinate c, Coordinate d)//if line segments intersect
{
	return ccw(a, c, d) != ccw(b, c, d) && ccw(a, b, c) != ccw(a, b, d);
}

bool intersect_ext(Coordinate a, Coordinate b, Coordinate c, Coordinate d)
{
	double m1, m2;
	if (a.x != b.x)
	{
		m1 = slope(a.x, a.y, b.x, b.y);
	}

	if (c.x != d.x)
	{
		m2 = slope(c.x, c.y, d.x, d.y);
	}

	if ((a.x == b.x) && (c.x == d.x))
	{
		return false;//both ar perpendicular on x axis 
	}
	else if ((a.x == b.x) && (c.x != d.x))
	{
		return true;
	}
	else if ((a.x != b.x) && (c.x == d.x))
	{
		return true;
	}
	else if (m1 == m2)
	{
		return false;//parallal lines
	}
	else return true;
}
bool if_intersects_target(Coordinate position, Coordinate obstacle_p, SomeRandomRectangle target, Coordinate& intersectPoint, Coordinate p1, Coordinate p2)
{

	Coordinate target_p1 = target.upper_left;
	Coordinate target_p2;
	target_p2.x = target.upper_left.x;
	target_p2.y = target.lower_right.y;
	Coordinate target_p3;
	target_p3.x = target.lower_right.x;
	target_p3.y = target.upper_left.y;
	Coordinate target_p4 = target.lower_right;

	float lowx, lowy, highx, highy;
	lowx = min(p1.x, p2.x);
	lowy = min(p1.y, p2.y);
	highx = max(p1.x, p2.x);
	highy = max(p1.y, p2.y);

	Coordinate Tpoints[8] = { target_p1, target_p2, target_p1, target_p3, target_p3, target_p4, target_p2, target_p4 };
	for (int i = 0; i<8; i += 2)
	{
		if (intersect_ext(position, obstacle_p, Tpoints[i], Tpoints[i + 1]) == true)
		{
			intersectPoint = intersection_point(position, obstacle_p, Tpoints[i], Tpoints[i + 1]);
			if (intersectPoint.x <= highx && intersectPoint.x >= lowx && intersectPoint.y <= highy && intersectPoint.y >= lowy)//intersect point is in that region
			{
				return true;
			}

		}
	}
	return false;


}



bool change_vRegion_one_side(Coordinate position, Coordinate obstacle_p1, SomeRandomRectangle target,Coordinate unchanged_side_val , Coordinate changed_side_val, Coordinate& intersectPoint, VisibleRegionOverT itr)
{
	bool check = if_intersects_target(position, obstacle_p1, target, intersectPoint, unchanged_side_val, changed_side_val);
	if (check == true)
	{
		float a = Distance(unchanged_side_val, (intersectPoint));
		float b = Distance(unchanged_side_val, changed_side_val);
		if (a <= b)
		{

			VisibleRegionOverT v;
			v.p1_overT = unchanged_side_val;
			v.p2_overT = (intersectPoint);

			return true;
		}

	}
	return false;
}



bool isPointOf_VR_changed(Coordinate p1_overT, Coordinate position, Coordinate obstacle_p1,
	Coordinate obstacle_p2, Coordinate obstacle_p3, Coordinate obstacle_p4)

{
	Coordinate Opoints[8] = { obstacle_p1, obstacle_p2, obstacle_p1, obstacle_p3, obstacle_p2, obstacle_p4, obstacle_p3, obstacle_p4 };

	for (int i = 0; i<8; i += 2)
	{
		if (intersect(position, p1_overT, Opoints[i], Opoints[i + 1]))
		{
			return true;
		}
	}

	return false;
}






#endif;