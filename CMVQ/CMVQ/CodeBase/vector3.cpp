#include <cmath>
#include "Vector3.h"
#include "macros.h"


#include <vector>
#include <list>
#include <map>
#include <set>
#include <deque>
#include <stack>
#include <bitset>
#include <algorithm>
#include <functional>
#include <numeric>
#include <utility>
#include <sstream>
#include <iostream>
#include <iomanip>
#include <cstdio>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <cstring>
#include <queue>
#include <cassert>


//#include<GL/glut.h>


using namespace std;


Vector3::Vector3()
{
        x = 0.0;
        y = 0.0;
        z = 0.0;
}

Vector3::Vector3(double dx,double dy,double dz)
{
        x = dx;
        y = dy;
        z = dz;
}

Vector3::Vector3(Vector3& vector)
{
        x = vector.getX();
        y = vector.getY();
        z = vector.getZ();
}

void Vector3::set(Vector3 vector)
{
        x = vector.getX();
        y = vector.getY();
        z = vector.getZ();
}

void Vector3::set(double dx,double dy,double dz)
{
        x = dx;
        y = dy;
        z = dz;
}

void Vector3::setX(double dx)
{
        x = dx;
}

void Vector3::setY(double dy)
{
        y = dy;
}

void Vector3::setZ(double dz)
{
        z = dz;
}

double Vector3::getX()
{
        return x;
}

double Vector3::getY()
{
        return y;
}

double Vector3::getZ()
{
        return z;
}

double Vector3::dotProd(Vector3 vector)
{
        double dot = (x * vector.getX()) + (y * vector.getY()) +
                                (z * vector.getZ());
        return dot;
}

Vector3 Vector3::crossProd(Vector3 vector)
{
        double i = (y * vector.getZ()) - (z * vector.getY());
        double j = (x * vector.getZ()) - (z * vector.getX());
        double k = (x * vector.getY()) - (y * vector.getX());

        Vector3 cross(i,-j,k);
        return cross;
}


Vector3 Vector3::add(Vector3 vector)
{
	return Vector3( x+vector.x , y+vector.y,z+vector.z );
}

Vector3 Vector3::sub(Vector3 vector)
{
	return Vector3( x-vector.x , y-vector.y,z-vector.z );
}

Vector3 Vector3::mul(double scale)
{
	return Vector3( x*scale , y*scale,z*scale );
}


Vector3 Vector3::div(double scale)
{
	return Vector3( x/scale , y/scale,z/scale );
}


void Vector3::normalize()
{
        double length = sqrt((x*x)+(y*y)+(z*z));

        x = x / length;
        y = y / length;
        z = z / length;
}

double Vector3::mag2()
{
	return x*x+y*y+z*z;
}


void Vector3::print()
{
	printf("%.2lf %.2lf %.2lf\n",x,y,z);

}

void Vector3:: scan()
{
	scanf("%2lf %2lf %2lf",&x,&y,&z);
}


void Vector3:: fscan(FILE* fp)
{
	fscanf(fp,"%lf %lf %lf",&x,&y,&z);
}


bool Vector3:: operator<( const Vector3& v )
{
	if( Z(v.x-x) && Z(v.y-y) )return v.z<z;
	if( Z(v.x-x)  )return v.y<y;
	return v.x<x;
}



Plane::Plane(){}
Plane::Plane( double n1,double n2,double n3,double d0 ){n.x=this->n1=n1,n.y=this->n2=n2,n.z=this->n3=n3,this->d0=d0;}
Plane::Plane (Vector3 p1,Vector3 p2,Vector3 p3)
{
	Vector3 n=(p3.sub(p1)).crossProd(p2.sub(p1));
	n1=n.x;
	n2=n.y;
	n3=n.z;

	this->n=n;

	d0=n.dotProd(p1);

	v0=v1=p1;

	v0.x=min(v0.x,p2.x);v0.y=min(v0.y,p2.y);v0.z=min(v0.z,p2.z);
	v0.x=min(v0.x,p3.x);v0.y=min(v0.y,p3.y);v0.z=min(v0.z,p3.z);


	v1.x=max(v1.x,p2.x);v1.y=max(v1.y,p2.y);v1.z=max(v1.z,p2.z);
	v1.x=max(v1.x,p3.x);v1.y=max(v1.y,p3.y);v1.z=max(v1.z,p3.z);


	this->n.normalize();

}


	


bool Plane::caclProjectionOfPoint(    Vector3 C, Vector3 P,Vector3& Q  ) // project p on plane wrt c
{


//	if( Z( C.x-P.x ) && Z( C.y-P.y ) && Z( C.z-P.z )  ){Q=P;return true;}

	// from slide of text

	double d1=n1*C.x+n2*C.y+n3*C.z;
	double d=d0-d1;

	double alp=( n1*( P.x-C.x )+n2*( P.y-C.y )+n3*( P.z-C.z )  );

	if( Z(alp) || alp<0 )return false;

	alp=d/alp;
	
		


	/*

	// projection matrix

	d+an1 an2 an3 -ad0
	bn1 d+bn2 bn3 -bd0
	cn1 cn2 d+cn3 -cd0
	n1 n2 n3 -d1

	*/




	// point is in form [ x,y,z,w ]


	double pp[4];
	double ap[4]={P.x,P.y,P.z,1};

	double x=P.x,y=P.y,z=P.z;
	double a=C.x,b=C.y,c=C.z;


		
	double M[4][4]={{d+a*n1, a*n2, a*n3, -a*d0},{b*n1, d+b*n2, b*n3, -b*d0},{c*n1, c*n2 ,d+c*n3 ,-c*d0},{n1 ,n2 ,n3 ,-d1}};




		
	for(int i=0;i<4;i++ )
	{
		pp[i]=0;
		for(int j=0;j<4;j++ )
		{
			pp[i]+=M[i][j]*ap[j];
		}
	}

	Q.x=pp[0];
	Q.y=pp[1];
	Q.z=pp[2];


	if( Z(pp[3]) )return false;


	Q=Q.div(pp[3]);

	return true;

}

bool Plane :: rayPlaneIntersction(Vector3 &p0,Vector3 &p1,Vector3 &I)
{

	//return false;

	Vector3 u=p1.sub( p0 );
	Vector3 w=p0.sub( v0 );

	double D=n.dotProd(u);
	double N=-n.dotProd(w);


	if(fabs(D)<EPS)return false;

	double sI=N/D;

	if(sI<0 || sI>1)return false;


	I=p0.add( u.mul(sI) );

	return true;
}

//class Box;


void Box::init(Vector3 &v0,Vector3 &v1,Vector3 &origin)
{

	v0=v0.sub( origin );
	v1=v1.sub( origin );

	//debuging purpose...
	//v0.x=-v0.x;
	//v1.x=-v1.x;

	o=v0.add( v1 ).div(2);
	

	//v0=Vector3(0,0,0).sub(v0);
	//v1=Vector3(0,0,0).sub(v1);
	

	

//	v0=v0.sub(o);
	//v1=v1.sub(o);
	//o=o.sub(o);

	this->v0=v0;
	this->v1=v1;

	x[0]=v0.x;
	y[0]=v0.y;
	z[0]=v0.z;

	x[1]=v1.x;
	y[1]=v1.y;
	z[1]=v1.z;


	a=Vector3(x[1],y[1],z[1]);
	b=Vector3(x[1],y[0],z[1]);
	c=Vector3(x[0],y[0],z[1]);
	d=Vector3(x[0],y[1],z[1]);

		
	a1=Vector3(x[1],y[1],z[0]);
	b1=Vector3(x[1],y[0],z[0]);
	c1=Vector3(x[0],y[0],z[0]);
	d1=Vector3(x[0],y[1],z[0]);



	lx=fabs( x[1]-x[0] );
	ly=fabs( y[1]-y[0] );
	lz=fabs( z[1]-z[0] );

	



	//transfer the origin to the center of the target

	



}

Box::Box(Vector3 v0,Vector3 v1) // tow corner point
{
	init(v0,v1,Vector3(0,0,0));
}



void Box::scan()
{
	Vector3 v0,v1;
	v0.scan();
	v1.scan();
	init(v0,v1,Vector3(0,0,0));
	setPlanes();
}

void Box::fscan(FILE* fp,Vector3 origin)
{
	Vector3 v0,v1;
	v0.fscan(fp);
	v1.fscan(fp);
	init(v0,v1,origin);
	setPlanes();
}





void Box::setPlanes() // fixing the 6 planes treating them like a box
{
		
//	pz=new Plane[2];//bz=new Box[2];
	pz[1]=Plane( a,b,c );//bz[1]=Box(a,c);
	pz[0]=Plane( a1,b1,c1 );//bz[0]=Box(a1,c1);
	
	bx=new Box[2];//px=new Plane[2];
	px[1]=Plane( a,a1,b1 );bx[1]=Box(a,b1);
	px[0]=Plane( d,d1,c1 );bx[0]=Box(d,c1);
	
	
	//py=new Plane[2];//by=new Box[2];
	py[1]=Plane( a,a1,d1 );//by[1]=Box(a,d1);
	py[0]=Plane( b,b1,c1 );//by[0]=Box(b,c1);
	
}

void Box::print()
{
		
	cout<<x[0]<<" "<<y[0]<<" "<<z[0]<<endl;
	cout<<x[1]<<" "<<y[1]<<" "<<z[1]<<endl;
}

void Box::draw()
{
	Vector3 c(x[0]+x[1],y[0]+y[1],z[0]+z[1]);
	c=c.div(2);

	double dx=fabs(x[1]-x[0]);
	double dy=fabs(y[1]-y[0]);
	double dz=fabs(z[1]-z[0]);


	Vector3 dv(dx,dy,dz);
		
		
		/*
	glPushMatrix();
	{
		glTranslated( CO(c) );
		glScaled( CO(dv) );
		glutSolidCube(1);
	}glPopMatrix();*/

}




bool commonBox( Box &a,Box &b,Box &c )
{
	
	Vector3 v0(max( a.x[0],b.x[0] ),max( a.y[0],b.y[0] ),max( a.z[0],b.z[0] ));
	Vector3 v1(min( a.x[1],b.x[1] ),min( a.y[1],b.y[1] ),min( a.z[1],b.z[1] ));
	
	c=Box(v0,v1);

	if( c.x[1]-c.x[0]<0  )return false;
	if( c.y[1]-c.y[0]<0  )return false;
	if( c.z[1]-c.z[0]<0  )return false;
	
	return true;
}



