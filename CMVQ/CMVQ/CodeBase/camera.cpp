#include "Camera.h"
#include<cstdio>


Camera::Camera()
{
      // eye.set(300,0,30);
       // look.set(200,0.0,0.0);
        double scale=100;
	
		//eye.set(-85.20, 3.06, 36.56);//  8604.43
//0.04 1.00 -0.00  1.00
//1.00 -0.04 0.00  1.00
//-0.00 0.00 1.00  1.00
		
		

	//	eye.set(662.8096, 1843.234, 10.65378); //london

		eye.set(0,60.09,7.35/2); //sample1.txt
		look.set(0,48.09,7.35/2);
		up.set(0.0,0,1.0);
        set(eye, look, up);



		
		cameraAngle = acos(-1.0);	//// init the cameraAngle
		cameraAngleDelta = 0.005;
		cameraHeight = 0;
		cameraRadius = 150;



}

void Camera::rotateclock( double a )
{
	cameraAngle+=a;
}
void Camera::increaseheight( double h )
{
	cameraHeight+=h;
}
void Camera::increaseradius( double r )
{
	cameraRadius+=r;
}

void Camera::set(Vector3 Eye, Vector3 lookAt, Vector3 up)
{ 
        eye.set(Eye); // store the given eye position
		n.set( lookAt.sub(eye)); // make n
        u.set(up.crossProd(n)); // make u = up X n
        n.normalize(); u.normalize(); // make them unit length
        v.set(n.crossProd(u)); //make v = n X u
}





// yaw,pitch,roll with respect to global axis
void Camera::rotate(Vector3 axis, double angle)
{
        double cs = cos(3.14159265/180 * angle);
        double sn = sin(3.14159265/180 * angle);
        //axis.normalize();

        if (axis.getX() == 1)
        {
                Vector3 t(v);
                v.set(cs * t.getX() + sn * n.getX(), cs * t.getY() + sn * n.getY(), cs * t.getZ() + sn * n.getZ());
                //n.set(sn *t.getX() + cs * n.getX(), sn * t.getY() + cs * n.getY(), sn * t.getZ() + cs * n.getZ());
                n.set(u.crossProd(v));
        }
        if (axis.getY() == 1)
        {
                Vector3 t(u);
                u.set(cs * t.getX() + sn * n.getX(),cs * t.getY() + sn * n.getY(),cs * t.getZ() + sn * n.getZ());
                //n.set(cs * n.getX() + sn * t.getX(),cs * n.getY() + sn * t.getY(),cs * n.getZ() + sn * t.getZ());
                n.set(u.crossProd(v));
        }
        if (axis.getZ() == 1)
        {
                Vector3 t(u);
                u.set(cs * t.getX() - sn * v.getX(), cs * t.getY() - sn * v.getY(), cs * t.getZ() - sn * v.getZ());
                //v.set(sn * t.getX() + cs * v.getX(), sn * t.getY() + cs * v.getY(), sn * v.getZ() + cs * v.getZ());
                v.set(n.crossProd(u));
        }

}





//rotate the camera around its local z axis
void Camera::roll(double angle)
{       // roll the camera through angle degrees
        //convert degrees to radians


	//	printf( "rolling...\n" );

        double cs = cos(3.14159265/180 * angle);
        double sn = sin(3.14159265/180 * angle);
        Vector3 t(u); // remember old u

        u.set(cs * t.getX() - sn * v.getX(), cs * t.getY() - sn * v.getY(), cs * t.getZ() - sn * v.getZ());
        //v.set(sn * t.getX() + cs * v.getX(), sn * t.getY() + cs * v.getY(), sn * v.getZ() + cs * v.getZ());
        v.set(n.crossProd(u));
}

//rotate the camera around its local x axis
void Camera::pitch(double angle)
{       // roll the camera through angle degrees
        //convert degrees to radians
		
	//	printf( "pitching...\n" );
		
        double cs = cos(3.14159265/180 * angle);
        double sn = sin(3.14159265/180 * angle);
        Vector3 t(v);
        v.set(cs * t.getX() + sn * n.getX(), cs * t.getY() + sn * n.getY(), cs * t.getZ() + sn * n.getZ());
        //n.set(sn *t.getX() + cs * n.getX(), sn * t.getY() + cs * n.getY(), sn * t.getZ() + cs * n.getZ());
        n.set(u.crossProd(v));
}

//rotate the camera around its local y axis
void Camera::yaw(double angle)
{       // roll the camera through angle degrees
        //convert degrees to radians
		
	//	printf( "yawing...\n" );

        double cs = cos(3.14159265/180 * angle);
        double sn = sin(3.14159265/180 * angle);
        Vector3 t(u);
        u.set(cs * t.getX() + sn * n.getX(),cs * t.getY() + sn * n.getY(),cs * t.getZ() + sn * n.getZ());
        //n.set(cs * n.getX() + sn * t.getX(),cs * n.getY() + sn * t.getY(),cs * n.getZ() + sn * t.getZ());
        n.set(u.crossProd(v));
}


void Camera::slide(double delU, double delV, double delN)
{
        eye.setX(eye.getX() + delU * u.getX() + delV * v.getX() + delN * n.getX());
        eye.setY(eye.getY() + delU * u.getY() + delV * v.getY() + delN * n.getY());
        eye.setZ(eye.getZ() + delU * u.getZ() + delV * v.getZ() + delN * n.getZ());
}

