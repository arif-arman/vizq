#include "myCamera.h"
#include<cstdio>

myCamera::myCamera() 
{
	camPosition.set(0, -80, 30);
	look.set(0, 0, 0);
	upDir.set(0, 0, 1);
}

void myCamera::pitch(double p)
{
	dir.set(camPosition.sub(look));//added
	dirMag = dir.mag2();//addded
	dirMag = sqrt(dirMag);
	along.set(dir.crossProd(upDir));//added
	along.negate();
	along.normalize();
	normAlong.set(along);//added
	dir.normalize();
	normForward.set(dir);//added

	angle = 180 * 3.1416 / 180 + p;

	newLook.set(camPosition.add(normForward.mul(dirMag*cos(angle))));//added
	newLook.set(newLook.add(normAlong.mul(dirMag*sin(angle))));//added
	look.set(newLook);//added
}

void myCamera::yaw(double y) 
{
	dir.set(camPosition.sub(look));//added
	dirMag = dir.mag2();//addded
	dirMag = sqrt(dirMag);

	along.set(dir.crossProd(upDir));//added
	along.negate();
	
	dir.normalize();
	normForward.set(dir);//added
	upDir.normalize();
	normUp.set(upDir);//added

	angle = 180 * 3.1416 / 180 + y;

	newLook.set(camPosition.add(normForward.mul(dirMag*cos(angle))));//added
	newLook.set(newLook.add(normUp.mul(dirMag*sin(angle))));//added
	look.set(newLook);//added

	dir.set(camPosition.sub(look));//added

	upDir.set(dir.crossProd(along));//added
	upDir.normalize();
}

void myCamera::roll(double r)
{
	dir.set(camPosition.sub(look));//added
	dirMag = dir.mag2();//addded
	dirMag = sqrt(dirMag);

	along.set(dir.crossProd(upDir));//added
	along.negate();
	along.normalize();

	normAlong.set(along);//added
	upDir.normalize();
	normUp.set(upDir);//added

	angle = 0 * 3.1416 / 180 + r;

	newUp.set(camPosition.add(normUp.mul(dirMag*cos(angle))));//added
	newUp.set(newUp.add(normAlong.mul(dirMag*sin(angle))));//added

	upDir.set(newUp.sub(camPosition));//added
	upDir.normalize();
}

void myCamera::slide(double speed, int axis)
{
	Vector3 norm;
	if (axis == 1) {
		dir.set(camPosition.sub(look));//new added
		dir.normalize();
		norm.set(dir);//added

		newLook.set(look.add(norm.mul(speed)));//added
		look.set(newLook);//added
		newcamPosition.set(camPosition.add(norm.mul(speed)));//added
		
		camPosition.set(newcamPosition);//added
	}
	else if (axis == 2) {
		dir.set(camPosition.sub(look));//new added
		dir.normalize();
		norm.set(dir);//added

		newLook.set(look.sub(norm.mul(speed)));//added
		look.set(newLook);//added
		newcamPosition.set(camPosition.sub(norm.mul(speed)));//added

		camPosition.set(newcamPosition);//added
	}
	else if (axis == 3) {
		dir.set(camPosition.sub(look));//new added
		along.set(dir.crossProd(upDir));//added
		along.normalize();
		norm.set(along);//added

		newcamPosition.set(camPosition.add(norm.mul(speed)));//added
		camPosition.set(newcamPosition);//added

		newLook.set(look.add(norm.mul(speed)));//added
		look.set(newLook);//added
	}
	else if (axis == 4) {
		dir.set(camPosition.sub(look));//new added
		along.set(dir.crossProd(upDir));//added
		along.normalize();
		norm.set(along);//added

		newcamPosition.set(camPosition.sub(norm.mul(speed)));//added
		camPosition.set(newcamPosition);//added

		newLook.set(look.sub(norm.mul(speed)));//added
		look.set(newLook);//added
	}
	else if (axis == 5) {
		upDir.normalize();
		norm.set(upDir);//added

		newcamPosition.set(camPosition.add(norm.mul(speed)));//added
		camPosition.set(newcamPosition);//added

		newLook.set(look.add(norm.mul(speed)));//added
		look.set(newLook);//added
	}
	else {
		upDir.normalize();
		norm.set(upDir);//added

		newcamPosition.set(camPosition.sub(norm.mul(speed)));//added
		camPosition.set(newcamPosition);//added

		newLook.set(look.sub(norm.mul(speed)));//added
		look.set(newLook);//added
	}

	
}