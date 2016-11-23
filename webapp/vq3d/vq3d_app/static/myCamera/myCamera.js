MyCamera = function() {
	this.camPosition = new THREE.Vector3(20, -60, 20);
	this.look = new THREE.Vector3(0, 0, 0);
	this.upDir = new THREE.Vector3(0, 0, 1);

	this.dir = new THREE.Vector3();
	this.newAlong = new THREE.Vector3();
	this.normAlong = new THREE.Vector3();
	this.normForward = new THREE.Vector3();
	this.normUp = new THREE.Vector3();
	this.newcamPosition = new THREE.Vector3();
	this.newLook = new THREE.Vector3();
	this.newUp = new THREE.Vector3();
	this.along = new THREE.Vector3();

	this.dirMag = 0;
	this.angle = 0;
	this.norm = new THREE.Vector3();
}

MyCamera.prototype = {
	constructor: MyCamera,
	negate:function (a) {
		var t = new THREE.Vector3(-a.x, -a.y, -a.z);
		return t;
	},

	pitch: function(p) {
		this.dir.subVectors(this.camPosition, this.look);//added
		this.dirMag = this.dir.length();//addded
		
		this.along.crossVectors(this.dir, this.upDir);//added
		this.along.copy(this.along.negate());
		this.along.normalize();

		this.normAlong.copy(this.along);//added
		this.dir.normalize();

		this.normForward.copy(this.dir);//added

		this.angle = 180 * 3.1416 / 180 + p;

		this.newLook.addVectors(this.camPosition, (this.normForward.multiplyScalar(this.dirMag*Math.cos(this.angle))));//added
		this.newLook.add(this.normAlong.multiplyScalar(this.dirMag*Math.sin(this.angle)));//added
		
		this.look.copy(this.newLook);//added
	},

	yaw: function(y) {
		this.dir.subVectors(this.camPosition, this.look);//added
		this.dirMag = this.dir.length();//addded
		

		this.along.crossVectors(this.dir, this.upDir);//added
		this.along.negate();
		
		this.dir.normalize();
		this.normForward.copy(this.dir);//added
		this.upDir.normalize();
		this.normUp.copy(this.upDir);//added

		this.angle = 180 * 3.1416 / 180 + y;

		this.newLook.addVectors(this.camPosition, this.normForward.multiplyScalar(this.dirMag*Math.cos(this.angle)));//added
		this.newLook.add(this.normUp.multiplyScalar(this.dirMag*Math.sin(this.angle)));//added
		this.look.copy(this.newLook);//added

		this.dir.subVectors(this.camPosition, this.look);//added

		this.upDir.crossVectors(this.dir, this.along);//added
		this.upDir.normalize();
	}, 

	roll:function (r) {
		this.dir.subVectors(this.camPosition, this.look);//added
		this.dirMag = this.dir.length();//addded

		this.along.crossVectors(this.dir, this.upDir);//added
		this.along.negate();
		this.along.normalize();

		this.normAlong.copy(this.along);//added
		this.upDir.normalize();
		this.normUp.copy(this.upDir);//added

		this.angle = 0 * 3.1416 / 180 + r;

		this.newUp.addVectors(this.camPosition, (this.normUp.multiplyScalar(this.dirMag*Math.cos(this.angle))));//added
		this.newUp.add(this.normAlong.multiplyScalar(this.dirMag*Math.sin(this.angle)));//added

		this.upDir.subVectors(this.newUp, this.camPosition);//added
		this.upDir.normalize();
	},

	slide:function(speed, axis) {

		if (axis == 1) {
			
			this.dir.subVectors(this.camPosition, this.look);//new added
			this.dir.normalize();
			this.norm.copy(this.dir);//added

			this.newLook.addVectors(this.look, this.norm.multiplyScalar(speed));//added
			this.look.copy(this.newLook);//added
			this.newcamPosition.addVectors(this.camPosition, (this.norm));//added
			
			this.camPosition.copy(this.newcamPosition);//added
		}
		else if (axis == 2) {
			this.dir.subVectors(this.camPosition, this.look);//new added
			this.dir.normalize();
			this.norm.copy(this.dir);//added

			this.newLook.subVectors(this.look, this.norm.multiplyScalar(speed));//added
			this.look.copy(this.newLook);//added
			this.newcamPosition.subVectors(this.camPosition, this.norm);//added

			this.camPosition.copy(this.newcamPosition);//added
		}
		else if (axis == 3) {
			this.dir.subVectors(this.camPosition, this.look);//new added
			this.along.crossVectors(this.dir, this.upDir);//added
			this.along.normalize();
			this.norm.copy(this.along);//added
			//console.log(this.norm);

			this.newcamPosition.addVectors(this.camPosition, this.norm.multiplyScalar(speed));//added
			this.camPosition.copy(this.newcamPosition);//added

			this.newLook.addVectors(this.look, (this.norm));//added
			this.look.copy(this.newLook);//added
		}
		else if (axis == 4) {
			this.dir.subVectors(this.camPosition, this.look);//new added
			this.along.crossVectors(this.dir, this.upDir);//added
			this.along.normalize();
			this.norm.copy(this.along);//added

			this.newcamPosition.subVectors(this.camPosition, (this.norm.multiplyScalar(speed)));//added
			this.camPosition.copy(this.newcamPosition);//added

			this.newLook.subVectors(this.look, (this.norm));//added
			this.look.copy(this.newLook);//added
		}
		else if (axis == 5) {
			this.upDir.normalize();
			this.norm.copy(this.upDir);//added

			this.newcamPosition.addVectors(this.camPosition, (this.norm.multiplyScalar(speed)));//added
			this.camPosition.copy(this.newcamPosition);//added

			this.newLook.addVectors(this.look, (this.norm));//added
			this.look.copy(this.newLook);//added
		}
		else {
			this.upDir.normalize();
			this.norm.copy(this.upDir);//added

			this.newcamPosition.subVectors(this.camPosition, this.norm.multiplyScalar(speed));//added
			this.camPosition.copy(this.newcamPosition);//added

			this.newLook.subVectors(this.look, (this.norm));//added
			this.look.copy(this.newLook);//added
		}
		//console.log(this.camPosition);
		//console.log(this.look);

		
	}



};


/*
function yaw(y) 
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

function roll(r)
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

function slide(speed, axis)
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
*/