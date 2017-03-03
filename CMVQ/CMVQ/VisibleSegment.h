#ifndef VisibleSegment_H

#define VisibleSegment_H

#include "Coordinate.h"
#include "QueryPoint.h"
#include "Utility.h"
class VisibleSegment{

public:
	Coordinate point1, point2;
	Coordinate queryPointPosition;
	int queryPointID;

	VisibleSegment *vslist;

	VisibleSegment(QueryPoint q, MBR target,int queryPointID){
		this->queryPointID = queryPointID;
		queryPointPosition = q.coord;
		if (intersecting(q.coord, target.topRight, target.topLeft, target.bottomLeft) 
			&& intersecting(q.coord, target.bottomRight, target.topLeft, target.bottomLeft)){
			
			point1 = target.topLeft; point2 = target.bottomLeft;
		}
		else if (intersecting(q.coord, target.topRight, target.topLeft, target.bottomRight)){
			
			point1 = target.topLeft; point2 = target.bottomRight;
		}
		else if (intersecting(q.coord, target.topLeft, target.bottomLeft, target.bottomRight)
			&& intersecting(q.coord, target.topRight, target.bottomLeft, target.bottomRight)){
			
			point1 = target.bottomLeft; point2 = target.bottomRight;
		}
		else if (intersecting(q.coord, target.topLeft, target.bottomLeft, target.topRight)){
			
			point1 = target.bottomLeft; point2 = target.topRight;
		}
		else if (intersecting(q.coord, target.topLeft, target.bottomRight, target.topRight)
			&& intersecting(q.coord, target.bottomLeft, target.bottomRight, target.topRight)){
			
			point1 = target.bottomRight; point2 = target.topRight;
		}
		else if (intersecting(q.coord, target.bottomLeft, target.bottomRight, target.topLeft)){
			
			point1 = target.bottomRight; point2 = target.topLeft;
		}
		
		

	}

	void updateVisibleSegment(MBR obstacle){
	
		Coordinate obPoint1, obsPoint2;

		if (intersecting(queryPointPosition, obstacle.topRight, obstacle.topLeft, obstacle.bottomLeft)
			&& intersecting(queryPointPosition, obstacle.bottomRight, obstacle.topLeft, obstacle.bottomLeft)){

			obPoint1 = obstacle.topLeft; obsPoint2 = obstacle.bottomLeft;
		}
		else if (intersecting(queryPointPosition, obstacle.topRight, obstacle.topLeft, obstacle.bottomRight)){

			obPoint1 = obstacle.topLeft; obsPoint2 = obstacle.bottomRight;
		}
		else if (intersecting(queryPointPosition, obstacle.topLeft, obstacle.bottomLeft, obstacle.bottomRight)
			&& intersecting(queryPointPosition, obstacle.topRight, obstacle.bottomLeft, obstacle.bottomRight)){

			obPoint1 = obstacle.bottomLeft; obsPoint2 = obstacle.bottomRight;
		}
		else if (intersecting(queryPointPosition, obstacle.topLeft, obstacle.bottomLeft, obstacle.topRight)){

			obPoint1 = obstacle.bottomLeft; obsPoint2 = obstacle.topRight;
		}
		else if (intersecting(queryPointPosition, obstacle.topLeft, obstacle.bottomRight, obstacle.topRight)
			&& intersecting(queryPointPosition, obstacle.bottomLeft, obstacle.bottomRight, obstacle.topRight)){

			obPoint1 = obstacle.bottomRight; obsPoint2 = obstacle.topRight;
		}
		else if (intersecting(queryPointPosition, obstacle.bottomLeft, obstacle.bottomRight, obstacle.topLeft)){

			obPoint1 = obstacle.bottomRight; obsPoint2 = obstacle.topLeft;
		}
		

		//if ()

	}

};

#endif