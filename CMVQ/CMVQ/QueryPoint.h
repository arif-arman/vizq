#define _CRT_SECURE_NO_WARNINGS

#ifndef QUERYPOINT_H
#define QUERYPOINT_H

#include "Coordinate.h"
#include "Vector3d.h"
#include "VisibleRegion.h"
#include "Utility.h"
#include "Ray.h"
//#include "VisibleRegionOverT.h"
#include "Rectangle.h"
#include <vector>

using namespace std;

class QueryPoint{

public:

	Coordinate coord;
	Vector3d direction;
	VisibleRegion vr;

	SomeRandomRectangle r;

	//from ICDE
	double total_visibility; //for this query_point only
	vector <VisibleRegionOverT>  visibleSegment;//segments over Target object T, that are visible from this query point
	// list <Rectangle> ObstacleList; 

	bool IsInVisibleRegion(SomeRandomRectangle rect){
		//check if rect intersects with any of the items in VisibleRegion list
		Coordinate p1 = rect.upper_left;
		Coordinate p2;
		p2.x = rect.upper_left.x;
		p2.y = rect.lower_right.y;
		Coordinate p3;
		p3.x = rect.lower_right.x;
		p3.y = rect.upper_left.y;
		Coordinate p4 = rect.lower_right;

		int itr;

		for (itr = 0; itr < visibleSegment.size(); itr++)
		{
			//if rect is in the region (fully or partially) of triangle formed by querypoint, VisibleRegion.p1,VisibleRegion.p2 
			//return true

			if (intersect(coord, visibleSegment[itr].p1_overT, p1, p2)) return true;
			else if (intersect(coord, visibleSegment[itr].p1_overT, p1, p3)) return true;
			else if (intersect(coord, visibleSegment[itr].p1_overT, p2, p4)) return true;
			else if (intersect(coord, visibleSegment[itr].p1_overT, p3, p4)) return true;

			else if (intersect(coord, visibleSegment[itr].p2_overT, p1, p2)) return true;
			else if (intersect(coord, visibleSegment[itr].p2_overT, p1, p3)) return true;
			else if (intersect(coord, visibleSegment[itr].p2_overT, p2, p4)) return true;
			else if (intersect(coord, visibleSegment[itr].p2_overT, p3, p4)) return true;

			else if (intersect(visibleSegment[itr].p2_overT, visibleSegment[itr].p1_overT, p1, p2)) return true;
			else if (intersect(visibleSegment[itr].p2_overT, visibleSegment[itr].p1_overT, p1, p3)) return true;
			else if (intersect(visibleSegment[itr].p2_overT, visibleSegment[itr].p1_overT, p2, p4)) return true;
			else if (intersect(visibleSegment[itr].p2_overT, visibleSegment[itr].p1_overT, p3, p4)) return true;

			//is rect fully inside the visible region
			//else if 
			else if (IsPointInsideTriangle(coord, visibleSegment[itr].p1_overT, visibleSegment[itr].p2_overT, p1) == true)//rect er jeno ekta point (p1 here) check korlei hobe, jehetu ager else if gula check kora ache
			{
				return true;
			}
			else return false;

		}
		return false;

	}

	bool  del_from_ObstacleList(SomeRandomRectangle rect);

	//IsInVisibleRegion true hole oi obstacle er jonno update korte hobe
	void update_visibliliyRegion(SomeRandomRectangle obstacle, SomeRandomRectangle target)
	{
		Coordinate obstacle_p1 = obstacle.upper_left;
		Coordinate obstacle_p2(obstacle.upper_left.x, obstacle.lower_right.y);
		Coordinate obstacle_p3(obstacle.lower_right.x, obstacle.upper_left.y);
		Coordinate obstacle_p4 = obstacle.lower_right;

		Coordinate target_p1 = target.upper_left;
		Coordinate target_p2(target.upper_left.x, target.lower_right.y);
		Coordinate target_p3(target.lower_right.x, target.upper_left.y);
		Coordinate target_p4 = target.lower_right;

		bool p1_seen, p2_seen, p3_seen, p4_seen, p1_overT_changed, p2_overT_changed;
		p1_seen = p2_seen = p3_seen = p4_seen = p1_overT_changed = p2_overT_changed = false;

		int itr;
		Coordinate temp_p;
		Coordinate& intersectPoint = temp_p;

		Coordinate& intersectPoint1 = temp_p; Coordinate& intersectPoint2 = temp_p;

		//construct temp with the changed v. regions. finally make visibleRegion=temp
		vector<VisibleRegionOverT> temp;

		for (itr = 0; itr < visibleSegment.size(); itr++)
		{
			//which part of the v.region is changed
			p1_overT_changed = isPointOf_VR_changed((visibleSegment[itr]).p1_overT, coord, obstacle_p1, obstacle_p2, obstacle_p3, obstacle_p4);
			p2_overT_changed = isPointOf_VR_changed((visibleSegment[itr]).p2_overT, coord, obstacle_p1, obstacle_p2, obstacle_p3, obstacle_p4);

			bool entirely_inside_region = false;
			if (p2_overT_changed == false && p1_overT_changed == false
				&& IsPointInsideTriangle(coord, (visibleSegment[itr]).p1_overT, (visibleSegment[itr]).p2_overT, obstacle_p1) == true)//rect er jeno ekta point (p1 here) check korlei hobe, jehetu ager else if gula check kora ache
			{
				entirely_inside_region = true;
			}

			bool one_side_changed = false;
			Coordinate unchanged_side_val;
			Coordinate changed_side_val;
			bool insert_this_segment = true;

			if (p1_overT_changed == true && p2_overT_changed == true)//totally blocked this visibilitysegment
			{
				insert_this_segment = false;
				total_visibility -= visibleSegment[itr].partial_visibility;
			}


			else if (entirely_inside_region == true)
			{
				bool intersectPoint1_found, intersectPoint2_found;
				intersectPoint1_found = intersectPoint2_found = false;

				//split the region

				if (intersect(coord, obstacle_p1, obstacle_p3, obstacle_p4) == false &&
					intersect(coord, obstacle_p1, obstacle_p2, obstacle_p4) == false)//doesn't intersect with own arm, for point obstacle_p1
				{
					bool check = if_intersects_target(coord, obstacle_p1, target, intersectPoint1, (visibleSegment[itr]).p1_overT, (visibleSegment[itr]).p2_overT);
					if (check == true)
					{
						intersectPoint1_found = true;
					}


				}

				if (intersect(coord, obstacle_p2, obstacle_p1, obstacle_p3) == false &&
					intersect(coord, obstacle_p2, obstacle_p3, obstacle_p4) == false)//doesn't intersect with own arm, for point obstacle_p2
				{
					bool check;
					if (intersectPoint1_found == false)
					{
						check = if_intersects_target(coord, obstacle_p2, target, intersectPoint1, (visibleSegment[itr]).p1_overT, (visibleSegment[itr]).p2_overT);
						if (check == true)
						{
							intersectPoint1_found = true;
						}
					}

					else
					{
						check = if_intersects_target(coord, obstacle_p2, target, intersectPoint2, (visibleSegment[itr]).p1_overT, (visibleSegment[itr]).p2_overT);
						if (check == true)
						{
							intersectPoint2_found = true;
						}
					}


				}

				if (intersect(coord, obstacle_p3, obstacle_p2, obstacle_p4) == false &&
					intersect(coord, obstacle_p3, obstacle_p1, obstacle_p2) == false)//doesn't intersect with own arm, for point obstacle_p3
				{
					bool check;
					if (intersectPoint1_found == false)
					{
						check = if_intersects_target(coord, obstacle_p3, target, intersectPoint1, (visibleSegment[itr]).p1_overT, (visibleSegment[itr]).p2_overT);
						if (check == true)
						{
							intersectPoint1_found = true;
						}
					}

					else if (intersectPoint2_found == false)
					{
						check = if_intersects_target(coord, obstacle_p3, target, intersectPoint2, (visibleSegment[itr]).p1_overT, (visibleSegment[itr]).p2_overT);
						if (check == true)
						{
							intersectPoint2_found = true;
						}
					}


				}

				if (intersect(coord, obstacle_p4, obstacle_p1, obstacle_p2) == false &&
					intersect(coord, obstacle_p4, obstacle_p1, obstacle_p3) == false)//doesn't intersect with own arm, for point obstacle_p4
				{
					bool check;
					if (intersectPoint1_found == false)
					{
						check = if_intersects_target(coord, obstacle_p4, target, intersectPoint1, (visibleSegment[itr]).p1_overT, (visibleSegment[itr]).p2_overT);
						if (check == true)
						{
							intersectPoint1_found = true;
						}
					}

					else if (intersectPoint2_found == false)
					{
						check = if_intersects_target(coord, obstacle_p4, target, intersectPoint2, (visibleSegment[itr]).p1_overT, (visibleSegment[itr]).p2_overT);
						if (check == true)
						{
							intersectPoint2_found = true;
						}
					}
				}

				VisibleRegionOverT v;
				if (Distance(visibleSegment[itr].p1_overT, intersectPoint1) < Distance(visibleSegment[itr].p1_overT, intersectPoint2))
				{
					// edit visibility and construct new segments
					total_visibility -= visibleSegment[itr].partial_visibility;
					Coordinate mid1; double angle_with_rect; double visibility_reduce_factor;
					if (intersectPoint1_found == true)
					{
						v.p1_overT = visibleSegment[itr].p1_overT;
						v.p2_overT = intersectPoint1;

						mid1 = mid_of_line_segment(v.p1_overT, v.p2_overT); //rectangle target, so p1,p2 length == p3,p4 length 
						angle_with_rect = angle_between_2_lines(mid1.x, mid1.y, coord.x, coord.y, v.p1_overT.x, v.p1_overT.y, v.p2_overT.x, v.p2_overT.y);//angle between the line connecting query point and mid of the p1,p2 and the line p1,p2
						visibility_reduce_factor = reduced_visibility_factor4angle(angle_with_rect);

						v.partial_visibility = Distance(visibleSegment[itr].p1_overT, intersectPoint1)*visibility_reduce_factor;
						total_visibility += v.partial_visibility;
						temp.push_back(v);
					}
					if (intersectPoint2_found == true)
					{
						v.p1_overT = visibleSegment[itr].p2_overT;
						v.p2_overT = intersectPoint2;

						mid1 = mid_of_line_segment(v.p1_overT, v.p2_overT);
						angle_with_rect = angle_between_2_lines(mid1.x, mid1.y, coord.x, coord.y, v.p1_overT.x, v.p1_overT.y, v.p2_overT.x, v.p2_overT.y);//angle between the line connecting query point and mid of the p1,p2 and the line p1,p2
						visibility_reduce_factor = reduced_visibility_factor4angle(angle_with_rect);

						v.partial_visibility = Distance(visibleSegment[itr].p2_overT, intersectPoint2)*visibility_reduce_factor;

						total_visibility += v.partial_visibility;
						temp.push_back(v);
					}
				}

				else
				{
					// edit visibility and construct new segments
					total_visibility -= visibleSegment[itr].partial_visibility;
					Coordinate mid1; double angle_with_rect; double visibility_reduce_factor;
					if (intersectPoint1_found == true)
					{
						v.p1_overT = visibleSegment[itr].p1_overT;
						v.p2_overT = intersectPoint2;

						mid1 = mid_of_line_segment(v.p1_overT, v.p2_overT); //rectangle target, so p1,p2 length == p3,p4 length 
						angle_with_rect = angle_between_2_lines(mid1.x, mid1.y, coord.x, coord.y, v.p1_overT.x, v.p1_overT.y, v.p2_overT.x, v.p2_overT.y);//angle between the line connecting query point and mid of the p1,p2 and the line p1,p2
						visibility_reduce_factor = reduced_visibility_factor4angle(angle_with_rect);

						v.partial_visibility = Distance(visibleSegment[itr].p1_overT, intersectPoint2)*visibility_reduce_factor;

						total_visibility += v.partial_visibility;

						temp.push_back(v);
					}
					if (intersectPoint1_found == true)
					{
						v.p1_overT = visibleSegment[itr].p2_overT;
						v.p2_overT = intersectPoint1;
						mid1 = mid_of_line_segment(v.p1_overT, v.p2_overT); //rectangle target, so p1,p2 length == p3,p4 length 
						angle_with_rect = angle_between_2_lines(mid1.x, mid1.y, coord.x, coord.y, v.p1_overT.x, v.p1_overT.y, v.p2_overT.x, v.p2_overT.y);//angle between the line connecting query point and mid of the p1,p2 and the line p1,p2
						visibility_reduce_factor = reduced_visibility_factor4angle(angle_with_rect);

						v.partial_visibility = Distance(visibleSegment[itr].p2_overT, intersectPoint1)*visibility_reduce_factor;

						total_visibility += v.partial_visibility;
						temp.push_back(v);
					}
				}

				insert_this_segment = false;

			}//end of inside_region elseif 



			//one value change

			else if (p1_overT_changed == true && p2_overT_changed == false)
			{
				one_side_changed = true;
				unchanged_side_val = (visibleSegment[itr]).p2_overT;
				changed_side_val = (visibleSegment[itr]).p1_overT;
			}

			else if (p2_overT_changed == true && p1_overT_changed == false)
			{
				one_side_changed = true;
				unchanged_side_val = (visibleSegment[itr]).p1_overT;
				changed_side_val = (visibleSegment[itr]).p2_overT;
			}

			if (one_side_changed == true)
			{
				//intersectPoint is initialized in change_vRegion_one_side(...) function
				if (intersect(coord, obstacle_p1, obstacle_p3, obstacle_p4) == false &&
					intersect(coord, obstacle_p1, obstacle_p2, obstacle_p4) == false)//doesn't intersect with own arm, for point obstacle_p1
				{
					// is position-obstacle_p1 line intersects target? 
					//if so, is the point nearer the unchanged_side_val than the changed_side_val?

					p1_seen = change_vRegion_one_side(coord, obstacle_p1, target, unchanged_side_val, changed_side_val, intersectPoint,
						visibleSegment[itr]);
					//if so, then change the value of the v.region
					if (p1_seen)
					{
						visibleSegment[itr].p1_overT = unchanged_side_val;
						visibleSegment[itr].p2_overT = intersectPoint;
						insert_this_segment = true;

						total_visibility -= visibleSegment[itr].partial_visibility;

						Coordinate mid1 = mid_of_line_segment(visibleSegment[itr].p1_overT, visibleSegment[itr].p2_overT);
						double angle_with_rect = angle_between_2_lines(mid1.x, mid1.y, coord.x, coord.y, visibleSegment[itr].p1_overT.x, visibleSegment[itr].p1_overT.y, visibleSegment[itr].p2_overT.x, visibleSegment[itr].p2_overT.y);//angle between the line connecting query point and mid of the p1,p2 and the line p1,p2
						double visibility_reduce_factor = reduced_visibility_factor4angle(angle_with_rect);

						visibleSegment[itr].partial_visibility = Distance(unchanged_side_val, intersectPoint)*visibility_reduce_factor;
						total_visibility += visibleSegment[itr].partial_visibility;
					}

				}

				if (p1_seen == false && intersect(coord, obstacle_p2, obstacle_p1, obstacle_p3) == false &&
					intersect(coord, obstacle_p2, obstacle_p3, obstacle_p4) == false)//doesn't intersect with own arm, for point obstacle_p2
				{
					p2_seen = change_vRegion_one_side(coord, obstacle_p2, target, unchanged_side_val, changed_side_val, intersectPoint,
						visibleSegment[itr]);
					if (p2_seen)
					{
						visibleSegment[itr].p1_overT = unchanged_side_val;
						visibleSegment[itr].p2_overT = intersectPoint;
						insert_this_segment = true;

						total_visibility -= visibleSegment[itr].partial_visibility;
						Coordinate mid1 = mid_of_line_segment(visibleSegment[itr].p1_overT, visibleSegment[itr].p2_overT);
						double angle_with_rect = angle_between_2_lines(mid1.x, mid1.y, coord.x, coord.y, visibleSegment[itr].p1_overT.x, visibleSegment[itr].p1_overT.y, visibleSegment[itr].p2_overT.x, visibleSegment[itr].p2_overT.y);//angle between the line connecting query point and mid of the p1,p2 and the line p1,p2
						double visibility_reduce_factor = reduced_visibility_factor4angle(angle_with_rect);

						visibleSegment[itr].partial_visibility = Distance(unchanged_side_val, intersectPoint)* visibility_reduce_factor;
						total_visibility += visibleSegment[itr].partial_visibility;
					}

				}

				if (p1_seen == false && p2_seen == false && intersect(coord, obstacle_p3, obstacle_p2, obstacle_p4) == false &&
					intersect(coord, obstacle_p3, obstacle_p1, obstacle_p2) == false)//doesn't intersect with own arm, for point obstacle_p3
				{
					p3_seen = change_vRegion_one_side(coord, obstacle_p3, target, unchanged_side_val, changed_side_val, intersectPoint,
						visibleSegment[itr]);
					if (p3_seen)
					{
						visibleSegment[itr].p1_overT = unchanged_side_val;
						visibleSegment[itr].p2_overT = intersectPoint;
						insert_this_segment = true;

						total_visibility -= visibleSegment[itr].partial_visibility;
						Coordinate mid1 = mid_of_line_segment(visibleSegment[itr].p1_overT, visibleSegment[itr].p2_overT);
						double angle_with_rect = angle_between_2_lines(mid1.x, mid1.y, coord.x, coord.y, visibleSegment[itr].p1_overT.x, visibleSegment[itr].p1_overT.y, visibleSegment[itr].p2_overT.x, visibleSegment[itr].p2_overT.y);//angle between the line connecting query point and mid of the p1,p2 and the line p1,p2
						double visibility_reduce_factor = reduced_visibility_factor4angle(angle_with_rect);

						visibleSegment[itr].partial_visibility = Distance(unchanged_side_val, intersectPoint)*visibility_reduce_factor;
						total_visibility += visibleSegment[itr].partial_visibility;
					}
				}

				if (p1_seen == false && p2_seen == false && p3_seen == false && intersect(coord, obstacle_p4, obstacle_p1, obstacle_p2) == false &&
					intersect(coord, obstacle_p4, obstacle_p1, obstacle_p3) == false)//doesn't intersect with own arm, for point obstacle_p4
				{
					p4_seen = change_vRegion_one_side(coord, obstacle_p4, target, unchanged_side_val, changed_side_val, intersectPoint,
						visibleSegment[itr]);
					if (p4_seen)
					{
						visibleSegment[itr].p1_overT = unchanged_side_val;
						visibleSegment[itr].p2_overT = intersectPoint;
						insert_this_segment = true;

						total_visibility -= visibleSegment[itr].partial_visibility;
						Coordinate mid1 = mid_of_line_segment(visibleSegment[itr].p1_overT, visibleSegment[itr].p2_overT);
						double angle_with_rect = angle_between_2_lines(mid1.x, mid1.y, coord.x, coord.y, visibleSegment[itr].p1_overT.x, visibleSegment[itr].p1_overT.y, visibleSegment[itr].p2_overT.x, visibleSegment[itr].p2_overT.y);//angle between the line connecting query point and mid of the p1,p2 and the line p1,p2
						double visibility_reduce_factor = reduced_visibility_factor4angle(angle_with_rect);

						visibleSegment[itr].partial_visibility = Distance(unchanged_side_val, intersectPoint)*visibility_reduce_factor;
						total_visibility += visibleSegment[itr].partial_visibility;
					}
				}
			}//end of if one side


			if (insert_this_segment == true) temp.push_back(visibleSegment[itr]);
		}
		visibleSegment = temp;
	}

	double visibiliy(SomeRandomRectangle Target);

	//jokhon kono obstacle nai consider kore
	double init_visibility(SomeRandomRectangle Target)//considering no obstacle
	{
		Coordinate p1 = Target.upper_left;
		Coordinate p2(Target.upper_left.x, Target.lower_right.y);
		Coordinate p3(Target.lower_right.x, Target.upper_left.y);
		Coordinate p4 = Target.lower_right;

		bool p1_seen, p2_seen, p3_seen, p4_seen;
		p1_seen = p2_seen = p3_seen = p4_seen = false;

		total_visibility = 0.0;
		//initially kon kon side dekhte pay seta check korte hobe

		//check for line SEGMENT intersect 
		if (intersect(coord, p1, p2, p4) == false && intersect(coord, p1, p3, p4) == false) p1_seen = true;
		if (intersect(coord, p4, p1, p2) == false && intersect(coord, p4, p1, p3) == false) p4_seen = true;

		if (intersect(coord, p2, p3, p4) == false && intersect(coord, p2, p1, p3) == false) p2_seen = true;
		if (intersect(coord, p3, p1, p2) == false && intersect(coord, p3, p2, p4) == false) p3_seen = true;

		Coordinate mid1, point1, point2;

		double angle_with_rect = 0.0;
		double visibility_reduce_factor = 1.0;
		double partial_visibility = 0.0;

		if ((p1_seen == true && p2_seen == true) || (p3_seen == true && p4_seen == true))
		{
			if (p1_seen == true && p2_seen == true){
				point1 = p1; point2 = p2;
			}
			else{		// if p1,p2 is seen, p3,p4 cannot be seen
				point1 = p3; point2 = p4;
			}

			mid1 = mid_of_line_segment(point1, point2); //rectangle target, so p1,p2 length == p3,p4 length 
			angle_with_rect = angle_between_2_lines(mid1.x, mid1.y, coord.x, coord.y, point1.x, point1.y, point2.x, point2.y);//angle between the line connecting query point and mid of the p1,p2 and the line p1,p2
			visibility_reduce_factor = reduced_visibility_factor4angle(angle_with_rect);

			partial_visibility = Distance(point1, point2) * visibility_reduce_factor;
			total_visibility += partial_visibility;

			//construct the visible region list
			Coordinate a;
			struct VisibleRegionOverT segment;
			segment.p1_overT = point1;
			segment.p2_overT = point2;
			segment.partial_visibility = partial_visibility;
			visibleSegment.push_back(segment);
		}


		if ((p1_seen == true && p3_seen == true) || (p2_seen == true && p4_seen == true))
		{
			if (p1_seen == true && p3_seen == true){
				point1 = p1; point2 = p3;
			}
			else{		// if p1,p3 is seen, p2,p4 cannot be seen
				point1 = p2; point2 = p4;
			}

			mid1 = mid_of_line_segment(point1, point2); //rectangle target, so p1,p2 length == p3,p4 length 
			angle_with_rect = angle_between_2_lines(mid1.x, mid1.y, coord.x, coord.y, point1.x, point1.y, point2.x, point2.y);//angle between the line connecting query point and mid of the p1,p2 and the line p1,p2
			visibility_reduce_factor = reduced_visibility_factor4angle(angle_with_rect);

			partial_visibility = Distance(point1, point2) * visibility_reduce_factor;
			total_visibility += partial_visibility;

			//construct the visible region list
			Coordinate a;
			struct VisibleRegionOverT segment;
			segment.p1_overT = point1;
			segment.p2_overT = point2;
			segment.partial_visibility = partial_visibility;
			visibleSegment.push_back(segment);
		}

		return total_visibility;
	}


	QueryPoint(){}

	QueryPoint(Coordinate coordinate, Vector3d direction)
	{
		this->coord = coordinate;
		this->direction = direction;
		this->vr = buildVR(coordinate,direction);
	}

	void readQueryPoint(){
		scanf_s("%lf %lf %lf %lf %lf %lf", &coord.x, &coord.y, &coord.z, &direction.x, &direction.y, &direction.z);
		vr = buildVR(coord, direction);
			
	}

	VisibleRegion buildVR(Coordinate coordinate,Vector3d direction){
		
		double theta = FOV / 2;
		theta = theta*3.1416 / 180;
		VisibleRegion vr;
		vr.c = coordinate;//set the query point first

		direction = direction.normalize();
		// multiply with 2d rotation matrix where theta=fov/2;
		//this is code for 2D ... no idea what to do for 3D


		//calculation for first point
		Vector3d newDir;
		newDir.x = direction.x*cos(theta) - direction.y*sin(theta);
		newDir.y = direction.x*sin(theta) + direction.y*cos(theta);
		newDir.z = 0;
		Ray *ray;
		ray = new Ray(coordinate, newDir);

		vr.a = ray->getPointAtDistance(D);

		//second point .. negative rotation
		newDir.x = direction.x*cos(-theta) - direction.y*sin(-theta);
		newDir.y = direction.x*sin(-theta) + direction.y*cos(-theta);
		newDir.z = 0;
		ray = new Ray(coordinate, newDir);
		vr.b = ray->getPointAtDistance(D);

		return vr;
	}

	int operator<(const QueryPoint & b)const{

		return total_visibility > b.total_visibility;

	}




};

#endif