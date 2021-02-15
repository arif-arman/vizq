#include <sstream>
#include <iostream>
#include <iomanip>
#include <cstdio>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <fstream>
#include <cstring>
#include <queue>
#include <cassert>

#include "vcm.h"

#include<GL/glut.h>
#include<GL/glui.h>

#include "camera.h"
#include "myCamera.h"
#include "macros.h"
#include "gpc.h"
#include "monotone.h"
#include "text3d.h"

//R-tree stuffs
#include "./rtree/rtree.h"
#include "./rtree/rtnode.h"
#include "./rtree/entry.h"
#include "./blockfile/blk_file.h"
#include "./blockfile/cache.h"
#include "./linlist/linlist.h"
#include "./rtree/rtree_cmd.h"
#include "./rtree/distance.h"
#include "./his/histogram.h"
#include "tinyfiledialogs.h"

VCM::VCM() {
	fonttodist[8] = 60;
	fonttodist[10] = 70;
	fonttodist[12] = 80;
	fonttodist[14] = 90;
	fonttodist[16] = 100;
	fonttodist[18] = 110;
	fonttodist[20] = 120;
	fonttodist[22] = 130;
	fonttodist[24] = 140;
	fonttodist[26] = 150;
	fonttodist[28] = 160;
	fonttodist[30] = 170;
	fonttodist[32] = 180;
	fonttodist[34] = 190;
	fonttodist[36] = 200;
	fonttodist[38] = 210;
	fonttodist[40] = 220;

}

char * VCM::console_strings[7] = { " ", "Loading Database ... ", "Database Loaded!", "Target Set", "Target Not Set", "Generating VCM ... ",
"VCM Generated!\nRed: Obstacle not in direction\nGreen: Obstacle at near distance\nYellow: Obstacle out of projection plane\nBlue: Hidden by other obstacles" };

const char * VCM::STRS[4] = { "Video", "Tutorials", "Rock", ".com" };

const char * VCM::string_list[6] = { "-X", "+X", "-Y", "+Y", "-Z", "+Z" };
const char * VCM::lFilterPatterns[2] = { "*.txt", "*.text" };

vector<Box> VCM::obstacles;
map< int, double > VCM::dstFromTar;


// calculate projection of a Box
bool VCM::calcProjectionWrtPoint(Plane plane, Vector3 c, Box tb, Box &pr, int dir)
{
	/*

	calculate projection of tb on plane wrt c store in pr
	tb is the a surface of Box T
	we are getting a single Box pr from projRect and assign it's corner points to +INF and -INF

	*/

	vector<Vector>projPoints;

	int i1, j1, k1;

	pr.x[0] = pr.y[0] = pr.z[0] = +INF;
	pr.x[1] = pr.y[1] = pr.z[1] = -INF;

	//bool ret=false;
	for (i1 = 0; i1<2; i1++)
	for (j1 = 0; j1<2; j1++)
	for (k1 = 0; k1<2; k1++)
	{
		Vector3 pp;

		Vector3 p(tb.x[i1], tb.y[j1], tb.z[k1]);
		/* take all 8 points of tb in p and project in pp */
		if (!plane.caclProjectionOfPoint(c, p, pp))return false;

		//ret=true;
		/* min/max nicchi because initially +INF -INF dhora hoyechilo */
		pr.x[0] = min(pr.x[0], pp.x);
		pr.y[0] = min(pr.y[0], pp.y);
		pr.z[0] = min(pr.z[0], pp.z);

		pr.x[1] = max(pr.x[1], pp.x);
		pr.y[1] = max(pr.y[1], pp.y);
		pr.z[1] = max(pr.z[1], pp.z);

	}

	return true;


}
/* for target point c, calculate projection of points of obstacle tb */
bool VCM::calcCovexProjectionWrtPoint(Plane plane, Vector3 c, Box tb, gpc_polygon &g, int dir)
{
	/*
	calculate projection of tb on plane wrt c store in pr
	*/

	vector<Vector>projPoints;

	int i1, j1, k1;


	/*

	pr.x[0]=pr.y[0]=pr.z[0]=+INF;
	pr.x[1]=pr.y[1]=pr.z[1]=-INF;
	*/
	//bool ret=false;
	for (i1 = 0; i1<2; i1++)
	for (j1 = 0; j1<2; j1++)
	for (k1 = 0; k1<2; k1++)
	{
		Vector3 pp;

		Vector3 p(tb.x[i1], tb.y[j1], tb.z[k1]);
		/* if projection of some point on obstacle goes out of plane, return false */
		if (!plane.caclProjectionOfPoint(c, p, pp))return false;

		//ret=true;


		/*
		pr.x[0]=min( pr.x[0],pp.x );
		pr.y[0]=min( pr.y[0],pp.y );
		pr.z[0]=min( pr.z[0],pp.z );


		pr.x[1]=max( pr.x[1],pp.x );
		pr.y[1]=max( pr.y[1],pp.y );
		pr.z[1]=max( pr.z[1],pp.z );

		*/

		projPoints.push_back(Adapter.adapt32(dir, pp.x, pp.y, pp.z));


	}
	/* obstacle er shob projected point er convex hull tai final projection */
	vector<Vector> pr = convexHull(projPoints);

	Adapter.adapt(pr, g);

	return true;


}
bool VCM::calcProjectionWrtCube(Plane plane, Box cb, Box pb, Box &pr) //project a plane wrt a box , projected plane is supposed to be axix alingned
{

	/*
	calculate projection of pb on plane wrt cb store in pr
	*/

	//cb.print();
	//cp.print();

	int i, j, k;
	int i1, j1, k1;




	pr.x[0] = pr.y[0] = pr.z[0] = +INF;
	pr.x[1] = pr.y[1] = pr.z[1] = -INF;

	bool ret = false;

	for (i = 0; i<2; i++)
	for (j = 0; j<2; j++)
	for (k = 0; k<2; k++)
	for (i1 = 0; i1<2; i1++)
	for (j1 = 0; j1<2; j1++)
	for (k1 = 0; k1<2; k1++)
	{
		Vector3 pp;
		Vector3 c(cb.x[i], cb.y[j], cb.z[k]);
		Vector3 p(pb.x[i1], pb.y[j1], pb.z[k1]);

		if (!plane.caclProjectionOfPoint(c, p, pp))return false;

		ret = true;

		pr.x[0] = min(pr.x[0], pp.x);
		pr.y[0] = min(pr.y[0], pp.y);
		pr.z[0] = min(pr.z[0], pp.z);


		pr.x[1] = max(pr.x[1], pp.x);
		pr.y[1] = max(pr.y[1], pp.y);
		pr.z[1] = max(pr.z[1], pp.z);

	}


	return ret;

}
bool VCM::calcConvexProjectionWrtCube(Plane plane, Box cb, Box tb, gpc_polygon &global_p, int dir, bool Union) //project a plane wrt a box , projected plane is supposed to be axix alingned
{

	/*
	calculate projection of tb on plane wrt cb store in pr
	*/

	//cb.print();
	//cp.print();

	int i, j, k;
	int i1, j1, k1;

	bool ret = false;

	global_p.contour = 0;
	bool start = 0;
	for (i = 0; i<2; i++)
	for (j = 0; j<2; j++)
	for (k = 0; k<2; k++)
	{
		Vector3 pp;
		Vector3 c(cb.x[i], cb.y[j], cb.z[k]);
		//				Vector3 p( tb.x[i1],tb.y[j1],tb.z[k1] );


		gpc_polygon local_p;

		if (!calcCovexProjectionWrtPoint(plane, c, tb, local_p, dir))return false;



		//				calcCovexProjectionWrtPoint(plane,c,tb,local_p );
		//				Adapter.adapt( v,local_p );

		if (!start)
		{
			global_p = local_p;
			start = 1;
			continue;
		}


		if (!Union)Adapter.polygonIntersection(global_p, local_p);
		else Adapter.polygonUnion(global_p, local_p);

	}


	return true;
}



// stores segment distances from T
void VCM::calcSegmentDistance(int dir)
{
	segDist.clear();
	double da = ANGRES;
	int n = 90.0 / da;
	double a = 90;
	if (dir <= 1) {
		segDist.push_back(target.lx / 2);
		for (int i = 1; i<n; i++)
		{
			a -= da;
			segDist.resize(segDist.size() + 1);
			segDist[i] = target.lx / 2;
			segDist[i] += (target.lx / 2.0) / (tan(a*acos(-1) / 180.0));
			//cout<<segDist[i]<<endl;
		}
	}
	else if (dir == 2 || dir == 3) {
		segDist.push_back(target.ly / 2);
		for (int i = 1; i<n; i++)
		{
			a -= da;
			segDist.resize(segDist.size() + 1);
			segDist[i] = target.ly / 2;
			segDist[i] += (target.ly / 2.0) / (tan(a*acos(-1) / 180.0));
			//cout<<segDist[i]<<endl;
		}
	}
	else {
		segDist.push_back(target.lz / 2);
		for (int i = 1; i<n; i++)
		{
			a -= da;
			segDist.resize(segDist.size() + 1);
			segDist[i] = target.lz / 2;
			segDist[i] += (target.lz / 2.0) / (tan(a*acos(-1) / 180.0));
			//cout<<segDist[i]<<endl;
		}
	}

	/*
	segDist=vector<double>(NDISTESEG); // this will be calculated based on properties of eye
	segDist[0]=target.lx/2+1;
	for( int i=1;i<NDISTESEG;i++ )segDist[i]=segDist[i-1]+INTSEGDIST;	// for debug purpose only
	*/

	NDISTESEG = n;
	//	while(1);

}

// ekta problem ase...
// coordinate system k target cube er center e nia jabo

void VCM::calcProjectionPlanes()
{
	/* creates temp array that holds 6 planes of Target stored as Box */
	Box btemp[] = { target.bx[0], target.bx[1], target.by[0], target.by[1], target.bz[0], target.bz[1] };
	for (int i = 0; i<totDir; i++) dirBox[i] = btemp[i];
	/* adds the unit vectors */
	Vector3 tempv[] = { Vector3(-1, 0, 0), Vector3(+1, 0, 0), Vector3(0, -1, 0), Vector3(0, +1, 0), Vector3(0, 0, -1), Vector3(0, 0, +1) };
	for (int i = 0; i<totDir; i++) unitDir[i] = tempv[i];
	/* translates the projection plane to center of cube */
	double transfactor[] = { target.o.x, target.o.x, target.o.y, target.o.y, target.o.z, target.o.z }; // we have to translate the plane centering at cube center
	Plane projPlane;
	/* 6ta direction, by unit vector*/
	for (int dir = 0; dir<totDir; dir++)
	{
		/* number of segments => koto dure dure projection plane create hobe */
		for (int curSeg = 0; curSeg<NDISTESEG; curSeg++)
		{
			projPlane = Plane(unitDir[dir].x, unitDir[dir].y, unitDir[dir].z, segDist[curSeg]);
			calcProjectionWrtPoint(projPlane, target.o, dirBox[dir], projRect[dir][curSeg], dir);
		}
	}
}

void VCM::update(int dir, int curSeg)
{
	vector<int>&id = dirObstacleId[dir];


	gProjPolys[dir][curSeg].num_contours = 0;

	for (int i = 0; i<id.size(); i++)
	{
		gpc_polygon local_p;
		Plane projPlane = Plane(unitDir[dir].x, unitDir[dir].y, unitDir[dir].z, segDist[curSeg]);
		calcConvexProjectionWrtCube(projPlane, target, obstacles[id[i]], local_p, dir);
		Adapter.polygonMerge(gProjPolys[dir][curSeg], local_p);
	}


}

// dir,curseg,point id on target

void VCM::updateVcm(vector<int>&obstacleWrtPoint, int dir, int curSeg, int tid)
{

	vector<int>&id = obstacleWrtPoint;
	Vector3 c = targetPoints[tid];
	gpc_polygon &g = gVcmProjPolys[dir][curSeg][tid];


	g.num_contours = 0;


	for (int i = 0; i<id.size(); i++)
	{
		gpc_polygon local_p;
		Plane projPlane = Plane(unitDir[dir].x, unitDir[dir].y, unitDir[dir].z, segDist[curSeg] + c.x);
		calcCovexProjectionWrtPoint(projPlane, c, obstacles[id[i]], local_p, dir);
		Adapter.polygonMerge(g, local_p);
	}


}


void VCM::reductionInNegX(int dir) //dir=0
{

	//puts("reduction in -x direction...");
	int discarded = 0;

	priority_queue<int, vector<int>, decltype(&CompareHiX)> pq(&CompareHiX);
	//priority_queue<int, vector<int>, CompareHiX> pq;

	// jei shob obstacle er target er respect e right  e thake
	bool bdir = dir & 1;
	for (int i = 0; i < obstacles.size(); i++) {
		if (obstacles[i].x[bdir ^ 1] <= target.x[bdir])
			pq.push(i); //change here ofr direction
		else {
			obstacles[i].color = Vector3(RED);
			//printf("discarding %d not in -x direction\n", i);
			discarded++;
		}
	}


	//cout<<"no of obstacles in negative x direction"<<endl;
	//cout<<pq.size()<<endl;


	//ei direction e total kotogula distance segment lagbe.. eitar ekta max limit ase
	// seita cross korbe na but jodi obstacle sesh hoia jay taileo r agabe na amar code

	int &curSeg = totSeg[dir];
	curSeg = 0;


	while (!pq.empty())
	{
		int u = pq.top();
		pq.pop();

		//	cout<<u<<" ";

		while (curSeg<NDISTESEG - 1 && target.x[dir] - segDist[curSeg] >= obstacles[u].x[dir]) //change here
		{
			curSeg++;
			update(dir, curSeg); // eikhane optimize korbo aro...
			Adapter.adapt(dir, -segDist[curSeg], gProjPolys[dir][curSeg], ProjPolys[dir][curSeg]); //change here
		}


		Box projPoly;

		Plane projPlane = Plane(-1, 0, 0, fabs(-segDist[curSeg] + target.o.x)); //change here for direction
		Box boundRect = projRect[dir][curSeg];
		if (!calcProjectionWrtCube(projPlane, target, obstacles[u], projPoly))
		{
			obstacles[u].color = Vector3(GREEN);
			//printf("discarding id %d because of near distance\n", u);
			discarded++;
			continue;
		}
		Box projPoly1;
		if (!commonBox(projPoly, boundRect, projPoly1))
		{
			obstacles[u].color = Vector3(YELLOW);
			//printf("discarding id %d because of because out of the projection rectangle\n", u);
			discarded++;;
			continue;
		}
		gpc_polygon gProjPoly;
		bool b = calcConvexProjectionWrtCube(projPlane, target, obstacles[u], gProjPoly, dir);
		if (gProjPoly.num_contours == 0)continue;

		//Adapter.adapt( projPoly1,gProjPoly );

		if (Adapter.contains(gProjPolys[dir][curSeg], gProjPoly))
		{
			obstacles[u].color = Vector3(BLUE);
			//printf("discarding id %d view blocked by other obstacles\n", u);
			discarded++;
			continue;
		}

		dirObstacleId[dir].push_back(u);
		dirObstacle[dir].resize(dirObstacle[dir].size() + 1);
		dirObstacle[dir].back() = obstacles[u];
	}

	curSeg = NDISTESEG - 2;
	//printf("Total discarded: %d\n", discarded);
}

void VCM::reductionInNegY(int dir) //dir=2
{
	//puts("reduction in -y direction...");


	priority_queue<int, vector<int>, decltype(&CompareHiY)> pq(&CompareHiY);
	//priority_queue<int, vector<int>, CompareHiY> pq;
	int discarded = 0;
	// jei shob obstacle er target er respect e right  e thake
	bool bdir = dir & 1;
	for (int i = 0; i < obstacles.size(); i++) {
		if (obstacles[i].y[bdir ^ 1] <= target.y[bdir])
			pq.push(i); //change here ofr direction
		else {
			obstacles[i].color = Vector3(RED);
			//printf("discarding %d not in -y direction\n", i);
			discarded++;
		}
	}


	//cout<<"no of obstacles in negative x direction"<<endl;
	//cout<<pq.size()<<endl;


	//ei direction e total kotogula distance segment lagbe.. eitar ekta max limit ase
	// seita cross korbe na but jodi obstacle sesh hoia jay taileo r agabe na amar code

	int &curSeg = totSeg[dir];
	curSeg = 0;


	while (!pq.empty())
	{
		int u = pq.top();
		pq.pop();



		while (curSeg<NDISTESEG - 1 && target.y[0] - segDist[curSeg] >= obstacles[u].y[0]) //change here
		{
			curSeg++;
			update(dir, curSeg); // eikhane optimize korbo aro...
			Adapter.adapt(dir, -segDist[curSeg], gProjPolys[dir][curSeg], ProjPolys[dir][curSeg]); //change here
		}


		Box projPoly;

		Plane projPlane = Plane(CO(unitDir[dir]), fabs(-segDist[curSeg] + target.o.y)); //change here for direction
		Box boundRect = projRect[dir][curSeg];
		if (!calcProjectionWrtCube(projPlane, target, obstacles[u], projPoly))
		{
			obstacles[u].color = Vector3(GREEN);
			discarded++;
			//printf("discarding id %d because of near distance\n", u);
			continue;
		}
		Box projPoly1;
		if (!commonBox(projPoly, boundRect, projPoly1))
		{
			obstacles[u].color = Vector3(YELLOW);
			discarded++;
			//printf("discarding id %d because of because out of the projection rectangle\n", u);
			continue;
		}
		gpc_polygon gProjPoly;
		bool b = calcConvexProjectionWrtCube(projPlane, target, obstacles[u], gProjPoly, dir);
		if (gProjPoly.num_contours == 0)continue;
		//Adapter.adapt( projPoly1,gProjPoly );

		if (Adapter.contains(gProjPolys[dir][curSeg], gProjPoly))
		{
			obstacles[u].color = Vector3(BLUE);
			discarded++;
			//printf("discarding id %d view blocked by other obstacles\n", u);
			continue;
		}

		dirObstacleId[dir].push_back(u);
		dirObstacle[dir].resize(dirObstacle[dir].size() + 1);
		dirObstacle[dir].back() = obstacles[u];
	}

	curSeg = NDISTESEG - 2;
	//printf("Total discarded: %d\n", discarded);
}

void VCM::reductionInNegZ(int dir) //dir=4
{	
	//puts("reduction in -z direction...");
	priority_queue<int, vector<int>, decltype(&CompareHiZ)> pq(&CompareHiZ);
	//priority_queue<int, vector<int>, VCM::CompareHiZ(&this)> pq(&this);
	int discarded = 0;

	// jei shob obstacle er target er respect e right  e thake
	bool bdir = dir & 1;
	for (int i = 0; i < obstacles.size(); i++) {
		if (obstacles[i].z[bdir ^ 1] <= target.z[bdir])
			pq.push(i); //change here ofr direction
		else {
			obstacles[i].color = Vector3(RED);
			//printf("discarding %d not in -z direction\n", i);
			discarded++;
		}
	}


	//cout<<"no of obstacles in negative x direction"<<endl;
	//cout<<pq.size()<<endl;


	//ei direction e total kotogula distance segment lagbe.. eitar ekta max limit ase
	// seita cross korbe na but jodi obstacle sesh hoia jay taileo r agabe na amar code

	int &curSeg = totSeg[dir];
	curSeg = 0;


	while (!pq.empty())
	{
		int u = pq.top();
		pq.pop();



		while (curSeg<NDISTESEG - 1 && target.z[0] - segDist[curSeg] >= obstacles[u].z[0]) //change here
		{
			curSeg++;
			update(dir, curSeg); // eikhane optimize korbo aro...
			Adapter.adapt(dir, -segDist[curSeg], gProjPolys[dir][curSeg], ProjPolys[dir][curSeg]); //change here
		}


		Box projPoly;

		Plane projPlane = Plane(CO(unitDir[dir]), fabs(-segDist[curSeg] + target.o.z)); //change here for direction
		Box boundRect = projRect[dir][curSeg];
		if (!calcProjectionWrtCube(projPlane, target, obstacles[u], projPoly))
		{
			obstacles[u].color = Vector3(GREEN);
			discarded++;
			//printf("discarding id %d because of near distance\n", u);
			continue;
		}
		Box projPoly1;
		if (!commonBox(projPoly, boundRect, projPoly1))
		{
			obstacles[u].color = Vector3(YELLOW);
			discarded++;
			//printf("discarding id %d because of because out of the projection rectangle\n", u);
			continue;
		}
		gpc_polygon gProjPoly;
		bool b = calcConvexProjectionWrtCube(projPlane, target, obstacles[u], gProjPoly, dir);
		if (gProjPoly.num_contours == 0)continue;
		//Adapter.adapt( projPoly1,gProjPoly );

		if (Adapter.contains(gProjPolys[dir][curSeg], gProjPoly))
		{
			obstacles[u].color = Vector3(BLUE);
			discarded++;
			//printf("discarding id %d view blocked by other obstacles\n", u);
			continue;
		}

		dirObstacleId[dir].push_back(u);
		dirObstacle[dir].resize(dirObstacle[dir].size() + 1);
		dirObstacle[dir].back() = obstacles[u];
	}

	curSeg = NDISTESEG - 2;
	//printf("Total discarded: %d\n", discarded);
}



// pq stores in a non decreasing order of distance


void VCM::extedProjToCurLayer(double  d, int dir)
{
	for (int c = 0; c<8; c++)
	{
		gpc_polygon &projPoly = projWrtTargetCorner[c];
		for (int i = 0; i <projPoly.num_contours; i++)
		{
			for (int j = 0; j < projPoly.contour[i].num_vertices; j++)
			{
				double px = curProjPlane[dir];
				double &py = projPoly.contour[i].vertex[j].x;
				double &pz = projPoly.contour[i].vertex[j].y;


				double x1 = px - targetCorner[c].x;
				double y1 = py - targetCorner[c].y;
				double z1 = pz - targetCorner[c].z;

				double x2 = d - targetCorner[c].x;
				double y2 = (y1 / x1)*x2;
				double z2 = (z1 / x1)*x2;


				py = targetCorner[c].y + y2;
				pz = targetCorner[c].z + z2;



			}
		}
	}


}

bool VCM::ncmp(Entry e1, Entry e2)
{
	return e1.bounces[0]<e2.bounces[0];
}
int VCM::countChild(int uson)
{

	return 0;
	int ret = 0;
	queue<int>Q;
	RTNode *rtn = new RTNode(rtree, uson);

	if (rtn->level)Q.push(uson);
	else
	{
		ret++;
		return ret;
	}

	while (!Q.empty())
	{
		int v = Q.front();

		RTNode *rtn = new RTNode(rtree, v);
		Q.pop();


		for (int e = 0; e<rtn->num_entries; e++)
		{
			Entry u = rtn->entries[e];
			if (rtn->level)
			{
				Q.push(u.son);
				continue;
			}
			ret++;
		}


	}

	return ++ret;

}

void VCM::reductionInPosXRtree(int dir)
{

	//	cout<<countChild(rtree->root)<<endl;exit(0);

	fprintf(logfp, "reduction in positive x direction...");
	//printf("reduction in positive x direction...");
	// get 8 corner points of Target
	for (int i1 = 0, id = 0; i1<2; i1++)
	for (int j1 = 0; j1<2; j1++)
	for (int k1 = 0; k1<2; k1++)
	{
		targetCorner[id++] = Vector3(target.x[i1], target.y[j1], target.z[k1]);
	}

	priority_queue< int, vi, decltype(&comparatorForPq) >Q(&comparatorForPq);
	//queue<int>Q;

	Q.push(rtree->root);
	int cnt = 0, discard = 0;
	while (!Q.empty())
	{
		int v = Q.top();
		//int v=Q.front();
		RTNode *rtn = new RTNode(rtree, v);
		Q.pop();


		//	sort( rtn->entries,rtn->entries+rtn->num_entries,ncmp );

		//	sort( rtn->entries,rtn->entries+rtn->num_entries,ncmp );

		for (int e = 0; e<rtn->num_entries; e++)
		{
			Entry u = rtn->entries[e];
			//cout<<u.bounces[0]<<endl;	
			//check can it be pruned..

			//reduction in +ve x direction prune the obstacle 

			//case1 out of view because of location
			Vector3 v0(u.bounces[0] - translate.x, u.bounces[2] - translate.y, u.bounces[4] - translate.z);
			Vector3 v1(u.bounces[1] - translate.x, u.bounces[3] - translate.y, u.bounces[5] - translate.z);
			Box bu;
			bu.init(v0, v1, Origin);


			if (target.x[1] >= u.bounces[1] - translate.x)
			{
				bu.color = Vector3(255, 0, 0);
				//discard+=countChild( u.son );
				All.resize(All.size() + 1);
				All.back() = bu;
				fprintf(logfp, "discarding %d not in +x direction\n", u.son);
				printf("discarding %d not in +x direction\n", u.son);
				//				printf();
				continue;
			}

			/*
			if( u.bounces[0]-target.x[1]>=2000 )
			{
			printf("discarding %d out of view range\n",u.son);
			continue;
			}
			*/
			// case 2 out of view because of projection



			//printf("Target %f Bounce %f\n", target.x[1], u.bounces[1]);

			bu.setPlanes();
			Box projPoly;
			double d = u.bounces[0];
			Plane projPlane = Plane(CO(unitDir[dir]), d); //change here for direction

			Box boundRect;
			calcProjectionWrtPoint(projPlane, target.o, dirBox[dir], boundRect, dir);

			projPoly = bu.bx[0];

			if (projPoly.x[0]>projPoly.x[1])swap(projPoly.x[0], projPoly.x[1]);
			if (projPoly.y[0]>projPoly.y[1])swap(projPoly.y[0], projPoly.y[1]);
			if (projPoly.z[0]>projPoly.z[1])swap(projPoly.z[0], projPoly.z[1]);

			Box projPoly1;
			bool b = commonBox(projPoly, boundRect, projPoly1);

			if (!b)
			{
				bu.color = Vector3(255, 0, 0);
				//discard+=countChild( u.son );
				All.resize(All.size() + 1);
				All.back() = bu;
				fprintf(logfp, "discarding %d out of projection plane\n", u.son);
				printf("discarding %d out of projection plane\n", u.son);
				continue; //discard as it's out of projection plane
			}

			gpc_polygon gProjPoly;
			//	b=calcConvexProjectionWrtCube( projPlane,target,bu,gProjPoly,dir,1 );
			//	if(gProjPoly.num_contours==0)continue;

			gpc_polygon local_p[8];
			int id = 8;

			for (int j = 0; j < id; j++)
			{
				calcCovexProjectionWrtPoint(projPlane, targetCorner[j], bu, local_p[j], dir);
			}

			gProjPoly = local_p[id - 1];
			for (int j = 0; j < id - 1; j++)
			{
				Adapter.polygonMerge(gProjPoly, local_p[j]);
			}

			gpc_polygon gb;

			Adapter.adapt(boundRect, gb);
			Adapter.polygonIntersection(gProjPoly, gb);

			// now check if it's blocked by other projections

			// extend the projection to current layer

			if (dirObstacle[dir].size())
			{
				extedProjToCurLayer(d);
			}

			gpc_polygon global_p = projWrtTargetCorner[7]; //contains projection of all polygon on the current proj plane
			for (int j = 0; j<7; j++)
			{
				Adapter.polygonIntersection(global_p, projWrtTargetCorner[j]);
			}

			b = Adapter.contains(global_p, gProjPoly);
			if (b)
			{
				bu.color = Vector3(255, 0, 0);
				discard += countChild(u.son);
				All.resize(All.size() + 1);
				All.back() = bu;
				fprintf(logfp, "discarding id %d view blocked by other obstacles\n", u.son);
				printf("discarding id %d view blocked by other obstacles\n", u.son);
				continue;
			}

			if (rtn->level)
			{
				dstFromTar[u.son] = u.bounces[0];
				Q.push(u.son);

				continue;
			}

			//calc projection
			//for(int d=0;d<6;d++)printf("%lf ",u.bounces[d]);cout<<endl;

			for (int j = 0; j < id; j++)
			{
				Adapter.polygonMerge(projWrtTargetCorner[j], local_p[j]);
			}
			int r = rand() % rColor.size();
			bu.color = Vector3(rColor[r].x, rColor[r].y, rColor[r].z);
			curProjPlane[dir] = d;
			dirObstacle[1].resize(dirObstacle[1].size() + 1);
			dirObstacle[1].back() = bu;

			cnt++;
		}

	}
	printf(" obstacle discarded %d, reduced obstacle set size %d\n", discard, cnt);
	printf("dir obstacle size %d, all obstacle size %d\n", dirObstacle[1].size(), All.size());
	fprintf(efile, " obstacle discarded %d, reduced obstacle set size%d\n", discard, cnt);
	totDiscard += discard;
	totCnt += cnt;

	//printf(" MBR discarded %d, total obstacle discarded %d, reduced obstacle set size%d\n",All.size(),discard,cnt);
	//cout<<All.size()<<endl;
	//cout<<"total "<<cnt<<endl;

}

void VCM::reductionInPosX(int dir) //dir=1
{
	//puts("reduction in +x direction...");
	int discarded = 0;

	priority_queue<int, vector<int>, decltype(&CompareLoX)> pq(&CompareLoX);
	//priority_queue<int, vector<int>, CompareLoX> pq;
	bool bdir = dir & 1;
	for (int i = 0; i<obstacles.size(); i++) {
		if (obstacles[i].x[bdir ^ 1] >= target.x[bdir]) pq.push(i); //change here ofr direction
		else {
			obstacles[i].color = Vector3(RED);
			//printf("discarding %d not in +x direction\n", i);
			discarded++;
		}
		//if( obstacles[i].x[bdir^1]>=target.x[bdir] )
	}


	int &curSeg = totSeg[dir];
	curSeg = 0;

	//cout<<"(";

	while (!pq.empty())
	{
		int u = pq.top();
		pq.pop();

		while (curSeg<NDISTESEG - 1 && target.x[dir] + segDist[curSeg] <= obstacles[u].x[dir])
		{
			curSeg++;
			update(dir, curSeg); // eikhane optimize korbo aro...
			Adapter.adapt(dir, segDist[curSeg], gProjPolys[dir][curSeg], ProjPolys[dir][curSeg]);

		}


		Box projPoly;
		Plane projPlane = Plane(CO(unitDir[dir]), segDist[curSeg] + target.o.x); //change here for direction
		Box boundRect = projRect[dir][curSeg];

		//cout<<"[";
		bool b = calcProjectionWrtCube(projPlane, target, obstacles[u], projPoly);
		//cout<<"]";
		if (!b)
		{
			obstacles[u].color = Vector3(GREEN);
			//printf("discarding id %d because of near distance\n", u);
			discarded++;
			continue;
		}
		Box projPoly1;
		//cout<<"{";
		b = commonBox(projPoly, boundRect, projPoly1);
		//cout<<"}";
		if (!b)
		{
			obstacles[u].color = Vector3(YELLOW);
			//printf("discarding id %d because of because out of the projection rectangle\n", u);
			discarded++;
			continue;
		}
		gpc_polygon gProjPoly;

		b = calcConvexProjectionWrtCube(projPlane, target, obstacles[u], gProjPoly, dir);

		if (gProjPoly.num_contours == 0)continue;

		//cout<<"<";
		b = Adapter.contains(gProjPolys[dir][curSeg], gProjPoly);
		//cout<<">";
		if (b)
		{
			obstacles[u].color = Vector3(BLUE);
			//printf("discarding id %d view blocked by other obstacles\n", u);
			discarded++;
			continue;
		}

		dirObstacleId[dir].push_back(u);
		dirObstacle[dir].resize(dirObstacle[dir].size() + 1);
		dirObstacle[dir].back() = obstacles[u];
	}

	curSeg = NDISTESEG - 2;
	//printf("Total discarded: %d\n", discarded);

	//cout<<")";
}

void VCM::reductionInPosY(int dir) //dir=3
{
	//puts("reduction in +y direction...");
	int discarded = 0;
	priority_queue<int, vector<int>, decltype(&CompareLoY)> pq(&CompareLoY);
	//priority_queue<int, vector<int>, CompareLoY> pq;

	bool bdir = dir & 1;
	for (int i = 0; i < obstacles.size(); i++)
	{
		if (obstacles[i].y[bdir ^ 1] >= target.y[bdir])
			pq.push(i); //change here for direction
		else {
			obstacles[i].color = Vector3(RED);
			//printf("discarding %d not in +y direction\n", i);
			discarded++;
		}
	}
	int &curSeg = totSeg[dir];
	curSeg = 0;

	//putchar('(');
	while (!pq.empty())
	{
		int u = pq.top();
		pq.pop();

		//	putchar('{');

		while (curSeg<NDISTESEG - 1 && target.y[1] + segDist[curSeg] <= obstacles[u].y[1])
		{
			curSeg++;
			update(dir, curSeg); // eikhane optimize korbo aro...
			Adapter.adapt(dir, segDist[curSeg], gProjPolys[dir][curSeg], ProjPolys[dir][curSeg]);

		}

		//	putchar('}');


		Box projPoly;
		Plane projPlane = Plane(CO(unitDir[dir]), segDist[curSeg] + target.o.y); //change here for direction
		Box boundRect = projRect[dir][curSeg];

		//putchar('[');
		bool b = calcProjectionWrtCube(projPlane, target, obstacles[u], projPoly);
		//	putchar(']');

		if (!b)
		{
			obstacles[u].color = Vector3(GREEN);
			//printf("discarding id %d because of near distance\n", u);
			discarded++;
			continue;
		}
		Box projPoly1;

		//	putchar('<');
		b = commonBox(projPoly, boundRect, projPoly1);
		//	putchar('>');
		if (!b)
		{
			obstacles[u].color = Vector3(YELLOW);
			//printf("discarding id %d because of because out of the projection rectangle\n", u);
			discarded++;
			continue;
		}
		gpc_polygon gProjPoly;

		calcConvexProjectionWrtCube(projPlane, target, obstacles[u], gProjPoly, dir);

		if (gProjPoly.num_contours == 0)continue;

		//	putchar('6');
		b = Adapter.contains(gProjPolys[dir][curSeg], gProjPoly);
		//putchar( '9' );
		if (b)
		{
			obstacles[u].color = Vector3(BLUE);
			//printf("discarding id %d view blocked by other obstacles\n", u);
			discarded++;
			continue;
		}

		dirObstacleId[dir].push_back(u);
		dirObstacle[dir].resize(dirObstacle[dir].size() + 1);
		dirObstacle[dir].back() = obstacles[u];
	}
	curSeg = NDISTESEG - 2;
	//printf("Total discarded: %d\n", discarded);
}

void VCM::reductionInPosZ(int dir) //dir=5
{
	//puts("reduction in +z direction...");
	int discarded = 0;
	priority_queue<int, vector<int>, decltype(&CompareLoZ)> pq(&CompareLoZ);
	//priority_queue<int, vector<int>, CompareLoZ> pq;

	bool bdir = dir & 1;
	for (int i = 0; i < obstacles.size(); i++) {
		if (obstacles[i].z[bdir ^ 1] >= target.z[bdir])
			pq.push(i); //change here for direction
		else {
			obstacles[i].color = Vector3(RED);
			//printf("discarding %d not in +z direction\n", i);
			discarded++;
		}
	}
	int &curSeg = totSeg[dir];
	curSeg = 0;


	while (!pq.empty())
	{
		int u = pq.top();
		pq.pop();

		while (curSeg<NDISTESEG - 1 && target.z[1] + segDist[curSeg] <= obstacles[u].z[1])
		{
			curSeg++;
			update(dir, curSeg); // eikhane optimize korbo aro...
			Adapter.adapt(dir, segDist[curSeg], gProjPolys[dir][curSeg], ProjPolys[dir][curSeg]);

		}


		Box projPoly;
		Plane projPlane = Plane(CO(unitDir[dir]), segDist[curSeg] + target.o.z); //change here for direction
		Box boundRect = projRect[dir][curSeg];
		if (!calcProjectionWrtCube(projPlane, target, obstacles[u], projPoly))
		{
			obstacles[u].color = Vector3(GREEN);
			discarded++;
			//printf("discarding id %d because of near distance\n", u);
			continue;
		}
		Box projPoly1;
		if (!commonBox(projPoly, boundRect, projPoly1))
		{
			obstacles[u].color = Vector3(YELLOW);
			discarded++;
			//printf("discarding id %d because of because out of the projection rectangle\n", u);
			continue;
		}
		gpc_polygon gProjPoly;

		bool b = calcConvexProjectionWrtCube(projPlane, target, obstacles[u], gProjPoly, dir);
		if (gProjPoly.num_contours == 0) continue;
		if (Adapter.contains(gProjPolys[dir][curSeg], gProjPoly))
		{
			obstacles[u].color = Vector3(BLUE);
			discarded++;
			//printf("discarding id %d view blocked by other obstacles\n", u);
			continue;
		}

		dirObstacleId[dir].push_back(u);
		dirObstacle[dir].resize(dirObstacle[dir].size() + 1);
		dirObstacle[dir].back() = obstacles[u];
	}

	curSeg = NDISTESEG - 2;
	//printf("Total discarded: %d\n", discarded);
}



void VCM::reductionOfObstacle()
{



	// making 6 partition not disjoint of the obstacles

	// do for positive x direction

	//reductionInNegX(0);
	reductionInPosX(1);
	//reductionInPosY(3);





	/*for(int i=0;i<obstacles.size();i++)if( obstacles[i].x[0]<=target.x[0] )dirObstacleId[1].push_back(i);
	sort(dirObstacleId[1].begin(),dirObstacleId[1].end(),cmphix);
	for(int i=0;i<obstacles.size();i++)if( obstacles[i].y[1]>=target.y[1] )dirObstacleId[2].push_back(i);
	sort(dirObstacleId[2].begin(),dirObstacleId[2].end(),cmploy);
	for(int i=0;i<obstacles.size();i++)if( obstacles[i].y[0]<=target.y[0] )dirObstacleId[3].push_back(i);
	sort(dirObstacleId[3].begin(),dirObstacleId[3].end(),cmphiy);
	for(int i=0;i<obstacles.size();i++)if( obstacles[i].z[1]>=target.z[1] )dirObstacleId[4].push_back(i);
	sort(dirObstacleId[4].begin(),dirObstacleId[4].end(),cmploz);
	for(int i=0;i<obstacles.size();i++)if( obstacles[i].z[0]<=target.z[0] )dirObstacleId[5].push_back(i);
	sort(dirObstacleId[5].begin(),dirObstacleId[5].end(),cmphiz);


	for(int dir=0;dir<1;dir++)
	{
	vector<int>&Id=dirObstacleId[dir];

	for(int i=0;i<Id.size();i++)
	{



	}
	}
	*/


}

void VCM::debug0()
{
	Plane p(0, 1, 0, 2);
	Vector3 c(0, 0, 0);



	Vector3 v(0, 0, 2);

	Vector3 u;

	//cout<<p.caclProjectionOfPoint(c,v,u)<<endl;
}

void VCM::input()
{
	FILE* fp = fopen("sample1.txt", "r");

	//target.fscan(fp);
	/*
	Vector3 v0,v1;
	v0.fscan(fp);
	v1.fscan(fp);
	Origin=v0.add(v1).div(2);
	target.init(v0,v1,Origin);
	target.setPlanes();

	*/



	fscanf(fp, "%d", &nObstacle);

	obstacles = vector<Box>(nObstacle);

	for (int i = 0; i<(int)obstacles.size(); i++)
	{
		obstacles[i].fscan(fp, Origin);
	}

	fclose(fp);
}

void VCM::drawGridX()
{

	for (int dir = 0; dir <= 1; dir++)
	{
		for (int curSeg = curSegId; curSeg <= curSegId; curSeg++)
		{
			double dim = getDim(dir, curSeg);
			double delta = dim / MAXSEG;

			double lx = -dim / 2.0; lx += delta;
			double ly = -dim / 2.0; ly += delta;

			int tot = targetPoints.size();
			//	int dir=1;
			double z = segDist[curSeg];
			if (!dir)z = -z;// for negative z

			for (int i = 0; i<MAXSEG; i++)
			{
				for (int j = 0; j<MAXSEG; j++)
				{
					vcmArray[dir][curSeg][i][j] = min(vcmArray[dir][curSeg][i][j], tot); //discarding noise 


					double col = 255 - 255.0*((double)vcmArray[dir][curSeg][i][j] / tot);
					glPushMatrix();
					if (1 || vcmArray[dir][curSeg][i][j]){

						//		cout<<"asd"<<endl;
						//	col=0;
						glColor3ub(col, col, col);
						glBegin(GL_QUADS); {

							glVertex3d(z, lx, ly);
							glVertex3d(z, lx + delta, ly);
							glVertex3d(z, lx + delta, ly + delta);
							glVertex3d(z, lx, ly + delta);

						}glEnd();
					}glPopMatrix();

					lx += delta;
				}
				ly += delta;
				lx = -dim / 2.0;
			}
		}
	}
}

void VCM::drawGridY()
{
	for (int dir = 2; dir <= 3; dir++)
	{
		for (int curSeg = curSegId; curSeg <= curSegId; curSeg++)
		{
			double dim = getDim(dir, curSeg);
			double delta = dim / MAXSEG;

			double lx = -dim / 2.0; lx += delta;
			double ly = -dim / 2.0; ly += delta;

			int tot = targetPoints.size();
			//	int dir=1;
			double z = segDist[curSeg];
			if (dir == 2)z = -z;// for negative z

			for (int i = 0; i<MAXSEG; i++)
			{
				for (int j = 0; j<MAXSEG; j++)
				{
					vcmArray[dir][curSeg][i][j] = min(vcmArray[dir][curSeg][i][j], tot); //discarding noise 


					double col = 255 - 255.0*((double)vcmArray[dir][curSeg][i][j] / tot);
					glPushMatrix();
					if (vcmArray[dir][curSeg][i][j]){
						glColor3ub(col, col, col);
						glBegin(GL_QUADS); {

							glVertex3d(lx, z, ly);
							glVertex3d(lx + delta, z, ly);
							glVertex3d(lx + delta, z, ly + delta);
							glVertex3d(lx, z, ly + delta);

						}glEnd();
					}glPopMatrix();

					lx += delta;
				}
				ly += delta;
				lx = -dim / 2.0;
			}
		}
	}
}


void VCM::drawGridZ()
{
	for (int dir = 4; dir <= 5; dir++)
	{
		for (int curSeg = curSegId; curSeg <= curSegId; curSeg++)
		{
			double dim = getDim(dir, curSeg);
			double delta = dim / MAXSEG;

			double lx = -dim / 2.0; lx += delta;
			double ly = -dim / 2.0; ly += delta;

			int tot = targetPoints.size();
			//	int dir=1;
			double z = segDist[curSeg];
			if (dir == 4)z = -z;// for negative z

			for (int i = 0; i<MAXSEG; i++)
			{
				for (int j = 0; j<MAXSEG; j++)
				{
					vcmArray[dir][curSeg][i][j] = min(vcmArray[dir][curSeg][i][j], tot); //discarding noise 


					double col = 255 - 255.0*((double)vcmArray[dir][curSeg][i][j] / tot);
					glPushMatrix();
					if (vcmArray[dir][curSeg][i][j]){
						glColor3ub(col, col, col);
						glBegin(GL_QUADS); {

							glVertex3d(lx, ly, z);
							glVertex3d(lx + delta, ly, z);
							glVertex3d(lx + delta, ly + delta, z);
							glVertex3d(lx, ly + delta, z);

						}glEnd();
					}glPopMatrix();

					lx += delta;
				}
				ly += delta;
				lx = -dim / 2.0;
			}
		}
	}
}


void VCM::drawGrid()
{

	//	int dir=0;
	//int curSeg=totSeg[dir];


	drawGridX();
	drawGridY();
	drawGridZ();



}

void VCM::debug1()
{
	glColor3ub(255, 193, 37);
	//SET_GOLD_FRONT
	target.draw();

	for (int dir = 0; dir<totDir; dir++)
	{
		int k = dir;
		for (int i = 0; i<dirObstacleId[k].size(); i++)
		{
			glColor3ub(rColor[dirObstacleId[k][i]].x, rColor[dirObstacleId[k][i]].y, rColor[dirObstacleId[k][i]].z);
			if (showObstacle)obstacles[dirObstacleId[k][i]].draw();
		}
	}


	//return;

	/*
	for(int dir=0;dir<totDir;dir++)
	{
	int k=dir;
	for(int i=0;i<=totSeg[k];i++)
	{
	glColor3ub(0,255,255);
	if(showDistRec)projRect[k][i].draw();
	glColor3ub(0,0,0);
	if(i<totSeg[k])if(showProjPoly)
	{
	//				Adapter.drawGpcPolygon( gProjPolys[k][i+1] );
	Adapter.draw3DPolygon( ProjPolys[k][i+1] );
	}
	}
	}
	*/

	if (showAll && !showObstacle)for (int i = 0; i<obstacles.size(); i++)
	{
		glColor3ub(rColor[i].x, rColor[i].y, rColor[i].z);
		obstacles[i].draw();
	}

	//Adapter.drawGpcPolygon( gProjPolys[1][totSeg[1]] );
	int dir = 1;
	//for(int j=0;j<targetPoints.size();j++)
	/*
	{
	int j=showid;
	gpc_polygon &g=gVcmProjPolys[dir][totSeg[dir]][j];

	for(int i=0;i<g.num_contours;i++)
	{
	glPushMatrix();{
	glColor3ub(CO(rColor[j]));
	glBegin(GL_LINE_LOOP);
	for(int  j=0;j<g.contour[i].num_vertices;j++ )
	{
	glVertex3d( segDist[totSeg[dir]],g.contour[i].vertex[j].x,g.contour[i].vertex[j].y );
	glVertex3d( segDist[totSeg[dir]],g.contour[i].vertex[(j+1)%g.contour[i].num_vertices].x,g.contour[i].vertex[(j+1)%g.contour[i].num_vertices].y );
	}
	glEnd();
	}glPopMatrix();
	}

	*/





	int j = showid;
	/*
	for(dir=0;dir<totDir;dir++)
	{

	for(int curSeg=1;curSeg<=totSeg[dir];curSeg++)
	{

	vector< vector<Vector3> >&p=vcmProjPolys[dir][curSeg][j];

	for(int i=0;i<p.size();i++)
	{
	glPushMatrix();{
	glColor3ub(CO(rColor[j]));
	glBegin(GL_LINE_LOOP);
	for(int  j=0;j<p[i].size();j++ )
	{
	glVertex3d( CO(p[i][j]) );
	glVertex3d( CO(p[i][(j+1)%p[i].size()]) );
	}
	glEnd();
	}glPopMatrix();
	}
	}
	}
	*/

	//	Adapter.drawGpcPolygon( gVcmProjPolys[1][totSeg[1]][i] );


	if (drawg == 1)drawGrid();

	/*
	//for(int i=0;i<targetPoints.size();i++)
	{
	int i=showid;
	glPushMatrix();{
	glColor3ub(CO(rColor[i]));
	glTranslated( CO( targetPoints[i] ) );
	glutSolidSphere( .5,10,10 );

	}glPopMatrix();
	//Adapter.drawGpcPolygon( gVcmProjPolys[1][totSeg[1]][i] );

	}*/



	return;


}

void VCM::generateRandCol()
{
	srand(time(NULL));
	for (int i = 0; i < 30; i++) {
		double r, g, b;
		while (1) {
			r = rand() % 255;
			g = rand() % 255;
			b = rand() % 255;
			if (!((r == 255 && !g && !b) || (!r && g == 255 && !b) || (!r && !g && b == 255) || (!r && g == 255 && b == 255) || (r == 255 && g == 255 && !b)))
				break;
		}
		rColor.push_back(Vector3(r, g, b));
	}
}

double VCM::getDim(int dir, int curSeg) // gives the dimension of the projection rectangle
{
	double dim = 0;

	dim = max(dim, fabs(projRect[dir][curSeg].x[0] - projRect[dir][curSeg].x[1]));
	dim = max(dim, fabs(projRect[dir][curSeg].y[0] - projRect[dir][curSeg].y[1]));
	dim = max(dim, fabs(projRect[dir][curSeg].z[0] - projRect[dir][curSeg].z[1]));

	return dim;
}


pair<int, int> VCM::pointToarrayIndex(double px, double py, int dir, int curSeg)
{

	//cout<<"**"<<px<<" "<<py<<endl;


	double dim = getDim(dir, curSeg);


	//	assert( ( (2*px>=-dim) && (2*px<=+dim) ));


	//making the -ve index positive 

	px += dim*.5;
	py += dim*.5;

	if (!((px <= dim) && (py <= dim) && (px >= 0) && (py >= 0)))
	{
		////cout<<"-->"<<dim<<" "<<px<<" "<<py<<endl;
		//while(1);
	}


	px = min(dim, px);
	py = min(dim, py);
	px = max(0, px);
	py = max(0, py);



	// we have to convert a dim*dim rectangle's point in geometric space to\
			2d array index

	// suppose the dir=1;
	// rectangles in direction 1 that is in x(-y) plane
	// so lower left point corresponds to 0,0

	int x, y;
	x = px*MAXSEG / dim + .5; x++;
	y = py*MAXSEG / dim + .5;


	x = min(MAXSEG - 1, x);
	y = min(MAXSEG - 1, y);
	x = max(0, x);
	y = max(0, y);



	return make_pair(x, y);

}
pair<int, int> VCM::pointToarrayIndex2(double px, double py, int dir, int curSeg)
{

	//cout<<"**"<<px<<" "<<py<<endl;


	double dim = getDim(dir, curSeg);


	//	assert( ( (2*px>=-dim) && (2*px<=+dim) ));


	//making the -ve index positive 

	px += dim*.5;
	py += dim*.5;

	if (!((px <= dim) && (py <= dim) && (px >= 0) && (py >= 0)))
	{
		////cout<<"-->"<<dim<<" "<<px<<" "<<py<<endl;
		//while(1);
	}


	px = min(dim, px);
	py = min(dim, py);
	px = max(0, px);
	py = max(0, py);



	// we have to convert a dim*dim rectangle's point in geometric space to\
		2d array index

	// suppose the dir=1;
	// rectangles in direction 1 that is in x(-y) plane
	// so lower left point corresponds to 0,0

	int x, y;
	x = px*MAXSEG / dim + .5; x--;
	y = py*MAXSEG / dim + .5; y--;


	x = min(MAXSEG - 1, x);
	y = min(MAXSEG - 1, y);
	x = max(0, x);
	y = max(0, y);

	return make_pair(x, y);

}

double VCM::getx(Vector v0, Vector v1, double y) // intersection of v0--v1 with y horizontal line
{
	// (x-x0)/(x0-x1)=(y-y0)/(y0-y1);\
		x=(y-y0)*(x0-x1)/(y0-y1) + x0


	double x = (y - v0.y)*(v0.x - v1.x) / (v0.y - v1.y) + v0.x;
	return x;
}


bool VCM::cmpFormonoPolygonTo2Darray(Vector v1, Vector v2)
{
	return v1.y<v2.y;
}
// map the monotone polygon to the 2D array
void VCM::monoPolygonTo2Darray(int dir, int curSeg, vector<Vector>mpoly)
{

	//return;

	int low = 0, hi = 0;

	// find the upper and lower y

	for (int i = 1; i<mpoly.size(); i++)if (mpoly[i].y<mpoly[low].y)low = i;
	for (int i = 1; i<mpoly.size(); i++)if (mpoly[i].y>mpoly[hi].y)hi = i;

	vector< Vector >pq;

	//for(int i=0;i<mpoly.size();i++)pq.push_back( mpoly[i] );
	double dim = getDim(dir, curSeg);


	double delta = dim / MAXSEG; // length of each small segment

	for (double y = (mpoly[low].y + delta); y <= mpoly[hi].y - delta; y += delta)
	{
		pq.push_back(Vector(+INF, y)); // x=INF means it is for horizontal lines
	}

	int n = mpoly.size();
	int left = low, right = low, nleft, nright;


	vector<bool>vst(mpoly.size(), 0);

	vst[left] = true;

	//int cury=pointToarrayIndex( +INF,low,dir,curSeg ).second;

	sort(pq.begin(), pq.end(), cmpFormonoPolygonTo2Darray);

	//pq.pop();
	nleft = (left - 1 + n) % n;
	nright = (right + 1) % n;
	for (int i = 0; i<pq.size(); i++)
	{

		Vector u = pq[i];
		//      pq.pop();


		while (nleft != hi && !(mpoly[left].y <= u.y && mpoly[nleft].y >= u.y)) {
			left = nleft;
			nleft = (left - 1 + n) % n;
		}



		while (nright != hi && !(mpoly[right].y <= u.y && mpoly[nright].y >= u.y)) {
			right = nright;
			nright = (right + 1 + n) % n;
		}



		int xleft = getx(mpoly[left], mpoly[nleft], u.y);
		int xright = getx(mpoly[right], mpoly[nright], u.y);



		pii arrayleft = pointToarrayIndex(xleft, u.y, dir, curSeg);
		pii arrayright = pointToarrayIndex(xright, u.y, dir, curSeg);



		//if(curSeg==167)cout<<endl << arrayleft.first<<" "<<arrayleft.second<<" "<<arrayright.first<<" "<<arrayright.second<<endl;


		vcmArray[dir][curSeg][arrayleft.second][arrayleft.first]++;
		vcmArray[dir][curSeg][arrayleft.second][arrayright.first]--;

		//      for( int col = arrayleft.first; col<=arrayright.first; col++) vcmArray[dir][curSeg][arrayleft.second][ col ]++;




	}


}
// map the monotone polygon to the 2D array
void VCM::monoPolygonTo2Darray1(int dir, int curSeg, vector<Vector>mpoly)
{

	//return;

	int low = 0, hi = 0;

	// find the upper and lower y

	for (int i = 1; i<mpoly.size(); i++)if (mpoly[i].y<mpoly[low].y)low = i;
	for (int i = 1; i<mpoly.size(); i++)if (mpoly[i].y>mpoly[hi].y)hi = i;

	vector< Vector >pq;

	for (int i = 0; i<mpoly.size(); i++)pq.push_back(mpoly[i]);
	double dim = getDim(dir, curSeg);


	double delta = dim / MAXSEG; // length of each small segment

	for (double y = (mpoly[low].y + delta); y <= mpoly[hi].y - delta; y += delta)
	{
		pq.push_back(Vector(+INF, y)); // x=INF means it is for horizontal lines
	}

	int n = mpoly.size();
	int left = low, right = low, nleft, nright;


	vector<bool>vst(mpoly.size(), 0);

	vst[left] = true;

	//int cury=pointToarrayIndex( +INF,low,dir,curSeg ).second;

	sort(pq.begin(), pq.end(), cmpFormonoPolygonTo2Darray);

	//pq.pop();

	for (int i = 1; i<pq.size(); i++)
	{

		Vector u = pq[i];
		//		pq.pop();


		nleft = (left - 1 + n) % n;
		nright = (right + 1) % n;
		if (u.x == INF)
		{

			if (!(mpoly[left].y <= u.y && mpoly[nleft].y >= u.y && mpoly[right].y <= u.y && mpoly[nright].y >= u.y))
			{

				//ektu error ase.. eita pore dekhbo
				//	while(1);
			}

			// found a y strip need to update

			//	continue;

			if (min(mpoly[left].y, mpoly[nleft].y) > u.y)continue;
			if (max(mpoly[left].y, mpoly[nleft].y) < u.y)continue;

			if (min(mpoly[right].y, mpoly[nright].y) > u.y)continue;
			if (max(mpoly[right].y, mpoly[nright].y) < u.y)continue;

			int xleft = getx(mpoly[left], mpoly[nleft], u.y);
			int xright = getx(mpoly[right], mpoly[nright], u.y);

			pii arrayleft = pointToarrayIndex(xleft, u.y, dir, curSeg);
			pii arrayright = pointToarrayIndex(xright, u.y, dir, curSeg);


			/*
			cout << endl;
			cout<<xleft<<" "<<u.y<<" "<<arrayleft.first<<" "<<arrayleft.second<<endl;
			cout<<xright<<" "<<u.y<<" "<<arrayright.first<<" "<<arrayright.second<<endl;
			cout << endl;
			*/
			vcmArray[dir][curSeg][arrayleft.second][arrayleft.first]++;
			vcmArray[dir][curSeg][arrayleft.second][arrayright.first]--;

			continue;
		}
		if (Z(u.y - mpoly[nleft].y))left = nleft;
		if (Z(u.y - mpoly[right].y)) right = nright;
	}

}
// map the monotone polygon to the 2D array, ekta ekta kore polygon map kore
void VCM::monoPolygonTo2Darray2(int dir, int curSeg, vector<Vector>mpoly)
{

	//return;

	int low = 0, hi = 0;

	// find the upper and lower y of polygon

	for (int i = 1; i<mpoly.size(); i++)if (mpoly[i].y<mpoly[low].y)low = i;
	for (int i = 1; i<mpoly.size(); i++)if (mpoly[i].y>mpoly[hi].y)hi = i;

	vector< Vector >pq;
	// pq te first e polygon er shob point push korlam
	for (int i = 0; i<mpoly.size(); i++) pq.push_back(mpoly[i]);
	// oi direction and current segment e jei projection rect hobe shetar dimension
	double dim = getDim(dir, curSeg);

	double delta = dim / MAXSEG; // length of each small segment (cell)
	// ekdom nicher  theke high  y porjonto, +INF x dhore point gulo pq te push korlam
	for (double y = (mpoly[low].y + delta); y <= mpoly[hi].y - delta; y += delta)
	{
		pq.push_back(Vector(+INF, y)); // x=INF means it is for horizontal lines
	}

	int n = mpoly.size();
	int left = low, right = low, nleft, nright;

	vector<bool>vst(mpoly.size(), 0);

	vst[left] = true;

	//int cury=pointToarrayIndex( +INF,low,dir,curSeg ).second;
	// sorting in ascending order of y
	// each vector in pq ashole ekta cell ??
	sort(pq.begin(), pq.end(), cmpFormonoPolygonTo2Darray);

	//pq.pop();

	for (int i = 1; i<pq.size(); i++)
	{
		Vector u = pq[i];
		//		pq.pop();
		// left right initially low te ase, nleft nright hoilo immediately ager ar porer vertex acc to y value
		nleft = (left - 1 + n) % n;
		nright = (right + 1) % n;
		/* point gulor kisu actual polygon er ar kisu +INF x value te ase */
		if (u.x == INF)
		{
			if (!(mpoly[left].y <= u.y && mpoly[nleft].y >= u.y && mpoly[right].y <= u.y && mpoly[nright].y >= u.y))
			{
				// ektu error ase.. eita pore dekhbo
				// while(1);
			}
			// found a y strip need to update

			//	continue;
			int xleft = getx(mpoly[left], mpoly[nleft], u.y);
			int xright = getx(mpoly[right], mpoly[nright], u.y);

			pii arrayleft = pointToarrayIndex(xleft, u.y, dir, curSeg);
			pii arrayright = pointToarrayIndex(xright, u.y, dir, curSeg);

			//cout<<xleft<<" "<<u.y<<" "<<arrayleft.first<<" "<<arrayleft.second<<endl;
			//cout<<xright<<" "<<u.y<<" "<<arrayright.first<<" "<<arrayright.second<<endl;

			vcmArray[dir][curSeg][arrayleft.second][arrayleft.first]++;
			vcmArray[dir][curSeg][arrayleft.second][arrayright.first]--;

			continue;
		}

		if (Z(u.y - mpoly[nleft].y))left = nleft;
		// mpoly[nright] hobe na?
		if (Z(u.y - mpoly[right].y)) right = nright;

	}

}


void VCM::mergeAll(int dir, int curSeg)
{
	for (int i = 0; i<MAXSEG; i++)
	{
		for (int j = 1; j<MAXSEG; j++)
		{
			vcmArray[dir][curSeg][i][j] += vcmArray[dir][curSeg][i][j - 1];
		}
	}
	if (argContainsText) {
		for (int i = 0; i<MAXSEG; i++)
		{
			for (int j = 1; j<MAXSEG; j++)
			{
				if (dist(target.c, arrayIndexToPoint(i, j, dir, curSeg)) > fonttodist[fontsize])
					vcmArray[dir][curSeg][i][j] = targetPoints.size();
			}
		}
	}

}
/* extend projection from previous segment to current */
void VCM::extedProjToCurLayer1(int tid, int curSeg, int dir)
{
	Vector3 c = targetPoints[tid];
	gpc_polygon &g = gVcmProjPolys[dir][curSeg][tid];
	double d = segDist[curSeg];

	if (curSeg)
	{
		gpc_polygon &gprev = gVcmProjPolys[dir][curSeg - 1][tid];

		for (int i = 0; i <gprev.num_contours; i++)
		{
			gpc_add_contour(&g, &gprev.contour[i], 0);

			for (int j = 0; j < gprev.contour[i].num_vertices; j++)
			{
				double px = segDist[curSeg - 1];
				double py = gprev.contour[i].vertex[j].x;
				double pz = gprev.contour[i].vertex[j].y;


				double x1 = px - c.x;
				double y1 = py - c.y;
				double z1 = pz - c.z;

				double x2 = d - c.x;
				double y2 = (y1 / x1)*x2;
				double z2 = (z1 / x1)*x2;


				py = c.y + y2;
				pz = c.z + z2;

				g.contour[i].vertex[j].x = py;
				g.contour[i].vertex[j].y = pz;



			}
		}

	}

}

/*
1. extend projection of previous layer
2. compute projection and merge to g for each obstacle wrt target point
*/
void VCM::updateVcm1(vector<Box>&obstacleWrtPoint, int dir, int curSeg, int tid)
{

	//vector<int>&id=obstacleWrtPoint;
	extedProjToCurLayer1(tid, curSeg, dir);

	Vector3 c = targetPoints[tid];
	gpc_polygon &g = gVcmProjPolys[dir][curSeg][tid];

	//g.num_contours=0;

	for (int i = 0; i<obstacleWrtPoint.size(); i++)
	{
		gpc_polygon local_p;
		Plane projPlane = Plane(unitDir[dir].x, unitDir[dir].y, unitDir[dir].z, segDist[curSeg] + c.x);
		calcCovexProjectionWrtPoint(projPlane, c, obstacleWrtPoint[i], local_p, dir);
		Adapter.polygonMerge(g, local_p);
	}

	obstacleWrtPoint.clear();

}

bool VCM::fcmp(Box b1, Box b2)
{
	return b1.x[0]<b2.x[0];
}

bool VCM::fcmpPosY(Box b1, Box b2)
{
	return b1.y[0]<b2.y[0];
}

bool VCM::fcmpPosZ(Box b1, Box b2)
{
	return b1.z[0]<b2.z[0];
}

bool VCM::fcmpNegX(Box b1, Box b2)
{
	return b1.x[1]<b2.x[1];
}

bool VCM::fcmpNegY(Box b1, Box b2)
{
	return b1.y[1]<b2.y[1];
}

bool VCM::fcmpNegZ(Box b1, Box b2)
{
	return b1.z[1]<b2.z[1];
}

/*
for some target point and some direction
1. sorts obstacles list by distance from target
2. for each obstacle:
3. check if common part ase with target, thakle continue
4. jotokhn na obstacle tar shesh matha porjonto jabe,
totokhn curseg++ i.e. projection shuru hobe obstacle er shesh point er plane theke
5. last segment e giye projection ber korbo, if it is not contained in alerady computed projection, we need to consider that obstacle
6. g containes projection of obstacles wrt to target point for some segment
7. for each segment:
8. g k 3d vector points e convert kora hoy and vcmProjPolys e rakha hoy
9. g er proti ta contour er monotone partition kore vector points store korlam
10. finally 2D array te map kora hoy monotone polygon gulo


*/


void VCM::generateMapInPosX(int tid, int dir)
{

	Vector3 tp = targetPoints[tid];
	//int dir=1;
	int curSeg = 1;
	vector<Box>obstacleWrtPoint;
	sort(dirObstacle[dir].begin(), dirObstacle[dir].end(), fcmp);
	//	soet(dirObstacle[dir].begin(),dirObstacle[dir].end());

	for (int i = 0; i<dirObstacle[dir].size(); i++)
	{
		Box u = dirObstacle[dir][i];

		Box projPoly1;
		/*
		target and obstacle have some common intersection point
		common part is stored in projPoly1
		*/
		bool b = commonBox(target, u, projPoly1);

		if (b)continue;


		while (curSeg<NDISTESEG && tp.x + segDist[curSeg] <= u.x[1])
		{

			updateVcm1(obstacleWrtPoint, dir, curSeg, tid); // eikhane optimize korbo aro...
			curSeg++;

			//	Adapter.adapt( dir,tp.x+segDist[curSeg],gVcmProjPolys[dir][curSeg][tid],vcmProjPolys[dir][curSeg][tid] );

		}
		// ei segment e amdr obstacle er projection felbo
		int lastSeg = curSeg - 1;

		//vector< vector<Vector3> > &p=vcmProjPolys[dir][curSeg-1][tid];
		/* current target point er polygon */
		gpc_polygon &g = gVcmProjPolys[dir][lastSeg][tid];

		Box projPoly;
		Plane projPlane = Plane(unitDir[dir].x, unitDir[dir].y, unitDir[dir].z, segDist[lastSeg]); //change here for direction
		Box boundRect = projRect[dir][lastSeg];

		gpc_polygon gProjPoly;
		/* calc projection of obstacle points on projPlane wrt target point */
		calcCovexProjectionWrtPoint(projPlane, tp, u, gProjPoly, dir);

		if (Adapter.contains(g, gProjPoly))
		{
			//printf("discarding id %d view blocked by other obstacles\n",u);
			continue;
		}
		obstacleWrtPoint.resize(obstacleWrtPoint.size() + 1);
		obstacleWrtPoint.back() = (u);
	}

	/* updates vcm for rest of the segments */
	while (curSeg<NDISTESEG)
	{
		updateVcm1(obstacleWrtPoint, dir, curSeg, tid); // eikhane optimize korbo aro...
		curSeg++;

		//	Adapter.adapt( dir,tp.x+segDist[curSeg],gVcmProjPolys[dir][curSeg][tid],vcmProjPolys[dir][curSeg][tid] );

	}

	for (int curSeg = 1; curSeg< NDISTESEG; curSeg++)
	{

		gpc_polygon &g = gVcmProjPolys[dir][curSeg][tid];

		//cout<<g.num_contours<<endl;

		Box prec = projRect[dir][curSeg];
		gpc_polygon gprec;

		Adapter.adapt(prec, gprec);
		Adapter.polygonIntersection(g, gprec);
		/* g k vector points e newa, proti ta contour er proti ta vertex er vector */
		vector< vector<Vector> >vp;

		Adapter.adapt(g, vp); //vp contains all separate polygons
		/* vector of partitioned polygon, partition naw hote pare */
		vector< vector<Vector> >monoTonePolys;
		/* proj poly k vector points e nilam */
		Adapter.adapt(dir, segDist[curSeg], gVcmProjPolys[dir][curSeg][tid], vcmProjPolys[dir][curSeg][tid]);

		for (int i = 0; i<vp.size(); i++)
		{
			//cout<<endl;
			/* reverse kora hocche karon insertion er shomoy ulta bhabe kora hoyese??? */
			reverse(vp[i].begin(), vp[i].end());
			vector< vector<Vector> >temp = M.getMonotonePartition(vp[i]);
			for (int j = 0; j<temp.size(); j++)monoTonePolys.push_back(temp[j]);
		}

		/* finally map to 2D array */
		for (int i = 0; i<monoTonePolys.size(); i++)
		{
			monoPolygonTo2Darray(dir, curSeg, monoTonePolys[i]);
		}


	}

}

void VCM::generateMapInPosX1(int tid, int dir)
{

	Vector3 tp = targetPoints[tid];
	//int dir=1;
	vector< int >&obstacleId = dirObstacleId[dir];

	int curSeg = 0;

	vector<int>obstacleWrtPoint;

	for (int i = 0; i<obstacleId.size(); i++)
	{
		int u = obstacleId[i];

		while (curSeg<NDISTESEG - 1 && tp.x + segDist[curSeg] <= obstacles[u].x[dir])
		{
			curSeg++;
			updateVcm(obstacleWrtPoint, dir, curSeg, tid); // eikhane optimize korbo aro...


			//	Adapter.adapt( dir,tp.x+segDist[curSeg],gVcmProjPolys[dir][curSeg][tid],vcmProjPolys[dir][curSeg][tid] );

		}

		vector< vector<Vector3> > &p = vcmProjPolys[dir][curSeg][tid];
		gpc_polygon &g = gVcmProjPolys[dir][curSeg][tid];


		Box projPoly;
		Plane projPlane = Plane(unitDir[dir].x, unitDir[dir].y, unitDir[dir].z, segDist[curSeg]); //change here for direction
		Box boundRect = projRect[dir][curSeg];

		/*
		if(!calcProjectionWrtCube(projPlane, target , obstacles[u] ,projPoly))
		{
		printf("discarding id %d because of near distance\n",u);
		continue;
		}
		Box projPoly1;
		if( !commonBox( projPoly,boundRect,projPoly1 ) )
		{
		printf("discarding id %d because of because out of the rojection rectangle\n",u);
		continue;
		}
		*/

		gpc_polygon gProjPoly;

		calcCovexProjectionWrtPoint(projPlane, tp, obstacles[u], gProjPoly, dir);

		if (Adapter.contains(g, gProjPoly))
		{
			//	printf("discarding id %d view blocked by other obstacles\n",u);
			continue;
		}

		obstacleWrtPoint.push_back(u);
	}


	while (curSeg<NDISTESEG - 2)
	{
		curSeg++;
		updateVcm(obstacleWrtPoint, dir, curSeg, tid); // eikhane optimize korbo aro...


		//	Adapter.adapt( dir,tp.x+segDist[curSeg],gVcmProjPolys[dir][curSeg][tid],vcmProjPolys[dir][curSeg][tid] );

	}

	for (int curSeg = 1; curSeg <= totSeg[dir]; curSeg++)
	{

		gpc_polygon &g = gVcmProjPolys[dir][curSeg][tid];
		Box prec = projRect[dir][curSeg];
		gpc_polygon gprec;

		Adapter.adapt(prec, gprec);
		Adapter.polygonIntersection(g, gprec);


		vector< vector<Vector> >vp;

		Adapter.adapt(g, vp); //vp contains all separate polygons

		vector< vector<Vector> >monoTonePolys;

		Adapter.adapt(dir, segDist[curSeg], gVcmProjPolys[dir][curSeg][tid], vcmProjPolys[dir][curSeg][tid]);

		for (int i = 0; i<vp.size(); i++)
		{
			//cout<<endl;
			reverse(vp[i].begin(), vp[i].end());
			vector< vector<Vector> >temp = M.getMonotonePartition(vp[i]);
			for (int j = 0; j<temp.size(); j++)monoTonePolys.push_back(temp[j]);
		}


		for (int i = 0; i<monoTonePolys.size(); i++)
		{
			monoPolygonTo2Darray(dir, curSeg, monoTonePolys[i]);
		}

	}



}

void VCM::generateMapInPosY(int tid, int dir)
{


	Vector3 tp = targetPoints[tid];
	//int dir=1;

	int curSeg = 1;

	vector<Box>obstacleWrtPoint;
	sort(dirObstacle[dir].begin(), dirObstacle[dir].end(), fcmpPosY);
	//	soet(dirObstacle[dir].begin(),dirObstacle[dir].end());

	for (int i = 0; i<dirObstacle[dir].size(); i++)
	{
		Box u = dirObstacle[dir][i];
		Box projPoly1;
		bool b = commonBox(target, u, projPoly1);
		if (b)continue;
		while (curSeg<NDISTESEG && tp.y + segDist[curSeg] <= u.y[1])
		{
			updateVcm1(obstacleWrtPoint, dir, curSeg, tid); // eikhane optimize korbo aro...
			curSeg++;

			//	Adapter.adapt( dir,tp.x+segDist[curSeg],gVcmProjPolys[dir][curSeg][tid],vcmProjPolys[dir][curSeg][tid] );
		}

		int lastSeg = curSeg - 1;

		//vector< vector<Vector3> > &p=vcmProjPolys[dir][curSeg-1][tid];
		gpc_polygon &g = gVcmProjPolys[dir][lastSeg][tid];

		Box projPoly;
		Plane projPlane = Plane(unitDir[dir].x, unitDir[dir].y, unitDir[dir].z, segDist[lastSeg]); //change here for direction
		Box boundRect = projRect[dir][lastSeg];

		gpc_polygon gProjPoly;

		calcCovexProjectionWrtPoint(projPlane, tp, u, gProjPoly, dir);

		if (Adapter.contains(g, gProjPoly))
		{
			//printf("discarding id %d view blocked by other obstacles\n",u);
			continue;
		}
		obstacleWrtPoint.resize(obstacleWrtPoint.size() + 1);
		obstacleWrtPoint.back() = (u);
	}


	while (curSeg<NDISTESEG)
	{
		updateVcm1(obstacleWrtPoint, dir, curSeg, tid); // eikhane optimize korbo aro...
		curSeg++;

		//	Adapter.adapt( dir,tp.x+segDist[curSeg],gVcmProjPolys[dir][curSeg][tid],vcmProjPolys[dir][curSeg][tid] );

	}

	for (int curSeg = 1; curSeg< NDISTESEG; curSeg++)
	{

		gpc_polygon &g = gVcmProjPolys[dir][curSeg][tid];

		//cout<<g.num_contours<<endl;

		Box prec = projRect[dir][curSeg];
		gpc_polygon gprec;

		Adapter.adapt(prec, gprec);
		Adapter.polygonIntersection(g, gprec);


		vector< vector<Vector> >vp;

		Adapter.adapt(g, vp); //vp contains all separate polygons

		vector< vector<Vector> >monoTonePolys;


		Adapter.adapt(dir, segDist[curSeg], gVcmProjPolys[dir][curSeg][tid], vcmProjPolys[dir][curSeg][tid]);

		for (int i = 0; i<vp.size(); i++)
		{
			//cout<<endl;
			reverse(vp[i].begin(), vp[i].end());
			vector< vector<Vector> >temp = M.getMonotonePartition(vp[i]);
			for (int j = 0; j<temp.size(); j++)monoTonePolys.push_back(temp[j]);
		}


		for (int i = 0; i<monoTonePolys.size(); i++)
		{
			monoPolygonTo2Darray(dir, curSeg, monoTonePolys[i]);
		}

	}

}

void VCM::generateMapInPosZ(int tid, int dir)
{

	Vector3 tp = targetPoints[tid];
	//int dir=1;

	int curSeg = 1;

	vector<Box>obstacleWrtPoint;
	sort(dirObstacle[dir].begin(), dirObstacle[dir].end(), fcmpPosZ);
	//	soet(dirObstacle[dir].begin(),dirObstacle[dir].end());

	for (int i = 0; i<dirObstacle[dir].size(); i++)
	{
		Box u = dirObstacle[dir][i];

		Box projPoly1;
		bool b = commonBox(target, u, projPoly1);

		if (b)continue;


		while (curSeg<NDISTESEG && tp.z + segDist[curSeg] <= u.z[1])
		{

			updateVcm1(obstacleWrtPoint, dir, curSeg, tid); // eikhane optimize korbo aro...
			curSeg++;

			//	Adapter.adapt( dir,tp.x+segDist[curSeg],gVcmProjPolys[dir][curSeg][tid],vcmProjPolys[dir][curSeg][tid] );

		}

		int lastSeg = curSeg - 1;

		//vector< vector<Vector3> > &p=vcmProjPolys[dir][curSeg-1][tid];
		gpc_polygon &g = gVcmProjPolys[dir][lastSeg][tid];

		Box projPoly;
		Plane projPlane = Plane(unitDir[dir].x, unitDir[dir].y, unitDir[dir].z, segDist[lastSeg]); //change here for direction
		Box boundRect = projRect[dir][lastSeg];

		gpc_polygon gProjPoly;

		calcCovexProjectionWrtPoint(projPlane, tp, u, gProjPoly, dir);

		if (Adapter.contains(g, gProjPoly))
		{
			//printf("discarding id %d view blocked by other obstacles\n",u);
			continue;
		}
		obstacleWrtPoint.resize(obstacleWrtPoint.size() + 1);
		obstacleWrtPoint.back() = (u);
	}


	while (curSeg<NDISTESEG)
	{
		updateVcm1(obstacleWrtPoint, dir, curSeg, tid); // eikhane optimize korbo aro...
		curSeg++;

		//	Adapter.adapt( dir,tp.x+segDist[curSeg],gVcmProjPolys[dir][curSeg][tid],vcmProjPolys[dir][curSeg][tid] );

	}

	for (int curSeg = 1; curSeg< NDISTESEG; curSeg++)
	{

		gpc_polygon &g = gVcmProjPolys[dir][curSeg][tid];

		//cout<<g.num_contours<<endl;

		Box prec = projRect[dir][curSeg];
		gpc_polygon gprec;

		Adapter.adapt(prec, gprec);
		Adapter.polygonIntersection(g, gprec);


		vector< vector<Vector> >vp;

		Adapter.adapt(g, vp); //vp contains all separate polygons

		vector< vector<Vector> >monoTonePolys;


		Adapter.adapt(dir, segDist[curSeg], gVcmProjPolys[dir][curSeg][tid], vcmProjPolys[dir][curSeg][tid]);

		for (int i = 0; i<vp.size(); i++)
		{
			//cout<<endl;
			reverse(vp[i].begin(), vp[i].end());
			vector< vector<Vector> >temp = M.getMonotonePartition(vp[i]);
			for (int j = 0; j<temp.size(); j++)monoTonePolys.push_back(temp[j]);
		}


		for (int i = 0; i<monoTonePolys.size(); i++)
		{
			monoPolygonTo2Darray(dir, curSeg, monoTonePolys[i]);
		}

	}

}

void VCM::generateMapInNegX(int tid, int dir)
{

	Vector3 tp = targetPoints[tid]; // target er upor ekta point nilam
	//vector< int >&obstacleId=dirObstacleId[dir]; // oi direction er obstacle er list ta nilam

	int curSeg = 1;

	vector<Box>obstacleWrtPoint; // ei khane thakbe amar kon kon obstacle nia deal korte hobe
	sort(dirObstacle[dir].begin(), dirObstacle[dir].end(), fcmpNegX);

	for (int i = 0; i<dirObstacle[dir].size(); i++)
	{
		Box u = dirObstacle[dir][i];

		Box projPoly1;
		bool b = commonBox(target, u, projPoly1);

		if (b)continue;

		while (curSeg<NDISTESEG && tp.x - segDist[curSeg] >= u.x[0])
		{

			updateVcm1(obstacleWrtPoint, dir, curSeg, tid); // eikhane optimize korbo aro...
			curSeg++;

			//	Adapter.adapt( dir,tp.x+segDist[curSeg],gVcmProjPolys[dir][curSeg][tid],vcmProjPolys[dir][curSeg][tid] );

		}

		int lastSeg = curSeg - 1;

		//vector< vector<Vector3> > &p=vcmProjPolys[dir][curSeg-1][tid];
		gpc_polygon &g = gVcmProjPolys[dir][lastSeg][tid];

		Box projPoly;
		Plane projPlane = Plane(unitDir[dir].x, unitDir[dir].y, unitDir[dir].z, segDist[lastSeg]); //change here for direction
		Box boundRect = projRect[dir][lastSeg];

		gpc_polygon gProjPoly;

		calcCovexProjectionWrtPoint(projPlane, tp, u, gProjPoly, dir);

		if (Adapter.contains(g, gProjPoly))
		{
			//printf("discarding id %d view blocked by other obstacles\n",u);
			continue;
		}
		obstacleWrtPoint.resize(obstacleWrtPoint.size() + 1);
		obstacleWrtPoint.back() = (u);
	}


	while (curSeg<NDISTESEG)
	{
		updateVcm1(obstacleWrtPoint, dir, curSeg, tid); // eikhane optimize korbo aro...
		curSeg++;

		//	Adapter.adapt( dir,tp.x+segDist[curSeg],gVcmProjPolys[dir][curSeg][tid],vcmProjPolys[dir][curSeg][tid] );

	}

	for (int curSeg = 1; curSeg< NDISTESEG; curSeg++)
	{

		gpc_polygon &g = gVcmProjPolys[dir][curSeg][tid];

		//cout<<g.num_contours<<endl;

		Box prec = projRect[dir][curSeg];
		gpc_polygon gprec;

		Adapter.adapt(prec, gprec);
		Adapter.polygonIntersection(g, gprec);


		vector< vector<Vector> >vp;

		Adapter.adapt(g, vp); //vp contains all separate polygons

		vector< vector<Vector> >monoTonePolys;


		Adapter.adapt(dir, segDist[curSeg], gVcmProjPolys[dir][curSeg][tid], vcmProjPolys[dir][curSeg][tid]);

		for (int i = 0; i<vp.size(); i++)
		{
			//cout<<endl;
			reverse(vp[i].begin(), vp[i].end());
			vector< vector<Vector> >temp = M.getMonotonePartition(vp[i]);
			for (int j = 0; j<temp.size(); j++)monoTonePolys.push_back(temp[j]);
		}

		for (int i = 0; i<monoTonePolys.size(); i++)
		{
			monoPolygonTo2Darray(dir, curSeg, monoTonePolys[i]);
		}

	}

}

void VCM::generateMapInNegY(int tid, int dir)
{

	Vector3 tp = targetPoints[tid]; // target er upor ekta point nilam
	//vector< int >&obstacleId=dirObstacleId[dir]; // oi direction er obstacle er list ta nilam

	int curSeg = 1;

	vector<Box>obstacleWrtPoint; // ei khane thakbe amar kon kon obstacle nia deal korte hobe
	sort(dirObstacle[dir].begin(), dirObstacle[dir].end(), fcmpNegY);

	for (int i = 0; i<dirObstacle[dir].size(); i++)
	{
		Box u = dirObstacle[dir][i];

		Box projPoly1;
		bool b = commonBox(target, u, projPoly1);

		if (b)continue;


		while (curSeg<NDISTESEG && tp.y - segDist[curSeg] >= u.y[0])
		{

			updateVcm1(obstacleWrtPoint, dir, curSeg, tid); // eikhane optimize korbo aro...
			curSeg++;

			//	Adapter.adapt( dir,tp.x+segDist[curSeg],gVcmProjPolys[dir][curSeg][tid],vcmProjPolys[dir][curSeg][tid] );

		}

		int lastSeg = curSeg - 1;

		//vector< vector<Vector3> > &p=vcmProjPolys[dir][curSeg-1][tid];
		gpc_polygon &g = gVcmProjPolys[dir][lastSeg][tid];

		Box projPoly;
		Plane projPlane = Plane(unitDir[dir].x, unitDir[dir].y, unitDir[dir].z, segDist[lastSeg]); //change here for direction
		Box boundRect = projRect[dir][lastSeg];

		gpc_polygon gProjPoly;

		calcCovexProjectionWrtPoint(projPlane, tp, u, gProjPoly, dir);

		if (Adapter.contains(g, gProjPoly))
		{
			//printf("discarding id %d view blocked by other obstacles\n",u);
			continue;
		}
		obstacleWrtPoint.resize(obstacleWrtPoint.size() + 1);
		obstacleWrtPoint.back() = (u);
	}


	while (curSeg<NDISTESEG)
	{
		updateVcm1(obstacleWrtPoint, dir, curSeg, tid); // eikhane optimize korbo aro...
		curSeg++;

		//	Adapter.adapt( dir,tp.x+segDist[curSeg],gVcmProjPolys[dir][curSeg][tid],vcmProjPolys[dir][curSeg][tid] );

	}

	for (int curSeg = 1; curSeg< NDISTESEG; curSeg++)
	{

		gpc_polygon &g = gVcmProjPolys[dir][curSeg][tid];

		//cout<<g.num_contours<<endl;

		Box prec = projRect[dir][curSeg];
		gpc_polygon gprec;

		Adapter.adapt(prec, gprec);
		Adapter.polygonIntersection(g, gprec);


		vector< vector<Vector> >vp;

		Adapter.adapt(g, vp); //vp contains all separate polygons

		vector< vector<Vector> >monoTonePolys;


		Adapter.adapt(dir, segDist[curSeg], gVcmProjPolys[dir][curSeg][tid], vcmProjPolys[dir][curSeg][tid]);

		for (int i = 0; i<vp.size(); i++)
		{
			//cout<<endl;
			reverse(vp[i].begin(), vp[i].end());
			vector< vector<Vector> >temp = M.getMonotonePartition(vp[i]);
			for (int j = 0; j<temp.size(); j++)monoTonePolys.push_back(temp[j]);
		}

		for (int i = 0; i<monoTonePolys.size(); i++)
		{
			monoPolygonTo2Darray(dir, curSeg, monoTonePolys[i]);
		}

	}



}

void VCM::generateMapInNegZ(int tid, int dir)
{

	Vector3 tp = targetPoints[tid]; // target er upor ekta point nilam
	//vector< int >&obstacleId=dirObstacleId[dir]; // oi direction er obstacle er list ta nilam

	int curSeg = 1;

	vector<Box>obstacleWrtPoint; // ei khane thakbe amar kon kon obstacle nia deal korte hobe
	sort(dirObstacle[dir].begin(), dirObstacle[dir].end(), fcmpNegZ);

	for (int i = 0; i<dirObstacle[dir].size(); i++)
	{
		Box u = dirObstacle[dir][i];

		Box projPoly1;
		bool b = commonBox(target, u, projPoly1);

		if (b)continue;


		while (curSeg<NDISTESEG && tp.z - segDist[curSeg] >= u.z[0])
		{

			updateVcm1(obstacleWrtPoint, dir, curSeg, tid); // eikhane optimize korbo aro...
			curSeg++;

			//	Adapter.adapt( dir,tp.x+segDist[curSeg],gVcmProjPolys[dir][curSeg][tid],vcmProjPolys[dir][curSeg][tid] );

		}

		int lastSeg = curSeg - 1;

		//vector< vector<Vector3> > &p=vcmProjPolys[dir][curSeg-1][tid];
		gpc_polygon &g = gVcmProjPolys[dir][lastSeg][tid];

		Box projPoly;
		Plane projPlane = Plane(unitDir[dir].x, unitDir[dir].y, unitDir[dir].z, segDist[lastSeg]); //change here for direction
		Box boundRect = projRect[dir][lastSeg];

		gpc_polygon gProjPoly;

		calcCovexProjectionWrtPoint(projPlane, tp, u, gProjPoly, dir);

		if (Adapter.contains(g, gProjPoly))
		{
			//printf("discarding id %d view blocked by other obstacles\n",u);
			continue;
		}
		obstacleWrtPoint.resize(obstacleWrtPoint.size() + 1);
		obstacleWrtPoint.back() = (u);
	}


	while (curSeg<NDISTESEG)
	{
		updateVcm1(obstacleWrtPoint, dir, curSeg, tid); // eikhane optimize korbo aro...
		curSeg++;

		//	Adapter.adapt( dir,tp.x+segDist[curSeg],gVcmProjPolys[dir][curSeg][tid],vcmProjPolys[dir][curSeg][tid] );

	}

	for (int curSeg = 1; curSeg< NDISTESEG; curSeg++)
	{

		gpc_polygon &g = gVcmProjPolys[dir][curSeg][tid];

		//cout<<g.num_contours<<endl;

		Box prec = projRect[dir][curSeg];
		gpc_polygon gprec;

		Adapter.adapt(prec, gprec);
		Adapter.polygonIntersection(g, gprec);


		vector< vector<Vector> >vp;

		Adapter.adapt(g, vp); //vp contains all separate polygons

		vector< vector<Vector> >monoTonePolys;


		Adapter.adapt(dir, segDist[curSeg], gVcmProjPolys[dir][curSeg][tid], vcmProjPolys[dir][curSeg][tid]);

		for (int i = 0; i<vp.size(); i++)
		{
			//cout<<endl;
			reverse(vp[i].begin(), vp[i].end());
			vector< vector<Vector> >temp = M.getMonotonePartition(vp[i]);
			for (int j = 0; j<temp.size(); j++)monoTonePolys.push_back(temp[j]);
		}

		for (int i = 0; i<monoTonePolys.size(); i++)
		{
			monoPolygonTo2Darray(dir, curSeg, monoTonePolys[i]);
		}

	}

}


void VCM::generateMap(int tid, int dir)
{
	if (dir == 0)generateMapInNegX(tid, dir);
	if (dir == 1)generateMapInPosX(tid, dir);
	if (dir == 2)generateMapInNegY(tid, dir);
	if (dir == 3)generateMapInPosY(tid, dir);
	if (dir == 4)generateMapInNegZ(tid, dir);
	if (dir == 5)generateMapInPosZ(tid, dir);

}

void VCM::reduction(int dir)
{
	if (dir == 1)reductionInPosX(1);
	if (dir == 0)reductionInNegX(0);
	if (dir == 3)reductionInPosY(3);
	if (dir == 2)reductionInNegY(2);
	if (dir == 5)reductionInPosZ(5);
	if (dir == 4)reductionInNegZ(4);
}

// for partial visibility
void VCM::generateTargetPoints(int dir)
{
	targetPoints.clear();
	//(totp+1)*(totp+1) points
	double delta = target.y[1] - target.y[0];
	// totp = 1;		// debug
	delta /= totp;
	// debug
	//targetPoints.push_back(Vector3((target.x[1] + target.x[0]) / 2, target.y[1], (target.z[1] + target.z[0]) / 2));
	//return;
	/* for text data, generate points only in one face */
	double x0 = target.x[0], x1 = target.x[1], y0 = target.y[0], y1 = target.y[1], z0 = target.z[0], z1 = target.z[1];
	for (double x = target.x[0]; x <= target.x[1]; x += delta)
	{
		if (dir <= 1) x = x1;
		for (double y = target.y[0]; y <= target.y[1]; y += delta)
		{
			if (dir == 2 || dir == 3) y = y1;
			for (double z = target.z[0]; z <= target.z[1]; z += delta)
			{
				if (dir == 4 || dir == 5) z = z1;
				if (dir == 0)
					targetPoints.push_back(Vector3(x0, y, z));
				else if (dir == 1)
					targetPoints.push_back(Vector3(x1, y, z));
				else if (dir == 2)
					targetPoints.push_back(Vector3(x, y0, z));
				else if (dir == 3)
					targetPoints.push_back(Vector3(x, y1, z));
				else if (dir == 4)
					targetPoints.push_back(Vector3(x, y, z0));
				else if (dir == 5)
					targetPoints.push_back(Vector3(x, y, z1));
			}
		}
	}
}


void VCM::buildTree(char *path, char *DATAFILENAME, char *Outpath)
{

	char TREEFILE[200];
	strcpy(TREEFILE, path);
	strcat(TREEFILE, DATAFILENAME);
	strcat(TREEFILE, ".tree");

	//input file

	char DATAFILE[200];
	strcpy(DATAFILE, path);
	strcat(DATAFILE, DATAFILENAME);
	strcat(DATAFILE, ".txt");

	int blocksize = 1024;
	int b_length = 1024;

	int dimension = 3;

	char OUTFILE[200] = "";

	r = new RTreeCmdIntrpr();

	//puts(DATAFILE);

	r->build_tree(TREEFILE, DATAFILE, b_length, dimension, 0);

	rtree = r->tree;

	//rtree->reductionInPosX(1,target);

	return;
}

void VCM::createRtreeFromData(char *f)
{

	// home pc directory
	char *Inpath = "";
	//char *Outpath = "H:\\Study\\VCM\ Demo\\vcm_3D\\3D\\3D\\db\\";
	char Outpath[200];
	strcpy(Outpath, "");
	//char *Inpath = "G:\\VCM3DUpdated\\vcm-3d-updated\\3D\\3D\\";
	//char *Outpath = "G:\\VCM3DUpdated\\vcm-3d-updated\\3D\\3D\\";
	//puts(f);
	char *t = new char[strlen(f) + 1];
	char *s = t;
	while (*f != '.' && *f != 0)
		*t++ = *f++;
	*t = '\0';
	char *DATAFILENAME = s;
	//puts(s);
	//int NumOfObstacles = 3;

	buildTree(Inpath, DATAFILENAME, Outpath);

	return;
}

bool VCM::rayRectangleIntersction(Vector3 &u, Vector3 &v, Plane &p)
{
	Vector3 I;
	if (!p.rayPlaneIntersction(u, v, I))return false;

	if (p.v0.x <= I.x && p.v1.x >= I.x)
	{
		if (p.v0.y <= I.y && p.v1.y >= I.y)
		{
			if (p.v0.z <= I.z && p.v1.z >= I.z)
			{
				return true;
			}
		}
	}

	return false;
}

bool VCM::rayBoxIntersection(Vector3 &u, Vector3 &v, Box &b)
{


	if (rayRectangleIntersction(u, v, b.px[0]))return true;
	if (rayRectangleIntersction(u, v, b.px[1]))return true;
	if (rayRectangleIntersction(u, v, b.py[0]))return true;
	if (rayRectangleIntersction(u, v, b.py[1]))return true;
	if (rayRectangleIntersction(u, v, b.pz[0]))return true;
	if (rayRectangleIntersction(u, v, b.pz[1]))return true;


	return false;
}

int VCM::naiveVisibility(Vector3 v)
{
	int cnt = 0;
	for (int i = 0; i<targetPoints.size(); i++)
	{
		Vector3 u = targetPoints[i];

		for (int j = 0; j<obstacles.size(); j++)
		{
			if (rayBoxIntersection(u, v, obstacles[j]))
			{
				cnt++;
				break;
			}
		}

	}

	return cnt;

}

void VCM::bruteForce()
{
	int n;

	Vector3 c;
	FILE *tfp;
	if ((tfp = fopen("target.txt", "r")) == NULL)
	{
		printf("Can not find the target file\n");
	}
	else
	{
		c.fscan(tfp);
		printf("target is centerd at %lf %lf %lf\n", c.x, c.y, c.z);
	}

	FILE* fp = fopen("sample1.txt", "r");
	fscanf(fp, "%d", &n);

	//n=100;


	//obstacles=vector<Box>(n);

	Vector3 v0, v1;



	for (int i = 0; i<n; i++)
	{
		v0.fscan(fp);
		v1.fscan(fp);

		v0 = v0.sub(c);
		v1 = v1.sub(c);


		obstacles.resize(obstacles.size() + 1);

		obstacles.back().init(v0, v1, Origin);
		obstacles.back().setPlanes();

		Box b;
		if (commonBox(target, obstacles.back(), b))obstacles.pop_back();


	}

	int dir = 1;

	for (int i = 0; i<NDISTESEG; i++)
	{
		printf("%d\n", i);

		Vector3 v;
		v.x = segDist[i];

		double dim = getDim(dir, i);
		double delta = dim / MAXSEG;

		double lz = -dim / 2.0; lz += delta / 2.0;



		for (int j = 0; j<MAXSEG; j++)
		{
			double ly = -dim / 2.0; ly += delta / 2.0;
			for (int k = 0; k<MAXSEG; k++)
			{

				v.y = ly;
				v.z = lz;


				//naiveVcmArray[dir][i][j][k]=naiveVisibility( v );
				vcmArray[dir][i][j][k] = naiveVisibility(v);



				//v.print();

				ly += delta;
			}
			lz += delta;
		}
	}

}

void VCM::initialize()
{
	//cout<<"1";
	for (int i = 0; i < obstacles.size(); ++i) {
		int r = rand() % rColor.size();
		obstacles[i].color = Vector3(rColor[r].x, rColor[r].y, rColor[r].z);
	}
	int dir = 1;
	segDist.clear();
	for (int i = 0; i<6; i++)totSeg[i] = 0;

	for (int dir = 0; dir < 6; dir++) {
		for (int i = 0; i<NDISTESEG; i++)gpc_free_polygon(&gProjPolys[dir][i]);
		for (int i = 0; i<NDISTESEG; i++)ProjPolys[dir][i].clear();
		for (int i = 0; i<NDISTESEG; i++)
		{
			for (int j = 0; j<targetPoints.size(); j++)gpc_free_polygon(&gVcmProjPolys[dir][i][j]);
		}//cout<<"4";
		for (int i = 0; i<NDISTESEG; i++)
		{
			for (int j = 0; j<targetPoints.size(); j++)vcmProjPolys[dir][i][j].clear();
		}//cout<<"5";
		dirObstacleId[dir].clear();
		dirObstacle[dir].clear();
		for (int i = 0; i<NDISTESEG; i++)
		{
			for (int j = 0; j<MAXSEG; j++)
			{
				for (int k = 0; k<MAXSEG; k++)
				{
					vcmArray[dir][i][j][k] = 0;
					naiveVcmArray[dir][i][j][k] = 0;
				}
			}
		}
	}

	targetPoints.clear();


	//cout<<"6";

	for (int i = 0; i<8; i++)curProjPlane1[i] = 0;

	for (int i = 0; i<8; i++)gpc_free_polygon(&projWrtTargetCorner[i]);//cout<<"7";
	for (int i = 0; i<6; i++)curProjPlane[i] = 0;

	All.clear();
	dstFromTar.clear();
	rColor.clear();
	//cout<<"8";

	//cout<<"9";
	if (r) r->free_tree();
	//cout<<"10";
	return;
}

void VCM::clearTarget() {
	if (isTargetSet) {
		Box b;
		b.init(Vector3(0, 0, 0), Vector3(0, 0, 0), Vector3(0, 0, 0));
		target = b;
	}
}

void VCM::setTarget(Box &b) {

	translate.x = b.o.x;
	translate.y = b.o.y;
	translate.z = b.o.z;
	target.init(Vector3(b.x[0] - translate.x, b.y[0] - translate.y, b.z[0] - translate.z), Vector3(b.x[1] - translate.x, b.y[1] - translate.y, b.z[1] - translate.z), Origin);
	target.setPlanes();
	for (int i = 0; i < obstacles.size(); ++i) {
		obstacles[i].init(Vector3(obstacles[i].x[0] - translate.x, obstacles[i].y[0] - translate.y, obstacles[i].z[0] - translate.z),
			Vector3(obstacles[i].x[1] - translate.x, obstacles[i].y[1] - translate.y, obstacles[i].z[1] - translate.z),
			Origin
			);
		if (obstacles[i].color.x == 255 && !obstacles[i].color.y && !obstacles[i].color.z) {
			int r = rand() % rColor.size();
			obstacles[i].color = Vector3(rColor[r].x, rColor[r].y, rColor[r].z);
		}
	}
	
	
	target.color = Vector3(0, 255, 255);
	isTargetSet = true;
	console_index = TARGETSET;
	curSegId = 0;
}

void VCM::run(char *f)
{
	initialize();

	logfp = fopen("log.txt", "w");
	efile = fopen("efile.txt", "w");

	//input();	

	// 
	//5985

	int testCase = 1;

	for (int it = 1; it <= testCase; it++)
	{
		clock_t startTime = clock();


		//	Vector3 v0(-7, -7, -7);
		//	Vector3 v1(+7, +7, +7);

		//Origin=v0.add(v1).div(2);
		//Origin.print();

		//v0=v0.sub(Origin);
		//v1=v1.sub(Origin);

		//v0.print();
		//v1.print();

		//1750.00 1800.00 200.00

		//return ;

		//23.00 23.00 23.00

		//	target.print();
		//	while(1);

		calcSegmentDistance(1);
		generateRandCol();
		calcProjectionPlanes();

		int dir = 1;
		generateTargetPoints(dir);

		//bruteForce();	
		createRtreeFromData(f);
		reductionInPosXRtree(1);
		//reductionInPosX(1);

		{
			fprintf(logfp, "%d\n", targetPoints.size());
			printf("target points %d\n", targetPoints.size());
			for (int i = 0; i<targetPoints.size(); i++)
			{
				fprintf(logfp, "%d ", i);
				printf("%d ", i);
				generateMap(i, dir);
			}
			fprintf(logfp, "\n");
			printf("\n");


			for (int curSeg = 1; curSeg<NDISTESEG; curSeg++)
			{
				mergeAll(dir, curSeg);

			}
		}


		double timeneeded = (double)(clock() - startTime) / (double)CLOCKS_PER_SEC;

		fprintf(efile, "execution time %lf seconds.\n", timeneeded);
		printf("test case %d : execution time %lf seconds.\n", it, timeneeded);

		totTime += timeneeded;

		//	initialize();

	}
	fprintf(logfp, "avg discarded %lf   avg reduced obstacle size %lf   avg time %lf\n", (double)totDiscard / (double)testCase, (double)totCnt / (double)testCase, totTime / (double)testCase);
	fclose(logfp);
	fclose(efile);
}

void VCM::run2(int dir) {
	//initialize();
	//target.o = Vector3(0, 0, 0);
	logfp = fopen("log.txt", "w");
	efile = fopen("efile.txt", "w");

	printf("Running VCM ... \n");

	//input();	

	// 
	//5985

	int testCase = 1;

	for (int it = 1; it <= testCase; it++)
	{
		clock_t startTime = clock();


		//	Vector3 v0(-7, -7, -7);
		//	Vector3 v1(+7, +7, +7);

		//Origin=v0.add(v1).div(2);
		//Origin.print();

		//v0=v0.sub(Origin);
		//v1=v1.sub(Origin);

		//v0.print();
		//v1.print();

		//1750.00 1800.00 200.00

		//return ;

		//23.00 23.00 23.00

		//	target.print();
		//	while(1);

		//int dir = curr_string;
		generateRandCol();
		calcSegmentDistance(dir);
		calcProjectionPlanes();

		generateTargetPoints(dir);

		//bruteForce();	
		//createRtreeFromData(f);
		//reductionInPosXRtree(dir);
		//reductionInPosY(dir);
		reduction(dir);
		{
			fprintf(logfp, "%d\n", targetPoints.size());
			//printf("target points %d\n", targetPoints.size());
			for (int i = 0; i<targetPoints.size(); i++)
			{
				fprintf(logfp, "%d ", i);
				//printf("%d ", i);
				//targetPoints[i].print();	// debug
				generateMap(i, dir);
				//break;
			}
			fprintf(logfp, "\n");
			//printf("\n");


			for (int curSeg = 1; curSeg<NDISTESEG; curSeg++)
			{
				mergeAll(dir, curSeg);
				// debug
				/*if (curSeg == NDISTESEG - 1) {
				cout << "\nafter mergeall \n";
				for (int i = 0; i < MAXSEG; ++i) {
				for (int j = 0; j < MAXSEG; ++j)
				cout << vcmArray[dir][curSeg][i][j];
				cout << endl;
				}
				}*/
			}
		}
		ofstream out;
		out.open("vcm_output.txt");
		cout << "ndisteseg: " << NDISTESEG << endl;
		cout << "maxseg: " << MAXSEG << endl;
		cout << "totp: " << targetPoints.size() << endl;
		for (int curSeg = 1; curSeg < NDISTESEG; curSeg++) {
			cout << "curseg;;" << endl;
			cout << "dim: " << getDim(dir, curSeg) << endl;
			cout << "segdist: " << segDist[curSeg] << endl;
			for (int i = 0; i < MAXSEG; ++i) {
				for (int j = 0; j < MAXSEG; ++j) {
					cout << vcmArray[dir][curSeg][i][j] << " ";
					//printf("%d ", vcmArray[dir][curSeg][i][j]);
				}
				//printf("\n");
				cout << endl;
			}
			cout << ";;curseg" << endl;
		}
		out.close();


		double timeneeded = (double)(clock() - startTime) / (double)CLOCKS_PER_SEC;

		fprintf(efile, "execution time %lf seconds.\n", timeneeded);
		//printf("test case %d : execution time %lf seconds.\n", it, timeneeded);

		totTime += timeneeded;

		//	initialize();

	}
	// after translation complete
	vcmGenerated = true;
	fprintf(logfp, "avg discarded %lf   avg reduced obstacle size %lf   avg time %lf\n", (double)totDiscard / (double)testCase, (double)totCnt / (double)testCase, totTime / (double)testCase);
	fclose(logfp);
	fclose(efile);
	printf("Exiting VCM\n");
}
///
void VCM::debug3()
{
	glPushMatrix(); {

		glTranslated(1750, 1800, 200);
		glColor3ub(0, 0, 0);
		glutSolidCube(100);


	}glPopMatrix();

	for (int i = 0; i<lo.size(); i++)
	{
		Vector3 v0 = lo[i];
		Vector3 v1 = up[i];
		Vector3 c = v0.add(v1).div(2);


		glPushMatrix(); {

			glColor3ub(CO(rColor[i % 3]));
			glTranslated(CO(c));
			glScaled(CO(Vector3(v1.sub(v0))));
			glutSolidCube(1);

		}glPopMatrix();

	}
}
void VCM::debug4()
{
	//return;
	//glColor3ub(0,0,0);
	draw_axis();
	target.draw();
	//cout << dirObstacle[1].size() << endl;
	for (int i = 0; i<dirObstacle[1].size(); i++)
	{
		int j = i%rColor.size();
		//glColor3ub( rColor[j].x,rColor[j].y,rColor[j].z );
		dirObstacle[1][i].draw();
	}
	//cout << All.size() << endl;
	if (showAll)
	for (int i = 0; i<All.size(); i++)
	{
		//int j=i%rColor.size();
		//glColor3ub( rColor[j].x,rColor[j].y,rColor[j].z );
		//glColor3ub( 255,0,0 );
		All[i].draw();
	}

	//return;

	for (int curSeg = curSegId, dir = 1; curSeg <= curSegId; curSeg++)
	{
		double dim = getDim(dir, curSeg);
		double delta = dim / MAXSEG;

		double lx = -dim / 2.0;//lx+=delta;
		double ly = -dim / 2.0;//ly+=delta;

		int tot = targetPoints.size();
		//	int dir=1;
		double z = segDist[curSeg];
		if (!dir)z = -z;// for negative z

		for (int i = 0; i<MAXSEG; i++)
		{
			for (int j = 0; j<MAXSEG; j++)
			{
				vcmArray[dir][curSeg][i][j] = min(vcmArray[dir][curSeg][i][j], tot); //discarding noise 


				double col = 255 - 255.0*((double)vcmArray[dir][curSeg][i][j] / tot);
				glPushMatrix();
				if (1 || vcmArray[dir][curSeg][i][j]){

					//		cout<<"asd"<<endl;
					//	col=0;
					glColor3ub(col, col, col);
					glBegin(GL_QUADS); {

						glVertex3d(z, lx, ly);
						glVertex3d(z, lx + delta, ly);
						glVertex3d(z, lx + delta, ly + delta);
						glVertex3d(z, lx, ly + delta);

					}glEnd();
				}glPopMatrix();

				lx += delta;
			}
			ly += delta;
			lx = -dim / 2.0;
		}
	}

	for (int i = 0; i<obstacles.size(); i++)
	{
		int j = i%rColor.size();
		//glColor3ub( rColor[j].x,rColor[j].y,rColor[j].z );
		obstacles[i].draw();
	}
}

double VCM::dist(Vector3 a, Vector3 b) {
	return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2) + pow(a.z - b.z, 2));
}

Vector3 VCM::arrayIndexToPoint(int row, int col, int dir, int curSeg)
{

	double dim = getDim(dir, curSeg);


	/// maping the 2D array index to y, z plane ..
	/// 0 to MAXSEG-1 range to -dim/2 to +dim2 range

	double a, b;
	a = row*dim / MAXSEG - dim*.5;
	b = col*dim / MAXSEG - dim*.5;

	if (dir == 0) {
		return Vector3(+segDist[curSeg], a, b);
	}

	if (dir == 1) {
		return Vector3(-segDist[curSeg], a, b);
	}
	if (dir == 2) {
		return Vector3(b, +segDist[curSeg], a);
	}

	if (dir == 3) {
		return Vector3(b, -segDist[curSeg], a);
	}

	if (dir == 4) {
		return Vector3(a, b, +segDist[curSeg]);
	}
	if (dir == 5) {
		return Vector3(a, b, -segDist[curSeg]);
	}



}

void VCM::debug5() {
	textbox->set_text(console_strings[console_index]);
	if (prev_string != curr_string) {
		curSegId = 0;
		prev_string = curr_string;
	}
	draw_axis();
	target.draw();
	for (int i = 0; i < obstacles.size(); ++i)
	{
		if (obstacles[i].color.x == 255 && !obstacles[i].color.y && !obstacles[i].color.z) {
			if (showAll) obstacles[i].draw();
			else continue;
		}
		else obstacles[i].draw();

	}

	if (isTargetSet && vcmGenerated)
	{
		/*
		for (int i = 0; i<dirObstacle[1].size(); i++)
		{
		//glColor3ub( rColor[j].x,rColor[j].y,rColor[j].z );
		dirObstacle[1][i].draw();
		}
		//cout << All.size() << endl;
		if (showAll)
		for (int i = 0; i<All.size(); i++)
		{
		//int j=i%rColor.size();
		//glColor3ub( rColor[j].x,rColor[j].y,rColor[j].z );
		//glColor3ub( 255,0,0 );
		All[i].draw();
		}
		*/

		for (int curSeg = curSegId, dir = curr_string; curSeg <= curSegId; curSeg++)
		{
			double dim = getDim(dir, curSeg);
			double delta = dim / MAXSEG;

			double lx = -dim / 2.0;//lx+=delta;
			double ly = -dim / 2.0;//ly+=delta;

			int tot = targetPoints.size();
			//	int dir=1;
			double z = segDist[curSeg];
			if (!(dir & 1))
				z = -z;// for negative z

			for (int i = 0; i<MAXSEG; i++)
			{
				for (int j = 0; j<MAXSEG; j++)
				{
					vcmArray[dir][curSeg][i][j] = min(vcmArray[dir][curSeg][i][j], tot); //discarding noise 
					double col = 255 - 255.0*((double)vcmArray[dir][curSeg][i][j] / tot);
					if (!text.empty()) {
						if (dist(target.c, arrayIndexToPoint(i, j, dir, curSeg)) > 180) {
							col = 0;
						}
					}

					glPushMatrix();
					if (1 || vcmArray[dir][curSeg][i][j]){

						//		cout<<"asd"<<endl;
						//	col=0;
						glColor3ub(col, col, col);
						if (dir <= 1) {
							glBegin(GL_QUADS); {

								glVertex3d(z, lx, ly);
								glVertex3d(z, lx + delta, ly);
								glVertex3d(z, lx + delta, ly + delta);
								glVertex3d(z, lx, ly + delta);

							}glEnd();
						}
						else if (dir == 3 || dir == 2) {
							glBegin(GL_QUADS); {

								glVertex3d(lx, z, ly);
								glVertex3d(lx + delta, z, ly);
								glVertex3d(lx + delta, z, ly + delta);
								glVertex3d(lx, z, ly + delta);

							}glEnd();
						}
						else {
							glBegin(GL_QUADS); {

								glVertex3d(lx, ly, z);
								glVertex3d(lx + delta, ly, z);
								glVertex3d(lx + delta, ly + delta, z);
								glVertex3d(lx, ly + delta, z);

							}glEnd();
						}

					}glPopMatrix();

					lx += delta;
				}
				ly += delta;
				lx = -dim / 2.0;
			}
		}
	}
	GLfloat ambientColor[] = { 0.7f, 0.7f, 0.7f, 1.0f };
	glLightModelfv(GL_LIGHT_MODEL_AMBIENT, ambientColor);

	GLfloat lightColor0[] = { 0.6f, 0.6f, 0.6f, 1.0f };
	GLfloat lightPos0[] = { -0.5f, 0.5f, 1.0f, 0.0f };
	glLightfv(GL_LIGHT0, GL_DIFFUSE, lightColor0);
	glLightfv(GL_LIGHT0, GL_POSITION, lightPos0);
	/* draw text */
	if (!text.empty() && isTargetSet) {
		_scale = fontsize / 10.0;
		glColor3f(1.0f, 0.0f, 0.3f);
		if (curr_string == 1) {
			glPushMatrix();
			glRotatef(90, 1, 0, 0);
			glRotatef(90, 0, 1, 0);
			glTranslatef(0, 0, target.lx / 2);
			glScalef(_scale, _scale, _scale);
			t3dDraw3D(text, 0, 0, 0.2f);
			glPopMatrix();
		}
		else if (curr_string == 0) {
			glPushMatrix();
			glRotatef(90, 1, 0, 0);
			glRotatef(-90, 0, 1, 0);
			glTranslatef(0, 0, target.lx / 2);
			glScalef(_scale, _scale, _scale);
			t3dDraw3D(text, 0, 0, 0.2f);
			glPopMatrix();
		}
		else if (curr_string == 2) {
			glPushMatrix();
			glRotatef(90, 1, 0, 0);
			glTranslatef(0, 0, target.ly / 2);
			glScalef(_scale, _scale, _scale);
			t3dDraw3D(text, 0, 0, 0.2f);
			glPopMatrix();
		}
		else if (curr_string == 3) {
			glPushMatrix();
			glRotatef(90, 1, 0, 0);
			glRotatef(180, 0, 1, 0);
			glTranslatef(0, 0, target.ly / 2);
			glScalef(_scale, _scale, _scale);
			t3dDraw3D(text, 0, 0, 0.2f);
			glPopMatrix();
		}
		else if (curr_string == 4) {
			glPushMatrix();
			glRotatef(180, 1, 0, 0);
			glRotatef(90, 0, 0, 1);
			glTranslatef(0, 0, target.lz / 2);
			glScalef(_scale, _scale, _scale);
			t3dDraw3D(text, 0, 0, 0.2f);
			glPopMatrix();
		}
		else {
			glPushMatrix();
			glRotatef(-90, 0, 0, 1);
			glTranslatef(0, 0, target.lz / 2);
			glScalef(_scale, _scale, _scale);
			t3dDraw3D(text, 0, 0, 0.2f);
			glPopMatrix();

		}
	}



}

bool VCM::changeTarget(GLdouble x, GLdouble y, GLdouble z) {
	for (int i = 0; i < obstacles.size(); ++i){
		if (obstacles[i].checkInside(x, y, z)) {
			//i = i % rColor.size();
			//dirObstacle[1][i].color = Vector3(rColor[i].x, rColor[i].y, rColor[i].z);
			setTarget(obstacles[i]);
			return true;
		}
	}
	return false;
}

void VCM::setup() {
	if (isTargetSet) clearTarget();
	isTargetSet = false; vcmGenerated = false;
	obstacles.clear();
	initialize();
	//printf("Initializing world ... \n");
	createRtreeFromData(f);
	generateRandCol();
	priority_queue< int, vi, decltype(&comparatorForPq) > Q(&comparatorForPq);
	Q.push(rtree->root);
	int cnt = 0, discard = 0;
	while (!Q.empty())
	{
		int v = Q.top();
		//int v=Q.front();
		RTNode *rtn = new RTNode(rtree, v);
		Q.pop();
		for (int e = 0; e<rtn->num_entries; e++)
		{
			Entry u = rtn->entries[e];
			if (rtn->level)
			{
				dstFromTar[u.son] = u.bounces[0];
				Q.push(u.son);
				continue;
			}
			Vector3 v0(u.bounces[0], u.bounces[2], u.bounces[4]);
			Vector3 v1(u.bounces[1], u.bounces[3], u.bounces[5]);
			Box bu;
			bu.init(v0, v1, Origin);
			bu.setPlanes();
			int i = cnt % rColor.size();
			bu.color = Vector3(rColor[i].x, rColor[i].y, rColor[i].z);
			obstacles.resize(obstacles.size() + 1);
			obstacles.back() = bu;
			cnt++;
		}
	}
}

//Computes a scaling value so that the strings
void VCM::computeScale() {
	float maxWidth = 0;
	for (int i = 0; i < 4; i++) {
		float width = t3dDrawWidth(STRS[i]);
		if (width > maxWidth) {
			maxWidth = width;
		}
	}

	_scale = 10.6f / maxWidth;
}

void VCM::draw_axis(){
	glColor3f(0, 1, 0);
	glBegin(GL_LINES); {
		glVertex3f(0, -150, 0);
		glVertex3f(0, 150, 0);
	}glEnd();
	glColor3f(1, 0, 0);
	glBegin(GL_LINES); {
		glVertex3f(-150, 0, 0);
		glVertex3f(150, 0, 0);
	}glEnd();
	glColor3f(0, 0, 1);
	glBegin(GL_LINES); {
		glVertex3f(0, 0, -150);
		glVertex3f(0, 0, 150);
	}glEnd();
	glColor3f(0.5, 0.5, 0.5);
	glBegin(GL_LINES); {
		for (int i = -150; i <= 150; i += 10){
			glVertex3f(i, -150, 0);
			glVertex3f(i, 150, 0);
		}
		for (int i = -150; i <= 150; i += 10){
			glVertex3f(-150, i, 0);
			glVertex3f(150, i, 0);
		}
	}glEnd();

}

