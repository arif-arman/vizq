#ifndef VCM_H
#define VCM_H
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

#define MAXSEG 200 // MAXSEG*MAXSEG grid
#define MAXNDISTESEG 200 
#define INTSEGDIST 50
#define TOTALPOINTONTARGET 250
#define ANGRES .5

#define BLANK 0
#define LOADINGDB 1
#define DBLOADED 2
#define TARGETSET 3
#define TARGETNOTSET 4
#define GENGVCM 5
#define VCMGEND 6


class polygonAdapter //convert 3d to 2d
{
public:
	gpc_polygon poly;
	int dir;
	polygonAdapter(){}
	polygonAdapter(int dir)
	{
		this->dir = dir;
	}

	Vector3 adapt23(int dir, double dist, double x, double y)
	{
		Vector3 ret;
		if (dir == 1 || dir == 0)
		{
			ret = Vector3(dist, x, y);
		}

		if (dir == 2 || dir == 3)
		{
			ret = Vector3(x, dist, y);
			//			ret=Vector3( dist,x,y );
		}


		if (dir == 4 || dir == 5)
		{
			ret = Vector3(x, y, dist);
			//			ret=Vector3( dist,x,y );
		}

		return ret;
	}

	Vector adapt32(int dir, double x, double y, double z)
	{
		if (dir == 1 || dir == 0)return Vector(y, z);
		if (dir == 2 || dir == 3)return Vector(x, z);
		if (dir == 4 || dir == 5)return Vector(x, y);
	}

	vector<Vector3> adapt23(int dir, double dist, vector<Vector>&v)
	{
		vector<Vector3>ret;
		for (int i = 0; i<v.size(); i++)ret.push_back(adapt23(dir, dist, v[i].x, v[i].y));
		return ret;
	}

	void adapt(int dir, double dist, gpc_polygon& g, vector< vector< Vector3 > >&planerPoly) //convert 2D to 3D
	{
		for (int i = 0; i<g.num_contours; i++)
		{
			planerPoly.resize(planerPoly.size() + 1);
			for (int j = 0; j<g.contour[i].num_vertices; j++)
			{
				planerPoly.back().push_back(adapt23(dir, dist, g.contour[i].vertex[j].x, g.contour[i].vertex[j].y));
			}
		}
	}

	void adapt(Box &b, gpc_polygon &g) //convert 3D box to 2D rectangle
	{

		g.num_contours = 1;
		g.hole = 0;
		g.contour = (gpc_vertex_list *)malloc(g.num_contours*sizeof(gpc_vertex_list));
		g.contour[0].num_vertices = 4;
		g.contour[0].vertex = (gpc_vertex *)malloc(g.contour[0].num_vertices*sizeof(gpc_vertex));

		Vector3 v0 = Vector3(b.x[0], b.y[0], b.z[0]);
		Vector3 v1 = Vector3(b.x[1], b.y[1], b.z[1]);
		Vector3 dif = v1.sub(v0);
		double x[2], y[2];

		if (Z(dif.x)) //Box is in the x perp plane
		{
			x[0] = v0.y;
			x[1] = v1.y;

			y[0] = v0.z;
			y[1] = v1.z;
		}

		else if (Z(dif.y)) //Box is in the y perp plane
		{
			x[0] = v0.z;
			x[1] = v1.z;

			y[0] = v0.x;
			y[1] = v1.x;
		}


		else if (Z(dif.z)) //Box is in the z perp plane
		{
			x[0] = v0.x;
			x[1] = v1.x;

			y[0] = v0.y;
			y[1] = v1.y;
		}

		g.contour[0].vertex[0].x = x[0];
		g.contour[0].vertex[0].y = y[0];

		g.contour[0].vertex[1].x = x[0];
		g.contour[0].vertex[1].y = y[1];

		g.contour[0].vertex[2].x = x[1];
		g.contour[0].vertex[2].y = y[1];

		g.contour[0].vertex[3].x = x[1];
		g.contour[0].vertex[3].y = y[0];

	}

	void adapt(vector<Vector>&b, gpc_polygon &g)
	{
		g.num_contours = 1;
		g.hole = 0;
		g.contour = (gpc_vertex_list *)malloc(g.num_contours*sizeof(gpc_vertex_list));
		g.contour[0].num_vertices = b.size();
		g.contour[0].vertex = (gpc_vertex *)malloc(g.contour[0].num_vertices*sizeof(gpc_vertex));

		for (int i = 0; i<b.size(); i++)
		{
			g.contour[0].vertex[i].x = b[i].x;
			g.contour[0].vertex[i].y = b[i].y;
		}

	}

	void adapt(gpc_polygon &g, vector< vector<Vector> >&vp)
	{
		for (int i = 0; i<g.num_contours; i++)
		{
			vp.resize(vp.size() + 1);
			for (int j = 0; j<g.contour[i].num_vertices; j++)
			{
				vp.back().push_back(Vector(g.contour[i].vertex[j].x, g.contour[i].vertex[j].y));
			}
		}
	}


	void polygonMerge(gpc_polygon &p1, gpc_polygon &p2) //merge p1,p2 result is in p1
	{
		gpc_polygon temp;
		gpc_polygon_clip(GPC_UNION, &p2, &p1, &temp);
		swap(p1, temp);
	}

	void polygonIntersection(gpc_polygon &p1, gpc_polygon &p2) //merge p1,p2 result is in p1
	{
		gpc_polygon temp;
		gpc_polygon_clip(GPC_INT, &p2, &p1, &temp);
		swap(p1, temp);
	}

	void polygonUnion(gpc_polygon &p1, gpc_polygon &p2) //merge p1,p2 result is in p1
	{
		gpc_polygon temp;
		gpc_polygon_clip(GPC_UNION, &p2, &p1, &temp);
		swap(p1, temp);
	}


	bool contains(gpc_polygon &gp1, gpc_polygon &gp2) //if p1 contains p2
	{
		myPolygon p1, p2;
		Vector v;


		for (int k = 0; k<gp1.num_contours; k++)
		{
			p1.clear();
			int i;
			for (i = 0; i<gp1.contour[k].num_vertices; i++)p1.P.push_back(Vector(gp1.contour[k].vertex[i].x, gp1.contour[k].vertex[i].y));
			for (i = 0; i<gp2.contour[0].num_vertices; i++)
			{
				v = (Vector(gp2.contour[0].vertex[i].x, gp2.contour[0].vertex[i].y));
				//	putchar('(');
				bool b = p1.PointInPoly(v);
				//	putchar(')');
				if (!b)break;
			}
			if (i == gp2.contour[0].num_vertices)return true;
		}

		return false;

	}


	void drawGpcPolygon(gpc_polygon &g)
	{
		for (int i = 0; i<g.num_contours; i++)
		{
			glBegin(GL_LINE_LOOP);
			for (int j = 0; j<g.contour[i].num_vertices; j++)
			{
				glVertex3d(g.contour[i].vertex[j].x, g.contour[i].vertex[j].y, 0);
				glVertex3d(g.contour[i].vertex[(j + 1) % g.contour[i].num_vertices].x, g.contour[i].vertex[(j + 1) % g.contour[i].num_vertices].y, 0);
			}
			glEnd();
		}
	}

	void draw3DPolygon(vector< vector<Vector3> >&g)
	{
		for (int i = 0; i<(int)g.size(); i++)
		{
			glBegin(GL_LINE_LOOP);
			for (int j = 0; j<(int)g[i].size(); j++)
			{
				glVertex3d(CO(g[i][j]));
				glVertex3d(CO(g[i][(j + 1) % g[i].size()]));
			}
			glEnd();
		}
	}

	void draw3DPolygon(vector<Vector3>&g)
	{
		glBegin(GL_LINE_LOOP);
		for (int j = 0; j<(int)g.size(); j++)
		{
			glVertex3d(CO(g[j]));
			glVertex3d(CO(g[(j + 1) % g.size()]));
		}
		glEnd();
	}






};





class VCM {
public:
	VCM();
	//char *f = "H:\\Study\\Others\\Visibility\ Query\\3D\\3D\\db\\sample3";
	char f[200];
	GLUI_TextBox *textbox;
	GLUI_EditText *edittext;
	GLUI_Spinner *fontspinner;

	string text = "";
	int fontsize = 10;
	bool textset = false;
	static const char * lFilterPatterns[2];
	Vector3 translate;

	/* for list box to select axis */
	static const char *string_list[6];
	int curr_string = 1;
	int prev_string = 1;

	int fonttodist[41];
	bool argContainsText = false;


	int NDISTESEG;

	int totDir = 6;
	int totp = 4; //(totp+1)^3 points
	//
	FILE *logfp, *efile;

	Vector3 Origin;

	static vector<Box>obstacles;
	vector<double>segDist;
	Box target;
	bool isTargetSet = false;
	bool vcmGenerated = false;
	vector<int>dirObstacleId[6]; // important obstacles in the specific direction , only reserves the index
	// total 6 ta direction
	vector< Box >dirObstacle[6];

	Box dirBox[6];
	Vector3 unitDir[6];
	Box projRect[6][MAXNDISTESEG]; //projection plane
	int totSeg[6];
	gpc_polygon gProjPolys[6][MAXNDISTESEG]; //projected polygon
	vector< vector<Vector3> > ProjPolys[6][MAXNDISTESEG]; //3D planer polygons
	vector<Vector3> targetPoints;
	gpc_polygon gVcmProjPolys[6][MAXNDISTESEG][TOTALPOINTONTARGET];
	vector< vector<Vector3> > vcmProjPolys[6][MAXNDISTESEG][TOTALPOINTONTARGET]; //3D planer polygons
	RTree *rtree;

	double curProjPlane1[8];
	static map< int, double > dstFromTar;
	vector<Vector3>lo, up;


	gpc_polygon projWrtTargetCorner[8];
	Vector3 targetCorner[8];
	double curProjPlane[6];
	vector<Box>All;

	int totDiscard = 0, totCnt = 0;
	double totTime = 0;
	int nObstacle;
	bool showDistRec = 1, showProjPoly = 1, showObstacle = 1, showAll = 1, drawg = 1, showobstacles = 1;
	int showid = 0, curSegId = 0;

	int vcmArray[6][MAXNDISTESEG][MAXSEG][MAXSEG]; //this array will contain the final result\
												if a grid is visible from view point or not
	int naiveVcmArray[6][MAXNDISTESEG][MAXSEG][MAXSEG];
	MonotonDecomposer M;
	RTreeCmdIntrpr *r;

	vector<Vector3> rColor;

	float _angle = -30.0f;
	float _scale;
	//The four strings that are drawn
	static const char* STRS[4];
	static char *console_strings[7];
	int console_index = BLANK;

	polygonAdapter Adapter;


	bool calcProjectionWrtPoint(Plane plane, Vector3 c, Box tb, Box &pr, int dir);
	bool calcCovexProjectionWrtPoint(Plane plane, Vector3 c, Box tb, gpc_polygon &g, int dir);
	bool calcProjectionWrtCube(Plane plane, Box cb, Box pb, Box &pr);
	bool calcConvexProjectionWrtCube(Plane plane, Box cb, Box tb, gpc_polygon &global_p, int dir, bool Union = 0);

	void calcSegmentDistance(int dir);
	void calcProjectionPlanes();
	void update(int dir, int curSeg);
	void updateVcm(vector<int>&obstacleWrtPoint, int dir, int curSeg, int tid);
	void reductionInNegX(int dir);
	void reductionInNegY(int dir);
	void reductionInNegZ(int dir);



	void extedProjToCurLayer(double  d, int dir = 1);
	bool ncmp(Entry e1, Entry e2);
	int countChild(int uson);
	void reductionInPosXRtree(int dir);
	void reductionInPosX(int dir);
	void reductionInPosY(int dir);
	void reductionInPosZ(int dir);
	void reductionOfObstacle();
	void debug0();
	void input();
	void debug1();
	void drawGridX();
	void drawGridY();
	void drawGridZ();
	void drawGrid();
	void generateRandCol();
	double getDim(int dir, int curSeg);
	pair<int, int> pointToarrayIndex(double px, double py, int dir, int curSeg);
	pair<int, int> pointToarrayIndex2(double px, double py, int dir, int curSeg);
	double getx(Vector v0, Vector v1, double y);
	static bool cmpFormonoPolygonTo2Darray(Vector v1, Vector v2);
	void monoPolygonTo2Darray(int dir, int curSeg, vector<Vector>mpoly);
	void monoPolygonTo2Darray1(int dir, int curSeg, vector<Vector>mpoly);
	void monoPolygonTo2Darray2(int dir, int curSeg, vector<Vector>mpoly);
	void mergeAll(int dir, int curSeg);
	void extedProjToCurLayer1(int tid, int curSeg, int dir = 1);
	void updateVcm1(vector<Box>&obstacleWrtPoint, int dir, int curSeg, int tid);

	static bool fcmp(Box b1, Box b2);
	static bool fcmpPosY(Box b1, Box b2);
	static bool fcmpPosZ(Box b1, Box b2);
	static bool fcmpNegX(Box b1, Box b2);
	static bool fcmpNegY(Box b1, Box b2);
	static bool fcmpNegZ(Box b1, Box b2);

	void generateMapInPosX(int tid, int dir);
	void generateMapInPosX1(int tid, int dir);
	void generateMapInPosY(int tid, int dir);
	void generateMapInPosZ(int tid, int dir);
	void generateMapInNegX(int tid, int dir);
	void generateMapInNegY(int tid, int dir);
	void generateMapInNegZ(int tid, int dir);
	void generateMap(int tid, int dir);
	void reduction(int dir);

	void generateTargetPoints(int dir);
	void buildTree(char *path, char *DATAFILENAME, char *Outpath);
	void createRtreeFromData(char *f);
	bool rayRectangleIntersction(Vector3 &u, Vector3 &v, Plane &p);
	bool rayBoxIntersection(Vector3 &u, Vector3 &v, Box &b);
	int naiveVisibility(Vector3 v);
	void bruteForce();
	void initialize();
	void clearTarget();
	void setTarget(Box &b);
	void run(char *f);
	void run2(int dir);
	void debug3();
	void debug4();
	double dist(Vector3 a, Vector3 b);
	Vector3 arrayIndexToPoint(int row, int col, int dir, int curSeg);
	void debug5();

	bool changeTarget(GLdouble x, GLdouble y, GLdouble z);
	void setup();
	void computeScale();
	void draw_axis();

	// pq stores in a non decreasing order of distance
	static bool comparatorForPq(int &a, int &b)
	{
		return dstFromTar[a]>dstFromTar[b];
	}

	static bool CompareLoX(int b1, int b2)
	{
		return obstacles[b1].x[1]>obstacles[b2].x[1];
	}
	

	static bool CompareHiX(int b1, int b2)
	{
		return obstacles[b1].x[0]<obstacles[b2].x[0];
	}

	static bool CompareLoY(int b1, int b2)
	{
		return obstacles[b1].y[1]>obstacles[b2].y[1];
	}

	static bool CompareHiY(int b1, int b2)
	{
		return obstacles[b1].y[0]<obstacles[b2].y[0];
	}

	static bool CompareLoZ(int b1, int b2)
	{
		return obstacles[b1].z[1]>obstacles[b2].z[1];
	}

	static bool CompareHiZ(int b1, int b2)
	{
		return obstacles[b1].z[0]<obstacles[b2].z[0];
	}

	void GUI_setup(GLUI *glui);

};












#endif