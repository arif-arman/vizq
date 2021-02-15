/* rtree.h
   this file defines the class RTree*/

#ifndef __RTREE
#define __RTREE
#define PI 3.14159265
//------------------------------------------------------------
#include "../func/gendef.h"
#include "../heap/heap.h"
#include <vector>
#include <list>
#include <GL\glut.h>

#include "gpc.h"

//Added by Tanzima
//#include "../global.h"
//...............

//Added by Tanzima
#include <vector>
using namespace std;


//added for TP KNN------------------------------------------------
#define SEQUENCE_SENSITIVE false
  //set it true if you want a new partition point every time the order of the NNs change
#define PAST_PAIR 1000

//Added for kGNN by Tanzima


const double MAXDOUBLE = numeric_limits<double>::infinity();
const int kMAX=100; // changed by Eunus : it was set to 500
const int MAXDATALIMIT=20000;
//const int MAXGROUP = 128;
//const int MINSUBGROUP = MAXGROUP/2;
//------------------------------------------------------------
class LinList;
class SortedLinList;
class Cache;
class RTNode;
class Entry;
//Added by Tanzima
struct Rectangle1;
struct DistfromPOI;
//------------------------------------------------------------


//added by fa
struct VisibleRegionOverT //
{
     Pointlocation p1_overT;
     Pointlocation p2_overT;

	 double partial_visibility;
	// int operator==(VisibleRegionOverT b);
};

class Rectangle2
{
	public:
      Pointlocation upper_left;
	  Pointlocation lower_right;
	  bool operator==(const Rectangle2& a) const;
	  bool operator<(Rectangle2 a);
	  bool operator<=(Rectangle2 a);
	  bool operator>(Rectangle2 a);
	  bool equals_to(Rectangle2 a)
	  {
		  return (upper_left.x==a.upper_left.x) && (upper_left.y==a.upper_left.y) && (lower_right.x==a.lower_right.x) && (lower_right.y==a.lower_right.y);
	  }
};

class Rectangle3D
{
public:
	Point3D p1;
	Point3D p2;
	Point3D p3;
	Point3D p4;
	Rectangle3D(Point3D a, Point3D b,Point3D c,Point3D d){p1=a;p2=b;p3=c;p4=d;}
	Rectangle3D()
	{
	}
};

struct line
{
	Point3D a;
	Point3D b;
	line(){}
	line(Point3D _a,Point3D _b){a=_a;b=_b;}

	bool operator==(const line& x) const;
};
struct polygon
{
	vector <line> sides;
	float Polyarea;
	double r = rand() % 255;
	double g = rand() % 255;
	double b = rand() % 255;
	polygon(){};
	polygon(vector<line> lines)
	{
		sides = lines;
	}
	bool operator==(const polygon& poly) const;
	void draw(int planeId, double c) {
		//double c = 0.2;
		double x1 = 0, y1 = 0, z1 = 0;
		if (planeId == 0) z1 = -c;
		else if (planeId == 1) z1 = c;
		else if (planeId == 2) x1 = c;
		else if (planeId == 3) x1 = -c;
		else if (planeId == 4) y1 = c;
		else if (planeId == 5) y1 = -c;
		
		glBegin(GL_POLYGON);
		for (int i = 0; i < sides.size(); ++i) {
			glColor3f(0.5, 0.5, 0.5);
			//glColor3ub(r, g, b);
			//printf("%f %f %f \n", sides[i].a.x + x1, sides[i].a.y + y1, sides[i].a.z + z1);
			glVertex3f(sides[i].a.x+x1, sides[i].a.y+y1, sides[i].a.z+z1);
		}
		glEnd();
	
	}
};

struct VisibleRegion3D
{
	Rectangle3D boundary;
	float partial_visibility;
	int planeId;
	list<polygon> invisible_parts;
	VisibleRegion3D()
	{
	}
	// debug zitu
	void draw(double c) {
		//double c = 0.1;
		double x1 = 0, y1 = 0, z1 = 0;
		if (planeId == 0) z1 = -c;
		else if (planeId == 1) z1 = c;
		else if (planeId == 2) x1 = c;
		else if (planeId == 3) x1 = -c;
		else if (planeId == 4) y1 = c;
		else if (planeId == 5) y1 = -c;

		glBegin(GL_QUADS);
		glVertex3f(boundary.p1.x+x1, boundary.p1.y+y1, boundary.p1.z+z1);
		glVertex3f(boundary.p2.x+x1, boundary.p2.y+y1, boundary.p2.z+z1);
		glVertex3f(boundary.p3.x+x1, boundary.p3.y+y1, boundary.p3.z+z1);
		glVertex3f(boundary.p4.x+x1, boundary.p4.y+y1, boundary.p4.z+z1);
		glEnd();

		
	}
};

struct inVR
{
	Point3D obstacle_corner;
	VisibleRegion3D vRegion;
	Point3D intersection_p;
};

/*int VisibleRegionOverT::operator==(VisibleRegionOverT b)
{
	return (p1_overT.x==b.p1_overT.x) && (p1_overT.y==b.p1_overT.y) && (p2_overT.x==b.p2_overT.x) && (p2_overT.y==b.p2_overT.y); 
}*/

class Box2
{
public:
	Point3D a;
	Point3D b;
	Box2(Point3D _a,Point3D _b){a=_a;b=_b;}
	Box2(){}
	
	bool equals_to(Box2 T)
	{
		if(T.a.x == a.x && T.a.y == a.y && T.a.z == a.z && T.b.x == b.x && T.b.y == b.y && T.b.z ==b.z)
			return true;
		return false;
	}
	void draw() {
		Point3D c((a.x + b.x)/2, (a.y + b.y)/2, (a.z + b.z)/2);

		double dx = fabs(a.x - b.x);
		double dy = fabs(a.y - b.y);
		double dz = fabs(a.z - b.z);

		Point3D dv(dx, dy, dz);

		glPushMatrix();
		{
			glTranslated(c.x, c.y, c.z);
			glScaled(dv.x, dv.y, dv.z);
			//glColor3ub(color.x, color.y, color.z);
			glutSolidCube(1);
		}glPopMatrix();
	}
	
};


class BoxPlanes
{
public:
	int a[4];//corner point number
	bool visible;
	// face of object defined by 4 corner points
	BoxPlanes(int _a,int _b,int _c,int _d)
	{
		a[0]=_a;a[1]=_b;a[2]=_c;a[3]=_d;
	}
	BoxPlanes()
	{
		a[0]=a[1]=a[2]=a[3]=0;
	}
	// point p is a corner point of this box plane
	bool isCornerInBoxPlanes(int p)
	{
		if(p == a[0] || p == a[1] || p == a[2] || p == a[3]) return true;
		else return false;
	}
};

class QueryPoint3D
{
public:
	Point3D position;
	float total_visibility;
	Heap *obstacleList; //obstacles that affects visiblilty, according to their mindist
	vector <VisibleRegion3D> visiblePlanes;
	vector <double> factor;	// area reduce factor of corresponding plane
	float init_visibility(Box2 Target);//considering no obstacle
	bool IsInsideVisibleRegion(Box2 T,Box2 obj); 
	void update_visibilityRegion(Box2 obstacle, Box2 target);
	double r = rand() % 255, g = rand() % 255, b = rand() % 255;
	//float update_naiveVisibility(Box obj,Box T);
	void draw(bool drawVR) {
		glColor3ub(r,g,b);
		// draw visible regions
		if (drawVR) {
			
			for (int j = 0; j < visiblePlanes.size(); ++j) {
				glPushMatrix();
				glColor3f(1, 1, 1);
				visiblePlanes[j].draw(0.02);
				glPopMatrix();
				list<polygon> invisible = visiblePlanes[j].invisible_parts;
				glPushMatrix();
				glColor3f(0, 0, 0);
				for (list<polygon>::iterator itr = invisible.begin(); itr != invisible.end(); itr++) {
					itr->draw(visiblePlanes[j].planeId, 0.03);
					
				}
				glColor3f(1, 1, 1);
				glPopMatrix();
				
			}
			
		}
		// draw qp marker
		glPushMatrix();
		//glColor3f(1, 1, 1);
		//glScalef(1, 1, 2);
		glTranslatef(position.x, position.y, position.z);
		glBegin(GL_QUADS);
		glVertex3f(2, 2, 5);
		glVertex3f(2, -2, 5);
		glVertex3f(-2, -2, 5);
		glVertex3f(-2, 2, 5);
		glEnd();
		glBegin(GL_TRIANGLES);
		glVertex3f(2, -2, 5);
		glVertex3f(-2, -2, 5);
		glVertex3f(0, 0, 0);
		glEnd();
		glBegin(GL_TRIANGLES);
		glVertex3f(-2, -2, 5);
		glVertex3f(-2, 2, 5);
		glVertex3f(0, 0, 0);
		glEnd();
		glBegin(GL_TRIANGLES);
		glVertex3f(-2, 2, 5);
		glVertex3f(2, 2, 5);
		glVertex3f(0, 0, 0);
		glEnd();
		glBegin(GL_TRIANGLES);
		glVertex3f(2, 2, 5);
		glVertex3f(2, -2, 5);
		glVertex3f(0, 0, 0);
		glEnd();

		glPopMatrix();
	}
};

class QueryPoint
{
    public:  
		
	  Pointlocation position;
	  
	  double total_visibility; //for this query_point only
	  
	  Heap *obstacleList; //obstacles that affects visiblilty, according to their mindist
      vector <VisibleRegionOverT>  VisibleRegion;//segments over Target object T, that are visible from this query point
     // list <Rectangle2> ObstacleList; 
      bool IsInVisibleRegion(Rectangle2 rect); 

	  bool  del_from_ObstacleList(Rectangle2 rect);

	  void update_visibliliyRegion(Rectangle2 obstacle, Rectangle2 target);//IsInVisibleRegion true hole oi obstacle er jonno update korte hobe
	  double visibiliy(Rectangle2 Target);
      
	  double init_visibility(Rectangle2 Target);//jokhon kono obstacle nai consider kore
      
};


//-------------------------------------------------------------
class RTree : public Cacheable
{
public:
	//debug 
	int io_access;
//--===on disk===--
	int dimension;                       
	int num_of_data;	                 
    int num_of_dnodes;	                 
    int num_of_inodes;	                 
	int root;                            
	bool root_is_data;                   
//--===others===--
	RTNode *root_ptr;
    bool *re_level;  
    LinList *re_data_cands; 
	LinList *deletelist;

//--===added for TP KNN===--
	int last_pair[PAST_PAIR][2]; //records the pairs of points that produce the minimum inflence time
	int lastcnt; //next last pair to be replaced
	Heap *tpheap;

//--===functions===--
	RTree(char *fname, int _b_length, Cache* c, int _dimension);
    RTree(char *fname, Cache* c);
    RTree(char *inpname, char *fname, int _blength, Cache* c, int _dimension);

    ~RTree();
	void del_root();
	bool delete_entry(Entry *d);
	bool FindLeaf(Entry *e);
    int get_num() { return num_of_data; }
	void insert(Entry *d);
	void load_root();  
	void rangeQuery(float *mbr, SortedLinList *res);
	void read_header(char *buffer);      
	void write_header(char *buffer);
	int update_rslt(Entry *_e, float _dist, Entry *_rslt, 
					 float *_key, int _k);

	// This function was added to perform TP-kNN queries by Bobby
	void TPNN_TP(float *_qline, int _k, Entry *_nn, Entry *_rslt, float _max_trvl);
	
	//--added for valdity region queries---
	void rect_win_query(float *mbr, LinList *in_objs, LinList *out_objs_so_far);
	void rect_win_query(float *mbr, float *exclmbr, LinList *c_inf_objs);
	void BFNN(float *_qpt, int _k, Entry *_rslt);
	void BFNNCont(float *qmbr, float *qmbr2, Heap *heap, HeapEntry *e, int k);
	//Added by Tanzima
	float KMin(Point2D m, int c, float cl, int k, Pointlocation _rslt[], int *num_of_data,float d_safe_corner,DistfromPOI _cornertoPOI[]);
	void UpdateCount(Rectangle1 R, float cl, int k, Pointlocation _rslt[], int *num_of_data, float r, int *count, DistfromPOI _cornertoPOI[]);
	int UpdateStatus(Rectangle1 R, float cl, int k, Pointlocation _rslt[], int *num_of_data, int *count, DistfromPOI _cornertoPOI[]);
	void Rect_kNNQ(Rectangle1 R, float cl, int k, Pointlocation *_rslt, int *num_of_data);
	void Point_BFN_NNQ(Point2D o, double *_rslt);

	//Added by Tanzima for kGNN
	float RTree::KMax(int k, Pointlocation _rslt[], int *num_of_data);
	void private_kGNN_sum(Rectangle1 R[], int g_size, int k, Pointlocation _rslt[], int *num_of_data);
	void private_kGNN_max(Rectangle1 R[], int g_size, int k, Pointlocation rslt[], int *num_of_data);

	//Added by Eunus for Consensus
	void RTree::consensus_kGNN(Rectangle1 R[], int g_size, int sg_size, int k, int f, SG L[][32], Pointlocation rslt[], int *num_of_data);


	//fa
	void RTree::MOV(QueryPoint q[], int num_of_query_points, Rectangle2 T,int k,QueryPoint k_answers[], int& page,int& obstacle_considered);
	void RTree::Alternate_approach(QueryPoint q[], int num_of_query_points, Rectangle2 T,int k, QueryPoint k_answers[], int& page,int& obstacle_considered);
	

	void RTree::MOV3D(QueryPoint3D q[], int num_of_query_points, Box2 T,int k, QueryPoint3D k_answers[],int& page, int& obstacle_considered);
	void RTree::Alternate_approach3D(QueryPoint3D q[], int num_of_query_points, Box2 T,int k, QueryPoint3D k_answers[], int& page,int& obstacle_considered);
	void RTree::MOV_obs(QueryPoint q[], int num_of_query_points, Rectangle2 T,int k, QueryPoint k_answers[],int& page, int& obstacle_considered);
	// edit by zitu
	void RTree::MOV_obs3D(vector<QueryPoint3D> &qu, int num_of_query_points, Box2 T, int k, vector<QueryPoint3D> &k_answers, int& page, int& obstacle_considered);
	void RTree::naiveMOV3D(QueryPoint3D q[], int num_of_query_points, Box2 T,int k, QueryPoint3D k_answers[],int& page, int& obstacle_considered);
	float RTree::VCM_visibility(QueryPoint q,Rectangle2 T);
	float RTree::VCM_visibility3D(QueryPoint3D q,Box2 T);

	void rtreeVcmAlgo1(char *path);

};
bool ifIntersect3D_lineSegment(Point3D a, Point3D b, Point3D c, Point3D d, Point3D &intersect_Point);
#endif // __RTREE
