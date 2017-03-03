#pragma warning  ( disable : 4786)
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



#include "camera.h"
#include "macros.h"
#include "gpc.h"
#include "monotone.h"




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




using namespace std;

//
#define MAXSEG 200 // MAXSEG*MAXSEG grid
#define MAXNDISTESEG 200
#define INTSEGDIST 50
#define TOTALPOINTONTARGET 550
double ANGRES = 1;


int NDISTESEG;

int totDir = 2;
int totp=3; //(totp+1)^3 points
//

FILE *logfp,*efile;



Vector3 Origin;



//grahpics stuff
/*
Camera cam;
int window;


void animate(){
	glutPostRedisplay();
	//codes for any changes in Models, Camera
}

void draw_axis(){
        glColor3f(1, 1, 1);
        glBegin(GL_LINES);{
                glVertex3f(0, -150, 0);
                glVertex3f(0,  150, 0);
                glVertex3f(-150, 0, 0);
                glVertex3f( 150, 0, 0);
        }glEnd();
        glColor3f(0.5, 0.5, 0.5);
        glBegin(GL_LINES);{
            for(int i = -150; i<=150; i+=10){
                glVertex3f(i, -150, 0);
                glVertex3f(i,  150, 0);
            }
            for(int i = -150; i<=150; i+=10){
                glVertex3f(-150, i, 0);
                glVertex3f(150,  i, 0);
            }
        }glEnd();

}

void init(void)
{
	//for surface mesh
	/*
	glMap2f(GL_MAP2_VERTEX_3, 0, 1, 3, 4,
	0, 1, 12, 4, &ctrlpoints[0][0][0]);
	glEnable(GL_MAP2_VERTEX_3);
	glEnable(GL_AUTO_NORMAL);
	glMapGrid2f(20, 0.0, 1.0, 20, 0.0, 1.0);
	initlights();

	*/
/*
	glMatrixMode(GL_PROJECTION);

	//initialize the matrix
	glLoadIdentity();

	//give PERSPECTIVE parameters
	gluPerspective(70,	1,	0.1,	10000.0);
	// gluNurbsCallback(theNurb, GLU_ERROR, nurbsError);
}

*/



MonotonDecomposer M;



int bb=0;
class polygonAdapter //convert 3d to 2d
{
public:

	gpc_polygon poly;
	int dir;
	polygonAdapter(){}

	polygonAdapter( int dir )
	{
		this->dir=dir;
	}

	Vector3 adapt23(int dir,double dist,double x,double y)
	{
		Vector3 ret;
		if( dir==1 || dir==0 )
		{
			ret=Vector3( dist,x,y );
		}

		if( dir==2 || dir==3 )
		{
			ret=Vector3( x,dist,y );
//			ret=Vector3( dist,x,y );
		}


		if( dir==4 || dir==5 )
		{
			ret=Vector3(x,y,dist);
//			ret=Vector3( dist,x,y );
		}

		return ret;
	}

	Vector adapt32(int dir,double x,double y,double z)
	{
		if(dir==1 || dir==0)return Vector(y,z);
		if( dir==2 || dir==3 )return Vector( x,z );
		if( dir==4 || dir==5 )return Vector( x,y );
	}

	vector<Vector3> adapt23(int dir,double dist,vector<Vector>&v)
	{
		vector<Vector3>ret;
		for(int i=0;i<v.size();i++)ret.push_back( adapt23(dir,dist,v[i].x,v[i].y) );
		return ret;
	}

	void adapt(int dir,double dist,gpc_polygon& g,vector< vector< Vector3 > >&planerPoly) //convert 2D to 3D
	{
		for( int i=0;i<g.num_contours;i++ )
		{
			planerPoly.resize( planerPoly.size()+1 );
			for(int j=0;j<g.contour[i].num_vertices;j++)
			{
				planerPoly.back().push_back( adapt23( dir,dist,g.contour[i].vertex[j].x,g.contour[i].vertex[j].y ) );
			}
		}
	}

	void adapt( Box &b,gpc_polygon &g) //convert 3D box to 2D rectangle
	{

		g.num_contours=1;
		g.hole=0;
		g.contour= ( gpc_vertex_list *) malloc( g.num_contours*sizeof( gpc_vertex_list ) ) ;
		g.contour[0].num_vertices=4;
		g.contour[0].vertex=( gpc_vertex *)malloc( g.contour[0].num_vertices*sizeof( gpc_vertex ) );

		Vector3 v0=Vector3(b.x[0],b.y[0],b.z[0]);
		Vector3 v1=Vector3(b.x[1],b.y[1],b.z[1]);
		Vector3 dif=v1.sub(v0);
		double x[2],y[2];

		if( Z(dif.x) ) //Box is in the x perp plane
		{
			x[0]=v0.y;
			x[1]=v1.y;

			y[0]=v0.z;
			y[1]=v1.z;
		}

		else if( Z(dif.y) ) //Box is in the y perp plane
		{
			x[0]=v0.z;
			x[1]=v1.z;

			y[0]=v0.x;
			y[1]=v1.x;
		}


		else if( Z(dif.z) ) //Box is in the z perp plane
		{
			x[0]=v0.x;
			x[1]=v1.x;

			y[0]=v0.y;
			y[1]=v1.y;
		}

		g.contour[0].vertex[0].x=x[0];
		g.contour[0].vertex[0].y=y[0];

		g.contour[0].vertex[1].x=x[0];
		g.contour[0].vertex[1].y=y[1];

		g.contour[0].vertex[2].x=x[1];
		g.contour[0].vertex[2].y=y[1];

		g.contour[0].vertex[3].x=x[1];
		g.contour[0].vertex[3].y=y[0];

	}

	void adapt( vector<Vector>&b,gpc_polygon &g )
	{
		g.num_contours=(b.size()!=0);
		g.hole=0;
		g.contour= ( gpc_vertex_list *) malloc( g.num_contours*sizeof( gpc_vertex_list ) ) ;
		g.contour[0].num_vertices=b.size();

		g.contour[0].vertex=( gpc_vertex *)malloc( g.contour[0].num_vertices*sizeof( gpc_vertex ) );

		for(int i=0;i<b.size();i++)
		{
			g.contour[0].vertex[i].x=b[i].x;
			g.contour[0].vertex[i].y=b[i].y;
		}

	}

	void adapt( gpc_polygon &g,vector< vector<Vector> >&vp )
	{
		for( int i=0;i<g.num_contours;i++ )
		{
			vp.resize( vp.size()+1 );
			for(int j=0;j<g.contour[i].num_vertices;j++)
			{
				vp.back().push_back( Vector( g.contour[i].vertex[j].x,g.contour[i].vertex[j].y ) );
			}
		}
	}


	void polygonMerge( gpc_polygon &p1,gpc_polygon &p2  ) //merge p1,p2 result is in p1
	{
		gpc_polygon temp;
		gpc_polygon_clip( GPC_UNION,&p2,&p1,&temp );
		gpc_free_polygon(&p1);
		p1=temp;
	}
	
	void polygonIntersection( gpc_polygon &p1,gpc_polygon &p2  ) //merge p1,p2 result is in p1
	{
		
		gpc_polygon temp;
		gpc_polygon_clip( GPC_INT,&p2,&p1,&temp );		
		gpc_free_polygon(&p1);
		p1=temp;
	}

	void polygonUnion( gpc_polygon &p1,gpc_polygon &p2  ) //merge p1,p2 result is in p1
	{
		gpc_polygon temp;
		gpc_polygon_clip( GPC_UNION,&p2,&p1,&temp );
		gpc_free_polygon(&p1);
		p1=temp;
	}


	bool contains( gpc_polygon &gp1,gpc_polygon &gp2  ) //if p1 contains p2
	{
		myPolygon p1,p2;
		Vector v;

		if(!gp2.num_contours)return true;

		for( int k=0;k<gp1.num_contours;k++ )
		{
			p1.clear();
			int i;
			for(  i=0;i<gp1.contour[k].num_vertices;i++ )p1.P.push_back( Vector( gp1.contour[k].vertex[i].x,gp1.contour[k].vertex[i].y ) );
			for(  i=0;i<gp2.contour[0].num_vertices;i++ )
			{
				v=( Vector( gp2.contour[0].vertex[i].x,gp2.contour[0].vertex[i].y ) );
			//	putchar('(');
				bool b=p1.PointInPoly(v);
			//	putchar(')');
				if(!b)break;
			}
			if(i==gp2.contour[0].num_vertices)return true;
		}

		return false;

	}


	void drawGpcPolygon( gpc_polygon &g )
	{
		for(int i=0;i<g.num_contours;i++)
		{
			glBegin(GL_LINE_LOOP);
			for(int  j=0;j<g.contour[i].num_vertices;j++ )
			{
				glVertex3d( g.contour[i].vertex[j].x,g.contour[i].vertex[j].y,0 );
				glVertex3d( g.contour[i].vertex[(j+1)%g.contour[i].num_vertices].x,g.contour[i].vertex[(j+1)%g.contour[i].num_vertices].y,0 );
			}
			glEnd();
		}
	}

	void draw3DPolygon( vector< vector<Vector3> >&g )
	{
		for(int i=0;i<(int)g.size();i++)
		{
			glBegin(GL_LINE_LOOP);
			for(int  j=0;j<(int)g[i].size();j++ )
			{
				glVertex3d( CO(g[i][j]) );
				glVertex3d( CO(g[i][(j+1)%g[i].size()]) );
			}
			glEnd();
		}
	}

	void draw3DPolygon( vector<Vector3>&g )
	{
		glBegin(GL_LINE_LOOP);
		for(int  j=0;j<(int)g.size();j++ )
		{
			glVertex3d( CO(g[j]) );
			glVertex3d( CO(g[(j+1)%g.size()]) );
		}
		glEnd();
	}






}Adapter;




// calculate projection of a Box

bool calcProjectionWrtPoint(Plane &plane,Vector3 &c,Box &tb,Box &pr,int &dir)
{
	/*
		calculate projection of tb on plane wrt c store in pr
	*/



	vector<Vector>projPoints;

	int i1,j1,k1;




	pr.x[0]=pr.y[0]=pr.z[0]=+INF;
	pr.x[1]=pr.y[1]=pr.z[1]=-INF;

	//bool ret=false;
	for( i1=0;i1<2;i1++ )
		for( j1=0;j1<2;j1++ )
			for( k1=0;k1<2;k1++ )
			{
				Vector3 pp;

				Vector3 p( tb.x[i1],tb.y[j1],tb.z[k1] );

				if( !plane.caclProjectionOfPoint( c,p,pp ) )return false;

				//ret=true;



				pr.x[0]=min( pr.x[0],pp.x );
				pr.y[0]=min( pr.y[0],pp.y );
				pr.z[0]=min( pr.z[0],pp.z );


				pr.x[1]=max( pr.x[1],pp.x );
				pr.y[1]=max( pr.y[1],pp.y );
				pr.z[1]=max( pr.z[1],pp.z );





			}










	return true;


}
bool calcCovexProjectionWrtPoint(Plane &plane,Vector3 &c,Box &tb,gpc_polygon &g,int &dir)
{
	/*
		calculate projection of tb on plane wrt c store in pr
	*/



	vector<Vector>projPoints;

	int i1,j1,k1;

	

	/*

	pr.x[0]=pr.y[0]=pr.z[0]=+INF;
	pr.x[1]=pr.y[1]=pr.z[1]=-INF;
	*/
	//bool ret=false;
	for( i1=0;i1<2;i1++ )
		for( j1=0;j1<2;j1++ )
			for( k1=0;k1<2;k1++ )
			{
				Vector3 pp;

				Vector3 p( tb.x[i1],tb.y[j1],tb.z[k1] );

				if( !plane.caclProjectionOfPoint( c,p,pp ) )return false;
				

				//ret=true;


				/*
				pr.x[0]=min( pr.x[0],pp.x );
				pr.y[0]=min( pr.y[0],pp.y );
				pr.z[0]=min( pr.z[0],pp.z );


				pr.x[1]=max( pr.x[1],pp.x );
				pr.y[1]=max( pr.y[1],pp.y );
				pr.z[1]=max( pr.z[1],pp.z );

				*/

				projPoints.push_back( Adapter.adapt32( dir,pp.x,pp.y,pp.z ) );


			}




	vector<Vector> pr=convexHull( projPoints );

	Adapter.adapt(pr,g);







	return true;


}
bool calcProjectionWrtCube( Plane &plane,Box &cb,Box &pb,Box &pr ) //project a plane wrt a box , projected plane is supposed to be axix alingned
{

	/*
		calculate projection of pb on plane wrt cb store in pr
	*/

	//cb.print();
	//cp.print();

	int i,j,k;
	int i1,j1,k1;




	pr.x[0]=pr.y[0]=pr.z[0]=+INF;
	pr.x[1]=pr.y[1]=pr.z[1]=-INF;

	bool ret=false;

	for( i=0;i<2;i++ )
		for( j=0;j<2;j++ )
			for( k=0;k<2;k++ )
				for( i1=0;i1<2;i1++ )
					for( j1=0;j1<2;j1++ )
						for( k1=0;k1<2;k1++ )
						{
							Vector3 pp;
							Vector3 c( cb.x[i],cb.y[j],cb.z[k] );
							Vector3 p( pb.x[i1],pb.y[j1],pb.z[k1] );

							if( !plane.caclProjectionOfPoint( c,p,pp ) )return false;

							ret=true;

							pr.x[0]=min( pr.x[0],pp.x );
							pr.y[0]=min( pr.y[0],pp.y );
							pr.z[0]=min( pr.z[0],pp.z );


							pr.x[1]=max( pr.x[1],pp.x );
							pr.y[1]=max( pr.y[1],pp.y );
							pr.z[1]=max( pr.z[1],pp.z );

						}


	return ret;

}
bool calcConvexProjectionWrtCube( Plane &plane,Box &cb,Box &tb,gpc_polygon &global_p,int &dir,bool Union=0 ) //project a plane wrt a box , projected plane is supposed to be axix alingned
{

	/*
		calculate projection of tb on plane wrt cb store in pr
	*/

	//cb.print();
	//cp.print();

	int i,j,k;
	int i1,j1,k1;





	bool ret=false;

	global_p.contour=0;
	bool start=0;
	for( i=0;i<2;i++ )
		for( j=0;j<2;j++ )
			for( k=0;k<2;k++ )
			{
				Vector3 pp;
				Vector3 c( cb.x[i],cb.y[j],cb.z[k] );
//				Vector3 p( tb.x[i1],tb.y[j1],tb.z[k1] );


				gpc_polygon local_p;

				if( !calcCovexProjectionWrtPoint(plane,c,tb,local_p,dir ) )return false;



//				calcCovexProjectionWrtPoint(plane,c,tb,local_p );
//				Adapter.adapt( v,local_p );

				if(!start)
				{
					global_p=local_p;
					start=1;
					continue;
				}


				if(!Union)Adapter.polygonIntersection( global_p,local_p );
				else Adapter.polygonUnion( global_p,local_p );

			}


	return true;
}


#include<queue>

vector<Box>obstacles;
Box target;


class CompareLoX
{
	public:
    bool operator() (int b1, int b2)
    {
        return obstacles[b1].x[1]>obstacles[b2].x[1];
    }
};

class CompareHiX
{
	public:
    bool operator() (int b1, int b2)
    {
        return obstacles[b1].x[0]<obstacles[b2].x[0];
    }
};

class CompareLoY
{
	public:
    bool operator() (int b1, int b2)
    {
        return obstacles[b1].y[1]>obstacles[b2].y[1];
    }
};

class CompareHiY
{
	public:
    bool operator() (int b1, int b2)
    {
        return obstacles[b1].y[0]<obstacles[b2].y[0];
    }
};
class CompareLoZ
{
	public:
    bool operator() (int b1, int b2)
    {
        return obstacles[b1].z[1]>obstacles[b2].z[1];
    }
};

class CompareHiZ
{
	public:
    bool operator() (int b1, int b2)
    {
        return obstacles[b1].z[0]<obstacles[b2].z[0];
    }
};

vector<int>dirObstacleId[6]; // important obstacles in the specific direction , only reserves the index
	// total 6 ta direction


vector< Box >dirObstacle[6];

/*

// london
#define NDISTESEG 50
#define INTSEGDIST 40

*/



vector<double>segDist;

void calcSegmentDistance()
{

	segDist.clear();
	segDist.push_back(target.lx/2);

	/*
	segDist=vector<double>(NDISTESEG); // this will be calculated based on properties of eye
	segDist[0]=target.lx/2+1;
	for( int i=1;i<NDISTESEG;i++ )segDist[i]=segDist[i-1]+INTSEGDIST;	// for debug purpose only
	*/

	double da=ANGRES;
	int n=90.0/da;
	double a=90;
	for( int i=1;i<n;i++ )
	{
		a-=da;
		segDist.resize(segDist.size()+1);
		segDist[i]=target.lx/2;
		segDist[i]+=( target.lx/2.0 )/( tan( a*acos(-1.0)/180.0 ) );
		//cout<<segDist[i]<<endl;
		segDist[i]*=2;
	}
	NDISTESEG=n;



	cout<<n<<endl;
//	while(1);




}

Box dirBox[6];
Vector3 unitDir[6];
Box projRect[6][MAXNDISTESEG]; //projection plane
int totSeg[6];
gpc_polygon gProjPolys[6][MAXNDISTESEG]; //projected polygon
vector< vector<Vector3> > ProjPolys[6][MAXNDISTESEG]; //3D planer polygons


RTree *rtree;

//ekta problem ase...
// coordinate system k target cube er center e nia jabo

void calcProjectionPlanes()
{
	Box btemp[]={ target.bx[0],target.bx[1]/*,target.by[0],target.by[1],target.bz[0],target.bz[1]*/ };
	for(int i=0;i<totDir;i++)dirBox[i]=btemp[i];
	Vector3 tempv[]={ Vector3(-1,0,0),Vector3(+1,0,0),Vector3(0,-1,0),Vector3(0,+1,0),Vector3(0,0,-1),Vector3(0,0,+1) };
	for(int i=0;i<totDir;i++)unitDir[i]=tempv[i];

	double transfactor[]={ target.o.x,target.o.x,target.o.y,target.o.y,target.o.z,target.o.z }; // we have to translate the plane centering at cube center
	Plane projPlane;

	for(int dir=0;dir<totDir;dir++)
	{
		for( int curSeg=0;curSeg<NDISTESEG;curSeg++ )
		{
			projPlane=Plane( unitDir[dir].x,unitDir[dir].y,unitDir[dir].z , segDist[curSeg]  );
			calcProjectionWrtPoint( projPlane,target.o,dirBox[dir],projRect[dir][curSeg],dir );
		}
	}
}





void update( int dir,int curSeg )
{
	vector<int>&id=dirObstacleId[ dir ];


	gProjPolys[dir][curSeg].num_contours=0;

	for( int i=0;i<id.size();i++ )
	{
		gpc_polygon local_p;
		Plane projPlane=Plane( unitDir[dir].x,unitDir[dir].y,unitDir[dir].z,segDist[curSeg] );
		calcConvexProjectionWrtCube(projPlane, target , obstacles[id[i]] ,local_p,dir);
		Adapter.polygonMerge( gProjPolys[dir][curSeg],local_p );
	}


}



vector<Vector3> targetPoints;
gpc_polygon gVcmProjPolys[6][MAXNDISTESEG][TOTALPOINTONTARGET];
vector< vector<Vector3> > vcmProjPolys[6][MAXNDISTESEG][TOTALPOINTONTARGET]; //3D planer polygons
// dir,curseg,point id on target


void updateVcm( vector<int>&obstacleWrtPoint,int dir,int curSeg,int tid )
{

	vector<int>&id=obstacleWrtPoint;
	Vector3 c=targetPoints[tid];
	gpc_polygon &g=gVcmProjPolys[dir][curSeg][tid];


	g.num_contours=0;


	for( int i=0;i<id.size();i++ )
	{
		gpc_polygon local_p;
		Plane projPlane=Plane( unitDir[dir].x,unitDir[dir].y,unitDir[dir].z,segDist[curSeg]+c.x );
		calcCovexProjectionWrtPoint(projPlane, c , obstacles[id[i]] ,local_p,dir);
		Adapter.polygonMerge( g,local_p );
	}


}



double curProjPlane1[8];
/*
void extedProjToCurLayer1( double  d,int dir=1)
{

	double x1=curProjPlane1[dir];

	for(int c=0;c<8;c++)
	{
		gpc_polygon &projPoly=projWrtTargetCorner[c];
		for (int i = 0; i <projPoly.num_contours ; i++)
		{
			for (int j = 0; j < projPoly.contour[i].num_vertices; j++)
			{
				double &py=projPoly.contour[i].vertex[j].x;
				double &pz=projPoly.contour[i].vertex[j].y;


				double y1=targetCorner[c].y-py;
				double z1=targetCorner[c].z-pz;



				double x2=d;

				double y2=(y1/x1)*x2;
				double z2=(z1/x1)*x2;


				py=targetCorner[c].y+y2;
				pz=targetCorner[c].z+z2;



			}
		}
	}


}*/




void reductionInNegX(int dir) //dir=0
{

	puts("reduction in -x direction...");

	priority_queue<int, vector<int>, CompareHiX> pq;

	// jei shob obstacle er target er respect e right  e thake
	bool bdir=dir&1;
	for(int i=0;i<obstacles.size();i++)if( obstacles[i].x[bdir^1]<=target.x[bdir] )pq.push(i); //change here ofr direction



	//cout<<"no of obstacles in negative x direction"<<endl;
	//cout<<pq.size()<<endl;


	//ei direction e total kotogula distance segment lagbe.. eitar ekta max limit ase
	// seita cross korbe na but jodi obstacle sesh hoia jay taileo r agabe na amar code

	int &curSeg=totSeg[dir];
	curSeg=0;


	while(!pq.empty())
	{
	 	int u=pq.top();
		pq.pop();

	//	cout<<u<<" ";

		while( curSeg<NDISTESEG-1 && target.x[dir]-segDist[curSeg]>=obstacles[ u ].x[ dir ] ) //change here
		{
			curSeg++;
			update( dir,curSeg ); // eikhane optimize korbo aro...
			Adapter.adapt( dir,-segDist[curSeg],gProjPolys[dir][curSeg],ProjPolys[dir][curSeg] ); //change here
		}


		Box projPoly;

		Plane projPlane=Plane( -1,0,0, fabs(-segDist[curSeg]+target.o.x) ); //change here for direction
		Box boundRect=projRect[dir][curSeg];
		if(!calcProjectionWrtCube(projPlane, target , obstacles[u] ,projPoly))
		{
			//printf("discarding id %d because of near distance\n",u);
			continue;
		}
		Box projPoly1;
		if( !commonBox( projPoly,boundRect,projPoly1 ) )
		{
			//printf("discarding id %d because of because out of the rojection rectangle\n",u);
			continue;
		}
		gpc_polygon gProjPoly;

		Adapter.adapt( projPoly1,gProjPoly );

		if(Adapter.contains( gProjPolys[dir][curSeg],gProjPoly ))
		{
			//printf("discarding id %d view blocked by other obstacles\n",u);
			continue;
		}

		dirObstacleId[dir].push_back( u );
	}

	curSeg=NDISTESEG-2;
}

void reductionInNegY(int dir) //dir=2
{
	puts("reduction in -y direction...");
	priority_queue<int, vector<int>, CompareHiY> pq;

	// jei shob obstacle er target er respect e right  e thake
	bool bdir=dir&1;
	for(int i=0;i<obstacles.size();i++)if( obstacles[i].y[bdir^1]<=target.y[bdir] )pq.push(i); //change here ofr direction



	//cout<<"no of obstacles in negative x direction"<<endl;
	//cout<<pq.size()<<endl;


	//ei direction e total kotogula distance segment lagbe.. eitar ekta max limit ase
	// seita cross korbe na but jodi obstacle sesh hoia jay taileo r agabe na amar code

	int &curSeg=totSeg[dir];
	curSeg=0;


	while(!pq.empty())
	{
	 	int u=pq.top();
		pq.pop();



		while( curSeg<NDISTESEG-1 && target.y[dir]-segDist[curSeg]>=obstacles[ u ].y[ dir ] ) //change here
		{
			curSeg++;
			update( dir,curSeg ); // eikhane optimize korbo aro...
			Adapter.adapt( dir,-segDist[curSeg],gProjPolys[dir][curSeg],ProjPolys[dir][curSeg] ); //change here
		}


		Box projPoly;

		Plane projPlane=Plane( CO(unitDir[dir]), fabs(-segDist[curSeg]+target.o.y) ); //change here for direction
		Box boundRect=projRect[dir][curSeg];
		if(!calcProjectionWrtCube(projPlane, target , obstacles[u] ,projPoly))
		{
			//printf("discarding id %d because of near distance\n",u);
			continue;
		}
		Box projPoly1;
		if( !commonBox( projPoly,boundRect,projPoly1 ) )
		{
			//printf("discarding id %d because of because out of the rojection rectangle\n",u);
			continue;
		}
		gpc_polygon gProjPoly;

		Adapter.adapt( projPoly1,gProjPoly );

		if(Adapter.contains( gProjPolys[dir][curSeg],gProjPoly ))
		{
			//printf("discarding id %d view blocked by other obstacles\n",u);
			continue;
		}

		dirObstacleId[dir].push_back( u );
	}

	curSeg=NDISTESEG-2;
}



void reductionInNegZ(int dir) //dir=4
{
	puts("reduction in -z direction...");
	priority_queue<int, vector<int>, CompareHiY> pq;


	// jei shob obstacle er target er respect e right  e thake
	bool bdir=dir&1;
	for(int i=0;i<obstacles.size();i++)if( obstacles[i].z[bdir^1]<=target.z[bdir] )pq.push(i); //change here ofr direction



	//cout<<"no of obstacles in negative x direction"<<endl;
	//cout<<pq.size()<<endl;


	//ei direction e total kotogula distance segment lagbe.. eitar ekta max limit ase
	// seita cross korbe na but jodi obstacle sesh hoia jay taileo r agabe na amar code

	int &curSeg=totSeg[dir];
	curSeg=0;


	while(!pq.empty())
	{
	 	int u=pq.top();
		pq.pop();



		while( curSeg<NDISTESEG-1 && target.z[dir]-segDist[curSeg]>=obstacles[ u ].z[ dir ] ) //change here
		{
			curSeg++;
			update( dir,curSeg ); // eikhane optimize korbo aro...
			Adapter.adapt( dir,-segDist[curSeg],gProjPolys[dir][curSeg],ProjPolys[dir][curSeg] ); //change here
		}


		Box projPoly;

		Plane projPlane=Plane( CO(unitDir[dir]), fabs(-segDist[curSeg]+target.o.z) ); //change here for direction
		Box boundRect=projRect[dir][curSeg];
		if(!calcProjectionWrtCube(projPlane, target , obstacles[u] ,projPoly))
		{
			//printf("discarding id %d because of near distance\n",u);
			continue;
		}
		Box projPoly1;
		if( !commonBox( projPoly,boundRect,projPoly1 ) )
		{
			//printf("discarding id %d because of because out of the rojection rectangle\n",u);
			continue;
		}
		gpc_polygon gProjPoly;

		Adapter.adapt( projPoly1,gProjPoly );

		if(Adapter.contains( gProjPolys[dir][curSeg],gProjPoly ))
		{
			//printf("discarding id %d view blocked by other obstacles\n",u);
			continue;
		}

		dirObstacleId[dir].push_back( u );
	}

	curSeg=NDISTESEG-2;
}


/*
void reductionInPosX1(int dir) //dir=1
{
	priority_queue<int, vector<int>, CompareLoX> pq;
	for(int i=0;i<obstacles.size();i++)if( obstacles[i].x[dir]>=target.x[dir] )pq.push(i); //change here ofr direction

	int &curSeg=totSeg[dir];
	curSeg=0;


	while(!pq.empty())
	{
	 	int u=pq.top();
		pq.pop();



		while( curSeg<NDISTESEG-1 && target.x[dir]+segDist[curSeg]<=obstacles[ u ].x[ dir ] )
		{
			curSeg++;
			update( dir,curSeg ); // eikhane optimize korbo aro...
			Adapter.adapt( dir,target.x[dir]+segDist[curSeg],gProjPolys[dir][curSeg],ProjPolys[dir][curSeg] );

		}


		Box projPoly;
		Plane projPlane=Plane( +1,0,0,segDist[curSeg]+target.o.x ); //change here for direction
		Box boundRect=projRect[dir][curSeg];
		if(!calcProjectionWrtCube(projPlane, target , obstacles[u] ,projPoly))
		{
			//printf("discarding id %d because of near distance\n",u);
			continue;
		}
		Box projPoly1;
		if( !commonBox( projPoly,boundRect,projPoly1 ) )
		{
			printf("discarding id %d because of because out of the rojection rectangle\n",u);
			continue;
		}
		gpc_polygon gProjPoly;

		Adapter.adapt( projPoly1,gProjPoly );

		if(Adapter.contains( gProjPolys[dir][curSeg],gProjPoly ))
		{
			printf("discarding id %d view blocked by other obstacles\n",u);
			continue;
		}

		dirObstacleId[dir].push_back( u );
	}
}
*/

map< int,double > dstFromTar;

class comparatorForPq
{
	public:
		bool operator()( int &a,int &b )
		{
			return dstFromTar[a]>dstFromTar[b];
		}
};


gpc_polygon projWrtTargetCorner[ 8 ];
Vector3 targetCorner[8];
double curProjPlane[6];
vector<Box>All;


void extedProjToCurLayer( double  d,int dir=1)
{



	for(int c=0;c<8;c++)
	{
		gpc_polygon &projPoly=projWrtTargetCorner[c];
		for (int i = 0; i <projPoly.num_contours ; i++)
		{
			for (int j = 0; j < projPoly.contour[i].num_vertices; j++)
			{
				double px=curProjPlane[dir];
				double &py=projPoly.contour[i].vertex[j].x;
				double &pz=projPoly.contour[i].vertex[j].y;


				double x1=px-targetCorner[c].x;
				double y1=py-targetCorner[c].y;
				double z1=pz-targetCorner[c].z;

				double x2=d-targetCorner[c].x;
				double y2=(y1/x1)*x2;
				double z2=(z1/x1)*x2;


				py=targetCorner[c].y+y2;
				pz=targetCorner[c].z+z2;



			}
		}
	}


}


bool ncmp( Entry e1,Entry e2 )
{
	return e1.bounces[0]<e2.bounces[0];
}
int countChild(RTNode *rtn )
{
	//return 0;
	int ret=0;

	queue<RTNode*>Q;
	
	

	if( rtn->level )Q.push( rtn );
	else
	{
		ret++;
		return ret;
	}


	while (!Q.empty())
	{
		RTNode *rtn=Q.front();

		Q.pop();


		for(int  e=0;e<rtn->num_entries;e++ )
		{
			RTNode *u= new RTNode( rtree,rtn->entries[e].son );
			
			if( rtn->level )
			{
				Q.push( u );
				continue;
			}
			ret++;

			//cout<<ret<<endl;
		}


	}

	return ret;

}


int countChild1( )
{
	//return 0;
	int ret=0;

	queue<int>Q;
	
	Q.push( rtree->root );
	

	RTNode *rtn;

	while (!Q.empty())
	{

		int v= Q.front();
		rtn = new RTNode(rtree, v);
		Q.pop();


		for(int  e=0;e<rtn->num_entries;e++ )
		{
			Entry &u= rtn->entries[e];
			
			if( rtn->level )
			{
				Q.push( u.son );
				continue;
			}
			ret++;

			//cout<<ret<<endl;
		}


	}

	return ret;

}


int totDiscard=0,totCnt=0;
double totTime=0;
priority_queue< int,vi,comparatorForPq >Q;
void gpc_clear(gpc_polygon &g)
{
	delete g.contour;
	delete g.hole;
}

void box_clear( Box &b )
{
//	return;
	delete b.bx;
}

// this is the function kaysar sir showed us
void reductionInPosXRtree(int dir)
{

	
	// 8 corners of the target object are calculated for determining shadow wrt each corner
	for( int i1=0,id=0;i1<2;i1++ )
		for( int j1=0;j1<2;j1++ )
			for( int k1=0;k1<2;k1++ )
				targetCorner[id++]=Vector3( target.x[i1],target.y[j1],target.z[k1] );


	
	while(!Q.empty())Q.pop();
	
	


	target.print();

	Q.push( rtree->root );
	int cnt=0,discard=0;

	Box projPoly,boundRect,bu,projPoly1;
	Plane projPlane;
	RTNode *rtn;
	gpc_polygon gProjPoly,local_p[8],gb,global_p;
	
	bool debug=0,b;

	srand(time(NULL));
	//int prune=1300+rand()%100;
	//prune=100;

	int io=1;

	while(!Q.empty())
	{
		if(debug==1)printf("s-1");
		int v= Q.top();
		rtn = new RTNode(rtree, v);
		Q.pop();



		io+=rtn->num_entries;

		for(int  e=0;e<rtn->num_entries;e++ )
		{
			Entry &u = rtn->entries[e];






			//case1 out of view because of location
			
			if( target.x[1]>=u.bounces[1] )
			{

		//		fprintf(logfp,"discarding %d not in +x direction\n",u.son);
		//		printf("discarding %d not in +x direction\n",u.son);

				continue;
			}
				
			
			
			
			
			// out of bounding view box
			if( u.bounces[0]>2*segDist[NDISTESEG-1] )
			{
					
			//	printf("out of the far distance %lf %lf\n",segDist[NDISTESEG-1],u.bounces[0]);
				continue;
			}
			







			//case 2 out of view because of projection
			 
		
			// we will calculate projection on a plane , determine that plane
			double d=u.bounces[0];
			projPlane=Plane( CO(unitDir[dir]),d ); //change here for direction


			//now determineing the dimension of that plane, boundRect will contain rectangular plane
			b=calcProjectionWrtPoint( projPlane,target.o,dirBox[dir],boundRect,dir );
			if(!b)continue;


			// create the box object for the current obstacle
			Vector3 v0(u.bounces[0],u.bounces[2],u.bounces[4]);
			Vector3 v1(u.bounces[1],u.bounces[3],u.bounces[5]);
			bu.init(v0,v1,Origin);
			bu.setPlanes();
			



			// check if the 0 dir face of the current obstacle 
			// is out of projection rectangle
			projPoly=bu.bx[0];

			if(projPoly.x[0]>projPoly.x[1])swap(projPoly.x[0],projPoly.x[1]);
			if(projPoly.y[0]>projPoly.y[1])swap(projPoly.y[0],projPoly.y[1]);
			if(projPoly.z[0]>projPoly.z[1])swap(projPoly.z[0],projPoly.z[1]);

			b=commonBox( projPoly,boundRect,projPoly1 );
			
			if( !b )
			{
	
			//	fprintf(logfp,"discarding %d out of projection plane\n",u.son);
			//	printf("discarding %d out of projection plane\n",u.son);
				continue; //discard as it's out of projection plane
			}

			
			



			//calculate the projection of the current obstacles 
			// wrt. all 8 corner points of the target object
			// local_p[i] => projection wrt i'th corner point
			
			int id=8;
			if(debug==1)printf("s4");

			for (int j = 0; j < id; j++)
			{
				gpc_free_polygon(&local_p[j]);
				b=calcCovexProjectionWrtPoint(projPlane,targetCorner[j],bu,local_p[j],dir);
				if(!b)break;
				//assert(  local_p[j].num_contours==0 || local_p[j].contour[0].num_vertices>=0 );
			}
			if(!b)continue;

			if(debug==1)printf("s5");



			// now merging the all the projection in gProjPoly

			gpc_free_polygon(&gProjPoly);
			gpc_copy_polygon(&gProjPoly,&local_p[id-1]);
			
			for (int j = 0; j < id-1; j++)
			{
				Adapter.polygonMerge( gProjPoly,local_p[j] );
			}

			if(debug==1)printf("s6");



			//clipping the mega projection polygon within boundingrectangle 
			gpc_free_polygon(&gb);
			Adapter.adapt(boundRect,gb);
			Adapter.polygonIntersection(gProjPoly,gb);

			
			if(debug==1)printf("s7");

			//now check if it's blocked by other projections
			//extend the projection to current layer

			extedProjToCurLayer( d );
			
			if(debug==1)printf("s8");



			//now calculate the intersection of all 8 projection polygon
			//of current layer

			

			gpc_free_polygon(&global_p);
			gpc_copy_polygon(&global_p,&projWrtTargetCorner[7]);
			

			for( int j=0;j<7;j++ )
			{
				Adapter.polygonIntersection( global_p,projWrtTargetCorner[j] );
			}

			if(debug==1)printf("s9");
		

			//check if the current layer projection contains the current obstacles fully.

			b=Adapter.contains( global_p,gProjPoly );
			
			/*
			b=1;
			for(int i=0;i<8;i++)
			{
				b&=Adapter.contains( projWrtTargetCorner[i],gProjPoly );
			}
			*/

			if(debug==1)printf("s10");
			if(b  )
			{
				//fprintf(logfp,"discarding id %d view blocked by other obstacles\n",u.son);
				//printf("discarding id %d view blocked by other obstacles\n",u.son);
				continue;
			}

			if(debug==1)printf("s11");


			//current object can not be discarded
			//so break it
			
			if( rtn->level  )
			{
				dstFromTar[u.son]=u.bounces[0];
				Q.push( u.son );

				continue;
			}

			
			cnt++;

			

			//include the projection of current obstacles 
			//to the global 8 projections

			for (int j = 0; j < id; j++)
			{
			//	printf("\n%d--",projWrtTargetCorner[j].num_contours);
				Adapter.polygonMerge( projWrtTargetCorner[j],local_p[j] );
				//printf("%d\n",projWrtTargetCorner[j].num_contours);
				//assert(  local_p[j].num_contours==1 && local_p[j].contour[0].num_vertices>=0 );
			}
			
			if(debug==1)printf("s12");



			//save the potenially visible obstacle
			curProjPlane[dir]=d;
			dirObstacle[1].resize(dirObstacle[1].size()+1);
			dirObstacle[1].back()= bu ;
			
			
			if(debug==1)printf("s13");

			/*
			if(cnt%50==0)
			{
				int sz=0,csz=0;
				for(int i=0;i<8;i++)
				{

					csz+=projWrtTargetCorner[i].num_contours;
			//		printf("%d : ->",projWrtTargetCorner[i].num_contours);
					for(int j=0;j<projWrtTargetCorner[i].num_contours;j++)
					{
		//				printf("%d ",projWrtTargetCorner[i].contour[j].num_vertices);
//						assert(projWrtTargetCorner[i].contour[j].num_vertices>0);
						sz+=projWrtTargetCorner[i].contour[j].num_vertices;
					}
				//	printf("\n");
				}
				printf("%d %d %d\n",cnt,sz,csz);
				fprintf(logfp,"%d\n",cnt);
				fflush(logfp);
			}*/


		}

		delete rtn;
//		if(cnt>prune)break;
	}


	//printf(" obstacle discarded %d, reduced obstacle set size%d\n",discard,cnt);
	//fprintf(efile," obstacle discarded %d, reduced obstacle set size%d\n",discard,cnt);
	totDiscard+=discard;
	totCnt+=cnt;

	printf("reduced obstacle set size %d\n",cnt);
	fprintf(logfp,"reduced obstacle set size %d\n",cnt);
	fprintf(logfp,"total io %d\n",io);
	fflush(logfp);
	//while(1);

	//cout<<All.size()<<endl;
	//cout<<"total "<<cnt<<endl;

}


void reductionInPosXRtree1(int dir)
{

	
	// 8 corners of the target object are calculated for determining shadow wrt each corner
	for( int i1=0,id=0;i1<2;i1++ )
		for( int j1=0;j1<2;j1++ )
			for( int k1=0;k1<2;k1++ )
				targetCorner[id++]=Vector3( target.x[i1],target.y[j1],target.z[k1] );


	
	while(!Q.empty())Q.pop();
	
	

	target.print();

	Q.push( rtree->root );
	int cnt=0,discard=0;

	Box projPoly,boundRect,bu,projPoly1;
	Plane projPlane;
	RTNode *rtn;
	gpc_polygon gProjPoly,local_p[8],gb,global_p;
	
	bool debug=0,b;


	while(!Q.empty())
	{
		if(debug==1)printf("s-1");
		int v= Q.top();
		rtn = new RTNode(rtree, v);
		Q.pop();



		for(int  e=0;e<rtn->num_entries;e++ )
		{
			Entry &u = rtn->entries[e];






			//case1 out of view because of location
			
			if( target.x[1]>=u.bounces[1] )
			{

		//		fprintf(logfp,"discarding %d not in +x direction\n",u.son);
		//		printf("discarding %d not in +x direction\n",u.son);

				continue;
			}
				
			
			
			
			/*
			// out of bounding view box
			if( u.bounces[0]>segDist[NDISTESEG-1] )
			{
					
				printf("out of the far distance %lf %lf\n",segDist[NDISTESEG-1],u.bounces[0]);
				continue;
			}
			*/







			//case 2 out of view because of projection
			 
		
			// we will calculate projection on a plane , determine that plane
			double d=u.bounces[0];
			projPlane=Plane( CO(unitDir[dir]),d ); //change here for direction


			//now determineing the dimension of that plane, boundRect will contain rectangular plane
			b=calcProjectionWrtPoint( projPlane,target.o,dirBox[dir],boundRect,dir );
			if(!b)continue;


			// create the box object for the current obstacle
			Vector3 v0(u.bounces[0],u.bounces[2],u.bounces[4]);
			Vector3 v1(u.bounces[1],u.bounces[3],u.bounces[5]);
			bu.init(v0,v1,Origin);
			bu.setPlanes();
			



			// check if the 0 dir face of the current obstacle 
			// is out of projection rectangle
			projPoly=bu.bx[0];

			if(projPoly.x[0]>projPoly.x[1])swap(projPoly.x[0],projPoly.x[1]);
			if(projPoly.y[0]>projPoly.y[1])swap(projPoly.y[0],projPoly.y[1]);
			if(projPoly.z[0]>projPoly.z[1])swap(projPoly.z[0],projPoly.z[1]);

			b=commonBox( projPoly,boundRect,projPoly1 );
			
			if( !b )
			{
	
				fprintf(logfp,"discarding %d out of projection plane\n",u.son);
			//	printf("discarding %d out of projection plane\n",u.son);
				continue; //discard as it's out of projection plane
			}

			
			



			//calculate the projection of the current obstacles 
			// wrt. all 8 corner points of the target object
			// local_p[i] => projection wrt i'th corner point
			
			int id=8;
			if(debug==1)printf("s4");

			for (int j = 0; j < id; j++)
			{
				gpc_free_polygon(&local_p[j]);
				b=calcCovexProjectionWrtPoint(projPlane,targetCorner[j],bu,local_p[j],dir);
				if(!b)break;
				assert(  local_p[j].num_contours==0 || local_p[j].contour[0].num_vertices>=0 );
			}
			if(!b)continue;

			

			//clipping all the projection polygon

			gpc_free_polygon(&gb);
			Adapter.adapt(boundRect,gb);
			for( int i=0;i<8;i++ )
			{
				Adapter.polygonIntersection(local_p[i],gb);
			}
			
			if(debug==1)printf("s7");

			//now check if it's blocked by other projections
			//extend the projection to current layer

			extedProjToCurLayer( d );
			
			
			//check if the current layer projection contains the current obstacles fully.
			b=1;
			for(int i=0;i<8;i++)
			{
				b&=Adapter.contains( projWrtTargetCorner[i],local_p[i] );
			}

			

			if(debug==1)printf("s10");
			if(b)
			{
				fprintf(logfp,"discarding id %d view blocked by other obstacles\n",u.son);
				//printf("discarding id %d view blocked by other obstacles\n",u.son);
				continue;
			}

			if(debug==1)printf("s11");


			//current object can not be discarded
			//so break it
			
			if( rtn->level )
			{
				dstFromTar[u.son]=u.bounces[0];
				Q.push( u.son );

				continue;
			}

			cnt++;


			//include the projection of current obstacles 
			//to the global 8 projections

			for (int j = 0; j < id; j++)
			{
			//	printf("\n%d--",projWrtTargetCorner[j].num_contours);
				Adapter.polygonMerge( projWrtTargetCorner[j],local_p[j] );
				//printf("%d\n",projWrtTargetCorner[j].num_contours);
				assert(  local_p[j].num_contours==1 && local_p[j].contour[0].num_vertices>=0 );
			}
			
			if(debug==1)printf("s12");



			//save the potenially visible obstacle
			curProjPlane[dir]=d;
			dirObstacle[1].resize(dirObstacle[1].size()+1);
			dirObstacle[1].back()= bu ;
			
			
			if(debug==1)printf("s13");
			
			if(cnt%50==0)
			{
				int sz=0,csz=0;
				for(int i=0;i<8;i++)
				{

					csz+=projWrtTargetCorner[i].num_contours;
			//		printf("%d : ->",projWrtTargetCorner[i].num_contours);
					for(int j=0;j<projWrtTargetCorner[i].num_contours;j++)
					{
		//				printf("%d ",projWrtTargetCorner[i].contour[j].num_vertices);
//						assert(projWrtTargetCorner[i].contour[j].num_vertices>0);
						sz+=projWrtTargetCorner[i].contour[j].num_vertices;
					}
				//	printf("\n");
				}
				printf("%d %d %d\n",cnt,sz,csz);
				fprintf(logfp,"%d\n",cnt);
				fflush(logfp);
			}

		}

		delete rtn;
	}


	//printf(" obstacle discarded %d, reduced obstacle set size%d\n",discard,cnt);
	//fprintf(efile," obstacle discarded %d, reduced obstacle set size%d\n",discard,cnt);
	totDiscard+=discard;
	totCnt+=cnt;

	//printf("reduced obstacle set size %d\n",cnt);
	fprintf(logfp,"reduced obstacle set size %d\n",cnt);
	//while(1);

	//cout<<All.size()<<endl;
	//cout<<"total "<<cnt<<endl;

}


void reductionInPosX(int dir) //dir=1
{
	puts("reduction in positive x direction...");

	priority_queue<int, vector<int>, CompareLoX> pq;
	bool bdir=dir&1;
	for(int i=0;i<obstacles.size();i++)if( obstacles[i].x[bdir^1]>=target.x[bdir] )pq.push(i); //change here ofr direction

	int &curSeg=totSeg[dir];
	curSeg=0;

	//cout<<"(";

	while(!pq.empty())
	{
	 	int u=pq.top();
		pq.pop();



		while( curSeg<NDISTESEG-1 && target.x[dir]+segDist[curSeg]<=obstacles[ u ].x[ dir ] )
		{
			curSeg++;
			update( dir,curSeg ); // eikhane optimize korbo aro...
			Adapter.adapt( dir,segDist[curSeg],gProjPolys[dir][curSeg],ProjPolys[dir][curSeg] );

		}


		Box projPoly;
		Plane projPlane=Plane( CO(unitDir[dir]),segDist[curSeg]+target.o.x ); //change here for direction
		Box boundRect=projRect[dir][curSeg];

		//cout<<"[";
		bool b=calcProjectionWrtCube(projPlane, target , obstacles[u] ,projPoly);
		//cout<<"]";
		if(!b)
		{
		//	printf("discarding id %d because of near distance\n",u);
			continue;
		}
		Box projPoly1;
		//cout<<"{";
		b=commonBox( projPoly,boundRect,projPoly1 );
		//cout<<"}";
		if( !b )
		{
		//	printf("discarding id %d because of because out of the projection rectangle\n",u);
			continue;
		}
		gpc_polygon gProjPoly;

		b=calcConvexProjectionWrtCube( projPlane,target,obstacles[u],gProjPoly,dir );

		if(gProjPoly.num_contours==0)continue;

		//cout<<"<";
		b=Adapter.contains( gProjPolys[dir][curSeg],gProjPoly );
		//cout<<">";
		if(b)
		{
		//	printf("discarding id %d view blocked by other obstacles\n",u);
			continue;
		}

		dirObstacleId[dir].push_back( u );
	}

	curSeg=NDISTESEG-2;

	//cout<<")";
}


void reductionInPosY(int dir) //dir=3
{

	puts("(reduction in +y direction...");
	priority_queue<int, vector<int>, CompareLoY> pq;

	bool bdir=dir&1;
	for(int i=0;i<obstacles.size();i++)if( obstacles[i].y[bdir^1]>=target.y[bdir] )pq.push(i); //change here for direction

	int &curSeg=totSeg[dir];
	curSeg=0;

	//putchar('(');
	while(!pq.empty())
	{
	 	int u=pq.top();
		pq.pop();

	//	putchar('{');

		while( curSeg<NDISTESEG-1 && target.y[dir]+segDist[curSeg]<=obstacles[ u ].y[ dir ] )
		{
			curSeg++;
			update( dir,curSeg ); // eikhane optimize korbo aro...
			Adapter.adapt( dir,segDist[curSeg],gProjPolys[dir][curSeg],ProjPolys[dir][curSeg] );

		}

	//	putchar('}');


		Box projPoly;
		Plane projPlane=Plane( CO(unitDir[dir]),segDist[curSeg]+target.o.y ); //change here for direction
		Box boundRect=projRect[dir][curSeg];

		//putchar('[');
		bool b=calcProjectionWrtCube(projPlane, target , obstacles[u] ,projPoly);
	//	putchar(']');

		if(!b)
		{
			printf("discarding id %d because of near distance\n",u);
			continue;
		}
		Box projPoly1;

	//	putchar('<');
		b=commonBox( projPoly,boundRect,projPoly1 );
	//	putchar('>');
		if( !b )
		{
			printf("discarding id %d because of because out of the rojection rectangle\n",u);
			continue;
		}
		gpc_polygon gProjPoly;

		calcConvexProjectionWrtCube( projPlane,target,obstacles[u],gProjPoly,dir );

		if(gProjPoly.num_contours==0)continue;

	//	putchar('6');
		b=Adapter.contains( gProjPolys[dir][curSeg],gProjPoly );
		//putchar( '9' );
		if(b)
		{
			printf("discarding id %d view blocked by other obstacles\n",u);
			continue;
		}

		dirObstacleId[dir].push_back( u );
	}
	putchar(')');
	curSeg=NDISTESEG-2;
}


void reductionInPosZ(int dir) //dir=5
{
	puts("reduction in +z direction...");

	priority_queue<int, vector<int>, CompareLoZ> pq;

	bool bdir=dir&1;
	for(int i=0;i<obstacles.size();i++)if( obstacles[i].z[bdir^1]>=target.z[bdir] )pq.push(i); //change here for direction

	int &curSeg=totSeg[dir];
	curSeg=0;


	while(!pq.empty())
	{
	 	int u=pq.top();
		pq.pop();



		while( curSeg<NDISTESEG-1 && target.z[dir]+segDist[curSeg]<=obstacles[ u ].z[ dir ] )
		{
			curSeg++;
			update( dir,curSeg ); // eikhane optimize korbo aro...
			Adapter.adapt( dir,segDist[curSeg],gProjPolys[dir][curSeg],ProjPolys[dir][curSeg] );

		}


		Box projPoly;
		Plane projPlane=Plane( CO(unitDir[dir]),segDist[curSeg]+target.o.z ); //change here for direction
		Box boundRect=projRect[dir][curSeg];
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
		gpc_polygon gProjPoly;

		calcConvexProjectionWrtCube( projPlane,target,obstacles[u],gProjPoly,dir );

		if(Adapter.contains( gProjPolys[dir][curSeg],gProjPoly ))
		{
			printf("discarding id %d view blocked by other obstacles\n",u);
			continue;
		}

		dirObstacleId[dir].push_back( u );
	}

	curSeg=NDISTESEG-2;
}



void reductionOfObstacle(  )
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

void vcmPhase1()
{
//	reductionOfObstacle();
}

int nObstacle;

void input()
{
	FILE* fp=fopen("sample1.txt","r");


	//target.fscan(fp);




	Vector3 v0,v1;
	v0.fscan(fp);
	v1.fscan(fp);
	Origin=v0.add(v1).div(2);
	target.init(v0,v1,Origin);
	target.setPlanes();





	fscanf(fp,"%d",&nObstacle);

	obstacles=vector<Box>(nObstacle);

	for(int i=0;i<(int)obstacles.size();i++)
	{
		obstacles[i].fscan(fp,Origin);
	}

	fclose(fp);
}

vector<Vector3> rColor;

bool showDistRec=1,showProjPoly=1,showObstacle=1,showAll=1,drawg=1;
int showid=0,curSegId=0;





int vcmArray[6][MAXNDISTESEG][MAXSEG][MAXSEG]; //this array will contain the final result\
											if a grid is visible from view point or not
int naiveVcmArray[6][MAXNDISTESEG][MAXSEG][MAXSEG];

// draw the cell
double getDim(int dir,int curSeg);


void generateRandCol()
{
	for(int i=0;i<30;i++)rColor.push_back(Vector3( rand()%255,rand()%255,rand()%255 ));
}






double getDim(int dir,int curSeg) // gives the dimension of the projection rectangle
{
	double dim=0;

	dim=max( dim, fabs(projRect[ dir ][ curSeg ].x[0]-projRect[ dir ][ curSeg ].x[1]) );
	dim=max( dim, fabs(projRect[ dir ][ curSeg ].y[0]-projRect[ dir ][ curSeg ].y[1]) );
	dim=max( dim, fabs(projRect[ dir ][ curSeg ].z[0]-projRect[ dir ][ curSeg ].z[1]) );

	return dim;
}

pair<int,int> pointToarrayIndex( double px, double py,int dir,int curSeg )
{

	//cout<<"**"<<px<<" "<<py<<endl;


	double dim=getDim( dir,curSeg );


//	assert( ( (2*px>=-dim) && (2*px<=+dim) ));


	//making the -ve index positive

	px+=dim*.5;
	py+=dim*.5;

	if(!( ( px<=dim ) && ( py<=dim ) && ( px>=0 ) && ( py>=0 ) ) )
	{
		////cout<<"-->"<<dim<<" "<<px<<" "<<py<<endl;
		//while(1);
	}


	px=min( dim,px );
	py=min( dim,py );
	px=max( 0,px );
	py=max( 0,py );



	// we have to convert a dim*dim rectangle's point in geometric space to\
	2d array index

	// suppose the dir=1;
	// rectangles in direction 1 that is in x(-y) plane
	// so lower left point corresponds to 0,0

	int x,y;
	x= px*MAXSEG/dim +.5;x--;
	y= py*MAXSEG/dim +.5;y--;


	x=min( MAXSEG-1,x );
	y=min( MAXSEG-1,y );
	x=max( 0,x );
	y=max( 0,y );



	return make_pair( x,y );

}



double getx( Vector v0,Vector v1,double y ) // intersection of v0--v1 with y horizontal line
{
	// (x-x0)/(x0-x1)=(y-y0)/(y0-y1);\
	x=(y-y0)*(x0-x1)/(y0-y1) + x0


	double x=(y-v0.y)*(v0.x-v1.x)/(v0.y-v1.y) + v0.x;
	return x;
}


bool cmpFormonoPolygonTo2Darray(Vector v1,Vector v2)
{
	return v1.y<v2.y;
}

// map the monotone polygon to the 2D array
void monoPolygonTo2Darray(int dir,int curSeg,vector<Vector>mpoly  )
{

	//return;

	int low=0,hi=0;

	// find the upper and lower y

	for(int i=1;i<mpoly.size();i++ )if( mpoly[i].y<mpoly[low].y )low=i;
	for(int i=1;i<mpoly.size();i++ )if( mpoly[i].y>mpoly[hi].y )hi=i;

	vector< Vector >pq;

	for(int i=0;i<mpoly.size();i++)pq.push_back( mpoly[i] );
	double dim= getDim( dir,curSeg ) ;


	double delta = dim/MAXSEG; // length of each small segment

	for( double y=(mpoly[low].y+delta);y<=mpoly[hi].y-delta;y+=delta )
	{
		pq.push_back( Vector( +INF,y )  ); // x=INF means it is for horizontal lines
	}

	int n=mpoly.size();
	int left=low,right=low,nleft,nright;


	vector<bool>vst( mpoly.size(),0 );

	vst[left]=true;

	//int cury=pointToarrayIndex( +INF,low,dir,curSeg ).second;

	sort(pq.begin(),pq.end(),cmpFormonoPolygonTo2Darray);

	//pq.pop();

	for(int i=1;i<pq.size();i++)
	{

		Vector u=pq[i];
//		pq.pop();


		nleft=(left-1+n)%n;
		nright=(right+1)%n;


		if(u.x==INF)
		{

			if( !(mpoly[left].y<=u.y && mpoly[nleft].y>=u.y && mpoly[right].y<=u.y && mpoly[nright].y>=u.y  ))
			{

				//ektu error ase.. eita pore dekhbo
			//	while(1);
			}


			// found a y strip need to update

		//	continue;
			int xleft=getx( mpoly[left],mpoly[nleft],u.y );
			int xright=getx( mpoly[right],mpoly[nright],u.y );



			pii arrayleft=pointToarrayIndex(  xleft ,u.y ,dir,curSeg );
			pii arrayright=pointToarrayIndex(  xright ,u.y ,dir,curSeg );




		//	//cout<<xleft<<" "<<u.y<<" "<<arrayleft.first<<" "<<arrayleft.second<<endl;
	//		//cout<<xright<<" "<<u.y<<" "<<arrayright.first<<" "<<arrayright.second<<endl;

			vcmArray[dir][curSeg][arrayleft.second][ arrayleft.first ]++;
			vcmArray[dir][curSeg][arrayleft.second][ arrayright.first ]--;

			continue;
		}









		if( Z(u.y-mpoly[nleft].y)  )left=nleft;
		if( Z(u.y-mpoly[right].y)  ) right=nright;




	}


}


void mergeAll(int dir,int curSeg)
{


	for( int i=0;i<MAXSEG;i++ )
	{
		for( int j=1;j<MAXSEG;j++ )
		{
			vcmArray[dir][curSeg][i][j]+=vcmArray[dir][curSeg][i][j-1];
		}
	}


}



void extedProjToCurLayer1( int tid,int curSeg,int dir=1)
{




	Vector3 c=targetPoints[tid];
	gpc_polygon &g=gVcmProjPolys[dir][curSeg][tid];
	double d=segDist[curSeg];

	if(curSeg)
	{
		gpc_polygon &gprev=gVcmProjPolys[dir][curSeg-1][tid];

		for (int i = 0; i <gprev.num_contours ; i++)
		{
			gpc_add_contour( &g, &gprev.contour[i],0 );

			for (int j = 0; j < gprev.contour[i].num_vertices; j++)
			{
				double px=segDist[curSeg-1];
				double py=gprev.contour[i].vertex[j].x;
				double pz=gprev.contour[i].vertex[j].y;


				double x1=px-c.x;
				double y1=py-c.y;
				double z1=pz-c.z;

				double x2=d-c.x;
				double y2=(y1/x1)*x2;
				double z2=(z1/x1)*x2;


				py=c.y+y2;
				pz=c.z+z2;

				g.contour[i].vertex[j].x=py;
				g.contour[i].vertex[j].y=pz;



			}
		}

	}


}


void updateVcm1( vector<Box>&obstacleWrtPoint,int dir,int curSeg,int tid )
{

	//vector<int>&id=obstacleWrtPoint;


	extedProjToCurLayer1(tid,curSeg,dir);

	Vector3 c=targetPoints[tid];
	gpc_polygon &g=gVcmProjPolys[dir][curSeg][tid];


	//g.num_contours=0;


	for( int i=0;i<obstacleWrtPoint.size();i++ )
	{
		gpc_polygon local_p;
		Plane projPlane=Plane( unitDir[dir].x,unitDir[dir].y,unitDir[dir].z,segDist[curSeg]+c.x );
		calcCovexProjectionWrtPoint(projPlane, c , obstacleWrtPoint[i] ,local_p,dir);
		Adapter.polygonMerge( g,local_p );
	}

	obstacleWrtPoint.clear();

}




bool fcmp(Box b1,Box b2)
{
	return b1.x[0]<b2.x[0];
}

void generateMapInPosX( int tid,int dir )
{

	Vector3 tp=targetPoints[tid];
	//int dir=1;

	int curSeg=1;


	vector<Box>obstacleWrtPoint;
	sort(dirObstacle[dir].begin(),dirObstacle[dir].end(),fcmp);
//	soet(dirObstacle[dir].begin(),dirObstacle[dir].end());

	//tp.print();

	for(int i=0;i<dirObstacle[dir].size();i++)
	{
		

		Box u=dirObstacle[dir][i];

		//u.print();

		Box projPoly1;
		bool b=commonBox( target,u,projPoly1 );

		if(b)continue;


		while( curSeg<NDISTESEG && tp.x+segDist[curSeg]<=u.x[ 0 ] )
		{

			updateVcm1( obstacleWrtPoint,dir,curSeg,tid ); // eikhane optimize korbo aro...
			curSeg++;

		//	Adapter.adapt( dir,tp.x+segDist[curSeg],gVcmProjPolys[dir][curSeg][tid],vcmProjPolys[dir][curSeg][tid] );

		}


		int lastSeg=curSeg-1;

		//vector< vector<Vector3> > &p=vcmProjPolys[dir][curSeg-1][tid];
		gpc_polygon &g=gVcmProjPolys[dir][lastSeg][tid];


		Box projPoly;
		Plane projPlane=Plane( unitDir[dir].x,unitDir[dir].y,unitDir[dir].z,segDist[lastSeg]); //change here for direction
		Box boundRect=projRect[dir][lastSeg];


		gpc_polygon gProjPoly;

		calcCovexProjectionWrtPoint( projPlane,tp,u,gProjPoly,dir );

		if(Adapter.contains( g,gProjPoly ))
		{
			//printf("discarding id %d view blocked by other obstacles\n",u);
			continue;
		}
		//printf("asd");
		obstacleWrtPoint.resize(obstacleWrtPoint.size()+1);
		obstacleWrtPoint.back()=( u );
	}


	while( curSeg<NDISTESEG)
	{

		updateVcm1( obstacleWrtPoint,dir,curSeg,tid ); // eikhane optimize korbo aro...
		curSeg++;

	//	Adapter.adapt( dir,tp.x+segDist[curSeg],gVcmProjPolys[dir][curSeg][tid],vcmProjPolys[dir][curSeg][tid] );

	}

	//cout<<"here..."<<endl;

	for( int curSeg=1;curSeg< NDISTESEG ;curSeg++ )
	{

		gpc_polygon &g=gVcmProjPolys[dir][curSeg][tid];


		
		//cout<<g.num_contours<<endl;

		Box prec=projRect[dir][curSeg];
		gpc_polygon gprec;

		Adapter.adapt(prec,gprec);
		Adapter.polygonIntersection(g,gprec);


		vector< vector<Vector> >vp;

		Adapter.adapt(g,vp); //vp contains all separate polygons

		vector< vector<Vector> >monoTonePolys;


		Adapter.adapt( dir,segDist[curSeg],gVcmProjPolys[dir][curSeg][tid],vcmProjPolys[dir][curSeg][tid] );

		for( int i=0;i<vp.size();i++ )
		{
			//cout<<endl;
			reverse( vp[i].begin(),vp[i].end() );
			vector< vector<Vector> >temp=M.getMonotonePartition( vp[i] );
			for(int j=0;j<temp.size();j++)monoTonePolys.push_back( temp[j] );
		}

		//printf("%d\n",monoTonePolys.size());

		for( int i=0;i<monoTonePolys.size();i++ )
		{
			monoPolygonTo2Darray( dir,curSeg,monoTonePolys[i]  );
		}


	}



}



void generateMapInPosX1( int tid,int dir )
{

	Vector3 tp=targetPoints[tid];
	//int dir=1;
	vector< int >&obstacleId=dirObstacleId[dir];

	int curSeg=0;


	vector<int>obstacleWrtPoint;

	for(int i=0;i<obstacleId.size();i++)
	{
		int u=obstacleId[i];




		while( curSeg<NDISTESEG-1 && tp.x+segDist[curSeg]<=obstacles[ u ].x[ dir ] )
		{
			curSeg++;
			updateVcm( obstacleWrtPoint,dir,curSeg,tid ); // eikhane optimize korbo aro...


		//	Adapter.adapt( dir,tp.x+segDist[curSeg],gVcmProjPolys[dir][curSeg][tid],vcmProjPolys[dir][curSeg][tid] );

		}


		vector< vector<Vector3> > &p=vcmProjPolys[dir][curSeg][tid];
		gpc_polygon &g=gVcmProjPolys[dir][curSeg][tid];


		Box projPoly;
		Plane projPlane=Plane( unitDir[dir].x,unitDir[dir].y,unitDir[dir].z,segDist[curSeg]); //change here for direction
		Box boundRect=projRect[dir][curSeg];

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

		calcCovexProjectionWrtPoint( projPlane,tp,obstacles[u],gProjPoly,dir );

		if(Adapter.contains( g,gProjPoly ))
		{
		//	printf("discarding id %d view blocked by other obstacles\n",u);
			continue;
		}

		obstacleWrtPoint.push_back( u );
	}


	while( curSeg<NDISTESEG-2)
	{
		curSeg++;
		updateVcm( obstacleWrtPoint,dir,curSeg,tid ); // eikhane optimize korbo aro...


	//	Adapter.adapt( dir,tp.x+segDist[curSeg],gVcmProjPolys[dir][curSeg][tid],vcmProjPolys[dir][curSeg][tid] );

	}



	for( int curSeg=1;curSeg<=totSeg[dir];curSeg++ )
	{

		gpc_polygon &g=gVcmProjPolys[dir][curSeg][tid];
		Box prec=projRect[dir][curSeg];
		gpc_polygon gprec;

		Adapter.adapt(prec,gprec);
		Adapter.polygonIntersection(g,gprec);


		vector< vector<Vector> >vp;

		Adapter.adapt(g,vp); //vp contains all separate polygons

		vector< vector<Vector> >monoTonePolys;


		Adapter.adapt( dir,segDist[curSeg],gVcmProjPolys[dir][curSeg][tid],vcmProjPolys[dir][curSeg][tid] );

		for( int i=0;i<vp.size();i++ )
		{
			//cout<<endl;
			reverse( vp[i].begin(),vp[i].end() );
			vector< vector<Vector> >temp=M.getMonotonePartition( vp[i] );
			for(int j=0;j<temp.size();j++)monoTonePolys.push_back( temp[j] );
		}


		for( int i=0;i<monoTonePolys.size();i++ )
		{
			monoPolygonTo2Darray( dir,curSeg,monoTonePolys[i]  );
		}


	}



}

void generateMapInPosY( int tid,int dir )
{

	Vector3 tp=targetPoints[tid];
	//int dir=1;
	vector< int >&obstacleId=dirObstacleId[dir];

	int curSeg=0;


	vector<int>obstacleWrtPoint;

	for(int i=0;i<obstacleId.size();i++)
	{
		int u=obstacleId[i];




		while( curSeg<NDISTESEG-1 && tp.y+segDist[curSeg]<=obstacles[ u ].y[ dir ] )
		{
			curSeg++;
			updateVcm( obstacleWrtPoint,dir,curSeg,tid ); // eikhane optimize korbo aro...


		//	Adapter.adapt( dir,tp.x+segDist[curSeg],gVcmProjPolys[dir][curSeg][tid],vcmProjPolys[dir][curSeg][tid] );

		}


		vector< vector<Vector3> > &p=vcmProjPolys[dir][curSeg][tid];
		gpc_polygon &g=gVcmProjPolys[dir][curSeg][tid];


		Box projPoly;
		Plane projPlane=Plane( unitDir[dir].x,unitDir[dir].y,unitDir[dir].z,segDist[curSeg]); //change here for direction
		Box boundRect=projRect[dir][curSeg];

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

		calcCovexProjectionWrtPoint( projPlane,tp,obstacles[u],gProjPoly,dir );

		if(Adapter.contains( g,gProjPoly ))
		{
			printf("discarding id %d view blocked by other obstacles\n",u);
			continue;
		}

		obstacleWrtPoint.push_back( u );
	}


	while( curSeg<NDISTESEG-2)
	{
		curSeg++;
		updateVcm( obstacleWrtPoint,dir,curSeg,tid ); // eikhane optimize korbo aro...


	//	Adapter.adapt( dir,tp.x+segDist[curSeg],gVcmProjPolys[dir][curSeg][tid],vcmProjPolys[dir][curSeg][tid] );

	}



	for( int curSeg=1;curSeg<=totSeg[dir];curSeg++ )
	{

		gpc_polygon &g=gVcmProjPolys[dir][curSeg][tid];
		Box prec=projRect[dir][curSeg];
		gpc_polygon gprec;

		Adapter.adapt(prec,gprec);
		Adapter.polygonIntersection(g,gprec);


		vector< vector<Vector> >vp;

		Adapter.adapt(g,vp); //vp contains all separate polygons

		vector< vector<Vector> >monoTonePolys;


		Adapter.adapt( dir,segDist[curSeg],gVcmProjPolys[dir][curSeg][tid],vcmProjPolys[dir][curSeg][tid] );

		for( int i=0;i<vp.size();i++ )
		{
			//cout<<endl;
			reverse( vp[i].begin(),vp[i].end() );
			vector< vector<Vector> >temp=M.getMonotonePartition( vp[i] );
			for(int j=0;j<temp.size();j++)monoTonePolys.push_back( temp[j] );
		}


		for( int i=0;i<monoTonePolys.size();i++ )
		{
			monoPolygonTo2Darray( dir,curSeg,monoTonePolys[i]  );
		}


	}



}



void generateMapInPosZ( int tid,int dir )
{

	Vector3 tp=targetPoints[tid];
	//int dir=1;
	vector< int >&obstacleId=dirObstacleId[dir];

	int curSeg=0;


	vector<int>obstacleWrtPoint;

	for(int i=0;i<obstacleId.size();i++)
	{
		int u=obstacleId[i];




		while( curSeg<NDISTESEG-1 && tp.z+segDist[curSeg]<=obstacles[ u ].z[ dir ] )
		{
			curSeg++;
			updateVcm( obstacleWrtPoint,dir,curSeg,tid ); // eikhane optimize korbo aro...


		//	Adapter.adapt( dir,tp.x+segDist[curSeg],gVcmProjPolys[dir][curSeg][tid],vcmProjPolys[dir][curSeg][tid] );

		}


		vector< vector<Vector3> > &p=vcmProjPolys[dir][curSeg][tid];
		gpc_polygon &g=gVcmProjPolys[dir][curSeg][tid];


		Box projPoly;
		Plane projPlane=Plane( unitDir[dir].x,unitDir[dir].y,unitDir[dir].z,segDist[curSeg]); //change here for direction
		Box boundRect=projRect[dir][curSeg];

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

		calcCovexProjectionWrtPoint( projPlane,tp,obstacles[u],gProjPoly,dir );

		if(Adapter.contains( g,gProjPoly ))
		{
			printf("discarding id %d view blocked by other obstacles\n",u);
			continue;
		}

		obstacleWrtPoint.push_back( u );
	}


	while( curSeg<NDISTESEG-2)
	{
		curSeg++;
		updateVcm( obstacleWrtPoint,dir,curSeg,tid ); // eikhane optimize korbo aro...


	//	Adapter.adapt( dir,tp.x+segDist[curSeg],gVcmProjPolys[dir][curSeg][tid],vcmProjPolys[dir][curSeg][tid] );

	}



	for( int curSeg=1;curSeg<=totSeg[dir];curSeg++ )
	{

		gpc_polygon &g=gVcmProjPolys[dir][curSeg][tid];
		Box prec=projRect[dir][curSeg];
		gpc_polygon gprec;

		Adapter.adapt(prec,gprec);
		Adapter.polygonIntersection(g,gprec);


		vector< vector<Vector> >vp;

		Adapter.adapt(g,vp); //vp contains all separate polygons

		vector< vector<Vector> >monoTonePolys;


		Adapter.adapt( dir,segDist[curSeg],gVcmProjPolys[dir][curSeg][tid],vcmProjPolys[dir][curSeg][tid] );

		for( int i=0;i<vp.size();i++ )
		{
			//cout<<endl;
			reverse( vp[i].begin(),vp[i].end() );
			vector< vector<Vector> >temp=M.getMonotonePartition( vp[i] );
			for(int j=0;j<temp.size();j++)monoTonePolys.push_back( temp[j] );
		}


		for( int i=0;i<monoTonePolys.size();i++ )
		{
			monoPolygonTo2Darray( dir,curSeg,monoTonePolys[i]  );
		}


	}



}




void generateMapInNegX( int tid,int dir )
{

	Vector3 tp=targetPoints[tid]; // target er upor ekta point nilam
	vector< int >&obstacleId=dirObstacleId[dir]; // oi direction er obstacle er list ta nilam

	int curSeg=0;

	vector<int>obstacleWrtPoint; // ei khane thakbe amar kon kon obstacle nia deal korte hobe


	for(int i=0;i<obstacleId.size();i++)
	{
		int u=obstacleId[i]; // ekta obstacle nilam




		while( curSeg<NDISTESEG-1 && tp.x-segDist[curSeg]>=obstacles[ u ].x[ dir ] )
		{
			//cout<<curSeg<<endl;

			curSeg++;
			updateVcm( obstacleWrtPoint,dir,curSeg,tid ); // eikhane optimize korbo aro...


		//	Adapter.adapt( dir,tp.x+segDist[curSeg],gVcmProjPolys[dir][curSeg][tid],vcmProjPolys[dir][curSeg][tid] );

		}


		vector< vector<Vector3> > &p=vcmProjPolys[dir][curSeg][tid]; // ei segment er vcmProjection polygon er reference
		gpc_polygon &g=gVcmProjPolys[dir][curSeg][tid]; // seitar 2D version


		Box projPoly;
		//desired projection plane ta nilam
		Plane projPlane=Plane( unitDir[dir].x,unitDir[dir].y,unitDir[dir].z,segDist[curSeg] ); //change here for direction
		// sei plan er bounding rectangle ta nilam
		Box boundRect=projRect[dir][curSeg];

		/*

		if(!calcProjectionWrtPoint(projPlane, tp , obstacles[u] ,projPoly,dir))
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
	//	cout<<"agee"<<endl;
		calcCovexProjectionWrtPoint( projPlane,tp,obstacles[u],gProjPoly,dir );
	//	cout<<"pree"<<endl;
		if(Adapter.contains( g,gProjPoly ))
		{
			printf("discarding id %d view blocked by other obstacles\n",u);
			continue;
		}

		obstacleWrtPoint.push_back( u );
	}


	while( curSeg<NDISTESEG-1)
	{
		curSeg++;
		updateVcm( obstacleWrtPoint,dir,curSeg,tid ); // eikhane optimize korbo aro...


	//	Adapter.adapt( dir,tp.x+segDist[curSeg],gVcmProjPolys[dir][curSeg][tid],vcmProjPolys[dir][curSeg][tid] );

	}



	for( int curSeg=1;curSeg<=totSeg[dir];curSeg++ )
	{

		gpc_polygon &g=gVcmProjPolys[dir][curSeg][tid];
		Box prec=projRect[dir][curSeg];
		gpc_polygon gprec;

		Adapter.adapt(prec,gprec);
		Adapter.polygonIntersection(g,gprec);


		vector< vector<Vector> >vp;

		Adapter.adapt(g,vp); //vp contains all separate polygons

		vector< vector<Vector> >monoTonePolys;


		Adapter.adapt( dir,-segDist[curSeg],gVcmProjPolys[dir][curSeg][tid],vcmProjPolys[dir][curSeg][tid] );

		for( int i=0;i<vp.size();i++ )
		{
			//cout<<endl;
			reverse( vp[i].begin(),vp[i].end() );

			/*
			cout<<vp[i].size()<<endl;
			if(vp[i].size()==3)
			{
				for(int j=0;j<vp[i].size();j++)vp[i][j].print();
			}*/
			vector< vector<Vector> >temp=M.getMonotonePartition( vp[i] );


			//cout<<"end"<<endl;


			for(int j=0;j<temp.size();j++)monoTonePolys.push_back( temp[j] );
		}


		for( int i=0;i<monoTonePolys.size();i++ )
		{
			monoPolygonTo2Darray( dir,curSeg,monoTonePolys[i]  );
		}


	}



}

void generateMapInNegY( int tid,int dir )
{

	Vector3 tp=targetPoints[tid]; // target er upor ekta point nilam
	vector< int >&obstacleId=dirObstacleId[dir]; // oi direction er obstacle er list ta nilam

	int curSeg=0;

	vector<int>obstacleWrtPoint; // ei khane thakbe amar kon kon obstacle nia deal korte hobe


	for(int i=0;i<obstacleId.size();i++)
	{
		int u=obstacleId[i]; // ekta obstacle nilam




		while( curSeg<NDISTESEG-1 && tp.y-segDist[curSeg]>=obstacles[ u ].y[ dir ] )
		{
			//cout<<curSeg<<endl;

			curSeg++;
			updateVcm( obstacleWrtPoint,dir,curSeg,tid ); // eikhane optimize korbo aro...


		//	Adapter.adapt( dir,tp.x+segDist[curSeg],gVcmProjPolys[dir][curSeg][tid],vcmProjPolys[dir][curSeg][tid] );

		}


		vector< vector<Vector3> > &p=vcmProjPolys[dir][curSeg][tid]; // ei segment er vcmProjection polygon er reference
		gpc_polygon &g=gVcmProjPolys[dir][curSeg][tid]; // seitar 2D version


		Box projPoly;
		//desired projection plane ta nilam
		Plane projPlane=Plane( unitDir[dir].x,unitDir[dir].y,unitDir[dir].z,segDist[curSeg] ); //change here for direction
		// sei plan er bounding rectangle ta nilam
		Box boundRect=projRect[dir][curSeg];

		/*

		if(!calcProjectionWrtPoint(projPlane, tp , obstacles[u] ,projPoly,dir))
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

		calcCovexProjectionWrtPoint( projPlane,tp,obstacles[u],gProjPoly,dir );

		if(Adapter.contains( g,gProjPoly ))
		{
			printf("discarding id %d view blocked by other obstacles\n",u);
			continue;
		}

		obstacleWrtPoint.push_back( u );
	}


	while( curSeg<NDISTESEG-2)
	{
		curSeg++;
		updateVcm( obstacleWrtPoint,dir,curSeg,tid ); // eikhane optimize korbo aro...


	//	Adapter.adapt( dir,tp.x+segDist[curSeg],gVcmProjPolys[dir][curSeg][tid],vcmProjPolys[dir][curSeg][tid] );

	}



	for( int curSeg=1;curSeg<=totSeg[dir];curSeg++ )
	{

		gpc_polygon &g=gVcmProjPolys[dir][curSeg][tid];
		Box prec=projRect[dir][curSeg];
		gpc_polygon gprec;

		Adapter.adapt(prec,gprec);
		Adapter.polygonIntersection(g,gprec);


		vector< vector<Vector> >vp;

		Adapter.adapt(g,vp); //vp contains all separate polygons

		vector< vector<Vector> >monoTonePolys;


		Adapter.adapt( dir,-segDist[curSeg],gVcmProjPolys[dir][curSeg][tid],vcmProjPolys[dir][curSeg][tid] );


		for( int i=0;i<vp.size();i++ )
		{
			//cout<<endl;
			reverse( vp[i].begin(),vp[i].end() );
			vector< vector<Vector> >temp=M.getMonotonePartition( vp[i] );
			for(int j=0;j<temp.size();j++)monoTonePolys.push_back( temp[j] );
		}


		for( int i=0;i<monoTonePolys.size();i++ )
		{
			monoPolygonTo2Darray( dir,curSeg,monoTonePolys[i]  );
		}


	}



}


void generateMapInNegZ( int tid,int dir )
{

	Vector3 tp=targetPoints[tid]; // target er upor ekta point nilam
	vector< int >&obstacleId=dirObstacleId[dir]; // oi direction er obstacle er list ta nilam

	int curSeg=0;

	vector<int>obstacleWrtPoint; // ei khane thakbe amar kon kon obstacle nia deal korte hobe


	for(int i=0;i<obstacleId.size();i++)
	{
		int u=obstacleId[i]; // ekta obstacle nilam




		while( curSeg<NDISTESEG-1 && tp.z-segDist[curSeg]>=obstacles[ u ].z[ dir ] )
		{
			//cout<<curSeg<<endl;

			curSeg++;
			updateVcm( obstacleWrtPoint,dir,curSeg,tid ); // eikhane optimize korbo aro...


		//	Adapter.adapt( dir,tp.x+segDist[curSeg],gVcmProjPolys[dir][curSeg][tid],vcmProjPolys[dir][curSeg][tid] );

		}


		vector< vector<Vector3> > &p=vcmProjPolys[dir][curSeg][tid]; // ei segment er vcmProjection polygon er reference
		gpc_polygon &g=gVcmProjPolys[dir][curSeg][tid]; // seitar 2D version


		Box projPoly;
		//desired projection plane ta nilam
		Plane projPlane=Plane( unitDir[dir].x,unitDir[dir].y,unitDir[dir].z,segDist[curSeg] ); //change here for direction
		// sei plan er bounding rectangle ta nilam
		Box boundRect=projRect[dir][curSeg];

		/*

		if(!calcProjectionWrtPoint(projPlane, tp , obstacles[u] ,projPoly,dir))
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

		calcCovexProjectionWrtPoint( projPlane,tp,obstacles[u],gProjPoly,dir );

		if(Adapter.contains( g,gProjPoly ))
		{
			printf("discarding id %d view blocked by other obstacles\n",u);
			continue;
		}

		obstacleWrtPoint.push_back( u );
	}


	while( curSeg<NDISTESEG-2)
	{
		curSeg++;
		updateVcm( obstacleWrtPoint,dir,curSeg,tid ); // eikhane optimize korbo aro...


	//	Adapter.adapt( dir,tp.x+segDist[curSeg],gVcmProjPolys[dir][curSeg][tid],vcmProjPolys[dir][curSeg][tid] );

	}



	for( int curSeg=1;curSeg<=totSeg[dir];curSeg++ )
	{

		gpc_polygon &g=gVcmProjPolys[dir][curSeg][tid];
		Box prec=projRect[dir][curSeg];
		gpc_polygon gprec;

		Adapter.adapt(prec,gprec);
		Adapter.polygonIntersection(g,gprec);


		vector< vector<Vector> >vp;

		Adapter.adapt(g,vp); //vp contains all separate polygons

		vector< vector<Vector> >monoTonePolys;


		Adapter.adapt( dir,-segDist[curSeg],gVcmProjPolys[dir][curSeg][tid],vcmProjPolys[dir][curSeg][tid] );

		for( int i=0;i<vp.size();i++ )
		{
			//cout<<endl;
			reverse( vp[i].begin(),vp[i].end() );
			vector< vector<Vector> >temp=M.getMonotonePartition( vp[i] );
			for(int j=0;j<temp.size();j++)monoTonePolys.push_back( temp[j] );
		}


		for( int i=0;i<monoTonePolys.size();i++ )
		{
			monoPolygonTo2Darray( dir,curSeg,monoTonePolys[i]  );
		}


	}



}


void generateMap(int tid,int dir)
{
	if(dir==0)generateMapInNegX(tid,dir);
	if(dir==1)generateMapInPosX(tid,dir);
	if(dir==2)generateMapInNegY(tid,dir);
	if(dir==3)generateMapInPosY(tid,dir);
	if(dir==4)generateMapInNegZ(tid,dir);
	if(dir==5)generateMapInPosZ(tid,dir);
}

void reduction(int dir)
{
	if(dir==1)reductionInPosX(1);
	if(dir==0)reductionInNegX(0);
	if(dir==3)reductionInPosY(3);
	if(dir==2)reductionInNegY(2);
	if(dir==5)reductionInPosZ(5);
	if(dir==4)reductionInNegZ(4);
}

void generateTargetPoints(int dir)
{
	targetPoints.clear();

	//(totp+1)*(totp+1) points

		double delta=target.y[1]-target.y[0];
		delta/=totp;


		for( double x=target.x[0];x<=target.x[1]  ;x+=delta )
		{
			for( double y=target.y[0];y<=target.y[1] ;y+=delta )
			{
				for( double z=target.z[0];z<=target.z[1]  ;z+=delta )
				{
					targetPoints.push_back(Vector3(x,y,z));
				}
			}
		}


}


RTreeCmdIntrpr *r;
void buildTree(char *path, char *DATAFILENAME, char *Outpath)
{

	char TREEFILE[200];
	strcpy(TREEFILE,path);
	strcat(TREEFILE,DATAFILENAME);
	strcat(TREEFILE,".tree");

	//input file

	char DATAFILE[200];
	strcpy(DATAFILE,path);
	strcat(DATAFILE,DATAFILENAME);
	strcat(DATAFILE,".txt");


	int blocksize = 1024;
	int b_length = 1024;


	int dimension=3;

	char OUTFILE[200]="";

	r=new RTreeCmdIntrpr();
	printf("%s",DATAFILE);
	r->build_tree(TREEFILE,DATAFILE,b_length,dimension,0);

	rtree=r->tree;




	//rtree->reductionInPosX(1,target);


	return;
}

void createRtreeFromData(  )
{
	char *Inpath = "E:\\Rakinsfiles\\rakin\'s\ L\-4\ T\-1\\Thesis\\code\\vcm\\";
	char *Outpath = "E:\\Rakinsfiles\\rakin\'s\ L\-4\ T\-1\\Thesis\\code\\vcm\\";

	//char *DATAFILENAME = "cityGml_british_3D_normalize";
	char *DATAFILENAME = "cityGml_british_3D_normalize10";

	//int NumOfObstacles = 3;

	buildTree(Inpath, DATAFILENAME, Outpath);

	return;
}

bool rayRectangleIntersction( Vector3 &u,Vector3 &v,Plane &p )
{
	Vector3 I;
	if(!p.rayPlaneIntersction( u,v,I ))return false;

	if( p.v0.x<=I.x && p.v1.x>=I.x )
	{
		if( p.v0.y<=I.y && p.v1.y>=I.y )
		{
			if( p.v0.z<=I.z && p.v1.z>=I.z )
			{
				return true;
			}
		}
	}

	return false;
}

bool rayBoxIntersection( Vector3 &u,Vector3 &v,Box &b )
{


	if( rayRectangleIntersction( u,v,b.px[0] ) )return true;
	if( rayRectangleIntersction( u,v,b.px[1] ) )return true;
	if( rayRectangleIntersction( u,v,b.py[0] ) )return true;
	if( rayRectangleIntersction( u,v,b.py[1] ) )return true;
	if( rayRectangleIntersction( u,v,b.pz[0] ) )return true;
	if( rayRectangleIntersction( u,v,b.pz[1] ) )return true;


	return false;
}

int tot;

int naiveVisibility( Vector3 v )
{
	int cnt=0;
	for(int i=0;i<targetPoints.size();i++)
	{
		Vector3 u=targetPoints[i];

		for(int j=0;j<obstacles.size();j++)
		{
			tot++;
			if( rayBoxIntersection( u,v,obstacles[j] ) )
			{
				cnt++;
				break;
			}
		}

	}

	return cnt;


}


FILE *ntfp=fopen("randtarget.txt","r");


struct triple
{
	int i,j,k;
	triple(){}
	triple(int _i,int _j,int _k){i=_i,j=_j,k=_k;}
};
vector<triple>errcell;


void genErrCell(int n)
{

	srand(time(NULL));
	/*
	for(int i=0;i<NDISTESEG;i++)
	{
		for(int j=0;j<MAXSEG;j++)
		{
			for(int k=0;k<MAXSEG;k++)
			{
				if( vcmArray[1][i][j][k] & rand()%3==0 )
				{
					errcell.resize( errcell.size()+1 );
					errcell.back(  )  = triple( i , j , k );
					if(errcell.size(  )==100)goto hell;
				}
			}
		}
	}
	*/
	for(int it=errcell.size();it<n;it++)
	{
		errcell.resize( errcell.size()+1 );
		errcell.back(  )  = triple( rand()%NDISTESEG , rand()%MAXSEG , rand()%MAXSEG );
		//cout<<errcell.back().i<<" "<<errcell.back().j<<" "<<errcell.back().k<<endl;
	}

	hell:;
}


void bruteForce()
{
	//return;
	//errcell.clear();

	genErrCell(100);

	int n;

	Vector3 c;
	
	c.fscan(ntfp);
	printf("target is centerd at %lf %lf %lf\n",c.x,c.y,c.z);
	

//	FILE* fp=fopen( "my0.txt","r" );
	//fscanf(fp,"%d",&n);

	//n=100;


	//obstacles=vector<Box>(n);

	Vector3 v0,v1;

	//obstacles.clear();


	n=dirObstacle[1].size();
	cout<<"total "<<n<<" obstacles"<<endl;

	for(int i=0;i<n;i++)
	{
//		v0.fscan(fp);
//		v1.fscan(fp);

//		v0=v0.sub(c);
//		v1=v1.sub(c);


		obstacles.resize( obstacles.size()+1 );

		obstacles.back()=dirObstacle[1][i];

	//	obstacles.back().init(v0,v1,Origin);
	//	obstacles.back().setPlanes();

		Box b;
		if(commonBox(target,obstacles.back(),b))obstacles.pop_back();


	}





	int dir=1;


	for( int it=0;it<errcell.size();it++ )
	{
		triple &tri=errcell[it];
		int i,j,k;
		i=tri.i;
		j=tri.j;
		k=tri.k;

		Vector3 v;
		v.x=segDist[i];

		double dim=getDim( dir,i );
		double delta= dim/MAXSEG;

		double lz=-dim/2.0;lz+=delta/2.0;
		double ly=-dim/2.0;ly+=delta/2.0;

		v.y=ly+k*delta;
		v.z=lz+j*delta;

		naiveVcmArray[dir][i][j][k]=naiveVisibility( v );
	}

	/*

	for(int i=0;i<NDISTESEG;i++)
	{
		printf("%d\n",i);

		Vector3 v;
		v.x=segDist[i];

		double dim=getDim( dir,i );
		double delta= dim/MAXSEG;

		double lz=-dim/2.0;lz+=delta/2.0;



		for(int j=0;j<MAXSEG;j++)
		{
			double ly=-dim/2.0;ly+=delta/2.0;
			for(int k=0;k<MAXSEG;k++)
			{

				v.y=ly;
				v.z=lz;


				naiveVcmArray[dir][i][j][k]=naiveVisibility( v );
				//vcmArray[dir][i][j][k]=naiveVisibility( v );



				//v.print();

				ly+=delta;
			}
			lz+=delta;
		}
	}



	*/



}



void initialize()
{
	//cout<<"1";

	int dir=1;

	segDist.clear();
	for(int i=0;i<6;i++)totSeg[i]=0;


	for(int i=0;i<NDISTESEG;i++)gpc_free_polygon(&gProjPolys[dir][i]);
	for(int i=0;i<NDISTESEG;i++)ProjPolys[dir][i].clear();

//	cout<<"sad";

	for(int i=0;i<NDISTESEG;i++)
	{
		for(int j=0;j<targetPoints.size();j++)gpc_free_polygon(&gVcmProjPolys[dir][i][j]);
	}//cout<<"4";

	//cout<<"das";

	for(int i=0;i<NDISTESEG;i++)
	{
		for(int j=0;j<targetPoints.size();j++)vcmProjPolys[dir][i][j].clear();
	}//cout<<"5";

	targetPoints.clear();

	dirObstacleId[dir].clear();
	dirObstacle[dir].clear();
	//cout<<"6";

	for(int i=0;i<8;i++)curProjPlane1[i]=0;

	for(int i=0;i<8;i++)gpc_free_polygon(&projWrtTargetCorner[ i ]);//cout<<"7";
	for(int i=0;i<6;i++)curProjPlane[i]=0;

	All.clear();
	dstFromTar.clear();
	rColor.clear();
	//cout<<"8";
	for( int i=0;i<NDISTESEG;i++ )
	{
		for(int j=0;j<MAXSEG;j++)
		{
			for(int k=0;k<MAXSEG;k++)
			{
				vcmArray[dir][i][j][k]=0;
				naiveVcmArray[dir][i][j][k]=0;
			}
		}
	}
	//cout<<"9";
	if(r)r->free_tree();
	//cout<<"10";

	errcell.clear();
	obstacles.clear();
	return;
}



void run()
{
	FILE *obfp=fopen( "rob.txt","r" );
	logfp=fopen( "log.txt","w+" );
	efile=fopen("efile.txt","w");

	//input();

	//
	//5985

	int testCase=10;
	double totNaiveTime=0,totalError=0;
	double startTime,timeneeded;

	//FILE *ofp=fopen("out.txt","w");

	//initialize();


	double obsz = 10;

	//createRtreeFromData();

	for(int it=1;it<=testCase;it++)
	{

		fprintf(logfp,"test %d:\n",it);
		printf("test %d:\n",it);



		int dim=60;
		dim/=2;

		Vector3 v0(-dim, -dim, -dim);
		Vector3 v1(+dim, +dim, +dim);


		target.init(v0,v1,Origin);
		target.setPlanes();

		calcSegmentDistance();
		//cout<<segDist[NDISTESEG-1]<<endl;while(1);
		generateRandCol();
		calcProjectionPlanes();


		int dir=1;
		generateTargetPoints(dir);

		target.print();

		
		clock_t startTime=clock();

		createRtreeFromData();


		double timeneeded=(double)(clock()-startTime)/(double)CLOCKS_PER_SEC;



		fprintf(logfp,"time needed  for building rtree %lf\n",timeneeded);
		printf("time needed  for building rtree %lf\n",timeneeded);





		startTime=clock();
		
		dirObstacle[1].clear();
		reductionInPosXRtree(1);
		timeneeded=(double)(clock()-startTime)/(double)CLOCKS_PER_SEC;


		fprintf(logfp,"time needed  for reduction %lf\n",timeneeded);
		printf("time needed  for reduction %lf\n",timeneeded);

		//break;
		
		fprintf(logfp,"total target points %d\n",targetPoints.size());
		printf("total target points %d\n",targetPoints.size());
		



		startTime=clock();
		/*
		int obs;
	//	fscanf(obfp,"%d",&obs);

	//	dirObstacle[1].resize( obs );

		//cout<<dirObstacle[1].size()<<endl;
		
//		fprintf(obfp,"%d\n",dirObstacle[1].size());	
	
		for(int i=0;i<dirObstacle[1].size();i++)
		{
			fscanf(obfp,"%lf %lf %lf",&dirObstacle[1][i].x[0],&dirObstacle[1][i].y[0],&dirObstacle[1][i].z[0]);	
			fscanf(obfp,"%lf %lf %lf",&dirObstacle[1][i].x[1],&dirObstacle[1][i].y[1],&dirObstacle[1][i].z[1]);	
		}

		printf("scaned %d objects\n",obs);
		*/
	//	continue;

		//fflush(obfp);
		fflush(logfp);
		
//		continue;

		for(int i=0;i<targetPoints.size();i++)
		{
		//	cout<<i<<" ";
			generateMap( i,dir );
		}//cout<<endl;

		timeneeded=(double)(clock()-startTime)/(double)CLOCKS_PER_SEC;
		fprintf(logfp,"time needed  for calculating vcm %lf\n",timeneeded);
		
		printf("time needed  for calculating vcm %lf\n",timeneeded);


		startTime=clock();

		for(int curSeg=1;curSeg<NDISTESEG;curSeg++)
		{
			mergeAll(dir,curSeg);
		}


		timeneeded=(double)(clock()-startTime)/(double)CLOCKS_PER_SEC;
		
		fprintf(logfp,"time needed  for merging vcm %lf\n",timeneeded);
		printf("time needed  for merging vcm %lf\n",timeneeded);

		
		fprintf(logfp,"naive approach:\n");
		printf("naive approach:\n");

		

		tot=0;

		startTime=clock();
		bruteForce();
		timeneeded=(double)(clock()-startTime)/(double)CLOCKS_PER_SEC;

		//cout<<errcell.size()<<"  "<<targetPoints.size()<<" "<<obstacles.size()<<" "<<tot<<endl;

		double naiveTime=(timeneeded*obsz/(double)errcell.size()*(double)MAXSEG*(double)MAXSEG*(double)NDISTESEG)/errcell.size(); 

		fprintf(logfp,"naive time %lf\n",naiveTime);
		printf("naive time %lf\n",naiveTime);


		double error=0;

		for( int i=0;i<errcell.size();i++ )
		{
			triple &tri= errcell[i] ;
			error+=abs( naiveVcmArray[dir][tri.i][tri.j][tri.k]-vcmArray[dir][tri.i][tri.j][tri.k] );
		
			//cout<<naiveVcmArray[dir][tri.i][tri.j][tri.k]<<" "<<vcmArray[dir][tri.i][tri.j][tri.k]<<" "<<targetPoints.size()<<endl;
		}

		error/=targetPoints.size();
		error/=errcell.size();

		fprintf( logfp,"total error %lf\n",error*100 );
		printf( "total error %lf\n",error*100 );
		
		fflush(logfp);
		initialize();
		
	}


	fflush(logfp);
	//cout<<"done"<<endl;

	//while(1);

	//printf("overall avg time %lf\n",megatime/10.0);



	//printf("total discarded %d   total reduced obstacle size %d   total time %lf\n",totDiscard,totCnt,totTime);
	//fprintf(logfp,"avg discarded %lf   avg reduced obstacle size %lf   avg time %lf\n",(double)totDiscard/(double)testCase,(double)totCnt/(double)testCase,totTime/(double)testCase);
	//fflush(logfp);
	
	//getchar();
	//getchar();



}


int main(void)
{
	
	run();//while(1);
	getchar();
	return 0;

    gpc_polygon subject, clip, result;
    FILE *sfp, *cfp, *ofp;
	
    sfp= fopen("subjfile.txt", "r");
    cfp= fopen("clipfile.txt", "r");
    gpc_read_polygon(sfp, 0, &subject);
    gpc_read_polygon(cfp, 0, &clip);
	
	ofp= fopen("outfile.txt", "w");

	for(int i=0;i<1000;i++)
	{
		gpc_polygon_clip(GPC_INT, &subject, &clip, &result);
		gpc_write_polygon(ofp, 0, &result);
		gpc_free_polygon(&result);
		
	}

/*    ofp= fopen("outfile.txt", "w");
    gpc_write_polygon(ofp, 0, &result);
	*/
    gpc_free_polygon(&subject);
    gpc_free_polygon(&clip);
    

    fclose(sfp);
    fclose(cfp);
   // fclose(ofp);
    return 0;
}

