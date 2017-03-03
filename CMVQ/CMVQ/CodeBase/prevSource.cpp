
#include "prevSource.h"



Vector :: Vector(){}

 Vector:: Vector(Coord x,Coord y){
	this->x=x;
	this->y=y;
}

void  Vector:: scan()
{
    cin>>x>>y;
}
void  Vector:: print()
{
    cout<<x<<" "<<y<<endl;
}
bool  Vector:: isZero(){
	return Z(x) && Z(y);
}
Coord  Vector:: mag(){
	return sqrt(mag2());
}
Coord  Vector:: mag2(){
	return S(x)+S(y);
}
void  Vector:: normalize(){ 
	Coord m = mag();
	x/=m;
	y/=m;
}
Vector  Vector:: unit(){	
	Vector u(x,y);
	u.normalize();
	return u;
}
bool Vector:: equla( Vector &v){ return Z( x-v.x ) && Z( y-v.y ); }


bool operator<(Vector a,Vector b) // this is for priority queue
{
	//cout<<"asd";
	return a.y<b.y;
}

Coord dotp(Vector a, Vector b){
	return a.x*b.x + a.y*b.y ;
}
Coord crossp(Vector a, Vector b){
	return a.x*b.y - a.y*b.x;
}

Vector operator+(Vector a,Vector b){
	return Vector(a.x+b.x, a.y+b.y);	}

Vector operator-(Vector a){
	return Vector(-a.x, -a.y);	}

Vector operator-(Vector a,Vector b){
	return Vector(a.x-b.x, a.y-b.y);	}

Coord operator*(Vector a,Vector b){
	return crossp(a,b);	}

Vector operator*(Vector a, Coord b){
	return Vector(a.x*b, a.y*b);	}

Vector operator*(Coord b, Vector a){
	return Vector(a.x*b, a.y*b);	}

Vector operator/(Vector a, Coord b){
	return Vector(a.x/b, a.y/b);	}


Coord  angle(Vector a, Vector b){
	//if(fabs(a.mag2())<EPS||fabs(b.mag2())<EPS)return 0;
	Coord v= dotp(a,b) / (a.mag()*b.mag()) ;
	v=min(v,(Coord)1);
	v=max(v,(Coord)-1);
	return acos(v);
}


int Turn(Vector &V0,Vector &V1,Vector &V2)
{
    Coord v=(V1-V0)*(V2-V0);
    if(fabs(v)<EPS)return 0;
    if(v>0.0)return 1;
    return -1;
}




class LineSegment
{
public:
    Vector v0;
    Vector v1;

	LineSegment(){}
    LineSegment(const Vector& begin, const Vector& end)
        : v0(begin), v1(end) {}

    enum IntersectResult { PARALLEL, COINCIDENT, NOT_INTERESECTING, INTERESECTING };

    IntersectResult Intersect(const LineSegment& L, Vector& I,bool f=0)
    {
        double denom = ((L.v1.y - L.v0.y)*(v1.x - v0.x)) -
                      ((L.v1.x - L.v0.x)*(v1.y - v0.y));

        double nume_a = ((L.v1.x - L.v0.x)*(v0.y - L.v0.y)) -
                       ((L.v1.y - L.v0.y)*(v0.x - L.v0.x));

        double nume_b = ((v1.x - v0.x)*(v0.y - L.v0.y)) -
                       ((v1.y - v0.y)*(v0.x - L.v0.x));

        if(denom == 0.0f)
        {
            if(nume_a == 0.0f && nume_b == 0.0f)
            {
                return COINCIDENT;
            }
            return PARALLEL;
        }

        double ua = nume_a / denom;
        double ub = nume_b / denom;

        if(ua >= 0.0f && ua <= 1.0f && ub >= 0.0f && ub <= 1.0f)
        {
            // Get the intersection point.
            I.x = v0.x + ua*(v1.x - v0.x);
            I.y = v0.y + ua*(v1.y - v0.y);

            return INTERESECTING;
        }



		if( f )
		{
			if(ua >= 0.0f && ub >= 0.0f)
			{
				I.x = v0.x + ua*(v1.x - v0.x);
				I.y = v0.y + ua*(v1.y - v0.y);

				return INTERESECTING;
			}
		}

        return NOT_INTERESECTING;
    }
};


vector< Vector >P;
bool cmpCCW(Vector V1,Vector V2)
{
	int t=Turn(P[0],V1,V2);
	if(!t)
	{
		return (V1-P[0]).mag2()<(V2-P[0]).mag2();
	}
	return t>0;
}


void myPolygon:: sortCCW()
{

	int i,id=0,N=SZ(P);
		
	for(i=1;i<N;i++)
	{
		if(P[i].y==P[id].y)
		{
			if(P[i].x<P[id].x)id=i;
		}
		if(P[i].y<P[id].y)id=i;
	}

	swap(P[0],P[id]);

	::P=P;

	sort(ALL(P),cmpCCW);
}


void myPolygon:: print()
{
	int i;

	REP( i,SZ( P ) )
	{
		P[i].print();
	}

}


void myPolygon:: fprint( FILE *fp )
{
	fprintf( fp,"1\n" );
	fprintf(fp,"%d\n",SZ( P ));
		
	int i;

	REP(i,SZ( P ))
	{
		fprintf(fp,"%lf %lf\n",P[ i ].x,P[i].y);
	}

}


void myPolygon::  clear(){CL(P);}


bool myPolygon:: onPolySegment(Vector &P0)
{
	int i,n=SZ( P );
	REP(i,n)
	{
		int j=(i+1)%n;
		if(!Turn(P[i],P[j],P0))//crossp is 0
		{
			if((P[i].x-P0.x)*(P[j].x-P0.x)<=0&&(P[i].y-P0.y)*(P[j].y-P0.y)<=0)return true;//P0 is between two points
		}
	}
	return false;
}

bool myPolygon:: inPoly( Vector &p )
{
	LineSegment L1,L2;
	L1.v0=p;
	L1.v1.x=10007;L1.v1.y=p.y;
	int cn = 0,i,j,n=SZ(P);    // the crossing number counter
	REP(i,n) {    // edge from V[i] to V[i+1]
		j=(i+1)%n;
		if (((P[i].y <= p.y) && (P[j].y > p.y))    // an upward crossing
		|| ((P[i].y > p.y) && (P[j].y <= p.y))) { // a downward crossing
			L2=LineSegment(P[i],P[j]);
			Vector I;
			if(L1.Intersect(L2,I)==3)++cn;
		}
	}

	return (cn&1);
}

bool myPolygon:: PointInPoly( Vector p )
{
	if( onPolySegment(p) )return true;
	if( inPoly(p) )return true;
	return false;
}

bool myPolygon:: polyInter( myPolygon &poly)
{
	
	int N=SZ(poly.P);
	int M=SZ(P);
			

	int i;

	REP(i,M)
	{
		if( poly.PointInPoly( P[i]  ) )return true;
			
	}

	return false;
}




Vector V[50007];
int hull[50007];
bool cmp(Vector V1,Vector V2)
{
    int t=Turn(V[0],V1,V2);
    if(!t)
    {
        return (V1-V[0]).mag2()<(V2-V[0]).mag2();
    }
    return t>0;
}

int cHull(int N)
{
    int i,id=0;
    int top=0;
    if(N<=2)
    {
        hull[top++]=0;
        if(N==2)hull[top++]=1;
        return top;
    }
    for(i=1;i<N;i++)
    {
        if(V[i].y==V[id].y)
        {
            if(V[i].x<V[id].x)id=i;
        }
        if(V[i].y<V[id].y)id=i;
    }
    swap(V[0],V[id]);
    sort(V+1,V+N,cmp);
    hull[top++]=0;
    hull[top++]=1;
    for(i=2;i<N;i++)
    {
        while(Turn(V[hull[top-2]],V[hull[top-1]],V[i])<1&&top>1)top--;
        hull[top++]=i;
    }
    return top;
}


vector<Vector> convexHull(vector<Vector>&P)
{
	if( P.size()>50004 )
	{
		cout<<"increase the hull array size";
		exit(0);
	}


	for(int i=0;i<P.size();i++)V[i]=P[i];
	int top=cHull( P.size() );

	hull[top]=hull[0];

	vector< Vector > Q;

	for(int i=0;i<top;i++)Q.push_back( V[hull[i]] );

	return Q;



}