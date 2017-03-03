#define ll long long
#define vi vector <int>
#define pii pair <int,int>
#define FOR(i, a, b) for (i = (a); i <= (b); i++)
#define REP(i, a) for (i = 0; i < (a); i++)
#define ALL(v) (v).begin(), (v).end()
#define SET(a, x) memset((a), (x), sizeof(a))
#define SZ(a) ((int)(a).size())
#define CL(a) ((a).clear())
#define SORT(x) sort(ALL(x))
#define mp make_pair
#define pb push_back
#define MAX(a,b) ((a)>(b)?(a):(b))
#define MIN(a,b) ((a)<(b)?(a):(b))

#define filer() freopen("in.txt","r",stdin)
#define filew() freopen("out.txt","w",stdout)


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



using namespace std;


#define INF 1e6
#define EPS 1e-10
// +0.0000000001
#define S(x)	((x)*(x))
#define Z(x)	(fabs(x) < EPS)

#define Coord double

struct Vector{
	Coord x,y;
	Vector();
	Vector(Coord x,Coord y);
    void scan();
	void print();
	bool isZero();
	Coord mag();
	Coord mag2();
	void normalize();
	Vector unit();
	bool equla( Vector &v);
};
bool operator<(Vector a,Vector b);
struct myPolygon
{
	vector< Vector >P;
	
	int size();
	void push( Vector v );
	void sortCCW();
	void print();
	void fprint( FILE *fp );
	void clear();
	bool onPolySegment(Vector &P0);
	bool inPoly( Vector &p );
	bool PointInPoly( Vector p );
	bool polyInter( myPolygon &poly);
};

int Turn(Vector &V0,Vector &V1,Vector &V2);
Coord dotp(Vector a, Vector b);
Coord crossp(Vector a, Vector b);
Vector operator+(Vector a,Vector b);
Vector operator-(Vector a);
Vector operator-(Vector a,Vector b);
Coord operator*(Vector a,Vector b);
Vector operator*(Vector a, Coord b);
Vector operator*(Coord b, Vector a);
Vector operator/(Vector a, Coord b);
Coord  angle(Vector a, Vector b);



//covex hull
bool cmp(Vector V1,Vector V2);
int cHull(int N);
vector<Vector> convexHull(vector<Vector>&P);


