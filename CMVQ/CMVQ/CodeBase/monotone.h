#include "prevSource.h"
using namespace std;


/// monotone partitioning

class Vertex
{
public:
	Vector p;
	int id,t,h;
	Vertex();
	Vertex(int id,int t,int h);
};

bool vertexComp(Vertex a,Vertex b);




#define START_VERTEX 0
#define END_VERTEX 1
#define REGULAR_VERTEX 2
#define SPLIT_VERTEX 3
#define MERGE_VERTEX 4


class Edge
{
public:
	int id;
	Vector e0,e1;
	Edge();
	Edge(int id,Vector e0,Vector e1);
};


double xInterSection(Edge e);
struct EdgeComparator
{
	bool operator()(const Edge& e1,const Edge& e2)const;
};

class BTree
{

#define TYPE set<Edge,EdgeComparator>
#define TYPEIT TYPE::iterator
public:
	TYPE Tree;
	TYPEIT it;

	void add( Edge e );
	
	TYPEIT find(Edge e);

	void remove(TYPEIT it);

	void remove( Edge e );


	int findLeft( double x );

};




class MonotonDecomposer
{
public:
	int n;
	vector<Vertex>P; // 1-indexing
	vector<Edge>Diagonals;
	vector< vector<int> >Did;
	vector<Vertex>Event;
	vector<int>dNext,dPrev;
	BTree T;

	

	void clear();

	void input();

	void assignType();

	void assignHelper(int i,int h);

	

	void dInsert( int u,int v );

	void print( int u,int v );

	
	Edge assignEdge(Vertex u);

	void handleStartVertex(int i);

	void handleEndVertex( int i );

	void handleSplitVertex( int i );

	void handleMergeVertex(int i);

	void handleRegularVertex(int i);

	void addDiagonals();

	void run();

	vector< vector<Vector> > getMonotonePartition( vector< Vector >_P );


};



