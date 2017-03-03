
#include "monotone.h"


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


double sweep;
/// monotone partitioning

Vertex::Vertex(){}
Vertex::Vertex(int id,int t,int h)
{
		
    this->id=id;
	this->t=t;
	this->h=h;
}


bool vertexComp(Vertex a,Vertex b)
{
	if(Z(a.p.y-b.p.y))return (a.p.x<b.p.x);
	return (a.p.y>b.p.y);
}




Edge::Edge(){}
Edge::Edge(int id,Vector e0,Vector e1)
{
    this->id=id;
	this->e0=e0;
	this->e1=e1;
}


double xInterSection(Edge e)
{
	double dy=e.e1.y-e.e0.y;
	double dx=(e.e1.x-e.e0.x);
    if(Z(dy))return e.e0.x;
    return e.e0.x+dx*(sweep-e.e0.y)/dy;
}


bool EdgeComparator:: operator()(const Edge& e1,const Edge& e2)const
{
    if(xInterSection(e1)<xInterSection(e2))return true;
    else return false;
}


void BTree::add( Edge e )
{
	Tree.insert( e );
}
	
TYPEIT BTree::find(Edge e)
{
	return Tree.find(e);
}

void BTree::remove(TYPEIT it)
{
	Tree.erase(it);
}

void BTree::remove( Edge e )
{
	TYPEIT it=find(e);
	if(it!=Tree.end())Tree.erase( e );
}

int BTree::findLeft( double x )
{
	Edge e=Edge(-1,Vector(x,+INF),Vector(x,-INF));
	add(e);
	
	it=find(e);
	
	if(it==Tree.begin())
	{
		return -1;
	}

	--it;
	int left=it->id;
	remove(e);
	
	return left;
}




void MonotonDecomposer::clear()
{
	P.clear();
	Diagonals.clear();
	Did.clear();
	Event.clear();
	dNext.clear();
	dPrev.clear();
	T.Tree.clear();
}

void MonotonDecomposer::input()
{
	clear();
	//freopen("in.txt","r",stdin);
	scanf("%d",&n);

	P=vector<Vertex>(n+2);
	Did=vector< vector<int> >(n+2);

	for(int i=1;i<=n;i++)
	{
		P[i].p.scan();
		P[i].id=i;
		P[i].t=REGULAR_VERTEX;
		P[i].h=-1;
		Event.push_back( P[i] );
	}

	P[0]=P[n];
	P[n+1]=P[1];

	sort(ALL(Event),vertexComp);

	dNext=dPrev=vector<int>( n+2 );

	for(int i=1;i<=n;i++)
	{
		if(i!=n)dNext[ i ]=i+1;
		else dNext[ i ]=1;
		if(i!=1)dPrev[ i ]=i-1;
		else dPrev[ i ]=n;
	}

}

void MonotonDecomposer::assignType()
{
	for(int i=1;i<=n;i++)
	{
		if(!vertexComp(P[i-1],P[i])&&!vertexComp(P[i+1],P[i]))
		{
			if(Turn(P[i-1].p,P[i].p,P[i+1].p)==1)P[i].t=START_VERTEX;
			else P[i].t=SPLIT_VERTEX;
		}
		else if(vertexComp(P[i-1],P[i])&&vertexComp(P[i+1],P[i]))
		{
			if(Turn(P[i-1].p,P[i].p,P[i+1].p)==1)P[i].t=END_VERTEX;
			else P[i].t=MERGE_VERTEX;
		}
	}
	P[0]=P[n];
	P[n+1]=P[1];
	return;
}

void MonotonDecomposer::assignHelper(int i,int h)
{
	if(i==0||i==n)
	{
		P[0].h=h;
		P[n].h=h;
		return;
	}
	if(i==1||i==n+1)
	{
		P[1].h=h;
		P[n+1].h=h;
		return;
	}
	P[i].h=h;
	return;
}

void MonotonDecomposer::dInsert( int u,int v ) // insert edge between u,v u is one with larger x
{

		
	if( u<v )swap( u,v );
	Did[u].push_back(v);

	return;
		
	dNext[u]=v;
	dPrev[v]=u;



}

void MonotonDecomposer::print( int u,int v ) // print the chain from u to v
{return;
	////printf("%d ",u);
		
	if( u==v )
	{
		return;
		//printf("%d ",u);
		u=dNext[u];
		while( u!=v )
		{
			//printf("%d ",u);
			u=dNext[u];
		}
		return;
	}

//		if( u>v )swap( u,v );

	while(u!=v)
	{
		//printf("%d ",u);
		u=dNext[u];
	}
	//printf("%d",u);
	//printf("\n");
	//while(1);
}
	
Edge MonotonDecomposer::assignEdge(Vertex u)
{
	Vertex v;
	v=P[u.id+1];
	Edge e;
	if(vertexComp(u,v))e=Edge(u.id,u.p,v.p);
	else e=Edge(u.id,v.p,u.p);

	return e;
}

void MonotonDecomposer::handleStartVertex(int i)
{
	sweep=P[i].p.y;
	T.add(assignEdge(P[i]));
	assignHelper(i,i);
}

void MonotonDecomposer::handleEndVertex( int i )
{
	if(P[P[i-1].h].t==MERGE_VERTEX)
	{
		Diagonals.push_back(Edge(-1,P[i].p,P[P[i-1].h].p));
			
			
		print( P[i-1].h,i );
		dInsert( P[i-1].h,i );

	}
	sweep=P[i].p.y;

	T.remove(Edge(-1,Vector(P[i].p.x,+INF),Vector(P[i].p.x,-INF)));
}

void MonotonDecomposer::handleSplitVertex( int i )
{
	sweep=P[i].p.y;
	int left=T.findLeft(P[i].p.x);

	if(left!=-1)
	{
		Diagonals.push_back(Edge(-1,P[i].p,P[P[left].h].p));

		
		print( P[left].h,i );
		dInsert( P[left].h,i );

		assignHelper(left,i);
	}
	sweep=P[i].p.y;
	T.add(assignEdge(P[i]));
	assignHelper(i,i);
}

void MonotonDecomposer::handleMergeVertex(int i)
{
	if(P[P[i-1].h].t==MERGE_VERTEX)
	{
		Diagonals.push_back(Edge(-1,P[i].p,P[P[i-1].h].p));

			
		print( P[i-1].h,i );
		dInsert( P[i-1].h,i );
	}
	sweep=P[i].p.y;
	T.remove(Edge(-1,Vector(P[i].p.x,+INF),Vector(P[i].p.x,-INF)));
	sweep=P[i].p.y;
	int left=T.findLeft(P[i].p.x);
	if(left!=-1 && P[P[left].h].t==MERGE_VERTEX)
	{
		Diagonals.push_back(Edge(-1,P[i].p,P[P[left].h].p));

			
		print( P[left].h,i );
		dInsert( P[left].h,i );
	}
	if(left!=-1)assignHelper(left,i);
}

void MonotonDecomposer::handleRegularVertex(int i)
{
	if(vertexComp(P[i],P[i+1]))
	{
		if(P[P[i-1].h].t==MERGE_VERTEX)
		{
			Diagonals.push_back(Edge(-1,P[i].p,P[P[i-1].h].p));
				
				
			print( P[i-1].h,i );
			dInsert( P[i-1].h,i );
		}
		sweep=P[i].p.y;
			
		T.remove(Edge(-1,Vector(P[i].p.x,+INF),Vector(P[i].p.x,-INF)));
		T.add(assignEdge(P[i]));
			
		assignHelper(i,i);
	}
	else
	{
		sweep=P[i].p.y;
		int left=T.findLeft(P[i].p.x);
		if(left!=-1 && P[P[left].h].t==MERGE_VERTEX)
		{
			Diagonals.push_back(Edge(-1,P[i].p,P[P[left].h].p));

				
			print( P[left].h,i );
			dInsert( P[left].h,i );
		}
		if(left!=-1)assignHelper(left,i);
	}
}

void MonotonDecomposer::addDiagonals()
{
	for( int i=0;i<Event.size();i++ )
	{
		int id=Event[i].id;
		switch (P[id].t)
		{
		case START_VERTEX :
			handleStartVertex(id);
			break;
		case END_VERTEX :
			handleEndVertex(id);
			break;
		case REGULAR_VERTEX :
			handleRegularVertex(id);
			break;
		case SPLIT_VERTEX :
			handleSplitVertex(id);
			break;
		case MERGE_VERTEX :
			handleMergeVertex(id);
			break;
		default:
			break;
		}
	}

	print( Event.back().id,Event.back().id );
}

void MonotonDecomposer::run()
{
	input();
	assignType();
	addDiagonals();
}
//static int cnt=0;
vector< vector<Vector> > MonotonDecomposer::getMonotonePartition( vector< Vector >_P )
{
//	cout<<"asd\n";
	//cout<<"in"<<endl;
	//cnt++;
	//cout<<cnt<<endl;
	clear();

	int j=0;
	for(int i=0;i<_P.size();i++)
	{
		if(i && fabs(_P[i].x-_P[i-1].x)<.0001 && fabs(_P[i].y-_P[i-1].y)<.0001)continue;
		_P[j++]=_P[i];
	}

	_P.resize(j);
	if(_P.size()<3)
	{
		return vector< vector<Vector> >(0);
	}

	for( int i=0;i<_P.size();i++ )
	{
			_P[i].x+=(rand()%100)*.00001;
			_P[i].y+=(rand()%100)*.00001;
			//printf("%.10lf %.10lf\n",_P[i].x,_P[i].y);
	}

	n=_P.size();
	P=vector<Vertex>(n+2);
	Did=vector< vector<int> >(n+2);

	for(int i=1;i<=n;i++)
	{

		P[i].p=_P[i-1];
		//P[i].p.print();
		P[i].id=i;
		P[i].t=REGULAR_VERTEX;
		P[i].h=-1;
		Event.push_back( P[i] );
	}
//	while(1);
	P[0]=P[n];
	P[n+1]=P[1];

	sort(ALL(Event),vertexComp);

	dNext=dPrev=vector<int>( n+2 );

	for(int i=1;i<=n;i++)
	{
		if(i!=n)dNext[ i ]=i+1;
		else dNext[ i ]=1;
		if(i!=1)dPrev[ i ]=i-1;
		else dPrev[ i ]=n;
	}

	//cout<<"asd"<<endl;
	assignType();
	addDiagonals();

	


	//cout<<"out"<<endl;
	//cout<<"asd"<<endl;


	for( int i=1;i<=n;i++ )SORT( Did[i] );



	//cout<<"in.."<<endl;

	vector< vector<Vector> >ret;

	//return ret;

	set<int>s;
	for(int i=1;i<=n;i++)s.insert(i);

	for(int i=1;i<=n;i++ )
	{
		for(int k=0;k<Did[i].size();k++ )
		{


				
			vector<Vector>cur;

			int j=Did[i][k];

				
			{
		//		//printf( "%d ",j );

				cur.push_back( P[j].p ) ;

				set<int>::iterator it = s.find( j );

				if(it==s.end())
				{
					return vector< vector<Vector> >(0);
					cout<<"error in monotone partitioning.."<<endl;
					continue;
				}
				it++;


				vi v;
				while(*it!=i)
				{
					cur.push_back( P[*it].p ) ;
			//		//printf("%d ",*it);
					v.pb( *it );
					it++;
				}

				cur.push_back( P[i].p ) ;
		//		//printf( "%d\n",i );


				for(k=0;k<v.size();k++)s.erase( s.find(v[k]) );

			}


			ret.push_back( cur );
		}


	}


	vector<Vector>cur;
	for( set<int>::iterator it = s.begin(  );it!=s.end();it++  )
	{
		cur.push_back( P[*it].p );
	//	//cout<<*it<<" ";
	}////cout<<endl;


	ret.push_back( cur );
	//cout<<"asd"<<endl;

	//cout<<"out.."<<endl;
	return ret;
}




