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

template<class X>void debug(vector<X>&v){for(int i=0;i<v.size();i++)cout<<v[i]<<" ";cout<<endl;}

double fun(char ss[])
{
    string s,s1;

    s=string(ss);
    stringstream sin(s);

    while(sin>>s1);
    //cout<<s1<<endl;

    stringstream din(s1);
    double d;
    din>>d;
    return d;

}

int main()
{
    freopen("z20000.txt","r",stdin);
    char ss[100];
    double rtree=0,rob=0,io=0,reduction=0,calvcm=0,mvcm=0,ntime=0,error=0;
    int T=13;
    for(int i=0;i<T;i++)
    {
        gets(ss);//

        gets(ss);rtree+=fun(ss);cout<<ss<<endl;
        gets(ss);rob+=fun(ss);cout<<ss<<endl;
        gets(ss);io+=fun(ss);cout<<ss<<endl;
        gets(ss);reduction+=fun(ss);cout<<ss<<endl;



        gets(ss);//


        gets(ss);calvcm+=fun(ss);cout<<ss<<endl;
        gets(ss);mvcm+=fun(ss);cout<<ss<<endl;

        gets(ss);//

        gets(ss);ntime+=fun(ss);cout<<ss<<endl;
        gets(ss);error+=fun(ss);cout<<ss<<endl;





    }


    double tot=rtree+reduction+calvcm+mvcm;
    cout<<io/T<<" "<<rob/T<<" "<<reduction/T<<" "<<tot/T<<" "<<3*ntime/T<<" "<<error/T<<endl;


    return 0;
}
