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

int main()
{
    freopen("zipF5000.txt","r",stdin);



    int n,i,j;
    double a,b;
    scanf("%d",&n);
    double mx[]={-1,-1,-1};
    double mn[]={-1,-1,-1};
    int def[3];

    for(int i=0;i<n;i++)
    {
        scanf("%*d");
        for(int j=0;j<3;j++)
        {
            scanf("%lf%lf",&a,&b);
            if( mn[j]<0 || mn[j]>a )mn[j]=a;
            if( mx[j]<0 || mx[j]<b )mx[j]=b;
        }
    }
    for(int j=0;j<3;j++)
    {
        printf("%lf %lf\n",mn[j],mx[j]);
        def[j]=mx[j]-mn[j];
    }
















    return 0;
}
