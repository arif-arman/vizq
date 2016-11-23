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

    freopen("my.txt","r",stdin);
    freopen("my3.txt","w",stdout);

    int n;
    scanf("%d",&n);
    //printf("%d\n",n);
    int cnt=0;
    double k[6];

    srand(time(NULL));

    for(int i=0;i<n;i++)
    {
        for(int j=0;j<6;j++)
        {
            scanf("%lf",&k[j]);

        }

        int r=rand()%10;
        if( r<4 )
        {
            cnt++;
            for(int j=0;j<6;j++)
            {
                if(j)printf(" ");
                printf("%lf",k[j]);
            }
            printf("\n");
        }
    }
    cout<<cnt<<endl;

    return 0;
}
