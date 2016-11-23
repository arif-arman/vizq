#define CO(v) v.x,v.y,v.z
#define P(v)  {CO(v)}
#define SET_FRONT_COLOR glColor3ub(159,182,205);
#define SET_SIDE_COLOR glColor3ub(119,136,153);
#define SET_ROOF_COLOR glColor3ub(198,226,255);
#define SET_GLASS_COLOR glColor3ub(0,255,255);
#define SET_GOLD_FRONT glColor3ub(255,193,37);
#define SET_GOLD_SIDE glColor3ub(255,215,0);
#define SET_GOLD_ROOF glColor3ub(139,101,8);
#define SET_BLUE_FRONT glColor3ub(0,0,150);
#define SET_BLUE_SIDE glColor3ub(0,0,200);
#define SET_BLUE_ROOF glColor3ub(0,0,255);

#define RED 255,0,0
#define GREEN 0,255,0
#define BLUE 0,0,255
#define YELLOW 255,255,0

#define ESCAPE 27


#define EPS .000000001
#define Z(x) (fabs(x)<EPS)
#define INF 1000000