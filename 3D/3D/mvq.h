#ifndef MVQ_H
#define MVQ_H

#include <math.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <limits>
#include <fstream>
#include <iostream>
#include <vector>
#include <list>
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
#include <map>

#include "./rtree/rtree.h"
#include "./rtree/rtnode.h"
#include "./rtree/entry.h"
#include "./blockfile/blk_file.h"
#include "./blockfile/cache.h"
#include "./linlist/linlist.h"
#include "./rtree/rtree_cmd.h"
#include "./rtree/distance.h"
#include "./his/histogram.h"
#include "./global.h"
#include "./func/gendef.h"

class MVQ {
public:
	MVQ();
	// uiu pc
	/*char *f = "F:\\3D\\3D\\db\\sample4";
	char *queryPath = "F:\\3D\\3D\\db\\";
	char *Outpath = "F:\\3D\\3D\\db\\";*/
	// home pc
	//char *f = "H:\\Study\\Others\\Visibility\ Query\\3D\\Executable\\3D\\3D\\db\\sample5";
	//char *queryPath = "H:\\Study\\Others\\Visibility\ Query\\3D\\Executable\\3D\\3D\\db\\";
	//char *Outpath = "H:\\Study\\Others\\Visibility\ Query\\3D\\Executable\\3D\\3D\\db\\";
	// for executable
	char f[200], queryPath[200], Outpath[200];
	//char *f = "H:\\Study\\Others\\Visibility\ Query\\3D\\Executable\\3D\\3D\\db\\";
	//char *queryPath = "H:\\Study\\Others\\Visibility\ Query\\3D\\Executable\\3D\\3D\\db\\";
	//char *Outpath = "H:\\Study\\Others\\Visibility\ Query\\3D\\Executable\\3D\\3D\\db\\";
	//char *DATAFILENAME = "sample3";
	Box2 T;
	//int NumOfObstacles = 5;
	int numOfQueryPoints = 0;

	bool dbLoaded = false;
	bool qpLoaded = false;

	int vr = 0;

	bool showObstacles = true;

	bool isTargetSet = false, mvqGenerated = false;
	static map< int, double > dstFromTar;

	vector<Point3D> rColor;

	vector<Box2> obstacles;
	vector<QueryPoint3D> queryPoints;
	vector<QueryPoint3D> k_ans;
	RTreeCmdIntrpr *r;
	RTree *rtree;

	bool Mov_exp3D(int upperk);
	void debug();
	void setup();
	void loadQueryPoints();
	void clearTarget();
	void generateRandCol();
	void createRtreeFromData(char *f);
	void buildTree(char *path, char *DATAFILENAME, char *Outpath);
	void draw_axis();
	static bool comparatorForPq(int &a, int &b)
	{
		return dstFromTar[a]>dstFromTar[b];
	}
};

#endif