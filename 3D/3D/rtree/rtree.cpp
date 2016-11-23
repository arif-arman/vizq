/*rtree.cpp
  this file implements the RTree class*/
#include <math.h>
#include <string.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <limits>
#include "rtree.h"
#include "entry.h"
#include "rtnode.h"
#include "distance.h"
#include "../blockfile/cache.h"
#include "../blockfile/blk_file.h"
#include "../linlist/linlist.h"
#include <array>
#include <list>
#include <math.h>
#include <queue>
#include <vector>
#include <map>


//Added by Tanzima
extern int io_access;
extern int disktime;
extern int updatetime;
extern int counttime;
extern int kmintime;
//......................
//------------------------------------------------------------
RTree::RTree(char *fname, int _b_length, Cache *c, int _dimension)
  //use this constructor to build a new tree
{
    file = new BlockFile(fname, _b_length);
    cache = c;

    re_data_cands = new LinList();
	deletelist = new LinList();

    dimension = _dimension;
    root = 0;
    root_ptr = NULL;
    root_is_data = TRUE;
    num_of_data = num_of_inodes = num_of_dnodes = 0;

    root_ptr = new RTNode(this);
	  //note that when a tree is constructed, the root is automatically created
	  //though at this time there is no entry in the root yet.
    num_of_dnodes++;
    root_ptr -> level = 0;
    root = root_ptr -> block;

	//added for TP KNN----------------------------------------
	tpheap=NULL;
}
//------------------------------------------------------------
RTree::RTree(char *fname, Cache *c)
  //use this constructor to restore a tree from a file
{
    file = new BlockFile(fname, 0);
    cache =c;

    re_data_cands = new LinList();
	deletelist = new LinList();

    char *header = new char [file->get_blocklength()];
    file -> read_header(header);
    read_header(header);
	delete [] header;

    root_ptr = NULL;

	//added for TP KNN----------------------------------------
	tpheap=NULL;
}
//------------------------------------------------------------
FILE *tfp = NULL;
RTree::RTree(char *inpname, char *fname, int _b_length, Cache *c, int _dimension)
// construct new R-tree from a specified input textfile with rectangles
{
	Entry *d;
	FILE *fp;
	file = new BlockFile(fname, _b_length);
	cache = c;

	re_data_cands = new LinList();
	deletelist = new LinList();

	char *header = new char[file->get_blocklength()];
	read_header(header);
	delete[] header;

	dimension = _dimension;
	root = 0;
	root_ptr = NULL;
	root_is_data = TRUE;
	num_of_data = num_of_inodes = num_of_dnodes = 0;

	root_ptr = new RTNode(this);
	num_of_dnodes++;
	root_ptr->level = 0;
	root = root_ptr->block;


	int record_count = 0;


	double cx, cy, cz;//target center



	if (tfp == NULL)
	{
		//printf("Can not find the target file\n");
	}
	else
	{
		fscanf(tfp, "%lf%lf%lf", &cx, &cy, &cz);
		//printf("target is centerd at %lf %lf %lf\n",cx,cy,cz);
	}




	double minx, miny, minz, maxx, maxy, maxz;
	maxx = maxy = maxz = -1000000;
	minx = miny = minz = +1000000;

	if ((fp = fopen(inpname, "r")) == NULL)
	{
		delete this;
		error("Cannot open R-Tree text file", TRUE);
	}
	else
	{
		int n; fscanf(fp, "%d", &n);
		for (record_count = 1; record_count <= n; record_count++)
		{


			d = new Entry(dimension, NULL);

			d->son = record_count;
			// bounces 0 - x1, bounces 1 - x2
			for (int i = 0; i<3; i++)fscanf(fp, "%f", &d->bounces[2 * i]);
			for (int i = 0; i<3; i++)fscanf(fp, "%f", &d->bounces[2 * i + 1]);

			//printf("\n%d->",record_count);
			//for(int i=0;i<3;i++)printf("%f ",d->bounces[i*2]);

			minx = min(minx, (double)d->bounces[0]); maxx = max(maxx, (double)d->bounces[0]);
			minx = min(minx, (double)d->bounces[1]); maxx = max(maxx, (double)d->bounces[1]);

			miny = min(miny, (double)d->bounces[2]); maxy = max(maxy, (double)d->bounces[2]);
			miny = min(miny, (double)d->bounces[3]); maxy = max(maxy, (double)d->bounces[3]);

			minz = min(minz, (double)d->bounces[4]); maxz = max(maxz, (double)d->bounces[4]);
			minz = min(minz, (double)d->bounces[5]); maxz = max(maxz, (double)d->bounces[5]);

			/*
			d->bounces[0]-=cx;
			d->bounces[1]-=cx;
			d->bounces[2]-=cy;
			d->bounces[3]-=cy;
			d->bounces[4]-=cz;
			d->bounces[5]-=cz;
			*/

			/*
			fscanf(fp, "%d", &(d -> son));
			//	fscanf(fp, " %f %f %f %f", &(d->bounces[0]),
			//	 	&(d->bounces[1]), &(d->bounces[2]), &(d->bounces[3]));
			//printf("ID=%d ",d->son);
			for (int i = 0; i < 2 * dimension; i ++)
			{
			fscanf(fp, " %f", &(d -> bounces[i]));
			//fscanf(fp, " %f %f %f %f", &(d->bounces[0]),
			//&(d->bounces[1]), &(d->bounces[2]), &(d->bounces[3]));
			}

			*/

			//fscanf(fp, "\n");

			//if(record_count==20000 || record_count==20001)	printf(": %f %f %f %f\n", d->bounces[0], d->bounces[2], d->bounces[1], d->bounces[3]);
			insert(d);

			//d will be deleted in insert()

			if (record_count % 100 == 0)
			{
				for (int i = 0; i < 79; i++); //clear a line
					//printf("\b");

				//printf("inserting object %d", record_count);
			}
		}
	}
	//TANZIMA
	//printf("\inserted object %d", record_count - 1);


	//cout<<endl<<minx<<" "<<maxx<<" "<<miny<<" "<<maxy<<" "<<minz<<" "<<maxz<<endl;
	//while(1);
	fclose(fp);

	//printf("\n");
	delete root_ptr;
	root_ptr = NULL;
}
//------------------------------------------------------------
RTree::~RTree()
{
	char *header = new char[file -> get_blocklength()];
    write_header(header);
    file->set_header(header);
    delete [] header;

    if (root_ptr != NULL)
    {
        delete root_ptr;
        root_ptr = NULL;
    }

	if (cache)
      cache -> flush();

    delete file;

    delete re_data_cands;
	delete deletelist;

    //printf("This R-Tree contains %d internal, %d data nodes and %d data\n",
	//   num_of_inodes, num_of_dnodes, num_of_data);
}
//------------------------------------------------------------
void RTree::del_root()
{
	delete root_ptr;
	root_ptr = NULL;
}
//------------------------------------------------------------
bool RTree::delete_entry(Entry *d)
{
	load_root();

	R_DELETE del_ret;
	del_ret=root_ptr->delete_entry(d);

	if (del_ret == NOTFOUND) return false;
	if (del_ret == ERASED) 
		error("RTree::delete_entry--The root has been deleted\n",true);
 
	if (root_ptr -> level > 0 && root_ptr -> num_entries == 1)
		//there is only one entry in the root but the root
		//is not leaf.  in this case, the child of the root is exhalted to root
	{
		root = root_ptr -> entries[0].son;
		delete root_ptr;
		root_ptr = NULL;
		load_root();
		num_of_inodes--;
	}

	//Now will reinsert the entries
	while (deletelist -> get_num() > 0)
	{
		Linkable *e;
		e = deletelist -> get_first();
		Entry *new_e = new Entry(dimension, NULL);
		new_e -> set_from_Linkable(e);
		deletelist -> erase();
		insert(new_e);
	}

	delete root_ptr;
	root_ptr = NULL;

	return true;
}
//------------------------------------------------------------
void RTree::insert(Entry* d)
{
    int i, j;
	RTNode *sn = NULL;;
    RTNode *nroot_ptr;
    int nroot;
    Entry *de;
    R_OVERFLOW split_root;
    Entry *dc;
    float *nmbr;

    // load root into memory
    load_root();

    // no overflow occured until now
    re_level = new bool[root_ptr -> level + 1];
    for (i = 0; i <= root_ptr -> level; i++)
        re_level[i] = FALSE;

    // insert d into re_data_cands as the first entry to insert
    // make a copy of d because it should be erased later
    Linkable *new_link;
	new_link = d -> gen_Linkable();
	re_data_cands -> insert(new_link);

	delete d;  //we follow the convention that the entry will be deleted when insertion finishes

    j = -1;
    while (re_data_cands -> get_num() > 0)
    {
        // first try to insert data, then directory entries
	    Linkable *d_cand;
		d_cand = re_data_cands -> get_first();
        if (d_cand != NULL)
        {
            // since "erase" deletes the data itself from the
            // list, we should make a copy of the data before
            // erasing it
			dc = new Entry(dimension, NULL);
            dc -> set_from_Linkable(d_cand);
            re_data_cands -> erase();

            // start recursive insert with root
			split_root = root_ptr -> insert(dc, &sn);
        }
        else
	        error("RTree::insert: inconsistent list re_data_cands", TRUE);

    	if (split_root == SPLIT)
    	// insert has lead to split --> new root-page with two sons (i.e. root and sn)
    	{
    	    nroot_ptr = new RTNode(this);
    	    nroot_ptr -> level = root_ptr -> level + 1;
    	    num_of_inodes++;
    	    nroot = nroot_ptr -> block;

    	    de = new Entry(dimension, this);
    	    nmbr = root_ptr -> get_mbr();
    	    memcpy(de->bounces, nmbr, 2*dimension*sizeof(float));
    	    delete [] nmbr;
    	    de->son = root_ptr->block;
    	    de->son_ptr = root_ptr;
    	    nroot_ptr -> enter(de);

    	    de = new Entry(dimension, this);
    	    nmbr = sn -> get_mbr();
    	    memcpy(de -> bounces, nmbr, 2*dimension*sizeof(float));
    	    delete [] nmbr;
    	    de -> son = sn -> block;
    	    de -> son_ptr = sn;
    	    nroot_ptr->enter(de);

    	    root = nroot;
            root_ptr = nroot_ptr;

            root_is_data = FALSE;
        }
        j++;
    }

    num_of_data++;

    delete [] re_level;

	delete root_ptr;
	root_ptr = NULL;
}
//------------------------------------------------------------
void RTree::load_root()
{
    if (root_ptr == NULL)
        root_ptr = new RTNode(this, root);
}
//------------------------------------------------------------
void RTree::rangeQuery(float *mbr, SortedLinList *res)
{
    load_root();
	//Added by Tanzima
	io_access++;
	//..............
    root_ptr -> rangeQuery(mbr,res);

	delete root_ptr;
	root_ptr = NULL;	
}
//------------------------------------------------------------
void RTree::read_header(char *buffer)
{
    int i;

    memcpy(&dimension, buffer, sizeof(dimension));
    i = sizeof(dimension);

    memcpy(&num_of_data, &buffer[i], sizeof(num_of_data));
    i += sizeof(num_of_data);

    memcpy(&num_of_dnodes, &buffer[i], sizeof(num_of_dnodes));
    i += sizeof(num_of_dnodes);

    memcpy(&num_of_inodes, &buffer[i], sizeof(num_of_inodes));
    i += sizeof(num_of_inodes);

    memcpy(&root_is_data, &buffer[i], sizeof(root_is_data));
    i += sizeof(root_is_data);

    memcpy(&root, &buffer[i], sizeof(root));
    i += sizeof(root);
}
//------------------------------------------------------------
int RTree::update_rslt(Entry *_e, float _dist, Entry *_rslt, 
						float *_key, int _k)
{
	for (int i = 0; i < _k; i ++)
	{
		if (_dist < _key[i])
		{
			for (int j = _k - 1; j > i; j --)
			{
				_rslt[j] = _rslt[j - 1];
				_key[j] = _key[j - 1];
			}
			_rslt[i] = *_e;
			_key[i] = _dist;
			return i;
		}
	}
	error("Error in update_rslt\n", true);
	return -1;
}
//------------------------------------------------------------
void RTree::write_header(char *buffer)
{
    int i;

    memcpy(buffer, &dimension, sizeof(dimension));
    i = sizeof(dimension);

    memcpy(&buffer[i], &num_of_data, sizeof(num_of_data));
    i += sizeof(num_of_data);

    memcpy(&buffer[i], &num_of_dnodes, sizeof(num_of_dnodes));
    i += sizeof(num_of_dnodes);

    memcpy(&buffer[i], &num_of_inodes, sizeof(num_of_inodes));
    i += sizeof(num_of_inodes);

    memcpy(&buffer[i], &root_is_data, sizeof(root_is_data));
    i += sizeof(root_is_data);

    memcpy(&buffer[i], &root, sizeof(root));
    i += sizeof(root);
}
//------------------------------------------------------------
//---added for valdity region queries-------------------------
// perform a window query mbr to get the query result into in_objs, put the outer objects retrieved into out_objs_so_far
void RTree::rect_win_query(float *mbr, LinList *in_objs, LinList *out_objs_so_far)
{
    load_root();

    root_ptr->rect_win_query(mbr, in_objs, out_objs_so_far);

	delete root_ptr;
	root_ptr = NULL;
}

// perform a window query mbr (excluding the window excr) to get the query result into c_inf_objs
void RTree::rect_win_query(float *mbr, float *exclmbr, LinList *c_inf_objs)
{
    load_root();

    root_ptr->rect_win_query(mbr, exclmbr, c_inf_objs);

	delete root_ptr;
	root_ptr = NULL;
}

void RTree::BFNN(float *_qpt, int _k, Entry *_rslt)
{
	//first init an array for storing the keys of retrieve objects
	float *key = new float [_k];
	for (int i = 0; i < _k; i ++)
		key[i] = (float) MAXREAL; //initially they are infinity
	//init a heap that stores the non-leaf entries to be accessed-
	Heap *heap = new Heap();
	heap->init(dimension);
	//------------------------------------------------------------
	
	int son = root; //this entry is to be visited next
	while (son != -1)
	{
		RTNode *rtn = new RTNode(this, son);
		for (int i = 0; i < rtn -> num_entries; i ++)
		{
			float edist = MINDIST(_qpt, rtn->entries[i].bounces, dimension);
			if (rtn->level == 0)
			{
				if (edist < key[_k - 1])
					update_rslt(&(rtn->entries[i]), edist, _rslt, key, _k);
			}
			else
			{
				if (edist<key[_k - 1]) 
					//meaning that edist is valid and we insert it to heap
				{
					HeapEntry *he = new HeapEntry();
					he -> key = edist;
					he -> level = rtn -> level;
					he -> son1 = rtn->entries[i].son;
					heap -> insert(he);
					delete he;
				}
			}
		}
	
		delete rtn;

		//get next entry from the heap----------------------------
		HeapEntry *he = new HeapEntry();
		bool again = true;
		while (again)
		{
			again = false;
			if (!heap->remove(he))  //heap is empty
				son = -1;
			else
			{
				if (he->key>key[_k - 1]) //the algorithm terminates
					son = -1;
				else if (he->level == 0) 
						//protection. if you see this message, debug
						error("testing... leaf entries found in heap\n", true);
					 else
						son=he->son1;
			}
		}
		delete he;
		//--------------------------------------------------------
	}

	delete [] key;
	delete heap;
}





//RTree::BFNN --- Modified by Tanzima for Rectangle

float RTree::KMin(Point2D m, int c, float cl, int k, Pointlocation _rslt[], int *num_of_data, float d_safe_corner,DistfromPOI _cornertoPOI[])
{
	float *kmindist = new float[k];
	float tempdist;
	for(int i=0; i<k; i++)
	{
			kmindist[i]=(float) MAXREAL;
	}	
	
	for(int i=0; i<*num_of_data;i++)
	{
		if(cl* _cornertoPOI[i].d[c] <= d_safe_corner)
		{
			tempdist=Dist(_rslt[i],m);			

			for(int j=0; j<k; j++)
			{				

				if(kmindist[j]>tempdist)
				{
					for(int l=k-1;l>j;l--)
					{
						kmindist[l]= kmindist[l-1];
					}
					kmindist[j]= tempdist;
					break;
				}
			}
		}
		
	}
	return kmindist[k-1];
}


void RTree::UpdateCount(Rectangle1 R, float cl, int k, Pointlocation _rslt[], int *num_of_data, float d_safe_corner, int *count, DistfromPOI _cornertoPOI[])
{
	Point2D c[4];
	c[0][0] = R.x1; c[0][1]=R.y1;
	c[1][0] = R.x1; c[1][1]=R.y2;
	c[2][0] = R.x2; c[2][1]=R.y2;
	c[3][0] = R.x2; c[3][1]=R.y1;
	
		
	_cornertoPOI[*num_of_data-1].d[0]= Dist(_rslt[*num_of_data-1],c[0]);
	_cornertoPOI[*num_of_data-1].d[1]= Dist(_rslt[*num_of_data-1],c[1]);
	_cornertoPOI[*num_of_data-1].d[2]= Dist(_rslt[*num_of_data-1],c[2]);
	_cornertoPOI[*num_of_data-1].d[3]= Dist(_rslt[*num_of_data-1],c[3]);
	
	for(int j=0; j<4; j++)
	{
		if(count[j]<k)
		{
			count[j]=0;
			for(int i=0; i<*num_of_data;i++)
			{
				if(cl*_cornertoPOI[i].d[j] <= d_safe_corner)	count[j]++;
				
			}
		}
	}
	/*
	for(int i=0; i<_rslt.size();i++)
	{
		if(i<_cornertoPOI.size())
		{
			if(cl*_cornertoPOI[i].d[0] <= d_safe_corner)	count[0]++;
			if(cl*_cornertoPOI[i].d[1] <= d_safe_corner)	count[1]++;
			if(cl*_cornertoPOI[i].d[2] <= d_safe_corner)	count[2]++;
			if(cl*_cornertoPOI[i].d[3] <= d_safe_corner)	count[3]++;
		
		}
		else
		{
			Pointlocation p = _rslt[i];
			dist_c_P.d[0]= Dist(p,c[0]);
			if(cl*dist_c_P.d[0] <= d_safe_corner)	count[0]++;
			dist_c_P.d[1]= Dist(p,c[1]);
			if(cl*dist_c_P.d[1] <= d_safe_corner)	count[1]++;
			dist_c_P.d[2]= Dist(p,c[2]);
			if(cl*dist_c_P.d[2] <= d_safe_corner)	count[2]++;
			dist_c_P.d[3]= Dist(p,c[3]);
			if(cl*dist_c_P.d[3] <= d_safe_corner)	count[3]++;
			_cornertoPOI.push_back(dist_c_P);

		}
	}
	*/
}

int RTree::UpdateStatus(Rectangle1 R, float cl, int k, Pointlocation _rslt[], int *num_of_data, int *count, DistfromPOI _cornertoPOI[])
{
	Point2D o;
	o[0] = (R.x1+R.x2)/2;
	o[1] = (R.y1+R.y2)/2;
	float r = Dist(_rslt[*num_of_data-1],o);
	
	Point2D c[4];
	c[0][0] = R.x1; c[0][1]=R.y1;
	c[1][0] = R.x1; c[1][1]=R.y2;
	c[2][0] = R.x2; c[2][1]=R.y2;
	c[3][0] = R.x2; c[3][1]=R.y1;

	float d1=Dist(c[0],c[1]);
	float d2=Dist(c[1],c[2]);
	float d_safe_corner = r-0.5*sqrt(d1*d1+d2*d2);

	

	
	UpdateCount(R,cl,k,_rslt,num_of_data,d_safe_corner,count,_cornertoPOI);
	

	for(int i=0; i<4; i++)
	{
		if (count[i]<k)	return 0;
	}

	

	float d_i, d_j,d_max_m,d_max = 0;
	int i,j;
	Point2D m;
	for (i=0; i<4; i++)
	{
		j=(i+1)%4;
		
		m[0]= (c[i][0]+c[j][0])/2;
		m[1]= (c[i][1]+c[j][1])/2;
		d_i= KMin(m,i,cl,k,_rslt,num_of_data,d_safe_corner,_cornertoPOI);
		d_j= KMin(m,j,cl,k,_rslt,num_of_data,d_safe_corner,_cornertoPOI);
		if(d_i>d_j) d_max_m = d_i;
		else d_max_m = d_j;
		if(d_max_m > d_max)
			d_max = d_max_m;
		
	}
	float c_dmax=d1;
	if(d2>d1)	c_dmax=d2;
	float d_safe = r- 0.5 * c_dmax;	

	if(cl*d_max>d_safe)
		return ceil(r+cl*(d_max-d_safe));
	else
		return -1;
}
void RTree::Rect_kNNQ(Rectangle1 R, float cl, int k, Pointlocation _rslt[], int *num_of_data)
{
	int io=0;
	//init status
	int count[4];
	for(int i=0; i<4; i++)	count[i]=0;
	int status=0;
	Point2D o;
	o[0] = (R.x1+R.x2)/2;
	o[1] = (R.y1+R.y2)/2;
	DistfromPOI cornertoPOI[10000];	

	//init a heap that stores the non-leaf entries to be accessed-
	Heap *heap = new Heap();
	heap->init(dimension);
	//------------------------------------------------------------
	
	int son = root; //this entry is to be visited next
	while (son != -1 && status != -1)
	{
		
		
		RTNode *rtn = new RTNode(this, son);
		//Experiment
		io_access++;
		for (int i = 0; i < rtn -> num_entries; i++)
		{
			float o1[2];
			o1[0]=(float)o[0];
			o1[1]=(float)o[1];
			float edist = MINDIST(o1, rtn->entries[i].bounces, dimension);
			
			HeapEntry *he = new HeapEntry();
			he -> key = edist;
			he -> level = rtn -> level;
			he -> son1 = rtn->entries[i].son;
			he-> x1 = rtn->entries[i].bounces[0];
			//he-> x2 = rtn->entries[i].bounces[1];
			he-> y1 = rtn->entries[i].bounces[2];
			//he-> y2 = rtn->entries[i].bounces[3];
			heap -> insert(he);
			delete he;			

		}
	
		delete rtn;

		
		//get next entry from the heap----------------------------
		HeapEntry *he = new HeapEntry();
		bool again = true;
		while (again)
		{			
			again = false;
			if (!heap->remove(he))  //heap is empty
				son = -1;
			else
			{	
				if (he->level == 0) //p is an object 
				{
					if(*num_of_data==10000)
						//printf("\nGreater than 10000\n");
						error("Rect_kNNQ:DataPoint maximum Limit exceeded\n",TRUE);
					
					_rslt[*num_of_data].x=he->x1;
					_rslt[*num_of_data].y=he->y1;
					*num_of_data=*num_of_data+1;
					//if(*num_of_data%1000==0) printf("\n%d",*num_of_data);
						
					
					
					if(status==0)
					{												
						status = UpdateStatus(R,cl,k,_rslt,num_of_data,count,cornertoPOI);
												
						if(status != -1)	again=true;						
					}
					else if (status < Dist(_rslt[*num_of_data-1],o))
					{							
						status = -1;
					}
					else
					{
						again = true;
					}
				}
				else
				{
					if(status>0)
					{
						if(status<he->key)
							status = -1;
						else
							son=he->son1;
					}
					else
					{
						son=he->son1;
					}
				}
			}
		}
		
		delete he;
	}
	delete heap;
}

void RTree::Point_BFN_NNQ(Point2D o, double *_rslt)
{
	
	//init a heap that stores the non-leaf entries to be accessed-
	Heap *heap = new Heap();
	heap->init(dimension);
	//------------------------------------------------------------
	
	int son = root; //this entry is to be visited next
	while (son != -1)
	{
		RTNode *rtn = new RTNode(this, son);
		io_access++;
		for (int i = 0; i < rtn -> num_entries; i ++)
		{
			float o1[2];
			o1[0]=(float)o[0];
			o1[1]=(float)o[1];
			float edist = MINDIST(o1, rtn->entries[i].bounces, dimension);
			
			HeapEntry *he = new HeapEntry();
			he -> key = edist;
			he -> level = rtn -> level;
			he -> son1 = rtn->entries[i].son;
			he-> x1 = rtn->entries[i].bounces[0];
			//he-> x2 = rtn->entries[i].bounces[1];
			he-> y1 = rtn->entries[i].bounces[2];
			//he-> y2 = rtn->entries[i].bounces[3];
			heap -> insert(he);
			delete he;			

		}
	
		delete rtn;

		
		//get next entry from the heap----------------------------
		HeapEntry *he = new HeapEntry();
		bool again = true;
		while (again)
		{
			again = false;
			if (!heap->remove(he))  //heap is empty
				son = -1;
			else
			{
				if (he->level == 0) //p is an object 
				{
					_rslt[0] = he->x1;
					_rslt[1] = he->y1;
					son=-1;
														
				}
				else
				{
					son=he->son1;
					
				}
			}
		}
		
		delete he;
	}
	delete heap;
}
//END




// The following code was copied from the implementation of TP-kNN queries from Tony
void RTree::TPNN_TP(float *_qline, int _k, Entry *_nn, Entry *_rslt, float _max_trvl)
{
	float key = _max_trvl; 
	  // the minimum distance that the query point must travel 

//we comment these lines to avoid initing the heap everytime--
//this function is called
//Heap *heap = new Heap();
//heap->init(dimension);
//------------------------------------------------------------
	if (tpheap==NULL)
		error("tpheap is not initialized\n", true);
	tpheap->used=0;
	
	int son = root;
	while (son != -1)
	{
		RTNode *rtn = new RTNode(this, son);
		for (int i = 0; i < rtn -> num_entries; i ++)
		{
			//first create an array m2 for e to cal dist----------
			float *m2 = new float [4 * dimension];
			memset(m2, 0, 4 * dimension * sizeof(float));
			memcpy(m2, rtn -> entries[i].bounces, 2 * dimension * sizeof(float));
			//----------------------------------------------------
//testing--------------------------
//if (rtn->entries[i].son==573673 && cnt==84-1)
//	printf("testing...\n");
//---------------------------------

			float edist = (float) MAXREAL;
			if (rtn -> level == 0)
//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^leaf node case^^^^^^^^^^^^^^^^^^^^^
			{	
				int eNNsub=-1;
				//if e (i.e., m2) is one of the current NN points-
				//eNNsub stores its subsript in _nn; otherwise, 
				//eNNsub=-1
				for (int j=0; j<_k; j++)
					if (rtn->entries[i].son==_nn[j].son)
					{ eNNsub=j; j=_k;}
				//------------------------------------------------
		
				if (eNNsub==-1 || SEQUENCE_SENSITIVE)
					//namely, if sequence insensitive and e is indeed a NN
					//then we do not handle this entry
				{

					float *m1 = new float [4 * dimension];
					//find the NN that leads to the minimum-------
					//influence time
					int nn_num=-1; //the subsript of the NN to be found
					for (int j = 0; j < _k; j ++)
						//for each of the NN found in the 1st step
					{
						bool yesdo=true; //whether to compute
						  //the inflence time of nn[j] and e
						if (j==eNNsub) yesdo=false;
						
						//check if this pair has produced --------
						//influence time before
						for (int l=0; l<PAST_PAIR; l++)
							if (min(_nn[j].son, rtn->entries[i].son)==min(last_pair[l][0], last_pair[l][1]) &&
								max(_nn[j].son, rtn->entries[i].son)==max(last_pair[l][0], last_pair[l][1]))
							{	yesdo=false; l=PAST_PAIR; }
						//----------------------------------------

						if (yesdo)
						{
//these codes use NNinf===========================================
/*
							//first create an array m1 (for nn[j])
							//to compute dist
							memset(m1, 0, 4 * dimension * sizeof(float));
							memcpy(m1, _nn[j].bounces, 2 * dimension * sizeof(float));
							//get the influence time of m2-------- 
							//(for entry e) with respect to(nn[j])
							float this_inf = NNinf(m1, m2, _qline, dimension); 
							//------------------------------------
*/
//================================================================

//these codes use NNinf2==========================================
							//first create an array m1 (for nn[j])
							//to compute dist
							memset(m1, 0, 4 * dimension * sizeof(float));
							memcpy(m1, _nn[j].bounces, 2 * dimension * sizeof(float));
							m1[1]=m1[2]; m2[1]=m2[2];
							//create an arry m3 for _qline----------------
							float *m3=new float[2*dimension];
							m3[0]=_qline[0]; m3[1]=_qline[2];
							m3[2]=_qline[4]; m3[3]=_qline[6];
							//--------------------------------------------
							//get the influence time of m2-------- 
							//(for entry e) with respect to(nn[j])
							float this_inf = NNinf2(m1, m2, m3); 
							//------------------------------------
							delete []m3;
//================================================================

							if (this_inf>0 && this_inf<edist)
								//this_inf=0 means that there is another point that has the same distance
								//to the current query position as the current NN. In this implementation,
								//we choose to ignore handling such special cases, which, however, may cause
								//problems for datasets with big cardinality
//							if (this_inf>=0 && this_inf<edist)
							{
								edist=this_inf; nn_num=j;
							}
						}  //END if (yesdo)
					}//END checking all neighbors
					//-------------------------------------------------
					//if (edist<key && edist!=0)
					if (edist<key)
					{
						update_rslt(&(rtn->entries[i]), edist, _rslt, &key, 1);
						_rslt->nn_num=nn_num;
					}
					delete []m1;
				}
			}
//^^^^^^^^^^^^^^^^^^^^^^^^^non-leaf node case^^^^^^^^^^^^^^^^^^^^^
			else
				//Next handle non-leaf node case
			{
				float *m1 = new float [4 * dimension];
				for (int j = 0; j < _k; j ++)
				{
					//first create an array m1 to cal dist--------
					memset(m1, 0, 4 * dimension * sizeof(float));
					memcpy(m1, _nn[j].bounces, 2 * dimension * sizeof(float));
					//--------------------------------------------
					float this_mininf = NNmininf(m1, m2, _qline, dimension);
					if (this_mininf < edist)
						edist = this_mininf;
				}
				delete [] m1;

				if (edist < key)
				{
					HeapEntry *he = new HeapEntry();
					he -> key = edist;
					he -> level = rtn -> level;
					he -> son1 = rtn->entries[i].son;
					tpheap -> insert(he);
					delete he;
				}
			}
			delete [] m2;

		}
//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^	
		delete rtn;

		//get next entry from the heap
		bool again = true;
		while (again)
		{
			again = false;
			HeapEntry *he = new HeapEntry();
			if (!tpheap -> remove(he))  //heap is empty
				son = -1;
			else
				if (he -> key > key)
					//the algorithm can terminate
					son = -1;
				else
					son = he -> son1;
			delete he;
		}
	}
//delete heap;
}



//RTree::private_kGNN --- Created by Tanzima for kGNN



void RTree::private_kGNN_sum(Rectangle1 R[], int g_size, int k, Pointlocation rslt[], int *num_of_data)
{	
	//variable initialization
	int end=0;

	//Variables
	int i, j;

	double maxdist[kMAX];
	for(i=0; i<k; i++)
		maxdist[i] = MAXDOUBLE;
	
	//Find the MBR from provided rectangles
	Rectangle1 mbr;
	mbr.x1=R[0].x1;
	mbr.x2=R[0].x2;
	mbr.y1=R[0].y1;
	mbr.y2=R[0].y2;
	
	for(i=1; i<g_size; i++)
	{
		if(R[i].x1 < mbr.x1)	mbr.x1=R[i].x1;
		if(R[i].x2 > mbr.x2)	mbr.x2=R[i].x2;
		if(R[i].y1 < mbr.y1)	mbr.y1=R[i].y1;
		if(R[i].y2 > mbr.y2)	mbr.y2=R[i].y2;
	}
	//end

	//init a heap that stores the non-leaf entries to be accessed-
	Heap *heap = new Heap();
	heap->init(dimension);
	//------------------------------------------------------------
	
	int son = root; //this entry is to be visited next
	
	while (son != -1 && end==0)
	{		
		RTNode *rtn = new RTNode(this, son);
		
		for (int i = 0; i < rtn -> num_entries; i++)
		{
			double edist1=0, edist2=0;
			edist1 = MINRECTDIST(mbr, rtn->entries[i].bounces);	
			if((g_size*edist1)>maxdist[k-1])	continue;
			edist1 = MINRECTDIST(R[0], rtn->entries[i].bounces);
			for (j=1; j < g_size; j++)
			{				
				edist1 += MINRECTDIST(R[j], rtn->entries[i].bounces);
			}

			if(edist1>maxdist[k-1])	continue;
			
			edist2 = MAXRECTDIST(R[0], rtn->entries[i].bounces);
			for (j=1; j < g_size; j++)
			{	
				edist2 += MAXRECTDIST(R[j], rtn->entries[i].bounces);
			}

			//update maxdistk		
			for(int l=0; l<k; l++)
			{
				if (edist2 < maxdist[l])
				{
					for(int j=k-1; j>l; j--)
					{
						maxdist[j]=maxdist[j-1];						
					}
					maxdist[l]=edist2;
					break;
				}
			}
			
			HeapEntry *he = new HeapEntry();
			he -> key = edist1;
			he -> key1 = edist2;
			he -> level = rtn -> level;
			he -> son1 = rtn->entries[i].son;
			he-> x1 = rtn->entries[i].bounces[0];
			//he-> x2 = rtn->entries[i].bounces[1];
			he-> y1 = rtn->entries[i].bounces[2];
			//he-> y2 = rtn->entries[i].bounces[3];
			heap -> insert(he);
			delete he;			

		}
	
		delete rtn;
		
		//get next entry from the heap----------------------------
		HeapEntry *he = new HeapEntry();
		bool again = true;
		while (again)
		{			
			again = false;
			if (!heap->remove(he))  //heap is empty
				son = -1;
			else if (he->key>maxdist[k-1])
			{
				end = 1;

			}
			else
			{	
				if (he->level == 0) //p is an object 
				{
						//enter into result set
						if(*num_of_data==MAXDATALIMIT)
							//printf("\nGreater than 10000\n");
							error("Rect_kNNQ:DataPoint maximum Limit exceeded\n",TRUE);
						
						rslt[*num_of_data].x=he->x1;
						rslt[*num_of_data].y=he->y1;
						rslt[*num_of_data].dmin=he->key;
						rslt[*num_of_data].dmax=he->key1;


						*num_of_data=*num_of_data+1;
						
						//get next data  from heap
						again =true;						
				}
				else //not leaf node
				{
					son=he->son1;
				}
			}
		}
		
		delete he;
	}
	delete heap;
}


//END


void RTree::private_kGNN_max(Rectangle1 R[], int g_size, int k, Pointlocation rslt[], int *num_of_data)
{
	

	//variable initialization
	int end=0;

	//Variables
	int i, j;

	double maxdist[kMAX];
	for(i=0; i<k; i++)
		maxdist[i] = MAXDOUBLE;
	
	//Find the MBR from provided rectangles
	Rectangle1 mbr;
	mbr.x1=R[0].x1;
	mbr.x2=R[0].x2;
	mbr.y1=R[0].y1;
	mbr.y2=R[0].y2;
	
	for(i=1; i<g_size; i++)
	{
		if(R[i].x1 < mbr.x1)	mbr.x1=R[i].x1;
		if(R[i].x2 > mbr.x2)	mbr.x2=R[i].x2;
		if(R[i].y1 < mbr.y1)	mbr.y1=R[i].y1;
		if(R[i].y2 > mbr.y2)	mbr.y2=R[i].y2;
	}
	//end


	//init a heap that stores the non-leaf entries to be accessed-
	Heap *heap = new Heap();
	heap->init(dimension);
	//------------------------------------------------------------
	
	int son = root; //this entry is to be visited next

	
	while (son != -1 && end==0)
	{
		
		
		RTNode *rtn = new RTNode(this, son);
	
		for (int i = 0; i < rtn -> num_entries; i++)
		{
			double edist1=0, edist2=0, tdist=0;
			edist1 = MINRECTDIST(mbr, rtn->entries[i].bounces);	
			if((edist1)>maxdist[k-1])	continue;

			
			edist1 = MINRECTDIST(R[0], rtn->entries[i].bounces);
			for (j=1; j < g_size; j++)
			{
				tdist = MINRECTDIST(R[j], rtn->entries[i].bounces);
				if(tdist>edist1)
					edist1=tdist;			
			}			

			if(edist1>maxdist[k-1])	continue;
			
			edist2 = MAXRECTDIST(R[0], rtn->entries[i].bounces);
			for (j=1; j < g_size; j++)
			{
				tdist= MAXRECTDIST(R[j], rtn->entries[i].bounces);
				if(tdist>edist2)
					edist2=tdist;			
			}

			

			//update maxdistk		
			for(int l=0; l<k; l++)
			{
				if (edist2 < maxdist[l])
				{
					for(int j=k-1; j>l; j--)
					{
						maxdist[j]=maxdist[j-1];						
					}
					maxdist[l]=edist2;
					break;
				}
			}
		
			HeapEntry *he = new HeapEntry();
			he -> key = edist1;
			he -> key1 = edist2;
			he -> level = rtn -> level;
			he -> son1 = rtn->entries[i].son;
			he-> x1 = rtn->entries[i].bounces[0];
			//he-> x2 = rtn->entries[i].bounces[1];
			he-> y1 = rtn->entries[i].bounces[2];
			//he-> y2 = rtn->entries[i].bounces[3];
			heap -> insert(he);
			delete he;			

		}
	
		delete rtn;

		
		//get next entry from the heap----------------------------
		HeapEntry *he = new HeapEntry();
		bool again = true;
		while (again)
		{			
			again = false;
			if (!heap->remove(he))  //heap is empty
				son = -1;
			else if (he->key>maxdist[k-1])
			{
				end = 1;

			}
			else
			{	

				if (he->level == 0) //p is an object 
				{
						//enter into result set
						if(*num_of_data==MAXDATALIMIT)
							//printf("\nGreater than 10000\n");
							error("Rect_kNNQ:DataPoint maximum Limit exceeded\n",TRUE);
						
						rslt[*num_of_data].x=he->x1;
						rslt[*num_of_data].y=he->y1;
						rslt[*num_of_data].dmin=he->key;
						rslt[*num_of_data].dmax=he->key1;

						*num_of_data=*num_of_data+1;
						//if(*num_of_data%1000==0) printf("\n%d",*num_of_data);
						
						//get next data  from heap
						again =true;						
				}
				else //not leaf node
				{
					son=he->son1;
				}
			}
		}
		
		delete he;
	}
	delete heap;
	
}



//added by Eunus: 

//I AM using Rectangle1 as query points

void RTree::consensus_kGNN(Rectangle1 R[], int g_size, int sg_size, int k, int f, SG L[][32], Pointlocation rslt[], int *num_of_data)
{
	

	//variable initialization
	int end=0;

	//Variables
	int i, j;

	double maxdist[kMAX];
	for(i=0; i<k; i++)
		maxdist[i] = MAXDOUBLE;
	
	

	double BestAggDist[128];
	bool status[128];

	//SG L[g_size-sg_size+1][kMAX]; //result list

	for(i=0; i<g_size-sg_size+1; i++){
		BestAggDist[i] = MAXDOUBLE;
		status[i] = false; //temporary; true-> final
		for(j=0; j<k; j++){
			//need to initialize
			//SG sg;
			//L[i][j] = sg; //RESULT List

			L[i][j].sgDist = MAXDOUBLE;
			L[i][j].m = sg_size+i; //subgrop size
			L[i][j].o = -1;
			L[i][j].gDist = MAXDOUBLE;
			L[i][j].status = false;

			
		}
	
	}

	Point2D x;
	GEOCENTROID(R,g_size,x);
	//calculate maximum distance to query points
	float maxQDist = MAXGROUPDIST(R,g_size,x);


	//Find the MBR from provided rectangles
	Rectangle1 mbr;
	mbr.x1=R[0].x1;
	mbr.x2=R[0].x2;
	mbr.y1=R[0].y1;
	mbr.y2=R[0].y2;
	
	for(i=1; i<g_size; i++)
	{
		if(R[i].x1 < mbr.x1)	mbr.x1=R[i].x1;
		if(R[i].x2 > mbr.x2)	mbr.x2=R[i].x2;
		if(R[i].y1 < mbr.y1)	mbr.y1=R[i].y1;
		if(R[i].y2 > mbr.y2)	mbr.y2=R[i].y2;
	}
	//end


	//init a heap that stores the non-leaf entries to be accessed-
	Heap *heap = new Heap();
	heap->init(dimension);
	//------------------------------------------------------------
	
	int son = root; //this entry is to be visited next

	//son == -1 means empty && end = 0 means exit

	while (son != -1 && end==0)
	{
		
		
		RTNode *rtn = new RTNode(this, son);
	
		for (i = 0; i < rtn -> num_entries; i++)
		{
			double edist1=0;//edist2=0, tdist=0;
			//edist1 = MINRECTDIST(mbr, rtn->entries[i].bounces);	
			
			Pointlocation xp;
			xp.x = x[0];
			xp.y = x[1];

			Rectangle1 nd;
			

			
			nd.x1 = rtn->entries[i].bounces[0];
			nd.x2 = rtn->entries[i].bounces[1];
			nd.y1 = rtn->entries[i].bounces[2];
			nd.y2 = rtn->entries[i].bounces[3];

			if(rtn->level == 0){ //child node is a object
				nd.x2 = nd.x1;
				nd.y2 = nd.y1;
			}

			edist1 = MINRECTDIST1(xp, nd); // mindist from x to node
			
			/*
			if((edist1)>maxdist[k-1])	continue;
			edist1 = MINRECTDIST(R[0], rtn->entries[i].bounces);
			for (j=1; j < g_size; j++)
			{
				tdist = MINRECTDIST(R[j], rtn->entries[i].bounces);
				if(tdist>edist1)
					edist1=tdist;			
			}			

			if(edist1>maxdist[k-1])	continue;
			
			edist2 = MAXRECTDIST(R[0], rtn->entries[i].bounces);
			for (j=1; j < g_size; j++)
			{
				tdist= MAXRECTDIST(R[j], rtn->entries[i].bounces);
				if(tdist>edist2)
					edist2=tdist;			
			}

			*/			

			//update maxdistk		
			/*
			for(int l=0; l<k; l++)
			{
				if (edist2 < maxdist[l])
				{
					for(int j=k-1; j>l; j--)
					{
						maxdist[j]=maxdist[j-1];						
					}
					maxdist[l]=edist2;
					break;
				}
			}
			*/
		
			HeapEntry *h = new HeapEntry();
			h -> key = edist1;
			//he -> key1 = edist2;
			h -> level = rtn -> level;
			h -> son1 = rtn->entries[i].son;
			
			
			h-> x1 = nd.x1; //rtn->entries[i].bounces[0];
			h-> x2 = nd.x2; //rtn->entries[i].bounces[1]; //UNCOMMENTED BY EUNUS ; WAS commented IN sHETU
			h-> y1 = nd.y1; //rtn->entries[i].bounces[2];
			h-> y2 = nd.y2;//rtn->entries[i].bounces[3]; //UNCOMMENTED BY eUNUS
			heap -> insert(h);
			delete h;			

		}
	
		delete rtn;

		
		//get next entry from the heap----------------------------
		HeapEntry *he = new HeapEntry();
		bool again = true;
		while (again)
		{			
			again = false;
			
			bool isempty = !heap->remove(he);
			
			Rectangle1 p;
			p.x1 = he-> x1;
			p.y1 = he-> y1;
			p.x2 = he-> x2;
			p.y2 = he-> y2;


			if (isempty)  //heap is empty
				son = -1;
			else if (!TERMINATE_LB(R, g_size,sg_size,x,p,maxQDist,f,BestAggDist,status))//(he->key>maxdist[k-1]) //check terminating condition
			{
				//bool TERMINATE_LB(Rectangle1 R[], int g_size, int sg_size, Point2D x, Rectangle1 p, float maxQDist, int f, double BestAggDist[], bool status[])
				
				end = 1;

			}
			else
			{	

				if (he->level == 0) //p is an object ; code for data object
				{
						
					//enter into result set
						if(*num_of_data==MAXDATALIMIT)
							//printf("\nGreater than 10000\n");
							error("Rect_kNNQ:DataPoint maximum Limit exceeded\n",TRUE);
						/*
						rslt[*num_of_data].x=he->x1;
						rslt[*num_of_data].y=he->y1;
						rslt[*num_of_data].dmin=he->key;
						rslt[*num_of_data].dmax=he->key1;
						*/

						*num_of_data=*num_of_data+1;
						//if(*num_of_data%1000==0) printf("\n%d",*num_of_data);
						

						//Eunus Code

						Heap *heapQ = new Heap();
						heapQ->init(dimension);
						double totalDist = 0.0;
						//float p[2];
						//p[0] = he->x1;
						//p[1] = he->y1;

						Point2D dp; //data point
						dp[0] = he->x1;
						dp[1] = he->y1;

						for(i=0; i<g_size;i++){
							
							HeapEntry *hq = new HeapEntry();
							
							Point2D qp; //query point
							qp[0] = R[i].x1;
							qp[1] = R[i].y1;

							hq -> key = Dist(dp,qp);//edist1; distance between q and p
							//he -> key1 = edist2;
							//he -> level = rtn -> level;
							hq -> son1 = i;//rtn->entries[i].son;
							hq-> x1 = qp[0];//rtn->entries[i].bounces[0];
							//he-> x2 = rtn->entries[i].bounces[1];
							hq-> y1 = qp[1];//rtn->entries[i].bounces[2];
							//he-> y2 = rtn->entries[i].bounces[3];
							heapQ -> insert(hq);
							

							if(i==0){
								totalDist = hq -> key;
							}else{
								totalDist = fDIST(totalDist,hq -> key,f);
							}

							delete hq;
						} //end of creating heapQ for query based on the distnace from data point

						//popping elements from HeapQ

						double aggDist = 0.0;
						i = 0;
						
						char sgList[65];
						sgList[64] = '\0';

						HeapEntry *hq = new HeapEntry();
						while (heapQ->remove(hq))
						{			
							if(i==0){
								aggDist = hq -> key;
							}else{
								aggDist = fDIST(aggDist,hq -> key,f);
							}

							sgList[hq -> son1] = 1;

							i++;
							if(i >= sg_size && BestAggDist[i-sg_size] > aggDist){
								
								SG sgi;
								sgi.m = i;
								sgi.o = he -> son1;
								sgi.sgDist = aggDist;
								sgi.gDist = totalDist;
								sgi.status = false;

								//sgi.sgList[64] = '\0';	
								strcpy(sgi.sgList,sgList);
								
								//check with a exisiting subgroup and remove if it exists:
								
								/*
								int isduplicate = 0;
								for(int l=0; l<k; l++)
								{
									if(strcmp(L[i-sg_size][l].sgList,sgi.sgList)==0){
										if(L[i-sg_size][l].sgDist > sgi.sgDist){ //remove existing and procede
											for(int j = l; j<k; j++){ //shift everythign one cell up
												L[i-sg_size][j].sgDist = L[i-sg_size][j+1].sgDist;
												L[i-sg_size][j].m = L[i-sg_size][j+1].m;
												L[i-sg_size][j].o = L[i-sg_size][j+1].o;
											
												L[i-sg_size][j].gDist = L[i-sg_size][j+1].gDist;
												L[i-sg_size][j].status = L[i-sg_size][j+1].status;
									
												strcpy(L[i-sg_size][j].sgList,L[i-sg_size][j+1].sgList);

											}


										}else{ // not a candidate - continue while
											isduplicate = 1;
										}
										break;
									}
								}

								if(isduplicate ==1 ) continue;
								*/

								//order the result list RL and update BestAggDist

								
								for(int l=0; l<k; l++)
								{
									if (sgi.sgDist < L[i-sg_size][l].sgDist)
									{
										for(int j=k-1; j>l; j--)
										{
											//maxdist[j]=maxdist[j-1];	
											L[i-sg_size][j].sgDist = L[i-sg_size][j-1].sgDist;
											L[i-sg_size][j].m = L[i-sg_size][j-1].m;
											L[i-sg_size][j].o = L[i-sg_size][j-1].o;
											//L[i-1][j].aggDist = L[i-1][j-1].aggDist;
											L[i-sg_size][j].gDist = L[i-sg_size][j-1].gDist;
											L[i-sg_size][j].status = L[i-sg_size][j-1].status;
									
											strcpy(L[i-sg_size][j].sgList,L[i-sg_size][j-1].sgList);

										}
										//maxdist[l]=edist2;
										L[i-sg_size][l].sgDist = sgi.sgDist;
										L[i-sg_size][l].m = sgi.m;
										L[i-sg_size][l].o = sgi.o;
										L[i-sg_size][l].gDist = sgi.gDist;
										L[i-sg_size][l].status = sgi.status;
										strcpy(L[i-sg_size][l].sgList,sgi.sgList);
										
										//update BestAggDist
										//BestAggDist[i-sg_size] = sgi.sgDist;
										
										break;
									} //if
								} //for

								//update BestAggDist with the kth best
								BestAggDist[i-sg_size] = L[i-sg_size][k-1].sgDist;

							} //if

							

						} //while


						//get next data  from heap
						again =true;						
				}
				else //not leaf node
				{
					son=he->son1;
				}
			}
		}
		
		delete he;
	}
	delete heap;
	
}







//MOV

bool ccw(Pointlocation A,Pointlocation B,Pointlocation C)
{
	return (C.y-A.y)*(B.x-A.x) > (B.y-A.y)*(C.x-A.x);
}

bool intersect(Pointlocation A,Pointlocation B,Pointlocation C,Pointlocation D)//if line segments intersect
{
        return ccw(A,C,D) != ccw(B,C,D) && ccw(A,B,C) != ccw(A,B,D);
}

double slope(double x1,double y1,double x2,double y2)//2 points of a line is given
{
       double s= (double)(y1-y2)/(x1-x2);
       return s;
}
bool intersect_ext(Pointlocation a,Pointlocation b, Pointlocation c, Pointlocation d)
{
	double m1,m2;
	if(a.x!=b.x)
	{
		m1 = slope(a.x,a.y,b.x,b.y);
	}

	if(c.x!=d.x)
	{
		m2 = slope(c.x,c.y,d.x,d.y);
	}

	if( (a.x==b.x) && (c.x==d.x) )
	{
		return false;//both ar perpendicular on x axis 
	}
	else if((a.x==b.x) && (c.x!=d.x) )
	{
		return true;
	}
	else if((a.x!=b.x) && (c.x==d.x) )
	{
		return true;
	}
	else if(m1 == m2)
	{
		return false;//parallal lines
	}
	else return true;
}


// query, midpoint, top left, lower right
double angle_between_2_lines(double x1,double y1,double x2,double y2, double x11,double y11,double x21,double y21)
{
	double m1,m2;
	int flag1=0; int flag2=0;
    if(x1 != x2)
	{
		flag1=1;
		m1=slope(x1,y1,x2,y2);
	}
	if(x11!=x21)
	{
		flag2=1;
		m2=slope(x11,y11,x21,y21);
	}
	if((x1 == x2) && (x11 == x21))
	{
		return 0.0;
	}

	else if(x1 == x2)
	{
		return 90 - (atan((float)m2) * (180/PI));
	}
	else if(x11 == x21)
	{
		return 90 - (atan((float)m1) * (180/PI));
	}
	else
	{
		double tan_theta=((double)(m1-m2)/(1+m1*m2));
		if(tan_theta<0.0)tan_theta=(tan_theta*-1);//acute angle only
		return (atan((float)tan_theta) * (180/PI));
	}
       
}


Pointlocation mid_of_line_segment(Pointlocation a,Pointlocation b)
{
	Pointlocation mid;
	mid.x=(a.x+b.x)/2.0;
	mid.y=(a.y+b.y)/2.0;
	return mid;
}

Pointlocation intersection_point(Pointlocation p1,Pointlocation p2,Pointlocation p3,Pointlocation p4)
{
	Pointlocation intersect;
	intersect.x = (((p1.x*p2.y- p2.x*p1.y)*(p3.x-p4.x)) - ((p1.x-p2.x)*(p3.x*p4.y - p3.y*p4.x))) / (((p1.x-p2.x)*(p3.y-p4.y)) - ((p1.y-p2.y)*(p3.x-p4.x)));

	intersect.y = (((p1.x*p2.y- p2.x*p1.y)*(p3.y-p4.y)) - ((p1.y-p2.y)*(p3.x*p4.y - p3.y*p4.x))) / (((p1.x-p2.x)*(p3.y-p4.y)) - ((p1.y-p2.y)*(p3.x-p4.x)));
	return intersect;
}
double reduced_visibility_factor4angle(double angle)
{
	return angle/90.0;
}


bool IsPointInsideTriangle(Pointlocation a,Pointlocation b,Pointlocation c,Pointlocation x)
{
	double angle1=angle_between_2_lines(x.x,x.y,a.x,a.y,x.x,x.y,c.x,c.y);
	double angle2=angle_between_2_lines(x.x,x.y,a.x,a.y,x.x,x.y,b.x,b.y);
	double angle3=angle_between_2_lines(x.x,x.y,b.x,b.y,x.x,x.y,c.x,c.y);

	if(abs(angle1+angle2+angle3-180)<=0.0001)
	{
		return true;
	}
}

double Distance(Pointlocation p1, Pointlocation p2)
{
	return sqrt( ((p1.x-p2.x)*(p1.x-p2.x)) + ((p1.y-p2.y)*(p1.y-p2.y)) );
}

double QueryPoint :: init_visibility(Rectangle2 Target)//considering no obstacle
{
	Pointlocation p1=Target.upper_left;
	Pointlocation p2(Target.upper_left.x,Target.lower_right.y);
	Pointlocation p3(Target.lower_right.x,Target.upper_left.y);
	Pointlocation p4 = Target.lower_right;

	bool p1_seen,p2_seen,p3_seen,p4_seen;
	p1_seen=p2_seen=p3_seen=p4_seen=false;

	total_visibility = 0.0;
	//initially kon kon side dekhte pay seta check korte hobe

	//check for line SEGMENT intersect 
	if(intersect(position, p1 , p2,p4)== false &&  intersect(position, p1 , p3,p4)== false) p1_seen=true;
	if(intersect(position, p4 , p1,p2)== false &&  intersect(position, p4 , p1,p3)== false) p4_seen=true;

	if(intersect(position, p2 , p3,p4)== false &&  intersect(position, p2 , p1,p3)== false) p2_seen=true;
	if(intersect(position, p3 , p1,p2)== false &&  intersect(position, p3 , p2,p4)== false) p3_seen=true;
	
	Pointlocation mid1, point1, point2;

	double angle_with_rect=0.0;
	double visibility_reduce_factor=1.0;
	double partial_visibility=0.0;

	if((p1_seen==true && p2_seen==true) || (p3_seen==true && p4_seen==true))
	{
		if(p1_seen==true && p2_seen==true){
			point1=p1; point2=p2;
		}
		else{		// if p1,p2 is seen, p3,p4 cannot be seen
			point1=p3; point2=p4;
		}
		
		mid1=mid_of_line_segment(point1,point2); //rectangle target, so p1,p2 length == p3,p4 length 
		angle_with_rect = angle_between_2_lines(mid1.x,mid1.y,position.x,position.y, point1.x,point1.y,point2.x,point2.y);//angle between the line connecting query point and mid of the p1,p2 and the line p1,p2
		visibility_reduce_factor = reduced_visibility_factor4angle(angle_with_rect);

		partial_visibility = Distance(point1, point2) * visibility_reduce_factor;
		total_visibility +=  partial_visibility;

		//construct the visible region list
		Pointlocation a;
		struct VisibleRegionOverT segment;
		segment.p1_overT=point1;
		segment.p2_overT=point2;
		segment.partial_visibility = partial_visibility;
		VisibleRegion.push_back(segment);
	}

	
	if((p1_seen==true && p3_seen==true) || (p2_seen==true && p4_seen==true))
	{
		if(p1_seen==true && p3_seen==true) {
			point1=p1; point2=p3;
		}
		else{		// if p1,p3 is seen, p2,p4 cannot be seen
			point1=p2; point2=p4;
		}
		
		mid1=mid_of_line_segment(point1,point2); //rectangle target, so p1,p2 length == p3,p4 length 
		angle_with_rect = angle_between_2_lines(mid1.x,mid1.y,position.x,position.y, point1.x,point1.y,point2.x,point2.y);//angle between the line connecting query point and mid of the p1,p2 and the line p1,p2
		visibility_reduce_factor = reduced_visibility_factor4angle(angle_with_rect);

		partial_visibility = Distance(point1, point2) * visibility_reduce_factor;
		total_visibility +=  partial_visibility;

		//construct the visible region list
		Pointlocation a;
		struct VisibleRegionOverT segment;
		segment.p1_overT=point1;
		segment.p2_overT=point2;
		segment.partial_visibility = partial_visibility;
		VisibleRegion.push_back(segment);
	}

	return total_visibility;
}


//visible region is the list of triangles where a triangle is formed of the query point and 2 points over the target
bool QueryPoint::IsInVisibleRegion(Rectangle2 rect)
{
	//check if rect intersects with any of the items in VisibleRegion list
	Pointlocation p1=rect.upper_left;
	Pointlocation p2;
	p2.x=rect.upper_left.x;
	p2.y=rect.lower_right.y;
	Pointlocation p3;
	p3.x=rect.lower_right.x;
	p3.y=rect.upper_left.y;
	Pointlocation p4= rect.lower_right;

	int itr;

	for ( itr = 0; itr < VisibleRegion.size(); itr++ )
	{
		//if rect is in the region (fully or partially) of triangle formed by querypoint, VisibleRegion.p1,VisibleRegion.p2 
		//return true

		if(intersect(position, VisibleRegion[itr].p1_overT, p1,p2)) return true;
		else if(intersect(position, VisibleRegion[itr].p1_overT, p1,p3)) return true;
		else if(intersect(position, VisibleRegion[itr].p1_overT, p2,p4)) return true;
		else if(intersect(position, VisibleRegion[itr].p1_overT, p3,p4)) return true;

		else if(intersect(position, VisibleRegion[itr].p2_overT, p1,p2)) return true;
		else if(intersect(position, VisibleRegion[itr].p2_overT, p1,p3)) return true;
		else if(intersect(position, VisibleRegion[itr].p2_overT, p2,p4)) return true;
		else if(intersect(position, VisibleRegion[itr].p2_overT, p3,p4)) return true;

		else if(intersect(VisibleRegion[itr].p2_overT, VisibleRegion[itr].p1_overT, p1,p2)) return true;
		else if(intersect(VisibleRegion[itr].p2_overT, VisibleRegion[itr].p1_overT, p1,p3)) return true;
		else if(intersect(VisibleRegion[itr].p2_overT, VisibleRegion[itr].p1_overT, p2,p4)) return true;
		else if(intersect(VisibleRegion[itr].p2_overT, VisibleRegion[itr].p1_overT, p3,p4)) return true;

		//is rect fully inside the visible region
		//else if 
		else if(IsPointInsideTriangle(position,VisibleRegion[itr].p1_overT,VisibleRegion[itr].p2_overT, p1)==true)//rect er jeno ekta point (p1 here) check korlei hobe, jehetu ager else if gula check kora ache
		{
			return true;
		}
		else return false;

	}
     return false;
                                        
}



bool if_intersects_target(Pointlocation position, Pointlocation obstacle_p, Rectangle2 target, Pointlocation& intersectPoint,Pointlocation p1,Pointlocation p2)
{
	
	Pointlocation target_p1 = target.upper_left;
	Pointlocation target_p2;
	target_p2.x=target.upper_left.x;
	target_p2.y=target.lower_right.y;
	Pointlocation target_p3;
	target_p3.x=target.lower_right.x;
	target_p3.y=target.upper_left.y;
	Pointlocation target_p4 = target.lower_right;

	float lowx,lowy,highx,highy;
	lowx =min(p1.x, p2.x);
	lowy = min(p1.y,p2.y);
	highx = max(p1.x,p2.x);
	highy = max(p1.y,p2.y);
	
	Pointlocation Tpoints[8] = {target_p1,target_p2, target_p1, target_p3, target_p3,target_p4, target_p2,target_p4};
	for(int i=0;i<8;i+=2)
	{
		if(intersect_ext(position,obstacle_p, Tpoints[i],Tpoints[i+1])==true)
		{
			intersectPoint = intersection_point(position,obstacle_p, Tpoints[i],Tpoints[i+1]);
			if(intersectPoint.x<=highx && intersectPoint.x>=lowx && intersectPoint.y<=highy && intersectPoint.y>=lowy)//intersect point is in that region
			{
				return true;
			}
		
		}
	}
	return false;
	
	
}

bool change_vRegion_one_side(Pointlocation position, Pointlocation obstacle_p1, Rectangle2 target, 
	Pointlocation unchanged_side_val, Pointlocation changed_side_val,Pointlocation& intersectPoint, 
	VisibleRegionOverT itr)
{
	bool check = if_intersects_target(position, obstacle_p1 , target, intersectPoint,unchanged_side_val,changed_side_val);
	if(check == true)
	{
		float a=Distance(unchanged_side_val, (intersectPoint));
		float b=Distance(unchanged_side_val,changed_side_val); 
		if(a<= b)
		{
						
			VisibleRegionOverT v;
			v.p1_overT = unchanged_side_val;
			v.p2_overT = (intersectPoint);
			
			return true;
		}
	
	}
	return false;
}


bool isPointOf_VR_changed(Pointlocation p1_overT, Pointlocation position, Pointlocation obstacle_p1,
	Pointlocation obstacle_p2,Pointlocation obstacle_p3,Pointlocation obstacle_p4)
	
{
	Pointlocation Opoints[8] = {obstacle_p1,obstacle_p2,obstacle_p1,obstacle_p3,obstacle_p2,obstacle_p4,obstacle_p3,obstacle_p4};
	
	for(int i=0;i<8;i+=2)
	{
		if(intersect(position, p1_overT, Opoints[i],Opoints[i+1]))
		{
			return true;
		}
	}
	
	return false;
}




void QueryPoint :: update_visibliliyRegion(Rectangle2 obstacle, Rectangle2 target)
{
	Pointlocation obstacle_p1 = obstacle.upper_left;
	Pointlocation obstacle_p2(obstacle.upper_left.x,obstacle.lower_right.y);
	Pointlocation obstacle_p3(obstacle.lower_right.x,obstacle.upper_left.y);
	Pointlocation obstacle_p4 = obstacle.lower_right;

	Pointlocation target_p1 = target.upper_left;
	Pointlocation target_p2(target.upper_left.x,target.lower_right.y);
	Pointlocation target_p3(target.lower_right.x,target.upper_left.y);
	Pointlocation target_p4 = target.lower_right;

	bool p1_seen,p2_seen,p3_seen,p4_seen,p1_overT_changed,p2_overT_changed;
	p1_seen=p2_seen=p3_seen=p4_seen=p1_overT_changed=p2_overT_changed=false;

	int itr;
	Pointlocation temp_p;
	Pointlocation& intersectPoint=temp_p;
	
	Pointlocation& intersectPoint1=temp_p;Pointlocation& intersectPoint2=temp_p;

	//construct temp with the changed v. regions. finally make visibleRegion=temp
	vector<VisibleRegionOverT> temp;

	for ( itr = 0 ; itr < VisibleRegion.size(); itr++ )
	{
		//which part of the v.region is changed
		p1_overT_changed = isPointOf_VR_changed((VisibleRegion[itr]).p1_overT,position, obstacle_p1,obstacle_p2, obstacle_p3,obstacle_p4);
		p2_overT_changed = isPointOf_VR_changed((VisibleRegion[itr]).p2_overT,position, obstacle_p1,obstacle_p2, obstacle_p3,obstacle_p4);
		
		bool entirely_inside_region= false;
		if(p2_overT_changed == false && p1_overT_changed ==false && IsPointInsideTriangle(position,(VisibleRegion[itr]).p1_overT,(VisibleRegion[itr]).p2_overT, obstacle_p1)==true)//rect er jeno ekta point (p1 here) check korlei hobe, jehetu ager else if gula check kora ache
		{
			entirely_inside_region= true;
		}

		bool one_side_changed=false;
		Pointlocation unchanged_side_val;
		Pointlocation changed_side_val;
		bool insert_this_segment=true;

		if(p1_overT_changed==true && p2_overT_changed==true)//totally blocked this visibilitysegment
		{
			insert_this_segment= false;	
			total_visibility -= VisibleRegion[itr].partial_visibility;
		}

		
		else if( entirely_inside_region == true)
		{
			bool intersectPoint1_found,intersectPoint2_found;
			intersectPoint1_found=intersectPoint2_found=false;
			
			//split the region

			if(intersect(position, obstacle_p1, obstacle_p3, obstacle_p4)==false &&
			intersect(position, obstacle_p1, obstacle_p2, obstacle_p4)==false)//doesn't intersect with own arm, for point obstacle_p1
			{
				bool check = if_intersects_target(position, obstacle_p1 , target, intersectPoint1,(VisibleRegion[itr]).p1_overT,(VisibleRegion[itr]).p2_overT );
				if(check == true)
				{
					intersectPoint1_found=true;
				}
	

			}

			if(intersect(position, obstacle_p2, obstacle_p1, obstacle_p3)==false &&
				intersect(position, obstacle_p2, obstacle_p3, obstacle_p4)==false)//doesn't intersect with own arm, for point obstacle_p2
			{
				bool check;
				if(intersectPoint1_found==false)
				{
					check = if_intersects_target(position, obstacle_p2 , target, intersectPoint1,(VisibleRegion[itr]).p1_overT,(VisibleRegion[itr]).p2_overT );
					if(check == true)
					{
						intersectPoint1_found=true;
					}
				}

				else
				{
					check = if_intersects_target(position, obstacle_p2 , target, intersectPoint2,(VisibleRegion[itr]).p1_overT,(VisibleRegion[itr]).p2_overT );
					if(check == true)
					{
						intersectPoint2_found=true;
					}
				}

				
			}

			if( intersect(position, obstacle_p3, obstacle_p2, obstacle_p4)==false &&
				intersect(position, obstacle_p3, obstacle_p1, obstacle_p2)==false)//doesn't intersect with own arm, for point obstacle_p3
			{
				bool check;
				if(intersectPoint1_found==false)
				{
					check = if_intersects_target(position, obstacle_p3 , target, intersectPoint1,(VisibleRegion[itr]).p1_overT,(VisibleRegion[itr]).p2_overT );
					if(check == true)
					{
						intersectPoint1_found=true;
					}
				}

				else if(intersectPoint2_found==false)
				{
					check = if_intersects_target(position, obstacle_p3 , target, intersectPoint2,(VisibleRegion[itr]).p1_overT,(VisibleRegion[itr]).p2_overT );
					if(check == true)
					{
						intersectPoint2_found=true;
					}
				}


			}

			if(intersect(position, obstacle_p4, obstacle_p1, obstacle_p2)==false &&
				intersect(position, obstacle_p4, obstacle_p1, obstacle_p3)==false)//doesn't intersect with own arm, for point obstacle_p4
			{
				bool check;
				if(intersectPoint1_found==false)
				{
					check = if_intersects_target(position, obstacle_p4 , target, intersectPoint1,(VisibleRegion[itr]).p1_overT,(VisibleRegion[itr]).p2_overT );
					if(check == true)
					{
						intersectPoint1_found=true;
					}
				}

				else if(intersectPoint2_found==false)
				{
					check = if_intersects_target(position, obstacle_p4 , target, intersectPoint2,(VisibleRegion[itr]).p1_overT,(VisibleRegion[itr]).p2_overT );
					if(check == true)
					{
						intersectPoint2_found=true;
					}
				}
			}

			VisibleRegionOverT v;
			if(Distance(VisibleRegion[itr].p1_overT, intersectPoint1) < Distance(VisibleRegion[itr].p1_overT, intersectPoint2))
			{
				// edit visibility and construct new segments
				total_visibility -= VisibleRegion[itr].partial_visibility;
				Pointlocation mid1;double angle_with_rect;double visibility_reduce_factor;
				if(intersectPoint1_found==true)
				{
					v.p1_overT=VisibleRegion[itr].p1_overT;
					v.p2_overT=intersectPoint1;

					mid1=mid_of_line_segment(v.p1_overT,v.p2_overT); //rectangle target, so p1,p2 length == p3,p4 length 
					angle_with_rect = angle_between_2_lines(mid1.x,mid1.y,position.x,position.y, v.p1_overT.x,v.p1_overT.y,v.p2_overT.x,v.p2_overT.y);//angle between the line connecting query point and mid of the p1,p2 and the line p1,p2
					visibility_reduce_factor = reduced_visibility_factor4angle(angle_with_rect);

					v.partial_visibility = Distance(VisibleRegion[itr].p1_overT, intersectPoint1)*visibility_reduce_factor;
					total_visibility +=v.partial_visibility;
					temp.push_back(v);
				}
				if(intersectPoint2_found==true)
				{
					v.p1_overT= VisibleRegion[itr].p2_overT;
					v.p2_overT=intersectPoint2;

					mid1=mid_of_line_segment(v.p1_overT,v.p2_overT); 
					angle_with_rect = angle_between_2_lines(mid1.x,mid1.y,position.x,position.y, v.p1_overT.x,v.p1_overT.y,v.p2_overT.x,v.p2_overT.y);//angle between the line connecting query point and mid of the p1,p2 and the line p1,p2
					visibility_reduce_factor = reduced_visibility_factor4angle(angle_with_rect);

					v.partial_visibility = Distance(VisibleRegion[itr].p2_overT, intersectPoint2)*visibility_reduce_factor;

					total_visibility +=v.partial_visibility;
					temp.push_back(v);
				}
			}

			else
			{
				// edit visibility and construct new segments
				total_visibility -= VisibleRegion[itr].partial_visibility;
				Pointlocation mid1;double angle_with_rect;double visibility_reduce_factor;
				if(intersectPoint1_found==true)
				{
					v.p1_overT=VisibleRegion[itr].p1_overT;
					v.p2_overT=intersectPoint2;

					mid1=mid_of_line_segment(v.p1_overT,v.p2_overT); //rectangle target, so p1,p2 length == p3,p4 length 
					angle_with_rect = angle_between_2_lines(mid1.x,mid1.y,position.x,position.y, v.p1_overT.x,v.p1_overT.y,v.p2_overT.x,v.p2_overT.y);//angle between the line connecting query point and mid of the p1,p2 and the line p1,p2
					visibility_reduce_factor = reduced_visibility_factor4angle(angle_with_rect);

					v.partial_visibility = Distance(VisibleRegion[itr].p1_overT, intersectPoint2)*visibility_reduce_factor;

					total_visibility +=v.partial_visibility;
				
					temp.push_back(v);
				}
				if(intersectPoint1_found==true)
				{
					v.p1_overT= VisibleRegion[itr].p2_overT;
					v.p2_overT=intersectPoint1;
					mid1=mid_of_line_segment(v.p1_overT,v.p2_overT); //rectangle target, so p1,p2 length == p3,p4 length 
					angle_with_rect = angle_between_2_lines(mid1.x,mid1.y,position.x,position.y, v.p1_overT.x,v.p1_overT.y,v.p2_overT.x,v.p2_overT.y);//angle between the line connecting query point and mid of the p1,p2 and the line p1,p2
					visibility_reduce_factor = reduced_visibility_factor4angle(angle_with_rect);

					v.partial_visibility = Distance(VisibleRegion[itr].p2_overT, intersectPoint1)*visibility_reduce_factor;

					total_visibility +=v.partial_visibility;
					temp.push_back(v);
				}
			}
			
			insert_this_segment= false;	
			
		}//end of inside_region elseif 

	
		
		//one value change
		
		else if(p1_overT_changed == true && p2_overT_changed == false)
		{
			one_side_changed=true;
			unchanged_side_val = (VisibleRegion[itr]).p2_overT;
			changed_side_val = (VisibleRegion[itr]).p1_overT;
		}

		else if(p2_overT_changed == true && p1_overT_changed == false)
		{
			one_side_changed= true;
			unchanged_side_val = (VisibleRegion[itr]).p1_overT;
			changed_side_val = (VisibleRegion[itr]).p2_overT;
		}

		if( one_side_changed == true)
		{
			//intersectPoint is initialized in change_vRegion_one_side(...) function
			if(intersect(position, obstacle_p1, obstacle_p3, obstacle_p4)==false &&
			intersect(position, obstacle_p1, obstacle_p2, obstacle_p4)==false)//doesn't intersect with own arm, for point obstacle_p1
			{
				// is position-obstacle_p1 line intersects target? 
				//if so, is the point nearer the unchanged_side_val than the changed_side_val?
				
				p1_seen = change_vRegion_one_side(position, obstacle_p1, target, unchanged_side_val, changed_side_val, intersectPoint, 
					 VisibleRegion[itr]);
				//if so, then change the value of the v.region
				if(p1_seen)
				{
					VisibleRegion[itr].p1_overT= unchanged_side_val;
					VisibleRegion[itr].p2_overT= intersectPoint;
					insert_this_segment= true;
					
					total_visibility -= VisibleRegion[itr].partial_visibility;
					
					Pointlocation mid1=mid_of_line_segment(VisibleRegion[itr].p1_overT,VisibleRegion[itr].p2_overT); 
					double angle_with_rect = angle_between_2_lines(mid1.x,mid1.y,position.x,position.y, VisibleRegion[itr].p1_overT.x,VisibleRegion[itr].p1_overT.y,VisibleRegion[itr].p2_overT.x,VisibleRegion[itr].p2_overT.y);//angle between the line connecting query point and mid of the p1,p2 and the line p1,p2
					double visibility_reduce_factor = reduced_visibility_factor4angle(angle_with_rect);

					VisibleRegion[itr].partial_visibility = Distance(unchanged_side_val,intersectPoint)*visibility_reduce_factor;
					total_visibility += VisibleRegion[itr].partial_visibility;
				}

			}

			if(p1_seen == false && intersect(position, obstacle_p2, obstacle_p1, obstacle_p3)==false &&
				intersect(position, obstacle_p2, obstacle_p3, obstacle_p4)==false)//doesn't intersect with own arm, for point obstacle_p2
			{
				p2_seen = change_vRegion_one_side(position, obstacle_p2, target, unchanged_side_val, changed_side_val, intersectPoint, 
					 VisibleRegion[itr]);
				if(p2_seen)
				{
					VisibleRegion[itr].p1_overT= unchanged_side_val;
					VisibleRegion[itr].p2_overT= intersectPoint;
					insert_this_segment= true;

					total_visibility -= VisibleRegion[itr].partial_visibility;
					Pointlocation mid1=mid_of_line_segment(VisibleRegion[itr].p1_overT,VisibleRegion[itr].p2_overT); 
					double angle_with_rect = angle_between_2_lines(mid1.x,mid1.y,position.x,position.y, VisibleRegion[itr].p1_overT.x,VisibleRegion[itr].p1_overT.y,VisibleRegion[itr].p2_overT.x,VisibleRegion[itr].p2_overT.y);//angle between the line connecting query point and mid of the p1,p2 and the line p1,p2
					double visibility_reduce_factor = reduced_visibility_factor4angle(angle_with_rect);

					VisibleRegion[itr].partial_visibility = Distance(unchanged_side_val,intersectPoint)* visibility_reduce_factor;
					total_visibility += VisibleRegion[itr].partial_visibility;
				}

			}

			if(p1_seen == false && p2_seen == false && intersect(position, obstacle_p3, obstacle_p2, obstacle_p4)==false &&
				intersect(position, obstacle_p3, obstacle_p1, obstacle_p2)==false)//doesn't intersect with own arm, for point obstacle_p3
			{
				p3_seen = change_vRegion_one_side(position, obstacle_p3, target, unchanged_side_val, changed_side_val, intersectPoint, 
					 VisibleRegion[itr]);
				if(p3_seen)
				{
					VisibleRegion[itr].p1_overT= unchanged_side_val;
					VisibleRegion[itr].p2_overT= intersectPoint;
					insert_this_segment= true;

					total_visibility -= VisibleRegion[itr].partial_visibility;
					Pointlocation mid1=mid_of_line_segment(VisibleRegion[itr].p1_overT,VisibleRegion[itr].p2_overT); 
					double angle_with_rect = angle_between_2_lines(mid1.x,mid1.y,position.x,position.y, VisibleRegion[itr].p1_overT.x,VisibleRegion[itr].p1_overT.y,VisibleRegion[itr].p2_overT.x,VisibleRegion[itr].p2_overT.y);//angle between the line connecting query point and mid of the p1,p2 and the line p1,p2
					double visibility_reduce_factor = reduced_visibility_factor4angle(angle_with_rect);

					VisibleRegion[itr].partial_visibility = Distance(unchanged_side_val,intersectPoint)*visibility_reduce_factor;
					total_visibility += VisibleRegion[itr].partial_visibility;
				}
			}

			if(p1_seen == false && p2_seen == false && p3_seen == false && intersect(position, obstacle_p4, obstacle_p1, obstacle_p2)==false &&
				intersect(position, obstacle_p4, obstacle_p1, obstacle_p3)==false)//doesn't intersect with own arm, for point obstacle_p4
			{
				p4_seen = change_vRegion_one_side(position, obstacle_p4, target, unchanged_side_val, changed_side_val, intersectPoint, 
					 VisibleRegion[itr]);
				if(p4_seen)
				{
					VisibleRegion[itr].p1_overT= unchanged_side_val;
					VisibleRegion[itr].p2_overT= intersectPoint;
					insert_this_segment= true;

					total_visibility -= VisibleRegion[itr].partial_visibility;
					Pointlocation mid1=mid_of_line_segment(VisibleRegion[itr].p1_overT,VisibleRegion[itr].p2_overT); 
					double angle_with_rect = angle_between_2_lines(mid1.x,mid1.y,position.x,position.y, VisibleRegion[itr].p1_overT.x,VisibleRegion[itr].p1_overT.y,VisibleRegion[itr].p2_overT.x,VisibleRegion[itr].p2_overT.y);//angle between the line connecting query point and mid of the p1,p2 and the line p1,p2
					double visibility_reduce_factor = reduced_visibility_factor4angle(angle_with_rect);

					VisibleRegion[itr].partial_visibility = Distance(unchanged_side_val,intersectPoint)*visibility_reduce_factor;
					total_visibility += VisibleRegion[itr].partial_visibility;
				}
			}
		}//end of if one side

		
		if(insert_this_segment == true) temp.push_back(VisibleRegion[itr]); 
	}
    VisibleRegion=temp;
}



class CompareRect 
{
    public:
    bool operator()(const Rectangle2& t1, const Rectangle2& t2) 
    {
		if(t1.upper_left.x<t2.upper_left.x) return true;
		return false;
    }
};


class CompareBox 
{
    public:
    bool operator()(const Box2& t1, const Box2& t2) 
    {
		// if (t1.a.x < t2.a.x) return true;
		//return false;
		return t1.a < t2.a && t1.b < t2.b;
    }
};

class CompareVisibility 
{
    public:
    bool operator()(QueryPoint& t1, QueryPoint& t2) // descending order
    {
		if(t2.total_visibility <= t1.total_visibility) return false;
		return true;
    }
};

class CompareVisibility3D 
{
    public:
    bool operator()(QueryPoint3D& t1, QueryPoint3D& t2) // descending order
    {
		if(t2.total_visibility<= t1.total_visibility ) return false;
		return true;
    }
};



Rectangle2 HeapEntry_to_rectangle(HeapEntry *he)
{
	Rectangle2 obj;
	float leftx,rightx,topy,lowy;	
	leftx = min(he->x1,he->x2);
	rightx = max(he->x1,he->x2);
	topy = max(he->y1,he->y2);
	lowy = min(he->y1,he->y2);
	
	obj.upper_left.x=leftx;
	obj.upper_left.y=topy;
	obj.lower_right.x=rightx;
	obj.lower_right.y=lowy;
	return obj;

}


Box2 HeapEntry_to_box(HeapEntry *he)
{
	Box2 obj;
	obj.a.x = he->x1;
	obj.b.x = he->x2;
	obj.a.y = he->y1;
	obj.b.y =he->y2;
	obj.a.z = he->z1;
	obj.b.z = he->z2;
	return obj;

}

bool Rectangle2::operator==(Rectangle2 a)
{
	return (upper_left.x==a.upper_left.x) && (upper_left.y==a.upper_left.y) && (lower_right.x==a.lower_right.x) && (lower_right.y==a.lower_right.y);
}

bool Point3D::operator==(Point3D a)
{
	return (a.x == x && a.y == y && a.z == z);
}

bool polygon::operator==(polygon poly)
{
	if(sides.size() != poly.sides.size()) return false;
	for(int i=0;i<poly.sides.size();i++)
	{
		bool found = false;
		for(int j=0; j<sides.size();j++)
		{
			if(poly.sides[i] == sides[j])
			{
				found == true; break;
			}
	
		}
		if(found == false) return false;

	}
	return true;
}
bool line::operator==(line l)
{
	return (l.a == a && l.b == b);
}

int relative_oct_rectangles(Rectangle2 a, Rectangle2 b)
{
	int oct=0;
	if(a.lower_right.x<=b.upper_left.x)
	{
		if(a.lower_right.y>=b.upper_left.y)
		{
			oct=0;
		}
		else if(a.upper_left.y<=b.lower_right.y)
		{
			oct=6;
		}
		else 
		{
			oct=7;
		}
	}
	else if(a.upper_left.x >= b.lower_right.x)
	{
		if(a.lower_right.y>=b.upper_left.y)
		{
			oct=2;
		}
		else if(a.upper_left.y<=b.lower_right.y)
		{
			oct=4;
		}
		else
		{
			oct=3;
		}
	}
	else if(a.lower_right.y>=b.upper_left.y)
	{
		oct=1;
	}
	else if(a.upper_left.y<=b.lower_right.y)
	{
		oct=5;
	}
	else return -1;//overlapping rect
	return oct;
	
}

double MinDistBetweenRect(Rectangle2 a, Rectangle2 b)
{
	Rectangle1 a1;
	a1.x1 = a.upper_left.x;
	a1.x2 = a.lower_right.x;
	a1.y1 = a.lower_right.y;
	a1.y2 = a.upper_left.y;

	float *bounces =(float *)(malloc(sizeof(float)*4));
	*bounces = b.upper_left.x; bounces++;
	*bounces = b.lower_right.x; bounces++;
	*bounces = b.lower_right.y; bounces++;
	*bounces = b.upper_left.y; 
	bounces--;bounces--;bounces--;
	
	float dist = MINRECTDIST(a1,bounces);
	free(bounces);
	return dist;
	/*int oct=relative_oct_rectangles(a,b);
	if(oct == -1) return 0.0;//rectangles intersect
	Pointlocation p,q;
	//Determin Distance based on Quad
	switch(oct)
	{
		case 0:
			return Distance(a.lower_right,b.upper_left);
			break;
		case 1:
			return (a.lower_right.y-b.upper_left.y);
			break;
		case 2:
			p.x=a.upper_left.x;p.y=a.lower_right.y;
			q.x=b.lower_right.x;q.y=b.upper_left.y;
			return Distance(p,q);
			break;
		case 3:
			return (a.upper_left.x-b.lower_right.x);
			break;
		case 4:
			return distance1(a.upper_left,b.lower_right);
			break;
		case 5:
			return (b.lower_right.y-a.upper_left.y);
			break;
		case 6:
			p.x=a.lower_right.x;
			p.y=a.upper_left.y;
			q.x=b.upper_left.x;
			q.y=b.lower_right.y;
			return distance1(p,q);
			break;
		case 7:
			return (b.upper_left.x-a.lower_right.x);
			break;
		default:
			return 0.0;
	}
	*/
}

float RTree::VCM_visibility(QueryPoint q,Rectangle2 T)
{
	Rectangle2 temp_rect;
	temp_rect.upper_left=temp_rect.lower_right=q.position;
	float edist1 = MinDistBetweenRect(temp_rect, T); // mindist from querypoint to node
	Pointlocation midpoint;
	midpoint.x= (T.lower_right.x+T.upper_left.x)/2.0;
	midpoint.y= (T.lower_right.y+T.upper_left.y)/2.0;
	float angle = angle_between_2_lines(q.position.x,q.position.y,midpoint.x,midpoint.y,T.upper_left.x,T.upper_left.y,T.lower_right.x,T.lower_right.y);
	float visibility = (angle/90.0)*(T.lower_right.x-T.upper_left.x);
	visibility = (2 * atan (visibility/(float)(2*edist1)))*(180.0/PI) ;//in degree
	return visibility;
}

float MindistBetweenBox(Box2 T, Box2 obj);
float RTree::VCM_visibility3D(QueryPoint3D q,Box2 T)
{
	Box2 temp_rect;
	temp_rect.a=temp_rect.b=q.position;
	float edist1 = MindistBetweenBox(temp_rect, T); // mindist from querypoint to node
	Point3D midpoint;
	midpoint.x= (T.a.x+T.b.x)/2.0;
	midpoint.y= (T.a.y+T.b.y)/2.0;
	midpoint.z = (T.a.z+T.b.z)/2.0;
	float angle = angle_between_2_lines(q.position.x,q.position.y,midpoint.x,midpoint.y,T.a.x,T.a.y,T.b.x,T.b.y);
	float visibility = (angle/90.0)*(T.a.x-T.b.x);
	visibility = (2 * atan (visibility/(float)(2*edist1)))*(180.0/PI) ;//in degree
	return visibility;
}


void RTree::MOV(QueryPoint q[], int num_of_query_points, Rectangle2 T,int k, QueryPoint k_answers[],int& page, int& obstacle_considered)
{
	int k_position=0;
	int k_original=k;
	if(k<=0) return;

	int end=0;
	int i,j;
	map<Rectangle2,bool,CompareRect> obstacle_checked;
	priority_queue<QueryPoint, vector<QueryPoint>, CompareVisibility> k_ans; 

	map<int,bool> rnodeTraversed;
	//init a heap that stores the non-leaf entries to be accessed-
	Heap *heap = new Heap();
	heap->init(dimension);

	//priority queue of query points according to their visibility metric val
	priority_queue<QueryPoint, vector<QueryPoint>, CompareVisibility> qp_priority_queue; 

	//initially, the visibility is calculated without considering the obstacles
	//init the priority queue
	for(i=0;i<num_of_query_points;i++)
	{
		q[i].init_visibility(T);
		qp_priority_queue.push(q[i]);	
		//q[i].obstacleList = new Heap();
		//q[i].obstacleList->init(dimension);
	}

	//------------------------------------------------------------
	
	int son = root; //this entry is to be visited next
	float leftx, topy, rightx, lowy;
	//son == -1 means empty && end = 0 means exit

	while (son != -1 && end==0)
	{	
		map<int,bool>::iterator rnodeMapItr;
		rnodeMapItr=rnodeTraversed.find(son);
		if(rnodeMapItr!=rnodeTraversed.end() && rnodeTraversed.find(son)->second==true)
		{
			//traversed before,so do nothing
		}
		else
		{			
			RTNode *rtn = new RTNode(this, son);
			rnodeTraversed[son]=true;
			page++;	
			for (i = 0; i < rtn -> num_entries; i++)
			{
				Rectangle2 nd;
			
				leftx = min(rtn->entries[i].bounces[0] , rtn->entries[i].bounces[1]);
				lowy = min(rtn->entries[i].bounces[2] , rtn->entries[i].bounces[3]);
				rightx = max(rtn->entries[i].bounces[0] , rtn->entries[i].bounces[1]);
				topy=max(rtn->entries[i].bounces[2] , rtn->entries[i].bounces[3]);
				
				Rectangle1 nd_1; 

				nd_1.x1=nd.upper_left.x=leftx;
				nd_1.x2=nd.upper_left.y=topy;
				nd_1.y1=nd.lower_right.x=rightx;
				nd_1.y2=nd.lower_right.y=lowy;

				double edist1;
				//check for all query points, if nd is in the visibility region of any q, insert than into corresponding qp_heap
				for(j=0;j<num_of_query_points;j++)
				{
					
					Rectangle2 temp_rect;
					temp_rect.upper_left=temp_rect.lower_right=q[j].position;
					edist1 = MinDistBetweenRect(temp_rect, nd); // mindist from querypoint to node
					float dist2 = MinDistBetweenRect(T,nd);
					if((edist1 <0.000001 && dist2 <0.000001) || (q[j].IsInVisibleRegion(nd)==true))
					{
						//q and target are inside that mbr, or mbr is inside the visible region
						HeapEntry *h = new HeapEntry();
						h -> key = edist1;//sort the obs_heap according to mindist of obstacle from that qp
						h -> level = rtn -> level;
						h -> son1 = rtn->entries[i].son;
						h-> x1 = nd.upper_left.x;
						h-> x2 = nd.lower_right.x;
						h-> y1 = nd.upper_left.y;
						h-> y2 = nd.lower_right.y;
						(q[j]).obstacleList->insert(h);

						delete h;			

					}
				}
			
			}// end of num_entries's for 
			delete rtn;
		}	
		
		//get next entry from the heap of the top query point--------------------------

		if(qp_priority_queue.empty()==true)
		{
			end=1;
			break;
		}

		QueryPoint current_best_point;// = qp_priority_queue.top();
		//qp_priority_queue.pop();//visibility value change kore abar pore push korte hobe 

		HeapEntry *he = new HeapEntry();
		bool again = true;
		while (again)
		{			
			again = false;
			current_best_point = qp_priority_queue.top();
			qp_priority_queue.pop();
			bool isempty = !current_best_point.obstacleList->remove(he);
			Rectangle2 obj;
			if (isempty)  //heap is empty, current_best_point is the answer
			{
				k_ans.push(current_best_point);
				k--;
				//k_answers[k_position]=current_best_point;
				//k_position++;
				//k--;

				if(k==0) end=1; //end;

				else
				{
					//qp_priority_queue theke best point already popped, so go for the next best point
					again=true;
					//break;
				}


			}
			//else if, check end condition, if reached the target
			////////////////////////////////////////////
			else if((obj=HeapEntry_to_rectangle(he)).equals_to(T))
			{
				//end = 1; 
				k_ans.push(current_best_point);
				k--;
				//k_answers[k_position]=current_best_point;
				//k_position++;
				//k--;

				if(k==0)end = 1; //end;

				else
				{
					//qp_priority_queue theke best point already popped, so go for the next best point
					again=true;
					//break;
				}
			}
			else
			{	
				Rectangle1 p;
				p.x1 = he-> x1;
				p.y1 = he-> y1;
				p.x2 = he-> x2;
				p.y2 = he-> y2;

				obj= HeapEntry_to_rectangle(he);
				//check if obj is already been considered, if yes, don't consider it again;
			
		

				if (he->level == 0) //p is an object ; code for data object
				{
					
					map<Rectangle2,bool,CompareRect>::iterator mapItr;
					mapItr=obstacle_checked.find(obj);
					if(mapItr!=obstacle_checked.end())
					{
						if(obstacle_checked.find(obj)->second==true)
						{
							again=true;
							qp_priority_queue.push(current_best_point);
							continue;
						}
						
					}
					else obstacle_considered++;
					//calculate visibility for current_best_point

					//change visibility region of current_best_point

					if(current_best_point.IsInVisibleRegion(obj)==true)
						current_best_point.update_visibliliyRegion(obj,T);
					//check for all query points, if obj is in visibility region of q[p], 
					//pop q[p],calculate visibility for q[p] and then insert into the priority queue
					
					//change visibility region of q[p]

					int size1=qp_priority_queue.size();
					priority_queue<QueryPoint, vector<QueryPoint>, CompareVisibility> qp_temp; 
					for(int x=0; x<size1; x++)
					{
						QueryPoint qp_check = qp_priority_queue.top();
						qp_priority_queue.pop();
						if(qp_check.IsInVisibleRegion(obj) == true)
						{
							qp_check.update_visibliliyRegion(obj,T);
						}
						qp_temp.push(qp_check);
					}
					qp_priority_queue = qp_temp; 
					
					//mark obj has been checked
					obstacle_checked[obj]=true;

					//get next data  from heap
					again =true;	
					qp_priority_queue.push(current_best_point);
				}
				else //not leaf node
				{
					qp_priority_queue.push(current_best_point);
					son=he->son1;
				}
			}
		}
		
		delete he;
	}
	delete heap;
	/*for(int l=0;l<k_original;l++)
	{
		k_answers[l]=k_ans.top();
		k_ans.pop();
	}*/
}




//alternate approach

bool IsObstacleOutsideQPoint(QueryPoint qp_check,Rectangle2 obj , Rectangle2 T)
{
	Rectangle2 qp_check_rect;
	qp_check_rect.upper_left=qp_check.position;
	qp_check_rect.lower_right=qp_check.position;
	
	if(MinDistBetweenRect(T,obj) > MinDistBetweenRect(qp_check_rect,T))
	{
		return true;
	}
	return false;
}

void RTree::Alternate_approach(QueryPoint q[], int num_of_query_points, Rectangle2 T,int k, QueryPoint k_answers[], int& page,int& obstacle_considered)
{
	int k_position=0;
	int k_original=k;
	if(k<=0) return;

	int end=0;
	int i,j;
	//init a heap that stores the non-leaf entries to be accessed-
	Heap *heap = new Heap();
	heap->init(dimension);

	//priority queue of query points according to their visibility metric val
	priority_queue<QueryPoint, vector<QueryPoint>, CompareVisibility> qp_priority_queue; 
	priority_queue<QueryPoint, vector<QueryPoint>, CompareVisibility> k_ans; 
	//initially, the visibility is calculated without considering the obstacles
	//init the priority queue
	for(i=0;i<num_of_query_points;i++)
	{
		q[i].init_visibility(T);
		qp_priority_queue.push(q[i]);		
	}
	
	//------------------------------------------------------------
	
	int son = root; //this entry is to be visited next
	float leftx, topy, rightx, lowy;
	//son == -1 means empty && end = 0 means exit

	while (son != -1 && end==0)
	{	
		RTNode *rtn = new RTNode(this, son);
		page++;
		for (i = 0; i < rtn -> num_entries; i++)
		{
			Rectangle2 nd;
			nd.upper_left.x=leftx = min(rtn->entries[i].bounces[0] , rtn->entries[i].bounces[1]);
			nd.lower_right.y=lowy = min(rtn->entries[i].bounces[2] , rtn->entries[i].bounces[3]);
			nd.lower_right.x=rightx = max(rtn->entries[i].bounces[0] , rtn->entries[i].bounces[1]);
			nd.upper_left.y=topy=max(rtn->entries[i].bounces[2] , rtn->entries[i].bounces[3]);
			
			double edist1=0.0;
			edist1 = MinDistBetweenRect(T, nd); // mindist from target to node
			HeapEntry *h = new HeapEntry();
			h -> key = edist1;//sort the obs_heap according to mindist of obstacle from that qp
			h -> level = rtn -> level;
			h -> son1 = rtn->entries[i].son;
			
			h-> x1 = nd.upper_left.x;
			h-> x2 = nd.lower_right.x;
			h-> y1 = nd.upper_left.y;
			h-> y2 = nd.lower_right.y;
			heap->insert(h);
			delete h;			

		}// end of num_entries's for 
	
		delete rtn;
		
		//get next entry from the heap 

		HeapEntry *he = new HeapEntry();
		bool again = true;
		while (again)
		{			
			again = false;
			
			bool isempty = !heap->remove(he);
			Rectangle2 obj;
			if (isempty)  //heap is empty, no  obstacles left
			{
				while(k>0)
				{
					k_ans.push(qp_priority_queue.top());
					qp_priority_queue.pop();
					k--;
					//k_answers[k_position]=qp_priority_queue.top();
					//qp_priority_queue.pop();
					//k_position++;
					//k--;
				}
				if(k==0)end = 1; //end;
				break;
				
			}
			////////////////////////////////////////////
			if(k==0)
			{
				end = 1;
				break;
			}
			else
			{	
				
				obj= HeapEntry_to_rectangle(he);

				if (he->level == 0) //p is an object ; code for data object
				{
					
					//check for all query points, if obj is in visibility region of q[p], 
					//pop q[p],calculate visibility for q[p] and then insert into the priority queue
					
					//change visibility region of q[p]
					obstacle_considered++;
					int size1=qp_priority_queue.size();
					priority_queue<QueryPoint, vector<QueryPoint>, CompareVisibility> qp_temp; 
					int Topflag = 0;
					for(int x=0; x<size1; x++)
					{
						QueryPoint qp_check = qp_priority_queue.top();
						qp_priority_queue.pop();

						bool AllObsCheckedForQ = IsObstacleOutsideQPoint(qp_check,obj,T);
						if(AllObsCheckedForQ==true && Topflag ==0)
						{
							k_ans.push(qp_check);
							k--;
							//k_answers[k_position]=qp_check;
							//k_position++;
							//k--;

							if(k==0)
							{
								end = 1; //end;
								break;
							}
							continue; //don't push back in qp_priority_queue
						}

						else
						{
							Topflag =1;
							if( AllObsCheckedForQ == false && qp_check.IsInVisibleRegion(obj) == true)
							{
								qp_check.update_visibliliyRegion(obj,T);
						//		printf("visibility region changed for (%f,%f), (%f,%f) ",obj.upper_left.x,obj.upper_left.y,obj.lower_right.x,obj.lower_right.y);
							}
						}
						
						qp_temp.push(qp_check);
					}
					
					qp_priority_queue=qp_temp; 
					//get next data  from heap
					again =true;	
					
				}
				else //not leaf node
				{
					son=he->son1;
				}
			}
		}
		
		delete he;
	}
	delete heap;

	/*for(int l=0;l<k_original;l++)
	{
		k_answers[l]=k_ans.top();
		k_ans.pop();
	}*/
}


//obstacle centric
float obstacleAffectsRegion_val(QueryPoint q, Rectangle2 obs, Rectangle2 target)
{
	QueryPoint temp = q;
	temp.update_visibliliyRegion(obs,target);
	return  temp.total_visibility - q.total_visibility; //order is important, for ascending order, the returned value should be -ve
}




void RTree::MOV_obs(QueryPoint q[], int num_of_query_points, Rectangle2 T,int k, QueryPoint k_answers[],int& page, int& obstacle_considered)
{
	int k_position=0;
	int k_original=k;
	if(k<=0) return;

	int end=0;
	int i,j;
	map<Rectangle2,bool,CompareRect> obstacle_checked;
	priority_queue<QueryPoint, vector<QueryPoint>, CompareVisibility> k_ans; 

	map<int,bool> rnodeTraversed;
	//init a heap that stores the non-leaf entries to be accessed-
	Heap *heap = new Heap();
	heap->init(dimension);

	//priority queue of query points according to their visibility metric val
	priority_queue<QueryPoint, vector<QueryPoint>, CompareVisibility> qp_priority_queue; 

	//initially, the visibility is calculated without considering the obstacles
	//init the priority queue
	for(i=0;i<num_of_query_points;i++)
	{
		q[i].init_visibility(T);
		qp_priority_queue.push(q[i]);	
	}

	//------------------------------------------------------------
	
	int son = root; //this entry is to be visited next
	float leftx, topy, rightx, lowy;
	//son == -1 means empty && end = 0 means exit

	while (son != -1 && end==0)
	{	
		map<int,bool>::iterator rnodeMapItr;
		rnodeMapItr=rnodeTraversed.find(son);
		if(rnodeMapItr!=rnodeTraversed.end() && rnodeTraversed.find(son)->second==true)
		{
			//traversed before,so do nothing
		}
		else
		{			
			RTNode *rtn = new RTNode(this, son);
			rnodeTraversed[son]=true;
			page++;	
			for (i = 0; i < rtn -> num_entries; i++)
			{
				Rectangle2 nd; Rectangle1 nd_1; 
			
				nd_1.x1=nd.upper_left.x=leftx = min(rtn->entries[i].bounces[0] , rtn->entries[i].bounces[1]);
				nd_1.y2=nd.lower_right.y=lowy = min(rtn->entries[i].bounces[2] , rtn->entries[i].bounces[3]);
				nd_1.y1=nd.lower_right.x=rightx = max(rtn->entries[i].bounces[0] , rtn->entries[i].bounces[1]);
				nd_1.x2=nd.upper_left.y=topy=max(rtn->entries[i].bounces[2] , rtn->entries[i].bounces[3]);
				
				double affectAmount,edist1;
				//check for all query points, if nd is in the visibility region of any q, insert than into corresponding qp_heap
				for(j=0;j<num_of_query_points;j++)
				{
					Rectangle2 temp_rect;
					temp_rect.upper_left=temp_rect.lower_right=q[j].position;
					edist1 = MinDistBetweenRect(temp_rect, nd); // mindist from querypoint to node
					float dist2 = MinDistBetweenRect(T,nd);

					affectAmount = obstacleAffectsRegion_val(q[j], nd, T);
					
					if((edist1 <0.0001 && dist2 <0.0001) || (q[j].IsInVisibleRegion(nd)==true))
					{
						//q and target are inside that mbr, or mbr is inside the visible region
						HeapEntry *h = new HeapEntry();
						h -> key = affectAmount;//sort the obs_heap according to the amount of visibility affected by that mbr, high to low val
						h -> level = rtn -> level;
						h -> son1 = rtn->entries[i].son;
						h-> x1 = nd.upper_left.x;
						h-> x2 = nd.lower_right.x;
						h-> y1 = nd.upper_left.y;
						h-> y2 = nd.lower_right.y;
						(q[j]).obstacleList->insert(h);

						delete h;			

					}
				}
			
			}// end of num_entries's for 
			delete rtn;
		}	
		
		//get next entry from the heap of the top query point--------------------------

		if(qp_priority_queue.empty()==true)
		{
			end=1;
			break;
		}

		QueryPoint current_best_point;// = qp_priority_queue.top();

		HeapEntry *he = new HeapEntry();
		bool again = true;
		while (again)
		{			
			again = false;
			current_best_point = qp_priority_queue.top();
			qp_priority_queue.pop();
			bool isempty = !current_best_point.obstacleList->remove(he);
			Rectangle2 obj;
			if (isempty)  //heap is empty, current_best_point is the answer
			{
				k_ans.push(current_best_point);
				k--;
				
				if(k==0) end=1; //end;

				else
				{
					//qp_priority_queue theke best point already popped, so go for the next best point
					again=true;
					//break;
				}


			}
			//else if, check end condition, if reached the target
			else if((obj=HeapEntry_to_rectangle(he)).equals_to(T))
			{
				k_ans.push(current_best_point);
				k--;
				
				if(k==0)end = 1; //end;
				else
				{
					//qp_priority_queue theke best point already popped, so go for the next best point
					again=true;
					//break;
				}
			}
			else
			{	
				Rectangle1 p;
				p.x1 = he-> x1;
				p.y1 = he-> y1;
				p.x2 = he-> x2;
				p.y2 = he-> y2;

				obj= HeapEntry_to_rectangle(he);
				//check if obj is already been considered, if yes, don't consider it again;
			
				if (he->level == 0) //p is an object ; code for data object
				{
					map<Rectangle2,bool,CompareRect>::iterator mapItr;
					mapItr=obstacle_checked.find(obj);
					if(mapItr!=obstacle_checked.end())
					{
						if(obstacle_checked.find(obj)->second==true)
						{
							again=true;
							qp_priority_queue.push(current_best_point);
							continue;
						}
						
					}
					else obstacle_considered++;
					//calculate visibility for current_best_point

					//change visibility region of current_best_point

					if(current_best_point.IsInVisibleRegion(obj)==true)
						current_best_point.update_visibliliyRegion(obj,T);
					//check for all query points, if obj is in visibility region of q[p], 
					//pop q[p],calculate visibility for q[p] and then insert into the priority queue
					
					//change visibility region of q[p]

					int size1=qp_priority_queue.size();
					priority_queue<QueryPoint, vector<QueryPoint>, CompareVisibility> qp_temp; 
					for(int x=0; x<size1; x++)
					{
						QueryPoint qp_check = qp_priority_queue.top();
						qp_priority_queue.pop();
						if(qp_check.IsInVisibleRegion(obj) == true)
						{
							qp_check.update_visibliliyRegion(obj,T);
						}
						qp_temp.push(qp_check);
					}
					qp_priority_queue = qp_temp; 
					
					//mark obj has been checked
					obstacle_checked[obj]=true;

					//get next data  from heap
					again =true;	
					qp_priority_queue.push(current_best_point);
				}
				else //not leaf node
				{
					qp_priority_queue.push(current_best_point);
					son=he->son1;
				}
			}
		}
		
		delete he;
	}
	delete heap;
	
}



//3D MOv
//3D
Point3D OriginShift(Point3D origin, Point3D b)
{
	Point3D newPoint;
	newPoint.x = b.x - origin.x;
	newPoint.y = b.y - origin.y;
	newPoint.z = b.z - origin.z;
	return newPoint;
}
bool isDeterminant0(Point3D b,Point3D c, Point3D d)
{
	if((b.x* ((c.y*d.z) - (d.y*c.z)) - b.y* ((c.x*d.z) - (d.x*c.z)) + b.z* ((c.x*d.y) - (d.x*c.y))) == 0)
	{
		return true;
	}
	else return false;
}

bool intersectPoint(Point3D a,Point3D b, Point3D c, Point3D d, Point3D &intersectPoint )
{
	float t,s;
	float s_d = a.x*b.y + b.x*c.y + c.x*a.y - a.y*b.x - b.y*c.x - c.y*a.x;
	float n = (b.y-a.y)*(d.x-c.x) -(b.x-a.x)*(d.y-c.y);
	if(n!=0)
		s=(float)s_d/n;
	else 
		s=1;

	float t_d= a.x*d.y - a.y*d.x + a.y*c.x - a.x*c.y + c.y*d.x - c.x*d.y;
	if(n!=0)
		t = (float)t_d/n;
	else 
		t= 1;
	//check if t,s holds for z
	float z1 = a.z + t*(b.z-a.z);
	float z2 = c.z + s*(d.z-c.z);
	if(abs(z1-z2)<0.00001)
	{
		intersectPoint.x = a.x + t*(b.x-a.x);
		intersectPoint.y = a.y + t*(b.y-a.y);
		intersectPoint.z = a.z + t*(b.z-a.z);
		return true;
	}
	return false;
}

bool isInSamePlane(Point3D a,Point3D b, Point3D c, Point3D d)
{
	//take a as the origin
	b = OriginShift(a,b); 	
	c = OriginShift(a,c);
	d = OriginShift(a,d); 	

	if(isDeterminant0(b,c,d) == true) 
	{
		return true;	
	}
	else return false;
}

Point3D normalVector(Point3D p1,Point3D p2)
{
	Point3D n;
	n.x = p1.y*p2.z - p2.y*p1.z;
	n.y = p2.x*p1.z - p1.x*p2.z;
	n.z = p1.x*p2.y - p2.x*p1.y;
	return n;
}

Point3D unitNormalVector(Point3D p1,Point3D p2)
{
	Point3D n = normalVector(p1,p2); 
	//unit normal vector
	Point3D n1;
	float frac = sqrt(n.x*n.x + n.y*n.y + n.z*n.z);
	if(frac == 0){n1.x=n1.y=n1.z=0.0;}
	else
	{
		n1.x = (float)n.x/frac;
		n1.y = (float)n.y/frac;
		n1.z = (float)n.z/frac;
	}
	return n1;
}

float dotProductVal(Point3D a, Point3D b)
{
	float dot_val = a.x*b.x + a.y*b.y + a.z*b.z;
	return dot_val;
}

Point3D dotProduct(Point3D a, Point3D b)
{
	Point3D dot;
	dot.x = a.x*b.x;
	dot.y = a.y*b.y;
	dot.z = a.z*b.z;
	return dot;
}

Point3D crossProduct(Point3D a,Point3D b)
{
	Point3D cross;
	cross.x = a.y*b.z - b.y*a.z;
	cross.y = - a.x*b.z + b.x*a.z;
	cross.z = a.x*b.y - b.x*a.y;
	return cross;
}
float distance3D(Point3D a, Point3D b)
{
	return sqrt((a.x-b.x)*(a.x-b.x) + (a.y-b.y)*(a.y-b.y) + (a.z-b.z)*(a.z-b.z));
}
float angleBetween2Vectors(Point3D n, Point3D l)
{
	float n_val = sqrt(n.x*n.x + n.y*n.y + n.z*n.z);
	float l_val = sqrt(l.x*l.x + l.y*l.y + l.z*l.z);

	float dot_val = dotProductVal(n,l);
	if(n_val ==0 ||l_val==0)return 0;
	float angle = (float)dot_val/(n_val*l_val);
	if (angle>1 || angle <0) angle = 180 - acos(angle) * (180 / PI);
	else angle = acos(angle) * (180/PI);
	return angle;
}
// debug zitu
float angleBetween2VectorsZ(Point3D n, Point3D l)
{
	float n_val = sqrt(n.x*n.x + n.y*n.y + n.z*n.z);
	float l_val = sqrt(l.x*l.x + l.y*l.y + l.z*l.z);

	float dot_val = dotProductVal(n, l);
	if (n_val == 0 || l_val == 0)return 0;
	float angle = (float)dot_val / (n_val*l_val);
	angle = acos(angle) * (180 / PI);
	return angle;
}

Point3D midPointofPlane(Point3D p1, Point3D p2)
{
	Point3D midP;
	midP.x = (float)(p1.x +p2.x) /2;
	midP.y = (float)(p1.y +p2.y) /2;
	midP.z = (float)(p1.z +p2.z) /2;
	return midP;
}
float angleBetweenLine_Plane(Point3D p1,Point3D p2, Point3D p3, Point3D mid, Point3D l2)
{
	//find the unit normal vector of the plane
	Point3D n = unitNormalVector(p1-p3,p2-p3);

	//shift the origin of n to the point l1
	n = OriginShift(mid,n);
	/*l2.x = l2.x-mid.x;
	l2.y = l2.y -mid.y;
	l2.z = l2.z -mid.z;*/
	l2= OriginShift(mid,l2);
	return 90-angleBetween2Vectors(n, l2);
}

bool isPointWithinLineSegment(Point3D l1, Point3D l2, Point3D point)
{
	float minx,miny,minz, maxx,maxy,maxz;
	minx = min(l1.x,l2.x);
	maxx = max(l1.x,l2.x);
	miny = min(l1.y,l2.y);
	maxy = max(l1.y,l2.y);
	minz = min(l1.z,l2.z);
	maxz = max(l1.z,l2.z);
	if(point.x <= maxx && point.x >=minx && point.y <= maxy && point.y >=miny && point.z <= maxz && point.z >=minz)
		return true;
	else return false;
}

bool ifIntersect3D_lineSegment(Point3D a, Point3D b, Point3D c, Point3D d, Point3D &intersect_Point)
{
	Point3D intersectP;
	if(isInSamePlane(a,b,c,d) == false)
	{
		return false;
	}
	
	else if(intersectPoint(a,b,c,d,intersect_Point) ==true)
	{
		intersectP = intersect_Point;
		if(isPointWithinLineSegment(a,b,intersectP) == true && isPointWithinLineSegment(c,d,intersectP) == true)
		{
			return true;
		}
		else return false;
	}
	else
		return false;

}

bool equationOfPlane(Point3D a, Point3D b, Point3D c, Point3D d, Plane2 &p)
{
	if(isInSamePlane(a,b,c,d) == true)
	{
		b = OriginShift(a,b);
		c = OriginShift(a,c);
		d = OriginShift(a,d);

		Point3D n = normalVector(b,d);

		p.A= n.x;
		p.B=n.y;
		p.C=n.z;
		p.D=n.x*a.x +n.y*a.y+n.z*a.z;
		return true;
	}
	return false;
}
Point3D normalVectorOfPlane(Plane2 p)
{
	Point3D n;
	n.x=p.A;
	n.y=p.B;
	n.z=p.C;
	return n;
}
bool intersectionOfLine_Plane(Point3D a, Point3D b, Plane2 p, Point3D &intersectionP)
{
	Point3D n = normalVectorOfPlane(p);
	if(dotProductVal(a,n) == 0) return false; //line and plane are parallel

	float t;
	float t_d= p.D - p.A*a.x - p.B*a.y-p.C*a.z;	// value if we plug point a coords to plain equation
	float t_n = b.x*p.A - a.x*p.A + b.y*p.B -a.y*p.B + b.z*p.C - a.z*p.C;
	
	if(t_n==0) return false;
	t=(float)t_d/t_n;

	if (t < 0) return false;	// we only want to find intersection point that is in front of a->b, discard behind

	intersectionP.x = a.x + t*(b.x-a.x);
	intersectionP.y = a.y + t*(b.y-a.y);
	intersectionP.z = a.z + t*(b.z-a.z);
	return true;
}
float max4(float a,float b,float c, float d)
{
	float maxval;
	if(a>b)
	{
		maxval= a;
	}
	else
	{
		maxval = b;
	}
	if(c>maxval)
	{
		maxval = c;
	}
	if(d>maxval)
	{
		maxval =d;
	}
	return maxval;
}


float min4(float a,float b,float c, float d)
{
	float minval;
	if(a<b)
	{
		minval= a;
	}
	else
	{
		minval = b;
	}
	if(c<minval)
	{
		minval = c;
	}
	if(d<minval)
	{
		minval =d;
	}
	return minval;
}

bool isPointInsideRect(Point3D point, Point3D a, Point3D b,Point3D c,Point3D d)  
{
	float minx,miny,minz, maxx,maxy,maxz;

	minx = min4(a.x,b.x,c.x,d.x);
	maxx = max4(a.x,b.x,c.x,d.x);

	miny = min4(a.y,b.y,c.y,d.y);
	maxy = max4(a.y,b.y,c.y,d.y);

	minz = min4(a.z,b.z,c.z,d.z);
	maxz = max4(a.z,b.z,c.z,d.z);

	if(point.x >= minx && point.x <=maxx && point.y >= miny && point.y <= maxy && point.z >=minz && point.z <=maxz)
	{
		return true;
	}
	return false;

}
bool intersectionOfLineSeg_Rectangle(Point3D l1, Point3D l2, Point3D a,Point3D b,Point3D c,Point3D d, Point3D &intersectionPoint)
{
	Plane2 p;
	Point3D intersectionP; 

	
	if(equationOfPlane(a,b,c,d,p) == true)
	{
		if(intersectionOfLine_Plane(l1,l2,p, intersectionP) == true)
		{
			if(isPointWithinLineSegment(l1,l2,intersectionP) == true)
			{
				if(isPointInsideRect(intersectionP, a,b,c,d) == true)
				{
					intersectionPoint = intersectionP; 
					return true;
				}
			
				
			}
		
		}
	}
	return false;
}

float PolygonArea(polygon poly)
{
	Point3D cross;
	Point3D n;
	float area=0;
	
	if(poly.sides.size() >1)
	{
		if(poly.sides[0].a == poly.sides[0].b)return 0.0;
		n = unitNormalVector(poly.sides[0].a, poly.sides[0].b);//polygon's sides are co-planer any side can be taken
	}
	else
	{
		return 0.0; //no side or just a line
	}

	for(int i=0;i<poly.sides.size();i++)
	{
		cross = crossProduct(poly.sides[i].a,poly.sides[i].b);
		area += dotProductVal(n,cross);
	}
	area = (float)area/2;
	return abs(area);
}

float QueryPoint3D::init_visibility(Box2 t)
{
	Point3D cornerPoints[8];
	float minx,miny,minz, maxx,maxy,maxz;
	
	minx= min(t.a.x,t.b.x);
	maxx = max(t.a.x,t.b.x);

	miny = min(t.a.y,t.b.y);
	maxy = max(t.a.y,t.b.y);

	minz = min(t.a.z,t.b.z);
	maxz = max(t.a.z,t.b.z);
	Box2 target(Point3D(minx,maxy,minz),Point3D(maxx,miny,maxz));	// why change maxy and miny?
	// find 8 cornerpoints from 2
	cornerPoints[0]=target.a;
	cornerPoints[1] = Point3D(target.b.x,target.a.y,target.a.z);
	cornerPoints[2] = Point3D(target.b.x,target.b.y,target.a.z);
	cornerPoints[3] = Point3D(target.a.x,target.b.y,target.a.z);
	cornerPoints[4] = Point3D(target.a.x,target.a.y,target.b.z);
	cornerPoints[5] = Point3D(target.b.x,target.a.y,target.b.z);
	cornerPoints[6] = target.b;
	cornerPoints[7] = Point3D(target.a.x,target.b.y,target.b.z);

	bool visible[8] = {true,true,true,true,true,true,true,true};
	//check which points of target are visibile from q

	BoxPlanes bp[6] = {BoxPlanes(0,1,2,3), BoxPlanes(4,5,6,7), BoxPlanes(1,5,6,2), BoxPlanes(0,4,7,3),
		BoxPlanes(0,4,5,1), BoxPlanes(3,7,6,2)};	// bottom, top, right, left, front, back
	
	Point3D intersectionP;
	/*
	 for each cornerpoint we are checking whether the connecting line to query point would intersect any of the box planes
	 if so, then that cornerpoint is not visible
	*/
	for(int i=0;i<8;i++)
	{
		for(int j=0;j<6;j++)
		{
			// if query point is a corner point of box plane, then of course visible
			// otherwise
			if(bp[j].isCornerInBoxPlanes(i) == false) // box plane is defined in terms of 0, 1, 2 etc. so we are checking if ith point
			{
				if(intersectionOfLineSeg_Rectangle(position,cornerPoints[i], cornerPoints[bp[j].a[0]], cornerPoints[bp[j].a[1]], cornerPoints[bp[j].a[2]], cornerPoints[bp[j].a[3]], intersectionP))
				{
					visible[i]=false;//point i is not visible from queryPoint
				}
			}
		}
		//if(visible[i] == true)
	}

	//check which planes of target are visible
	total_visibility = 0.0;
	for(int j=0;j<6;j++)
	{
		if(visible[bp[j].a[0]] ==true && visible[bp[j].a[1]] ==true && visible[bp[j].a[2]] ==true && visible[bp[j].a[3]] ==true)
		{
			bp[j].visible = true;
			//now calculate visibility for that plane
			Point3D midP = midPointofPlane(cornerPoints[bp[j].a[0]],cornerPoints[bp[j].a[2]]);
			
			float angle = angleBetweenLine_Plane(cornerPoints[bp[j].a[0]],cornerPoints[bp[j].a[2]],cornerPoints[bp[j].a[1]],midP,position);
			float areaReduceFactor = angle/90.0;

			float h = distance3D(cornerPoints[bp[j].a[0]],cornerPoints[bp[j].a[1]]);
			float w = distance3D(cornerPoints[bp[j].a[0]], cornerPoints[bp[j].a[3]]);

			float area = h*w;
			float partial_visibility = area * areaReduceFactor;
			total_visibility += partial_visibility;

			//construct the visiblePlanes
			VisibleRegion3D temp;
			temp.planeId = j;
			temp.boundary = Rectangle3D(cornerPoints[bp[j].a[0]],cornerPoints[bp[j].a[1]], cornerPoints[bp[j].a[2]],cornerPoints[bp[j].a[3]]);
			temp.partial_visibility = partial_visibility;
			visiblePlanes.push_back(temp);
			factor.push_back(areaReduceFactor);
		}
	}
	
	return total_visibility;
}


class ComparePoint3D 
{
    public:
    bool operator()(const Point3D& t1, const Point3D& t2) 
    {
		if(t1.x<t2.x) return true;
		return false;
    }
};
// check if point is inside of polygon for some compval
bool compareMinMax(float point, int compval, bool isMax)
{
	if(isMax == true)
	{
		if(point > compval)return true;
		else return false;
	}
	else 
	{
		if(point < compval)return true;
		else return false;
	}
}

void polyRectUnionPrune(line polyLine, float rectCompval, int xyz, bool isMax, list<line>& polyin, polygon& outerPolygon,
	list<line>&rectLines, map<Point3D,int,ComparePoint3D> &componentNo,	map<Point3D,bool,ComparePoint3D> &intersectionPoints,
	int &componentCount)
{
	float polyA, polyB;
	if(xyz == 0)
	{
		polyA = polyLine.a.x;
		polyB = polyLine.b.x;
	}
	else if(xyz ==1)
	{
		polyA = polyLine.a.y;
		polyB = polyLine.b.y;
	}
	else 
	{
		polyA = polyLine.a.z;
		polyB = polyLine.b.z;
	}

	bool boolA = compareMinMax(polyA,rectCompval,isMax);
	bool boolB = compareMinMax(polyB,rectCompval,isMax);
	{
		if(boolA == true && boolB == true) //both are outside range
		{
			map<Point3D,int,ComparePoint3D>::iterator componentItr;
			map<Point3D,int,ComparePoint3D>::iterator componentItr2;
			componentItr = componentNo.find(polyLine.a);
			componentItr2 = componentNo.find(polyLine.b);
			
			int componentNoA = -1;int componentNoB = -1;

			if(componentItr!=componentNo.end())
			{
				componentNoA = componentNo.find(polyLine.a)->second;
			}

			if(componentItr2!=componentNo.end())
			{
				componentNoB = componentNo.find(polyLine.b)->second;
			}

			if(componentNoA == -1 && componentNoB ==-1)
			{
				componentNo[polyLine.a] = componentCount; 
				componentNo[polyLine.b] = componentCount; 
				componentCount++;
			}
			else if(componentNoA == -1 && componentNoB >-1)
			{
				componentNo[polyLine.a] = componentNoB; 
			}
			else if(componentNoB == -1 && componentNoA >-1)
			{
				componentNo[polyLine.b] = componentNoA; 
			}
			else if(componentNoA<componentNoB)
			{
				componentNo[polyLine.b] = componentNoA; 
				map<Point3D,int,ComparePoint3D>::iterator componentItr;
				for(componentItr = componentNo.begin(); componentItr != componentNo.end(); componentItr++)
				{
					if((*componentItr).second == componentNoB)
					{
						(*componentItr).second = componentNoA;
					}
				}
			}
			else if(componentNoB<componentNoA)
			{
				componentNo[polyLine.a] = componentNoB; 
				map<Point3D,int,ComparePoint3D>::iterator componentItr;
				for(componentItr = componentNo.begin(); componentItr != componentNo.end(); componentItr++)
				{
					if((*componentItr).second == componentNoA)
					{
						(*componentItr).second = componentNoB;
					}
				}
			}

			outerPolygon.sides.push_back(polyLine);
			polyin.remove(polyLine);
		}

		// one point inside, another outside
		else if( (boolA == true && boolB ==false) || (boolA == false && boolB ==true))
		{
			Point3D outpoint,inpoint; Point3D intersectP;
			if(boolA == true) {outpoint = polyLine.a ; inpoint = polyLine.b;}
			else {outpoint = polyLine.b; inpoint = polyLine.a;}

			list<line> recttemp;
			recttemp = rectLines;	// new shadow
			list<line>::iterator rectItr;
			for(rectItr = rectLines.begin();rectItr!=rectLines.end();rectItr++)
			{
				float rectItrValA, rectItrValB;
				if(xyz==0)
				{
					rectItrValA = (*rectItr).a.x;
					rectItrValB = (*rectItr).b.x;
				}
				else if(xyz == 1 )
				{
					rectItrValA = (*rectItr).a.y;
					rectItrValB = (*rectItr).b.y;
				}
				else
				{
					rectItrValA = (*rectItr).a.z;
					rectItrValB = (*rectItr).b.z;
				}
				if(abs(rectItrValA-rectCompval)<0.001 || abs(rectItrValB-rectCompval)<0.001)//equality check for float number
				{
					if(ifIntersect3D_lineSegment((*rectItr).a,(*rectItr).b,polyLine.a,polyLine.b,intersectP) == true)
					{
						outerPolygon.sides.push_back(line(outpoint, intersectP));
						polyin.push_back(line(inpoint,intersectP));
						polyin.remove(polyLine);
						recttemp.push_back(line((*rectItr).a, intersectP));
						recttemp.push_back(line((*rectItr).b, intersectP)); 
						recttemp.remove(*rectItr);
						intersectionPoints[intersectP]=true;

						map<Point3D,int,ComparePoint3D>::iterator componentItr;
						
						componentItr = componentNo.find(outpoint);
						int compOutpoint = -1;
		
						if(componentItr!=componentNo.end())
						{
							compOutpoint = componentNo.find(outpoint)->second;
							componentNo[intersectP] = compOutpoint;
						}
						else 
						{
							componentNo[outpoint] = componentCount; 
							componentNo[intersectP] = componentCount; 
							componentCount++;
						}

						break; 
					}

				}
				
			}
			rectLines= recttemp;
			
		}
		else //both are inside rect, >minx
		{
			//do nothing
		}

	}
}

// r is new shadow, poly is existing invisible region
polygon polyRectUnion(polygon poly, Rectangle3D r, polygon& innerPolygon) //returns the union of polygon and rect,
	//innerPolygon is returned as the intersection polygon of the polygon and rect
{
	
	float minx = min4(r.p1.x, r.p2.x, r.p3.x, r.p4.x);
	float miny = min4(r.p1.y, r.p2.y, r.p3.y, r.p4.y);
	float minz = min4(r.p1.z, r.p2.z, r.p3.z, r.p4.z);

	float maxx = max4(r.p1.x, r.p2.x, r.p3.x, r.p4.x);
	float maxy = max4(r.p1.y, r.p2.y, r.p3.y, r.p4.y);
	float maxz = max4(r.p1.z, r.p2.z, r.p3.z, r.p4.z);

	map<Point3D,int,ComparePoint3D> componentNo;
	
	map<Point3D,bool,ComparePoint3D> intersectionPoints;
	list<line> rectLines;

	rectLines.push_back(line(r.p1,r.p2));
	rectLines.push_back(line(r.p2,r.p3));
	rectLines.push_back(line(r.p3,r.p4));
	rectLines.push_back(line(r.p4,r.p1));

	int componentCount = 0;
	int npoly = poly.sides.size();	// existing inv. region size
	
	polygon outerPolygon;
	
	list<line> polyin;	// polyin initially contains existing inv. region
	for(int i=0;i<poly.sides.size();i++)polyin.push_back(poly.sides[i]);

	for(int i=0;i<npoly;i++)
	{
		//minx
		polyRectUnionPrune(poly.sides[i],minx,0,false,polyin,outerPolygon,rectLines,componentNo,intersectionPoints,componentCount);
		//miny
		polyRectUnionPrune(poly.sides[i],miny,1,false,polyin,outerPolygon,rectLines,componentNo,intersectionPoints,componentCount);
		//minz
		polyRectUnionPrune(poly.sides[i],minz,2,false,polyin,outerPolygon,rectLines,componentNo,intersectionPoints,componentCount);

		//maxx
		polyRectUnionPrune(poly.sides[i],maxx,0,true,polyin,outerPolygon,rectLines,componentNo,intersectionPoints,componentCount);
		//maxy
		polyRectUnionPrune(poly.sides[i],maxy,1,true,polyin,outerPolygon,rectLines,componentNo,intersectionPoints,componentCount);
		//maxz
		polyRectUnionPrune(poly.sides[i],maxz,2,true,polyin,outerPolygon,rectLines,componentNo,intersectionPoints,componentCount);
	
	}//end of for 
	//now, rectLines contains the segments of the rectangle lines
	//polyout contains the segments of the polygon that are in the union for sure

	
	
	for(list<line>::iterator itr = polyin.begin();itr!=polyin.end();itr++)
	{
		innerPolygon.sides.push_back(*itr);
	}

	for(list<line>::iterator itr = rectLines.begin();itr!=rectLines.end();itr++)
	{
		map<Point3D,int,ComparePoint3D>::iterator componentItr,componentItr2;
						
		componentItr = componentNo.find((*itr).a);
		componentItr2 = componentNo.find((*itr).b);
		int comppoint1 = -1; int comppoint2 = -1; 
		
		if(componentItr!=componentNo.end())
		{
			comppoint1 = componentNo.find((*itr).a)->second;
		}
		if(componentItr2!=componentNo.end())
		{
			comppoint2 = componentNo.find((*itr).b)->second;
		}		

		if(comppoint1 == comppoint2)
		{
			innerPolygon.sides.push_back(*itr);
		}
		else
		{
			outerPolygon.sides.push_back(*itr);
		}
	}

	return outerPolygon;
}

double dist(Point3D a, Point3D b) {
	return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2) + pow(a.z - b.z, 2));
}

bool shadow(Point3D q, Box2 obstacle,  Rectangle3D T ,Rectangle3D& shadowOfObs)
{
	Point3D cornerPoints[8];
	float minx,miny,minz, maxx,maxy,maxz, targetminx,targetminy,targetminz,targetmaxx,targetmaxy,targetmaxz;
	
	targetminx = min4(T.p1.x,T.p2.x,T.p3.x,T.p4.x);
	targetminy = min4(T.p1.y,T.p2.y,T.p3.y,T.p4.y);
	targetminz = min4(T.p1.z,T.p2.z,T.p3.z,T.p4.z);

	targetmaxx = max4(T.p1.x,T.p2.x,T.p3.x,T.p4.x);
	targetmaxy = max4(T.p1.y,T.p2.y,T.p3.y,T.p4.y);
	targetmaxz = max4(T.p1.z,T.p2.z,T.p3.z,T.p4.z);

	minx= min(obstacle.a.x,obstacle.b.x);
	maxx = max(obstacle.a.x,obstacle.b.x);

	miny = min(obstacle.a.y,obstacle.b.y);
	maxy = max(obstacle.a.y,obstacle.b.y);

	minz = min(obstacle.a.z,obstacle.b.z);
	maxz = max(obstacle.a.z,obstacle.b.z);
	
	Box2 obs(Point3D(minx,maxy,minz),Point3D(maxx,miny,maxz));

	cornerPoints[0] = obs.a;
	cornerPoints[1] = Point3D(obs.b.x,obs.a.y,obs.a.z);
	cornerPoints[2] = Point3D(obs.b.x,obs.b.y,obs.a.z);
	cornerPoints[3] = Point3D(obs.a.x,obs.b.y,obs.a.z);
	cornerPoints[4] = Point3D(obs.a.x,obs.a.y,obs.b.z);
	cornerPoints[5] = Point3D(obs.b.x,obs.a.y,obs.b.z);
	cornerPoints[6] = obs.b;
	cornerPoints[7] = Point3D(obs.a.x,obs.b.y,obs.b.z);

	bool visible[8] = {true,true,true,true,true,true,true,true};
	//check which points of target are visibile from q

	BoxPlanes bp[6] = {BoxPlanes(0,1,2,3), BoxPlanes(4,5,6,7), BoxPlanes(1,5,6,2), BoxPlanes(0,4,7,3),
		BoxPlanes(0,4,5,1), BoxPlanes(3,7,6,2)};
	
	Point3D intersectionP;
	for(int i=0;i<8;i++)
	{
		for(int j=0;j<6;j++)
		{
			if(bp[j].isCornerInBoxPlanes(i) == false)
			{
				if(intersectionOfLineSeg_Rectangle(q,cornerPoints[i], cornerPoints[bp[j].a[0]], cornerPoints[bp[j].a[1]], cornerPoints[bp[j].a[2]], cornerPoints[bp[j].a[3]], intersectionP))
				{
					visible[i]=false;//point i is not visible from queryPoint
				}
				// debug zitu, if 
				/*if (intersectionOfLineSeg_Rectangle(q, cornerPoints[i], T.p1, T.p2, T.p3, T.p4, intersectionP))
				{
					visible[i] = false;
				}*/

			}
		}
		//if(visible[i] == true)
	}

	Plane2 p;
	equationOfPlane(T.p1, T.p2 ,T.p3, T.p4 ,p);

	int count =0;
	vector<Point3D> shadowObs;

	bool isAnyPointInsideTarget = false;
	bool minset = false;
	for(int i = 0;i<8;i++)
	{
		if(visible[i] ==true)	// obstacle er point ta q theke visible
		{
			if(intersectionOfLine_Plane(q,cornerPoints[i],p, intersectionP) == true)
			{
				 
				
				if (isPointInsideRect(intersectionP, T.p1, T.p2, T.p3, T.p4) && dist(q, intersectionP) >= dist(q, cornerPoints[i])) // debug zitu 
				{
					shadowObs.push_back(intersectionP);
					visible[i] = true;
					isAnyPointInsideTarget = true;
				}
				else visible[i]= false;

				if (!minset){ minset = true; minx = maxx = intersectionP.x; miny = maxy = intersectionP.y; minz = maxz = intersectionP.z; }
				else
				{
					if(intersectionP.x < minx ) minx = intersectionP.x;
					else if(intersectionP.x > maxx ) maxx = intersectionP.x;
					if(intersectionP.y < miny ) miny = intersectionP.y;
					else if(intersectionP.y > maxy ) maxy = intersectionP.y;
					if(intersectionP.z < minz ) minz = intersectionP.z;
					else if(intersectionP.z > maxz ) maxz = intersectionP.z;

				}
				
			}
			
		}
	}//end of for

	if(isAnyPointInsideTarget == true)
		
	{
		if(minx<targetminx)minx=targetminx;
		if(miny<targetminy)miny=targetminy;
		if(minz<targetminz)minz=targetminz;

		if(maxx>targetmaxx)maxx=targetmaxx;
		if(maxy>targetmaxy)maxy=targetmaxy;
		if(maxz>targetmaxz)maxz=targetmaxz;
		
		// debug zitu
		if (minz == maxz) {	// shadow in xy plane
			shadowOfObs.p1 = Point3D(minx, miny, minz);
			shadowOfObs.p2 = Point3D(minx, maxy, maxz);
			shadowOfObs.p3 = Point3D(maxx, maxy, maxz);
			shadowOfObs.p4 = Point3D(maxx, miny, minz);
		}
		else if (miny == maxy) {	// shadow in zx plane
			shadowOfObs.p1 = Point3D(minx, miny, minz);
			shadowOfObs.p2 = Point3D(minx, miny, maxz);
			shadowOfObs.p3 = Point3D(maxx, maxy, maxz);
			shadowOfObs.p4 = Point3D(maxx, maxy, minz);
		}
		else if (minx == maxx) {
			shadowOfObs.p1 = Point3D(minx, miny, minz);
			shadowOfObs.p2 = Point3D(minx, miny, maxz);
			shadowOfObs.p3 = Point3D(maxx, maxy, maxz);
			shadowOfObs.p4 = Point3D(maxx, maxy, minz);
		}
		/*shadowOfObs.p1 = Point3D(minx,miny,minz);
		shadowOfObs.p4 = Point3D(maxx,maxy,maxz);
		shadowOfObs.p2 = Point3D(minx,miny,maxz);
		shadowOfObs.p3 = Point3D(maxx,maxy,minz);*/
		return true;
	}
	
	else
	{
		// debug zitu
		if (minz == maxz) {	// shadow in xy plane
			shadowOfObs.p1 = Point3D(minx, miny, minz);
			shadowOfObs.p2 = Point3D(minx, maxy, maxz);
			shadowOfObs.p3 = Point3D(maxx, maxy, maxz);
			shadowOfObs.p4 = Point3D(maxx, miny, minz);
		}
		else if (miny == maxy) {	// shadow in zx plane
			shadowOfObs.p1 = Point3D(minx, miny, minz);
			shadowOfObs.p2 = Point3D(minx, miny, maxz);
			shadowOfObs.p3 = Point3D(maxx, maxy, maxz);
			shadowOfObs.p4 = Point3D(maxx, maxy, minz);
		}
		else if (minx == maxx) {
			shadowOfObs.p1 = Point3D(minx, miny, minz);
			shadowOfObs.p2 = Point3D(minx, miny, maxz);
			shadowOfObs.p3 = Point3D(maxx, maxy, maxz);
			shadowOfObs.p4 = Point3D(maxx, maxy, minz);
		}
		/*shadowOfObs.p1 = Point3D(minx,miny,minz);
		shadowOfObs.p4 = Point3D(maxx,maxy,maxz);
		shadowOfObs.p2 = Point3D(minx,miny,maxz);
		shadowOfObs.p3 = Point3D(maxx,maxy,minz);*/
	}
	return false;
	
	
}

bool isPolyIntesectsWithRect(polygon poly, Rectangle3D r)
{
	float minx = min4(r.p1.x, r.p2.x, r.p3.x, r.p4.x);
	float miny = min4(r.p1.y, r.p2.y, r.p3.y, r.p4.y);
	float minz = min4(r.p1.z, r.p2.z, r.p3.z, r.p4.z);

	float maxx = max4(r.p1.x, r.p2.x, r.p3.x, r.p4.x);
	float maxy = max4(r.p1.y, r.p2.y, r.p3.y, r.p4.y);
	float maxz = max4(r.p1.z, r.p2.z, r.p3.z, r.p4.z);

	int count = 0;
	for(int i = 0; i< poly.sides.size();i++)
	{
		//check how many sides of the polygon is entirely outside the rectangle boundary
		if(poly.sides[i].a.x < minx && poly.sides[i].b.x < minx)
		{
			count++;
		}
		else if(poly.sides[i].a.x > maxx && poly.sides[i].b.x > maxx)
		{
			count++;
		}
		else if(poly.sides[i].a.y < miny && poly.sides[i].b.y < miny)
		{
			count++;
		}
		else if(poly.sides[i].a.y > maxy && poly.sides[i].b.y > maxy)
		{
			count++;
		}
		else if(poly.sides[i].a.z < minz && poly.sides[i].b.z < minz)
		{
			count++;
		}
		else if(poly.sides[i].a.z > maxz && poly.sides[i].b.z > maxz)
		{
			count++;
		}

	}

	if(count == poly.sides.size()) return false; //if all sides of the poly are outside rect boundary, they don't intersect
	else return true;
}


bool isPointInsidePlanarShape(Point3D point, vector<line> shape)
{
	float angle =0.0;
	for(int i=0;i<shape.size();i++)
	{
		Point3D a,b;
		a = OriginShift(point,shape[i].a);
		b = OriginShift(point,shape[i].b);
		angle += angleBetween2VectorsZ(a,b);
	}
	// debug zitu
	if (abs(angle - 360) < 0.001) return true;
	/*if(abs(angle -180) <0.01)
	{
		return true;
	}*/
	else return false;
}

float MindistBetweenBox(Box2 T, Box2 obj)
{
	float tminx = min(T.a.x,T.b.x);
	float tminy = min(T.a.y,T.b.y);
	float tminz = min(T.a.z,T.b.z);

	float tmaxx = max(T.a.x,T.b.x);
	float tmaxy = max(T.a.y,T.b.y);
	float tmaxz = max(T.a.z,T.b.z);

	float ominx = min(obj.a.x,obj.b.x);
	float ominy = min(obj.a.y,obj.b.y);
	float ominz = min(obj.a.z,obj.b.z);

	float omaxx = max(obj.a.x,obj.b.x);
	float omaxy = max(obj.a.y,obj.b.y);
	float omaxz = max(obj.a.z,obj.b.z);

	bool isOLessThanMin = false;
	if(omaxz < tminz || ominz > tmaxz)
	{
		
		if(omaxz < tminz) 
			isOLessThanMin = true;
		else 
			isOLessThanMin = false;

		if(ominx > tmaxx)
		{
			if(ominy > tmaxy)
			{
				if(isOLessThanMin == true)
					return distance3D(Point3D(ominx,ominy,omaxz), Point3D(tmaxx,tmaxy,tminz));
				else
					return distance3D(Point3D(ominx,ominy,ominz), Point3D(tmaxx,tmaxy,tmaxz));
			}
			else if(omaxy < tminy)
			{
				if(isOLessThanMin == true)
					return distance3D(Point3D(ominx,omaxy,omaxz), Point3D(tmaxx,tminy,tminz));
				else
					return distance3D(Point3D(ominx,omaxy,ominz), Point3D(tmaxx,tminy,tmaxz));
			}
			else
			{
				if(isOLessThanMin == true)
					return sqrt(((tmaxx- ominx)*(tmaxx- ominx)) + ((omaxz-tminz)*(omaxz-tminz)));
				else
					return sqrt(((tmaxx- ominx)*(tmaxx- ominx)) + ((ominz-tmaxz)*(ominz-tmaxz)));
			}

		}//x

		else if(omaxx < tminx)
		{
			if(ominy > tmaxy)
			{
				if(isOLessThanMin == true)
					return distance3D(Point3D(omaxx,ominy,omaxz), Point3D(tminx,tmaxy,tminz));
				else
					return distance3D(Point3D(omaxx,ominy,ominz), Point3D(tminx,tmaxy,tmaxz));
			}
			else if(omaxy < tminy)
			{
				if(isOLessThanMin == true)
					return distance3D(Point3D(omaxx,omaxy,omaxz), Point3D(tminx,tminy,tminz));
				else
					return distance3D(Point3D(omaxx,omaxy,ominz), Point3D(tminx,tminy,tmaxz));
			}
			else
			{
				if(isOLessThanMin == true)
					return sqrt(((tminx- omaxx)*(tminx- omaxx)) + ((omaxz-tminz)*(omaxz-tminz)));
				else
					return sqrt(((tminx- omaxx)*(tminx- omaxx)) + ((ominz-tmaxz)*(ominz-tmaxz)));
			}

		}//x

		else
		{
			if(ominy > tmaxy)
			{
				if(isOLessThanMin == true)
					return sqrt(((tmaxy- ominy)*(tmaxy- ominy)) + ((omaxz-tminz)*(omaxz-tminz)));
				else
					return sqrt(((tmaxy- ominy)*(tmaxy- ominy)) + ((ominz-tmaxz)*(ominz-tmaxz)));
			}
			else if(omaxy < tminy)
			{
				if(isOLessThanMin == true)
					return sqrt(((tminy- omaxy)*(tminy- omaxy)) + ((omaxz-tminz)*(omaxz-tminz)));
				else
					return sqrt(((tminy- omaxy)*(tminy- omaxy)) + ((ominz-tmaxz)*(ominz-tmaxz)));	
			}
			else //plane to plane dist
			{
				if(isOLessThanMin == true)
					return abs(tminz - omaxz);
				else
					return abs(ominz - tmaxz);
				
			}

		}//x
	}//z

	else if(ominx > tmaxx || omaxx < tminx)
	{
		if(omaxx < tminx) 
			isOLessThanMin = true;
		else 
			isOLessThanMin = false;

		if(ominz > tmaxz)
		{
			if(ominy > tmaxy)
			{
				if(isOLessThanMin == true)
					return distance3D(Point3D(omaxx,ominy,ominz), Point3D(tminx,tmaxy,tmaxz));
				else
					return distance3D(Point3D(ominx,ominy,ominz), Point3D(tmaxx,tmaxy,tmaxz));
			}
			else if(omaxy < tminy)
			{
				if(isOLessThanMin == true)
					return distance3D(Point3D(omaxx,omaxy,ominz), Point3D(tminx,tminy,tmaxz));
				else
					return distance3D(Point3D(ominx,omaxy,ominz), Point3D(tmaxx,tminy,tmaxz));
			}
			else
			{
				if(isOLessThanMin == true)
					return sqrt(((tminx- omaxx)*(tminx- omaxx)) + ((ominz-tmaxz)*(ominz-tmaxz)));
				else
					return sqrt(((tmaxx- ominx)*(tmaxx- ominx)) + ((ominz-tmaxz)*(ominz-tmaxz)));
			}

		}//z

		else if(omaxz < tminz)
		{
			if(ominy > tmaxy)
			{
				if(isOLessThanMin == true)
					return distance3D(Point3D(omaxx,ominy,omaxz), Point3D(tminx,tmaxy,tminz));
				else
					return distance3D(Point3D(ominx,ominy,omaxz), Point3D(tmaxx,tmaxy,tminz));
			}
			else if(omaxy < tminy)
			{
				if(isOLessThanMin == true)
					return distance3D(Point3D(omaxx,omaxy,omaxz), Point3D(tminx,tminy,tminz));
				else
					return distance3D(Point3D(ominx,omaxy,omaxz), Point3D(tmaxx,tminy,tminz));
			}
			else
			{
				if(isOLessThanMin == true)
					return sqrt(((tminx- omaxx)*(tminx- omaxx)) + ((omaxz-tminz)*(omaxz-tminz)));
				else
					return sqrt(((tmaxx- ominx)*(tmaxx- ominx)) + ((omaxz-tminz)*(omaxz-tminz)));
			}

		}//z

		else
		{
			if(ominy > tmaxy)
			{
				if(isOLessThanMin == true)
					return sqrt(((tminx- omaxx)*(tminx- omaxx)) + ((ominy-tmaxy)*(ominy-tmaxy)));
				else
					return sqrt(((tmaxx- ominx)*(tmaxx- ominx)) + ((ominy-tmaxy)*(ominy-tmaxy)));
			}
			else if(omaxy < tminy)
			{
				if(isOLessThanMin == true)
					return sqrt(((tminx- omaxx)*(tminx- omaxx)) + ((omaxy-tminy)*(omaxy-tminy)));
				else
					return sqrt(((tmaxx- ominx)*(tmaxx- ominx)) + ((omaxy-tminy)*(omaxy-tminy)));
			}
			else
			{
				if(isOLessThanMin == true)
					return abs(tminx - omaxx);
				else
					return abs(ominx - tmaxx);
			}
		}//z

	}//x

	else if(ominy > tmaxy || omaxy < tminy)
	{
		if(omaxy < tminy) 
			isOLessThanMin = true;
		else 
			isOLessThanMin = false;

		if(ominz > tmaxz)
		{
			if(ominx > tmaxx)
			{
				if(isOLessThanMin == true)
					return distance3D(Point3D(ominx,omaxy,ominz), Point3D(tmaxx,tminy,tmaxz));
				else
					return distance3D(Point3D(ominx,ominy,ominz), Point3D(tmaxx,tmaxy,tmaxz));
			}
			else if( omaxx < tminx)
			{
				if(isOLessThanMin == true)
					return distance3D(Point3D(omaxx,omaxy,ominz), Point3D(tminx,tminy,tmaxz));
				else
					return distance3D(Point3D(omaxx,ominy,ominz), Point3D(tminx,tmaxy,tmaxz));
			}
			else
			{
				if(isOLessThanMin == true)
					return sqrt(((tminy- omaxy)*(tminy- omaxy)) + ((ominz-tmaxz)*(ominz-tmaxz)));
				else
					return sqrt(((tmaxy- ominy)*(tmaxy- ominy)) + ((ominz-tmaxz)*(ominz-tmaxz)));
			}
		}
		else if(omaxz < tminz)
		{
			if(ominx > tmaxx)
			{
				if(isOLessThanMin == true)
					return distance3D(Point3D(ominx,omaxy,omaxz), Point3D(tmaxx,tminy,tminz));
				else
					return distance3D(Point3D(ominx,ominy,omaxz), Point3D(tmaxx,tmaxy,tminz));
			}
			else if( omaxx < tminx)
			{
				if(isOLessThanMin == true)
					return distance3D(Point3D(omaxx,omaxy,omaxz), Point3D(tminx,tminy,tminz));
				else
					return distance3D(Point3D(omaxx,ominy,omaxz), Point3D(tminx,tmaxy,tminz));
			}
			else
			{
				if(isOLessThanMin == true)
					return sqrt(((tminy- omaxy)*(tminy- omaxy)) + ((omaxz-tminz)*(omaxz-tminz)));
				else
					return sqrt(((tmaxy- ominy)*(tmaxy- ominy)) + ((omaxz-tminz)*(omaxz-tminz)));
			}

		}
		else
		{
			if(ominx > tmaxx)
			{
				if(isOLessThanMin == true)
					return sqrt(((tminy- omaxy)*(tminy- omaxy)) + ((ominx-tmaxz)*(ominx-tmaxx)));
				else
					return sqrt(((tmaxy- ominy)*(tmaxy- ominy)) + ((ominx-tmaxx)*(ominx-tmaxx)));
			}
			else if( omaxx < tminx)
			{
				if(isOLessThanMin == true)
					return sqrt(((tminy- omaxy)*(tminy- omaxy)) + ((omaxx-tminx)*(omaxx-tminx)));
				else
					return sqrt(((tmaxy- ominy)*(tmaxy- ominy)) + ((omaxx-tminx)*(omaxx-tminx)));
			}
			else
			{
				if(isOLessThanMin == true)
					return abs(omaxy-tminy);
				else
					return abs(ominy-tmaxy);
			}

		}//z

	}//y
	return 0.0;
}


bool QueryPoint3D::IsInsideVisibleRegion(Box2 T,Box2 obstacle)
{
	Rectangle3D shadowOfObs;
	bool outside =false; 
	//check for all visible planes
	for(int i=0;i<visiblePlanes.size();i++)
	{
		
		//find the shadow of obstacle in that plane
		//....
		//check if the shadow is totally outside that visiblePlane
		//if yes then invisible = true
		
		if(shadow(position,obstacle,visiblePlanes[i].boundary,shadowOfObs) == false)
		{
			
			vector<line> rect;
			rect.push_back(line(shadowOfObs.p1, shadowOfObs.p2));
			rect.push_back(line(shadowOfObs.p2, shadowOfObs.p3));
			rect.push_back(line(shadowOfObs.p3, shadowOfObs.p4));
			rect.push_back(line(shadowOfObs.p4, shadowOfObs.p1));

			//check if totally obstructed by shadow
			if(MindistBetweenBox(T,obstacle) <0.0001)
			{
				return true;
			}
			if(isPointInsidePlanarShape(T.a,rect) == true || isPointInsidePlanarShape(T.b, rect) == true)	// since totally outside hole shadow false 
			{
				return true;
			}
			outside = true;
			continue;
		}
		else
		{
			
		//else, check for all invisible region of that plane
			list<polygon> tempInvisible_parts = visiblePlanes[i].invisible_parts;
			for(list<polygon>::iterator itr = tempInvisible_parts.begin(); itr!= tempInvisible_parts.end();itr++)
			{
				//if intersects, return true
				if(isPolyIntesectsWithRect(*itr,shadowOfObs)==true)
				{
					return true;
				}
				//if doesn't intersect, but a corner point of rect is inside poly, then return false//shadow totally inside

				else 
				{
					if(isPointInsidePlanarShape(shadowOfObs.p1,(*itr).sides) == true)
					{
						return false;
					}

					vector<line> rect;
					rect.push_back(line(shadowOfObs.p1, shadowOfObs.p2));
					rect.push_back(line(shadowOfObs.p2, shadowOfObs.p3));
					rect.push_back(line(shadowOfObs.p3, shadowOfObs.p4));
					rect.push_back(line(shadowOfObs.p4, shadowOfObs.p1));

					//if doesn't intersect, but a point of poly is inside rect, return true
					if(isPointInsidePlanarShape((*itr).sides[0].a, rect) == true)
					{
						return true;
					}
				}
				
				
			}
			outside = false;
			return true;//inside a visiblePlane, and not pruned by any invisible region
		}
	}
	if(outside == true) return false;
}



void QueryPoint3D::update_visibilityRegion(Box2 obstacle, Box2 target)
{
	Rectangle3D shadowOfObs;
	bool outside =false; 
	bool intersectsInvisibleR = false;
	//check for all visible planes
	vector<VisibleRegion3D> tempVisibleRegion;
	vector<double> tempFactor;
	int vsize = visiblePlanes.size();
	for(int i=0;i<vsize;i++)
	{
		
		//find the shadow of obstacle in that plane
		//.... shadowOfObs
		//check if the shadow is totally outside that visiblePlane
		//if yes then invisible = true
		
		if(shadow(position,obstacle,visiblePlanes[i].boundary,shadowOfObs) == false)
		{
			vector<line> rect;
			rect.push_back(line(shadowOfObs.p1, shadowOfObs.p2));
			rect.push_back(line(shadowOfObs.p2, shadowOfObs.p3));
			rect.push_back(line(shadowOfObs.p3, shadowOfObs.p4));
			rect.push_back(line(shadowOfObs.p4, shadowOfObs.p1));

			//check if totally obstructed by shadow
			//Point3D t(6, 5, -5);
			if (isPointInsidePlanarShape(visiblePlanes[i].boundary.p1, rect) && isPointInsidePlanarShape(visiblePlanes[i].boundary.p2, rect)
				&& isPointInsidePlanarShape(visiblePlanes[i].boundary.p3, rect) && isPointInsidePlanarShape(visiblePlanes[i].boundary.p4, rect))	// debug here? don't we need to check all 4 points?
			//if (isPointInsidePlanarShape(t, rect) == true)
			{
				total_visibility -= abs(visiblePlanes[i].partial_visibility);
				continue;
			}

			outside = true;
			
		}
		else //shadowOfObs is in the visiblePlanes[i]
		{
			intersectsInvisibleR = false;
			vector<line> rect;
			rect.push_back(line(shadowOfObs.p1, shadowOfObs.p2));
			rect.push_back(line(shadowOfObs.p2, shadowOfObs.p3));
			rect.push_back(line(shadowOfObs.p3, shadowOfObs.p4));
			rect.push_back(line(shadowOfObs.p4, shadowOfObs.p1));
				
			// check for all invisible region of that plane
			list<polygon> tempInvisible_parts = visiblePlanes[i].invisible_parts;
			for(list<polygon>::iterator itr = tempInvisible_parts.begin(); itr!= tempInvisible_parts.end();itr++)
			{
				polygon innerPolygon, unionPolygon;
				
				if(isPolyIntesectsWithRect(*itr,shadowOfObs)==false)
				{
					if (isPointInsidePlanarShape(shadowOfObs.p1, (*itr).sides) && isPointInsidePlanarShape(shadowOfObs.p2, (*itr).sides) 
						&& isPointInsidePlanarShape(shadowOfObs.p3, (*itr).sides) && isPointInsidePlanarShape(shadowOfObs.p4, (*itr).sides))
					{
						//shadow is totally inside a invisible region. so break and check for next visiblePlane
						intersectsInvisibleR = true;
						break;
					}
					
					//if doesn't intersect, but a point of poly is inside rect, so the Shadow is now a new invisible region
					//so the invisibleregion[j] is deleted
					else if (isPointInsidePlanarShape((*itr).sides[0].a, rect) && isPointInsidePlanarShape((*itr).sides[1].a, rect) && isPointInsidePlanarShape((*itr).sides[2].a, rect) && isPointInsidePlanarShape((*itr).sides[3].a, rect))
					{
						polygon newRect(rect);
						newRect.Polyarea = PolygonArea(polygon(rect));

						float commonArea = (*itr).Polyarea;

						float newarea = -abs(newRect.Polyarea) + abs(commonArea);
						newarea *= factor[i];
						/*total_visibility -= abs(newRect.Polyarea);
						total_visibility += abs(commonArea);*/
						total_visibility += newarea;	// debug zitu

						/*visiblePlanes[i].partial_visibility -= abs(newRect.Polyarea);
						visiblePlanes[i].partial_visibility += abs(commonArea);*/

						visiblePlanes[i].partial_visibility += newarea;
						
						visiblePlanes[i].invisible_parts.push_back(newRect);
						visiblePlanes[i].invisible_parts.remove(*itr);
						outside = false;
						intersectsInvisibleR = true;
					}
				}
				else//intersects
				{
					gpc_polygon existing = { 0, NULL, NULL };
					gpc_vertex *v = new gpc_vertex [itr->sides.size()];
					for (int j = 0; j < itr->sides.size(); ++j) {
						if (visiblePlanes[i].planeId == 0 || visiblePlanes[i].planeId == 1) {
							v[j].x = itr->sides[j].a.x;
							v[j].y = itr->sides[j].a.y;
						}
						else if (visiblePlanes[i].planeId == 2 || visiblePlanes[i].planeId == 3) {
							v[j].x = itr->sides[j].a.y;
							v[j].y = itr->sides[j].a.z;
						}
						else {
							v[j].x = itr->sides[j].a.x;
							v[j].y = itr->sides[j].a.z;
						}
						
						
					}
					gpc_vertex_list tv = { itr->sides.size(), v};
					gpc_add_contour(&existing, &tv, 0);
					//printf("\nExisting inv. region: plane ID %d\n", visiblePlanes[i].planeId);
					for (int m = 0; m < itr->sides.size();++m) {
						//printf("%.2f %.2f\n", existing.contour[0].vertex[m].x, existing.contour[0].vertex[m].y);

					}

					gpc_polygon newShadow = { 0, NULL, NULL };
					v = new gpc_vertex[4];
					
					if (visiblePlanes[i].planeId == 0 || visiblePlanes[i].planeId == 1) {
						v[0].x = shadowOfObs.p1.x; v[0].y = shadowOfObs.p1.y;
						v[1].x = shadowOfObs.p2.x; v[1].y = shadowOfObs.p2.y;
						v[2].x = shadowOfObs.p3.x; v[2].y = shadowOfObs.p3.y;
						v[3].x = shadowOfObs.p4.x; v[3].y = shadowOfObs.p4.y;
					}
					else if (visiblePlanes[i].planeId == 2 || visiblePlanes[i].planeId == 3) {
						v[0].x = shadowOfObs.p1.y; v[0].y = shadowOfObs.p1.z;
						v[1].x = shadowOfObs.p2.y; v[1].y = shadowOfObs.p2.z;
						v[2].x = shadowOfObs.p3.y; v[2].y = shadowOfObs.p3.z;
						v[3].x = shadowOfObs.p4.y; v[3].y = shadowOfObs.p4.z;
					}
					else {
						v[0].x = shadowOfObs.p1.x; v[0].y = shadowOfObs.p1.z;
						v[1].x = shadowOfObs.p2.x; v[1].y = shadowOfObs.p2.z;
						v[2].x = shadowOfObs.p3.x; v[2].y = shadowOfObs.p3.z;
						v[3].x = shadowOfObs.p4.x; v[3].y = shadowOfObs.p4.z;
					}
					
					tv = { 4, v };
					gpc_add_contour(&newShadow, &tv, 0);
					
					//printf("\nNew shadow: plane ID %d\n", visiblePlanes[i].planeId);
					for (int m = 0; m < 4; ++m) {
						//printf("%.2f %.2f\n", newShadow.contour[0].vertex[m].x, newShadow.contour[0].vertex[m].y);

					}

					gpc_polygon common = { 0, NULL, NULL };
					gpc_polygon_clip(GPC_INT, &existing, &newShadow, &common);

					gpc_polygon un = { 0, NULL, NULL };
					gpc_polygon_clip(GPC_UNION, &existing, &newShadow, &un);

					//printf("\nIntersection: plane ID %d\n", visiblePlanes[i].planeId);
					if (common.contour) {
						for (int m = 0; m < common.contour[0].num_vertices; ++m) {
							//printf("%.2f %.2f\n", common.contour[0].vertex[m].x, common.contour[0].vertex[m].y);

						}
					}
					

					//printf("\nUnion: plane ID %d\n", visiblePlanes[i].planeId);
					for (int m = 0; m < un.contour[0].num_vertices; ++m) {
						//printf("%.2f %.2f\n", un.contour[0].vertex[m].x, un.contour[0].vertex[m].y);
						line side; 
						int n = (m + 1) % un.contour[0].num_vertices;
						if (visiblePlanes[i].planeId == 0 || visiblePlanes[i].planeId == 1) {
							side.a.x = un.contour[0].vertex[m].x;
							side.a.y = un.contour[0].vertex[m].y;
							side.a.z = shadowOfObs.p1.z;
							side.b.x = un.contour[0].vertex[n].x;
							side.b.y = un.contour[0].vertex[n].y;
							side.b.z = shadowOfObs.p1.z;

						}
						else if (visiblePlanes[i].planeId == 2 || visiblePlanes[i].planeId == 3) {
							side.a.x = shadowOfObs.p1.x;
							side.a.y = un.contour[0].vertex[m].x;
							side.a.z = un.contour[0].vertex[m].y;
							side.b.x = shadowOfObs.p1.x;
							side.b.y = un.contour[0].vertex[n].x;
							side.b.z = un.contour[0].vertex[n].y;
						}
						else {
							side.a.x = un.contour[0].vertex[m].x;
							side.a.y = shadowOfObs.p1.y;
							side.a.z = un.contour[0].vertex[m].y;
							side.b.x = un.contour[0].vertex[n].x;
							side.b.y = shadowOfObs.p1.y;
							side.b.z = un.contour[0].vertex[n].y;
						}
						unionPolygon.sides.push_back(side);

					}

					//unionPolygon = polyRectUnion(*itr,shadowOfObs,innerPolygon);
					unionPolygon.Polyarea = PolygonArea(unionPolygon);
					//float intersectArea = PolygonArea(innerPolygon);
					//unionPolygon.Polyarea = (*itr).Polyarea + PolygonArea(polygon(rect)) - intersectArea;

					float newarea = -abs(unionPolygon.Polyarea) + abs((*itr).Polyarea);
					newarea *= factor[i];

					/*total_visibility -= abs(unionPolygon.Polyarea);
					total_visibility += abs((*itr).Polyarea);*/

					total_visibility += newarea;

					/*visiblePlanes[i].partial_visibility -= abs(unionPolygon.Polyarea);
					visiblePlanes[i].partial_visibility += abs((*itr).Polyarea);*/

					visiblePlanes[i].partial_visibility += newarea;

					//debug zitu
					vector<line> tempShadow;
					tempShadow.push_back(line(shadowOfObs.p1, shadowOfObs.p2));
					tempShadow.push_back(line(shadowOfObs.p2, shadowOfObs.p3));
					tempShadow.push_back(line(shadowOfObs.p3, shadowOfObs.p4));
					tempShadow.push_back(line(shadowOfObs.p4, shadowOfObs.p1));

					polygon tempShadowPoly(tempShadow);
					tempShadowPoly.Polyarea = PolygonArea(tempShadowPoly);

					//visiblePlanes[i].invisible_parts.push_back(unionPolygon);
					visiblePlanes[i].invisible_parts.push_back(tempShadowPoly);
					//visiblePlanes[i].invisible_parts.remove(*itr);
					
					outside = false;
					intersectsInvisibleR = true;

					gpc_free_polygon(&existing); gpc_free_polygon(&newShadow); gpc_free_polygon(&common); gpc_free_polygon(&un);
				}
			}//end of for
			if(intersectsInvisibleR == false)//a new invisible region of that visible plane
			{
				polygon newRect(rect);
				newRect.Polyarea = PolygonArea(newRect);
				float newarea = -abs(newRect.Polyarea);
				newarea *= factor[i];
				// total_visibility -= abs(newRect.Polyarea);	// debug zitu
				//visiblePlanes[i].partial_visibility -= abs(newRect.Polyarea);	// angle reduce factor??
				total_visibility += newarea;
				visiblePlanes[i].partial_visibility += newarea;
				visiblePlanes[i].invisible_parts.push_back(newRect);		
			}

		}//end of else
		tempVisibleRegion.push_back(visiblePlanes[i]);	// visible plane not affected by obstacle
		tempFactor.push_back(factor[i]);
	}
	visiblePlanes = tempVisibleRegion;
	factor = tempFactor;

}

float obstacleAffectsRegion_val3D(QueryPoint3D q,Box2 nd,Box2 T)
{
	QueryPoint3D temp = q;

	temp.update_visibilityRegion(nd,T);
	return abs(temp.total_visibility - q.total_visibility);
}

void RTree::MOV3D(QueryPoint3D q[], int num_of_query_points, Box2 T,int k, QueryPoint3D k_answers[],int& page, int& obstacle_considered)
{
	int k_position=0;
	int k_original=k;
	if(k<=0) return;

	int end=0;
	int i,j;
	map<Box2,bool,CompareBox> obstacle_checked;
	priority_queue<QueryPoint3D, vector<QueryPoint3D>, CompareVisibility3D> k_ans; 

	map<int,bool> rnodeTraversed;
	//init a heap that stores the non-leaf entries to be accessed-
	Heap *heap = new Heap();
	heap->init(dimension);

	//priority queue of query points according to their visibility metric val
	priority_queue<QueryPoint3D, vector<QueryPoint3D>, CompareVisibility3D> qp_priority_queue; 

	//initially, the visibility is calculated without considering the obstacles
	//init the priority queue
	for(i=0;i<num_of_query_points;i++)
	{
		q[i].init_visibility(T);
		qp_priority_queue.push(q[i]);	
		//q[i].obstacleList = new Heap();
		//q[i].obstacleList->init(dimension);
	}

	//------------------------------------------------------------
	
	int son = root; //this entry is to be visited next
	float leftx, topy, rightx, lowy;
	//son == -1 means empty && end = 0 means exit

	while (son != -1 && end==0)
	{	
		if(qp_priority_queue.empty()==true)
		{
			end=1;
			break;
		}
		map<int,bool>::iterator rnodeMapItr;
		rnodeMapItr=rnodeTraversed.find(son);
		if(rnodeMapItr!=rnodeTraversed.end() && rnodeTraversed.find(son)->second==true)
		{
			//traversed before,so do nothing
		}
		else
		{			
			RTNode *rtn = new RTNode(this, son);
			rnodeTraversed[son]=true;
			page++;	
			for (i = 0; i < rtn -> num_entries; i++)
			{
				Box2 nd;

				nd.a.x = min(rtn->entries[i].bounces[0],rtn->entries[i].bounces[1]);
				nd.b.x = max(rtn->entries[i].bounces[0],rtn->entries[i].bounces[1]);
				nd.a.y = min(rtn->entries[i].bounces[2],rtn->entries[i].bounces[3]);
				nd.b.y = max(rtn->entries[i].bounces[2],rtn->entries[i].bounces[3]);
				nd.a.z = min(rtn->entries[i].bounces[4],rtn->entries[i].bounces[5]);
				nd.b.z = max(rtn->entries[i].bounces[4],rtn->entries[i].bounces[5]);
									
				double edist1;
						
				//check for all query points, if nd is in the visibility region of any q, insert than into corresponding qp_heap
				for(j=0;j<num_of_query_points;j++)
				{
					//edist1 = MINRECTDIST1(q[j].position, nd_1); // mindist from querypoint to node
					Box2 tempQ(q[j].position,q[j].position);
					edist1 = MindistBetweenBox(T,tempQ);// mindist from querypoint to node
					
					
					if((edist1<= 0.001 && MindistBetweenBox(T,nd)<= 0.001) || (q[j].IsInsideVisibleRegion(T,nd)==true) )
					{//q and target are inside that mbr, or mbr is inside the visible region of q

						HeapEntry *h = new HeapEntry();
						h -> key = edist1;//sort the obs_heap according to mindist of obstacle from that qp
					
						h -> level = rtn -> level;
						h -> son1 = rtn->entries[i].son;
			
			
						h-> x1 = nd.a.x;
						h-> x2 = nd.b.x;
						h-> y1 = nd.a.y;
						h-> y2 = nd.b.y;
						h->z1 = nd.a.z;
						h->z2 = nd.b.z;
						
						(q[j]).obstacleList->insert(h);

						delete h;			

					}
				}
			
			}// end of num_entries's for 
			delete rtn;
		}	
		
		
		//get next entry from the heap of the top query point--------------------------

		if(qp_priority_queue.empty()==true)
		{
			end=1;
			break;
		}

		QueryPoint3D current_best_point;// = qp_priority_queue.top();
		
		HeapEntry *he = new HeapEntry();
		bool again = true;
		while (again)
		{	
			again = false;
			if(qp_priority_queue.empty()==true)
			{
				end=1;
				break;
			}
			
			current_best_point = qp_priority_queue.top();
			qp_priority_queue.pop();
			bool isempty = !current_best_point.obstacleList->remove(he);
			Box2 obj;
			if (isempty)  //heap is empty, current_best_point is the answer
			{
				k_ans.push(current_best_point);
				k--;
				
				if(k==0) 
				{
						end=1; //end;
						break;
				}
				else
				{
					//qp_priority_queue theke best point already popped, so go for the next best point
					again=true;
					//break;
				}


			}
			//else if, check end condition, if reached the target
			////////////////////////////////////////////
			else if((obj=HeapEntry_to_box(he)).equals_to(T))
			{
				k_ans.push(current_best_point);
				k--;
				
				if(k==0)
				{
					end = 1; //end;
					break;
				}
				else
				{
					//qp_priority_queue theke best point already popped, so go for the next best point
					again=true;
					//break;
				}
			}
			else
			{	
				
				obj= HeapEntry_to_box(he);
				
				if (he->level == 0) //p is an object ; code for data object
				{
					//check if obj is already been considered, if yes, don't consider it again;
					map<Box2,bool,CompareBox>::iterator mapItr;
					mapItr=obstacle_checked.find(obj);
					if(mapItr!=obstacle_checked.end())
					{
						if(obstacle_checked.find(obj)->second==true)
						{
							again=true;
							qp_priority_queue.push(current_best_point);
							continue;
						}
						
					}
					else obstacle_considered++;
					//calculate visibility for current_best_point

					//change visibility region of current_best_point
					
					if(current_best_point.IsInsideVisibleRegion(T,obj))
						current_best_point.update_visibilityRegion(obj,T); 
					else
					{
						qp_priority_queue.push(current_best_point);
						again = true;
						continue;
					}
					//check for all query points, if obj is in visibility region of q[p], 
					//pop q[p],calculate visibility for q[p] and then insert into the priority queue
					
					//change visibility region of q[p]

					int size1=qp_priority_queue.size();
					priority_queue<QueryPoint3D, vector<QueryPoint3D>, CompareVisibility3D> qp_temp; 
					for(int x=0; x<size1; x++)
					{
						QueryPoint3D qp_check = qp_priority_queue.top();
						qp_priority_queue.pop();
						if(qp_check.IsInsideVisibleRegion(T,obj) == true)
						{
							qp_check.update_visibilityRegion(obj,T);
						}
						qp_temp.push(qp_check);
					}
					qp_priority_queue = qp_temp; 
					
					//mark obj has been checked
					obstacle_checked[obj]=true;

					//get next data  from heap
					again =true;	
					qp_priority_queue.push(current_best_point);
					if(qp_priority_queue.empty()==true)
					{
						again=false;
						end=1;
						break;
					}
				}
				else //not leaf node
				{
					qp_priority_queue.push(current_best_point);
					son=he->son1;
				}
			}
		}
		
		delete he;
	}
	delete heap;
	/*for(int l=0;l<k_original;l++)
	{
		k_answers[l]=k_ans.top();
		k_ans.pop();
	}*/
}


bool IsObstacleOutsideQPoint3D(QueryPoint3D q, Box2 obstacle, Box2 T)
{
/*	float *point= (float*) malloc(sizeof(float)*3);
	*point = q.position.x;
	point++;
	*point = q.position.y;
	point++;
	*point = q.position.z;
	point--;point--;

	float *bounces = (float*) malloc(sizeof(float)*6);
	*bounces = min(T.a.x,T.b.x); bounces++;
	*bounces = max(T.a.x,T.b.x); bounces++;
	*bounces = min(T.a.y,T.b.y); bounces++;
	*bounces = max(T.a.y,T.b.y); bounces++;
	*bounces = min(T.a.z,T.b.z); bounces++;
	*bounces = max(T.a.z,T.b.z); 
	bounces--;bounces--;bounces--;bounces--;bounces--;
	float qdist = MINDIST(point,bounces,3);// mindist from querypoint to target
	free(point);free(bounces);
	float obstacle_dist = MindistBetweenBox(T,obstacle);
	if(obstacle_dist>qdist)return true;
	else return false;
	
	*/
	
	float qdist = MindistBetweenBox(Box2(q.position,q.position),obstacle);
	float obstacle_dist = MindistBetweenBox(T,obstacle);
	if(obstacle_dist>qdist)return true;
	else return false;
}

void RTree::Alternate_approach3D(QueryPoint3D q[], int num_of_query_points, Box2 T,int k, QueryPoint3D k_answers[], int& page,int& obstacle_considered)
{
	int k_position=0;
	int k_original=k;
	if(k<=0) return;

	int end=0;
	int i,j;
	//init a heap that stores the non-leaf entries to be accessed-
	Heap *heap = new Heap();
	heap->init(dimension);

	//priority queue of query points according to their visibility metric val
	priority_queue<QueryPoint3D, vector<QueryPoint3D>, CompareVisibility3D> qp_priority_queue; 
	priority_queue<QueryPoint3D, vector<QueryPoint3D>, CompareVisibility3D> k_ans; 
	//initially, the visibility is calculated without considering the obstacles
	//init the priority queue
	for(i=0;i<num_of_query_points;i++)
	{
		q[i].init_visibility(T);
		qp_priority_queue.push(q[i]);		
	}
	
	//------------------------------------------------------------
	
	int son = root; //this entry is to be visited next
	float leftx, topy, rightx, lowy;
	//son == -1 means empty && end = 0 means exit

	while (son != -1 && end==0)
	{	
		RTNode *rtn = new RTNode(this, son);
		page++;
		for (i = 0; i < rtn -> num_entries; i++)
		{
			Box2 nd;
			nd.a.x = min(rtn->entries[i].bounces[0],rtn->entries[i].bounces[1]);
			nd.b.x = max(rtn->entries[i].bounces[0],rtn->entries[i].bounces[1]);
			nd.a.y = min(rtn->entries[i].bounces[2],rtn->entries[i].bounces[3]);
			nd.b.y = max(rtn->entries[i].bounces[2],rtn->entries[i].bounces[3]);
			nd.a.z = min(rtn->entries[i].bounces[4],rtn->entries[i].bounces[5]);
			nd.b.z = max(rtn->entries[i].bounces[4],rtn->entries[i].bounces[5]);
							
			int size1=qp_priority_queue.size();
			double edist1=0.0;

			
			HeapEntry *h = new HeapEntry();
			h -> key = edist1;//sort the obs_heap according to mindist of obstacle from that qp
			h -> level = rtn -> level;
			h -> son1 = rtn->entries[i].son;
			
			h-> x1 = nd.a.x;
			h-> x2 = nd.b.x;
			h-> y1 = nd.a.y;
			h-> y2 = nd.b.y;
			h->z1 = nd.a.z;
			h->z2 = nd.b.z;
			heap->insert(h);
			delete h;			

			
		}// end of num_entries's for 
	
		delete rtn;
		
		
		//get next entry from the heap 

		HeapEntry *he = new HeapEntry();
		bool again = true;
		while (again)
		{			
			again = false;
			
			bool isempty = !heap->remove(he);
			Box2 obj;
			if (isempty)  //heap is empty, no  obstacles left
			{
				while(k>0)
				{
					k_ans.push(qp_priority_queue.top());
					qp_priority_queue.pop();
					k--;
					
				}
				if(k==0)end = 1; //end;
				break;
				
			}
			////////////////////////////////////////////
			if(k==0)
			{
				end = 1;
				break;
			}
			else
			{	
				
				obj= HeapEntry_to_box(he);

				if (he->level == 0) //p is an object ; code for data object
				{
					
					//check for all query points, if obj is in visibility region of q[p], 
					//pop q[p],calculate visibility for q[p] and then insert into the priority queue
					
					//change visibility region of q[p]
					obstacle_considered++;
					int size1=qp_priority_queue.size();
					priority_queue<QueryPoint3D, vector<QueryPoint3D>, CompareVisibility3D> qp_temp; 
					int Topflag = 0;
					for(int x=0; x<size1; x++)
					{
						QueryPoint3D qp_check = qp_priority_queue.top();
						qp_priority_queue.pop();

						bool AllObsCheckedForQ = IsObstacleOutsideQPoint3D(qp_check,obj,T);
						if(AllObsCheckedForQ==true && Topflag ==0)
						{
							k_ans.push(qp_check);
							k--;
							//k_answers[k_position]=qp_check;
							//k_position++;
							//k--;

							if(k==0)
							{
								end = 1; //end;
								break;
							}
							continue; //don't push back in qp_priority_queue
						}

						else
						{
							Topflag =1;
							if( AllObsCheckedForQ == false && qp_check.IsInsideVisibleRegion(T,obj) == true)
							{
								qp_check.update_visibilityRegion(obj,T);
						//		printf("visibility region changed for (%f,%f), (%f,%f) ",obj.upper_left.x,obj.upper_left.y,obj.lower_right.x,obj.lower_right.y);
							}
						}
						
						qp_temp.push(qp_check);
					}
					
					qp_priority_queue=qp_temp; 
					//get next data  from heap
					again =true;	
					
				}
				else //not leaf node
				{
					priority_queue<QueryPoint3D, vector<QueryPoint3D>, CompareVisibility3D> qp_temp1;
					qp_temp1 = qp_priority_queue;
					int size1 = qp_priority_queue.size();
					double qdist = 0.0;
			
					double edist1 = MindistBetweenBox(T, obj); // mindist from target to node
					//qdist = Min
					int x=0;
					int flag = 0;
					for(x=0; x<size1; x++)
					{
						QueryPoint3D qp_check = qp_temp1.top();
						qp_temp1.pop();
				
						Box2 tempQ(qp_check.position,qp_check.position);
						qdist = MindistBetweenBox(T,tempQ);
				
						if(qp_check.IsInsideVisibleRegion(T,obj) == true || (edist1 <0.001 && qdist<0.001))
						{
							break;
						}
				
					}
			
					if(x == size1) again =true;
			

					if(again==false)son=he->son1;
				}
			}
		}
		
		delete he;
	}
	delete heap;

	/*for(int l=0;l<k_original;l++)
	{
		k_answers[l]=k_ans.top();
		k_ans.pop();
	}*/
}

/// obs

void RTree::MOV_obs3D(vector<QueryPoint3D> &q, int num_of_query_points, Box2 T,int k, vector<QueryPoint3D> &k_answers,int& page, int& obstacle_considered)
{
	int k_position=0;
	int k_original=k;
	if(k<=0) return;

	int end=0;
	int i,j;
	// keeps track of obstacles that are checked, sorted by mindist
	map<Box2,bool,CompareBox> obstacle_checked; // debug zitu
	//map<Box2, bool> obstacle_checked;
	// PQ of query points, vector is the container, priority = visibility, top k
	priority_queue<QueryPoint3D, vector<QueryPoint3D>, CompareVisibility3D> k_ans; 

	map<int,bool> rnodeTraversed;
	//init a heap that stores the non-leaf entries to be accessed-
	Heap *heap = new Heap();
	heap->init(dimension);

	//priority queue of query points according to their visibility metric val
	priority_queue<QueryPoint3D, vector<QueryPoint3D>, CompareVisibility3D> qp_priority_queue; 

	//initially, the visibility is calculated without considering the obstacles
	//init the priority queue
	for(i=0;i<num_of_query_points;i++)
	{
		q[i].init_visibility(T);
		qp_priority_queue.push(q[i]);	
	}
	//return;	// debug
	//------------------------------------------------------------
	// find which obstacles affect which query points
	int son = root; //this entry is to be visited next
	float leftx, topy, rightx, lowy;
	//son == -1 means empty && end = 0 means exit

	while (son != -1 && end==0)
	{	
		map<int,bool>::iterator rnodeMapItr;
		rnodeMapItr=rnodeTraversed.find(son);
		if(rnodeMapItr!=rnodeTraversed.end() && rnodeTraversed.find(son)->second==true)
		{
			//traversed before,so do nothing
		}
		else
		{			
			RTNode *rtn = new RTNode(this, son);
			rnodeTraversed[son]=true;
			page++;	
			for (i = 0; i < rtn -> num_entries; i++)
			{
				Box2 nd;
				nd.a.x = min(rtn->entries[i].bounces[0],rtn->entries[i].bounces[1]);
				nd.b.x = max(rtn->entries[i].bounces[0],rtn->entries[i].bounces[1]);
				nd.a.y = min(rtn->entries[i].bounces[2],rtn->entries[i].bounces[3]);
				nd.b.y = max(rtn->entries[i].bounces[2],rtn->entries[i].bounces[3]);
				nd.a.z = min(rtn->entries[i].bounces[4],rtn->entries[i].bounces[5]);
				nd.b.z = max(rtn->entries[i].bounces[4],rtn->entries[i].bounces[5]);
									
				double edist1,affectAmount;
						
				//check for all query points, if nd is in the visibility region of any q, insert than into corresponding qp_heap
				for(j=0;j<num_of_query_points;j++)
				{
					//edist1 = MINRECTDIST1(q[j].position, nd_1); // mindist from querypoint to node
					Box2 tempQ(q[j].position,q[j].position);
					edist1 = MindistBetweenBox(T,tempQ);// mindist from querypoint to target
										
					affectAmount = obstacleAffectsRegion_val3D(q[j], nd, T);

					if((edist1<= 0.0001 && MindistBetweenBox(T,nd)<= 0.0001) || (q[j].IsInsideVisibleRegion(T,nd)==true) )
					{//q and target are inside that mbr, or mbr is inside the visible region of q

						HeapEntry *h = new HeapEntry();
						h -> key = affectAmount;//sort the obs_heap according to amount of affect
					
						h -> level = rtn -> level;
						h -> son1 = rtn->entries[i].son;
			
			
						h-> x1 = nd.a.x;
						h-> x2 = nd.b.x;
						h-> y1 = nd.a.y;
						h-> y2 = nd.b.y;
						h->z1 = nd.a.z;
						h->z2 = nd.b.z;
						
						(q[j]).obstacleList->insert(h);

						delete h;			

					}
				}
			
			}// end of num_entries's for 
			delete rtn;
		}	
		
		
		//get next entry from the heap of the top query point--------------------------

		if(qp_priority_queue.empty()==true)
		{
			end=1;
			break;
		}

		QueryPoint3D current_best_point;// = qp_priority_queue.top();
		
		HeapEntry *he = new HeapEntry();
		bool again = true;
		while (again)
		{			
			again = false;
			current_best_point = qp_priority_queue.top();
			qp_priority_queue.pop();
			bool isempty = !current_best_point.obstacleList->remove(he);
			Box2 obj;
			if (isempty)  //heap is empty, current_best_point is the answer
			{
				k_ans.push(current_best_point);
				k--;
				
				if(k==0) end=1; //end;
				else
				{
					//qp_priority_queue theke best point already popped, so go for the next best point
					again=true;
					//break;
				}


			}
			//else if, check end condition, if reached the target
			////////////////////////////////////////////
			else if((obj=HeapEntry_to_box(he)).equals_to(T))
			{
				k_ans.push(current_best_point);
				k--;
				
				if(k==0)end = 1; //end;

				else
				{
					//qp_priority_queue theke best point already popped, so go for the next best point
					again=true;
					//break;
				}
			}
			else
			{	
				
				obj= HeapEntry_to_box(he);
				
				if (he->level == 0) //p is an object ; code for data object
				{
					//check if obj is already been considered, if yes, don't consider it again;
					 map<Box2,bool,CompareBox>::iterator mapItr; // debug zitu
					//map<Box2, bool>::iterator mapItr;
					mapItr=obstacle_checked.find(obj);
					if(mapItr!=obstacle_checked.end())
					{
						if(mapItr->second==true)
						{
							again=true;
							qp_priority_queue.push(current_best_point);
							continue;
						}
						
					}
					
					else obstacle_considered++;
					//calculate visibility for current_best_point

					//change visibility region of current_best_point
					
					if(current_best_point.IsInsideVisibleRegion(T,obj))
						current_best_point.update_visibilityRegion(obj,T); 
					else
					{
						qp_priority_queue.push(current_best_point);
						again = true;
						continue;
					}
					//check for all query points, if obj is in visibility region of q[p], 
					//pop q[p],calculate visibility for q[p] and then insert into the priority queue
					
					//change visibility region of q[p]

					int size1=qp_priority_queue.size();
					priority_queue<QueryPoint3D, vector<QueryPoint3D>, CompareVisibility3D> qp_temp; 
					for(int x=0; x<size1; x++)
					{
						QueryPoint3D qp_check = qp_priority_queue.top();
						qp_priority_queue.pop();
						if(qp_check.IsInsideVisibleRegion(T,obj) == true)
						{
							qp_check.update_visibilityRegion(obj,T);
						}
						qp_temp.push(qp_check);
					}
					qp_priority_queue = qp_temp; 
					
					//mark obj has been checked
					obstacle_checked[obj]=true;

					//get next data  from heap
					again =true;	
					qp_priority_queue.push(current_best_point);
				}
				else //not leaf node
				{
					qp_priority_queue.push(current_best_point);
					son=he->son1;
				}
			}
		}
		
		delete he;
	}
	delete heap;
	// debug zitu
	q.clear();
	k_answers.clear();
	for (; !k_ans.empty();) {
		//printf("%d ", k_ans.top());
		k_answers.push_back(k_ans.top());
		q.push_back(k_ans.top());
		k_ans.pop();
	}
	while (!qp_priority_queue.empty()) {
		q.push_back(qp_priority_queue.top()); qp_priority_queue.pop();
	}
}

float update_naiveVisibility(QueryPoint3D q,Box2 obj, Box2 T)
{
	QueryPoint3D temp = q;
	temp.update_visibilityRegion(obj,T);
	return temp.total_visibility;
}


void RTree::naiveMOV3D(QueryPoint3D q[], int num_of_query_points, Box2 T,int k, QueryPoint3D k_answers[],int& page, int& obstacle_considered)
{
	int k_position=0;
	int k_original=k;
	if(k<=0) return;
	int _n= num_of_query_points;
	int end=0;
	int i,j;
	map<Box2,bool,CompareBox> obstacle_checked;
	priority_queue<QueryPoint3D, vector<QueryPoint3D>, CompareVisibility3D> k_ans; 

	map<int,bool> rnodeTraversed;
	//init a heap that stores the non-leaf entries to be accessed-
	Heap *heap = new Heap();
	heap->init(dimension);

	//priority queue of query points according to their visibility metric val
	priority_queue<QueryPoint3D, vector<QueryPoint3D>, CompareVisibility3D> qp_priority_queue; 

	//initially, the visibility is calculated without considering the obstacles
	//init the priority queue
	for(i=0;i<num_of_query_points;i++)
	{
		q[i].init_visibility(T);
		qp_priority_queue.push(q[i]);	
		//q[i].obstacleList = new Heap();
		//q[i].obstacleList->init(dimension);
	}

	//------------------------------------------------------------
	
	int son = root; //this entry is to be visited next
	float leftx, topy, rightx, lowy;
	//son == -1 means empty && end = 0 means exit

	while (son != -1 && end==0)
	{	
		map<int,bool>::iterator rnodeMapItr;
		rnodeMapItr=rnodeTraversed.find(son);
		if(rnodeMapItr!=rnodeTraversed.end() && rnodeTraversed.find(son)->second==true)
		{
			//traversed before,so do nothing
		}
		else
		{			
			RTNode *rtn = new RTNode(this, son);
			rnodeTraversed[son]=true;
			page++;	
			for (i = 0; i < rtn -> num_entries; i++)
			{
				Box2 nd;
				nd.a.x = min(rtn->entries[i].bounces[0],rtn->entries[i].bounces[1]);
				nd.b.x = max(rtn->entries[i].bounces[0],rtn->entries[i].bounces[1]);
				nd.a.y = min(rtn->entries[i].bounces[2],rtn->entries[i].bounces[3]);
				nd.b.y = max(rtn->entries[i].bounces[2],rtn->entries[i].bounces[3]);
				nd.a.z = min(rtn->entries[i].bounces[4],rtn->entries[i].bounces[5]);
				nd.b.z = max(rtn->entries[i].bounces[4],rtn->entries[i].bounces[5]);
									
				double edist1;
						
				//check for all query points, if nd is in the visibility region of any q, insert than into corresponding qp_heap
				for(j=0;j<num_of_query_points;j++)
				{
					//edist1 = MINRECTDIST1(q[j].position, nd_1); // mindist from querypoint to node
					Box2 tempQ(q[j].position,q[j].position);
					edist1 = MindistBetweenBox(T,tempQ);// mindist from querypoint to node
					
					
					if((edist1<= 0.001 && MindistBetweenBox(T,nd)<= 0.001) || (q[j].IsInsideVisibleRegion(T,nd)==true) )
					{//q and target are inside that mbr, or mbr is inside the visible region of q

						HeapEntry *h = new HeapEntry();
						h -> key = edist1;//sort the obs_heap according to mindist of obstacle from that qp
					
						h -> level = rtn -> level;
						h -> son1 = rtn->entries[i].son;
			
			
						h-> x1 = nd.a.x;
						h-> x2 = nd.b.x;
						h-> y1 = nd.a.y;
						h-> y2 = nd.b.y;
						h->z1 = nd.a.z;
						h->z2 = nd.b.z;
						
						(q[j]).obstacleList->insert(h);

						delete h;			

					}
				}
			
			}// end of num_entries's for 
			delete rtn;
		}	
		
		
		//get next entry from the heap of the top query point--------------------------

		if(qp_priority_queue.empty()==true)
		{
			end=1;
			break;
		}

		QueryPoint3D current_best_point;// = qp_priority_queue.top();
		
		HeapEntry *he = new HeapEntry();
		bool again = true;
		while (again)
		{			
			again = false;
			current_best_point = qp_priority_queue.top();
			qp_priority_queue.pop();
			bool isempty = !current_best_point.obstacleList->remove(he);
			Box2 obj;
			if (isempty)  //heap is empty, current_best_point is the answer
			{
				k_ans.push(current_best_point);
				//k--;
				_n--;
				
				//if(k==0) 
				if(_n==0) 
				{
						end=1; //end;
						break;
				}
				else
				{
					//qp_priority_queue theke best point already popped, so go for the next best point
					again=true;
					//break;
				}


			}
			//else if, check end condition, if reached the target
			////////////////////////////////////////////
			else if((obj=HeapEntry_to_box(he)).equals_to(T))
			{
				k_ans.push(current_best_point);
				//k--;
				_n--;
				
				//if(k==0)
				if(_n==0)
				{
					end = 1; //end;
					break;
				}
				else
				{
					//qp_priority_queue theke best point already popped, so go for the next best point
					again=true;
					//break;
				}
			}
			else
			{	
				
				obj= HeapEntry_to_box(he);
				
				if (he->level == 0) //p is an object ; code for data object
				{
					//check if obj is already been considered, if yes, don't consider it again;
					map<Box2,bool,CompareBox>::iterator mapItr;
					mapItr=obstacle_checked.find(obj);
				/*	if(mapItr!=obstacle_checked.end())
					{
						if(obstacle_checked.find(obj)->second==true)
						{
							again=true;
							qp_priority_queue.push(current_best_point);
							continue;
						}
						
					}
					else */
						obstacle_considered++;
					//calculate visibility for current_best_point

					

					current_best_point.total_visibility = update_naiveVisibility(current_best_point,obj,T); 
					
					//check for all query points, if obj is in visibility region of q[p], 
					//pop q[p],calculate visibility for q[p] and then insert into the priority queue
					
					//change visibility region of q[p]

					int size1=qp_priority_queue.size();
					priority_queue<QueryPoint3D, vector<QueryPoint3D>, CompareVisibility3D> qp_temp; 
					for(int x=0; x<size1; x++)
					{
						QueryPoint3D qp_check = qp_priority_queue.top();
						qp_priority_queue.pop();
						if(qp_check.IsInsideVisibleRegion(T,obj) == true)
						{
							qp_check.total_visibility = update_naiveVisibility(qp_check,obj,T);
						}
						qp_temp.push(qp_check);
					}
					qp_priority_queue = qp_temp; 
					
					//mark obj has been checked
					obstacle_checked[obj]=true;

					//get next data  from heap
					again =true;	
					qp_priority_queue.push(current_best_point);
				}
				else //not leaf node
				{
					qp_priority_queue.push(current_best_point);
					son=he->son1;
				}
			}
		}
		
		delete he;
	}
	delete heap;
	/*for(int l=0;l<k_original;l++)
	{
		k_answers[l]=k_ans.top();
		k_ans.pop();
	}*/
}
































//////////////////////// VCM for Dynamic Target /////////////



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

#include "gpc.h"


using namespace std;


#define Coord double


#define INF 1e6
#define EPS 1e-10
// +0.0000000001
#define S(x)	((x)*(x))
#define Z(x)	(fabs(x) < EPS)

struct Vector2{
	Coord x,y;


	Vector2(Coord x=0,Coord y=0){
		this->x=x;
		this->y=y;
	}

    void scan()
    {
        cin>>x>>y;
    }
    void print()
    {
        cout<<x<<" "<<y<<endl;
    }
	bool isZero(){
		return Z(x) && Z(y);
	}
	Coord mag(){
		return sqrt(mag2());
	}
	Coord mag2(){
		return S(x)+S(y);
	}
	void normalize(){ 
		Coord m = mag();
		x/=m;
		y/=m;
	}
	Vector2 unit(){	
		Vector2 u(x,y);
		u.normalize();
		return u;
	}
	bool equla( Vector2 &v){ return Z( x-v.x ) && Z( y-v.y ); }
};

Coord dotp(Vector2 a, Vector2 b){
	return a.x*b.x + a.y*b.y ;
}
Coord crossp(Vector2 a, Vector2 b){
	return a.x*b.y - a.y*b.x;
}

Vector2 operator+(Vector2 a,Vector2 b){
	return Vector2(a.x+b.x, a.y+b.y);	}

Vector2 operator-(Vector2 a){
	return Vector2(-a.x, -a.y);	}

Vector2 operator-(Vector2 a,Vector2 b){
	return Vector2(a.x-b.x, a.y-b.y);	}

Coord operator*(Vector2 a,Vector2 b){
	return crossp(a,b);	}

Vector2 operator*(Vector2 a, Coord b){
	return Vector2(a.x*b, a.y*b);	}

Vector2 operator*(Coord b, Vector2 a){
	return Vector2(a.x*b, a.y*b);	}

Vector2 operator/(Vector2 a, Coord b){
	return Vector2(a.x/b, a.y/b);	}


Coord angle(Vector2 a, Vector2 b){
	//if(fabs(a.mag2())<EPS||fabs(b.mag2())<EPS)return 0;
	Coord v= dotp(a,b) / (a.mag()*b.mag()) ;
	v=min(v,(Coord)1);
	v=max(v,(Coord)-1);
	return acos(v);
}


int Turn(Vector2 &V0,Vector2 &V1,Vector2 &V2)
{
    Coord v=(V1-V0)*(V2-V0);
    if(fabs(v)<EPS)return 0;
    if(v>0.0)return 1;
    return -1;
}


struct myRectangle
{
    Vector2 corner[4];

    void scan()
    {
        corner[0].scan();
        corner[2].scan();
		build();
        
    }
    void print()
    {
        int i;
        REP(i,4)corner[i].print();
    }
	void build()
	{
		corner[1]=Vector2( corner[2].x,corner[0].y );
        corner[3]=Vector2( corner[0].x,corner[2].y );
	}
};

class LineSegment
{
public:
    Vector2 v0;
    Vector2 v1;

	LineSegment(){}
    LineSegment(const Vector2& begin, const Vector2& end)
        : v0(begin), v1(end) {}

    enum IntersectResult { PARALLEL, COINCIDENT, NOT_INTERESECTING, INTERESECTING };

    IntersectResult Intersect(const LineSegment& L, Vector2& I,bool f=0)
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



bool  isVisible(myRectangle &ob1, myRectangle &ob2,int i1,int j1)
{
	int i,j;
	LineSegment L1,L0=LineSegment( ob1.corner[i1],ob2.corner[j1] );
	Vector2 I;
	REP( i,4 )
	{
		L1=LineSegment( ob1.corner[i],ob1.corner[(i+1)%4] );
		if(L0.Intersect( L1,I )==3 )
		{
			if( I.equla(ob1.corner[i1]) )continue;
			return 0;
		}
	}
	

	REP( j,4 )
	{
		L1=LineSegment( ob2.corner[j],ob2.corner[(j+1)%4] );
		if(L0.Intersect( L1,I )==3 )
		{
			if( I.equla(ob2.corner[j1]) )continue;
			return 0;
		}
	}


	return 1;

}

vector< Vector2 >P;

bool cmpCCW(Vector2 V1,Vector2 V2)
{
	int t=Turn(P[0],V1,V2);
	if(!t)
	{
		return (V1-P[0]).mag2()<(V2-P[0]).mag2();
	}
	return t>0;
}



struct Polygon
{
	vector< Vector2 >P;
	
	int size(){ return P.size(); }
	void push( Vector2 v ){ P.push_back(v); return; }

	void sortCCW()
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


	void print()
	{
		int i;

		REP( i,SZ( P ) )
		{
			P[i].print();
		}

	}


	void fprint( FILE *fp )
	{
		fprintf( fp,"1\n" );
		fprintf(fp,"%d\n",SZ( P ));
		
		int i;

		REP(i,SZ( P ))
		{
			fprintf(fp,"%lf %lf\n",P[ i ].x,P[i].y);
		}

	}


	void clear(){CL(P);}


	bool onPolySegment(Vector2 &P0)
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

	bool inPoly( Vector2 &p )
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
				Vector2 I;
				if(L1.Intersect(L2,I)==3)++cn;
			}
		}

		return (cn&1);
	}

	bool PointInPoly( Vector2 p )
	{
		if( onPolySegment(p) )return true;
		if( inPoly(p) )return true;
		return false;
	}

	bool polyInter( Polygon &poly)
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

};


void getShadowPolygon( myRectangle &tar, myRectangle &ob,Polygon &poly )
{
	int i,j;
	vi v;

	REP(j,4)
	{
		REP(i,4)
		{
			if( isVisible( tar,ob,i,j ) )
			{
				v.pb(j);
				break;
			}
		}

	}


	assert(SZ(v)>=2);



	
	REP(i,SZ(v))poly.P.push_back( ob.corner[ v[i] ] );


	int i1,j1,i2,j2;
	int k;
	bool f=0;

	REP( i,4 )
	{
		REP(j,4)
		{
			REP( k,4 )
			{
				if( Turn( tar.corner[i],ob.corner[j],tar.corner[k] )>0 )break;
			}
			if( k!=4 )continue;

			REP( k,4 )
			{
				if( Turn( tar.corner[i],ob.corner[j],ob.corner[k] )>0 )break;
			}
			if( k!=4 )continue;


			i1=i;
			j1=j;

			f=1;

			break;

		}
		if(f)break;
	}
	

	f=0;

	REP( i,4 )
	{
		REP(j,4)
		{
			REP( k,4 )
			{
				if( Turn( tar.corner[i],ob.corner[j],tar.corner[k] )<0 )break;
			}
			if( k!=4 )continue;

			REP( k,4 )
			{
				if( Turn( tar.corner[i],ob.corner[j],ob.corner[k] )<0 )break;
			}
			if( k!=4 )continue;


			i2=i;
			j2=j;

		}
		if(f)break;
	}
	


	LineSegment L0=LineSegment(  tar.corner[i1],ob.corner[j1]  );
	LineSegment L1=LineSegment(  tar.corner[i2],ob.corner[j2]  );

	
	Vector2 I;

	if( L0.Intersect(  L1,I,1 )==3 )
	{
		poly.P.push_back( I );
	}
	else
	{
		
		I=ob.corner[j1] + ( ob.corner[j1] - tar.corner[i1]  )*INF;
		poly.P.push_back( I );

		I=ob.corner[j2] + ( ob.corner[j2] - tar.corner[i2]  )*INF;
		poly.P.push_back( I );
	}




	poly.sortCCW();
	reverse( poly.P.begin() , poly.P.end()  );


}




Polygon update( Polygon &p ,myRectangle &tar )
{

    int i,j,k,i1,i2,f,j1,j2;
    f=0;

    for( i=0;i<4;i++ )
    {
        for( j=0;j<p.P.size();j++ )
        {
            for( k=0;k<p.P.size();k++ )
            {
                if( Turn( tar.corner[i],p.P[j],p.P[k] )>0 )break;
            }
            if( k!=p.P.size() )continue;
            for( k=0;k<4;k++ )
            {
                if( Turn( tar.corner[i],p.P[j],tar.corner[k] )>0 )break;
            }

            if( k!=4 )continue;
            i1=j;
			j1=i;
            f=1;
            break;
        }if(f)break;
    }

    f=0;

    for( i=0;i<4;i++ )
    {
        for( j=0;j<p.P.size();j++ )
        {
            for( k=0;k<p.P.size();k++ )
            {
                if( Turn( tar.corner[i],p.P[j],p.P[k] )<0 )break;
            }
            if( k!=p.P.size() )continue;
            for( k=0;k<4;k++ )
            {
                if( Turn( tar.corner[i],p.P[j],tar.corner[k] )<0 )break;
            }
            if( k!=4 )continue;
            i2=j;
			j2=i;
            break;
        }if( f )break;
    }




    Polygon ret;

    for( j=i2;j!=i1;j=( j+1 )%p.P.size() )
    {

        ret.P.push_back( p.P[ j ] );

    }
	
	ret.P.push_back( p.P[ j ] );


	

	LineSegment L0=LineSegment(  tar.corner[j1],p.P[i1]  );
	LineSegment L1=LineSegment(  tar.corner[j2],p.P[i2]  );


	Vector2 I;

	if( L0.Intersect(  L1,I,1 )==3 )
	{
		ret.P.push_back( I );
	}
	else
	{

		I=p.P[i1] + ( p.P[i1] - tar.corner[j1]  )*INF;
		ret.P.push_back( I );

		I=p.P[i2] + ( p.P[i2] - tar.corner[j2]  )*INF;
		ret.P.push_back( I );
	}


	

    return ret;

}

void gpc_print( gpc_polygon *p)
{
	printf("no of contours  %d\n",p->num_contours);

	for (int i = 0; i < p->num_contours; i++)
	{
		printf("no of vertices %d\n",p->contour->num_vertices);

		for (int j = 0; j < p->contour->num_vertices; j++)
		{
			printf("%.2lf %.2lf\n",p->contour->vertex[j]);
		}
		printf("\n");
	}

}



void update_g( gpc_polygon &global,myRectangle &tar )
{
	
	//gpc_print( &global );

	int i,j;	
	for( i=0;i<global.num_contours;i++ )
	{
		Polygon p;

		
		for( j=0;j<global.contour[i].num_vertices;j++ )
		{
			p.P.push_back( Vector2(global.contour[i].vertex[j].x , global.contour[i].vertex[j].y) );
		}


		//p.print();

		p=update( p,tar );

		free(global.contour[i].vertex);
		
		global.contour[i].num_vertices=SZ( p.P );
		global.contour[i].vertex=( gpc_vertex * )malloc( global.contour[i].num_vertices * sizeof( gpc_vertex ) );
		


		for( j=0;j<p.P.size();j++ )
		{
			global.contour[ i ].vertex[ j ].x=p.P[j].x;
			global.contour[ i ].vertex[ j ].y=p.P[j].y;
		}

	}



}

double point2seg( Vector2 &P, LineSegment &S)
{
    Vector2 v = S.v1 - S.v0;
    Vector2 w = P - S.v0;

    double c1 = dotp(w,v);
    if ( c1 <= 0 )
        return (P-S.v0).mag2();

    double c2 = dotp(v,v);
    if ( c2 <= c1 )
        return (P-S.v1).mag2();

    double b = c1 / c2;
    Vector2 Pb = S.v0 + b * v;
    return (P-Pb).mag2();
}




double calcdist(myRectangle &ou,myRectangle &tar )
{
	double ret=INF,d;
	ret*=ret;
	LineSegment L;

	for (int i = 0; i < 4; i++)
	{
		for (int j = 0; j < 4; j++)
		{
			L.v0=tar.corner[j];
			L.v1=tar.corner[(j+1)%4];
			d = point2seg( ou.corner[i],L );


			ret=MIN(ret,d);
		


			L.v0=ou.corner[i];
			L.v1=ou.corner[(i+1)%4];
			d = point2seg( tar.corner[j],L );


			ret=MIN(ret,d);
		


		}
	}


	return sqrt(ret);
}

map< int,double > dstFromTar;

class comparatorForPq
{
	public:
		bool operator()( int &a,int &b )
		{
			return dstFromTar[a]>dstFromTar[b];
		}
};

void  RTree:: rtreeVcmAlgo1( char *path )
{
	
	myRectangle tar;


	// setting target

	//tar.corner[0] = Vector2( 0,0 );
	//tar.corner[2] = Vector2( 4.5,.5 );

	char tarfile[100];
	strcpy( tarfile,path );
	strcat( tarfile,"target.txt" );

	FILE *fp=fopen("precalc.txt","w");

	freopen( tarfile,"r",stdin );

	tar.corner[0].scan();
	tar.corner[2].scan();

	tar.build();

	

	CL(dstFromTar);

	vector< myRectangle > Ov;

	


	gpc_polygon global_gp,temp;
	gpc_vertex gv;
	gpc_vertex_list gvl;



	
	vector< myRectangle > vob;
	
	


	priority_queue< int,vi,comparatorForPq >Q;

	
	Q.push( root );
	int v;

	int tot=0,i,j,k;



	global_gp.num_contours=0;

	while( !Q.empty() )
	{
		v=Q.top();
		Q.pop();



		RTNode *rtn = new RTNode(this, v);
		
		tot++;

		for(  i=0;i<rtn->num_entries;i++ )
		{
			Entry u = rtn->entries[i];
			
			myRectangle ou;

			ou.corner[0]=Vector2(   min(u.bounces[0],u.bounces[1]),min(u.bounces[2],u.bounces[3]) );
			ou.corner[2]=Vector2(   max(u.bounces[0],u.bounces[1]),max(u.bounces[2],u.bounces[3]) );
			
			ou.build();

			if( tar.corner[0].y > ou.corner[0].y )continue;

			Polygon pu,pv;
			gpc_polygon gpu;

			getShadowPolygon( tar,ou,pu );

			

			gpu.num_contours=1;
			gpu.hole=0;
			gpu.contour= ( gpc_vertex_list *) malloc( gpu.num_contours*sizeof( gpc_vertex_list ) ) ;


			gpu.contour[0].num_vertices = pu.size();
			gpu.contour[0].vertex=( gpc_vertex *)malloc( gpu.contour[0].num_vertices*sizeof( gpc_vertex ) );


			for ( j = 0; j < pu.size() ; j++)
			{
				gpu.contour[0].vertex[j].x=pu.P[j].x;
				gpu.contour[0].vertex[j].y=pu.P[j].y;
			}


			




			
			
			
		//	cout<<u.bounces[0]<<" "<<u.bounces[1]<<" "<<u.bounces[2]<<" "<<u.bounces[3]<<endl;
		//	pu.print();
			

			//checking is shadow of current MBR is within another shadow polygon


			

			for( j=0;j<global_gp.num_contours;j++ )
			{
				CL(pv);
				for (int k = 0; k < global_gp.contour[j].num_vertices; k++)
				{
					pv.P.push_back( Vector2( global_gp.contour[j].vertex[k].x , global_gp.contour[j].vertex[k].y  ) );
				}
				for( k=0;k<pu.size();k++ )
				{
					if( !pv.PointInPoly( pu.P[ k ] ) )break;
				}
				if( k==pu.size() )break;
			}


			if( j!=global_gp.num_contours )
			{
				//cout<<"discarding..."<<endl;
				continue;
			}




			double dst = calcdist( ou,tar );

			dstFromTar[ u.son ]=dst;


			// if not a root MBR continue
			if( rtn->level )
			{
				Q.push( u.son );
				continue;
			}
			else
			{
				
	//			ou.corner[0].print();
//				ou.corner[2].print();
				//cout<<endl;
				
				fprintf(fp,"%.10lf %.10lf %.10lf %.10lf\n", ou.corner[0].x, ou.corner[0].y,ou.corner[2].x,ou.corner[2].y);

			}
			

			gpc_polygon_clip(GPC_UNION,&gpu,&global_gp,&temp);
			swap( temp,global_gp );
			update_g( global_gp,tar );

		}
		
		
			
	}
	
	fclose(fp);

	//gpc_print( &global_gp );
	return;

}
