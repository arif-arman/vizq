/* rtree.h
   this file defines the class RTree*/

#ifndef __RTREE
#define __RTREE

//------------------------------------------------------------

#include "../func/gendef.h"
#include "../heap/heap.h"
#include <vector>
#include <list>


//#include "../vector3.h"

using namespace std;


//------------------------------------------------------------
class LinList;
class SortedLinList;
class Cache;
class RTNode;
class Entry;

//------------------------------------------------------------


//-------------------------------------------------------------
class RTree : public Cacheable
{
public:
//--===on disk===--
	int dimension;                       
	int num_of_data;	                 
    int num_of_dnodes;	                 
    int num_of_inodes;	                 
	int root;                            
	bool root_is_data;                   
//--===others===--
	RTNode *root_ptr;
    bool *re_level;  
    LinList *re_data_cands; 
	LinList *deletelist;

//--====added for k-nn==----
	Heap *tpheap;


//--===functions===--
	RTree(char *fname, int _b_length, Cache* c, int _dimension);
    RTree(char *fname, Cache* c);
    RTree(char *inpname, char *fname, int _blength, Cache* c, int _dimension);

    ~RTree();
	void del_root();
	bool delete_entry(Entry *d);
	bool FindLeaf(Entry *e);
    int get_num() { return num_of_data; }
	void insert(Entry *d);
	void load_root();  
	void rangeQuery(float *mbr, SortedLinList *res);
	void read_header(char *buffer);      
	void write_header(char *buffer);
	int update_rslt(Entry *_e, float _dist, Entry *_rslt, 
					 float *_key, int _k);



	
};

#endif // __RTREE
