class Pair{

public:
	int blockNo;
	int entryNo;

	Pair(){}

	Pair(int blockNo, int entryNo){
		this->blockNo = blockNo;
		this->entryNo = entryNo;
	}

	bool operator<(const Pair& b) const {
		return blockNo < b.blockNo || (blockNo == b.blockNo && entryNo < b.entryNo);
	}
};