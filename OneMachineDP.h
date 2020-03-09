/************************************************************************************************************
 * One Machine Sequencing problem with Delayed Precedence Constraints:										*
 *																A Branch-and-bound algorithm implementation	*
 ************************************************************************************************************/

#ifndef ONEMACHDP_H
#define ONEMACHDP_H

#include "util.h"
#include<list>
#include<queue>
#include<map>
#include<stack>
#include<functional>
#include<chrono>
//#include<vld.h>

using namespace std;

#define STARTOFTIME -10000000
#define ENDOFTIME 10000000

struct fixedEdge;
struct JobStep;
class OneMachDPNode;
class OneMachDPBounds;
class OneMachDPPost;
class OneMachDPBranch;
class OneMachDPCritPath;
class OneMachDPUtil;
typedef multimap<int, OneMachDPNode*> NodesMap;
typedef map<int, NodesMap> ContourMap;
typedef map<int, int> LBMap;
typedef chrono::steady_clock myclock;
typedef list<fixedEdge*> edgePtrList;
typedef list<fixedEdge> edgeList;
typedef list<JobStep*>::iterator iterJobs;
typedef chrono::high_resolution_clock myclock;

/************************************************************************************************************
 * Stores information about jobsteps																		*
 ************************************************************************************************************/
typedef struct JobStep
{
public:
	JobStep() {}
	JobStep(int index, int time, int start, int end) : jobIndex(index), body(time), head(start), tail(end)
	{}
	int jobIndex;             // The jobstep index
	int head, body, tail;     // Time information: head, tail, and process time(body)
	int curHead;              // Updated head during longest tail heuristics
	int bestTime;
	int remTime;              // The remaining time needed to finish the step, needed for LB computation
	bool isTailUpdated;
	
} JobStep;

/************************************************************************************************************
 * Stores information about precedence contraints															*
 ************************************************************************************************************/
typedef struct fixedEdge
{
	JobStep* from;
	JobStep* to;
	int delay;
	fixedEdge(JobStep* pred, JobStep* succ, int time) : from(pred), to(succ), delay(time) {}
} fixedEdge;

/************************************************************************************************************
 * Stores information about critical path check																*
 ************************************************************************************************************/
typedef struct critPathCheck
{
	int type;
	int numPrecInCritPath;
	int stepsAftCritStep;
	JobStep* critJob;
	JobStep* lastPrecStep;
	critPathCheck() {}
	critPathCheck(int tp, int num, int stepsAftCrit, JobStep* step, JobStep* preStep) : 
		type(tp), numPrecInCritPath(num), stepsAftCritStep(stepsAftCrit), critJob(step), lastPrecStep(preStep) {}
} critPathCheck;

/************************************************************************************************************
 * Stores information about options to initialize solver													*
 ************************************************************************************************************/
typedef struct options
{
	double timeLimit;
	int iterationLimit;
	Mode mod;
	tbMode tb;
	bool revChk;
	bool heuChk;
	options() {}
	options(double time, int iter, Mode m) : timeLimit(time), iterationLimit(iter), mod(m) { revChk = false; }
	options(double time, int iter, Mode m, bool r) : timeLimit(time), iterationLimit(iter), mod(m), revChk(r) {}
} options;

/************************************************************************************************************
 * Comparison functions																						*
 ************************************************************************************************************/
class stepComp
{
public:
	stepComp(int ht) { headTail = ht; }
	bool operator() (const JobStep* a, const JobStep* b) const
	{
		if (headTail == 1)
			// If headTail is 1, then compare head
			return a->head > b->head;
		else if (headTail == 2)
			// If headTail is 2, then compare tail
			return a->tail < b->tail;
		else if (headTail == 3)
			// If headTail is 3, then compare curHead
			return a->curHead > b->curHead;
		else
			throw ERROR << "Not valid comparison";
	}
private:
	int headTail;
};

typedef priority_queue<JobStep*, vector<JobStep*>, stepComp> minHeadQueue;
typedef priority_queue<JobStep*, vector<JobStep*>, stepComp> maxTailQueue;

class OneMachDPData
{
public:
	OneMachDPData() {}
	OneMachDPData(const char* filename);
	void initialize(options* opt);
	int solve();
	int solveNode(OneMachDPNode* node);
	int solveRevNode(OneMachDPNode* node);
	int chkDelayJobCritPathes(OneMachDPNode* node);
	int LLTHs(OneMachDPNode* node);

	void addNode(OneMachDPNode* node);
	void delNode(OneMachDPNode* node);
	void dumpAllNodes();
	void resetTailUpdateChk();
	void printJobsteps();
	void printInfo(OneMachDPNode* node);
	void cleanUp();
	OneMachDPNode* getNextNode();
	OneMachDPNode* getNextNode(tbMode tbRule);
	JobStep* getJobStep(int jobIndex);

	int calContour(OneMachDPNode* node);
	int calSolution(list<JobStep*> &solPath);
	int calSolution(OneMachDPNode* node);
	bool chkFinalSolution();
	bool haveDPC(int fromIndex, int toIndex);
	int getCurLB();

	int increaseNodeID() { return ++curID; }
	int getDelay(int from, int to) { return mInitFixDPDelay[from][to]; }
	void setOutJson(FILE* jsonFile) { mJsonFile = jsonFile; }
	void setOutSolPath(FILE* pathFile) { mPathFile = pathFile; }
	void setOutInfo(FILE* infoFile) { mInfoFile = infoFile; }
	void setOutCritPath(FILE* critPathFile) { mCritPathFile = critPathFile; }
	void setIterator() { mCurContour = mContours.begin(); }
	void revToPrevContour() { mCurContour = mPreContour; }
	int testHeuristics();

	void updatePercentage();

	int globLB, globUB;							// globLB is currently unused. TODO: Find a way to update globLB
	int initLB;
	int curID;
	int numToExplore, numIter, bstFoundAtIter, bstFoundCritSize;
	int leftConst, rightConst;
	int numJobs;                                // Number of job steps
	int numInitFix;								// Number of precedence constraint
	int numNodes, numLNodes, numGNodes;
	int maxDepth;
	
	list<JobStep> mJobsData;
	list<fixedEdge> mInitFix;                   // List of starting precedence arcs
	vector<JobStep*> mJobsByIndex;				// vector of job ptr for easy access
	vector<edgePtrList> mInitPreds;
	vector<edgePtrList> mInitSuccs;
	vector<int> mInitHead, mInitTail;			// array store the initial head and tail
	vector<int> testHeuRes;
	vector<vector<int>> mInitFixDPDelay;		// 2 dimensional vector to store delay for initial precedence arcs
	list<JobStep*> mBstSolPath;                 // Current best solution
	OneMachDPNode* mBstSolNode;
	OneMachDPBounds* mComputeBounds;            // Solver for the LB and UB
	OneMachDPBranch* mBranching;				// Branching module
	OneMachDPPost* mPost;
	OneMachDPUtil* mUtil;
	OneMachDPCritPath* mCritPathes;
	OneMachDPCritPath* mRevCritPathes;
	LBMap mLowerBd;
	LBMap mRexSolCount;
	ContourMap mContours;                       // Use map to store contours
	ContourMap::iterator mCurContour;           // Iterator to enable cycling through contours
	ContourMap::iterator mPreContour;
	stack<OneMachDPNode*> mNodesStack;          // Stack for DFS
	string mOneMachineName;                     // name of the problem

	double mTimeLim;
	int mIterLim, mTerminateMode;
	int mNumLLTH;
	long mElapsTime;
	bool mRevChkOn, mCombineOn, mCombineOnRev;
	Mode mMode;                                 // Control the contour mode
	tbMode mTbMode;
	int mMesrBest;								// Determine the measure of best criteria
	FILE* mJsonFile;
	FILE* mPathFile;
	FILE* mInfoFile;
	FILE* mCritPathFile;
};

class OneMachineDPProblem
{
public:
	OneMachineDPProblem(const char* filename, double time, int iter, Mode mod, tbMode tb, bool revChk, bool heuChk,
		char* jsonFile, char* solPathFile, char* critPathFile, char* infoPathFile);
	void solve();
	void printSolToJson();
	void printBranching();
	void cleanup();
	void testRun();

	FILE* mJson;
	FILE* mSolPath;
	FILE* mCritPath;
	FILE* mInfo;
	options mOptions;
	OneMachDPData mModel;
};

class OneMachDPNode
{
public:
	OneMachDPNode() {}
	OneMachDPNode(OneMachDPData* oneMach);
	OneMachDPNode(OneMachDPNode* pre);
	OneMachDPNode(OneMachDPNode* org, bool inMap);
	~OneMachDPNode();
	void doedge();
	void undoedge();
	void updateEdge();
	void populateFixes();
	void addFix(JobStep* from, JobStep* to, int delay);
	void addFixNoChk(JobStep* from, JobStep* to, int delay);
	void addFixBasic(JobStep* from, JobStep* to, int delay);
	void resetcurHead();
	void fillInPos();
	void updateHeadInSol();
	void cleanConstrs();
	void clearAll();

	bool haveDPC(int fromIndex, int toIndex);
	bool havePrecConstr(int fromIndex, int toIndex, int remGap);
	bool havePrecConstrBasic(int fromIndex, int toIndex);
	int numJobsBtwn(int fromIndex, int toIndex);
	
	int reschCritPathes();
	JobStep* findDelayedJob(list<JobStep*> &critPath);
	int branchingScenario();
	critPathCheck checkCritPath(list<JobStep*> &critPath);
	iterJobs getStepInSol(int jobIndex);
	iterJobs getStepInCritPath(int jobIndex);
	int updateHeadHelper(int jobIndex);
	int updateTailHelper(int jobIndex);
	int updateTailBySucc(int jobIndex);
	int fixMore(int curSol);
	OneMachDPNode* revNode();
	void printPath(FILE* inFile);
	void printCritPath(FILE* inFile);

	OneMachDPData* mOneMachDPData;
	vector<int> mUpdatedHead, mUpdatedTail;
	vector<int> mJobScheduled;							// The starting time of each scheduled job
	vector<int> mLongestToCur;							// In solution path, the longest path from previous to current job step
	vector<int> mIndInPathByPos;						// job index in solution path by position
	vector<int> mPosInPathByJob;						// position of jobs in solution path by job index
	list<JobStep*> mCritPath;
	list<JobStep*> mSolPath, mLBSolPath;
	JobStep* mCritStep;
	JobStep* mSpecialStep;
	JobStep* mLastPrecStep;
	edgeList allFixes;
	vector<edgePtrList> mAllPreds;
	vector<edgePtrList> mAllSuccs;
	int mLBound, mRexSol, mFeaSol, mParentSol;
	int mSumProc, mMinHead, mMinTail;					// LocalLB, minHead and minTail for JobStep set J (on critical path from critical jobstep to end)
	int mNodeID, mParentID;								// mNodeID to keep track of them, and mParent is parent ID
	int mContour, mDepth;								// Keep track of the node's position in the search tree
	int mLweight, mRweight;
	bool isInMap;
};

/*
class OneMachDPSolve
{
public:
	OneMachDPSolve() {}
	OneMachDPSolve(OneMachDPData* omdp) : mOneMachDPData(omdp) {}

	void doedge();
	void undoedge();
	void updateEdge();
	list<JobStep*> mCritPath;
	list<JobStep*> mSolPath;
	list<JobStep*>::iterator mCritStep;
	OneMachDPData* mOneMachDPData;
};
*/

class OneMachDPBounds
{
public:
	OneMachDPBounds() {}
	OneMachDPBounds(OneMachDPData* omdp) : mOneMachDPData(omdp) {}
	void initialize();
	int getLBStd(OneMachDPNode* node);
	int getLBStd(OneMachDPNode* node, list<JobStep*> &jobsToSchd);
	int getLBFromSol(OneMachDPNode* node);
	int getUB(OneMachDPNode* node);
	int getUBMod(OneMachDPNode* node);
	int getUBMod2(OneMachDPNode* node);
	int getUBMod3(OneMachDPNode* node);
	int mNumJobSteps;
	OneMachDPData* mOneMachDPData;
	FILE* mJsonFile;
};

class OneMachDPPost
{
public:
	OneMachDPPost() {}
	OneMachDPPost(OneMachDPData* omdp) : mOneMachDPData(omdp) {}
	void initialize();
	bool main(OneMachDPNode* node);
	bool chkPrecedPost(OneMachDPNode* node);
	bool chkSuccedPost(OneMachDPNode* node);
	int mNumHeadUpdts, mNumTailUpdts;
	int mCurJobErased;
	list<JobStep*>* mCurPath;
	OneMachDPData* mOneMachDPData;
	FILE* mJsonFile;
};

class OneMachDPBranch
{
public:
	OneMachDPBranch() {}
	OneMachDPBranch(OneMachDPData* omdp) : mOneMachDPData(omdp) {}
	int main(OneMachDPNode* node, int BrchScn);
	int increRevCount() { mNumRevBrch++; return mNumRevBrch; }
	void initialize();
	OneMachDPData* mOneMachDPData;

	int mNumStrongBch, mNumWeakBch1, mNumWeakBch2, mNumFeaSolFound, mNumRevBrch;
	int mTotalBchCount;
	int mFixMoreCount;
	int mNumDiscarded;													// Number of nodes discarded during branching because its LB > global UB
	int mNumNodes, mNumNodesInitLB;
	int curLeft, curRight;
	bool mRevChkOn;
	FILE* mJsonFile;
};

class OneMachDPCritPath
{
public:
	OneMachDPCritPath() {}
	OneMachDPCritPath(OneMachDPData* omdp) : mOneMachDPData(omdp) {}
	void main(OneMachDPNode* node);
	void initialize();
	void fillPosAndIndex();
	void findAllCritPath();
	void filterCritPath();
	void findValidCritPath();
	void clearPathes();
	int findAllLgestPathToJob(JobStep* curJob);

	int mCurFeaSol;
	OneMachDPData* mOneMachDPData;
	OneMachDPNode* mCurNode;
	list<list<JobStep*>> mAllCritPath;
	vector<list<list<JobStep*>>> mLgstPathesByJob;
	vector<JobStep*> mIndexedJob;
	vector<int> mMaxLgthToJob;
	vector<int> mPosInPathByJob;									// position in path of each jobsteps
	vector<int> mIndInPathByPos;									// Index of jobsteps in path
	vector<int> mCanBeInCritPath;
	FILE* mJsonFile;
};

class OneMachDPUtil
{
public:
	OneMachDPUtil() {}
	void swapCritPathes(OneMachDPCritPath* first, OneMachDPCritPath* second);
	void copyNodeBasic(OneMachDPNode* first, OneMachDPNode* second);
	void copyNodeSolInfo(OneMachDPNode* first, OneMachDPNode* second);
	void copyNodeAll(OneMachDPNode* first, OneMachDPNode* second);
};

#endif	// ONEMACHDP_H
