#include "OneMachineDP.h"

OneMachineDPProblem::OneMachineDPProblem(const char* filename, double time, int iter, Mode mod, tbMode tb, bool revChk, bool heuChk,
	char* jsonFile, char* solPathFile, char* critPathFile, char* infoPathFile)
{
	mModel = OneMachDPData(filename);
	mOptions.timeLimit = time;
	mOptions.iterationLimit = iter;
	mOptions.mod = mod;
	mOptions.tb = tb;
	mOptions.revChk = revChk;
	mOptions.heuChk = heuChk;
	if (jsonFile != nullptr)
		mJson = fopen(jsonFile, "a");
	if (solPathFile != nullptr)
		mSolPath = fopen(solPathFile, "w");
	if (critPathFile != nullptr)
		mCritPath = fopen(critPathFile, "w");
	if (infoPathFile != nullptr)
		mInfo = fopen(infoPathFile, "w");
}

void OneMachineDPProblem::solve()
{
	mModel.initialize(&mOptions);
	mModel.setOutJson(mJson);
	mModel.setOutSolPath(mSolPath);
	mModel.setOutCritPath(mCritPath);
	mModel.setOutInfo(mInfo);
	int solution, flag;

	solution = mModel.solve();
	mModel.updatePercentage();
	flag = mModel.mTerminateMode;

	switch (flag) {
	case 0:
		if (mModel.chkFinalSolution()) {
			printf("The solution is correct.\n");
		} else {
			printf("The solution is not correct.\n");
			throw ERROR << "Solution does not match.";
		}
		break;
	case 1:
		printf("Terminate. Iteration limit %d reached.\n", mModel.mIterLim);
		printf("Current global lower bound is %d.\n", mModel.globLB);
		break;
	case 2:
		printf("Terminate. Time limit %f reached.\n", mModel.mTimeLim);
		printf("Current global lower bound is %d.\n", mModel.globLB);
		break;
	default:
		throw ERROR << "Termination Mode Error.";
	}
	
	printf("Time is: %d.\n", mModel.mElapsTime);
	printf("Initial global lower bound is: %d.\n", mModel.initLB);
	printf("The solution is: %d.\n", solution);
	printf("The total iteration is: %d.\n", mModel.numIter);
}

void OneMachineDPProblem::printBranching() 
{
	printf("The number of feasible solution found is: %d.\n", mModel.mBranching->mNumDiscarded);
	printf("The number of strong branching is: %d.\n", mModel.mBranching->mNumStrongBch);
	printf("The number of weak branching case 1 is: %d.\n", mModel.mBranching->mNumWeakBch1);
	printf("The number of weak branching case 2 is: %d.\n", mModel.mBranching->mNumWeakBch2);
	printf("The number of reverse solution used is: %d.\n", mModel.mBranching->mNumRevBrch);
	printf("Updates: Head, %d; Tail, %d.\n", mModel.mPost->mNumHeadUpdts, mModel.mPost->mNumTailUpdts);
}

void OneMachineDPProblem::printSolToJson()
{
	if (mJson == nullptr) return;
	fprintf(mJson, "{");

	fprintf(mJson, "\"name\": \"%s\", \"TerCond\": %d, \"iter\": %d, \"time\": %d, \"obj_value\": %d, \"obj_lb\": %d, \"total_nodes\": %d", mModel.mOneMachineName.c_str(), mModel.mTerminateMode,
		mModel.numIter, mModel.mElapsTime, mModel.globUB, mModel.globLB, mModel.curID);
	
	fprintf(mJson, ", \"strong\": %d, \"weak_1\": %d, \"weak_2\": %d", mModel.mBranching->mNumStrongBch, mModel.mBranching->mNumWeakBch1, mModel.mBranching->mNumWeakBch2);
	
	fprintf(mJson, ", \"updtHead\": %d, \"updtTail\": %d", mModel.mPost->mNumHeadUpdts, mModel.mPost->mNumTailUpdts);

	fprintf(mJson, ", \"init_lb\": %d", mModel.initLB);

	fprintf(mJson, ", \"bstFoundAtIter\": %d, \"bstFoundCritSize\": %d", mModel.bstFoundAtIter, mModel.bstFoundCritSize);

	fprintf(mJson, ", \"numNodesInitLB\": %d", mModel.mBranching->mNumNodesInitLB);

	fprintf(mJson, ", \"numLLTH\": %d", mModel.mNumLLTH);

	fprintf(mJson, ", \"numLNodes\": %d, \"numGNodes\": %d, \"numNodes\": %d", mModel.numLNodes, mModel.numGNodes, mModel.numNodes);

	fprintf(mJson, ", \"maxDepth\": %d", mModel.maxDepth);

	fprintf(mJson, "}\n");
}

void OneMachineDPProblem::cleanup() 
{
	mModel.cleanUp();
	if (mJson != nullptr)
		fclose(mJson);
	if (mSolPath != nullptr)
		fclose(mSolPath);
	if (mCritPath != nullptr)
		fclose(mCritPath);
	if (mInfo != nullptr)
		fclose(mInfo);
}

void OneMachineDPProblem::testRun() 
{
	mModel.initialize(&mOptions);
	mModel.setOutJson(mJson);
	mModel.setOutSolPath(mSolPath);
	mModel.setOutCritPath(mCritPath);
	mModel.setOutInfo(mInfo);
	int numSol;
	numSol = mModel.testHeuristics();
	for (int i = 0; i < numSol; i++) {
		fprintf(mJson, "%d ", mModel.testHeuRes[i]);
	}
	fprintf(mJson, "\n");
}

void OneMachDPUtil::swapCritPathes(OneMachDPCritPath* first, OneMachDPCritPath* second)
{
	OneMachDPCritPath* temp;
	temp = first;
	first = second;
	second = temp;
}

void OneMachDPUtil::copyNodeAll(OneMachDPNode* from, OneMachDPNode* to)
{
	to->mOneMachDPData = from->mOneMachDPData;
	to->mUpdatedHead = from->mUpdatedHead;
	to->mUpdatedTail = from->mUpdatedTail;
	to->mJobScheduled = from->mJobScheduled;
	to->mLongestToCur = from->mLongestToCur;
	to->mCritPath = from->mCritPath;
	to->mSolPath = from->mSolPath;
	to->mLBSolPath = from->mLBSolPath;
	to->mCritStep = from->mCritStep;
	to->mSpecialStep = from->mSpecialStep;
	to->mLastPrecStep = from->mLastPrecStep;
	to->allFixes = from->allFixes;
	to->mAllPreds = from->mAllPreds;
	to->mAllSuccs = from->mAllSuccs;
	to->mLBound = from->mLBound; to->mRexSol = from->mRexSol, to->mFeaSol = from->mFeaSol, to->mParentSol = from->mParentSol;
	to->mSumProc = from->mSumProc, to->mMinHead = from->mMinHead, to->mMinTail = from->mMinTail;
	to->mNodeID = from->mNodeID, to->mParentID = from->mParentID;
	to->mContour = from->mContour, to->mDepth = from->mDepth;
}

void OneMachDPUtil::copyNodeBasic(OneMachDPNode* from, OneMachDPNode* to) 
{
	to->mOneMachDPData = from->mOneMachDPData;
	to->mUpdatedHead = from->mUpdatedHead;
	to->mUpdatedTail = from->mUpdatedTail;
	to->allFixes = from->allFixes;
	to->mAllPreds = from->mAllPreds;
	to->mAllSuccs = from->mAllSuccs;
	to->mLBound = from->mLBound;
	to->mParentSol = from->mParentSol;
	to->mNodeID = from->mNodeID;
	to->mParentID = from->mParentID;
	to->mContour = from->mContour;
	to->mDepth = from->mDepth;
}

void OneMachDPUtil::copyNodeSolInfo(OneMachDPNode* from, OneMachDPNode* to)
{
	to->mSolPath = from->mSolPath;
	to->mCritPath = from->mCritPath;
	to->mCritStep = from->mCritStep;
	to->mFeaSol = from->mFeaSol;
	to->mSumProc = from->mSumProc;
	to->mMinHead = from->mMinHead;
	to->mMinTail = from->mMinTail;
	to->mJobScheduled = from->mJobScheduled;
}