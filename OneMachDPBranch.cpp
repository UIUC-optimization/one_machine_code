#include "OneMachineDP.h"

void OneMachDPBranch::initialize()
{
	mTotalBchCount = mNumDiscarded = 0;
	mNumStrongBch = mNumWeakBch1 = mNumWeakBch2 = mNumFeaSolFound = 0;
	mNumRevBrch = 0;
	mFixMoreCount = 0;
	mNumNodes = mNumNodesInitLB = 1;
	if (mOneMachDPData != nullptr) {
		mJsonFile = mOneMachDPData->mJsonFile;
		mRevChkOn = mOneMachDPData->mRevChkOn;
	}
}

int OneMachDPBranch::main(OneMachDPNode* node, int BrchScn) 
{
	mTotalBchCount++;
	printf("Start branching process...\n");
	int revFlag;
	iterJobs critStep, iter, end;
	JobStep* specialStep; 
	JobStep* lastPrecStep;
	OneMachDPCritPath* revCritPathes;
	OneMachDPCritPath* tempStore;
	if (BrchScn == 0) {
		// flag: 0 --- Solution is optimal for the node, just return the flag
		mNumFeaSolFound++;
	} else {
		// flag: 1,2,3,4 --- Solution is not optimal so compute LB
		int newLB = mOneMachDPData->mComputeBounds->getLBFromSol(node);
		if (newLB > node->mRexSol)
			node->mRexSol = newLB;
		if (node->mRexSol >= mOneMachDPData->globUB) {
			// LB is greater than global UB, then no need to branch
			mNumDiscarded++;
			return 4;
		}

		// Initialization of child nodes need to be done for each branching scenario to suit special needs
		OneMachDPNode* newOneMachNodeLeft;
		OneMachDPNode* newOneMachNodeRight;
		critStep = node->getStepInCritPath(node->mCritStep->jobIndex);
		if (BrchScn == 1) {
			// flag: 1 --- Strong branching
			mNumStrongBch++;
			mFixMoreCount += node->fixMore(mOneMachDPData->globUB);

			newOneMachNodeLeft = new OneMachDPNode(node);
			newOneMachNodeRight = new OneMachDPNode(node);

			iter = critStep;
			end = node->mCritPath.end();
			iter++;
			while (iter != end) {
				newOneMachNodeLeft->addFixBasic(*critStep, *iter, 0);
				newOneMachNodeRight->addFixBasic(*iter, *critStep, 0);
				iter++;
			}
		} else if (BrchScn == 2) {
			// flag: 2 --- Weak branching case 1
			mNumWeakBch1++;
			newOneMachNodeLeft = new OneMachDPNode(node);
			newOneMachNodeRight = new OneMachDPNode(node);

			specialStep = node->mSpecialStep;
			if (specialStep == nullptr)
				throw ERROR << "Error in branching job steps.";
			newOneMachNodeLeft->addFixBasic(*critStep, specialStep, 0);
			newOneMachNodeRight->addFixBasic(specialStep, *critStep, 0);
		} else {
			// flag: 3 --- Weak branching case 2
			mNumWeakBch2++;
			newOneMachNodeLeft = new OneMachDPNode(node);
			newOneMachNodeRight = new OneMachDPNode(node);

			specialStep = node->mSpecialStep;
			lastPrecStep = node->mLastPrecStep;
			if (specialStep == nullptr || lastPrecStep == nullptr)
				throw ERROR << "Error in branching job steps.";
			newOneMachNodeLeft->addFixBasic(lastPrecStep, specialStep, 0);
			newOneMachNodeRight->addFixBasic(specialStep, lastPrecStep, 0);
		}
		
		// Update node id
		newOneMachNodeLeft->mNodeID = mOneMachDPData->increaseNodeID();
		curLeft = newOneMachNodeLeft->mNodeID;
		newOneMachNodeRight->mNodeID = mOneMachDPData->increaseNodeID();
		curRight = newOneMachNodeRight->mNodeID;
		//printf("Node %d creates node %d and node %d.\n", node->mNodeID, newOneMachNodeLeft->mNodeID, newOneMachNodeRight->mNodeID);

		// Update left and right tree level count
		newOneMachNodeLeft->mLweight++;
		newOneMachNodeRight->mRweight++;
		mNumNodes += 2;
		//node->undoedge();
		// precise lower bound in branching step
		// Update heads and tails
		newOneMachNodeLeft->populateFixes();
		newOneMachNodeRight->populateFixes();
		newOneMachNodeLeft->updateEdge();
		newOneMachNodeRight->updateEdge();

		newOneMachNodeLeft->doedge();
		mOneMachDPData->mComputeBounds->getLBStd(newOneMachNodeLeft);
		newOneMachNodeLeft->mLBound = newOneMachNodeLeft->mRexSol;
		if (newOneMachNodeLeft->mLBound < node->mRexSol)
			newOneMachNodeLeft->mLBound = node->mRexSol;
		if (newOneMachNodeLeft->mLBound == mOneMachDPData->initLB)
			mNumNodesInitLB++;

		newOneMachNodeRight->doedge();
		mOneMachDPData->mComputeBounds->getLBStd(newOneMachNodeRight);
		newOneMachNodeRight->mLBound = newOneMachNodeRight->mRexSol;
		if (newOneMachNodeRight->mLBound < node->mRexSol)
			newOneMachNodeRight->mLBound = node->mRexSol;
		if (newOneMachNodeRight->mLBound == mOneMachDPData->initLB)
			mNumNodesInitLB++;

		// clean updated info to save space
		newOneMachNodeLeft->cleanConstrs();
		newOneMachNodeRight->cleanConstrs();

		// Store the nodes
		mOneMachDPData->addNode(newOneMachNodeLeft);
		mOneMachDPData->addNode(newOneMachNodeRight);
	}
	//node->undoedge();
	return BrchScn;
}