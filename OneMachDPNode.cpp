#include "OneMachineDP.h"

OneMachDPNode::OneMachDPNode(OneMachDPData* oneMachDP) 
{
	isInMap = true;
	mOneMachDPData = oneMachDP;
	allFixes = oneMachDP->mInitFix;
	auto end = oneMachDP->mInitFix.end();
	mNodeID = mOneMachDPData->curID;
	mLBound = 0;
	mRexSol = 0;
	mFeaSol = MaxInt;
	mParentSol = MaxInt;
	mDepth = 0;
	mLweight = 0, mRweight = 0;
}

OneMachDPNode::OneMachDPNode(OneMachDPNode* parent) 
{
	isInMap = true;
	mOneMachDPData = parent->mOneMachDPData;
	allFixes = parent->allFixes;
	mParentID = parent->mNodeID;
	mLBound = parent->mRexSol;
	mRexSol = -1;
	mFeaSol = MaxInt;
	mParentSol = parent->mFeaSol;
	mDepth = parent->mDepth + 1;
	mLweight = parent->mLweight;
	mRweight = parent->mRweight;
}

OneMachDPNode::OneMachDPNode(OneMachDPNode* org, bool inMap) 
{
	// This node is not going to be inserted into map
	isInMap = inMap;
	// Tail as head and head as tail
	mOneMachDPData = org->mOneMachDPData;
	mUpdatedHead = org->mUpdatedTail;
	mUpdatedTail = org->mUpdatedHead;
	// Copy other info
	mParentID = org->mParentID;
	mLBound = org->mLBound;
	mRexSol = org->mRexSol;
	mParentSol = org->mParentSol;
	mDepth = org->mDepth;
	// Initialize precedence vectors
	mAllPreds.resize(org->mOneMachDPData->numJobs);
	mAllSuccs.resize(org->mOneMachDPData->numJobs);
	// Reverse each precedence constraints
	auto iter = org->allFixes.begin();
	auto end = org->allFixes.end();
	while (iter != end) {
		addFix((*iter).to, (*iter).from, (*iter).delay);
		iter++;
	}
}

void OneMachDPNode::populateFixes() 
{
	mAllPreds.clear();
	mAllSuccs.clear();
	mAllPreds.resize(mOneMachDPData->numJobs);
	mAllSuccs.resize(mOneMachDPData->numJobs);
	for (auto iter = allFixes.begin(); iter != allFixes.end(); iter++) {
		mAllPreds[(*iter).to->jobIndex].push_back(&*iter);
		mAllSuccs[(*iter).from->jobIndex].push_back(&*iter);
	}
}

/************************************************************************************************************
 * Change heads and tails of all jobsteps for current problem												*
 ************************************************************************************************************/
void OneMachDPNode::doedge() 
{
	auto end = mOneMachDPData->mJobsData.end();
	for (auto iter = mOneMachDPData->mJobsData.begin(); iter != end; iter++) {
		(*iter).head = mUpdatedHead[(*iter).jobIndex];
		(*iter).tail = mUpdatedTail[(*iter).jobIndex];
	}
}

/************************************************************************************************************
 * Recover heads and tails of all jobsteps																	*
 ************************************************************************************************************/
void OneMachDPNode::undoedge() 
{
	auto end = mOneMachDPData->mJobsData.end();
	for (auto iter = mOneMachDPData->mJobsData.begin(); iter != end; iter++) {
		(*iter).head = mOneMachDPData->mInitHead[(*iter).jobIndex];
		(*iter).tail = mOneMachDPData->mInitTail[(*iter).jobIndex];
	}
}

int OneMachDPNode::fixMore(int curBstSol) 
{
	iterJobs iter = mSolPath.begin();
	iterJobs end = mSolPath.end();
	iterJobs jEnd = mCritPath.end();
	iterJobs jIter = getStepInCritPath(mCritStep->jobIndex);
	iterJobs jIterNext = jIter;
	jIterNext++;													// The next node on critPath from crit step
	vector<int> inCritSection;										// Determine if the job step is in the section of critical path after critical step
	inCritSection.resize(mOneMachDPData->numJobs, 0);
	for (; jIter != jEnd; jIter++) {
		inCritSection[(*jIter)->jobIndex] = 1;
	}

	int gap = curBstSol - (mMinHead + mMinTail + mSumProc);
	int fixmoreCount = 0;
	while (iter != end) {
		// This test is not for job steps that is in set J and critical step c
		if (inCritSection[(*iter)->jobIndex] == 0) {
			if ((*iter)->body > gap) {
				if ((*iter)->head + (*iter)->body + mSumProc + mMinTail > curBstSol) {
					for (auto kIter = jIterNext; kIter != jEnd; kIter++) {
						addFix(*kIter, *iter, 0);
					}
					fixmoreCount++;
				} else if (mMinHead + mSumProc + (*iter)->body + (*iter)->tail > curBstSol) {
					for (auto kIter = jIterNext; kIter != jEnd; kIter++) {
						addFix(*iter, *kIter, 0);
					}
					fixmoreCount++;
				}
			}
		}
		iter++;
	}
	//printf("%d additional edges are fixed. %d is the gap.\n", fixmoreCount, gap);
	return fixmoreCount;
}

/************************************************************************************************************
 * Update heads and tails information of all jobsteps														*
 ************************************************************************************************************/
void OneMachDPNode::updateEdge() 
{
	int numJobs = mOneMachDPData->numJobs;
	mUpdatedHead.clear();
	mUpdatedHead.resize(numJobs, -1);
	mUpdatedTail.clear();
	mUpdatedTail.resize(numJobs, -1);
	auto iter = mOneMachDPData->mJobsData.begin();
	auto end = mOneMachDPData->mJobsData.end();
	while (iter != end) {
		JobStep* curIter = &*iter;
		int curInd = curIter->jobIndex;
		//printf("Current job index is %d.\n", curInd);
		updateHeadHelper(curInd);
		updateTailHelper(curInd);
		iter++;
	}
}

/************************************************************************************************************
 * Recursively update all head of jobsteps that have predesessors											*
 ************************************************************************************************************/
int OneMachDPNode::updateHeadHelper(int jobIndex) 
{
	if (mUpdatedHead[jobIndex] != -1) {
		// if not -1, then head already updated, nothing needs to be done
	} else if (mAllPreds.empty() || mAllPreds[jobIndex].empty()) {
		mUpdatedHead[jobIndex] = mOneMachDPData->getJobStep(jobIndex)->head;
	} else {
		multimap<int, int> smallHead;
		int sumTime = 0, maxHead = mOneMachDPData->getJobStep(jobIndex)->head;
		int head, body, delay, temp;
		auto iter = mAllPreds[jobIndex].begin();
		auto end = mAllPreds[jobIndex].end();
		while (iter != end) {
			JobStep* curIter = (*iter)->from;
			head = updateHeadHelper(curIter->jobIndex);
			body = curIter->body;
			delay = (*iter)->delay;
			temp = head + body + delay;
			maxHead = (temp > maxHead) ? temp : maxHead;
			smallHead.insert(pair<int, int>(head, body));
			sumTime += body;
			iter++;
		}
		for (auto iter = smallHead.begin(); iter != smallHead.end(); iter++) {
			temp = (*iter).first + sumTime;
			maxHead = (temp > maxHead) ? temp : maxHead;
			sumTime -= (*iter).second;
		}
		mUpdatedHead[jobIndex] = maxHead;
	}
	//printf("Job %d has head %d.\n", jobIndex, mHeadUpdated[jobIndex]);
	return mUpdatedHead[jobIndex];
}

/************************************************************************************************************
 * Recursively update all tail of jobsteps that have successors												*
 ************************************************************************************************************/
int OneMachDPNode::updateTailHelper(int jobIndex) 
{
	JobStep* curJobStep = mOneMachDPData->getJobStep(jobIndex);
	if (mUpdatedTail[jobIndex] != -1) {
		// if not -1, then tail already updated, nothing needs to be done
	} else if (mAllSuccs.empty() || mAllSuccs[jobIndex].empty()) {
		mUpdatedTail[jobIndex] = curJobStep->tail;
	} else {
		multimap<int, int> smallTail;
		int sumTime = 0, maxTail = curJobStep->tail;
		int tail, body, delay, temp;
		int newMethodTail = 0;
		auto iter = mAllSuccs[jobIndex].begin();
		auto end = mAllSuccs[jobIndex].end();
		while (iter != end) {
			JobStep* curIter = (*iter)->to;
			tail = updateTailHelper(curIter->jobIndex);
			body = curIter->body;
			delay = (*iter)->delay;
			temp = tail + delay + body;
			maxTail = (temp > maxTail) ? temp : maxTail;
			smallTail.insert(pair<int, int>(tail, body));
			sumTime += body;
			iter++;
		}
		for (auto iter = smallTail.begin(); iter != smallTail.end(); iter++) {
			temp = (*iter).first + sumTime;
			maxTail = (temp > maxTail) ? temp : maxTail;
			sumTime -= (*iter).second;
		}
		/***************Experiment Procedure*******************/
		newMethodTail = updateTailBySucc(jobIndex);
		if (newMethodTail > maxTail)
			printf("New tail is larger.");
		maxTail = (newMethodTail > maxTail) ? newMethodTail : maxTail;
		/******************************************************/
		if (curJobStep->isTailUpdated && curJobStep->tail < maxTail)
			maxTail = curJobStep->tail;
		mUpdatedTail[jobIndex] = maxTail;
	}
	//printf("Job %d has tail %d.\n", jobIndex, mTailUpdated[jobIndex]);
	return mUpdatedTail[jobIndex];
}

int OneMachDPNode::updateTailBySucc(int jobIndex)
{
	if (mAllSuccs[jobIndex].front()->delay == 0)
		return 0;		// if no DPC, no need for the procedure
	if (mAllSuccs[jobIndex].size() < 2) 
		return 0;		// if only one or less successor, no need for the procedure

	int bound;
	JobStep* curStep;
	vector<int> jobsToSchdByInd;
	vector<int> orgJobHeads;
	vector<int> orgJobTails;
	list<JobStep*> jobsToSchd;
	jobsToSchdByInd.resize(mOneMachDPData->numJobs, 0);
	orgJobHeads.resize(mOneMachDPData->numJobs, 0);
	orgJobTails.resize(mOneMachDPData->numJobs, 0);

	OneMachDPNode* tempNode = new OneMachDPNode();
	tempNode->mOneMachDPData = mOneMachDPData;
	tempNode->mAllPreds.resize(mOneMachDPData->numJobs);
	tempNode->mAllSuccs.resize(mOneMachDPData->numJobs);

	auto iter = mAllSuccs[jobIndex].begin();
	auto end = mAllSuccs[jobIndex].end();
	curStep = (*iter)->from;
	while (iter != end) {
		jobsToSchdByInd[(*iter)->to->jobIndex] = 1;
		jobsToSchd.push_back((*iter)->to);
		orgJobHeads[(*iter)->to->jobIndex] = (*iter)->to->head;		// record head info
		orgJobTails[(*iter)->to->jobIndex] = (*iter)->to->tail;		// record tail info
		(*iter)->to->head = (*iter)->delay;							// set head based on delay
		(*iter)->to->tail = mUpdatedTail[(*iter)->to->jobIndex];	// set tail based on updated tail
		for (auto i = mAllSuccs[(*iter)->to->jobIndex].begin(); i != mAllSuccs[(*iter)->to->jobIndex].end(); i++) {
			if (jobsToSchdByInd[(*i)->to->jobIndex] == 1)
				tempNode->addFixNoChk((*iter)->to, (*i)->to, 0);	// delay info not useful in preemptive version, set to 0
		}
		iter++;
	}
	bound = mOneMachDPData->mComputeBounds->getLBStd(tempNode, jobsToSchd);
	delete tempNode;
	// recover heads and tails of all jobs involved
	iter = mAllSuccs[jobIndex].begin();
	while (iter != end) {
		(*iter)->to->head = orgJobHeads[(*iter)->to->jobIndex];
		(*iter)->to->tail = orgJobTails[(*iter)->to->jobIndex];
		iter++;
	}
	return bound;
}

bool OneMachDPNode::havePrecConstrBasic(int fromIndex, int toIndex) {
	for (auto iter = allFixes.begin(); iter != allFixes.end(); iter++) {
		if ((*iter).from->jobIndex == fromIndex && (*iter).to->jobIndex == toIndex)
			return true;
	}
	return false;
}

/************************************************************************************************************
 * Check for precedence constraints, remGap to control recursive level										*
 ************************************************************************************************************/
bool OneMachDPNode::havePrecConstr(int fromIndex, int toIndex, int remGap) 
{
	if (fromIndex == toIndex) return false;
	if (mAllSuccs[fromIndex].empty()) return false;
	for (auto iter = mAllSuccs[fromIndex].begin(); iter != mAllSuccs[fromIndex].end(); iter++) {
		if (((*iter)->to)->jobIndex == toIndex)
			return true;
	}
	if (remGap > 0) {
		remGap--;
		for (auto iter = mAllSuccs[fromIndex].begin(); iter != mAllSuccs[fromIndex].end(); iter++) {
			if (havePrecConstr((*iter)->to->jobIndex, toIndex, remGap)) {
				addFixNoChk(mOneMachDPData->mJobsByIndex[fromIndex], mOneMachDPData->mJobsByIndex[toIndex], 0);
				return true;
			}
		}
	}
	return false;
}

bool OneMachDPNode::haveDPC(int fromIndex, int toIndex) 
{
	int delay = mOneMachDPData->mInitFixDPDelay[fromIndex][toIndex];
	return (delay > 0);
}

void OneMachDPNode::fillInPos()
{
	int count = 0;
	mPosInPathByJob.clear();
	mIndInPathByPos.clear();
	mPosInPathByJob.resize(mOneMachDPData->numJobs);
	mIndInPathByPos.resize(mOneMachDPData->numJobs);
	for (auto iter = mSolPath.begin(); iter != mSolPath.end(); iter++, count++) {
		mPosInPathByJob[(*iter)->jobIndex] = count;
		mIndInPathByPos[count] = (*iter)->jobIndex;
	}
}

int OneMachDPNode::numJobsBtwn(int fromIndex, int toIndex) 
{
	if (mPosInPathByJob.empty()) {
		int count = 0;
		mPosInPathByJob.resize(mOneMachDPData->numJobs);
		for (auto iter = mSolPath.begin(); iter != mSolPath.end(); iter++, count++) {
			mPosInPathByJob[(*iter)->jobIndex] = count;
		}
	}
	return abs(mPosInPathByJob[fromIndex] - mPosInPathByJob[toIndex] - 1);
}

int OneMachDPNode::branchingScenario() 
{				
	int sumPath;									// The sum of body and delay on the testing path (excluding first head and last tail)
	int minNumPrec = MaxInt;						// Set the min number of precedence arcs in a path to number of fixes (large number)
	int maxStpAftCrit = 0;
	int tempSumProc, tempMinHead, tempMinTail;
	bool hasWeakCaseOne = false;
	bool hasStrgBrch = false;
	critPathCheck curCheck;
	list<JobStep*> curPath;

	auto pathIter = mOneMachDPData->mCritPathes->mAllCritPath.begin();
	auto pathEnd = mOneMachDPData->mCritPathes->mAllCritPath.end();
	while (pathIter != pathEnd) {
		curPath = *pathIter; 
		if (curPath.size() == 1) {
			mCritPath = curPath;
			return 0;
		}
		// Current path is a critical path, then check the type
		curCheck = checkCritPath(curPath);
		if (curCheck.type == 0) {
			// Type 0: current solution optimal
			mCritPath = curPath;
			return curCheck.type;
		} else if (curCheck.type == 1) {
			// Type 1: current solution can strong branch
			hasStrgBrch = true;
			// Choose the strong branching option where we can fix the most edges
			if (curCheck.stepsAftCritStep > maxStpAftCrit) {
				mCritPath = curPath;
				mCritStep = curCheck.critJob;
				tempSumProc = mSumProc;
				tempMinHead = mMinHead;
				tempMinTail = mMinTail;
			}
		} else if (curCheck.type == 2) {
			if (hasStrgBrch) {
				// has strong branching
			} else {
				// Type 2: current solution may be weak branch case 1 (we can only know after all critPath examined)
				mCritPath = curPath;
				mCritStep = curCheck.critJob;

				hasWeakCaseOne = true;
			}
		} else {
			// Type 3: current solution may be weak branch case 2 (we can only know after all critPath examined)
			if (hasStrgBrch || hasWeakCaseOne) {
				// If weak branch case 1 critPath exist, we will be in case 1 of weak branching so no need to record here
				// In this case, we are still checking if Type 0 and 1 are possible.
			} else if (curCheck.numPrecInCritPath < minNumPrec) {
				// Otherwise, we will be in case 2 of weak branching, store the critical path with smallest number of precedence constraints
				mCritPath = curPath;
				mCritStep = curCheck.critJob;
				mLastPrecStep = curCheck.lastPrecStep;

				minNumPrec = curCheck.numPrecInCritPath;
			}
		}
		pathIter++;
	}

	if (isInMap) {
		if (hasStrgBrch) {
			mSumProc = tempSumProc;
			mMinHead = tempMinHead;
			mMinTail = tempMinTail;
			return 1;
		}
		if (hasWeakCaseOne) {
			int curBest = MaxInt;
			int gap;
			auto tempIter = getStepInCritPath(mCritStep->jobIndex);
			tempIter++;
			for (; tempIter != mCritPath.end(); tempIter++) {
				gap = numJobsBtwn(mCritStep->jobIndex, (*tempIter)->jobIndex);
				if ((*tempIter)->head < curBest && !havePrecConstr(mCritStep->jobIndex, (*tempIter)->jobIndex, gap)) {
					curBest = (*tempIter)->head;
					mSpecialStep = *tempIter;
				}
			}
			return 2;
		} else {
			int curBest = MaxInt;
			int gap;
			auto tempIter = getStepInCritPath(mLastPrecStep->jobIndex);
			tempIter++;
			for (; tempIter != mCritPath.end(); tempIter++) {
				gap = numJobsBtwn(mLastPrecStep->jobIndex, (*tempIter)->jobIndex);
				if ((*tempIter)->head < curBest && !havePrecConstr(mLastPrecStep->jobIndex, (*tempIter)->jobIndex, gap)) {
					//printf("Current step: %d, with head %d.\n", (*tempIter)->jobIndex, (*tempIter)->head);
					curBest = (*tempIter)->head;
					mSpecialStep = *tempIter;
				}
			}
			return 3;
		}
	} else {
		// With reverse problem, weak case one or two does not matter (as they will be discarded)
		return (hasWeakCaseOne) ? 2 : 3;
	}
}

critPathCheck OneMachDPNode::checkCritPath(list<JobStep*> &critPath) 
{
	auto iter = critPath.rbegin();
	auto end = critPath.rend();
	auto prev = iter;
	
	iter++;
	JobStep* cirtStep;
	JobStep* rels = critPath.back();
	JobStep* lastPred = nullptr;										// The start of the last precedence arc
	int firstSchdTime, critStepSchdTime;
	int lastTail = rels->tail;
	int predCount = 0;													// Counter for precedence arcs
	int stepsAftCrit = 1;

	// variables used in fixmore(), since it only occures for strong branching, variables can be updated here.
	mSumProc = rels->body;												// Record process time for searched part of solution
	mMinHead = rels->head;												// For computing local LB
	mMinTail = lastTail;												// For computing local LB

	firstSchdTime = mJobScheduled[critPath.front()->jobIndex];
	
	while (iter != end) {
		/* Only check DPC. */
		if (haveDPC((*iter)->jobIndex, (*prev)->jobIndex)) {
			if (predCount == 0) {
				lastPred = *prev;
				if (prev == critPath.rbegin()) {
					if (isInMap)
						throw ERROR << "Precedence arc should not appear on final node on critical path.\n";
					else
						// Just return something when dealing with reverse problem
						return critPathCheck(3, MaxInt, stepsAftCrit, *prev, lastPred);
				}
			}
			predCount++;
		}

		if ((*prev)->head < mMinHead) {
			mMinHead = (*prev)->head;
			rels = *prev;
		}
		
		if ((*iter)->tail < lastTail) {
			cirtStep = *iter;
			critStepSchdTime = mJobScheduled[cirtStep->jobIndex];
			if (predCount == 0) {
				if (mMinHead >= firstSchdTime && mMinHead >= critStepSchdTime) {
					// Type 1: No precedence arcs and satisfy strong branching
					return critPathCheck(1, predCount, stepsAftCrit, cirtStep, nullptr);
				} else {
					// Type 2: No precedence arcs but does not satisfy strong branching
					return critPathCheck(2, predCount, stepsAftCrit, cirtStep, nullptr);
				}
			} else {
				// Type 3£º Precedence arcs exist
				return critPathCheck(3, predCount, stepsAftCrit, cirtStep, lastPred);
			}
		}
		
		prev = iter;
		mSumProc += (*iter)->body;
		stepsAftCrit++;
		iter++;
	}
	if (predCount > 0)
		// Type 3: Precedence arcs exist
		return critPathCheck(3, predCount, stepsAftCrit, critPath.front(), lastPred);
	if (mMinHead >= firstSchdTime)
		// Type 0: No precedence arcs and no critical step, optimal
		return critPathCheck(0, predCount, stepsAftCrit, nullptr, nullptr);
	else
		// Type 2: No precedence arcs but does not satisfy strong branching
		return critPathCheck(2, predCount, stepsAftCrit, critPath.front(), nullptr);
}

void OneMachDPNode::updateHeadInSol()
{
	int curIndex, temp;
	auto iter = mSolPath.begin();
	auto end = mSolPath.end();
	mLongestToCur.clear();
	mLongestToCur.resize(mOneMachDPData->numJobs);
	while (iter != end) {
		mLongestToCur[(*iter)->jobIndex] = (*iter)->head;
		iter++;
	}
	iter = mSolPath.begin();
	while (iter != end) {
		curIndex = (*iter)->jobIndex;
		for (auto i = mAllSuccs[curIndex].begin(); i != mAllSuccs[curIndex].end(); i++) {
			temp = mJobScheduled[curIndex] + (*iter)->body + (*i)->delay;
			mLongestToCur[(*i)->to->jobIndex] = (mLongestToCur[(*i)->to->jobIndex] < temp) ? temp : mLongestToCur[(*i)->to->jobIndex];
		}
		iter++;
	}
}

/************************************************************************************************************
 * Check if there exists delayed job, and reschedule when necessary											*
 ************************************************************************************************************/
int OneMachDPNode::reschCritPathes()
{
	auto pathIter = mOneMachDPData->mCritPathes->mAllCritPath.begin();
	auto pathEnd = mOneMachDPData->mCritPathes->mAllCritPath.end();
	int targetIndex, targetHead, targetPos, posToPut, posToPutIndex, prePosToPutIndex;
	int pathTopPos;
	int gap, preStepEnd;
	JobStep* target;

	while (pathIter != pathEnd) {
		target = findDelayedJob(*pathIter);
		// target is not nullptr means such late job step exist in current path
		if (target != nullptr) {
			pathTopPos = mPosInPathByJob[((*pathIter).front())->jobIndex];
			targetIndex = target->jobIndex;
			targetPos = mPosInPathByJob[targetIndex];
			targetHead = mLongestToCur[targetIndex];
			// find the earliest position target job can be in
			for (posToPut = 0; posToPut < mOneMachDPData->numJobs; posToPut++) {
				if (targetHead < mJobScheduled[mIndInPathByPos[posToPut]])
					break;
			}
			
			// insert job to right before first job in critical path
			posToPut = pathTopPos;
			posToPutIndex = mIndInPathByPos[posToPut];

			// if posTo less or equal to pathTopPos, then gap exists and reschedule will happen
			if (posToPut <= pathTopPos) {
				for (auto iter = mSolPath.begin(); iter != mSolPath.end(); iter++) {
					// insert to new position
					if ((*iter)->jobIndex == posToPutIndex) {
						mSolPath.insert(iter, target);
						continue;
					}
					// delete from original position
					if ((*iter)->jobIndex == targetIndex) {
						mSolPath.erase(iter);
						break;
					}
				}
				// update position/index vectors
				fillInPos();
				return posToPut;
			}
		}
		pathIter++;
	}
	return -1;
}

/************************************************************************************************************
 * Find delayed job																							*
 ************************************************************************************************************/
JobStep* OneMachDPNode::findDelayedJob(list<JobStep*> &critPath)
{
	auto iter = critPath.begin();
	auto end = critPath.end();
	int firstSchdTime, curSchdTime, curIndex;

	firstSchdTime = mJobScheduled[(*iter)->jobIndex];
	iter++;
	while (iter != end) {
		curIndex = (*iter)->jobIndex;
		curSchdTime = mLongestToCur[curIndex];
		if (curSchdTime < firstSchdTime)
			return (*iter);
		iter++;
	}
	return nullptr;
}

void OneMachDPNode::addFix(JobStep* from, JobStep* to, int delay) 
{
	if (from->jobIndex == to->jobIndex)
		return;
	// Only add if the edge is not previously present
	if (!havePrecConstr(from->jobIndex, to->jobIndex, 0)) {
		allFixes.push_back(fixedEdge(from, to, delay));
		fixedEdge* temp = &(allFixes.back());
		mAllPreds[to->jobIndex].push_back(temp);
		mAllSuccs[from->jobIndex].push_back(temp);
	}
}

void OneMachDPNode::addFixNoChk(JobStep* from, JobStep* to, int delay) 
{
	allFixes.push_back(fixedEdge(from, to, delay));
	fixedEdge* temp = &(allFixes.back());
	mAllPreds[to->jobIndex].push_back(temp);
	mAllSuccs[from->jobIndex].push_back(temp);
}

/************************************************************************************************************
 * Alternative fix addition, only add to allFixes list														*
 ************************************************************************************************************/
void OneMachDPNode::addFixBasic(JobStep* from, JobStep* to, int delay) 
{
	if (from->jobIndex == to->jobIndex)
		return;
	if (!havePrecConstrBasic(from->jobIndex, to->jobIndex))
		allFixes.push_back(fixedEdge(from, to, delay));
}

void OneMachDPNode::cleanConstrs() 
{
	mAllPreds.clear();
	mAllSuccs.clear();
	mUpdatedHead.clear();
	mUpdatedTail.clear();
}

void OneMachDPNode::clearAll() 
{
	cleanConstrs();
	allFixes.clear();
}

iterJobs OneMachDPNode::getStepInSol(int stepIndex) 
{
	for (iterJobs iter = mSolPath.begin(); iter != mSolPath.end(); iter++) {
		if (stepIndex == (*iter)->jobIndex)
			return iter;
	}
	throw ERROR << "Jobstep lookup error";
}

iterJobs OneMachDPNode::getStepInCritPath(int stepIndex) 
{
	for (iterJobs iter = mCritPath.begin(); iter != mCritPath.end(); iter++) {
		if (stepIndex == (*iter)->jobIndex)
			return iter;
	}
	throw ERROR << "Jobstep lookup error";
}

OneMachDPNode* OneMachDPNode::revNode() 
{
	OneMachDPNode* rev = new OneMachDPNode();
	// This node is not going to be inserted into map
	rev->isInMap = false;
	// Tail as head and head as tail
	rev->mOneMachDPData = mOneMachDPData;
	rev->mUpdatedHead = mUpdatedTail;
	rev->mUpdatedTail = mUpdatedHead;
	// Initialize precedence vectors
	rev->mAllPreds.resize(mOneMachDPData->numJobs);
	rev->mAllSuccs.resize(mOneMachDPData->numJobs);
	// Reverse each precedence constraints
	auto iter = allFixes.begin();
	auto end = allFixes.end();
	while (iter != end) {
		rev->addFix((*iter).to, (*iter).from, (*iter).delay);
		iter++;
	}
	return rev;
}

void OneMachDPNode::printPath(FILE* inFile) 
{
	if (inFile == nullptr) return;
	for (auto iter : mSolPath) {
		fprintf(inFile, "%d ", iter->jobIndex);
	}
	fprintf(inFile, "\n");
}

void OneMachDPNode::printCritPath(FILE* inFile) 
{
	if (inFile == nullptr) return;
	for (auto iter : mCritPath) {
		fprintf(inFile, "%d ", iter->jobIndex);
	}
	fprintf(inFile, "\n");
}

OneMachDPNode::~OneMachDPNode() 
{
	clearAll();
	if (isInMap)
		mOneMachDPData->delNode(this);
}

void OneMachDPNode::resetcurHead() 
{
	list<JobStep>::iterator iter = mOneMachDPData->mJobsData.begin();
	list<JobStep>::iterator end = mOneMachDPData->mJobsData.end();
	while (iter != end) {
		(*iter).curHead = -1;
		iter++;
	}
}