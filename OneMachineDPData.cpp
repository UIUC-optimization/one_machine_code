#include "OneMachineDP.h"

OneMachDPData::OneMachDPData(const char* filename)
{
	ifstream inFile(filename);
	int head, body, tail, job;
	int fix, from, to, delay;
	getline(inFile, mOneMachineName);
	inFile >> numJobs;
	mInitFixDPDelay.resize(numJobs);
	mMode = BFS;
	mTbMode = ARB;
	mTimeLim = 3600;
	mIterLim = 100000;
	mTerminateMode = -1;
	for (job = 0; job < numJobs; job++) {
		if (inFile.eof()) {
			printf("Something is wrong. Job read stop at: %d\n", job);
			throw ERROR << "Job read erroe.";
		}
		inFile >> head >> body >> tail;
		mJobsData.push_back(JobStep(job, body, head, tail));
		mInitHead.push_back(head);
		mInitTail.push_back(tail);
		mInitFixDPDelay[job].resize(numJobs, 0);
		//printf("Job %d has head %d, body %d, tail %d.\n", job, head, body, tail);
	}
	if (!inFile.eof()) {
		inFile >> numInitFix;
		for (fix = 0; fix < numInitFix; fix++) {
			inFile >> from >> to >> delay;
			mInitFix.push_back(fixedEdge(getJobStep(from), getJobStep(to), delay));
			mInitFixDPDelay[from][to] = delay;
			//printf("Job %d precedes job %d, with DP at %d.\n", from, to, delay);
		}
	} else {
		numInitFix = 0;
	}
}

/************************************************************************************************************
 * Initialization of options																				*
 ************************************************************************************************************/
void OneMachDPData::initialize(options* opt)
{
	curID = 0;
	numToExplore = 0; numIter = 0; numNodes = 0;
	globUB = MaxInt;
	globLB = initLB = 0;
	mNumLLTH = 0;
	// control parameters
	mMesrBest = 1;
	//mCombineOnRev = false;
	leftConst = 0, rightConst = 1;
	maxDepth = 0;

	mJobsByIndex.resize(numJobs);
	// Initialize problem options here
	if (opt != nullptr) {
		mTimeLim = opt->timeLimit;
		mIterLim = opt->iterationLimit;
		mMode = opt->mod;
		mTbMode = opt->tb;
		mRevChkOn = opt->revChk;
		mCombineOn = opt->heuChk;
	}
	for (auto job = mJobsData.begin(); job != mJobsData.end(); job++) {
		mJobsByIndex[(*job).jobIndex] = &(*job);
	}
	// Initialize Modules here
	mComputeBounds = new OneMachDPBounds(this);
	mRevCritPathes = new OneMachDPCritPath(this);
	mCritPathes = new OneMachDPCritPath(this);
	mBranching = new OneMachDPBranch(this);
	mPost = new OneMachDPPost(this);
	mComputeBounds->initialize();
	mCritPathes->initialize();
	mRevCritPathes->initialize();
	mBranching->initialize();
	mPost->initialize();
}

/************************************************************************************************************
 * Solve instance																							*
 ************************************************************************************************************/
int OneMachDPData::solve()
{
	// In each iteration, first explore the next subproblem by using
	// getNextNode(), then generate new nodes if necessary, delete the explored
	// one, and move on to the next iteration.
	myclock::time_point beginning = myclock::now();
	myclock::duration d;
	int flag, tempLB;
	OneMachDPNode* root = new OneMachDPNode(this);

	OneMachDPNode* curNode;
	addNode(root);
	//mBstSolNode = new OneMachDPNode(root);
	setIterator();
	while (numToExplore > 0) {

		// Termination check: time limit
		d = myclock::now() - beginning;
		mElapsTime = chrono::duration_cast<std::chrono::milliseconds>(d).count();
		if ( (mElapsTime / 1000) > mTimeLim) {
			mTerminateMode = 2;
			dumpAllNodes();
			return globUB;
		}
		// Termination check: iteration limit
		if (numIter >= mIterLim) {
			mTerminateMode = 1;
			dumpAllNodes();
			return globUB;
		}

		curNode = getNextNode();
		if (curNode->mLBound < globUB) {
			//if (numIter % 10 == 0)
			//	printf("NID    Iter    NLB     rexSol     feaSol    nFix    cLen    gLB    gUB    contr  \n");
			flag = solveNode(curNode);
			//switch (flag) {
			//case 0:
			//	printf("%d    %d    %d    %d    %d    %d    %d    %d    %d    %d  \n",
			//		curNode->mNodeID, numIter, curNode->mLBound, curNode->mRexSol, curNode->mFeaSol, curNode->allFixes.size(),
			//		curNode->mCritPath.size(), globLB, globUB, curNode->mContour);
			//	printf("At iter: %d, feasible solution %d found.\n", numIter, curNode->mFeaSol);
			//	break;
			//case 1:
			//	printf("%d    %d    %d    %d    %d    %d    %d    %d    %d    %d  \n",
			//		curNode->mNodeID, numIter, curNode->mLBound, curNode->mRexSol, curNode->mFeaSol, curNode->allFixes.size(),
			//		curNode->mCritPath.size(), globLB, globUB, curNode->mContour);
			//	printf("At iter: %d, strong branching is applied on node %d.\n",
			//		numIter, curNode->mNodeID);
			//	break;
			//case 2:
			//	printf("%d    %d    %d    %d    %d    %d    %d    %d    %d    %d  \n",
			//		curNode->mNodeID, numIter, curNode->mLBound, curNode->mRexSol, curNode->mFeaSol, curNode->allFixes.size(),
			//		curNode->mCritPath.size(), globLB, globUB, curNode->mContour);
			//	printf("At iter: %d, weak branching case 1 is applied on node %d.\n",
			//		numIter, curNode->mNodeID);
			//	break;
			//case 3:
			//	printf("%d    %d    %d    %d    %d    %d    %d    %d    %d    %d  \n",
			//		curNode->mNodeID, numIter, curNode->mLBound, curNode->mRexSol, curNode->mFeaSol, curNode->allFixes.size(),
			//		curNode->mCritPath.size(), globLB, globUB, curNode->mContour);
			//	printf("At iter: %d, weak branching case 2 is applied on node %d.\n",
			//		numIter, curNode->mNodeID);
			//	break;
			//case 4:
			//	printf("%d    %d    %d    %d    %d    %d    %d    %d    %d    %d  \n",
			//		curNode->mNodeID, numIter, curNode->mLBound, curNode->mRexSol, -1, curNode->allFixes.size(),
			//		-1, globLB, globUB, curNode->mContour);
			//	printf("At iter: %d, node LB %d exceed global upper bound, node %d will be pruned.\n",
			//		numIter, curNode->mRexSol, curNode->mNodeID);
			//	break;
			//}

			//if (mInfoFile != nullptr) {
			//	fprintf(mInfoFile, "%d    %d    %d    %d    %d    %d    %d    %d    %d    %d    %d    %d  \n", numIter,
			//		flag, curNode->mNodeID, curNode->mLBound, curNode->mRexSol, curNode->mFeaSol, curNode->allFixes.size(),
			//		curNode->mCritPath.size(), globLB, globUB, curNode->mContour, curNode->mDepth);
			//	if (flag == 1 || flag == 2 || flag == 3)
			//		fprintf(mInfoFile, "%d    %d    %d    %d    %d  \n", numIter, flag, curNode->mNodeID,
			//			mBranching->curLeft, mBranching->curRight);
			//}
			//printInfo(curNode);

			numIter++;
		} else {
			//printf("At iter: %d, parent LB %d exceed global upper bound, node %d will be pruned.\n",
			//	numIter, curNode->mLBound, curNode->mNodeID);
			// If pruned without exploration, flag is set at 5
			flag = 5;
			revToPrevContour();
			//if (mInfoFile != nullptr) {
			//	fprintf(mInfoFile, "%d    %d    %d    %d    %d    %d    %d    %d  \n", numIter, flag, curNode->mNodeID,
			//		curNode->mLBound, globLB, globUB, curNode->mContour, curNode->mDepth);
			//}
		}

		if (numIter != 1) {
			tempLB = getCurLB();
			if (tempLB <= globUB)
				globLB = tempLB;
		}

		delete curNode;
	}
	mTerminateMode = 0;
	return globUB;
}

/************************************************************************************************************
 * Explore node																								*
 * return value: 0 optimal																					*
 *				 1 strong branching																		    *
 *               2 weak branching 1																			*
 *               3 weak branching 2																			*
 *               4 pruned by LB																		        *
 ************************************************************************************************************/
int OneMachDPData::solveNode(OneMachDPNode* node)
{
	int flag, uBound, BrchScn;
	int redoCount = 0;
	bool useNLT;
	bool newBest = false;				// flag for whether new best solution found
	vector<int> tempSchd;
	list<JobStep*> tempSolPath;
	int tempFeaSol;
	int tempLB;

	//node->doedge();
	node->populateFixes();
	resetTailUpdateChk();				// reset all tail update check for all job steps

	// Get UB
	do {
		useNLT = false;					// initialize useNLT indicator for the use of new heuristic

		node->updateEdge();
		node->doedge();
		resetTailUpdateChk();			// reset all tail update check for all job steps
		// regular LT algorithm schedule
		uBound = mComputeBounds->getUB(node);
		// modified LT algorithm schedule
		if (mCombineOn) {
			tempSchd = node->mJobScheduled;
			tempSolPath = node->mSolPath;
			tempFeaSol = node->mFeaSol;
			//uBound = LLTHs(node);
			uBound = mComputeBounds->getUBMod2(node);
			useNLT = true;

			// always use rescheduling
			//node->fillInPos();
			//mCritPathes->main(node);
			//uBound = cleanNLTPathes(node);

			if (uBound >= tempFeaSol) {
				useNLT = false;
				node->mJobScheduled = tempSchd;
				node->mSolPath = tempSolPath;
				node->mFeaSol = tempFeaSol;
				uBound = tempFeaSol;
			}
		}
		
		// reverse algorithm schedule (not activated)
		//if (mCombineOnRev) {
		//	OneMachDPNode* rev = node->revNode();
		//	rev->doedge();
		//	mComputeBounds->getUB(rev);
		//	(rev->mSolPath).reverse();
		//	// Reset job parameters
		//	node->doedge();
		//	// This step updates the node currect mJobScheduled array
		//	rev->mFeaSol = calSolution(rev);
		//	if (rev->mFeaSol < uBound) {
		//		useNLT = false;
		//		node->mJobScheduled = rev->mJobScheduled;
		//		node->mSolPath = rev->mSolPath;
		//		node->mFeaSol = rev->mFeaSol;
		//		uBound = node->mFeaSol;
		//	}
		//	delete rev;
		//}
		
		// Update mPosInPathByJob array in node based on current solution
		node->fillInPos();
		mCritPathes->main(node);

		if (useNLT) {
			uBound = chkDelayJobCritPathes(node);
			if (uBound >= tempFeaSol) {
				useNLT = false;
				node->mJobScheduled = tempSchd;
				node->mSolPath = tempSolPath;
				node->mFeaSol = tempFeaSol;
				uBound = tempFeaSol;

				node->fillInPos();
				mCritPathes->main(node);
			}
		}

		redoCount++;
	} while (mPost->main(node));
	
	// Get Branching Scenario for current node
	BrchScn = node->branchingScenario();
	// If weak branching is the only option, check reverse problem when reverse check is on
	if (BrchScn > 1 && mRevChkOn) {
		int revBrchScn = solveRevNode(node);
		if (revBrchScn == 0 || revBrchScn == 1) {
			BrchScn = revBrchScn;
			mBranching->increRevCount();
		}
	}

	// If LLTH solution is applied, check the head situation.
	if (useNLT) {
		mNumLLTH++;
		if (node->findDelayedJob(node->mCritPath) < 0) {
			printf("Violation of assumption.\n");
			throw ERROR << "Violation of assumption.\n";
		}
	}

	// Update global solution when current solution is better
	if (globUB > node->mFeaSol) {
		globUB = node->mFeaSol;
		mBstSolPath = node->mSolPath;
		bstFoundAtIter = numIter;
		bstFoundCritSize = node->mCritPath.size();
		//mBstSolNode->copyNode(node);
	}

	// For root node calculate LB specifically for init_lb
	if (node->mDepth == 0) {
		mComputeBounds->getLBStd(node);
		globLB = node->mRexSol;
		initLB = globLB;
		mRexSolCount[globLB] = 1;
	}
	flag = mBranching->main(node, BrchScn);

	// Clean up
	mCritPathes->clearPathes();
	mRevCritPathes->clearPathes();
	node->undoedge();
		
	return flag;
}

/************************************************************************************************************
 * Check all variations of LLTH																				*
 * NOTE: may conflict with post processing, currently not used												*
 ************************************************************************************************************/
int OneMachDPData::LLTHs(OneMachDPNode* node)
{
	vector<int> tempSchd;
	list<JobStep*> tempSolPath;
	int tempFeaSol, uBound;

	uBound = mComputeBounds->getUBMod2(node);
	tempSchd = node->mJobScheduled;
	tempSolPath = node->mSolPath;
	tempFeaSol = node->mFeaSol;
	uBound = mComputeBounds->getUBMod(node);
	if (uBound < tempFeaSol) {
		tempSchd = node->mJobScheduled;
		tempSolPath = node->mSolPath;
		tempFeaSol = node->mFeaSol;
	} 
	uBound = mComputeBounds->getUBMod3(node);
	if (uBound >= tempFeaSol) {
		node->mJobScheduled = tempSchd;
		node->mSolPath = tempSolPath;
		node->mFeaSol = tempFeaSol;
		uBound = tempFeaSol;
	}
	return uBound;
}

/************************************************************************************************************
 * Determine if rescheduling is necessary																	*
 ************************************************************************************************************/
int OneMachDPData::chkDelayJobCritPathes(OneMachDPNode* node)
{
	int posToPut;
	int sol = node->mFeaSol;					// set to current solution
	while (true) {
		posToPut = node->reschCritPathes();
		if (posToPut < 0)
			break;
		// recalculate makespan
		sol = calSolution(node);
		node->mFeaSol = sol;
		node->updateHeadInSol();
		// recalculate critical pathes
		mCritPathes->main(node);
	}
	return sol;
}

/************************************************************************************************************
 * Reverse heads and tail of input node, solve that to get an alternative schedule							*
 ************************************************************************************************************/
int OneMachDPData::solveRevNode(OneMachDPNode* node) 
{
	int revBrchScn;
	OneMachDPCritPath* temp;
	OneMachDPNode* rev = node->revNode();
	OneMachDPNode* repNode = new OneMachDPNode();
	mUtil->copyNodeBasic(node, repNode);

	// Solve the reverse problem, and reverse the resulting schedule to get a good feasbile schedule for original problem
	rev->doedge();
	mComputeBounds->getUB(rev);
	(rev->mSolPath).reverse();

	// Give this schedule to repNode (this node will have currect order of precedence and updated parameter)
	repNode->isInMap = false;
	repNode->mSolPath = rev->mSolPath;
	// Reset job parameters
	node->doedge();
	// This step updates the node currect mJobScheduled array
	repNode->mFeaSol = calSolution(repNode);

	// Get all critical path for this schedule
	mRevCritPathes->main(repNode);
	// Post still needed but will not trigger rework even if tail has changed
	//mUtil->swapCritPathes(mCritPathes, mRevCritPathes);
	temp = mCritPathes;
	mCritPathes = mRevCritPathes;
	mRevCritPathes = temp;

	mPost->main(repNode);
	printf("Start checking reverse problem solution...\n");
	// Get branching scenario for repNode
	revBrchScn = repNode->branchingScenario();
	// If scenario is optimal or strong branching, then accept this solution
	if (revBrchScn == 0 || revBrchScn == 1) {
		// when reverse branching is used, we need to update node with new schedule
		// mJobScheduled is not updated to reduce work
		mUtil->copyNodeSolInfo(repNode, node);
	} else {
		// otherwise switch critpathes back
		//mUtil->swapCritPathes(mCritPathes, mRevCritPathes);
		temp = mCritPathes;
		mCritPathes = mRevCritPathes;
		mRevCritPathes = temp;
	}
	delete rev;
	delete repNode;
	return revBrchScn;
}

/************************************************************************************************************
 * Insert node into contour																					*
 ************************************************************************************************************/
void OneMachDPData::addNode(OneMachDPNode* node)
{
	int best;
	// choice of measure of best function: lower bound / parent feasible solution
	switch (mMesrBest) {
	case 1:
		best = node->mLBound;
		break;
	case 2:
		best = node->mParentSol;
		break;
	default:
		best = node->mLBound;
	}
	if (mMode == DFS) {
		mNodesStack.push(node);
	} else {
		node->mContour = calContour(node);
		mContours[node->mContour].insert({ best, node });
	}

	int lb = node->mLBound;
	if (mLowerBd.count(lb) == 0) {
		mLowerBd[lb] = 1;
	} else {
		mLowerBd[lb]++;
	}
	int rs = node->mRexSol;
	if (node->mDepth != 0)
	{
		if (mRexSolCount.count(rs) == 0)
			mRexSolCount[rs] = 1;
		else
			mRexSolCount[rs]++;
	}
	if (node->mDepth > maxDepth)
		maxDepth = node->mDepth;
	numToExplore++;
	numNodes++;
}

OneMachDPNode* OneMachDPData::getNextNode()
{
	int n;
	OneMachDPNode* out;
	vector<OneMachDPNode*> candidates;
	// NOTE: what if we don't update mCurContour every iteration?
	if (mMode == DFS) {
		out = mNodesStack.top();
		mNodesStack.pop();
	}
	else {
		mPreContour = mCurContour;
		++mCurContour;
		if (mCurContour == mContours.end())
			mCurContour = mContours.begin();
		// There should always be a node in here
		int search;
		search = ((mCurContour->second).begin())->first;
		auto range = mCurContour->second.equal_range(search);
		// choice of tie breaking rules
		switch (mTbMode) {
		case FIFO:
			// FIFO tie breaking
			out = (range.first)->second;
			break;
		case MinParent:
			// Tie breaking by choosing the one with the smallest mParentSol
			out = (range.first)->second;
			for (auto iter = range.first; iter != range.second; iter++) {
				if ((iter->second)->mParentSol < out->mParentSol)
					out = iter->second;
			}
			break;
		case LIFO:
			// LIFO tie breaking
			out = (--range.second)->second;
			break;
		case ARB:
			// Arbitrary Tie Breaking
			for (auto iter = range.first; iter != range.second; iter++)
				candidates.push_back(iter->second);
			n = rand() % candidates.size();
			out = candidates[n];
			break;
		default:
			// LIFO tie breaking
			out = (--range.second)->second;
			break;
		}
	}
	return out;
}

OneMachDPNode* OneMachDPData::getNextNode(tbMode tbRule)
{
	int n;
	OneMachDPNode* out;
	vector<OneMachDPNode*> candidates;
	// NOTE: what if we don't update mCurContour every iteration?
	if (mMode == DFS) {
		out = mNodesStack.top();
		mNodesStack.pop();
	} else {
		mPreContour = mCurContour;
		++mCurContour;
		if (mCurContour == mContours.end())
			mCurContour = mContours.begin();
		// There should always be a node in here
		int search;
		search = ((mCurContour->second).begin())->first;
		auto range = mCurContour->second.equal_range(search);
		// choice of tie breaking rules
		switch (tbRule) {
		case FIFO:
			// FIFO tie breaking
			out = (range.first)->second;
			break;
		case MinParent:
			// Tie breaking by choosing the one with the smallest mParentSol
			out = (range.first)->second;
			for (auto iter = range.first; iter != range.second; iter++) {
				if ((iter->second)->mParentSol < out->mParentSol)
					out = iter->second;
			}
			break;
		case LIFO:
			// LIFO tie breaking
			out = (--range.second)->second;
			break;
		case ARB:
			// Arbitrary Tie Breaking
			for (auto iter = range.first; iter != range.second; iter++)
				candidates.push_back(iter->second);
			n = rand() % candidates.size();
			out = candidates[n];
			break;
		default:
			// LIFO tie breaking
			out = (--range.second)->second;
			break;
		}
	}
	return out;
}

void OneMachDPData::delNode(OneMachDPNode* toDelete)
{
	int search;
	switch (mMesrBest) {
	case 1:
		search = toDelete->mLBound;
		break;
	case 2:
		search = toDelete->mParentSol;
		break;
	default:
		search = toDelete->mLBound;
		break;
	}
	if (mMode == DFS) {

	} else {
		//int search = toDelete->mParentSol;	
		int targetCont = toDelete->mContour;
		auto range = mContours[targetCont].equal_range(search);

		// Find the nodes that has the same Measure of Best, then find the one with the exact nodeID.
		// I'm still not sure whether I can clear memory by deleting iter (a pointer to the OneMachNode)
		for (auto iter = range.first; iter != range.second; iter++) {
			if ((iter->second)->mNodeID == toDelete->mNodeID) {
				mContours[targetCont].erase(iter);
				break;
			}
		}
		if (mContours[targetCont].empty()) {
			if (mCurContour == mContours.find(targetCont)) {
				if (mCurContour == mContours.begin())
					mCurContour = mContours.end();
				mCurContour--;
			}
			mContours.erase(targetCont);
		}
	}

	int lb = toDelete->mLBound;
	if (mLowerBd.count(lb) == 1 && mLowerBd[lb] > 0) {
		mLowerBd[lb]--;
		if (mLowerBd[lb] == 0)
			mLowerBd.erase(lb);
	} else {
		throw ERROR << "Cannot locate search item.";
	}
	numToExplore--;
}

int OneMachDPData::getCurLB() 
{
	auto lbIter = mLowerBd.begin();
	int lb = lbIter->first;
	int count = lbIter->second;
	if (count <= 0)
		throw ERROR << "Retrive global lower bound error.";
	return lb;
}

void OneMachDPData::dumpAllNodes() 
{
	while (numToExplore > 0) {
		// When dumping, use the simplest tie breaking rule
		OneMachDPNode* curNode = getNextNode(FIFO);
		delete curNode;
	}
}

void OneMachDPData::resetTailUpdateChk() {
	for (int i = 0; i < numJobs; i++) {
		mJobsByIndex[i]->isTailUpdated = false;
	}
}

/************************************************************************************************************
 * Compute the contour the node belongs to																	*
 ************************************************************************************************************/
int OneMachDPData::calContour(OneMachDPNode* node)
{
	int contour;
	switch (mMode) {
	case BFS:
		contour = 0;
		break;
	case DepthCont:
		contour = node->mDepth;
		break;
	case DepthCont2:
		contour = node->mDepth / 2;
		break;
	case DepthCont3:
		contour = node->mDepth / 3;
		break;
	case DepthCont5:
		contour = node->mDepth / 5;
		break;
	case DepthCont10:
		contour = node->mDepth / 10;
		break;
	case numFix3:
		contour = node->allFixes.size() / 3;
		break;
	case numFix5:
		contour = node->allFixes.size() / 5;
		break;
	case numFix10:
		contour = node->allFixes.size() / 10;
		break;
	case numFix15:
		contour = node->allFixes.size() / 15;
		break;
	case numFix20:
		contour = node->allFixes.size() / 20;
		break;
	case WeightCont:
		contour = node->mLweight * leftConst + node->mRweight * rightConst;
		break;
	default:
		contour = 0;
		break;
	}
	return contour;
}

bool OneMachDPData::chkFinalSolution()
{
	int curBst;
	int maxTime = 0;
	int head, body, tail;
	int curIndex;
	int sumPath, temp;
	vector<int> releaseTime;
	list<JobStep*>::iterator iter = mBstSolPath.begin();
	list<JobStep*>::iterator end = mBstSolPath.end();
	releaseTime.resize(numJobs, -1);
	sumPath = (*iter)->head;
	for (; iter != end; iter++) {
		curIndex = (*iter)->jobIndex;
		head = mInitHead[(*iter)->jobIndex];
		if (releaseTime[curIndex] > head)
			head = releaseTime[curIndex];

		body = (*iter)->body;
		tail = mInitTail[(*iter)->jobIndex];
		if (sumPath < head) sumPath = head;
		sumPath += body;
		for (int i = 0; i < numJobs; i++) {
			if (mInitFixDPDelay[curIndex][i] != 0) {
				temp = sumPath + mInitFixDPDelay[curIndex][i];
				releaseTime[i] = (releaseTime[i] < temp) ? temp : releaseTime[i];
			}
		}
		curBst = sumPath + tail;
		if (curBst > maxTime)
			maxTime = curBst;
		//printf("At job %d, curBst is: %d, sumPath is: %d and maxTime is: %d.\n", curIndex, curBst, sumPath, maxTime);
	}
	printf("The time in original problem is %d, and the time after BnB is %d.", maxTime, globUB);
	return (maxTime == globUB);
}

/************************************************************************************************************
 * Calculate makespan for a node with a solution, update starting time for each job							*
 ************************************************************************************************************/
int OneMachDPData::calSolution(OneMachDPNode* node)
{
	int curBst;
	int maxTime = 0;
	int head, body, tail;
	int curIndex;
	int sumPath, temp;
	vector<int> releaseTime;
	list<JobStep*>::iterator iter = node->mSolPath.begin();
	list<JobStep*>::iterator end = node->mSolPath.end();
	node->mJobScheduled.clear();
	node->mJobScheduled.resize(numJobs, -1);
	releaseTime.resize(numJobs, -1);
	sumPath = (*iter)->head;
	for (; iter != end; iter++) {
		curIndex = (*iter)->jobIndex;
		head = mInitHead[(*iter)->jobIndex];
		if (releaseTime[curIndex] > head)
			head = releaseTime[curIndex];

		body = (*iter)->body;
		tail = mInitTail[(*iter)->jobIndex];
		if (sumPath < head) sumPath = head;
		node->mJobScheduled[curIndex] = sumPath;
		//printf("%d  %d  %d  %d  %d\n", curIndex, sumPath, (*iter)->head, body, (*iter)->tail);
		sumPath += body;
		for (int i = 0; i < numJobs; i++) {
			if (mInitFixDPDelay[curIndex][i] != 0) {
				temp = sumPath + mInitFixDPDelay[curIndex][i];
				releaseTime[i] = (releaseTime[i] < temp) ? temp : releaseTime[i];
			}
		}
		curBst = sumPath + tail;
		if (curBst > maxTime)
			maxTime = curBst;
	}
	return maxTime;
}

int OneMachDPData::calSolution(list<JobStep*> &solPath) 
{
	int curBst;
	int maxTime = 0;
	int head, body, tail;
	int curIndex;
	int sumPath, temp;
	vector<int> releaseTime;
	list<JobStep*>::iterator iter = solPath.begin();
	list<JobStep*>::iterator end = solPath.end();
	releaseTime.resize(numJobs, -1);
	sumPath = mInitHead[(*iter)->jobIndex];
	for (; iter != end; iter++) {
		curIndex = (*iter)->jobIndex;
		head = mInitHead[(*iter)->jobIndex];
		if (releaseTime[curIndex] > head)
			head = releaseTime[curIndex];

		body = (*iter)->body;
		tail = mInitTail[(*iter)->jobIndex];
		if (sumPath < head) sumPath = head;
		//printf("%d %d\n", curIndex, sumPath);
		sumPath += body;
		for (int i = 0; i < numJobs; i++) {
			if (mInitFixDPDelay[curIndex][i] != 0) {
				temp = sumPath + mInitFixDPDelay[curIndex][i];
				releaseTime[i] = (releaseTime[i] < temp) ? temp : releaseTime[i];
			}
		}
		curBst = sumPath + tail;
		if (curBst > maxTime)
			maxTime = curBst;
		//printf("At job %d, curBst is: %d, sumPath is: %d and maxTime is: %d.\n", curIndex, curBst, sumPath, maxTime);
	}
	return maxTime;
}

bool OneMachDPData::haveDPC(int fromIndex, int toIndex) 
{
	int delay = mInitFixDPDelay[fromIndex][toIndex];
	return (delay > 0);
}

JobStep* OneMachDPData::getJobStep(int jobIndex)
{
	for (auto iter = mJobsData.begin(); iter != mJobsData.end(); iter++) 
	{
		if ((*iter).jobIndex == jobIndex) return &*iter;
	}
	printf("Fail to retrive jobstep %d.", jobIndex);
	throw ERROR << "Step does not exist.";
}

void OneMachDPData::printJobsteps()
{
	for (auto iter : mJobsData) {
		printf("Jobstep:%d, head: %d, body: %d, tail: %d.\n", iter.jobIndex, iter.head, iter.body, iter.tail);
	}
}

void OneMachDPData::printInfo(OneMachDPNode* curNode) 
{
	curNode->printPath(mPathFile);
	curNode->printCritPath(mCritPathFile);
}

void OneMachDPData::cleanUp() 
{
	delete mComputeBounds;
	delete mRevCritPathes;
	delete mCritPathes;
	delete mBranching;
	delete mPost;
}

/************************************************************************************************************
 * Test heuristics																							*
 ************************************************************************************************************/
int OneMachDPData::testHeuristics() 
{
	int numHeu = 2;
	OneMachDPNode* root = new OneMachDPNode(this);
	addNode(root);
	setIterator();
	testHeuRes.clear();
	testHeuRes.resize(numHeu);
	OneMachDPNode* curNode = getNextNode(FIFO);
	curNode->populateFixes();
	resetTailUpdateChk();
	curNode->updateEdge();
	curNode->doedge();
	testHeuRes[0] = mComputeBounds->getUB(curNode);

	testHeuRes[1] = mComputeBounds->getUBMod2(curNode);
	curNode->fillInPos();
	mCritPathes->main(curNode);
	testHeuRes[1] = chkDelayJobCritPathes(curNode);

	curNode->undoedge();

	delete curNode;

	return numHeu;
}

void OneMachDPData::updatePercentage()
{
	numLNodes = 0; numGNodes = 0;
	for (auto iter = mRexSolCount.begin(); iter != mRexSolCount.end(); iter++)
	{
		if (iter->first < globUB)
			numLNodes += iter->second;
		else if (iter->first > globUB)
			numGNodes += iter->second;
	}
}