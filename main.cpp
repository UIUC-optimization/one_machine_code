#include "OneMachineDP.h"

using namespace std;

int main(int argc,			// Number of strings in array argv  
		char *argv[],		// Array of command-line argument strings  
		char *envp[])		// Array of environment variable strings
{
	char* probPath;
	char* outPath;
	char json[255];
	char problem[255];
	int bodyMax = 50, htCoef = 50;
	int numProb = 1000;
	int prob = 0;

	//const char* path = "D:/ResearchTests/Theory/Practice/omJ50K15S";
	////char* path = "D:/ResearchTests/ForPaper/omJ50K20S";
	//int numJobs = 50, coef = 15;
	//for (prob = 0; prob <= 0; prob += 2) {
	//	sprintf(json, "%s/testResult/02_01_19/Base/oneMach_J%d_B%d_C%d_HC%d_P%d_BFSArb", path, numJobs, bodyMax, coef, htCoef, prob);
	//	for (int i = 0; i < numProb; i++) {
	//		sprintf(problem, "%s/oneMach_J%d_B%d_C%d_HC%d_P%d_%d.txt", path, numJobs, bodyMax, coef, htCoef, prob, i);
	//		srand(static_cast<long unsigned int>(myclock::now().time_since_epoch().count()));
	//		OneMachineDPProblem* machine = new OneMachineDPProblem(problem, 3600, 100000, BFS, ARB, true, false, json, nullptr, nullptr, nullptr);
	//		machine->solve();
	//		machine->printSolToJson();
	//		machine->cleanup();
	//		delete machine;
	//	}
	//}

	const char* paths = "D:/ResearchTests/ForPaper";
	//vector<int> jobs = { 50, 100, 200, 500, 1000 };
	vector<int> jobs = { 50 };
	//vector<int> ks = { 15, 20, 25 };
	vector<int> ks = { 10 };
	for (int numJobs : jobs) {
		for (int coef : ks) {
			for (prob = 1; prob <= 2; prob += 1) {
				char path[255];

				sprintf(path, "%s/omdpJ%dK%dP%dSAE", paths, numJobs, coef, prob);
				//sprintf(path, "%s/omJ%dK%dS", paths, numJobs, coef);

				//sprintf(json, "%s/testResult/06_22_17/Base/oneMach_J%d_B%d_C%d_HC%d_P%d_DepthCont", path, numJobs, bodyMax, coef, htCoef, prob);
				//for (int i = 0; i < numProb; i++) {
				//	sprintf(problem, "%s/oneMach_J%d_B%d_C%d_HC%d_P%d_%d.txt", path, numJobs, bodyMax, coef, htCoef, prob, i);
				//	OneMachineDPProblem* machine = new OneMachineDPProblem(problem, 3600, 100000, DepthCont, LIFO, true, false, json, nullptr, nullptr, nullptr);
				//	//machine->testRun();
				//	machine->solve();
				//	machine->printSolToJson();
				//	machine->cleanup();

				//	delete machine;
				//}

				sprintf(json, "%s/testResult/01_31_20/Type2LLTH/oneMach_J%d_B%d_C%d_HC%d_P%d_DepthCont", path, numJobs, bodyMax, coef, htCoef, prob);
				for (int i = 0; i < numProb; i++) {
					sprintf(problem, "%s/oneMach_J%d_B%d_C%d_HC%d_P%d_%d.txt", path, numJobs, bodyMax, coef, htCoef, prob, i);
					OneMachineDPProblem* machine = new OneMachineDPProblem(problem, 3600, 100000, DepthCont, ARB, true, true, json, nullptr, nullptr, nullptr);
					//machine->testRun();
					machine->solve();
					machine->printSolToJson();
					machine->cleanup();

					delete machine;
				}
			}
		}
	}

	cin.get();

	return 0;
}