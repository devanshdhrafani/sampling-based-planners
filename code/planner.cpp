/*=================================================================
 *
 * planner.c
 *
 *=================================================================*/

#include "planner.h"

double distance_angles(double* angles1, double* angles2, int numofDOFs)
{
    double dist = 0;
	for(int i=0; i<numofDOFs; i++)
	{
		dist = dist + (angles1[i]-angles2[i]) * (angles1[i]-angles2[i]);
	}
	return sqrt(dist);
}

int newConfig(double* q, double* q_near, double* q_new, int numofDOFs, double* map, int x_size, int y_size) 
{
    // move by EPSILON towards q from q_near and return q_new

    double dist = 0;
    int success = 0;
    int i,j;
    for (j = 0; j < numofDOFs; j++){
        if(dist < fabs(q_near[j] - q[j]))
            dist = fabs(q_near[j] - q[j]);
    }
    int numofsamples = (int)(dist/(PI/20));

    double* tmp_angles = (double*)malloc(numofDOFs*sizeof(double));
    double* saved_angles = NULL;

    for (i = 1; i < numofsamples; i++)
    {
    	for(j = 0; j<numofDOFs; j++)
    	{
    		tmp_angles[j] = q_near[j] + ((double)(i)/(numofsamples-1))*(q[j] - q_near[j]);
    	}
    	if(IsValidArmConfiguration(tmp_angles, numofDOFs, map, x_size, y_size) && 
    		distance_angles(tmp_angles, q_near, numofDOFs) < EPSILON)
    	{
    		memcpy(q_new, tmp_angles, numofDOFs*sizeof(double));
            success = 1;
    	}
    	else
    	{break;}
    }

    free(tmp_angles);
    return success;

}

int isAtGoal(double* angles, double* goal_angles, int numofDOFs)
{
    int reached = 0;
    double distance = 0;
    int i,j;
    for (j = 0; j < numofDOFs; j++){
        if(distance < fabs(angles[j] - goal_angles[j]))
            distance = fabs(angles[j] - goal_angles[j]);
    }
    int numofsamples = (int)(distance/(PI/20));
    if(numofsamples < 2){
        printf("Arm reached the goal\n");
        reached = 1;
    }
    return reached;
}

// Extend tree towards the sample node
int extendTree(Tree* tree, double* q, int numofDOFs, double* map, int x_size, int y_size)
{
    int advanced = 0;
    int q_near_id = tree->nearestNeighbour(q);
    double* q_near = tree->getNode(q_near_id);
    double* q_new = (double*)malloc(numofDOFs*sizeof(double));
    if(newConfig(q, q_near, q_new, numofDOFs, map, x_size, y_size)) 
    {
        int q_new_id = tree->addNode(q_new);
        tree->addEdge(q_new_id, q_near_id);
        advanced = 1;
    }
    return advanced;
}

// Generate Random Config
double* randomConfig(int numofDOFs, double* map, int x_size, int y_size) {
    double* sample_node_rad = new double[numofDOFs];
    // Check if config is valid
    int isvalid = 0;
    while(!isvalid) {
        for (int i = 0; i < numofDOFs; i++) {
            int random_deg = rand() % 360;
            sample_node_rad[i] = (double)random_deg / 180 * PI;
        }
        if(IsValidArmConfiguration(sample_node_rad, numofDOFs, map, x_size, y_size)) {
            isvalid = 1;
        }
    }
    return sample_node_rad;
}

static void RRTplanner(
			double* map,
			int x_size,
			int y_size,
			double* armstart_anglesV_rad,
			double* armgoal_anglesV_rad,
            int numofDOFs,
            double*** plan,
            int* planlength)
{

	// seed
	// srand( (unsigned)time( NULL ) );
	// srand(1);

	//no plan by default
	*plan = NULL;
	*planlength = 0;

	// Initialize tree structure with start node
	Tree tree(numofDOFs, armstart_anglesV_rad);

	// Number of samples
	int K = 10000;
	int k = 0;

	// Goal bias
	double goal_bias = 0.05;

	// target found flag
	bool target_found = 0;

	while(!target_found && k < K)
	{
		k++;

		double* q_rand = (double*)malloc(numofDOFs*sizeof(double));
		// Sample a random node with goal bias
		if((double)rand() / RAND_MAX < goal_bias)
		{
			// Sample goal node
			// cout << "Sampling goal node" << endl;
			q_rand = armgoal_anglesV_rad;
		}
		else
		{
			// Sample random node
			// cout << "Sampling random node" << endl;
			q_rand = randomConfig(numofDOFs, map, x_size, y_size);
		}
		
		// Extend the tree towards the sample node
		if(!extendTree(&tree, q_rand, numofDOFs, map, x_size, y_size))
		{
			continue;
		}

		int q_new_id = tree.getNewNodeID();
		double* q_new = tree.getNode(q_new_id);

		// Check if the new node is close to the goal
		if (isAtGoal(q_new, armgoal_anglesV_rad, numofDOFs))
		{
			target_found = 1;
		}
	}

	// If target is found, construct and return the plan
	if(target_found)
	{
		int q_new_id = tree.getNewNodeID();
		double* q_new = tree.getNode(q_new_id);
		vector<int> path;
		int next_id = q_new_id;
		while (next_id != 0) {
			path.insert(path.begin(), next_id);
			next_id = tree.getParentID(next_id);
		}
		path.insert(path.begin(), 0);
		*planlength = path.size();
		*plan = (double**) malloc(path.size()*sizeof(double*));
		for(int i=0; i<path.size(); i++)
		{
			(*plan)[i] = (double*) malloc(numofDOFs*sizeof(double));
			memcpy((*plan)[i], tree.getNode(path[i]), numofDOFs*sizeof(double));
		}
	}
	else
	{
		printf("Target not found\n");
	}	

}


static void planner(
			double* map,
			int x_size,
			int y_size,
			double* armstart_anglesV_rad,
			double* armgoal_anglesV_rad,
            int numofDOFs,
            double*** plan,
            int* planlength)
{

	RRTplanner(map, x_size, y_size, armstart_anglesV_rad, armgoal_anglesV_rad, numofDOFs, plan, planlength);

	//no plan by default
	// *plan = NULL;
	// *planlength = 0;
		
    // //for now just do straight interpolation between start and goal checking for the validity of samples

    // double distance = 0;
    // int i,j;
    // for (j = 0; j < numofDOFs; j++){
    //     if(distance < fabs(armstart_anglesV_rad[j] - armgoal_anglesV_rad[j]))
    //         distance = fabs(armstart_anglesV_rad[j] - armgoal_anglesV_rad[j]);
    // }
    // int numofsamples = (int)(distance/(PI/20));
    // if(numofsamples < 2){
    //     printf("The arm is already at the goal\n");
    //     return;
    // }
	// int countNumInvalid = 0;
    // *plan = (double**) malloc(numofsamples*sizeof(double*));
    // for (i = 0; i < numofsamples; i++){
    //     (*plan)[i] = (double*) malloc(numofDOFs*sizeof(double)); 
    //     for(j = 0; j < numofDOFs; j++){
    //         (*plan)[i][j] = armstart_anglesV_rad[j] + ((double)(i)/(numofsamples-1))*(armgoal_anglesV_rad[j] - armstart_anglesV_rad[j]);
    //     }
    //     if(!IsValidArmConfiguration((*plan)[i], numofDOFs, map, x_size, y_size)) {
	// 		++countNumInvalid;
    //     }
    // }
	// printf("Linear interpolation collided at %d instances across the path\n", countNumInvalid);
    // *planlength = numofsamples;
    
    return;
}


/** Your final solution will be graded by an grading script which will
 * send the default 6 arguments:
 *    map, numOfDOFs, commaSeparatedStartPos, commaSeparatedGoalPos, 
 *    whichPlanner, outputFilePath
 * An example run after compiling and getting the planner.out executable
 * >> ./planner.out map1.txt 5 1.57,0.78,1.57,0.78,1.57 0.392,2.35,3.14,2.82,4.71 0 output.txt
 * See the hw handout for full information.
 * If you modify this for testing (e.g. to try out different hyper-parameters),
 * make sure it can run with the original 6 commands.
 * Programs that do not will automatically get a 0.
 * */
int main(int argc, char** argv) {
	double* map;
	int x_size, y_size;

	tie(map, x_size, y_size) = loadMap(argv[1]);
	const int numOfDOFs = std::stoi(argv[2]);
	double* startPos = doubleArrayFromString(argv[3]);
	double* goalPos = doubleArrayFromString(argv[4]);
	int whichPlanner = std::stoi(argv[5]);
	string outputFile = argv[6];

	if(!IsValidArmConfiguration(startPos, numOfDOFs, map, x_size, y_size)||
			!IsValidArmConfiguration(goalPos, numOfDOFs, map, x_size, y_size)) {
		throw runtime_error("Invalid start or goal configuration!\n");
	}

	///////////////////////////////////////
	//// Feel free to modify anything below. Be careful modifying anything above.

	double** plan = NULL;
	int planlength = 0;

	switch(whichPlanner) {
		case RRT:
			RRTplanner(map, x_size, y_size, startPos, goalPos, numOfDOFs, &plan, &planlength);
			break;
		case RRTCONNECT:
			// RRTConnectplanner(map, x_size, y_size, startPos, goalPos, numOfDOFs, &plan, &planlength);
			break;
		case RRTSTAR:
			// RRTStarplanner(map, x_size, y_size, startPos, goalPos, numOfDOFs, &plan, &planlength);
			break;
		case PRM:
			// PRMplanner(map, x_size, y_size, startPos, goalPos, numOfDOFs, &plan, &planlength);
			break;
		default:
			throw runtime_error("Invalid planner number!\n");
	}

	//// Feel free to modify anything above.
	//// If you modify something below, please change it back afterwards as my 
	//// grading script will not work and you will recieve a 0.
	///////////////////////////////////////

    // Your solution's path should start with startPos and end with goalPos
    if (!equalDoubleArrays(plan[0], startPos, numOfDOFs) || 
    	!equalDoubleArrays(plan[planlength-1], goalPos, numOfDOFs)) {
		throw std::runtime_error("Start or goal position not matching");
	}

	/** Saves the solution to output file
	 * Do not modify the output log file output format as it is required for visualization
	 * and for grading.
	 */
	std::ofstream m_log_fstream;
	m_log_fstream.open(outputFile, std::ios::trunc); // Creates new or replaces existing file
	if (!m_log_fstream.is_open()) {
		throw std::runtime_error("Cannot open file");
	}
	m_log_fstream << argv[1] << endl; // Write out map name first
	/// Then write out all the joint angles in the plan sequentially
	for (int i = 0; i < planlength; ++i) {
		for (int k = 0; k < numOfDOFs; ++k) {
			m_log_fstream << plan[i][k] << ",";
		}
		m_log_fstream << endl;
	}
}
