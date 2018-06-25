// Author: Akash Patel (apatel435@gatech.edu)

// genPhiMatrixAsFile
// Purpose: Determine phi vectors for each input pose
//   This phi will be used for finding beta (weights of actual robot) via gradient descent
//
// Input: Ideal beta={mi, MXi, MYi, ...}, krang urdf model, perturbation value,
//   potentially unbalanced data points (q/poses) as a file,
// Output: Phi matrix as a file

// convergeToBeta
// Purpose: Converge to an optimal beta vector
//   This beta is the parameter vector of Krang
//
// Input: Perturbed beta={mi, MXi, MYi, ...}, phi matrix,
// Output: Converged beta vector as a file
//
//
// Overall Input: Poses in {heading, qBase, etc.} format
// Overall Output: Converged beta value
// Intermediary Input/Output Flow:
// Input Pose File -> Dart Poses -> Opt Dart Poses -> Phi Matrix
// Phi Matrix -> Converged Beta
//

////////////////////////////////////////////////////////////////////////////////
//           Use your search feature to find hardcoded inputs to this script
//           Search for "INPUT on below"
////////////////////////////////////////////////////////////////////////////////

// TODO: Need to combine methods together so I am not reading/writing files more
// than once and not creating multiple copies of the same data
// TODO: Perform C++ warning checks
// TODO: Check for memory leaks (valgrind)
// TODO: Optimize script for speed

#include <dart/dart.hpp>
#include <dart/utils/urdf/urdf.hpp>
#include <iostream>
#include <fstream>
#include <nlopt.hpp>

using namespace std;
using namespace dart::common;
using namespace dart::dynamics;
using namespace dart::simulation;
using namespace dart::math;

#define MAXBUFSIZE ((int) 1e6)


int genPhiMatrixAsFile() {

    // Put a hard stop on reading poses just in case
    // INPUT on below line (Hard stop to number of pose readings)
    //int controlPoseNums = 4700;
    int controlPoseNums = 10;
    // INPUT on below line (lines to skip so an even distribution of samples can
    // be taken) Dependent on file lines
    //int linesToSkip = 65000/controlPoseNums;

    int cols = 0, rows = 0;
    double buff[MAXBUFSIZE];


    // Read numbers (the pose params)
    ifstream infile;
    // infile.open("../defaultInit.txt");
    // INPUT on below line (input pose file)
    infile.open("/home/krang/Downloads/trainingData/dataQ.txt");
    cout << "Reading input q ...\n";
//    int lineNumber = 0;

    while(! infile.eof() && rows <= controlPoseNums) {
            string line;

            getline(infile, line);

            int temp_cols = 0;
            stringstream stream(line);
            while(! stream.eof())
                stream >> buff[cols*rows+temp_cols++];

            if (temp_cols == 0)
                continue;

            if (cols == 0)
                cols = temp_cols;

            rows++;
    }

    infile.close();
    infile.clear();
    cout << "|-> Done\n";
    rows--;

    // Populate matrix with numbers.
    // Eigen matrix is transpose of read file
    // every column is a pose, the rows are the pose params
    // Heading, qBase, x, y, z, qLWheel, qRWheel, qWaist, qTorso, qKinect,
    // qLArm0, ..., qLArm6, qRArm0, ..., qRArm6

    int numInputPoses = rows;
    int numParams = cols;

    Eigen::MatrixXd allInitq (rows, cols);
    for (int i = 0; i < rows; i++)
        for (int j = 0; j < cols; j++)
	  // allInitPoseParamsFromFile(i,j) = buff[cols*i+j];
	       allInitq(i,j) = buff[cols*i+j];  




    cols = 0, rows = 0;
    //double buff[MAXBUFSIZE];

    // Read numbers (the pose params)
    //ifstream infile;
    // infile.open("../defaultInit.txt");
    // INPUT on below line (input pose file)
    infile.open("/home/krang/Downloads/trainingData/dataQdot.txt");
    cout << "Reading input qdot ...\n";
//    int lineNumber = 0;

    while(! infile.eof() && rows <= controlPoseNums) {
            string line;

            getline(infile, line);

            int temp_cols = 0;
            stringstream stream(line);
            while(! stream.eof())
                stream >> buff[cols*rows+temp_cols++];

            if (temp_cols == 0)
                continue;

            if (cols == 0)
                cols = temp_cols;

            rows++;
    }

    infile.close();
    infile.clear();
    cout << "|-> Done\n";
    rows--;

    // Populate matrix with numbers.
    // Eigen matrix is transpose of read file
    // every column is a pose, the rows are the pose params
    // Heading, qBase, x, y, z, qLWheel, qRWheel, qWaist, qTorso, qKinect,
    // qLArm0, ..., qLArm6, qRArm0, ..., qRArm6

    numInputPoses = rows;
    numParams = cols;

    Eigen::MatrixXd allInitqdot (rows, cols);
    for (int i = 0; i < rows; i++)
        for (int j = 0; j < cols; j++)
	  // allInitPoseParamsFromFile(i,j) = buff[cols*i+j];
	       allInitqdot(i,j) = buff[cols*i+j]; 
	


    cols = 0, rows = 0;

    infile.open("/home/krang/Downloads/trainingData/dataQdotdot.txt");
    cout << "Reading input qdotdot ...\n";
//    int lineNumber = 0;

    while(! infile.eof() && rows <= controlPoseNums) {
            string line;

            getline(infile, line);

            int temp_cols = 0;
            stringstream stream(line);
            while(! stream.eof())
                stream >> buff[cols*rows+temp_cols++];

            if (temp_cols == 0)
                continue;

            if (cols == 0)
                cols = temp_cols;

            rows++;
    }

    infile.close();
    infile.clear();
    cout << "|-> Done\n";
    rows--;

    // Populate matrix with numbers.
    // Eigen matrix is transpose of read file
    // every column is a pose, the rows are the pose params
    // Heading, qBase, x, y, z, qLWheel, qRWheel, qWaist, qTorso, qKinect,
    // qLArm0, ..., qLArm6, qRArm0, ..., qRArm6

    numInputPoses = rows;
    numParams = cols;

    Eigen::MatrixXd allInitqdotdot (rows, cols);
    for (int i = 0; i < rows; i++)
        for (int j = 0; j < cols; j++)
      // allInitPoseParamsFromFile(i,j) = buff[cols*i+j];
           allInitqdotdot(i,j) = buff[cols*i+j]; 
       


    cols = 0, rows = 0;

    infile.open("/home/krang/Downloads/trainingData/dataTorque.txt");
    cout << "Reading input qdotdot ...\n";
//    int lineNumber = 0;

    while(! infile.eof() && rows <= controlPoseNums) {
 //       if (lineNumber == linesToSkip) {
            
            string line;

            getline(infile, line);

            int temp_cols = 0;
            stringstream stream(line);
            while(! stream.eof())
                stream >> buff[cols*rows+temp_cols++];

            if (temp_cols == 0)
                continue;

            if (cols == 0)
                cols = temp_cols;

            rows++;
    }

    infile.close();
    infile.clear();
    cout << "|-> Done\n";
    rows--;

    // Populate matrix with numbers.
    // Eigen matrix is transpose of read file
    // every column is a pose, the rows are the pose params
    // Heading, qBase, x, y, z, qLWheel, qRWheel, qWaist, qTorso, qKinect,
    // qLArm0, ..., qLArm6, qRArm0, ..., qRArm6

    numInputPoses = rows;
    numParams = cols;

    cout<<numInputPoses<<"\n";

    Eigen::MatrixXd allInitTorque (rows, cols);
    for (int i = 0; i < rows; i++)
        for (int j = 0; j < cols; j++)
      // allInitPoseParamsFromFile(i,j) = buff[cols*i+j];
           allInitTorque(i,j) = buff[cols*i+j]; 



    int cols1 = 0, rows1 = 0;

    infile.open("/home/krang/Downloads/trainingData/dataM.txt");
    cout << "Reading input Mass Matrix ...\n";
//    int lineNumber = 0;

    while(! infile.eof() && rows1 < controlPoseNums*7) {
 //       if (lineNumber == linesToSkip) {
            
            string line;

            getline(infile, line);

            int temp_cols = 0;
            stringstream stream(line);
            while(! stream.eof())
                stream >> buff[cols1*rows1+temp_cols++];

            if (temp_cols == 0)
                continue;

            if (cols1 == 0)
                cols1 = temp_cols;

            rows1++;
    }

    cout<<rows1<<"\n";
    cout<<cols1<<"\n";

    infile.close();
    infile.clear();
    cout << "|-> Done\n";
    rows--;





    // Populate matrix with numbers.
    // Eigen matrix is transpose of read file
    // every column is a pose, the rows are the pose params
    // Heading, qBase, x, y, z, qLWheel, qRWheel, qWaist, qTorso, qKinect,
    // qLArm0, ..., qLArm6, qRArm0, ..., qRArm6

    //numInputPoses1 = rows1;
    //numParams1 = cols1;

    Eigen::MatrixXd allInitM(rows1, cols1);
    for (int i = 0; i < rows1; i++)
        for (int j = 0; j < cols1; j++)
      // allInitPoseParamsFromFile(i,j) = buff[cols*i+j];
           allInitM(i,j) = buff[cols*i+j]; 



    cols = 0, rows = 0;

    infile.open("/home/krang/Downloads/trainingData/dataCg.txt");
    cout << "Reading input Cg ...\n";
//    int lineNumber = 0;

    while(! infile.eof() && rows <= controlPoseNums) {
 //       if (lineNumber == linesToSkip) {
            
            string line;

            getline(infile, line);

            int temp_cols = 0;
            stringstream stream(line);
            while(! stream.eof())
                stream >> buff[cols*rows+temp_cols++];

            if (temp_cols == 0)
                continue;

            if (cols == 0)
                cols = temp_cols;

            rows++;
    }

    infile.close();
    infile.clear();
    cout << "|-> Done\n";
    rows--;

    // Populate matrix with numbers.
    // Eigen matrix is transpose of read file
    // every column is a pose, the rows are the pose params
    // Heading, qBase, x, y, z, qLWheel, qRWheel, qWaist, qTorso, qKinect,
    // qLArm0, ..., qLArm6, qRArm0, ..., qRArm6

    numInputPoses = rows;
    numParams = cols;

    cout<<numInputPoses<<"\n";

    Eigen::MatrixXd allInitCg (rows, cols);
    for (int i = 0; i < rows; i++)
        for (int j = 0; j < cols; j++)
      // allInitPoseParamsFromFile(i,j) = buff[cols*i+j];
           allInitCg(i,j) = buff[cols*i+j]; 







    //  Eigen::MatrixXd allInitPoseParams(cols, rows);
    //  allInitPoseParams = allInitPoseParamsFromFile.transpose();

    // Perturbation Value
    // INPUT on below line (perturbation value for finding phi)
    double perturbedValue = std::pow(1, -17);

    // Instantiate "ideal" robot and n other robots
    cout << "Creating ideal beta vector ...\n";
    dart::utils::DartLoader loader;
    // INPUT on below line (absolute path of the Krang URDF file)
    dart::dynamics::SkeletonPtr idealRobot = loader.parseSkeleton("/home/krang/dart/09-URDF/7DOFArm/singlearm.urdf");
    idealRobot->setGravity(Eigen::Vector3d (0.0, -9.81, 0.0));

    // Create ideal beta
    // Wait do we get beta from the ideal robot or give it to the ideal robot?
    // Beta Definition/Format
    // mi, mxi, myi, mzi for each body

    int bodyParams = 10;
    int numBodies = idealRobot->getNumBodyNodes();
    dart::dynamics::BodyNodePtr bodyi;
    string namei;
    double mi;
    double xMi;
    double yMi;
    double zMi;
    double ixx;
    double ixy;
    double ixz;
    double iyy;
    double iyz;
    double izz;










    int numPertRobots = (numBodies-1)*bodyParams;
    Eigen::MatrixXd betaParams(1, numPertRobots);

    for (int i = 1; i < numBodies; i++) {
        bodyi = idealRobot->getBodyNode(i);

        namei = bodyi->getName();
        mi = bodyi->getMass();
        xMi = mi * bodyi->getLocalCOM()(0);
        yMi = mi * bodyi->getLocalCOM()(1);
        zMi = mi * bodyi->getLocalCOM()(2);
        bodyi->getMomentOfInertia (ixx, iyy,izz,ixy,ixz,iyz);


        betaParams(0, (i-1) * bodyParams + 0) = mi;
        betaParams(0, (i-1) * bodyParams + 1) = xMi;
        betaParams(0, (i-1) * bodyParams + 2) = yMi;
        betaParams(0, (i-1) * bodyParams + 3) = zMi;
        betaParams(0, (i-1)*  bodyParams + 4) = ixx;
        betaParams(0, (i-1)*  bodyParams +5)  = ixy;
        betaParams(0, (i-1)* bodyParams +6)   =   ixz;
        betaParams(0, (i-1)* bodyParams +7)   =  iyy;
        betaParams(0, (i-1)*bodyParams +8)    =  iyz;
        betaParams(0, (i-1)*bodyParams +9)    =    izz; 


    }
    cout << "|-> Done\n";
    cout << "Creating robot array ...\n";


    // TODO: Need to create an array of pertRobots in a fast time
    // Create array of robots out of pose loop for fast times
    // then change appropriate values (betaParams(i)) for each robot when
    // going through all the robots 


        // TODO: Need to create an array of pertRobots in a fast time
    // Create array of robots out of pose loop for fast times
    // then change appropriate values (betaParams(i)) for each robot when
    // going through all the robots


    dart::dynamics::SkeletonPtr pertRobotArray[sizeof(SkeletonPtr) * numPertRobots];



    for (int i = 0; i < numPertRobots; i++) {

        // TODO: Segfaulting right here
        // Trying to create an array of idealRobots by calling parseSkeleton
        // only once since it is time expenseive
        //memcpy(pertRobotArray[i], idealRobot, sizeof(SkeletonPtr));

        pertRobotArray[i] = loader.parseSkeleton("/home/krang/dart/09-URDF/7DOFArm/singlearm.urdf");

    }

    cout << "|-> Done\n";
    // Create file to print the actual balanced poses used in DART

    // Find phiMatrix


    cout << "Calculating Phi Matrix ...\n";



/*
    int an=9;

    int ab=10;

    cout<<an/ab<<"\n";

    cout<<ab/an<<"\n";

    cout<<an%ab<<"\n";*/

        


/*    int dataNum = 1000;

    idealRobot->setPositions(allInitq.row(dataNum));
    idealRobot->setVelocities(allInitqdot.row(dataNum));
    idealRobot->updateVelocityChange();

    
    Eigen::MatrixXd M1 = idealRobot->getMassMatrix(); // n x n
    Eigen::VectorXd Cg1 = idealRobot->getCoriolisAndGravityForces(); // n x 1

    
    Eigen::MatrixXd M = allInitM.block(7*dataNum,0,7,7);
    Eigen::VectorXd Cg = allInitCg.row(dataNum).transpose();    

    cout << "q from file: " << allInitq.row(dataNum) << endl;
    cout << "q from dart: " << idealRobot->getPositions().transpose() << endl << endl;

    cout << "dq from file: " << allInitqdot.row(dataNum) << endl;
    cout << "dq from dart: " << idealRobot->getVelocities().transpose() << endl << endl;

    cout << "ddq from file: " << allInitqdotdot.row(dataNum) << endl << endl;
    
    cout << "M from file: \n" << M.transpose() <<"\n";
    cout << "M from dart: \n" << M1 <<"\n\n";

    cout << "Cg from file: " << Cg.transpose() << "\n";
    cout << "Cg from dart: " << Cg1.transpose() << "\n\n";

    Eigen::VectorXd ddq = allInitqdotdot.row(dataNum).transpose();
    Eigen::VectorXd RHS_ideal= (M*ddq) + Cg;
    Eigen::VectorXd RHS_dart= (M1*ddq) + Cg1;

    cout << "RHS based on file: " << RHS_ideal.transpose() << "\n";
    cout << "RHS based on dart: " << RHS_dart.transpose() << "\n";
    cout << "Torques from file: " << allInitTorque.row(dataNum) << endl;
*/








 

/*    for (int i = 0; i < numInputPoses; i++) {

        idealRobot->setPositions(allInitq.row(i));

        idealRobot->setVelocities(allInitqdot.row(i));

        //idealRobot->setAccelerations(allInitqdotdot.row(3));



        Eigen::MatrixXd M = idealRobot->getMassMatrix(); // n x n

        Eigen::VectorXd Cg = idealRobot->getCoriolisAndGravityForces(); // n x 1

       // Eigen::VectorXd ddq = idealRobot->getAccelerations(); // n x 1

        Eigen::VectorXd ddq = allInitqdotdot.row(i);


        //cout<<ddq<<"\n";

        Eigen::VectorXd RHS_ideal= (M*ddq) + Cg;


        //cout<<"RHS_ideal\n"<<RHS_ideal<<"\n";

        //cout<<"Torque\n"<<allInitTorque.row(3);


        //Eigen::VectorXd q1 = idealRobot->getPositions(); // n x 1

        




    }
*/

            //cout << q1<<"\n";




int pertRobotNum = 0 ; 

//Eigen::VectorXd bodyInertia1(1, 6);
//bodyInertia1 << betaParams(0, pertRobotNum) + perturbedValue , betaParams(0, pertRobotNum + 1) , betaParams(0, pertRobotNum + 2) , betaParams(0, pertRobotNum + 3) , betaParams(0, pertRobotNum + 4), betaParams(0, pertRobotNum + 5);


//cout<<bodyInertia1<<"/n";



//Eigen::VectorXd bodyInertia1(betaParams(0, pertRobotNum) + perturbedValue , betaParams(0, pertRobotNum + 1) , betaParams(0, pertRobotNum + 2) , betaParams(0, pertRobotNum + 3) , betaParams(0, pertRobotNum + 4), betaParams(0, pertRobotNum + 5) );
                
//pertRobotArray[0]->getBodyNode(0)->setMomentOfInertia(betaParams(0, pertRobotNum) + perturbedValue , betaParams(0, pertRobotNum + 1) , betaParams(0, pertRobotNum + 2) , betaParams(0, pertRobotNum + 3) , betaParams(0, pertRobotNum + 4), betaParams(0, pertRobotNum + 5));

 /*   double ixx1;
    double ixy1;
    double ixz1;
    double iyy1;
    double iyz1;
    double izz1;

    pertRobotArray[0]->getBodyNode(pertRobotNum / bodyParams)->->getMomentOfInertia(ixx1, iyy1,izz1,ixy1,ixz1,iyz1);


    cout<<ixx1<<"\n"<<ixy1<<"\n"<<ixz1<<"\n"<<iyy1<<"\n"<<iyz1<<"\n"<<izz1<<"\n";
*/


/*        int i =10;

            pertRobotArray[pertRobotNum]->setGravity(Eigen::Vector3d (0.0, -9.81, 0.0));


            pertRobotArray[pertRobotNum]->setPositions(allInitq.row(i));

            pertRobotArray[pertRobotNum]->setVelocities(allInitqdot.row(i));

            Eigen::VectorXd ddq = allInitqdotdot.row(i);



            Eigen::MatrixXd M_pert = pertRobotArray[pertRobotNum]->getMassMatrix(); // n x n

            Eigen::VectorXd Cg_pert = pertRobotArray[pertRobotNum]->getCoriolisAndGravityForces(); // n x 1

            Eigen::VectorXd RHS_pert= M_pert*ddq + Cg_pert;


            cout<<RHS_pert;*/




    ofstream myfile;

    myfile.open ("dataTorque_RHS.txt");
    myfile<< "dataTorque" << endl;

    ofstream myfile1;

    myfile1.open ("data_phi.txt");
    myfile1<< "data_phi" << endl;





    Eigen::MatrixXd phi_Mat(numInputPoses*(numBodies-1), numPertRobots);


    Eigen::MatrixXd phiMatrix(numBodies-1, numPertRobots);


    Eigen::MatrixXd phi(numBodies-1,1);


    for (int i = 0; i < numInputPoses; i++) {


        idealRobot->setPositions(allInitq.row(i));

        idealRobot->setVelocities(allInitqdot.row(i));

        Eigen::VectorXd ddq = allInitqdotdot.row(i);
        
        //idealRobot->setAccelerations(allInitqdotdot.row(i));

        

        Eigen::MatrixXd M = idealRobot->getMassMatrix(); // n x n

        Eigen::VectorXd Cg = idealRobot->getCoriolisAndGravityForces(); // n x 1



        //Eigen::VectorXd ddq = idealRobot->getAccelerations(); // n x 1

        


        Eigen::VectorXd RHS_ideal= M*ddq + Cg;



        myfile << RHS_ideal.transpose() << endl;


        for (int pertRobotNum = 0; pertRobotNum < numPertRobots; pertRobotNum++) {

            if (pertRobotNum % bodyParams == 0) {
                
                pertRobotArray[pertRobotNum]->getBodyNode(pertRobotNum / bodyParams)->setMass(betaParams(0, pertRobotNum) + perturbedValue);
            }
            else if (pertRobotNum % bodyParams == 1) {
                Eigen::Vector3d bodyCOM(betaParams(0, pertRobotNum) + perturbedValue, betaParams(0, pertRobotNum + 1), betaParams(0, pertRobotNum + 2));
                
                pertRobotArray[pertRobotNum]->getBodyNode(pertRobotNum / bodyParams)->setLocalCOM(bodyCOM);

            } else if (pertRobotNum % bodyParams == 2) {
                Eigen::Vector3d bodyCOM(betaParams(0, pertRobotNum - 1), betaParams(0, pertRobotNum) + perturbedValue, betaParams(0, pertRobotNum + 1));
                
                pertRobotArray[pertRobotNum]->getBodyNode(pertRobotNum / bodyParams)->setLocalCOM(bodyCOM);

            } else if (pertRobotNum % bodyParams == 3) {
                Eigen::Vector3d bodyCOM(betaParams(0, pertRobotNum - 2), betaParams(0, pertRobotNum - 1), betaParams(0, pertRobotNum) + perturbedValue);
                
                pertRobotArray[pertRobotNum]->getBodyNode(pertRobotNum / bodyParams)->setLocalCOM(bodyCOM);

            }

            else if (pertRobotNum % bodyParams == 4) {
                //Eigen::Vector3d bodyInertia();
                
                pertRobotArray[pertRobotNum]->getBodyNode(pertRobotNum / bodyParams)->setMomentOfInertia(betaParams(0, pertRobotNum) + perturbedValue, betaParams(0, pertRobotNum + 1), betaParams(0, pertRobotNum + 2), betaParams(0, pertRobotNum + 3),betaParams(0, pertRobotNum + 4),betaParams(0, pertRobotNum + 5));

            } else if (pertRobotNum % bodyParams == 5) {
                //Eigen::Vector3d bodyInertia();
                
                pertRobotArray[pertRobotNum]->getBodyNode(pertRobotNum / bodyParams)->setMomentOfInertia(betaParams(0, pertRobotNum-1) , betaParams(0, pertRobotNum)+ perturbedValue, betaParams(0, pertRobotNum + 1), betaParams(0, pertRobotNum + 2),betaParams(0, pertRobotNum + 3),betaParams(0, pertRobotNum + 4));

            } else if (pertRobotNum % bodyParams == 6){
                //Eigen::Vector3d bodyInertia();
                
                pertRobotArray[pertRobotNum]->getBodyNode(pertRobotNum / bodyParams)->setMomentOfInertia(betaParams(0, pertRobotNum-2) , betaParams(0, pertRobotNum -1), betaParams(0, pertRobotNum )+ perturbedValue, betaParams(0, pertRobotNum + 1),betaParams(0, pertRobotNum + 2),betaParams(0, pertRobotNum + 3));
            }

            else if (pertRobotNum % bodyParams == 7) {
                //Eigen::Vector3d bodyInertia();
                
                pertRobotArray[pertRobotNum]->getBodyNode(pertRobotNum / bodyParams)->setMomentOfInertia(betaParams(0, pertRobotNum-3) , betaParams(0, pertRobotNum - 2), betaParams(0, pertRobotNum -1), betaParams(0, pertRobotNum)+ perturbedValue,betaParams(0, pertRobotNum + 1),betaParams(0, pertRobotNum + 2));

            } else if (pertRobotNum % bodyParams == 8) {
               // Eigen::Vector3d bodyInertia();
                
                pertRobotArray[pertRobotNum]->getBodyNode(pertRobotNum / bodyParams)->setMomentOfInertia(betaParams(0, pertRobotNum-4) , betaParams(0, pertRobotNum -3), betaParams(0, pertRobotNum -2), betaParams(0, pertRobotNum -1),betaParams(0, pertRobotNum )+ perturbedValue,betaParams(0, pertRobotNum + 1));

            } else if (pertRobotNum % bodyParams == 9){
                //Eigen::Vector3d bodyInertia();
                
                pertRobotArray[pertRobotNum]->getBodyNode(pertRobotNum / bodyParams)->setMomentOfInertia(betaParams(0, pertRobotNum-5) , betaParams(0, pertRobotNum -4), betaParams(0, pertRobotNum -3), betaParams(0, pertRobotNum -2),betaParams(0, pertRobotNum -1),betaParams(0, pertRobotNum)+ perturbedValue);

            }

            // Set perturbed robot position to pose

            pertRobotArray[pertRobotNum]->setGravity(Eigen::Vector3d (0.0, -9.81, 0.0));


            pertRobotArray[pertRobotNum]->setPositions(allInitq.row(i));

            pertRobotArray[pertRobotNum]->setVelocities(allInitqdot.row(i));

            pertRobotArray[pertRobotNum]->setAccelerations(allInitqdotdot.row(i));

            Eigen::MatrixXd M_pert = pertRobotArray[pertRobotNum]->getMassMatrix(); // n x n

            Eigen::VectorXd Cg_pert = pertRobotArray[pertRobotNum]->getCoriolisAndGravityForces(); // n x 1

            Eigen::VectorXd ddq_pert = pertRobotArray[pertRobotNum]->getAccelerations(); // n x 1

            Eigen::VectorXd RHS_pert= M_pert*ddq + Cg_pert;


            // Calculate phi for beta i and pose
            phi = (RHS_pert - RHS_ideal)/(perturbedValue);

            // Add phi to phiMatrix and then print it looks cleaner
            phiMatrix.col(pertRobotNum) = phi;


            //cout<<"phi_matrix calcualted at " <<pertRobotNum << " & data point "<<i <<endl <<phiMatrix<<endl<<endl;

    }

    phi_Mat.block(7*i,0,7,70)=phiMatrix;

    myfile1 << phi_Mat << endl;


     }


myfile.close();

myfile1.close();

}






int main() {

    genPhiMatrixAsFile();
    
}


