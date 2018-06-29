// Author: Akash Patel (apatel435@gatech.edu)
// Modified by: Zubair

// genPhiMatrixAsFile
// Purpose: Determine phi vectors for each input data point
//   This phi will be used for finding beta (weights of actual robot)
//
// Input: Ideal beta={mi, MXi, MYi, ...}, krang urdf model, perturbation value,
//   data points (q/poses) as a file,
// Output: Phi matrix as a file

// Overall Input: Data Points q, qdot, qddot (1 x 7) format
// Overall Output: Phi Matrix
// Intermediary Input/Output Flow:
// Input Data Points -> Phi Matrix

// Includes
#include <dart/dart.hpp>
#include <dart/utils/urdf/urdf.hpp>
#include <iostream>
#include <fstream>

// Namespaces
using namespace std;
using namespace dart::common;
using namespace dart::dynamics;
using namespace dart::simulation;
using namespace dart::math;
using namespace dart::utils;

// Defines
#define MAXBUFSIZE ((int) 1e6)

// Function Prototypes
// // Generate Phi Matrix
Eigen::MatrixXd genPhiMatrix(Eigen::MatrixXd inputQ, Eigen::MatrixXd inputQdot, Eigen::MatrixXd inputQdotdot, Eigen::MatrixXd inputTorque, Eigen::MatrixXd inputMassMatrix, Eigen::MatrixXd inputCg, string fullRobotPath, double perturbedValue);

// // Read file as matrix
Eigen::MatrixXd readInputFileAsMatrix(string inputFilename);
Eigen::MatrixXd readInputFileAsMatrix(string inputFilename, int stopCount);

// TODO: Need to merge
int genPhiMatrixAsFile();

// Main Method
int main() {
    // INPUT on below line (perturbation value for finding phi)
    double perturbedValue = std::pow(10, -10);

    // INPUT on below line (absolute robot path)
    string fullRobotPath = "/home/krang/dart/09-URDF/7DOFArm/singlearm.urdf";

    // INPUT on below line (input data points filenames)
    string inputQFilename = "/home/krang/Downloads/trainingData/dataQ.txt";
    string inputQdotFilename = "/home/krang/Downloads/trainingData/dataQdot.txt";
    string inputQdotdotFilename = "/home/krang/Downloads/trainingData/dataQdotdot.txt";
    string inputTorqueFilename = "/home/krang/Downloads/trainingData/dataTorque.txt";
    string inputMassMatrixFilename = "/home/krang/Downloads/trainingData/dataM.txt";
    string inputCgFilename = "/home/krang/Downloads/trainingData/dataCg.txt";

    // INPUT on below line (data point limit)
    int stopCount = 10;

    cout << "Reading input q ...\n";
    Eigen::MatrixXd inputQ = readInputFileAsMatrix(inputQFilename, stopCount);
    cout << "|-> Done\n";

    cout << "Reading input qdot ...\n";
    Eigen::MatrixXd inputQdot = readInputFileAsMatrix(inputQdotFilename, stopCount);
    cout << "|-> Done\n";

    cout << "Reading input qdotdot ...\n";
    Eigen::MatrixXd inputQdotdot = readInputFileAsMatrix(inputQdotdotFilename, stopCount);
    cout << "|-> Done\n";

    cout << "Reading input torque ...\n";
    Eigen::MatrixXd inputTorque = readInputFileAsMatrix(inputTorqueFilename, stopCount);
    cout << "|-> Done\n";

    cout << "Reading input Mass Matrix ...\n";
    Eigen::MatrixXd inputMassMatrix = readInputFileAsMatrix(inputMassMatrixFilename, stopCount * 7);
    cout << "|-> Done\n";

    cout << "Reading input Cg ...\n";
    Eigen::MatrixXd inputCg = readInputFileAsMatrix(inputCgFilename, stopCount);
    cout << "|-> Done\n";

    Eigen::MatrixXd phiMatrix = genPhiMatrix(inputQ, inputQdot, inputQdotdot, inputTorque, inputMassMatrix, inputCg, fullRobotPath, perturbedValue);

}

// // Generate Phi Matrix
Eigen::MatrixXd genPhiMatrix(Eigen::MatrixXd allInitq, Eigen::MatrixXd allInitqdot, Eigen::MatrixXd allInitqdotdot, Eigen::MatrixXd allInitTorque, Eigen::MatrixXd allInitM, Eigen::MatrixXd allInitCg, string fullRobotPath, double perturbedValue) {
    int numDataPoints = allInitq.rows();

    // Instantiate "ideal" robot and n other robots
    DartLoader loader;
    SkeletonPtr idealRobot = loader.parseSkeleton(fullRobotPath);
    // Set gravity for the robot
    idealRobot->setGravity(Eigen::Vector3d (0.0, -9.81, 0.0));

    cout << "Creating ideal beta vector, robot array, and perturbing robots ...\n";

    // Create ideal beta
    // Beta Definition/Format
    // mi, mxi, myi, mzi, ixx, ixy, ixz, iyy, iyz, izz for each body

    int bodyParams = 10;
    int numBodies = idealRobot->getNumBodyNodes();
    BodyNodePtr bodyi;
    string namei;
    double mi;
    double mxi;
    double myi;
    double mzi;
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
        mxi = mi * bodyi->getLocalCOM()(0);
        myi = mi * bodyi->getLocalCOM()(1);
        mzi = mi * bodyi->getLocalCOM()(2);
        bodyi->getMomentOfInertia(ixx, iyy, izz, ixy, ixz, iyz);

        betaParams(0, (i-1) * bodyParams + 0) = mi;
        betaParams(0, (i-1) * bodyParams + 1) = mxi;
        betaParams(0, (i-1) * bodyParams + 2) = myi;
        betaParams(0, (i-1) * bodyParams + 3) = mzi;
        betaParams(0, (i-1) * bodyParams + 4) = ixx;
        betaParams(0, (i-1) * bodyParams + 5) = ixy;
        betaParams(0, (i-1) * bodyParams + 6) = ixz;
        betaParams(0, (i-1) * bodyParams + 7) = iyy;
        betaParams(0, (i-1) * bodyParams + 8) = iyz;
        betaParams(0, (i-1) * bodyParams + 9) = izz;

    }

    // TODO: Need to create an array of pertRobots in a fast time
    // Create array of robots out of phi loop for fast times
    // then change appropriate values (betaParams(i)) for each robot when
    // going through all the robots

    SkeletonPtr pertRobotArray[sizeof(SkeletonPtr) * numPertRobots];

    for (int i = 0; i < numPertRobots; i++) {

        // TODO: Segfaulting right here
        // Trying to create an array of idealRobots by calling parseSkeleton
        // only once since it is time expenseive
        //memcpy(pertRobotArray[i], idealRobot, sizeof(SkeletonPtr));

        pertRobotArray[i] = loader.parseSkeleton(fullRobotPath);

    }

    for (int pertRobotNum = 0; pertRobotNum < numPertRobots; pertRobotNum++) {

        if (pertRobotNum % bodyParams == 0) {

            pertRobotArray[pertRobotNum]->getBodyNode(pertRobotNum / bodyParams)->setMass(betaParams(0, pertRobotNum) + perturbedValue);
        } else if (pertRobotNum % bodyParams == 1) {
            Eigen::Vector3d bodyCOM(betaParams(0, pertRobotNum) + perturbedValue, betaParams(0, pertRobotNum + 1), betaParams(0, pertRobotNum + 2));

            pertRobotArray[pertRobotNum]->getBodyNode(pertRobotNum / bodyParams)->setLocalCOM(bodyCOM);

        } else if (pertRobotNum % bodyParams == 2) {
            Eigen::Vector3d bodyCOM(betaParams(0, pertRobotNum - 1), betaParams(0, pertRobotNum) + perturbedValue, betaParams(0, pertRobotNum + 1));

            pertRobotArray[pertRobotNum]->getBodyNode(pertRobotNum / bodyParams)->setLocalCOM(bodyCOM);

        } else if (pertRobotNum % bodyParams == 3) {
            Eigen::Vector3d bodyCOM(betaParams(0, pertRobotNum - 2), betaParams(0, pertRobotNum - 1), betaParams(0, pertRobotNum) + perturbedValue);

            pertRobotArray[pertRobotNum]->getBodyNode(pertRobotNum / bodyParams)->setLocalCOM(bodyCOM);

        } else if (pertRobotNum % bodyParams == 4) {
            //Eigen::Vector3d bodyInertia();

            pertRobotArray[pertRobotNum]->getBodyNode(pertRobotNum / bodyParams)->setMomentOfInertia(betaParams(0, pertRobotNum) + perturbedValue, betaParams(0, pertRobotNum + 1), betaParams(0, pertRobotNum + 2), betaParams(0, pertRobotNum + 3),betaParams(0, pertRobotNum + 4),betaParams(0, pertRobotNum + 5));

        } else if (pertRobotNum % bodyParams == 5) {
            //Eigen::Vector3d bodyInertia();

            pertRobotArray[pertRobotNum]->getBodyNode(pertRobotNum / bodyParams)->setMomentOfInertia(betaParams(0, pertRobotNum-1) , betaParams(0, pertRobotNum)+ perturbedValue, betaParams(0, pertRobotNum + 1), betaParams(0, pertRobotNum + 2),betaParams(0, pertRobotNum + 3),betaParams(0, pertRobotNum + 4));

        } else if (pertRobotNum % bodyParams == 6) {
            //Eigen::Vector3d bodyInertia();

            pertRobotArray[pertRobotNum]->getBodyNode(pertRobotNum / bodyParams)->setMomentOfInertia(betaParams(0, pertRobotNum-2) , betaParams(0, pertRobotNum -1), betaParams(0, pertRobotNum )+ perturbedValue, betaParams(0, pertRobotNum + 1),betaParams(0, pertRobotNum + 2),betaParams(0, pertRobotNum + 3));

        } else if (pertRobotNum % bodyParams == 7) {
            //Eigen::Vector3d bodyInertia();

            pertRobotArray[pertRobotNum]->getBodyNode(pertRobotNum / bodyParams)->setMomentOfInertia(betaParams(0, pertRobotNum-3) , betaParams(0, pertRobotNum - 2), betaParams(0, pertRobotNum -1), betaParams(0, pertRobotNum)+ perturbedValue,betaParams(0, pertRobotNum + 1),betaParams(0, pertRobotNum + 2));

        } else if (pertRobotNum % bodyParams == 8) {
            // Eigen::Vector3d bodyInertia();

            pertRobotArray[pertRobotNum]->getBodyNode(pertRobotNum / bodyParams)->setMomentOfInertia(betaParams(0, pertRobotNum-4) , betaParams(0, pertRobotNum -3), betaParams(0, pertRobotNum -2), betaParams(0, pertRobotNum -1),betaParams(0, pertRobotNum )+ perturbedValue,betaParams(0, pertRobotNum + 1));

        } else if (pertRobotNum % bodyParams == 9){
            //Eigen::Vector3d bodyInertia();

            pertRobotArray[pertRobotNum]->getBodyNode(pertRobotNum / bodyParams)->setMomentOfInertia(betaParams(0, pertRobotNum-5) , betaParams(0, pertRobotNum -4), betaParams(0, pertRobotNum -3), betaParams(0, pertRobotNum -2),betaParams(0, pertRobotNum -1),betaParams(0, pertRobotNum)+ perturbedValue);

        }

        pertRobotArray[pertRobotNum]->setGravity(Eigen::Vector3d (0.0, -9.81, 0.0));

    }

    cout << "|-> Done\n";

    // Find phiMatrix
    cout << "Generating Phi Matrix ...\n";

    /*
    int dataNum = 1000;

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

    /*
    for (int i = 0; i < numInputPoses; i++) {

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

    /*
    double ixx1;
    double ixy1;
    double ixz1;
    double iyy1;
    double iyz1;
    double izz1;

    pertRobotArray[0]->getBodyNode(pertRobotNum / bodyParams)->->getMomentOfInertia(ixx1, iyy1,izz1,ixy1,ixz1,iyz1);

    cout<<ixx1<<"\n"<<ixy1<<"\n"<<ixz1<<"\n"<<iyy1<<"\n"<<iyz1<<"\n"<<izz1<<"\n";
    */

    /*
    int i =10;

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

    Eigen::MatrixXd phi_Mat(numDataPoints *(numBodies-1), numPertRobots);

    Eigen::MatrixXd phiMatrix(numBodies-1, numPertRobots);

    Eigen::MatrixXd phi(numBodies-1,1);

    for (int i = 0; i < numDataPoints; i++) {

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

            // Set perturbed robot position to pose

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

        phi_Mat.block(7*i,0,7,70) = phiMatrix;

        myfile1 << phi_Mat << endl;

    }

    myfile.close();

    myfile1.close();

    return phiMatrix;
}

// // Read file as matrix
Eigen::MatrixXd readInputFileAsMatrix(string inputFilename) {
    return readInputFileAsMatrix(inputFilename, INT_MAX);
}

Eigen::MatrixXd readInputFileAsMatrix(string inputFilename, int stopCount) {
    ifstream infile;
    infile.open(inputFilename);

    int cols = 0, rows = 0;
    double buff[MAXBUFSIZE];

    while(! infile.eof()) {
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
    rows--;

    // Populate matrix with numbers.
    Eigen::MatrixXd outputMatrix(rows, cols);
    for (int i = 0; i < rows; i++)
        for (int j = 0; j < cols; j++)
            outputMatrix(i,j) = buff[cols*i+j];

    return outputMatrix;
}
