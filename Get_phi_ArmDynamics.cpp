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

//Zubair's first commit
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

    try {
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

    } catch (exception& e) {
        cout << e.what() << endl;
        return EXIT_FAILURE;
    }


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
    double xi, mxi;
    double yi, myi;
    double zi, mzi;
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

    ofstream betafile;
    betafile.open ("betaparameters.txt");
    betafile<< betaParams.transpose()<<endl;
    betafile.close();

    // Create array of robots out of phi loop for fast times
    // then change appropriate values (betaParams(i)) for each robot when
    // going through all the robots

    SkeletonPtr pertRobotArray[numPertRobots];

    for (int i = 0; i < numPertRobots; i++) {
        pertRobotArray[i] = idealRobot->clone();
        pertRobotArray[i]->setGravity(Eigen::Vector3d (0.0, -9.81, 0.0));
    }

    /*
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
    */

    for (int i = 1; i < numBodies; i++) {
        bodyi = idealRobot->getBodyNode(i);
        mi = bodyi->getMass();
        xi = bodyi->getLocalCOM()(0);
        yi = bodyi->getLocalCOM()(1);
        zi = bodyi->getLocalCOM()(2);
        bodyi->getMomentOfInertia (ixx, iyy,izz,ixy,ixz,iyz);
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

    //int pertRobotNum = 0 ;

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

    myfile1.open ("phibeta-RHS");
    myfile1<< "phibeta-RHS" << endl;

    ofstream myfile2;

    myfile2.open ("RHS_perturb.txt");
    myfile2<< "RHS_perturb" << endl;

    ofstream phifile;

    phifile.open ("phi.txt");



    Eigen::MatrixXd phi_Mat(numDataPoints *(numBodies-1), numPertRobots);

    Eigen::MatrixXd phiMatrix(numBodies-1, numPertRobots);

    Eigen::MatrixXd phi(numBodies-1,1);

    Eigen::MatrixXd phiM0(numBodies-1,1); Eigen::MatrixXd phiM0Matrix(numBodies-1, numPertRobots);
    Eigen::MatrixXd phiM1(numBodies-1,1); Eigen::MatrixXd phiM1Matrix(numBodies-1, numPertRobots);
    Eigen::MatrixXd phiM2(numBodies-1,1); Eigen::MatrixXd phiM2Matrix(numBodies-1, numPertRobots);
    Eigen::MatrixXd phiM3(numBodies-1,1); Eigen::MatrixXd phiM3Matrix(numBodies-1, numPertRobots);
    Eigen::MatrixXd phiM4(numBodies-1,1); Eigen::MatrixXd phiM4Matrix(numBodies-1, numPertRobots);
    Eigen::MatrixXd phiM5(numBodies-1,1); Eigen::MatrixXd phiM5Matrix(numBodies-1, numPertRobots);
    Eigen::MatrixXd phiM6(numBodies-1,1); Eigen::MatrixXd phiM6Matrix(numBodies-1, numPertRobots);
    Eigen::MatrixXd phiC(numBodies-1,1);  Eigen::MatrixXd phiCMatrix(numBodies-1, numPertRobots);
    Eigen::MatrixXd phiG(numBodies-1,1);  Eigen::MatrixXd phiGMatrix(numBodies-1, numPertRobots);



    for (int i = 0; i < numDataPoints; i++) {

        idealRobot->setPositions(allInitq.row(i));

        idealRobot->setVelocities(allInitqdot.row(i));

        Eigen::VectorXd ddq = allInitqdotdot.row(i);

        //idealRobot->setAccelerations(allInitqdotdot.row(i));

        Eigen::MatrixXd M = idealRobot->getMassMatrix(); // n x n

        Eigen::VectorXd C = idealRobot->getCoriolisForces(); // n x 1

        Eigen::VectorXd G = idealRobot->getGravityForces(); // n x 1

        //Eigen::VectorXd ddq = idealRobot->getAccelerations(); // n x 1

        Eigen::VectorXd RHS_ideal= M*ddq + C + G;

        myfile << RHS_ideal.transpose() << endl;

        for (int pertRobotNum = 0; pertRobotNum < numPertRobots; pertRobotNum++) {

            // Set perturbed robot position to pose

            for(int i=1; i<numBodies; i++) {

                pertRobotArray[(i-1)*bodyParams + 0]->getBodyNode(i)->setMass(mi + perturbedValue);
                pertRobotArray[(i-1)*bodyParams + 1]->getBodyNode(i)->setLocalCOM(Eigen::Vector3d(xi + perturbedValue, yi, zi));
                pertRobotArray[(i-1)*bodyParams + 2]->getBodyNode(i)->setLocalCOM(Eigen::Vector3d(xi, yi + perturbedValue, zi));
                pertRobotArray[(i-1)*bodyParams + 3]->getBodyNode(i)->setLocalCOM(Eigen::Vector3d(xi, yi, zi + perturbedValue));
                pertRobotArray[(i-1)*bodyParams + 4]->getBodyNode(i)->setMomentOfInertia(ixx + perturbedValue, iyy, izz, ixy, ixz, iyz);
                pertRobotArray[(i-1)*bodyParams + 5]->getBodyNode(i)->setMomentOfInertia(ixx, iyy + perturbedValue, izz, ixy, ixz, iyz);
                pertRobotArray[(i-1)*bodyParams + 6]->getBodyNode(i)->setMomentOfInertia(ixx, iyy, izz + perturbedValue, ixy, ixz, iyz);
                pertRobotArray[(i-1)*bodyParams + 7]->getBodyNode(i)->setMomentOfInertia(ixx, iyy, izz, ixy + perturbedValue, ixz, iyz);
                pertRobotArray[(i-1)*bodyParams + 8]->getBodyNode(i)->setMomentOfInertia(ixx, iyy, izz, ixy, ixz + perturbedValue, iyz);
                pertRobotArray[(i-1)*bodyParams + 9]->getBodyNode(i)->setMomentOfInertia(ixx, iyy, izz, ixy, ixz, iyz + perturbedValue);

            }

            pertRobotArray[pertRobotNum]->setPositions(allInitq.row(i));

            pertRobotArray[pertRobotNum]->setVelocities(allInitqdot.row(i));

            Eigen::MatrixXd M_pert = pertRobotArray[pertRobotNum]->getMassMatrix(); // n x n

            Eigen::VectorXd C_pert = pertRobotArray[pertRobotNum]->getCoriolisForces(); // n x 1

            Eigen::VectorXd G_pert = pertRobotArray[pertRobotNum]->getGravityForces(); // n x 1

            Eigen::VectorXd RHS_pert= M_pert*ddq + C_pert + G_pert;

                for(int i=1; i<numBodies; i++) {

                    pertRobotArray[(i-1)*bodyParams + 0]->getBodyNode(i)->setMass(mi - perturbedValue);
                    pertRobotArray[(i-1)*bodyParams + 1]->getBodyNode(i)->setLocalCOM(Eigen::Vector3d(xi -  perturbedValue, yi, zi));
                    pertRobotArray[(i-1)*bodyParams + 2]->getBodyNode(i)->setLocalCOM(Eigen::Vector3d(xi, yi - perturbedValue, zi));
                    pertRobotArray[(i-1)*bodyParams + 3]->getBodyNode(i)->setLocalCOM(Eigen::Vector3d(xi, yi, zi - perturbedValue));
                    pertRobotArray[(i-1)*bodyParams + 4]->getBodyNode(i)->setMomentOfInertia(ixx - perturbedValue, iyy, izz, ixy, ixz, iyz);
                    pertRobotArray[(i-1)*bodyParams + 5]->getBodyNode(i)->setMomentOfInertia(ixx, iyy - perturbedValue, izz, ixy, ixz, iyz);
                    pertRobotArray[(i-1)*bodyParams + 6]->getBodyNode(i)->setMomentOfInertia(ixx, iyy, izz - perturbedValue, ixy, ixz, iyz);
                    pertRobotArray[(i-1)*bodyParams + 7]->getBodyNode(i)->setMomentOfInertia(ixx, iyy, izz, ixy - perturbedValue, ixz, iyz);
                    pertRobotArray[(i-1)*bodyParams + 8]->getBodyNode(i)->setMomentOfInertia(ixx, iyy, izz, ixy, ixz - perturbedValue, iyz);
                    pertRobotArray[(i-1)*bodyParams + 9]->getBodyNode(i)->setMomentOfInertia(ixx, iyy, izz, ixy, ixz, iyz - perturbedValue);

                }

            pertRobotArray[pertRobotNum]->setPositions(allInitq.row(i));

            pertRobotArray[pertRobotNum]->setVelocities(allInitqdot.row(i));

            Eigen::MatrixXd M_pertback = pertRobotArray[pertRobotNum]->getMassMatrix(); // n x n

            Eigen::VectorXd C_pertback = pertRobotArray[pertRobotNum]->getCoriolisForces(); // n x 1

            Eigen::VectorXd G_pertback = pertRobotArray[pertRobotNum]->getGravityForces(); // n x 1

            Eigen::VectorXd RHS_pertback= M_pertback*ddq + C_pertback + G_pertback;



            // Calculate phi for beta i and pose
            phi = (RHS_pert - RHS_pertback)/(2*perturbedValue);

            // Add phi to phiMatrix and then print it looks cleaner
            phiMatrix.col(pertRobotNum) = phi;

            // Calculate parts of phi
            /*
            phiM0 = (M_pert.col(0) - M.col(0))/perturbedValue; phiM0Matrix.col(pertRobotNum) = phiM0;
            phiM1 = (M_pert.col(1) - M.col(1))/perturbedValue; phiM1Matrix.col(pertRobotNum) = phiM1;
            phiM2 = (M_pert.col(2) - M.col(2))/perturbedValue; phiM2Matrix.col(pertRobotNum) = phiM2;
            phiM3 = (M_pert.col(3) - M.col(3))/perturbedValue; phiM3Matrix.col(pertRobotNum) = phiM3;
            phiM4 = (M_pert.col(4) - M.col(4))/perturbedValue; phiM4Matrix.col(pertRobotNum) = phiM4;
            phiM5 = (M_pert.col(5) - M.col(5))/perturbedValue; phiM5Matrix.col(pertRobotNum) = phiM5;
            phiM6 = (M_pert.col(6) - M.col(6))/perturbedValue; phiM6Matrix.col(pertRobotNum) = phiM6;
            phiC = (C_pert - C)/perturbedValue; phiCMatrix.col(pertRobotNum) = phiC;
            phiG = (G_pert - G)/perturbedValue; phiGMatrix.col(pertRobotNum) = phiG;
            */

            //if (pertRobotNum == numPertRobots-1) {

            //myfile2 << "phi_Mat_short at: " << i << endl;

            //myfile2 << phiMatrix << endl<< endl;

            //cout<<"Rows:" <<endl<<phiMatrix.rows()<<"Cols:" <<endl<<phiMatrix.cols();

            // }

            //cout<<"phi_matrix calcualted at " <<pertRobotNum << " & data point "<<i <<endl <<phiMatrix<<endl<<endl;
        }

        // if: rhs = a*m + b*m*x + c*m*y + d*m*z
        // then: d_m = a + b*x + c*y + d*z
        // d_x = b*m
        // d_y = c*m
        // d_z = d*m
        // So: [b, c, d] = [d_x, d_y, d_z]/m
        // And: a = d_m - b*x - c*y - d*z


        // rhs        = a*m + b*m*x + c*m*y + d*m*z + e*m*x^2 + f*m*y^2 + g*m*z^2

        // then d_m   = a + b*x + c*y + d*z + 2*e*m*x + 2*f*m*y + 2*g*m*z

        /*
           dd_m       = 2*e*x + 2*e*y +2*e*z;

           d_x        = b + 2*e*m;

           d_y        = c + 2*f*m;

           d_z        = d + 2*g*m;


           d_x_d_m    = 2*e;

           d_y_d_m    = 2*f;

           d_z_d_m    = 2*g;

           b          = d_x - m * d_x_d_m;

           c          = d_y - m * d_y_d_m;

           d          = d_z - m*  d_z_d_m;


           a          = d_m - b*x - c*y - d*z - m*dd_m;
           */


        for(int b=1; b<numBodies; b++) {
            int c = bodyParams*(b-1);
            double m = idealRobot->getBodyNode(b)->getMass();
            Eigen::Vector3d COM = idealRobot->getBodyNode(b)->getLocalCOM();

            phiMatrix.block<7,3>(0,c+1) = phiMatrix.block<7,3>(0,c+1)/m; phiMatrix.col(c)   =  phiMatrix.col(c)   - phiMatrix.col(c+1)*COM(0)   - phiMatrix.col(c+2)*COM(1)   - phiMatrix.col(c+3)*COM(2) ;
            /*
            phiM0Matrix.block<7,3>(0,c+1) = phiM0Matrix.block<7,3>(0,c+1)/m; phiM0Matrix.col(c) =  phiM0Matrix.col(c) - phiM0Matrix.col(c+1)*COM(0) - phiM0Matrix.col(c+2)*COM(1) - phiM0Matrix.col(c+3)*COM(2) ;
            phiM1Matrix.block<7,3>(0,c+1) = phiM1Matrix.block<7,3>(0,c+1)/m; phiM1Matrix.col(c) =  phiM1Matrix.col(c) - phiM1Matrix.col(c+1)*COM(0) - phiM1Matrix.col(c+2)*COM(1) - phiM1Matrix.col(c+3)*COM(2) ;
            phiM2Matrix.block<7,3>(0,c+1) = phiM2Matrix.block<7,3>(0,c+1)/m; phiM2Matrix.col(c) =  phiM2Matrix.col(c) - phiM2Matrix.col(c+1)*COM(0) - phiM2Matrix.col(c+2)*COM(1) - phiM2Matrix.col(c+3)*COM(2) ;
            phiM3Matrix.block<7,3>(0,c+1) = phiM3Matrix.block<7,3>(0,c+1)/m; phiM3Matrix.col(c) =  phiM3Matrix.col(c) - phiM3Matrix.col(c+1)*COM(0) - phiM3Matrix.col(c+2)*COM(1) - phiM3Matrix.col(c+3)*COM(2) ;
            phiM4Matrix.block<7,3>(0,c+1) = phiM4Matrix.block<7,3>(0,c+1)/m; phiM4Matrix.col(c) =  phiM4Matrix.col(c) - phiM4Matrix.col(c+1)*COM(0) - phiM4Matrix.col(c+2)*COM(1) - phiM4Matrix.col(c+3)*COM(2) ;
            phiM5Matrix.block<7,3>(0,c+1) = phiM5Matrix.block<7,3>(0,c+1)/m; phiM5Matrix.col(c) =  phiM5Matrix.col(c) - phiM5Matrix.col(c+1)*COM(0) - phiM5Matrix.col(c+2)*COM(1) - phiM5Matrix.col(c+3)*COM(2) ;
            phiM6Matrix.block<7,3>(0,c+1) = phiM6Matrix.block<7,3>(0,c+1)/m; phiM6Matrix.col(c) =  phiM6Matrix.col(c) - phiM6Matrix.col(c+1)*COM(0) - phiM6Matrix.col(c+2)*COM(1) - phiM6Matrix.col(c+3)*COM(2) ;
            phiCMatrix.block<7,3>(0,c+1) = phiCMatrix.block<7,3>(0,c+1)/m; phiCMatrix.col(c)  =  phiCMatrix.col(c)  - phiCMatrix.col(c+1)*COM(0)  - phiCMatrix.col(c+2)*COM(1)  - phiCMatrix.col(c+3)*COM(2) ;
            phiGMatrix.block<7,3>(0,c+1) = phiGMatrix.block<7,3>(0,c+1)/m; phiGMatrix.col(c)  =  phiGMatrix.col(c)  - phiGMatrix.col(c+1)*COM(0)  - phiGMatrix.col(c+2)*COM(1)  - phiGMatrix.col(c+3)*COM(2) ;
            */
        }


        Eigen::MatrixXd rhs_phibeta_diff(numBodies-1,3);
        rhs_phibeta_diff <<  RHS_ideal, (phiMatrix*betaParams.transpose()), ((phiMatrix*betaParams.transpose()) - RHS_ideal);
        myfile1<<"RHS, phi*beta, difference at "<< i << endl << endl << rhs_phibeta_diff << endl << endl;

        /*

        rhs_phibeta_diff <<  M.col(0), (phiM0Matrix*betaParams.transpose()), ((phiM0Matrix*betaParams.transpose()) - M.col(0));
        myfile1<<"M0, phiM0Matrix*beta, difference at "<< i << endl << endl << rhs_phibeta_diff << endl << endl;

        rhs_phibeta_diff <<  M.col(1), (phiM1Matrix*betaParams.transpose()), ((phiM1Matrix*betaParams.transpose()) - M.col(1));
        myfile1<<"M1, phiM1Matrix*beta, difference at "<< i << endl << endl << rhs_phibeta_diff << endl << endl;

        rhs_phibeta_diff <<  M.col(2), (phiM2Matrix*betaParams.transpose()), ((phiM2Matrix*betaParams.transpose()) - M.col(2));
        myfile1<<"M2, phiM2Matrix*beta, difference at "<< i << endl << endl << rhs_phibeta_diff << endl << endl;

        rhs_phibeta_diff <<  M.col(3), (phiM3Matrix*betaParams.transpose()), ((phiM3Matrix*betaParams.transpose()) - M.col(3));
        myfile1<<"M3, phiM3Matrix*beta, difference at "<< i << endl << endl << rhs_phibeta_diff << endl << endl;

        rhs_phibeta_diff <<  M.col(4), (phiM4Matrix*betaParams.transpose()), ((phiM4Matrix*betaParams.transpose()) - M.col(4));
        myfile1<<"M4, phiM4Matrix*beta, difference at "<< i << endl << endl << rhs_phibeta_diff << endl << endl;

        rhs_phibeta_diff <<  M.col(5), (phiM5Matrix*betaParams.transpose()), ((phiM5Matrix*betaParams.transpose()) - M.col(5));
        myfile1<<"M5, phiM5Matrix*beta, difference at "<< i << endl << endl << rhs_phibeta_diff << endl << endl;

        rhs_phibeta_diff <<  M.col(6), (phiM6Matrix*betaParams.transpose()), ((phiM6Matrix*betaParams.transpose()) - M.col(6));
        myfile1<<"M6, phiM6Matrix*beta, difference at "<< i << endl << endl << rhs_phibeta_diff << endl << endl;

        rhs_phibeta_diff <<  C, (phiCMatrix*betaParams.transpose()), ((phiCMatrix*betaParams.transpose()) - C);
        myfile1<<"C, phiCMatrix*beta, difference at "<< i << endl << endl << rhs_phibeta_diff << endl << endl;

        rhs_phibeta_diff <<  G, (phiGMatrix*betaParams.transpose()), ((phiGMatrix*betaParams.transpose()) - G);
        myfile1<<"G, phiGMatrix*beta, difference at "<< i << endl << endl << rhs_phibeta_diff << endl << endl;
        */
        myfile1<< "=========================================================================" << endl << endl << endl << endl;


        phifile<< phiMatrix<<endl;


        myfile2 <<"PHI_MAT AT:" << i <<endl<<endl;


        myfile2 << phiMatrix<<endl;

        //cout<<endl<<"rows of phi_mat at : "<<i<< endl<<endl<<phiMatrix.rows()<<endl<<endl<<"cols of phi_mat at : "<<i<< endl<<endl<<phiMatrix.cols();


        //phi_Mat.block(7*i,0,7,70)=phiMatrix;

        //myfile1 << phi_Mat << endl;

    }


    myfile.close();

    myfile1.close();

    myfile2.close();

    phifile.close();

    return phiMatrix;
}

// // Read file as matrix
Eigen::MatrixXd readInputFileAsMatrix(string inputFilename) {
    return readInputFileAsMatrix(inputFilename, INT_MAX);
}

Eigen::MatrixXd readInputFileAsMatrix(string inputFilename, int stopCount) {
    ifstream infile;
    infile.open(inputFilename);

    if (!infile.is_open()) {
        throw runtime_error(inputFilename + " can not be read, potentially does not exit!");
    }

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
