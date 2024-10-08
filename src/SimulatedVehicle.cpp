
#include "SimulatedVehicle.h"

SimulatedVehicle::SimulatedVehicle(/* args */) : mMass(12.7), mMOI(0.35), mThrusterForce(0.15), mMomentArm(0.1), mTimestep(0.05), mDamping(0.0005),
                                                    mPose(Eigen::Vector3d::Zero()), mVelocity(Eigen::Vector3d::Zero()), mThrustForce(Eigen::Vector3d::Zero())
{

}

SimulatedVehicle::~SimulatedVehicle()
{
}

void SimulatedVehicle::update()
{
    // Calculate the acceleration based on the force
    Eigen::Vector3d acceleration = mThrustForce/mMass - mDamping* mVelocity/mMass;

    // Calculate the velocity based on the acceleration
    mVelocity += acceleration *mTimestep; 

    // Calculate the pose based on the velocity
    mPose += mVelocity*mTimestep;

    mVehicleState = {mPose(0), mPose(1), mPose(2), mVelocity(0), mVelocity(1), mVelocity(2)};

    // Publish the pose
    printf("%f, %f, %f\n", mPose(0), mPose(1), mPose(2));
}

void SimulatedVehicle::convertThrusterCommandToForce(std::string thrusterCommand)
{   
    // assume the thruster command comes in as a string of 0's and 1's
    // 0 = off, 1 = on
    // thrusterCommand = "00000000" means all thrusters are off
    // thrusterCommand = "10000000" means thruster 1 is on, all others are off

    //thrusterCommand.erase(std::remove(thrusterCommand.begin(), thrusterCommand.end(), '9'), thrusterCommand.end());

    mThrustForce = Eigen::Vector3d::Zero(); // Default case

    if (thrusterCommand == "900000011") {
        mThrustForce = Eigen::Vector3d(2*mThrusterForce, 0, 0); // +x
    } 
    else if (thrusterCommand == "900110000") 
    {
        mThrustForce = Eigen::Vector3d(-2*mThrusterForce, 0, 0); // -x
    } 
    else if (thrusterCommand == "911000000") // +y 
    {
        mThrustForce = Eigen::Vector3d(0, 2*mThrusterForce, 0);
    } 
    else if (thrusterCommand == "900001100") // -y  
    {
        mThrustForce = Eigen::Vector3d(0, -2*mThrusterForce, 0);
    } 
    else if (thrusterCommand == "901000100") // +phi  
    {
        mThrustForce = Eigen::Vector3d(0, 0, 2*mThrusterForce*mMomentArm); 
    } 
    else if (thrusterCommand == "910001000") // -phi
    {
        mThrustForce = Eigen::Vector3d(0, 0, -2*mThrusterForce*mMomentArm);
    } 
    else if (thrusterCommand == "901000010") // +x, +y
    {
        mThrustForce = Eigen::Vector3d(mThrusterForce, mThrusterForce, 0);
    } 
    else if (thrusterCommand == "900001001") // +x, -y
    {
        mThrustForce = Eigen::Vector3d(mThrusterForce, -mThrusterForce, 0);
    } 
    else if (thrusterCommand == "910010000") // -x, +y
    {
        mThrustForce = Eigen::Vector3d(-mThrusterForce, mThrusterForce, 0); 
    } 
    else if (thrusterCommand == "900100100") // -x, -y
    {
        mThrustForce = Eigen::Vector3d(-mThrusterForce, -mThrusterForce, 0);
    } 
    else if (thrusterCommand == "900000001") // +x, +phi
    {
        mThrustForce = Eigen::Vector3d(mThrusterForce, 0, mThrusterForce*mMomentArm);
    } 
    else if (thrusterCommand == "900000010") // +x, -phi
    {
        mThrustForce = Eigen::Vector3d(mThrusterForce, 0, -mThrusterForce*mMomentArm);
    } 
    else if (thrusterCommand == "900010000") // -x, +phi
    {
        mThrustForce = Eigen::Vector3d(-mThrusterForce, 0, mThrusterForce*mMomentArm);
    }
    else if (thrusterCommand == "900100000") // -x, -phi
    {
        mThrustForce = Eigen::Vector3d(-mThrusterForce, 0, -mThrusterForce*mMomentArm);
    } 
    else if (thrusterCommand == "901000000") // +y, +phi
    {
        mThrustForce = Eigen::Vector3d(0, mThrusterForce, mThrusterForce*mMomentArm);
    } 
    else if (thrusterCommand == "910000000") // +y, -phi
    {
        mThrustForce = Eigen::Vector3d(0, mThrusterForce, -mThrusterForce*mMomentArm);
    } 
    else if (thrusterCommand == "900000100") //-y, +phi
    {
        mThrustForce = Eigen::Vector3d(0, -mThrusterForce, mThrusterForce*mMomentArm);
    } 
    else if (thrusterCommand == "900001000") //-y, -phi
    {
        mThrustForce = Eigen::Vector3d(0, -mThrusterForce, -mThrusterForce*mMomentArm);
    } 
    else if (thrusterCommand == "901000001") // +x, +y, +phi
    {
        mThrustForce = Eigen::Vector3d(mThrusterForce, mThrusterForce, mThrusterForce*mMomentArm);
    } 
    else if (thrusterCommand == "900000101") // +x, -y, +phi
    {
        mThrustForce = Eigen::Vector3d(mThrusterForce, -mThrusterForce, mThrusterForce*mMomentArm);
    } 
    else if (thrusterCommand == "901010000") // -x, +y, +phi
    {
        mThrustForce = Eigen::Vector3d(-mThrusterForce, mThrusterForce, mThrusterForce*mMomentArm);
    } 
    else if (thrusterCommand == "900010100") // -x, -y, +phi
    {
        mThrustForce = Eigen::Vector3d(-mThrusterForce, -mThrusterForce, mThrusterForce*mMomentArm);
    } 
    else if (thrusterCommand == "910000010") // +x, +y, -phi
    {
        mThrustForce = Eigen::Vector3d(mThrusterForce, mThrusterForce, -mThrusterForce*mMomentArm);
    } 
    else if (thrusterCommand == "900001010") // +x, -y, -phi
    {
        mThrustForce = Eigen::Vector3d(mThrusterForce, -mThrusterForce, -mThrusterForce*mMomentArm);
    } 
    else if (thrusterCommand == "910100000") // -x, +y, -phi
    {
        mThrustForce = Eigen::Vector3d(-mThrusterForce, mThrusterForce, -mThrusterForce*mMomentArm);
    } 
    else if (thrusterCommand == "900101000") // -x, -y, -phi
    {
        mThrustForce = Eigen::Vector3d(-mThrusterForce, -mThrusterForce, -mThrusterForce*mMomentArm);
    } 
    else if (thrusterCommand == "900000000") // all off
    {
        mThrustForce = Eigen::Vector3d::Zero();
    }
    else 
    {
        mThrustForce = Eigen::Vector3d::Zero();
    }

    //printf("Thrust Force: %f, %f, %f\n", mThrustForce(0), mThrustForce(1), mThrustForce(2));
}


