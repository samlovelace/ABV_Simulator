#ifndef SIMULATEDVEHICLE_H
#define SIMULATEDVEHICLE_H

#include "Eigen/Dense"
#include "VehicleState.h"

class SimulatedVehicle
{
public:
    SimulatedVehicle(/* args */);
    ~SimulatedVehicle();

    void convertThrusterCommandToForce(std::string thrusterCommand);
    void update();

    //* Getters and Setters *//
    Eigen::Vector3d getPose() { return mPose; }
    Eigen::Vector3d getVelocity() { return mVelocity; }
    VehicleState getVehicleState() { return mVehicleState; }

private:
    double mMass; 
    double mMOI;
    double mThrusterForce; 
    double mMomentArm; 
    double mTimestep;
    double mDamping;
    
    Eigen::Vector3d mPose; // x, y, yaw
    Eigen::Vector3d mVelocity; // vx, vy, omega
    Eigen::Vector3d mThrustForce; // fx, fy, 
    
    VehicleState mVehicleState;

};


#endif // SIMULATEDVEHICLE_H