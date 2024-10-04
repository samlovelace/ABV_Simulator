
// ABV_Simulator takes in the thruster command sequence that is sent to the Arduino onboard the ABV.
// From the thruster command sequence, the applied forces are determined and the resulting acceleration is calculated. 
// From the acceleration, the velocity and position of the ABV are determined through numerical integration. (Euler's method)
// The pose of the ABV is then published in a similar manner to the OptiTract system. 

#include <cstdio>
#include "SimulatedVehicle.h"
#include "VehicleVisualizer.h"
#include "VehicleState.h"
#include "Controller.h"
#include <SFML/Graphics.hpp>
#include <csignal>
#include <thread>


void signalHandler(int signum) {
    printf("Interrupt signal (%d) received.\n", signum);
    // Cleanup and close up stuff here
    // Terminate program
    exit(signum);
}

int main() {

    // Register signal and signal handler
    std::signal(SIGINT, signalHandler);

    SimulatedVehicle* abv = new SimulatedVehicle(); 
    Controller* controller = new Controller(); 
    VehicleVisualizer* visualizer = new VehicleVisualizer(800, 600);
    VehicleState vehicleState; 

    Eigen::Vector3d desiredPose = Eigen::Vector3d(1, 1, 0.157);
    Eigen::Vector3d desiredVelocity = Eigen::Vector3d(0, 0, 0);


    while(visualizer->isOpen()) {

        // graphics handling
        visualizer->handleEvents();

        // set the desired pose and velocity
        controller->set_values(abv->getPose(), abv->getVelocity(), desiredPose, desiredVelocity);
        controller->det_thrust_dir();
        controller->calc_thrust_com();

        // convert the thruster command to a force and update the vehicle
        abv->convertThrusterCommandToForce(controller->get_thrust_com());
        abv->update();

        // get the current vehicleState to draw
        vehicleState = abv->getVehicleState();

        // Update the visualizer with the new vehicle state
        visualizer->update(vehicleState);

        // Render the updated visualization
        visualizer->render();

        // Add a small delay to control the simulation speed
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    printf("Simulation complete\n");
    Eigen::Vector3d finalPose = abv->getPose();
    Eigen::Vector3d poseDifference = finalPose - desiredPose;

    printf("Final Pose: [%f, %f, %f]\n", finalPose.x(), finalPose.y(), finalPose.z());
    printf("Difference from Desired Pose: [%f, %f, %f]\n", poseDifference.x(), poseDifference.y(), poseDifference.z());

    delete abv;
    delete controller;
    return 0;
}
