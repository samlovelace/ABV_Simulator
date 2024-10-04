#ifndef VEHICLE_VISUALIZER_HPP
#define VEHICLE_VISUALIZER_HPP

#include <SFML/Graphics.hpp>
#include "VehicleState.h"

// A class to handle visualization of a vehicle's position and orientation
class VehicleVisualizer {
public:
    // Constructor: initialize the SFML window and vehicle shape
    VehicleVisualizer(int window_width, int window_height);

    // Method to update the vehicle's position and orientation in the visualizer
    void update(const VehicleState &state);

    // Method to render the updated position and orientation to the screen
    void render();

    // Check if the window is open
    bool isOpen() const;

    // Process events like closing the window
    void handleEvents();

private:
    sf::RenderWindow mWindow;               // The SFML window
    sf::RectangleShape mVehicleShape;       // Shape to represent the vehicle
    int mWindowWidth, mWindowHeight;        // Dimensions of the window
};

#endif // VEHICLE_VISUALIZER_HPP
