#ifndef VEHICLE_VISUALIZER_HPP
#define VEHICLE_VISUALIZER_HPP

#include <SFML/Graphics.hpp>
#include "VehicleState.h"

// A class to handle visualization of a vehicle's position and orientation on a table
class VehicleVisualizer {
public:
    // Constructor: initialize the SFML window, border, and vehicle shape
    VehicleVisualizer(float table_width, float table_height, float vehicle_size, float margin);

    // Method to update the vehicle's position and orientation in the visualizer
    void update(const VehicleState &state);

    // Method to render the updated position and orientation to the screen
    void render();

    // Check if the window is open
    bool isOpen() const;

    // Process events like closing the window
    void handleEvents();

private:
    sf::RenderWindow mWindow;              // The SFML window
    sf::RectangleShape mVehicleShape;       // Shape to represent the vehicle
    sf::RectangleShape mBorderShape;        // Shape to represent the table
    float mScaleFactor;                     // Scaling factor (pixels per meter)
    float mMargin;                          // Margin size

    // Method to adjust the vehicle's position relative to the border
    sf::Vector2f transformCoordinates(double x, double y);
};

#endif // VEHICLE_VISUALIZER_HPP
