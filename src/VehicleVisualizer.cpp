#include "VehicleVisualizer.h"
#include <cmath>

// Constructor to initialize the SFML mWindow and the vehicle shape
VehicleVisualizer::VehicleVisualizer(int width, int height)
    : mWindowWidth(width), mWindowHeight(height) {
    // Create an SFML window
    mWindow.create(sf::VideoMode(mWindowWidth, mWindowHeight), "Real-Time Vehicle Simulation");

    // Initialize the vehicle shape (a square)
    mVehicleShape.setSize(sf::Vector2f(50.0f, 50.0f));  // Square of size 50x50 pixels
    mVehicleShape.setFillColor(sf::Color::Green);
    mVehicleShape.setOrigin(mVehicleShape.getSize().x / 2.0f, mVehicleShape.getSize().y / 2.0f);  // Set origin to center
}

// Method to update the vehicle's position and orientation in the visualizer
void VehicleVisualizer::update(const VehicleState &state) {
    // Set the position of the square to the vehicle's position
    mVehicleShape.setPosition(static_cast<float>(state.x), static_cast<float>(state.y));

    // Set the rotation of the square based on the yaw angle
    mVehicleShape.setRotation(static_cast<float>(state.yaw * 180.0 / M_PI));  // Convert radians to degrees
}

// Method to render the updated position and orientation to the screen
void VehicleVisualizer::render() {
    // Clear the window
    mWindow.clear(sf::Color::White);

    // Draw the vehicle shape
    mWindow.draw(mVehicleShape);

    // Display the rendered contents on the screen
    mWindow.display();
}

// Check if the mWindow is still open
bool VehicleVisualizer::isOpen() const {
    return mWindow.isOpen();
}

// Handle SFML events (e.g., mWindow close)
void VehicleVisualizer::handleEvents() {
    sf::Event event;
    while (mWindow.pollEvent(event)) {
        if (event.type == sf::Event::Closed)
            mWindow.close();
    }
}
