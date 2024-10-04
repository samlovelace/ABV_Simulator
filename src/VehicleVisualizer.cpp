#include "VehicleVisualizer.h"
#include <cmath>

// Constructor to initialize the SFML window, the table border, and the vehicle shape
VehicleVisualizer::VehicleVisualizer(float table_width, float table_height, float vehicle_size, float margin)
    : mMargin(margin) {
    // Calculate the display window size to include margins
    int windowWidth = static_cast<int>((table_width + 2 * mMargin) * 100.0f);   // Convert to pixels with a scale factor of 100x
    int windowHeight = static_cast<int>((table_height + 2 * mMargin) * 100.0f);

    // Create the main SFML window
    mWindow.create(sf::VideoMode(windowWidth, windowHeight), "Vehicle and Table Visualization with Margin");

    // Calculate the scaling factor (pixels per meter)
    mScaleFactor = 100.0f;  // Each meter is represented by 100 pixels

    // Initialize the border rectangle (table) at the center with real-world scaling
    mBorderShape.setSize(sf::Vector2f(table_width * mScaleFactor, table_height * mScaleFactor));  // Table size in pixels
    mBorderShape.setOutlineColor(sf::Color::Black);
    mBorderShape.setOutlineThickness(2.0f);
    mBorderShape.setFillColor(sf::Color::Transparent);
    mBorderShape.setPosition(
        (windowWidth - mBorderShape.getSize().x) / 2.0f,   // Center horizontally
        (windowHeight - mBorderShape.getSize().y) / 2.0f); // Center vertically

    // Create the vehicle rectangle with the correct size based on real-world scaling
    mVehicleShape.setSize(sf::Vector2f(vehicle_size * mScaleFactor, vehicle_size * mScaleFactor));  // Vehicle size in pixels
    mVehicleShape.setFillColor(sf::Color::Green);
}

// Method to transform real-world vehicle coordinates into screen coordinates
sf::Vector2f VehicleVisualizer::transformCoordinates(double x, double y) {
    // Get the position of the bottom-left corner of the border rectangle (table)
    sf::Vector2f border_bottom_left = sf::Vector2f(
        mBorderShape.getPosition().x,
        mBorderShape.getPosition().y + mBorderShape.getSize().y);

    // Convert real-world coordinates to screen coordinates relative to the bottom-left corner
    return sf::Vector2f(
        static_cast<float>(border_bottom_left.x + x * mScaleFactor),          // x = x * scale + border_x
        static_cast<float>(border_bottom_left.y - y * mScaleFactor));         // y = border_y - y * scale (invert y-axis)
}

// Method to update the vehicle's position and orientation in the visualizer
void VehicleVisualizer::update(const VehicleState &state) {
    // Transform the position of the vehicle from real-world to screen coordinates
    sf::Vector2f transformed_position = transformCoordinates(state.x, state.y);
    mVehicleShape.setPosition(transformed_position);

    // Set the rotation of the vehicle based on the yaw angle
    mVehicleShape.setRotation(static_cast<float>(state.yaw * 180.0 / M_PI));  // Convert yaw from radians to degrees
}

// Method to render the updated position and orientation to the screen
void VehicleVisualizer::render() {
    // Clear the window
    mWindow.clear(sf::Color::White);

    // Draw the border rectangle (table)
    mWindow.draw(mBorderShape);

    // Draw the vehicle
    mWindow.draw(mVehicleShape);

    // Display the rendered contents on the screen
    mWindow.display();
}

// Check if the window is still open
bool VehicleVisualizer::isOpen() const {
    return mWindow.isOpen();
}

// Handle SFML events (e.g., window close)
void VehicleVisualizer::handleEvents() {
    sf::Event event;
    while (mWindow.pollEvent(event)) {
        if (event.type == sf::Event::Closed)
            mWindow.close();
    }
}
