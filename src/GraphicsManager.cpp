#include "GraphicsManager.h"

GraphicsManager::GraphicsManager(sf::RenderWindow& window)
    : mWindow(window)
{
}

GraphicsManager::~GraphicsManager()
{
}

void GraphicsManager::init()
{
    mVehicleShape.setSize(sf::Vector2f(50, 50));
    mVehicleShape.setFillColor(sf::Color::Green);
}

void GraphicsManager::update()
{
    // Draw the vehicle
    mWindow.draw(mVehicleShape);
}
