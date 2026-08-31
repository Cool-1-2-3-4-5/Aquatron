#include "pump.hpp"

// Create the pump motor and set its default behavior for the watering routine.
Pump::Pump(char PumpMotor_port) : PumpMotor(PumpMotor_port, false)
{
    Brain.Screen.printAt(10, 50, "Pump Initialized!");
    wait(1, seconds);
    PumpMotor.setStopping(brakeType::hold);
    PumpMotor.setVelocity(0, percent);
}

Pump::~Pump()
{
}

void Pump::stop()
{
    PumpMotor.stop();
}

// Run the pump for a measured amount of time to water the plant.
void Pump::PourWater(int seconds)
{
    const int adjustment = 3;
    PumpMotor.setVelocity(100, percent);
    PumpMotor.spin(reverse);
    timeout.reset();
    while (timeout.time(sec) < (seconds * adjustment))
    {
    }
    stop();
    Brain.Screen.printAt(10, 50, "Watered!");
    wait(1,sec);
    Brain.Screen.clearScreen();
}