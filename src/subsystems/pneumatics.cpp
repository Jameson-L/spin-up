#include "main.h"
#include "subsystems/pneumatics.hpp"

// int kPneumaticIndexerPort = 1; // indexer piston port
pros::ADIDigitalOut expansion(7);
pros::ADIDigitalOut blooper(5);
pros::ADIDigitalOut compression1(1); // actuate = downward
pros::ADIDigitalOut compression2(2);
 