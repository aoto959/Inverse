// Inverse.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <iostream>
#include "InverseKinematics.h"


int main() {
	LinkLengths linkLengths = { 1.0, 1.0, 1.0 };
	EndEffectorPosition position = { 1.0, 1.0, 0 };

	JointAngles angles = calculateInverseKinematics(linkLengths, position);

	std::cout << "Theta1: " << angles.theta1 << " radians" << std::endl;
	std::cout << "Theta2: " << angles.theta2 << " radians" << std::endl;
	std::cout << "Theta3: " << angles.theta3 << " radians" << std::endl;
}

