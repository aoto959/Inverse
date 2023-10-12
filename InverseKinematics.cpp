// simple geometric approach assuming zero twists and offsets, neglecting ee orientation
#include <iostream>
#include <cmath>

struct LinkLengths {
    double a1;
    double a2;
    double a3;
};

struct EndEffectorPosition {
    double x;
    double y;
    double z;
};

struct JointAngles {
    double theta1;
    double theta2;
    double theta3;
};

JointAngles calculateInverseKinematics(const LinkLengths& linkLengths, const EndEffectorPosition& position) {
    JointAngles angles;

    // end effector position
    double x = position.x;
    double y = position.y;
    double z = position.z;

    // basic trigonometry
    angles.theta1 = atan2(y, x);

    // the remaining lengths in the X-Y plane
    double L = sqrt(x * x + y * y);
    double D = sqrt(L * L + z * z);

    // law of cosines
    double cosTheta3 = (linkLengths.a1 * linkLengths.a1 + linkLengths.a2 * linkLengths.a2 - D * D) / (2 * linkLengths.a1 * linkLengths.a2);
    angles.theta3 = acos(cosTheta3);

    double sinTheta3 = sin(angles.theta3);
    double theta = atan2(z, L);
    double cosTheta2 = (L * L + z * z + linkLengths.a1 * linkLengths.a1 - linkLengths.a2 * linkLengths.a2) / (2 * L * linkLengths.a1);

    angles.theta2 = theta - atan2(sinTheta3, cosTheta2);

    return angles;
}

int main() {
    LinkLengths linkLengths = {1.0, 1.0, 1.0}; 
    EndEffectorPosition position = {1.0, 1.0, 1.0}; 

    JointAngles angles = calculateInverseKinematics(linkLengths, position);

    std::cout << "Theta1: " << angles.theta1 << " radians" << std::endl;
    std::cout << "Theta2: " << angles.theta2 << " radians" << std::endl;
    std::cout << "Theta3: " << angles.theta3 << " radians" << std::endl;
}
