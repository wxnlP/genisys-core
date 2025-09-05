#include <iostream>
#include <array>

using JointAngle = std::array<float, 6>;

/* C风格，使用复杂 */
void testC(JointAngle *joint)
{
    std::cout << (joint) << std::endl;
    std::cout << (*joint)[1] << std::endl;
    std::cout << (&joint[0]) << std::endl;
    std::cout << (&joint)[1] << std::endl;
    std::cout << &(*joint)[1] << std::endl;
}

/* CPP风格 */
void testCPP(const JointAngle & joint)
{
    std::cout << joint[0] << std::endl;
    std::cout << &joint << std::endl;
    std::cout << &joint[0] << std::endl;
    std::cout << &joint[1] << std::endl;
}

int main()
{
    JointAngle angle = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0};
    testCPP(angle);
}