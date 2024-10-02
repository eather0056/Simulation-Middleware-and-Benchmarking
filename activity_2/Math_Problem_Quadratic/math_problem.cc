#include <iostream>
#include <cmath>
#include <complex> 

int main() {
    double a, b, c;
    double discriminant, root1, root2;

    std::cout << "Enter coefficient a (not 0): ";
    std::cin >> a;
    std::cout << "Enter coefficient b: ";
    std::cin >> b;
    std::cout << "Enter coefficient c: ";
    std::cin >> c;

    // If a is zero, it is not a quadratic equation
    if (a == 0) {
        std::cout << "This is not a quadratic equation since a = 0." << std::endl;
        return 0;
    }

    // Calculate the discriminant
    discriminant = b * b - 4 * a * c;

    // Case 1: Discriminant < 0 → Complex solutions, no real solution
    if (discriminant < 0) {
        double realPart = -b / (2 * a);
        double imaginaryPart = std::sqrt(-discriminant) / (2 * a);
        std::cout << "Complex solutions:" << std::endl;
        std::cout << "x1 = " << realPart << " + " << imaginaryPart << "i" << std::endl;
        std::cout << "x2 = " << realPart << " - " << imaginaryPart << "i" << std::endl;
    }
    // Case 2: Discriminant == 0 → Single real solution
    else if (discriminant == 0) {
        root1 = -b / (2 * a);
        std::cout << "Single solution: x = " << root1 << std::endl;
    }
    // Case 3: Discriminant > 0 → Two real solutions
    else {
        root1 = (-b + std::sqrt(discriminant)) / (2 * a);
        root2 = (-b - std::sqrt(discriminant)) / (2 * a);
        std::cout << "Real solutions:" << std::endl;
        std::cout << "x1 = " << root1 << std::endl;
        std::cout << "x2 = " << root2 << std::endl;
    }

    return 0;
}
