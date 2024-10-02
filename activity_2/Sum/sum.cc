#include <iostream>

int main() {
    int number;
    int sum = 0;

    do {
        std::cout << "Enter a number (must be >= 1): ";
        std::cin >> number;
        
        if (number < 1) {
            std::cout << "Number must be greater than or equal to 1. Please try again." << std::endl;
        }

    } while (number < 1);

    for (int i = 1; i <= number; ++i) {
        sum += i;
    }

    std::cout << "The sum of 1 to " << number << " is " << sum << "." << std::endl;

    return 0;
}
