#include <iostream>

int main() {
    int number;
    int sum = 0;

    std::cout << "Enter a number: ";
    std::cin >> number;

    if (number >= 1) {
        for (int i = 1; i <= number; ++i) {
            sum += i;
        }
        std::cout << "The sum of 1 to " << number << " is " << sum << "." << std::endl;
    } else {
        std::cout << "Number must be greater than or equal to 1." << std::endl;
    }

    return 0;
}
