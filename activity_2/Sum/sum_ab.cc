#include <iostream>

int main() {
    int a, b, sum = 0;

    do {
        std::cout << "Enter a number >= 1: ";
        std::cin >> a;
        if (a < 1) {
            std::cout << "The number must be greater than or equal to 1. Please try again." << std::endl;
        }
    } while (a < 1);

    do {
        std::cout << "Enter b number >= 1: ";
        std::cin >> b;
        if (b < 1) {
            std::cout << "The number must be greater than or equal to 1. Please try again." << std::endl;
        }
    } while (b < 1);

    if (a > b) {
        std::swap(a, b);
    }

    for (int i = a; i <= b; ++i) {
        sum += i;
    }

    std::cout << "The sum from " << a << " to " << b << " is " << sum << "." << std::endl;

    return 0;
}
