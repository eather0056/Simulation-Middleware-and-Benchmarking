#include <iostream>
#include <string>

int main() {
    int yearsOfService;
    std::string performanceRating;
    int bonus = 0;

    std::cout << "Enter your years of service: ";
    std::cin >> yearsOfService;

    std::cout << "Enter your performance rating (Excellent, Good, Poor): ";
    std::cin >> performanceRating;

    // Logic to determine bonus
    if (yearsOfService >= 5) {
        if (performanceRating == "Excellent") {
            bonus = 1000;
        } else if (performanceRating == "Good") {
            bonus = 500;
        }
    } else {
        if (performanceRating == "Excellent") {
            bonus = 300;
        }
    }

    // Output the bonus amount
    if (bonus > 0) {
        std::cout << "Congratulations! You get a bonus of $" << bonus << "." << std::endl;
    } else {
        std::cout << "Sorry, you do not qualify for a bonus." << std::endl;
    }

    return 0;
}
