#include <pigpio.h>
#include <iostream>

const int pwmFreq = 333;
const int pwmRange = 300;
const int pwmPins[4] = {17, 18, 14, 15};

int main() {
    if (gpioInitialise() < 0) {
        std::cerr << "GPIO initialization failed!" << std::endl;
        return 1;
    }

    gpioSetMode(4, PI_OUTPUT);
    gpioWrite(4, 1);

    std::cout << "GPIO initialized successfully." << std::endl;

    for (int i = 0; i < 4; ++i) {
        gpioSetMode(pwmPins[i], PI_OUTPUT);
        gpioSetPWMfrequency(pwmPins[i], pwmFreq);
        gpioSetPWMrange(pwmPins[i], pwmRange);
        gpioPWM(pwmPins[i], 150);
    }

    std::cout << "Enter pin index (0-3) and PWM value (0-" << pwmRange << "), e.g., `1 150`" << std::endl;

    while (true) {
        int index, value;
        std::cout << "> ";
        std::cin >> index >> value;

        if (!std::cin || index < 0 || index > 3 || value < 0 || value > pwmRange) {
            std::cin.clear();
            std::cin.ignore(1000, '\n');
            std::cout << "Invalid input. Format: <pin_index: 0-3> <value: 0-" << pwmRange << ">" << std::endl;
            continue;
        }

        gpioPWM(pwmPins[index], value);
        std::cout << "PWM pin " << pwmPins[index] << " set to " << value << std::endl;
    }

    gpioTerminate();
    return 0;
}
