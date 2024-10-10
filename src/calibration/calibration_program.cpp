#include "rc/adc.h"
#include "rc/servo.h"
#include <cctype>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <termios.h>
#include <thread>
#include <unistd.h>
#include <vector>

using namespace std::chrono_literals;

#define DEFAULT_CHANGE_VAL 0.1f

bool motor_setup() {
	// read adc to make sure battery is connected
	if (rc_adc_init()) {
		fprintf(stderr, "[ERROR] failed to run rc_adc_init()\n");
		return false;
	}
	if (rc_adc_batt() < 6.0) {
		fprintf(stderr, "[ERROR] battery disconnected or insufficiently charged to drive servos\n");
		return false;
	}
	rc_adc_cleanup();

	if (rc_servo_init()) {
		fprintf(stderr, "[ERROR] Could not initialise servos.\n");
		return false;
	}
	if (rc_servo_power_rail_en(1)) {
		fprintf(stderr, "[ERROR] Could not enable 6V power rail.\n");
		return false;
	}

	std::cout << "Setting motors to middle positions." << std::endl;

	rc_servo_send_pulse_normalized(1, 0.0);
	rc_servo_send_pulse_normalized(2, 0.0);
	rc_servo_send_pulse_normalized(3, 0.0);

	std::this_thread::sleep_for(30ms);

	return true;
}

// Function to set terminal to raw mode
void setRawMode(bool enable) {
	static struct termios oldt, newt;
	if (enable) {
		tcgetattr(STDIN_FILENO, &oldt);
		newt = oldt;
		newt.c_lflag &= ~(ICANON | ECHO); // Disable canonical mode and echo
		tcsetattr(STDIN_FILENO, TCSANOW, &newt);
	} else {
		tcsetattr(STDIN_FILENO, TCSANOW, &oldt); // Restore the terminal
	}
}

// Function to display the current values and the index being modified
void displayValues(const std::vector<float> &values, int currentIndex, const std::string &currentInput) {
	std::cout << "\033[H";              // Set cursor to start position
	std::cout << "\033[J" << std::endl; // Clear screen

	std::cout << "Use '+' and '-' to adjust, Enter to confirm.\n\n";

	for (size_t axis = 0; axis < 3; axis++) {
		std::cout << "Axis " << axis + 1 << "\n";

		if (currentIndex == axis * 2)
			std::cout << "> Minimum: " << values[axis * 2] << "  <--\n";
		else
			std::cout << "Minimum: " << values[axis * 2] << "\n";

		if (currentIndex == axis * 2 + 1)
			std::cout << "> Maximum: " << values[axis * 2 + 1] << "  <--\n";
		else
			std::cout << "Maximum: " << values[axis * 2 + 1] << "\n";

		std::cout << "\n";
	}

	std::cout << "\033[" << 200 << ";" << 0 << "H"; // Move cursor to last line
	std::cout << currentInput;
}

void saveToFile(const std::vector<float> &values, const std::string &filename) {
	std::ofstream outFile(filename);
	if (!outFile) {
		std::cerr << "Error: Could not open file " << filename << " for writing.\n";
		return;
	}
	for (const float &value : values) {
		outFile << value << "\n";
	}
	std::cout << "Values successfully saved to " << filename << "\n";
}

int main(int argc, char *argv[]) {
	if (argc < 2) {
		std::cerr << "Usage: " << argv[0] << " <output_filename>\n";
		return 1;
	}

	if (!motor_setup())
		exit(1);

	std::string filename = argv[1];
	std::vector<float> values(6, 0.0f); // Initialize all 6 values to 0.0
	int currentIndex = 0;
	char input;
	std::string currentInput = ""; // Buffer for the current float input

	// Set terminal to raw mode to capture key presses immediately
	setRawMode(true);

	while (currentIndex < 6) {
		displayValues(values, currentIndex, currentInput);

		input = getchar(); // Read single character

		// If input is a digit or a dot, accumulate it in currentInput
		if (isdigit(input) || input == '.') {
			currentInput += input;
		}
		// If backspace
		else if (input == 127 && !currentInput.empty()) {
			currentInput.erase(currentInput.length() - 1);
		}
		// '+' or '-' to adjust the value once entered
		else if (input == '+' || input == '=') {
			if (!currentInput.empty()) {
				values[currentIndex] += std::stof(currentInput);
				currentInput.clear(); // Clear current input after using it
			} else
				values[currentIndex] += DEFAULT_CHANGE_VAL; // Increment
		} else if (input == '-') {
			if (!currentInput.empty()) {
				values[currentIndex] -= std::stof(currentInput);
				currentInput.clear();
			} else
				values[currentIndex] -= DEFAULT_CHANGE_VAL;
		}
		// Enter to confirm the value and move to the next one
		else if (input == '\n' || input == '\r') {
			currentInput.clear(); // Reset input buffer
			currentIndex++;       // Move to the next value
		}

		if (values[currentIndex] < -1.5)
			values[currentIndex] = -1.5;
		else if (values[currentIndex] > 1.5)
			values[currentIndex] = 1.5;

		rc_servo_send_pulse_normalized(currentIndex / 2 + 1, values[currentIndex]);
		std::this_thread::sleep_for(30ms);
	}

	// Restore terminal to normal mode before exiting
	setRawMode(false);

	// Save the values to the file
	saveToFile(values, filename);

	return 0;
}
