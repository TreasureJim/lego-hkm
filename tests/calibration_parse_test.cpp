#include "calibration.hpp"
#include <iostream>
#include <fstream>
#include <cassert>
#include <array>

int main() {
    // Create a temporary file with six double values, each on a new line
    const std::string file_name = "test_data.txt";
    std::ofstream outFile(file_name);
    outFile << "1.1\n2.2\n3.3\n4.4\n5.5\n6.6\n";
    outFile.close();

    try {
        // Call the function to read doubles from the file
        std::array<std::array<double, 2>, 3> data = read_calibration_file(file_name);
        
        // Define the expected data
        std::array<std::array<double, 2>, 3> expected = {{
            {{1.1, 2.2}},
            {{3.3, 4.4}},
            {{5.5, 6.6}}
        }};
        
        // Assert that the read data matches the expected data
        assert(data == expected);
        
        std::cout << "Test passed: read_calibration_file function works correctly.\n";
    }
    catch(const std::exception& e){
        std::cerr << "Test failed: " << e.what() << '\n';
        return 1;
    }

    // Clean up the temporary file
    remove(file_name.c_str());

    return 0;
}
