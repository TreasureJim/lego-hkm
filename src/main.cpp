#include "calibration.hpp"
#include "client.hpp"
#include "juliet_comms.hpp"
#include "lego_model.hpp"
#include "motors.hpp"
#include "robot.hpp"
#include <arpa/inet.h>
#include <asm-generic/socket.h>
#include <boost/program_options.hpp>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <netinet/in.h>
#include <signal.h>
#include <stdexcept>
#include <sys/socket.h>
#include <thread>

namespace po = boost::program_options;

Robot *robot;
std::thread *robot_thread;

void handle_sig(int sig) {
	cleanup_juliet_comms();

	stop_robot_thread = true;
	robot_thread->join();

	if (robot)
		delete robot;
	exit(0);
}

int main(int argc, char *argv[]) {
	signal(SIGINT, handle_sig);

	std::string ipv4_addr;
	int port;

	std::string robot_type;

	std::string calibration_file;
	bool calibrate = false;

	try {
		// Define the options
		po::options_description desc("Allowed options");

		// clang-format off
		desc.add_options()
			("help,h", "produce help message")
			("IPv4_addr", po::value<std::string>(&ipv4_addr), "IPv4 address (required if not calibrating)")
			("port", po::value<int>(&port), "port number (required if not calibrating)")
			("calibrate", po::bool_switch(&calibrate), "perform calibration")
			("cf", po::value<std::string>(&calibration_file)->default_value("./calibration.data"), "calibration file path")
			("robot", po::value<std::string>(&robot_type)->default_value("lego")->notifier([](const std::string& value) {
					if (value != "lego" && value != "sim") {
							throw po::validation_error(po::validation_error::invalid_option_value, "robot", value);
					}
			}), "robot type (either 'lego' or 'sim')");
		// clang-format on

		// Parse the command line arguments
		po::positional_options_description p;
		p.add("IPv4_addr", 1);
		p.add("port", 1);

		po::variables_map vm;
		po::store(po::command_line_parser(argc, argv).options(desc).positional(p).run(), vm);

		if (vm.count("help")) {
			std::cout << desc << "\n";
			return 0;
		}

		// Notify will throw if any required arguments are missing
		po::notify(vm);

		if (!calibrate) {
			if (!vm.count("IPv4_addr") || !vm.count("port")) {
				std::cerr << "[Error] IPv4_addr and port are required unless --calibrate is specified.\n";
				return 1;
			}
		} else {
			if (robot_type != "lego") {
				std::cerr << "[Error] Only lego robot is supported for calibration.\n";
				return 1;
			}
		}

	} catch (const po::error &e) {
		std::cerr << "[Error] " << e.what() << std::endl;
		return 1;
	}

	if (robot_type == "lego") {
		try {
			motor_offset_values = read_calibration_file(calibration_file);
		} catch (const std::runtime_error &e) {
			std::cerr << "[ERROR] " << e.what() << std::endl;
				exit(EXIT_FAILURE);
		}
	}

	if (calibrate) {
		if (!motor_setup()) {
			std::cerr << "[ERROR] Could not setup motors" << std::endl;
			exit(EXIT_FAILURE);
		}
		motor_reset_angle(); // Call the calibration function
		motor_shutdown();
		return 0; // Exit after calibration
	}

	int jl_socket;
	if ((jl_socket = connect_to_server(argv[1], argv[2])) < 0)
		exit(1);

	if (robot_type == "lego")
		robot = new LegoRobot(lego_model);
	else if (robot_type == "sim")
		robot = new FakeVisRobot();

	if (robot->error) {
		fprintf(stderr, "[ERROR] Could not initialise robot.\n");
		exit(EXIT_FAILURE);
	}

	robot_thread = new std::thread(robot_thread_func, robot);

	juliet_communication(jl_socket, robot->get_current_cart_loc(), &robot->get_model());

	return 0;
}
