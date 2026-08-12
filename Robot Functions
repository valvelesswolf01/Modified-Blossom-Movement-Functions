//This code uses the Dynamixel SDK and four XL330-M288-T Dynamixel motors

#include <iostream>
#include "C:\Program Files (x86)\dynamixel_sdk\include\dynamixel_sdk\dynamixel_sdk.h" //gets dynamixel_sdk library to use for functions
#include <string> //used for the commands

int main() {
	dynamixel::PortHandler* portHandler = dynamixel::PortHandler::getPortHandler("COM4"); //robot connected on com4
	dynamixel::PacketHandler* packetHandler = dynamixel::PacketHandler::getPacketHandler(2.0); //protocol 2.0
	dynamixel::GroupBulkWrite groupBulkWrite(portHandler, packetHandler);
	dynamixel::GroupBulkRead groupBulkRead(portHandler, packetHandler);

	using std::string;	//able to use strings for commands

	uint8_t dxl_error = 0; //returns 0 
	int dxl_comm_result = COMM_TX_FAIL;
	bool dxl_addparam_result = false;
	bool dxl_getdata_result = false;

	if (portHandler->openPort()) {
		std::cout << "Succeeded in opening port.\n"; //connects to robot 
	}
	else {
		std::cout << "Failed to open port.\n"; //error case if not connected or other port issues
		return 0;
	}

	if (portHandler->setBaudRate(57600)) {
		std::cout << "Succeeded in changing baudrate\n"; //baudrate of motors = 57600
	}
	else {
		std::cout << "Failed to change baudrate.\n"; //error case if the baudrate can't change
		return 0;
	}

	uint8_t dxl_base = 5; //base motor id = 5
	uint8_t dxl_face = 3; //front motor id = 3
	uint8_t dxl_left = 1; //left motor id = 1
	uint8_t dxl_right = 2; //right motor id = 2
	uint16_t torque_address = 64; //
	uint8_t data = 1; //turns torque on
	uint16_t profile_velocity_address = 112;

	dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, dxl_base, profile_velocity_address, 90, &dxl_error); //changes profile velocity for smooth motion
	if (dxl_comm_result != COMM_SUCCESS) {
		std::cout << packetHandler->getTxRxResult(dxl_comm_result) << std::endl;
	}
	else if (dxl_error != 0) {
		std::cout << packetHandler->getRxPacketError(dxl_error) << std::endl;
	}
	else {
		std::cout << "Base velocity changed. \n";
	}
	dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, dxl_left, profile_velocity_address, 90, &dxl_error); //changes profile velocity for smooth motion
	if (dxl_comm_result != COMM_SUCCESS) {
		std::cout << packetHandler->getTxRxResult(dxl_comm_result) << std::endl;
	}
	else if (dxl_error != 0) {
		std::cout << packetHandler->getRxPacketError(dxl_error) << std::endl;
	}
	else {
		std::cout << "Left velocity changed. \n";
	}
	dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, dxl_right, profile_velocity_address, 90, &dxl_error); //changes profile velocity for smooth motion
	if (dxl_comm_result != COMM_SUCCESS) {
		std::cout << packetHandler->getTxRxResult(dxl_comm_result) << std::endl;
	}
	else if (dxl_error != 0) {
		std::cout << packetHandler->getRxPacketError(dxl_error) << std::endl;
	}
	else {
		std::cout << "Right velocity changed. \n";
	}

	dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, dxl_base, torque_address, data, &dxl_error); //connects to each individual motor
	if (dxl_comm_result != COMM_SUCCESS) {
		std::cout << packetHandler->getTxRxResult(dxl_comm_result) << std::endl;
	}
	else if (dxl_error != 0) {
		std::cout << packetHandler->getRxPacketError(dxl_error) << std::endl;
	}
	else {
		std::cout << "Base is connected. \n";
	}

	dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, dxl_face, torque_address, data, &dxl_error); //Connects motor at the front
	if (dxl_comm_result != COMM_SUCCESS) {
		std::cout << packetHandler->getTxRxResult(dxl_comm_result) << std::endl;
	}
	else if (dxl_error != 0) {
		std::cout << packetHandler->getRxPacketError(dxl_error) << std::endl;
	}
	else {
		std::cout << "Face is connected. \n";
	}

	dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, dxl_left, torque_address, data, &dxl_error); // Left motor from robots point of view
	if (dxl_comm_result != COMM_SUCCESS) {
		std::cout << packetHandler->getTxRxResult(dxl_comm_result) << std::endl;
	}
	else if (dxl_error != 0) {
		std::cout << packetHandler->getRxPacketError(dxl_error) << std::endl;
	}
	else {
		std::cout << "Left side is connected. \n";
	}

	dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, dxl_right, torque_address, data, &dxl_error); //Right motor from robots point of view
	if (dxl_comm_result != COMM_SUCCESS) {
		std::cout << packetHandler->getTxRxResult(dxl_comm_result) << std::endl;
	}
	else if (dxl_error != 0) {
		std::cout << packetHandler->getRxPacketError(dxl_error) << std::endl;
	}
	else {
		std::cout << "Right side is connected. \n";
	}

	uint16_t data_len_4byte = 4; //Length of information sent to move motors

	string command; //saves entered command
	while (true) {
		std::cout << "Enter a command: "; //asks for manual command
		std::cin >> command;

		if (command == "5") { //resets robot to the default position. base-2000, face-1350, left-1150, right-1650
			int target_pos_base = 2000;

			uint8_t param_goal_position[4]; //moves base
			param_goal_position[0] = DXL_LOBYTE(DXL_LOWORD(target_pos_base));
			param_goal_position[1] = DXL_HIBYTE(DXL_LOWORD(target_pos_base));
			param_goal_position[2] = DXL_LOBYTE(DXL_HIWORD(target_pos_base));
			param_goal_position[3] = DXL_HIBYTE(DXL_HIWORD(target_pos_base));

			uint16_t goal_position_address = 116;
			dxl_addparam_result = groupBulkWrite.addParam(dxl_base, goal_position_address, data_len_4byte, param_goal_position);
			if (!dxl_addparam_result) {
				std::cout << "[ID:" << (int)dxl_base << "]groupBulkWrite addparam failed" << std::endl;
				return 0;
			}
			dxl_comm_result = groupBulkWrite.txPacket();
			if (dxl_comm_result != COMM_SUCCESS) {
				std::cout << packetHandler->getTxRxResult(dxl_comm_result) << std::endl;
			}

			int target_pos_face = 1250;

			uint8_t param_goal_face[4]; //Moves face
			param_goal_face[0] = DXL_LOBYTE(DXL_LOWORD(target_pos_face));
			param_goal_face[1] = DXL_HIBYTE(DXL_LOWORD(target_pos_face));
			param_goal_face[2] = DXL_LOBYTE(DXL_HIWORD(target_pos_face));
			param_goal_face[3] = DXL_HIBYTE(DXL_HIWORD(target_pos_face));

			dxl_addparam_result = groupBulkWrite.addParam(dxl_face, goal_position_address, data_len_4byte, param_goal_face);
			if (!dxl_addparam_result) {
				std::cout << "[ID" << (int)dxl_face << "]groupBulkWrite addparam failed" << std::endl;
				return 0;
			}
			dxl_comm_result = groupBulkWrite.txPacket();
			if (dxl_comm_result != COMM_SUCCESS) {
				std::cout << packetHandler->getTxRxResult(dxl_comm_result) << std::endl;
			}

			int target_pos_left = 1050;

			uint8_t param_goal_left[4]; //Moves left motor
			param_goal_left[0] = DXL_LOBYTE(DXL_LOWORD(target_pos_left));
			param_goal_left[1] = DXL_HIBYTE(DXL_LOWORD(target_pos_left));
			param_goal_left[2] = DXL_LOBYTE(DXL_HIWORD(target_pos_left));
			param_goal_left[3] = DXL_HIBYTE(DXL_HIWORD(target_pos_left));

			dxl_addparam_result = groupBulkWrite.addParam(dxl_left, goal_position_address, data_len_4byte, param_goal_left);
			if (!dxl_addparam_result) {
				std::cout << "[ID" << (int)dxl_left << "]groupBulkWrite addparam failed" << std::endl;
				return 0;
			}
			dxl_comm_result = groupBulkWrite.txPacket();
			if (dxl_comm_result != COMM_SUCCESS) {
				std::cout << packetHandler->getTxRxResult(dxl_comm_result) << std::endl;
			}

			int target_pos_right = 1550;

			uint8_t param_goal_right[4]; //Moves right motor
			param_goal_right[0] = DXL_LOBYTE(DXL_LOWORD(target_pos_right));
			param_goal_right[1] = DXL_HIBYTE(DXL_LOWORD(target_pos_right));
			param_goal_right[2] = DXL_LOBYTE(DXL_HIWORD(target_pos_right));
			param_goal_right[3] = DXL_HIBYTE(DXL_HIWORD(target_pos_right));

			dxl_addparam_result = groupBulkWrite.addParam(dxl_right, goal_position_address, data_len_4byte, param_goal_right);
			if (!dxl_addparam_result) {
				std::cout << "[ID" << (int)dxl_right << "]groupBulkWrite addparam failed" << std::endl;
				return 0;
			}
			dxl_comm_result = groupBulkWrite.txPacket();
			if (dxl_comm_result != COMM_SUCCESS) {
				std::cout << packetHandler->getTxRxResult(dxl_comm_result) << std::endl;
			}

			groupBulkWrite.clearParam();
		}


		else if (command == "2") { //sends the face down 
			int face_down = 600;

			uint8_t param_goal_position[4];
			param_goal_position[0] = DXL_LOBYTE(DXL_LOWORD(face_down));
			param_goal_position[1] = DXL_HIBYTE(DXL_LOWORD(face_down));
			param_goal_position[2] = DXL_LOBYTE(DXL_HIWORD(face_down));
			param_goal_position[3] = DXL_HIBYTE(DXL_HIWORD(face_down));

			uint16_t goal_position_address = 116;
			dxl_addparam_result = groupBulkWrite.addParam(dxl_face, goal_position_address, data_len_4byte, param_goal_position);
			if (!dxl_addparam_result) {
				std::cout << "[ID:" << (int)dxl_face << "]groupBulkWrite addparam failed" << std::endl;
				return 0;
			}
			dxl_comm_result = groupBulkWrite.txPacket();
			if (dxl_comm_result != COMM_SUCCESS) {
				std::cout << packetHandler->getTxRxResult(dxl_comm_result) << std::endl;
			}
			groupBulkWrite.clearParam();
		}

			else if (command == "6") { //sends the robot left 
				uint8_t goal_position_address = 116;

				int goal_pos_base = 800;

				uint8_t param_goal_position[4];
				param_goal_position[0] = DXL_LOBYTE(DXL_LOWORD(goal_pos_base));
				param_goal_position[1] = DXL_HIBYTE(DXL_LOWORD(goal_pos_base));
				param_goal_position[2] = DXL_LOBYTE(DXL_HIWORD(goal_pos_base));
				param_goal_position[3] = DXL_HIBYTE(DXL_HIWORD(goal_pos_base));

				dxl_addparam_result = groupBulkWrite.addParam(dxl_base, goal_position_address, data_len_4byte, param_goal_position);
				if (!dxl_addparam_result) {
					std::cout << "[ID:" << (int)dxl_base << "]groupBulkWrite addparam failed" << std::endl;
					return 0;
				}
				dxl_comm_result = groupBulkWrite.txPacket();
				if (dxl_comm_result != COMM_SUCCESS) {
					std::cout << packetHandler->getTxRxResult(dxl_comm_result) << std::endl;
				}

				int target_pos_left = 500;
				uint8_t param_goal_left[4]; //Moves left motor
				param_goal_left[0] = DXL_LOBYTE(DXL_LOWORD(target_pos_left));
				param_goal_left[1] = DXL_HIBYTE(DXL_LOWORD(target_pos_left));
				param_goal_left[2] = DXL_LOBYTE(DXL_HIWORD(target_pos_left));
				param_goal_left[3] = DXL_HIBYTE(DXL_HIWORD(target_pos_left));

				dxl_addparam_result = groupBulkWrite.addParam(dxl_left, goal_position_address, data_len_4byte, param_goal_left);
				if (!dxl_addparam_result) {
					std::cout << "[ID" << (int)dxl_left << "]groupBulkWrite addparam failed" << std::endl;
					return 0;
				}
				dxl_comm_result = groupBulkWrite.txPacket();
				if (dxl_comm_result != COMM_SUCCESS) {
					std::cout << packetHandler->getTxRxResult(dxl_comm_result) << std::endl;
				}

				groupBulkWrite.clearParam();
			}

			else if (command == "4") { //sends the robot right
				uint8_t goal_position_address = 116;

				int goal_pos_base = 3200;

				uint8_t param_goal_position[4];
				param_goal_position[0] = DXL_LOBYTE(DXL_LOWORD(goal_pos_base));
				param_goal_position[1] = DXL_HIBYTE(DXL_LOWORD(goal_pos_base));
				param_goal_position[2] = DXL_LOBYTE(DXL_HIWORD(goal_pos_base));
				param_goal_position[3] = DXL_HIBYTE(DXL_HIWORD(goal_pos_base));

				dxl_addparam_result = groupBulkWrite.addParam(dxl_base, goal_position_address, data_len_4byte, param_goal_position);
				if (!dxl_addparam_result) {
					std::cout << "[ID:" << (int)dxl_base << "]groupBulkWrite addparam failed" << std::endl;
					return 0;
				}
				dxl_comm_result = groupBulkWrite.txPacket();
				if (dxl_comm_result != COMM_SUCCESS) {
					std::cout << packetHandler->getTxRxResult(dxl_comm_result) << std::endl;
				}

				int target_pos_right = 1000;
				uint8_t param_goal_right[4]; //Moves right motor
				param_goal_right[0] = DXL_LOBYTE(DXL_LOWORD(target_pos_right));
				param_goal_right[1] = DXL_HIBYTE(DXL_LOWORD(target_pos_right));
				param_goal_right[2] = DXL_LOBYTE(DXL_HIWORD(target_pos_right));
				param_goal_right[3] = DXL_HIBYTE(DXL_HIWORD(target_pos_right));

				dxl_addparam_result = groupBulkWrite.addParam(dxl_right, goal_position_address, data_len_4byte, param_goal_right);
				if (!dxl_addparam_result) {
					std::cout << "[ID" << (int)dxl_right << "]groupBulkWrite addparam failed" << std::endl;
					return 0;
				}
				dxl_comm_result = groupBulkWrite.txPacket();
				if (dxl_comm_result != COMM_SUCCESS) {
					std::cout << packetHandler->getTxRxResult(dxl_comm_result) << std::endl;
				}

				groupBulkWrite.clearParam();
				}
			else if (command == "9") { //moves left motor down
				int target_pos_left = 500;
				uint8_t goal_position_address = 116;
				uint8_t param_goal_left[4]; //Moves right motor
				param_goal_left[0] = DXL_LOBYTE(DXL_LOWORD(target_pos_left));
				param_goal_left[1] = DXL_HIBYTE(DXL_LOWORD(target_pos_left));
				param_goal_left[2] = DXL_LOBYTE(DXL_HIWORD(target_pos_left));
				param_goal_left[3] = DXL_HIBYTE(DXL_HIWORD(target_pos_left));

				dxl_addparam_result = groupBulkWrite.addParam(dxl_left, goal_position_address, data_len_4byte, param_goal_left);
				if (!dxl_addparam_result) {
					std::cout << "[ID" << (int)dxl_left << "]groupBulkWrite addparam failed" << std::endl;
					return 0;
				}
				dxl_comm_result = groupBulkWrite.txPacket();
				if (dxl_comm_result != COMM_SUCCESS) {
					std::cout << packetHandler->getTxRxResult(dxl_comm_result) << std::endl;
				}

				groupBulkWrite.clearParam();
			}

			else if (command == "7") { //moves right motor down
				int target_pos_right = 1000;
				uint8_t goal_position_address = 116;
				uint8_t param_goal_right[4]; //Moves left motor
				param_goal_right[0] = DXL_LOBYTE(DXL_LOWORD(target_pos_right));
				param_goal_right[1] = DXL_HIBYTE(DXL_LOWORD(target_pos_right));
				param_goal_right[2] = DXL_LOBYTE(DXL_HIWORD(target_pos_right));
				param_goal_right[3] = DXL_HIBYTE(DXL_HIWORD(target_pos_right));

				dxl_addparam_result = groupBulkWrite.addParam(dxl_right, goal_position_address, data_len_4byte, param_goal_right);
				if (!dxl_addparam_result) {
					std::cout << "[ID" << (int)dxl_right << "]groupBulkWrite addparam failed" << std::endl;
					return 0;
				}
				dxl_comm_result = groupBulkWrite.txPacket();
				if (dxl_comm_result != COMM_SUCCESS) {
					std::cout << packetHandler->getTxRxResult(dxl_comm_result) << std::endl;
				}

				groupBulkWrite.clearParam();
			}

			else if (command == "0") { //Manual exit 
				data = 0;
				packetHandler->write1ByteTxRx(portHandler, dxl_base, torque_address, data);
				packetHandler->write1ByteTxRx(portHandler, dxl_face, torque_address, data);
				packetHandler->write1ByteTxRx(portHandler, dxl_left, torque_address, data);
				packetHandler->write1ByteTxRx(portHandler, dxl_right, torque_address, data);
				portHandler->closePort();
				return 0;
			}
	}
	data = 0;
	packetHandler->write1ByteTxRx(portHandler, dxl_base, torque_address, data);
	packetHandler->write1ByteTxRx(portHandler, dxl_face, torque_address, data);
	packetHandler->write1ByteTxRx(portHandler, dxl_left, torque_address, data);
	packetHandler->write1ByteTxRx(portHandler, dxl_right, torque_address, data);
	portHandler->closePort();

	return 0;

}
