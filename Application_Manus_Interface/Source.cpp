///////////////////////////////////////////////////////////////////////////////
//
//  Original System: Android CRPI Interaction
//  Subsystem:       Human-Robot Interaction
//  Workfile:        Android UI Tests
//  Revision:        1.0 6/7/19
//  Author:          Esteban Segarra Martinez
//
//  Description
//  ===========
//  Integrated TPC client for communicating between Unity and UR5 standalone using positional coordinates based on
//	conversions between unity and realworld calculations. Scaling and rotation applied. 
//
//	Custom string unity provided by unity is phrased by this program as well. 
//
//	Quik Start instructions
//	============
//
//	Run the Manus_interface.exe program. The program will start and ask for the amount of cycles you would like to run the 
//	program for. Each cycle has a 2 second delay and is nesseary to avoid overloading the UR5 controller with too many pose 
//	commands. The program also depends on having information being delivered at a constant rate greater than 1 second
//	in order to avoid overflowing the recv buffer (too much garbage data can collect). 
///////////////////////////////////////////////////////////////////////////////
 
#include <winsock2.h>
#include <ws2tcpip.h>
#include <iostream>
#include <time.h>
#include "crpi_robot.h"
#include "crpi_universal.h"
#include "ulapi.h"
#include "crpi_abb.h"
#include <string>
#include "CRPI_Android_interface.h"
#include "DataStreamClient.h"
#include <crtdbg.h>

// Need to link with Ws2_32.lib
#pragma comment (lib, "Ws2_32.lib")


#define DEFAULT_BUFLEN 256
#define DEFAULT_PORT "27000"
 
using namespace std;
using namespace crpi_robot;

struct addrinfo *result = NULL,
	*ptr = NULL,
	hints;

//Dummy constructor 
Server_CRPI::Server_CRPI() {
}


//General setup and running of the TCP server connection 
int Server_CRPI::start_CRPI_SRV(string input_IP_ADDR, string input_PORT) {
	ConnectSocket = INVALID_SOCKET;
	sendbuf = "this is a test";
	recvbuflen = DEFAULT_BUFLEN;

	cout << "Starting Connection." << endl;

	//CRPI Robot UR5
	CrpiRobot<CrpiUniversal> arm("universal_ur5.xml");
	//Start the CRPI Comms
	start_CRPI_encoding();

	// Initialize Winsock
	iResult = WSAStartup(MAKEWORD(2, 2), &wsaData);
	if (iResult != 0) {
		printf("WSAStartup failed with error: %d\n", iResult);
		return 1;
	}

	ZeroMemory(&hints, sizeof(hints));
	hints.ai_family = AF_INET;
	hints.ai_socktype = SOCK_STREAM;
	hints.ai_protocol = IPPROTO_TCP;

	// Resolve the server address and port
	//iResult = getaddrinfo("127.0.0.1", DEFAULT_PORT, &hints, &result);
	iResult = getaddrinfo(input_IP_ADDR.c_str(), input_PORT.c_str(), &hints, &result);

	if (iResult != 0) {
		printf("getaddrinfo failed with error: %d\n", iResult);
		WSACleanup();
		return 2;
	}

	// Attempt to connect to an address until one succeeds
	for (ptr = result; ptr != NULL; ptr = ptr->ai_next) {

		// Create a SOCKET for connecting to server
		ConnectSocket = socket(ptr->ai_family, ptr->ai_socktype,
			ptr->ai_protocol);
		if (ConnectSocket == INVALID_SOCKET) {
			printf("socket failed with error: %ld\n", WSAGetLastError());
			WSACleanup();
			return 3;
		}

		// Connect to server.
		iResult = connect(ConnectSocket, ptr->ai_addr, (int)ptr->ai_addrlen);
		if (iResult == SOCKET_ERROR) {
			closesocket(ConnectSocket);
			ConnectSocket = INVALID_SOCKET;
			continue;
		}
		break;
	}

	freeaddrinfo(result);

	if (ConnectSocket == INVALID_SOCKET) {
		printf("Unable to connect to server!\n");
		WSACleanup();
		return 4;
	}

	cout << "Initialization Done. Starting Client reception..." << endl;
	recieve_message();

	// Receive until the peer closes the connection
	close_client();
		
	return 0; 
}

//Start the CRPI connection 
void Server_CRPI::start_CRPI_encoding() {
	cout << "Starting Robot..." << endl;
	arm->SetAngleUnits("degree");
	arm->SetLengthUnits("mm");
	arm->SetRelativeSpeed(0.1);
	arm->Couple("griper_parallel");
	cout << "Done." << endl;

	//Double Comparison check constant
	const double TOLERANCE = 0.05;
}


//Function to determine and move UR5 to a specified location
void Server_CRPI::send_crpi_msg(robotAxes unity_pose) {
	robotAxes address;
	arm->GetRobotAxes(&address);

	//Avoid sleep statements here - Will Unbalance TPC client and fill with garbage. 
	//Read data_in. Conditional to test pose accuracy/send msg. 
	if (arm->MoveToAxisTarget(unity_pose) == CANON_SUCCESS)
	{
		//address.print();
		cout << endl << endl;
		cout << "Success" << endl;
	}
	else
	{
		cout << "Failure" << endl;
	}
	

	if (digital_data_in[1] == 1) {
		arm->SetRobotDO(8, true);		
	}
	else {
		arm->SetRobotDO(8, false);
	}



	//Avoid using gripper at the moment - Requires most likely some type of threading in order to use appropiately.
	//Otherwise it will freeze the whole robot from doing anything. 

	//if (open_grip >= 0.5F ) {
	//	if (arm.SetRobotDO(0, 1) == CANON_SUCCESS) {
	//		cout << "Gripped" << endl;
	//	}
	//}
	//else {
	//	if (arm.SetRobotDO(0, 0) == CANON_SUCCESS) {
	//		cout << "Un-Gripped" << endl;
	//	}//Open the gripper on UR
	//}


	cout << "Pose movement completed." << endl;
}


//Custom string phraser from incoming message from Unity
//I'm not well aware of standards on messages, so I made my own. 
//Apologies if this is gibberish :(
robotAxes Server_CRPI::string_converter(string msg) {
	//  UR5_pos:-42.58, -43.69, -99.57, 233.2, -89.66, -47.09;Gripper:0; 
	// array_of_pos[6] = {x,y,z,xrot,yrot,zrot};
	// Digital_data_in = {gripper, DO_1, DO_2, etc };


	// {rot0,rot1,rot2,rot3,rot4,rot5}
	double array_of_pos[6], gripper;
	robotAxes unity_pose = robotAxes(6);
	bool action_cmd = false; 
	string temp_msg;
	int ary_count = 0;
	bool chk_DO = false;


	if (msg.length() > 1) {

		//Categorized phraser for the string input by unity.
		for (int i = 0; i < msg.length(); i++) {
			if (msg[i] == '>') {
				i++;
				cout << endl;
				while (true) {
					if (msg[i] == ',') {
						array_of_pos[ary_count] = strtod((temp_msg).c_str(), 0);
						ary_count++;
						temp_msg = "";
					}
					else if (msg[i] == ';') {
						array_of_pos[ary_count] = strtod((temp_msg).c_str(), 0);
						temp_msg = "";
						ary_count++;
						break;
					}
					else {
						temp_msg += msg[i];
					}
					i++;
				}
			}

			ary_count = 0; 
			if (msg[i] == ',') {
				i++;
				while (true) {
					if (msg[i] == ';') {
						open_grip = strtod((temp_msg).c_str(), 0);
						chk_DO = true;
						temp_msg = "";
						break;
					}
					else {
						temp_msg += msg[i];
						digital_data_in[ary_count] = atoi(temp_msg.c_str());
						ary_count++; 
					}
					i++;
				}
			}

			/*
			if (msg[i] == ':') {
				i++;
				while (true) {
					if (msg[i] == ';') {
						open_grip = strtod((temp_msg).c_str(), 0);
						chk_grip = true;
						temp_msg = "";
						break;
					}
					else {
						temp_msg += msg[i];
					}
					i++;
				}
			}
			*/

			//If the array is already full, break; 
			if (chk_DO) {
				break;
			}
		}

		//Phrase inbound angles into the robotAxes 
		for (int i = 0; i < 6; i++) {
			cout << "Revieved angles: " << endl;
			cout << array_of_pos[i] << ", " << endl;
			unity_pose.axis[i] = array_of_pos[i];
		}
	}
	else {
		cout << "Message is null." << endl;
	}

	//Assign phrase final pose position. 
	////unity_pose.x	= array_of_pos[0];
	////unity_pose.xrot = array_of_pos[1];
	////unity_pose.y	= array_of_pos[2];
	////unity_pose.yrot = array_of_pos[3];
	////unity_pose.z	= array_of_pos[4];
	////unity_pose.zrot = array_of_pos[5];

	return unity_pose;
}

//TPC Reception message function
void Server_CRPI::recieve_message() {
	int close_sess = 0, cycle_counter = 0, CYCLE_RUNS = 10;
	//cout << "Enter amount of cycles the program should run: " << endl;
	//cin >> CYCLE_RUNS;
	do {

		//Highly sensitive stuff
		char recvbuf[DEFAULT_BUFLEN];
		action_string_TCP = "";
		iResult = recv(ConnectSocket, recvbuf, 256, 0);

		if (iResult > 0) {
			printf("Bytes received: %d\n", iResult);
		}
		else if (iResult == 0)
			printf("Connection closed\n");
		else
			printf("recv failed with error: %d\n", WSAGetLastError());


		//Bytes recieved - Phrase TPC input
		for (int i = 0; i < iResult; i++) {
			action_string_TCP += recvbuf[i];
			//cout << recvbuf[i];
		}

		cout << "Recieved message: " << action_string_TCP << endl;


		//Send a CRPI Message given the correct string
		pose_msg = string_converter(action_string_TCP);
		send_crpi_msg(pose_msg);

		//Put the brakes on TPC client
		for (int x = 0; x < 256; x++)
			recvbuf[x] = '\0';

		Sleep(500);
		cycle_counter++;
		cout << "Cycle " << cycle_counter << endl;
		cout << endl;

	} while (iResult > 0 && CYCLE_RUNS >= cycle_counter);
}

//Sends a message to the connected TCP server
void Server_CRPI::send_message(string message) {
	// Send an initial buffer
	iResult = send(ConnectSocket, sendbuf, (int)strlen(sendbuf), 0);
	if (iResult == SOCKET_ERROR) {
		printf("send failed with error: %d\n", WSAGetLastError());
		closesocket(ConnectSocket);
		WSACleanup();
		//--return 1;
	}
	printf("Bytes Sent: %ld\n", iResult);
}

//Closes the TCP server client to cleanly exit and liberate the port
void Server_CRPI::close_client() {
	iResult = shutdown(ConnectSocket, SD_SEND);
	if (iResult == SOCKET_ERROR) {
		printf("Shutdown Failure");
	}

	// cleanup
	closesocket(ConnectSocket);
	WSACleanup();
	cout << "Closing Client" << endl;
	//return 0;
}


//Depending on the input provided, the server will switch from 
//different robot arms using this function
void Server_CRPI::act_changer_unity(int changer){
 	close_client(); 
	if (changer == 0) {
		CrpiRobot<CrpiUniversal> arm("universal_ur5.xml");
	}
	else if (changer == 1) {
		CrpiRobot<CrpiUniversal> arm("universal_ur10_left.xml");
	}
	else if (changer == 2) {
		CrpiRobot<CrpiUniversal> arm("universal_ur10_right.xml");

	}
	else if (changer == 3) {
		CrpiRobot<CrpiUniversal> arm("abb_irb14000_left.xml");
	}
	else if (changer == 4) {
		CrpiRobot<CrpiUniversal> arm("abb_irb14000_right.xml");
	}
	else {
		printf("No suitable robot at the moment");
		return; 
	}
	start_CRPI_SRV(adr_IP, port_num);
}

int __cdecl main(int argc, char **argv) {
	Server_CRPI server; 
	server.start_CRPI_SRV("127.0.0.1","27000");
	cout << "Closing client-server." << endl;
	return 0;
	
}




/*Deprecated
void connect_vicon() {
ViconDataStreamSDK::CPP::Client MyClient;
Output_Connect Output = MyClient.Connect("localhost");
}


//Abandoned.
void Server_CRPI::port_scanner(string IP_name_in, int start_port, int end_port) {
string IP[20];
int start = start_port;
int end = end_port;
int err;
int nret;
SOCKET sock;
SOCKADDR_IN Info;
WSADATA wsadata;
int counter = 0;
err = WSAStartup(MAKEWORD(2, 2), &wsadata);
if (err != 0)
{
cout << "Error with winsock. Will Now Exit." << endl;
cin.get();
return;
}

while (start < end)
{
sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);

Info.sin_family = AF_INET;
Info.sin_port = htons(start);
nret = connect(sock, NULL, NULL);
// error is for line above
if (nret != SOCKET_ERROR)
{
cout << "Port " << start << " - OPEN! " << endl;
list_IPs[counter] = start;
counter++;
}
start++;
closesocket(sock);
}
cout << endl << "Finished With Scan..." << endl;
cin.get();
}
*/
