#ifndef CRPI_ANDROID_H
#define CRPI_ANDROID_H

#pragma once
#include "crpi_robot.h"
#include "crpi_universal.h"
#include "ulapi.h"

using namespace crpi_robot;

class Server_CRPI {
public: 
	//Functions to be used as a means to perform the required actions. 
	Server_CRPI(); 
	int start_CRPI_SRV(string input_IP_ADDR, string input_PORT);
	void start_CRPI_encoding();
	void send_crpi_msg(robotAxes unity_pose);
	robotAxes string_converter(string msg);
	void recieve_message();
	void send_message(string message);
	void close_client();
	void act_changer_unity(int changer);
	void port_scanner(string IP_name_in, int start_port, int end_port);

	

	//Server and Client TCP Settings
	WSADATA wsaData;
	SOCKET ConnectSocket;
	int iResult;
	int recvbuflen;
	string action_string_TCP;
	string adr_IP;
	string port_num;
	const char *sendbuf;
	string list_IPs[128]; //Amount of stored robots in IP scanning
	float digital_data_in[8]; //Adjust as needed


	//CRPI Settings
	robotAxes pose_msg;
	float open_grip;
	CrpiRobot <CrpiUniversal> *arm;


};


#endif