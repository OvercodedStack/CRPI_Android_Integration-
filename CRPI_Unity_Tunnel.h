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
	//int start_CRPI_SRV(string input_IP_ADDR, string input_PORT);
	int start_CRPI_SRV();
	void start_CRPI_encoding();
	void send_crpi_msg(robotAxes unity_pose);
	robotAxes string_converter(string msg);
	void recieve_message_android();
	void recieve_message_vicon(); 
	void send_message_android(string message);
	void send_message_vicon(string message);
	void close_client_vicon();
	void close_client_android();
	void act_changer_unity(int changer);
	void port_scanner(string IP_name_in, int start_port, int end_port);
 

	void send_gripper_cmd(float vals);
	void send_DO_cmds(bool ary_in[4]); 
	

	//Server and Client TCP Settings
	WSADATA vicon_cli;
	WSADATA android_cli;

	SOCKET vicon_socket;
	SOCKET android_socket; 

	int iResult_V;
	int iResult_A; 

	int recvbuflen_V;
	int recvbuflen_A; 

	string action_TCP_Vicon;
	string action_TCP_Android; 
	string adr_IP_vicon;
	string port_vicon;
	string adr_IP_android;
	string port_android; 

	const char *sendbuf_vicon; 
	const char *sendbuf_android;
	bool activate_shutdown = false; 

	int timeout_interval; 

	string list_IPs[128]; //Amount of stored robots in IP scanning
	float digital_data_in[8]; //Adjust as needed
	bool do_cmd_list[4];
	float gripper_ratio; 
	int robot_id, old_robot_id;

	//Additional blub for changing robots
	int override_robot_id; //Numbers correspond to a robot type
	int change_robots; 


	//CRPI Settings
	robotAxes pose_msg;
	float open_grip;
	CrpiRobot <CrpiUniversal> *arm;


};

//Single Client setup 
/*Server_CRPI(); 
	//int start_CRPI_SRV(string input_IP_ADDR, string input_PORT);
	int start_CRPI_SRV();
	void start_CRPI_encoding();
	void send_crpi_msg(robotAxes unity_pose);
	robotAxes string_converter(string msg);
	void recieve_message();
	void send_message(string message);
	void close_client();
	void act_changer_unity(int changer);
	void port_scanner(string IP_name_in, int start_port, int end_port);
	void access_shared_space();

	void send_gripper_cmd(float vals);
	void send_DO_cmds(bool ary_in[4]); 
	

	//Server and Client TCP Settings
	WSADATA wsaData;
	SOCKET ConnectSocket;
	int iResult;
	int recvbuflen;
	string action_string_TCP;
	string adr_IP;
	string port_num;
	const char *sendbuf;*/

/*
//Client to collect data from
class Client_CRPI {
public: 
	int start_client();
	void access_shared_space();
	void recieve_message();
	void close_client();

	//Server and Client TCP Settings
	WSADATA wsaData;
	SOCKET ConnectSocket;
	int iResult;
	int recvbuflen;
	string action_string_TCP;
	string adr_IP;
	string port_num;
	const char *sendbuf;
	string obtained_msg;

};
*/

#endif