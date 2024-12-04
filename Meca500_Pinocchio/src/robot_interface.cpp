#include <iostream>
#include "Meca500/robot_interface.h"
#include "socket_comm.cpp"
#include <iostream>
#include <chrono>
#include <thread>
#include <string>
#include <cstring>
#include <arpa/inet.h>
#include <unistd.h>

Socket_connection socket_connection;

Meca500connection::Meca500connection()
{

}

Meca500connection::~Meca500connection()
{

}

/*
int extract_code(const char *message)
{
	int code = 0;
	char *first_bracket = nullptr;
	char *second_bracket = nullptr;
	do
	{
		first_bracket = strchr(message, '[');
		second_bracket = strchr(message, ']');
		if ((int)(second_bracket - first_bracket) == 5)
		{
			code = 1000 * (first_bracket[1]-'0') +
			100 * (first_bracket[2]-'0') +
			10 * (first_bracket[3]-'0') +
			1 * (first_bracket[4]-'0');
		}
	} while (code == 0 || first_bracket == nullptr || second_bracket == nullptr);
	return code;
}
*/

int Meca500connection::connect_robot(const char* address)
{
	int result = 0;
	// Connect with the good port
	result = socket_connection.TcpIp_connect(address);
	//socket_connection.receive_data();
	
	if (result == -1)
	{
		std::cerr << "Robot connection failed on network error.\n"<< std::endl;
		return -1;
	}
	
	std::cout << "Connected to the Meca500 at addresss " << address << '\n';
	return 0;
	
}
	
int Meca500connection::disconnect_robot()
{
    socket_connection.TcpIp_disconnect();
    return 0;
}
	
	
int Meca500connection::activate_robot()
{
	const char* msg = "ActivateRobot\n";
	socket_connection.TcpIp_send(msg);
	
	/*
	int result = wait_for_return_code(2000, robot);
	if (result != 0)
	{
    	return -1;
	}
	*/
	return 0;
	
}




int Meca500connection::deactivate_robot()
{
	const char *msg = "DeactivateRobot\n";
	socket_connection.TcpIp_send(msg);
	/*
	int result = wait_for_return_code(2004, robot);
	if (result != 0)
	{
		return -1;
	}
	*/
	return 0;
}

int Meca500connection::home()
{
    const char *msg = "Home\n";
    int result = socket_connection.TcpIp_send(msg);
    /*
    if (result != 0)
    {
        return -1;
    }
    result = wait_for_return_code(2002, robot);
    if (result != 0)
    {
        return -1;
    }
    */
    return 0;
}

int Meca500connection::move_joints( float joints[6])
{
	char msg[100] = {0};
	sprintf(msg, "MoveJoints(%10.4f, %10.4f, %10.4f, %10.4f, %10.4f, %10.4f)\n", joints[0], joints[1], joints[2], joints[3], joints[4], joints[5]);
	
	int result = socket_connection.TcpIp_send(msg);
	if (result != 0)
	{
		return -1;
	}
	char *msg1 = "SetCheckpoint(1)\n";
	result = socket_connection.TcpIp_send(msg1);
	
	
	return 0;
}

int Meca500connection::move_pose(float euler[6])
{
	char msg[100] = {0};
	sprintf(msg, "MovePose(%10.4f, %10.4f, %10.4f, %10.4f, %10.4f, %10.4f)\n", euler[0], euler[1], euler[2], euler[3], euler[4], euler[5]);
	
    int result = socket_connection.TcpIp_send(msg);
    if (result != 0)
    {
        return -1;
    }
    
    return 0;
}

int Meca500connection::Joint_Speed(float J_Speed[6])
{
	char msg[100] = {0};
	sprintf(msg, "MoveJointsVel(%10.4f, %10.4f, %10.4f, %10.4f, %10.4f, %10.4f)\n", J_Speed[0], J_Speed[1], J_Speed[2], J_Speed[3], J_Speed[4], J_Speed[5]);
	
    int result = socket_connection.TcpIp_send(msg);
    
    
    if (result != 0)
    {
        return -1;
    }
    
    return 0;
}

int Meca500connection::Stop_Joint()
{
    float J_Speed[6] = {0};
    return Meca500connection::Joint_Speed(J_Speed);
}

//=============================================================================
//
int Meca500connection::move_to_zero()
{
    float joints[6] = {0};
    return Meca500connection::move_joints(joints);
}

//=============================================================================
//
int Meca500connection::move_to_shipping()
{
    float joints[6] = {0, -60, 60, 0, 0, 0};
    return Meca500connection::move_joints(joints);
}

bool Meca500connection::Set_Joint_Vel(float t)
{
	bool success;
	if(t<0 || t>100){
		success = false;
		std::cout << "Robot set Joint velocity must be between 0 to 100\n"<< std::endl;
	}else{
		char msg[100] = {0};
		std::cout << "Robot set velocity Timeout Done\n"<< std::endl;
		printf("SetJointVel(%1.3f)\n", t);
		sprintf(msg, "SetJointVel(%1.3f)\n", t);
		int result = socket_connection.TcpIp_send(msg);
		if (result != 0)
		{
		    return -1;
		}
		success = true;
	}
    return success;
}

bool Meca500connection::Set_Velocity_TimeOut(float t)
{
	bool success;
	if(t<0.001 || t>1){
		success = false;
		std::cout << "Robot set velocity Timeout must be between 0.001 to 1.\n"<< std::endl;
	}else{
		char msg[100] = {0};
		std::cout << "Robot set velocity Timeout Done\n"<< std::endl;
		printf("SetVelTimeout(%1.3f)\n", t);
		sprintf(msg, "SetVelTimeout(%1.3f)\n", t);
		int result = socket_connection.TcpIp_send(msg);
		if (result != 0)
		{
		    return -1;
		}
		success = true;
	}
    return success;
}

void Meca500connection::Delay_Motion(float t)
{
	char msg[100] = {0};
	sprintf(msg, "Delay(%1.3f)\n", t);
    return;
}


