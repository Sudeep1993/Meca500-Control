/*

#include <iostream>
#include <thread>
#include <chrono>
#include "Meca500/robot_interface.h"
#include <boost/asio.hpp>



int main(int argc, char** argv)
{
    Socket_connection socket_connection;
    int result = socket_connection.TcpIp_connect("192.168.0.100");
    if (result !=0)
    {
        return -1;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    socket_connection.TcpIp_disconnect();
    
    return 0;
}

*/
#include <iostream>
#include <thread>
#include <chrono>
#include "Meca500/robot_interface.h"


int main(int argc, char** argv)
{
    Meca500connection Meca500;
    int result = Meca500.connect_robot("192.168.0.100");
    if (result !=0)
    {
        return -1;
    }
    std::cout << "Connected to the Meca500 \n" <<  std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    
	Meca500.activate_robot();
    std::cout << "Activated Meca500 \n";
	std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    Meca500.home();
    std::cout << "Travelling to Home position \n" << std::endl;
	std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    
    float pose_1[6] = { 10,15,20,25,30,35 };
    float pose_2[6] = { 15,-34,57,18,35,78 };
    float pose_3[6] = { 20,-50,89,92,0,120 };
    float pose_4[6] = { 100,0,50,120,00,110 };
    
    Meca500.move_pose(pose_1);
    std::cout << "Travelling to Pose_1 \n" << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    
    bool res = Meca500.Set_Joint_Vel(10);
    
    Meca500.move_pose(pose_2);
    std::cout << "Travelling to Pose_2 \n" << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    
    Meca500.move_pose(pose_3);
    std::cout << "Travelling to Pose_3 \n" << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        
    Meca500.move_pose(pose_4);
    std::cout << "Travelling to Pose_4 \n" << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    
    Meca500.move_pose(pose_1);
    std::cout << "Back to Pose_1 \n" << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(10000));
    
    res = Meca500.Set_Joint_Vel(25);
    
    float Joint_2[6] = { 0,0,0,0,0,0 };
    Meca500.move_joints(Joint_2);
    std::cout << "Travelling to Home position \n" << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    
    
    Meca500.deactivate_robot();
    std::cout << "Deactivated to the Meca500 at addresss \n" << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    Meca500.disconnect_robot();
    std::cout << "DisConnected to the Meca500 at addresss \n" << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    
    
    return 0;
}
    

