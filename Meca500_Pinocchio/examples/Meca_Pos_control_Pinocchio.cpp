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
    
    float pose_1[6] = { 150,0,100,180,0,180 };
    float pose_2[6] = { 150,0,150,180,00,180 };
    
    Meca500.move_pose(pose_1);
    std::cout << "Travelling to Pose_1 \n" << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(5000));
    
    Meca500.move_pose(pose_2);
    std::cout << "Travelling to Pose_2 \n" << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(5000));
    
    
    float Joint_1[6] = { 20,30,0,0,0,0 };
    Meca500.move_joints(Joint_1);
    std::cout << "Travelling to Join_1 \n" << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(5000));
    
    Meca500.move_to_zero();
    std::cout << "Travelling to zero \n" << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(5000));
    Meca500.move_to_shipping();
    std::cout << "Travelling to Shipping \n" << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(5000));
    
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
    

