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
	
    float Joint_2[6] = { 0,0,0,0,0,0 };
    Meca500.move_joints(Joint_2);
    std::cout << "Travelling to Zero position \n" << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    
    bool Success = Meca500.Set_Velocity_TimeOut(0.01);
    //std::cerr << Success<< std::endl;
    if(!Success){
    	
		Meca500.deactivate_robot();
		std::cout << "Deactivated to the Meca500 at addresss \n" << std::endl;
		std::this_thread::sleep_for(std::chrono::milliseconds(1000));
		Meca500.disconnect_robot();
		std::cout << "DisConnected to the Meca500 at addresss \n" << std::endl;
		std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    	return 0;
    }
    
    
    float vel_1[6] = { 60,0,0,0,0,0 };
    //float pose_2[6] = { 150,0,150,180,00,180 };
    
    std::cout << "Travelling With a Speed 1 \n" << std::endl;
    int ii=1;
    while(true){
    	Meca500.Joint_Speed(vel_1);
    	//std::this_thread::sleep_for(std::chrono::milliseconds(10));
    	ii=ii+1;
    	if(ii>5){
    		break;
    	}
    }
    
    vel_1[0] = -30;
    //float pose_2[6] = { 150,0,150,180,00,180 };
    
    std::cout << "Travelling With a Speed 2 \n" << std::endl;
    ii = 1;
    while(true){
    	Meca500.Joint_Speed(vel_1);
    	//Meca500.Delay_Motion(0.005);
    	//std::this_thread::sleep_for(std::chrono::milliseconds(10));
    	ii=ii+1;
    	if(ii>2){
    		break;
    	}
    }
    
    //Meca500.Stop_Joint();
    
    /*
    float Joint_2[6] = { 0,0,0,0,0,0 };
    Meca500.move_joints(Joint_2);
    std::cout << "Travelling to Home position \n" << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    */
    
    Meca500.deactivate_robot();
    std::cout << "Deactivated to the Meca500 at addresss \n" << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    Meca500.disconnect_robot();
    std::cout << "DisConnected to the Meca500 at addresss \n" << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    
    
    return 0;
}
    

