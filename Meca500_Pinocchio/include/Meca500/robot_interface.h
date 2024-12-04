#ifndef MECA500_H
#define MECA500_H

#include <iostream>

#include "socket_comm.h"

class Meca500connection {

public:
	Meca500connection();
	~Meca500connection();
	
	int connect_robot(const char* address);
	
	int disconnect_robot();
	
	int activate_robot();
	
	int deactivate_robot();
	
	int home();
	
	int move_joints( float joints[6]);
	
	int move_pose(float euler[6]);
	
	int move_to_zero();
	
	int move_to_shipping();
	
	int Joint_Speed(float J_Speed[6]);
	
	int Stop_Joint();
	
	bool Set_Velocity_TimeOut(float t);
	
	void Delay_Motion(float t);
	
	bool Set_Joint_Vel(float t);

private:
    bool success_ = false;
    
};
#endif //MECA500_H
