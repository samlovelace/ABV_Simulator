#include "Controller.h"

Controller::Controller()
{
	mDesPos << 0.0, 0.0, 0.0;
	mActPos << 0.0, 0.0, 0.0;
	mDesVel << 0.0, 0.0, 0.0;
	mActVel << 0.0, 0.0, 0.0;
	mActErrPos << 0.0, 0.0, 0.0;
	mActErrVel << 0.0, 0.0, 0.0;
	prev_act_er_pos << 0.0, 0.0, 0.0;
	prev_act_er_vel << 0.0, 0.0, 0.0;
	thrust_dir << 0, 0, 0;
	thrust_dir_pre << 0, 0, 0;
	thrust_com << 0, 0, 0, 0, 0, 0, 0, 0;
	thrust_com_pre << 0, 0, 0, 0, 0, 0, 0, 0;

	Pgain << 10.0, 10.0, 10.0;
	Dgain << 15.0, 15.0, 18.0;
	
	angErPre = 0.0;
	cont = 1; 
	//Ks_main = Ks_main_in;
	//Ks_main_local << 0.0, 0.0, 0.0;
	//Kc_min = Kc_min_in;
	//max_er_pos = max_er_pos_in;
	//max_er_vel = max_er_vel_in;
	///min_durs = min_durs_in;
	
	torquearm = 0.1235;
	max_thrusters = 2; 
	//force4 = force4_in;
	//torquearm = torquearm_in;
	//main_freq = main_freq_in;
	/*Pgain[0] = Pgain_x; 
	Pgain[1] = Pgain_y; 
	Pgain[2] = Pgain_yaw; 
	Dgain[0] = Dgain_x; 
	Dgain[1] = Dgain_y; 
	Dgain[2] = Dgain_yaw; */
	
	comp << 1, -1, 0, 0, 0, 0, 1, 1, -1, -1, 1, 1, -1, -1, 0, 0, 0, 0, 1, 1, 1, -1, 1, -1, -1, -1, 0,
			0, 0, 1, -1, 0, 0, 1, -1, 1, -1, 0, 0, 0, 0, 1, 1, -1, -1, 1, 1, -1, 1, -1, 1, -1, -1, 0,
			0, 0, 0, 0, 1, -1, 0, 0, 0, 0, 1, -1, 1, -1, 1, -1, 1, -1, 1, -1, 1, 1, -1, -1, 1, -1, 0;

	if (cont == 0) // Hard limit cycle is used as controller
	{
		//Kc_min calculation based on thruster forces 
		Kc_min[0] = std::min(force1, force2) / vehiclemass;
		Kc_min[1] = Kc_min[0];
		Kc_min[2] = (2*std::min(force1, 2*force2) * torquearm) / inertia;
		

		/*std::cout << "Kc_min X,Y: " << Kc_min[0] << std::endl; 
		std::cout << "Kc_min Z: " << Kc_min[2] << std::endl; 
		*/
		for (int n = 0; n < 3; n++)
		{
			er_pos_thresh[n] = max_er_pos[n] - pow(max_er_vel[n], 2) / (2 * Kc_min[n]);
			eps[n] = max_er_pos[n] - er_pos_thresh[n];
			T_hard[n] = 4.0 * (Kc_min[n] * er_pos_thresh[n] + pow(max_er_vel[n], 2)) / (max_er_vel[n] * Kc_min[n]);
		}

		/*std::cout << "max_er_pos:" << max_er_pos[2] << std::endl;
		std::cout << "er_pos_thresh:" << er_pos_thresh[2] << std::endl; 
		std::cout << "eps:" << eps[2] << std::endl;*/
	}
}

Controller::~Controller()
{
}

void Controller::PrintVals()
{
	std::cout << "Mass: " << vehiclemass << std::endl;
	std::cout << "Inertia: " << inertia << std::endl; 
	std::cout << "Force1: " << force1 << std::endl; 
	std::cout << "Force2: " << force2 << std::endl; 
	std::cout << "MaxThrs: " << max_thrusters << std::endl; 
	std::cout << "min fire: " << min_fire << std::endl;
	std::cout << "min pause: " << min_pause << std::endl;

	std::cout << "max err_pos:" << max_er_pos << std::endl; 
	std::cout << "max err_vel:" << max_er_vel << std::endl;
	std::cout << "prop gains: " << Pgain << std::endl; 
	std::cout << "deriv gains: " << Dgain << std::endl; 
	//std::cout << "max err_pos_manip:" << max_er_pos_manip << std::endl;
	//std::cout << "max err_vel_manip:" << max_er_vel_manip << std::endl;
}


void Controller::set_thrust_com(std::string thrust_com_in)
{
	thrust_com_s = thrust_com_in; 
}

std::string Controller::get_thrust_com()
{
	return thrust_com_s; 
}

Eigen::Matrix<int, 8, 1> Controller::get_thrust_com_cont() 
{ 
	return thrust_com; 
}

Eigen::Vector3i Controller::get_thrust_dir()
{
	return thrust_dir; 
}

void Controller::set_values(Eigen::Vector3d act_pos_in, Eigen::Vector3d act_vel_in, Eigen::Vector3d des_pos_in, Eigen::Vector3d des_vel_in)
{
	mActPos = act_pos_in;
	mActVel = act_vel_in;
	mDesPos = des_pos_in;
	mDesVel = des_vel_in;

	calc_errors();
}


void Controller::calc_errors()
{
	// Translational errors
	for (int i = 0; i < 2; i++)
	{
		mActErrPos[i] = mDesPos[i] - mActPos[i];
		mActErrVel[i] = mDesVel[i] - mActVel[i];

		// TO DO: update the denominator of this calculation to be based on the commanded control loop freq. 
		mderiv_act_er_pos[i] = (mActErrPos[i] - prev_act_er_pos[i])/0.05; 
	}

	// Rotational errors
	if (abs(mDesPos[2] - mActPos[2]) > PI)
		mActErrPos[2] = (2.0 * PI - abs(mDesPos[2] - mActPos[2])) * copysign(1.0, mDesPos[2] - mActPos[2]) * (-1.0);
	else if (abs(mDesPos[2] - mActPos[2]) == PI)
		mActErrPos[2] = copysign(1.0, angErPre) * PI;
	else
		mActErrPos[2] = mDesPos[2] - mActPos[2];

	mActErrVel[2] = mDesVel[2] - mActVel[2];

	// Save angular error in previous angular error
	angErPre = mActErrPos[2];
	mderiv_act_er_pos[2] = (mActErrPos[2] - prev_act_er_pos[2])/0.1;

	if (cont == 1) {
		prev_act_er_pos = mActErrPos;
	}

	/*std::cout << "[C] Act Yaw: " << act_pos[2] << std::endl;
	std::cout << "[C] Des Yaw: " << des_pos[2] << std::endl;
	std::cout << "[C] Yaw error: " << act_er_pos[2] << std::endl;*/

	// Save angular error in previous angular error
	angErPre = mActErrPos[2];

	//std::cout << "mActErrPos: " << mActErrPos.transpose() << std::endl;
}

void Controller::det_thrust_dir()
{
	if(cont == 0)
	{
		LimitCycle(); 
	}
	else if (cont == 1)
	{
		PID(); 
	}
	
}

void Controller::LimitCycle()
{
	for (int n = 0; n < 3; n++)
	{
		if (thrust_dir_pre[n] == 0)
		{
			if (mActErrVel[n] >= 0)
			{
				switch_line_r[n] = er_pos_thresh[n] + eps[n] - 1.0 / (2 * Kc_min[n]) * pow(mActErrVel[n], 2);
				switch_line_l[n] = -er_pos_thresh[n] + eps[n] - 1.0 / (2 * Kc_min[n]) * pow(mActErrVel[n], 2);
				
				/*if (n == 2)
				{
					std::cout << "switch_line_rA: " << switch_line_r[n] << std::endl; 
					std::cout << "swtich_line_lA: " << switch_line_l[n] << std::endl; 
				}*/
				
				
				// Sectional description of the switching line
				/*
				if (switch_line_r[n] < (-max_er_pos[n]))
					switch_line_r[n] = -max_er_pos[n];
				if (switch_line_l[n] < (-max_er_pos[n]))
					switch_line_l[n] = -max_er_pos[n];
				*/

				if (mActErrPos[n] > switch_line_r[n])
					thrust_dir[n] = 1;
				else if (mActErrPos[n] < switch_line_l[n])
					thrust_dir[n] = -1;
				else
					thrust_dir[n] = 0;
			}
			else
			{
				switch_line_r[n] = er_pos_thresh[n] - eps[n] + 1.0 / (2 * Kc_min[n]) * pow(mActErrVel[n], 2);
				switch_line_l[n] = -er_pos_thresh[n] - eps[n] + 1.0 / (2 * Kc_min[n]) * pow(mActErrVel[n], 2);

				/*if (n == 2)
				{
					std::cout << "switch_line_rB: " << switch_line_r[n] << std::endl;
					std::cout << "swtich_line_lB: " << switch_line_l[n] << std::endl;
				}*/


				// Sectional description of the switching line
				/*
				if (switch_line_r[n] > max_er_pos[n])
					switch_line_r[n] = max_er_pos[n];
				if (switch_line_l[n] > max_er_pos[n])
					switch_line_l[n] = max_er_pos[n];
				*/

				if (mActErrPos[n] < switch_line_l[n])
					thrust_dir[n] = -1;
				else if (mActErrPos[n] > switch_line_r[n])
					thrust_dir[n] = 1;
				else
					thrust_dir[n] = 0;
			}
		}
		else if (thrust_dir_pre[n] == 1)
		{
			if (mActErrVel[n] >= 0)
			{
				switch_line_l[n] = er_pos_thresh[n] - eps[n] - 1.0 / (2 * Kc_min[n]) * pow(mActErrVel[n], 2);
				// Sectional description of the switching line
				/*
				if (switch_line_l[n] < (-max_er_pos[n]))
					switch_line_l[n] = -max_er_pos[n];
				*/

				if (mActErrPos[n] > switch_line_l[n])
					thrust_dir[n] = 1;
				else
					thrust_dir[n] = 0;
			}
			else
			{
				switch_line_r[n] = er_pos_thresh[n] - eps[n] + 1.0 / (2 * Kc_min[n]) * pow(mActErrVel[n], 2);
				// Sectional description of the switching line
				/*
				if (switch_line_r[n] > max_er_pos[n])
					switch_line_r[n] = max_er_pos[n];
				*/

				if (mActErrPos[n] > switch_line_r[n])
					thrust_dir[n] = 1;
				else
					thrust_dir[n] = 0;
			}
		}
		else // thrust_dir_pre[n] == -1
		{
			if (mActErrVel[n] >= 0)
			{
				switch_line_l[n] = -er_pos_thresh[n] + eps[n] - 1.0 / (2 * Kc_min[n]) * pow(mActErrVel[n], 2);
				
				//if (n == 2)
				//{
				//	//std::cout << "switch_line_r: " << switch_line_r[n] << std::endl;
				//	std::cout << "swtich_line_lC: " << switch_line_l[n] << std::endl;
				//}
				
				
				// Sectional description of the switching line
				/*
				if (switch_line_l[n] < (-max_er_pos[n]))
					switch_line_l[n] = -max_er_pos[n];
				*/

				if (mActErrPos[n] < switch_line_l[n])
					thrust_dir[n] = -1;
				else
					thrust_dir[n] = 0;
			}
			else
			{
				switch_line_r[n] = -er_pos_thresh[n] + eps[n] + 1.0 / (2 * Kc_min[n]) * pow(mActErrVel[n], 2);
				
				//if (n == 2)
				//{
				//	std::cout << "switch_line_rC: " << switch_line_r[n] << std::endl;
				//	//std::cout << "swtich_line_l: " << switch_line_l[n] << std::endl;
				//}
				
				// Sectional description of the switching line
				/*
				if (switch_line_r[n] > max_er_pos[n])
					switch_line_r[n] = max_er_pos[n];
				*/

				if (mActErrPos[n] < switch_line_r[n])
					thrust_dir[n] = -1;
				else
					thrust_dir[n] = 0;
			}
		}
	}
	thrust_dir_pre = thrust_dir;
	
	/*std::cout << "Thrust Dir: " << std::endl;
	std::cout << thrust_dir << std::endl; */
}

void Controller::PID()
{
	//er_pos_thresh[2] = er_pos_thresh[2] * glc::PI / 180.0;
	for (int i = 0; i <= 2; i++)
	{
		double uOn = 0.2;
		double uOff = 0.09;

		tau[i] = Pgain[i] * mActErrPos[i] + Dgain[i] * mderiv_act_er_pos[i];
		if (tau[i] >= uOn)
		{
			thrust_dir[i] = 1;
		}
		else if (tau[i] < -uOn)
		{
			thrust_dir[i] = -1;
		}
		else if (tau[i] <= uOff && tau[i] >= -uOff)
		{
			thrust_dir[i] = 0;
		}
		else
		{
			thrust_dir[i] = 0;
		}
		
	}

	//std::cout << "Thrust Dir: " << thrust_dir << std::endl;
}

void Controller::calc_thrust_com()
{
	if (max_thrusters == 4)
	{
		// Thruster commands of the basic cases
		Eigen::Matrix<int, 8, 1> no_corr;
		no_corr << 0, 0, 0, 0, 0, 0, 0, 0;

		Eigen::Matrix<int, 8, 1> plus_x;
		plus_x << 1, 1, 0, 0, 0, 0, 0, 0;

		Eigen::Matrix<int, 8, 1> minus_x;
		minus_x << 0, 0, 0, 0, 1, 1, 0, 0;

		Eigen::Matrix<int, 8, 1> plus_y;
		plus_y << 0, 0, 1, 1, 0, 0, 0, 0;

		Eigen::Matrix<int, 8, 1> minus_y;
		minus_y << 0, 0, 0, 0, 0, 0, 1, 1;

		Eigen::Matrix<int, 8, 1> plus_phi;
		plus_phi << 0, 1, 0, 1, 0, 1, 0, 1;

		Eigen::Matrix<int, 8, 1> minus_phi;
		minus_phi << 1, 0, 1, 0, 1, 0, 1, 0;

		Eigen::Vector4i ind_x;
		ind_x << 0, 1, 4, 5;

		Eigen::Vector4i ind_y;
		ind_y << 2, 3, 6, 7;

		Eigen::Matrix<int, 8, 1> temp;
		temp << 0, 0, 0, 0, 0, 0, 0, 0;

		// Building the final thruster command by using the basic cases
		int num_dir = 0;
		for (int n = 0; n < 3; n++)
		{
			if (thrust_dir(n) != 0)
				num_dir++;
		}

		// x direction (translation)
		if (thrust_dir[0] == 1)
			thrust_com = plus_x;
		else if (thrust_dir[0] == -1)
			thrust_com = minus_x;
		else
			thrust_com = no_corr;

		// y direction (translation)
		if (thrust_dir[1] == 1)
		{
			for (int n = 0; n < 8; n++)
				thrust_com[n] += plus_y[n];
		}
		else if (thrust_dir[1] == -1)
		{
			for (int n = 0; n < 8; n++)
				thrust_com[n] += minus_y[n];
		}

		// phi direction (rotation)
		if (num_dir == 1 && thrust_dir[2] != 0)
		{
			if (thrust_dir[2] == 1)
				thrust_com = plus_phi;
			else
				thrust_com = minus_phi;
		}
		else if (num_dir == 3)
		{
			if (thrust_dir[2] == 1)
			{
				for (int n = 0; n < 8; n++)
				{
					if (thrust_com[n] == 1 && plus_phi[n] == 1)
						thrust_com[n] = 1;
					else
						thrust_com[n] = 0;
				}
			}
			else
			{
				for (int n = 0; n < 8; n++)
				{
					if (thrust_com[n] == 1 && minus_phi[n] == 1)
						thrust_com[n] = 1;
					else
						thrust_com[n] = 0;
				}
			}
		}
		else if (num_dir == 2 && thrust_dir[2] != 0)
		{
			if (thrust_dir[0] != 0)
			{
				if (thrust_dir[2] == 1)
					temp = plus_phi;
				else
					temp = minus_phi;

				for (int n = 0; n < 8; n++)
				{
					if (n == ind_x[0] || n == ind_x[1] || n == ind_x[2] || n == ind_x[3])
					{
						temp[n] = 0;
					}
					thrust_com[n] += temp[n];
				}
			}
			else if (thrust_dir[1] != 0)
			{
				if (thrust_dir[2] == 1)
					temp = plus_phi;
				else
					temp = minus_phi;

				for (int n = 0; n < 8; n++)
				{
					if (n == ind_y[0] || n == ind_y[1] || n == ind_y[2] || n == ind_y[3])
					{
						temp[n] = 0;
					}
					thrust_com[n] += temp[n];
				}
			}
		}
	}
	else if (max_thrusters == 2)
	{
		//printf("Thrust dir: %d, %d, %d\n", thrust_dir[0], thrust_dir[1], thrust_dir[2]);
		// commented thrust com are originals in the frame defined by previous student
		// I could never get it to work without changing the coordinate system. It probably is due
		// to how it is defined in optitrack 
		if (thrust_dir.isApprox(comp.block<3, 1>(0, 0))) // +x
		{
			//thrust_com << 1, 1, 0, 0, 0, 0, 0, 0;
			//thrust_com_s = "911000000";
			thrust_com << 0, 0, 0, 0, 0, 0, 1, 1; 
			thrust_com_s = "900000011";
		}
		else if (thrust_dir.isApprox(comp.block<3, 1>(0, 1))) // -x
		{
			//thrust_com << 0, 0, 0, 0, 1, 1, 0, 0;
			//thrust_com_s = "900001100";
			thrust_com << 0, 0, 1, 1, 0, 0, 0, 0; 
			thrust_com_s = "900110000";
		}
		else if (thrust_dir.isApprox(comp.block<3, 1>(0, 2))) // +y
		{
			//thrust_com << 0, 0, 1, 1, 0, 0, 0, 0;
			//thrust_com_s = "900110000";
			thrust_com << 1, 1, 0, 0, 0, 0, 0, 0;
			thrust_com_s = "911000000";
		}
		else if (thrust_dir.isApprox(comp.block<3, 1>(0, 3))) // -y
		{
			//thrust_com << 0, 0, 0, 0, 0, 0, 1, 1;
			//thrust_com_s = "900000011";
			thrust_com << 0, 0, 0, 0, 1, 1, 0, 0; 
			thrust_com_s = "900001100";

		}
		else if (thrust_dir.isApprox(comp.block<3, 1>(0, 4))) // +phi
		{					
			//thrust_com << 0, 0, 0, 1, 0, 0, 0, 1;
			//thrust_com_s = "900010001";
			thrust_com << 0, 1, 0, 0, 0, 1, 0, 0;
			thrust_com_s = "901000100";
		}
		else if (thrust_dir.isApprox(comp.block<3, 1>(0, 5))) // -phi
		{
			//thrust_com << 0, 0, 1, 0, 0, 0, 1, 0;
			//thrust_com_s = "900100010";
			thrust_com << 1, 0, 0, 0, 1, 0, 0, 0;
			thrust_com_s = "910001000";
		}
		else if (thrust_dir.isApprox(comp.block<3, 1>(0, 6))) // +x +y
		{
			//thrust_com << 1, 0, 0, 1, 0, 0, 0, 0;
			//thrust_com_s = "910010000";
			thrust_com << 0, 1, 0, 0, 0, 0, 1, 0; 
			thrust_com_s = "901000010";
		}
		else if (thrust_dir.isApprox(comp.block<3, 1>(0, 7))) // +x -y
		{
			//thrust_com << 0, 1, 0, 0, 0, 0, 1, 0;
			//thrust_com_s = "901000010";
			thrust_com << 0, 0, 0, 0, 1, 0, 0, 1;
			thrust_com_s = "900001001";
		}
		else if (thrust_dir.isApprox(comp.block<3, 1>(0, 8))) // -x +y
		{
			//thrust_com << 0, 0, 1, 0, 0, 1, 0, 0;
			//thrust_com_s = "900100100";
			thrust_com << 1, 0, 0, 1, 0, 0, 0, 0;
			thrust_com_s = "910010000";
		}
		else if (thrust_dir.isApprox(comp.block<3, 1>(0, 9))) // -x -y
		{
			//thrust_com << 0, 0, 0, 0, 1, 0, 0, 1;
			//thrust_com_s = "900001001";
			thrust_com << 0, 0, 1, 0, 0, 1, 0, 0;
			thrust_com_s = "900100100";
		}
		else if (thrust_dir.isApprox(comp.block<3, 1>(0, 10))) // +x +phi
		{
			//thrust_com << 0, 1, 0, 0, 0, 0, 0, 0;
			//thrust_com_s = "901000000";
			thrust_com << 0, 0, 0, 0, 0, 0, 0, 1;
			thrust_com_s = "900000001";
		}
		else if (thrust_dir.isApprox(comp.block<3, 1>(0, 11))) // +x -phi
		{
			//thrust_com << 1, 0, 0, 0, 0, 0, 0, 0;
			//thrust_com_s = "910000000";
			thrust_com << 0, 0, 0, 0, 0, 0, 1, 0;
			thrust_com_s = "900000010";
		}

		else if (thrust_dir.isApprox(comp.block<3, 1>(0, 12))) // -x +phi
		{
			//thrust_com << 0, 0, 0, 0, 0, 1, 0, 0;
			//thrust_com_s = "900000100";
			thrust_com << 0, 0, 0, 1, 0, 0, 0, 0;
			thrust_com_s = "900010000";
		}
		else if (thrust_dir.isApprox(comp.block<3, 1>(0, 13))) // -x -phi
		{
			//thrust_com << 0, 0, 0, 0, 1, 0, 0, 0;
			//thrust_com_s = "900001000";
			thrust_com << 0, 0, 1, 0, 0, 0, 0, 0;
			thrust_com_s = "900100000";
		}
		else if (thrust_dir.isApprox(comp.block<3, 1>(0, 14))) // +y +phi
		{
			//thrust_com << 0, 0, 0, 1, 0, 0, 0, 0;
			//thrust_com_s = "900010000";
			thrust_com << 0, 1, 0, 0, 0, 0, 0, 0;
			thrust_com_s = "901000000";
		}
		else if (thrust_dir.isApprox(comp.block<3, 1>(0, 15))) // +y -phi
		{
			//thrust_com << 0, 0, 1, 0, 0, 0, 0, 0;
			//thrust_com_s = "900100000";
			thrust_com << 1, 0, 0, 0, 0, 0, 0, 0;
			thrust_com_s = "910000000";
		}
		else if (thrust_dir.isApprox(comp.block<3, 1>(0, 16))) // -y +phi
		{
			//thrust_com << 0, 0, 0, 0, 0, 0, 0, 1;
			//thrust_com_s = "900000001";
			thrust_com << 0, 0, 0, 0, 0, 1, 0, 0;
			thrust_com_s = "900000100";
		}
		else if (thrust_dir.isApprox(comp.block<3, 1>(0, 17))) // -y -phi
		{
			//thrust_com << 0, 0, 0, 0, 0, 0, 1, 0;
			//thrust_com_s = "900000010";
			thrust_com << 0, 0, 0, 0, 1, 0, 0, 0;
			thrust_com_s = "900001000";
		}
		else if (thrust_dir.isApprox(comp.block<3, 1>(0, 18))) // +x +y +phi
		{
			//thrust_com << 0, 1, 0, 1, 0, 0, 0, 0;
			//thrust_com_s = "901010000";
			thrust_com << 0, 1, 0, 0, 0, 0, 0, 1;
			thrust_com_s = "901000001";
		}
		else if (thrust_dir.isApprox(comp.block<3, 1>(0, 19))) // +x +y -phi
		{
			//thrust_com << 1, 0, 1, 0, 0, 0, 0, 0;
			//thrust_com_s = "91010000";
			thrust_com << 1, 0, 0, 0, 0, 0, 1, 0;
			thrust_com_s = "910000010";
		}
		else if (thrust_dir.isApprox(comp.block<3, 1>(0, 20))) // +x -y +phi
		{
			//thrust_com << 0, 1, 0, 0, 0, 0, 0, 1;
			//thrust_com_s = "901000001";
			thrust_com << 0, 0, 0, 0, 0, 1, 0, 1;
			thrust_com_s = "900000101";
		}
		else if (thrust_dir.isApprox(comp.block<3, 1>(0, 21))) // -x +y +phi
		{
			//thrust_com << 0, 0, 0, 1, 0, 1, 0, 0;
			//thrust_com_s = "900010100";
			thrust_com << 0, 1, 0, 1, 0, 0, 0, 0;
			thrust_com_s = "901010000";
		}
		else if (thrust_dir.isApprox(comp.block<3, 1>(0, 22))) // +x -y -phi
		{
			//thrust_com << 1, 0, 0, 0, 0, 0, 1, 0;
			//thrust_com_s = "910000010";
			thrust_com << 0, 0, 0, 0, 1, 0, 1, 0;
			thrust_com_s = "900001010";
		}
		else if (thrust_dir.isApprox(comp.block<3, 1>(0, 23))) // -x +y -phi
		{
			//thrust_com << 0, 0, 1, 0, 1, 0, 0, 0;
			//thrust_com_s = "900101000";
			thrust_com << 1, 0, 1, 0, 0, 0, 0, 0;
			thrust_com_s = "910100000";
		}
		else if (thrust_dir.isApprox(comp.block<3, 1>(0, 24))) // -x -y +phi
		{
			//thrust_com << 0, 0, 0, 0, 0, 1, 0, 1;
			//thrust_com_s = "900000101";
			thrust_com << 0, 0, 0, 1, 0, 1, 0, 0;
			thrust_com_s = "900010100";
		}
		else if (thrust_dir.isApprox(comp.block<3, 1>(0, 25))) // -x -y -phi
		{
			//thrust_com << 0, 0, 0, 0, 1, 0, 1, 0;
			//thrust_com_s = "900001010";
			thrust_com << 0, 0, 1, 0, 1, 0, 0, 0;
			thrust_com_s = "900101000";
		}
		else if (thrust_dir.isApprox(comp.block<3, 1>(0, 26))) // nothing
		{
			thrust_com << 0, 0, 0, 0, 0, 0, 0, 0;
			thrust_com_s = "900000000";
		}
	}

	// Take minimum durations into account
	// if (min_durs)
	// {
	// 	loop_fulfilled = true;
	// 	dur_limit << 0, 0;
	// 	time_switch_temp = time_switch;

	// 	for (int i = 0; i < 8; i++)
	// 	{
	// 		if (thrust_com[i] != thrust_com_pre[i])
	// 		{
	// 			error_time_span = time_info::get_duration(time_info::get_loop_time(), time_switch[i]);
	// 			if (thrust_com_pre[i] == 0)
	// 			{
	// 				if (time_info::get_duration(time_info::get_loop_time(), time_switch[i]) < min_pause)
	// 				{
	// 					thrust_com = thrust_com_pre;
	// 					loop_fulfilled = false;
	// 					dur_limit(0) = 1; // Minimum pause time was not fulfilled
	// 					break;
	// 				}
	// 				else
	// 				{
	// 					time_switch_temp[i] = time_info::get_loop_time();
	// 				}
	// 			}
	// 			else
	// 			{
	// 				if (time_info::get_duration(time_info::get_loop_time(), time_switch[i]) < min_fire)
	// 				{
	// 					thrust_com = thrust_com_pre;
	// 					loop_fulfilled = false;
	// 					dur_limit(1) = 1; // Minimum fire time was not fulfilled
	// 					break;
	// 				}
	// 				else
	// 				{
	// 					time_switch_temp[i] = time_info::get_loop_time();
	// 				}
	// 			}
	// 		}
	// 	}
	// 	if (loop_fulfilled)
	// 	{
	// 		time_switch = time_switch_temp;
	// 	}
	// }
	thrust_com_pre = thrust_com;
}

void Controller::clean()
{
	mDesPos << 0.0, 0.0, 0.0; // desired position
	mActPos << 0.0, 0.0, 0.0; // actual position
	mDesVel << 0.0, 0.0, 0.0; // desired velocity
	mActVel << 0.0, 0.0, 0.0; // actual velocity
	mActErrPos << 0.0, 0.0, 0.0; // actual error in position
	mActErrVel << 0.0, 0.0, 0.0; // actual error in velocity

	thrust_dir << 0, 0, 0; // shows in which directions thrust has to be applied to the vehicle
	thrust_com << 0, 0, 0, 0, 0, 0, 0, 0; // thrust command of every single thruster (8 thrusters in total)
	switch_line_r << 0.0, 0.0, 0.0; // this is the switch line on the right side
	switch_line_l << 0.0, 0.0, 0.0; // this is the switch line on the left side
	Ks_main_local << 0.0, 0.0, 0.0;
	//use_slc << 0, 0, 0;
	//vel_er_min = 0;
	alpha << 0.0, 0.0, 0.0;
	max_er_vel_calc << 0.0, 0.0, 0.0;
	max_er_pos_calc << 0.0, 0.0, 0.0;

	/*for (int i = 0; i < 3; i++)
	{
		pos_calc_used[i] = false;
	}

	if (cycle == 2)
	{
		er_pos_thresh << 0.0, 0.0, 0.0;
		eps << 0.0, 0.0, 0.0;
		T_soft << 0.0, 0.0, 0.0;
		T_hard << 0.0, 0.0, 0.0;
	}*/
}