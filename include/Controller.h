#pragma once

#include <string>
#include "Eigen/Dense"
#include <iostream>
#include <array>
#include <algorithm>

#define PI 3.14159265358979323846

class Controller
{
public:
	Controller(); 
	~Controller();

	///
	/// @brief setter function for the thrust command
	///  
	///	Sets the member variable thrust command 
	/// 
	/// @param thrust_com_in - a string representing the thrust command e.g "11000000" for setting the thrust command to fire thrusters 1 & 2 
	/// @return void
	///
	void set_thrust_com(std::string thrust_com_in);
	
	///
	/// @brief accessor function to get the thrust command member variable
	///  
	///	Gets the thrust command member variable stored as string type
	/// 
	/// @return: a string representing the current thrust command
	///
	std::string get_thrust_com(); 

	///
	/// @brief accessor function for the thrust command
	///  
	///	Gets the thrust command member variable stored as matrix type
	/// 
	/// @return: 8x1 Eigen::Matrix representing the thrust command 
	///
	Eigen::Matrix<int, 8, 1> get_thrust_com_cont();

	///
	/// @brief accessor function for the thrust_dir vector
	///  
	///	Gets the thrust direction vector representing the direction for the ABV to command thrust in. It is a 3x1 vector where the indexes represent which direction to apply thrust in. A 0 is no thrust in that axis direction, a 1 is a positive thrust and a -1 is a negative thrust, e.g. thrust_dir = {1, 0 -1} is commanding a thrust in the +X direction, no thrust in the y-direction and a thrust in the negative yaw direction. 
	/// 
	/// @return The thrust direction vector as a 3x1 Eigen::Vector3i
	///
	Eigen::Vector3i get_thrust_dir(); 

	void set_values(Eigen::Vector3d act_pos_in, Eigen::Vector3d act_vel_in, Eigen::Vector3d des_pos_in, Eigen::Vector3d des_vel_in);
	void set_values(Eigen::Vector3d act_pos_in, Eigen::Vector2d act_pos_manip_in, Eigen::Vector3d act_vel_in, Eigen::Vector2d act_vel_manip_in, Eigen::Vector3d des_pos_in, Eigen::Vector2d des_pos_manip_in, Eigen::Vector3d des_vel_in, Eigen::Vector2d des_vel_manip_in);
	
	///
	/// @brief calculate the error in the control loop
	///  
	///	calc_errors calculates the difference between the desired pose and the actual pose. The error for the x and y directions is simply the difference between	  the desired x and y values and the actual x and y values. The yaw angle error is the difference between the desired and actual yaw angle and is limited       between -pi and pi radians 
	/// 
	/// @return void
	///
	void calc_errors();

	void LimitCycle(); 
	
	///
	/// @brief computes the control input from a PID controller
	///  
	///	Computes the control input from a PID controller based on the pose error at the current timestep. Because the PID controller produces a continuous value and the thrusters can only turn on or off, a Schmitt-Trigger esque logic is applied to discretize the controller input and determine the thrust direction vector.
	/// 
	/// @return void
	///
	void PID(); 

	///
	/// @brief determines the control law used to determine the thrust direction vector
	///  
	///	Depending on the configuration of the system, the desired control law is selected and used to compute the thrust direction vector.
	/// 
	/// @return void
	///
	void det_thrust_dir();

	///
	/// @brief calculates the thrust command based on the thrust direction vector
	///  
	///	Calculates the configuration of which thrusters need to be fired in order to achieve the desired thrust direction vector. 
	/// 
	/// @return void
	///
	void calc_thrust_com(); 
	
	///
	/// @brief debugging helper function to print the controller config values
	///  
	///	Prints the values determined by the config.xml file or the main GUI
	/// 
	/// @return void
	///
	void PrintVals(); 

	///
	/// @brief resets the member variables to zeros after each iteration
	///  
	///	Resets the member variables to zero in order to relieve any propagation of bad values.
	/// 
	/// @return void
	///
	void clean(); 

private:

	// Variables that change every iteration
	Eigen::Vector3d mDesPos; // desired position
	Eigen::Vector3d mActPos; // actual position
	Eigen::Vector3d mDesVel; // desired velocity
	Eigen::Vector3d mActVel; // actual velocity
	Eigen::Vector3d mActErrPos; // actual error in position
	Eigen::Vector3d mActErrVel; // actual error in velocity
	Eigen::Vector3d mderiv_act_er_pos;

	Eigen::Matrix<double, 3, 1> prev_act_er_vel;
	Eigen::Matrix<double, 3, 1> prev_act_er_pos;
	Eigen::Matrix<double, 5, 1> tau;
	
	Eigen::Matrix<double, 5, 1> act_er_pos_SM; 
	Eigen::Matrix<double, 5, 1> act_er_vel_SM; 
	Eigen::Matrix<double, 5, 1> max_er_pos_SM;
	Eigen::Matrix<double, 5, 1> max_er_vel_SM;

	Eigen::Vector3i thrust_dir;							// shows in which directions thrust has to be applied to the vehicle
	Eigen::Matrix<int, 8, 1> thrust_com;				// thrust command of every single thruster (8 thrusters in total)
	std::string thrust_com_s;							// thrust command representing as a string 
	Eigen::Vector3i thrust_dir_pre;						// is the value of thrust_dir of the previous time step
	Eigen::Matrix<int, 8, 1> thrust_com_pre;			// thrust command from the previous time step. Necessary for the minimum fire and pause durations
	Eigen::Matrix<int, 3, 27> comp;						// Compare thrust_dir with this to get the individual situation
	
	// Constants for the whole program
	Eigen::Vector3d max_er_pos;							// maximum error in position of the limit cycle
	Eigen::Vector3d max_er_vel;							// maximum error in velocity of the limit cycle
	Eigen::Vector3d Ks_main;							// main acceleration due to disturbances (e.g. gravity)
	Eigen::Vector3d Ks_main_local;
	Eigen::Vector3d Kc_min;								// minimum acceleration possible by CGT (cold gas thrusters)
	Eigen::Vector3d switch_line_r;						// this is the switch line on the right side
	Eigen::Vector3d switch_line_l;						// this is the switch line on the left side

	Eigen::Vector3d er_pos_thresh;						// this value is the threshold for the error in position for the limit cycle
	Eigen::Vector3d eps;								// this value is the displacement due to the hysteresis
	Eigen::Vector3d T_soft;								// this is the period of the soft limit cycle
	Eigen::Vector3d T_hard;								// this is the period of the hard limit cycle
	Eigen::Vector3d alpha;								// ratio between acceleration by thrusters and by disturbances
	Eigen::Vector3d max_er_vel_calc;					// the soft limit cycle is defined by only one of max_er_vel and max_er_pos. Thus, the other value is														  // calculated to see, which one is the lower limit. Then, the cycle is defined by this value.
	Eigen::Vector3d max_er_pos_calc;
	
	Eigen::Vector3d Pgain;								// proportional gains for PD style switching fcn in bang bang controller
	Eigen::Vector3d Dgain;								// derivative gains for PD style switching fcn in bang bang controller
	Eigen::Vector3d errFcn;								// value of switching function used in bang bang controller
	

	double vehiclemass;
	double inertia;
	double force1;
	double force2;
	double force4;
	double torquearm;
	double main_freq;
	bool min_durs;
	double min_fire;
	double min_pause;
	double thresh[3]; 

																				// time_switch initialized like this directly sets the values to 0 nanoseconds														belonging to the moment when the computer was started. min_fire and														min_pause are short enough that this is possible.
	double angErPre;															// previous angular error

	// Miscellaneous (not important, just for temporary storage or some conditions)
	bool loop_fulfilled; // true, if the loop was executed completely
	Eigen::Vector2i dur_limit; // This is to safe, whether the duration in firing or pause fell below its limit. This is sent back to the remote computer
	double error_time_span; // This variable is just to help me during error fixing

	int cont; 
	int max_thrusters; 

};