#include <iostream>
#include <cmath>
#include <string>
#include "Thruster_Commander.h"
#include "eigen-3.4.0/Eigen/Dense"

Thruster_Commander::Thruster_Commander()
{
	// TODO: Move all the hardcoded values in this constructor to a config file
	//       this will make unit testing simpler
	
	rho_water = 1025; // Density of water (kg/m^3)
	
	// Values come from Onshape 2024 Vehicle V10 11/12/24
	num_thrusters = 8;
	Eigen::Matrix<float, 1, 3> mass_center_inches = { 0, 0.466, 1.561 };
	mass_center = mass_center_inches * 0.0254; // convert to meters
	
	volume_center = mass_center; // volume center, currently, is a complete guess
	volume_center(0, 2) += 0.1; // add 0.1 meters to z coordinate to account for the volume center being slightly above the mass center

	// avg(max distance, min distance) of motor part 4 cylindrical surface to orgin
	// front left top, front right top, rear left top, rear right top, front left bottom, front right bottom, rear left bottom, rear right bottom
	// x, y, z coordinates here are how the appear on onshape. May need to be corrected to match surge, sway, heave
	thruster_positions = Eigen::Matrix<float, 8, 3>::Zero();
	thruster_positions.row(0) <<  -.2035, .2535, .042 ;
	thruster_positions.row(1) << .2035, .2535, -.042;
	thruster_positions.row(2) << -.2035, -.2545, .042;
	thruster_positions.row(3) << .2035, -.2545, .042;
	thruster_positions.row(4) << -.1375, .167, -.049;
	thruster_positions.row(5) << .1375, .167, -.049;
	thruster_positions.row(6) << -.1165, -.1975, -.049;
	thruster_positions.row(7) << .1165, -.1975, -.049;
	
	// torques will be calulated about the center of mass
	thruster_moment_arms = thruster_positions - mass_center.replicate(8, 1);
	
	// directionality recorded as the direction the front of thruster is facing
	// force direction will be reversed
	const double PI = 3.141592653589793;
	float sin45 = sin(45 * PI / 180);
	thruster_directions = Eigen::Matrix<float, 8, 3>::Zero();
	thruster_directions.row(0) << 0, 0, 1;
	thruster_directions.row(1) << 0, 0, 1;
	thruster_directions.row(2) << 0, 0, 1;
	thruster_directions.row(3) << 0, 0, 1;
	thruster_directions.row(4) << -sin45, -sin45, 0;
	thruster_directions.row(5) <<  sin45, -sin45, 0;
	thruster_directions.row(6) << sin45, -sin45, 0;
	thruster_directions.row(7) << -sin45, -sin45, 0;

	thruster_torques = Eigen::Matrix<float, 8, 3>::Zero();
	for (int i = 0; i < num_thrusters; i++)
	{
		thruster_torques.row(i) = thruster_moment_arms.row(i).cross(thruster_directions.row(i));
	}

	float volume_inches = 449.157;            // volume of vehicle in inches^3, from onshape. This is likley less than the displacement volume and should be corrected
	volume = volume_inches * pow(0.0254, 3); // convert to meters^3	
	mass = 5.51; // mass of vehicle in kg, from onshape

	// all zeros for now
	position = Eigen::Matrix<float, 1, 3>::Zero();
	orientation = Eigen::Matrix<float, 1, 3>::Zero();
	velocity = Eigen::Matrix<float, 1, 3>::Zero();
	angular_velocity = Eigen::Matrix<float, 1, 3>::Zero();
	acceleration = Eigen::Matrix<float, 1, 3>::Zero();
	angular_acceleration = Eigen::Matrix<float, 1, 3>::Zero();
}

Thruster_Commander::~Thruster_Commander()
{
}

void Thruster_Commander::print_info() 
{
	
	std::cout << "Mass Center: \n" << mass_center << std::endl;
	std::cout << "Volume Center: \n" << volume_center << std::endl;
	std::cout << "Thruster Positions: \n" << thruster_positions << std::endl;
	std::cout << "Thruster Moment Arms: \n" << thruster_moment_arms << std::endl;
	std::cout << "Thruster Directions: \n" << thruster_directions << std::endl;
	std::cout << "Thruster Torques: \n" << thruster_torques << std::endl;
	std::cout << "Mass: \n" << mass << std::endl;
	std::cout << "Volume: \n" << volume << std::endl;
}
int Thruster_Commander::get_pwm(int thruster_num, float force) {
 //   std::ifstream dataset("data/14V_PWM_Correlation.csv"); // Replace with your CSV file name
 //   std::string line;
	//std::string PWM;
	//int PWM_value;

 //   // Get the header line (if exists)
 //   if (std::getline(file, line)) {
	//	for( auto s: line){
	//	while(s != ','){
	//		PWM += s;
	//	}
	//	}
	//	PWM_value = stoi(PWM);
 //       while (low <= high) {
 //       int mid = low + (high - low) / 2;

 //       // Check if x is present at mid
 //       if (arr[mid] == x)
 //           return mid;

 //       // If x greater, ignore left half
 //       if (arr[mid] < x)
 //           low = mid + 1;

 //       // If x is smaller, ignore right half
 //       else
 //           high = mid - 1;
 //   }
	return 1500;
}


Eigen::Matrix<float, 1, 3> Thruster_Commander::compute_forces(force_array forces)
{
	Eigen::Matrix<float, 1, 3> total_force = Eigen::Matrix<float, 1, 3>::Zero();
	for (int i = 0; i < num_thrusters; i++)
	{
		total_force += forces.forces[i] * thruster_directions.row(i);
	}
	return total_force;
}

Eigen::Matrix<float, 1, 3> Thruster_Commander::compute_torques(force_array forces)
{
	Eigen::Matrix<float, 1, 3> total_torque = Eigen::Matrix<float, 1, 3>::Zero();
	for (int i = 0; i < num_thrusters; i++)
	{
		total_torque += forces.forces[i] * thruster_torques.row(i);
	}
	return total_torque;
}

void Thruster_Commander::test_force_functions()
{
	// check thrust_compute_fz
	// check thrust_compute_fy
	// check thrust_compute_fx
	// check thrust_compute_fx_fy
	// check thrust_compute_mz
	// check thrust_compute_fz_mz
	// check thrust_compute_fy_mz
	// check thrust_compute_fx_mz
	// check thrust_compute_fx_fy_mz
	// check thrust_compute_fx_fy_fz
	// check thrust_compute_fx_fy_fz_mz
	// check thrust_compute_fx_fy_fz_mx_my_mz
}

// this one is done and seems to work
force_array Thruster_Commander::thrust_compute_fz(float z_force)
{

    // Force is euqally divided by the 4 vertical thrusters for this function
	
	float force_per_thruster = z_force / 4;
	force_array forces = force_array();
	
	for(int i = 0; i < num_thrusters; i++)
	{
		if (i < 4) { forces.forces[i] = force_per_thruster; }
		else { forces.forces[i] = 0; }
		std::cout << i << " " << forces.forces[i] << std::endl;
	} 
	return forces;
}
force_array Thruster_Commander::thrust_compute_fy(float y_force)
{

	// fx, fz and mz should be zero
	// my and mz should be small enough to keep the vehicle stable
	
	force_array forces;
	return forces;
}
force_array Thruster_Commander::thrust_compute_fx(float x_force)
{
	// fy, fz and mz should be zero
	// mx and my should be small enough to keep the vehicle stable
	force_array forces;
	return forces;

}
force_array Thruster_Commander::thrust_compute_fx_fy(float x_force, float y_force)
{
	// fz and mz should be zero
	// mx and my should be small enough to keep the vehicle stable
	force_array forces;
	return forces;
}
force_array Thruster_Commander::thrust_compute_mz(float z_torque)
{
	// fx, fy and fz should be zero
	// mx and my should be small enough to keep the vehicle stable
	force_array forces;
	return forces;
}
force_array Thruster_Commander::thrust_compute_fz_mz(float z_force, float z_torque) 
{
	// fx and fy should be zero
	// mx and my should be small enough to keep the vehicle stable
	force_array forces;
	return forces;
}
force_array Thruster_Commander::thrust_compute_fy_mz(float y_force, float z_torque) 
{
	// fx and fz should be zero
	// mx and my should be small enough to keep the vehicle stable
	force_array forces;
	return forces;
}
force_array Thruster_Commander::thrust_compute_fx_mz(float x_force, float z_torque) 
{
	// fy and fz should be zero
	// mx and my should be small enough to keep the vehicle stable
	force_array forces;
	return forces;
}
force_array Thruster_Commander::thrust_compute_fx_fy_mz(float x_force, float y_force, float z_torque)
{
	// fz should be zero
	// mx and my should be small enough to keep the vehicle stable
	force_array forces;
	return forces;
}
force_array Thruster_Commander::thrust_compute_fx_fy_fz(float x_force, float y_force) 
{
	// mz should be zero
	// mx and my should be small enough to keep the vehicle stable
	force_array forces;
	return forces;
}
force_array Thruster_Commander::thrust_compute_fx_fy_fz_mz(float x_distance, float y_force, float z_force, float z_torque) 
{
	// mx and my should be small enough to keep the vehicle stable
	force_array forces;
	return forces;
}
force_array Thruster_Commander::thrust_compute_fx_fy_fz_mx_my_mz(float x_force, float y_force, float z_force, float, float x_torque, float y_torque, float z_torque) 
{
	// this is the most general case
	// all forces and torques are specified
	force_array forces;
	return forces;
}

force_array Thruster_Commander::thrust_compute(Eigen::Matrix<float, 1, 6> force_torque, bool simple=true)
{
	// this is a general case force function
	// it will call other force functions depending on what forces and torques are specified
	// if simple=true, mz and my will be neglected
	force_array forces;
	return forces;
}
