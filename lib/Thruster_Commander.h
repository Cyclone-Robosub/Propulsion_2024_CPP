// Zenyn Ethridge, 2024
// zjethridge@ucdavis.edu

#pragma once
#include "eigen-3.4.0/Eigen/Dense"
#include "Command.h"
#include <vector>


/// @breif a set of values for each thruster
typedef Eigen::Matrix<float, 8, 1> thruster_set;
typedef Eigen::Matrix<float, 8, 3> thruster_set_3D;
typedef Eigen::Matrix<float, 8, 6> thruster_set_6D;

typedef Eigen::Matrix<float, 1, 6> six_axis;
typedef Eigen::Matrix<float, 1, 3> three_axis;

// The purpose of this class is to generate command objects
// A command object is a simple instruction to the vehicle
class Thruster_Commander
{
protected:

	// Shouldn't usually change
	three_axis mass_center;
	three_axis volume_center;
	three_axis mass_moment_of_inertia;  
	thruster_set_3D thruster_positions;
	
	thruster_set_3D thruster_moment_arms;  // Distance from thruster to mass center (m)
	thruster_set_3D thruster_directions;  // Direction of thruster force (unit vector)
	thruster_set_3D thruster_torques;   // Torque of thruster on sub about x,y,z (Nm)
	thruster_set thruster_voltages; // voltage supplied to each thruster (V) 

	int num_thrusters;  // Number of thrusters on the vehicle
	float mass;        // Mass of the vehicle (kg)
	float volume;     // Displacement volume of the vehicle (m^3)
	float rho_water; // Density of water (kg/m^3)
	float gravity;  // Acceleration due to gravity (m/s^2)
	float weight_magnitude; // Weight of the vehicle (N)
	float buoyant_magnitude; // Buoyant force on the vehicle (N)
	
	// todo: make these six axis
	three_axis orientation; // roll, puitch, yaw in rad. sign convention is right hand rule
	three_axis position;    // x, y, z, relative to start position in m
	three_axis velocity;    // Velocity of the vehicle (surge, sway, heave) in m/s
	three_axis angular_velocity; // rad/s
	three_axis acceleration; // x, y, z m/s^2
	three_axis angular_acceleration; // roll, pitch, yaw in rad/s^2
	three_axis water_current_velocity; // velocity of water in x, y, z in m/s

public:
	Thruster_Commander();

	// we should move to this constructor style asap
	Thruster_Commander(std::string file);
	~Thruster_Commander();

	// mostly for debugging purposes
	void print_info();
	// Returns the PWM value for a given thruster and force
	int get_pwm(int thruster_num, float force);

	// Returns the PWM values for a given set of forces
	pwm_array get_pwms(force_array forces);

	// this will mostly be used for debugging and unit testing
	int get_force_from_pwm(int thruster_num, int pwm);

	void test_force_functions();

	// also mostly for testing
	Eigen::Matrix<float, 1, 3> compute_forces(force_array);
	Eigen::Matrix<float, 1, 3> compute_torques(force_array);

	//todo: make these functions 6-axis
	six_axis weight_force(three_axis orientation); 
	six_axis bouyant_force(three_axis orientation);
	six_axis graviational_forces(three_axis orientation);
	six_axis predict_drag_forces(six_axis velocity);

	// environmental forces such as weight, boyancy, drag, ect
	six_axis net_env_forces(six_axis velocity, three_axis oritation);

	float top_speed(three_axis direction);

	// this function will integrate the environmental forces and thruster
	// forces to calculate the linear and angular impulse (change in momentum) 
	// on the vehicle
	six_axis integrate_impulse(
		six_axis start_velocity, 
		thruster_set thruster_sets, 
		float duration, int n);

	// net force produced by thrusters at a particular set of pwms. This will mostly be used for testing
	six_axis predict_net_force(pwm_array pwms);

	// functions begining with 'simple_' make the assumption that the vehicle is stable about the x and y axes
	// these functions only need to consider forces in the x, y, and z directions, and moments about the z axis
	// these should output 1x8 arrays of pwm signals, not commands
	
	bool simple_is_it_possible(float x_force, float y_force, float z_force, float z_torque);
	bool is_it_possible(float x_force, float y_force, float z_force, float x_torque, float y_torque, float z_torque);

	// force-torque computations
	// these functions compute the forces from each thruster necessary to acheive a given force or torque on the sub
	// these functions do not condier drag, or any other forces not produced directly by the thruster
	// suffixes state which forces and torques are being driven by the thrusters
	// if a force fx, fy, fz , or torque mz is not specified, it's desried value is zero
	// if a torque mx or my is not stated, it should be small enough for the vehicle to be stable
	thruster_set thrust_compute_fz(float z_force);
	thruster_set thrust_compute_fy(float y_force);
	thruster_set thrust_compute_fx(float x_force);
	thruster_set thrust_compute_fx_fy(float x_force, float y_force);
	thruster_set thrust_compute_mz(float z_torque);
	thruster_set thrust_compute_fz_mz(float z_force, float z_torque);
	thruster_set thrust_compute_fy_mz(float y_force, float z_torque);
	thruster_set thrust_compute_fx_mz(float force, float torque);
	thruster_set thrust_compute_fx_fy_mz(float x_force, float y_force, float torque);
	thruster_set thrust_compute_fx_fy_fz(float x_force, float y_force);
	thruster_set thrust_compute_fx_fy_fz_mz(float x_distance, float y_force, float z_force, float torque);
	thruster_set thrust_compute_fx_fy_fz_mx_my_mz(six_axis force_torque);

	// this is the general case force function
	// it will call other force functions depending on what forces and torques are specified
	// if simple=true, mz and my will be neglected
	thruster_set thrust_compute(six_axis force_torque, bool simple=true);

	// computes a force array from an acceleration vector (surge, sway, heave) in m/s^2, and (roll, pitch, yaw) in rad/s^2
	// this function will consider the mass of the vehicle and external forces such as drag
	force_array acceleration_compute(six_axis acceleration, bool simple=true);

	// generates a command to a target velocity
	// target velocity is a 1x6 matrix with (sway, surge, heave) linear velocities in m/s and (roll, pitch, yaw) angular velocities in r/s
	Command accelerate_to(six_axis target_velocity);

	// this is the big, important, general case function which we're building up to
	std::vector<Command> sequence_to(six_axis target_position);
};
