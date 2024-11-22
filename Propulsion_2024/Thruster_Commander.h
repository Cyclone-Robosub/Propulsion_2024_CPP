#pragma once
#include "eigen-3.4.0/Eigen/Dense"
#include "Command.h"
#include <vector>


// The purpose of this class is to generate command objects
// A command object is a simple instruction to the vehicle
class Thruster_Commander
{
protected:

	// Shouldn't usually change
	Eigen::Matrix<float, 1, 3> mass_center;
	Eigen::Matrix<float, 1, 3> volume_center;
	Eigen::Matrix<float, 8, 3> thruster_positions;
	Eigen::Matrix<float, 8, 3> thruster_moment_arms;  // Distance from thruster to mass center (m)
	Eigen::Matrix<float, 8, 3> thruster_directions;  // Direction of thruster force (unit vector)
	Eigen::Matrix<float, 8, 3> thruster_torques;    // Torque of thruster on the sub about the x,y and z axes (N*m)
	Eigen::Matrix<float, 8, 1> thruster_voltages;  // voltage supplied to each thruster (V) (this might not be a constant)

	int num_thrusters;  // Number of thrusters on the vehicle
	float mass;        // Mass of the vehicle (kg)
	float volume;     // Displacement volume of the vehicle (m^3)
	float rho_water; // Density of water (kg/m^3)
	float gravity;   // Acceleration due to gravity (m/s^2)
	float weight_magnitude; // Weight of the vehicle (N)
	float buoyant_magnitude; // Buoyant force on the vehicle (N)

	// Variables. These need to be updated continuously
	Eigen::Matrix<float, 1, 3> orientation; // Orientation of the vehicle (roll, pitch, yaw) in radians relative to starting orientation
	Eigen::Matrix<float, 1, 3> position;    // Position of the vehicle (x, y, z) in meters relative to starting position
	Eigen::Matrix<float, 1, 3> velocity;    // Velocity of the vehicle (surge, sway, heave) in m/s
	Eigen::Matrix<float, 1, 3> angular_velocity; // Angular velocity of the vehicle (roll, pitch, yaw) in rad/s
	Eigen::Matrix<float, 1, 3> acceleration; // Acceleration of the vehicle (surge, sway, heave) in m/s^2
	Eigen::Matrix<float, 1, 3> angular_acceleration; // Angular acceleration of the vehicle (roll, pitch, yaw) in rad/s^2

	Eigen::Matrix<float, 1, 3> water_current_velocity; // velocity of the water current in m/s. Should be near 0 in controlled environments

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

	// these functions will find weight and boyancy force vectors based on orientation and magnitudes
	Eigen::Matrix<float, 1, 3> weight_force();
	Eigen::Matrix<float, 1, 3> buoyant_force();


	// Drag force = 0.5 * rho_water * v^2 * Cd * A
	// rho water is known, A can be found as a function of direction, v is input, Cd is unknown, but can be estimated or found through trail and error
	Eigen::Matrix<float, 1, 3> predict_drag_forces(Eigen::Matrix<float, 1, 3> velocity, Eigen::Matrix<float, 1, 3> angular_velocity);
	
	// <drag_torque> = <r> x <drag_force> = <r> x < 0.5 * rho_water * (r*omega)^2 * Cd * A >
	// this will be a little trickier to calculate, might require integration
	Eigen::Matrix<float, 1, 3> predict_drag_torques(Eigen::Matrix<float, 1, 3> velocity, Eigen::Matrix<float, 1, 3> angular_velocity);

	// environmental forces such as weight, boyancy, drag, ect
	Eigen::Matrix<float, 1, 3> predict_env_forces(Eigen::Matrix<float, 1, 3> velocity, Eigen::Matrix<float, 1, 3> angular_velocity);

	// torques produced by environmental forces
	Eigen::Matrix<float, 1, 3> predict_env_torques(Eigen::Matrix<float, 1, 3> velocity, Eigen::Matrix<float, 1, 3> angular_velocity);

	// net force produced by thrusters at a particular set of pwms. This will mostly be used for testing
	Eigen::Matrix<float, 1, 3> predict_net_force(Eigen::Matrix<float, 1, 8> pwms);

	// net torque produced by thrusters at a particular set of pwms. This will mostly be used for testing
	Eigen::Matrix<float, 1, 3> predict_net_torque(Eigen::Matrix<float, 1, 8> pwms);

public:
	Thruster_Commander();

	// we should move to this constructor style asap
	Thruster_Commander(std::string file, std::string type="json");
	~Thruster_Commander();

	// mostly for debugging purposes
	void print_info();

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
	force_array thrust_compute_fz(float z_force);
	force_array thrust_compute_fy(float y_force);
	force_array thrust_compute_fx(float x_force);
	force_array thrust_compute_fx_fy(float x_force, float y_force);
	force_array thrust_compute_mz(float z_torque);
	force_array thrust_compute_fz_mz(float z_force, float z_torque);
	force_array thrust_compute_fy_mz(float y_force, float z_torque);
	force_array thrust_compute_fx_mz(float force, float torque);
	force_array thrust_compute_fx_fy_mz(float x_force, float y_force, float torque);
	force_array thrust_compute_fx_fy_fz(float x_force, float y_force);
	force_array thrust_compute_fx_fy_fz_mz(float x_distance, float y_force, float z_force, float torque);
	force_array thrust_compute_fx_fy_fz_mx_my_mz(float x_force, float y_force, float z_force, float, float x_torque, float y_torque, float z_torque);

	// this is a general case force function
	// it will call other force functions depending on what forces and torques are specified
	// if simple=true, mz and my will be neglected
	force_array thrust_compute(Eigen::Matrix<float, 1, 6> force_torque, bool simple=true);

	// computes a force array from an acceleration vector (surge, sway, heave) in m/s^2, and (roll, pitch, yaw) in rad/s^2
	force_array acceleration_compute(Eigen::Matrix<float, 1, 6> acceleration, bool simple=true);

	// generates a command to a target velocity
	// target velocity is a 1x6 matrix with (sway, surge, heave) linear velocities in m/s and (roll, pitch, yaw) angular velocities in r/s
	Command accelerate_to(Eigen::Matrix<float, 1, 6> target_velocity);
	
};

