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

	// this will mostly be used for debugging and unit testing
	int get_force_from_pwm(int thruster_num, int pwm);

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
	~Thruster_Commander();

	// mostly for debugging purposes
	void print_info();

	// functions begining with 'simple_' make the assumption that the vehicle is stable about the x and y axes
	// these functions only need to consider forces in the x, y, and z directions, and moments about the z axis
	// these should output 1x8 arrays of pwm signals, not commands
	
	bool simple_is_it_possible(float x_force, float y_force, float z_force, float z_torque);
	bool is_it_possible(float x_force, float y_force, float z_force, float x_torque, float y_torque, float z_torque);

	// force-torque commands
	// these functions compute the pwm signals for the thrusters to achieve the desired force and torque produced by thrusters
	// these functions do not condier drag, or any other forces not produced directly by the thruster 
	force_array simple_vertical_forces(float z_force);
	

	// /!\ TODO:
	// the following functions should be modified to produce force_array objects rather than pwm_array objects
	// this will allow for easier testing and debugging. The pwm_array objects can be generated from the force_array objects
	// using a separate function. These functions should also be renamed with _forces suffix for clarity
	pwm_array simple_forward(float y_force);
	pwm_array simple_sideways(float x_force);
	pwm_array simple_horizontal(float x_force, float y_force);
	
	pwm_array simple_rotation(float z_torque);
	pwm_array simple_rotating_vertical(float z_force, float z_torque);
	pwm_array simple_rotating_forward(float force, float torque);
	pwm_array simple_rotating_sideways(float force, float torque);
	pwm_array simple_rotating_horizontal(float x_force, float y_force, float torque);

	pwm_array simple_3d(float x_force, float y_force);
	pwm_array simple_rotating_3d(float x_distance, float y_force, float z_force, float torque);

	pwm_array complex_3d(float x_force, float y_force, float z_force, float, float x_torque, float y_torque, float z_torque);

	// distance commands
	Command simple_vertical_travel(float z_distance);
	
};
