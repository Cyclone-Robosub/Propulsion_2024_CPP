#pragma once

// A command is a simple instruction to the vehicle
// Commands will be combined in sequencing to create more complex instructions

enum Direction {Forwards, Backwards};

struct force_array{
	float forces[8];
};
struct pwm_array {
    int pwm_signals[8];

};
struct Command {
    pwm_array thruster_pwms; // PWM values for each thruster
    float duration;       // Duration of the command in seconds
};
struct Sequence {
	std::vector<Command> commands;
};

