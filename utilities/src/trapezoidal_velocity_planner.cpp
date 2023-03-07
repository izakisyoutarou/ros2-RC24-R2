#include "trapezoidal_velocity_planner.hpp"



namespace velocity_planner {
namespace trapezoidal_velocity_planner {

rclcpp::Clock system_clock(RCL_ROS_TIME);

int64_t micros(){
    return system_clock.now().nanoseconds()*1e-3;
}

//VelPlanner
void VelPlanner::cycle() {
	if (mode == Mode::vel) {
        const double time = (micros() - start_time) / 1000000.0;
		if (time < t1) {
			old_time = micros();
			current_.acc = using_acc;
			current_.vel = using_acc * time + first.vel;
			current_.pos = using_acc / 2.0 * time * time + first.vel * time + first.pos;
		}
		else {
			mode = Mode::uniform_acceleration;
			has_achieved_target = true;
			current_ = target;
			cycle();
		}
	}
	else if(mode == Mode::uniform_acceleration){
        const double dt = (micros() - old_time) / 1000000.0;
        old_time = micros();
		current_.pos += current_.acc / 2.0 * dt * dt + current_.vel * dt;
		current_.vel += current_.acc * dt;
	}
	else {
		current_.vel = 0.0;
		current_.acc = 0.0;
	}
}

void VelPlanner::current(Physics_t physics) {
	current_ = physics;
	mode = Mode::uniform_acceleration;
	old_time = micros();
}

void VelPlanner::vel(double vel) {
    this->vel(vel, micros());
}

void VelPlanner::vel(double vel, double start_time) {
    this->vel(vel, (int64_t)(start_time * 1000000));
}

void VelPlanner::vel(double vel, int64_t start_time_us) {
	start_time = start_time_us;
	has_achieved_target = false;
	mode = Mode::vel;
	first = current_;
	target.acc = 0.0;
	target.vel = constrain(vel, -limit_.vel, limit_.vel);
	const double diff_vel = target.vel - first.vel;
	using_acc = (diff_vel >= 0) ? limit_.acc : -limit_.dec;
	t1 = diff_vel / using_acc;
	target.pos = using_acc * t1 * t1 / 2.0 + first.vel * t1 + first.pos;
}


//posPlanner
void PosPlanner::cycle() {
	if (mode == Mode::pos) {
        const double time = (micros() - start_time) / 1000000.0;
		if(time >= relay_time && task == 0) {
			velPlanner.current(relay);
			velPlanner.vel(target_vel, start_time + (int64_t)(relay_time * 1000000));
			task++;
		}
		velPlanner.cycle();
		current_ = velPlanner.current();
		if(velPlanner.hasAchievedTarget() && task == 1) {
			mode = Mode::uniform_acceleration;
			has_achieved_target = true;
		}
	}
	else if (mode == Mode::uniform_acceleration) {
		velPlanner.cycle();
		current_ = velPlanner.current();
	}
	else {
		current_.vel = 0.0;
		current_.acc = 0.0;
	}
}

void PosPlanner::current(Physics_t physics) {
	mode = Mode::uniform_acceleration;
	task = 0;
	current_ = physics;
	velPlanner.current(current_);
}

void PosPlanner::pos(double pos, double vel) {
	start_time = micros();
	mode = Mode::pos;
	has_achieved_target = false;
	task = 0;
	target_vel = constrain(vel, -limit_.vel, limit_.vel);
	const double pos_diff = constrain(pos, -limit_.pos, limit_.pos) - current_.pos;

	double acc1, acc3;
	const double acc = (target_vel >= current_.vel) ? limit_.acc : -limit_.dec;

	if (pos_diff >= 0 && pos_diff >= (target_vel * target_vel - current_.vel * current_.vel) / (2.0 * acc)
		|| pos_diff < 0 && pos_diff > (target_vel * target_vel - current_.vel * current_.vel) / (2.0 * acc) ) {
		acc1 = limit_.acc;
		acc3 = -limit_.dec;
		relay.vel = constrain(limit_.vel, 0.0, std::sqrt((acc1 * target_vel * target_vel - acc3 * current_.vel * current_.vel - 2.0 * acc1 * acc3 * pos_diff) / (acc1 - acc3)));
		if(relay.vel < current_.vel) acc1 = acc3;
	}
	else {
		acc1 = -limit_.dec;
		acc3 = limit_.acc;
		relay.vel = constrain(-limit_.vel, -std::sqrt((acc1 * target_vel * target_vel - acc3 * current_.vel * current_.vel - 2.0 * acc1 * acc3 * pos_diff) / (acc1 - acc3)), 0.0);
		if(relay.vel > current_.vel) acc1 = acc3;
	}

	const double x1 = (relay.vel * relay.vel - current_.vel * current_.vel) / 2.0 / acc1;
	const double x3 = (target_vel * target_vel - relay.vel * relay.vel) / 2.0 / acc3;
	const double x2 = pos_diff - x1 - x3;
	const double t1 = (relay.vel - current_.vel) / acc1;
	relay_time = x2 / relay.vel + t1;
	relay.pos = x1 + x2 + current_.pos;
	relay.acc = 0.0;
	velPlanner.vel(relay.vel);
}

//TrapezoidalVelocityPlanner
void TrapezoidalVelocityPlanner::current(double pos, double vel, double acc) {
	mode = Mode::uniform_acceleration;
	current_ = Physics_t(pos, vel, acc);
	velPlanner.current(current_);
}

void TrapezoidalVelocityPlanner::limit(double pos, double vel, double acc, double dec) {
	auto limit_ = Limit_t(pos, vel, acc, dec);
	velPlanner.limit(limit_);
	posPlanner.limit(limit_);
}

void TrapezoidalVelocityPlanner::pos(double pos, double vel) {
	if(mode == Mode::vel || mode == Mode::uniform_acceleration || mode == Mode::null) {
		velPlanner.cycle();
		posPlanner.current(velPlanner.current());
	}
	else {
		posPlanner.cycle();
	}
	mode = Mode::pos;
	has_achieved_target = false;
	posPlanner.pos(pos, vel);
}

void TrapezoidalVelocityPlanner::vel(double vel) {
	if(mode == Mode::pos || mode == Mode::null) {
		posPlanner.cycle();
		velPlanner.current(posPlanner.current());
	}
	else {
		
		velPlanner.cycle();
	}
	mode = Mode::vel;
	has_achieved_target = false;
	velPlanner.vel(vel);
}

void TrapezoidalVelocityPlanner::cycle() {
	if (mode == Mode::pos) {
		posPlanner.cycle();
		current_ = posPlanner.current();
		if(posPlanner.hasAchievedTarget()) {
			mode = Mode::uniform_acceleration;
			velPlanner.current(current_);
			has_achieved_target = true;
		}
	}
	else if (mode == Mode::vel) {
		velPlanner.cycle();
		current_ = velPlanner.current();
		if(velPlanner.hasAchievedTarget()) {
			mode = Mode::uniform_acceleration;
			has_achieved_target = true;
		}
	}
	else if (mode == Mode::uniform_acceleration) {
		velPlanner.cycle();
		current_ = velPlanner.current();
	}
	else {
		current_.vel = 0.0;
		current_.acc = 0.0;
	}
}

}
}
