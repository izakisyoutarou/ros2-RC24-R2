#pragma once

/*******************************************
 * TrapezoidalVelocityPlanner ver2.1 2021/02/05
 * This program generate position,velocity and acceleration for trapezoidal control
 *
 * [Dependency]
 *
 * [Note]
 *
 * [Author]
 * Yuta Uehara
 *
 * [Change history]
 * ver2.1 2021/02/05 fix a bug that not work well when running vel() in continuous.
 * ver2.0 2020/11/25 rename file and class. ingertiance VelocityPlanner
 * ver1.0 2020/03/26 The first version
 ******************************************/

#include "velocity_planner.hpp"
#include <stdint.h>
#include <rclcpp/rclcpp.hpp>

using namespace std;

//rclcpp::Clock system_clock(RCL_ROS_TIME);
//rclcpp::Time now = system_clock.now();

template <class T>
    constexpr T constrain(T x, T min, T max) {
	if(x<min) return min;
	else if(max<x) return max;
	else return x;
}

// int64_t micros(){
//     return system_clock.now().nanoseconds()*1e-3;
// }

int64_t micros();

namespace velocity_planner {
namespace trapezoidal_velocity_planner {

struct Physics_t {
	Physics_t(){}
	Physics_t(double pos, double vel, double acc):pos(pos), vel(vel), acc(acc) {}
	double pos = 0.0;
	double vel = 0.0;
	double acc = 0.0;
};

struct Limit_t {
	Limit_t(double pos, double vel, double acc, double dec): pos(pos), vel(vel), acc(acc), dec(dec) {}
	double pos = 0.0;
	double vel = 0.0;
	double acc = 0.0;
	double dec = 0.0;
};

class VelPlanner {
public:
	VelPlanner(Limit_t limit): limit_(limit) {}
	void cycle();
	void current(Physics_t physics);
	void limit(Limit_t limit) { limit_ = limit; }

	void vel(double vel);
	void vel(double vel, double start_time);
	void vel(double vel, int64_t start_time_us);

	Physics_t current() { return current_; }

	bool hasAchievedTarget() { return has_achieved_target; }

private:
	Limit_t limit_;
	Physics_t first, target, current_;

	int64_t start_time = 0;
	int64_t old_time = 0;

	double t1 = 0.0;
	double using_acc = 0.0;

	enum class Mode {
		vel,
		uniform_acceleration
	} mode = Mode::uniform_acceleration;

	bool has_achieved_target = false;
};


class PosPlanner {
public:
	PosPlanner(Limit_t limit): velPlanner(limit), limit_(limit) {}
	void cycle();
	void current(Physics_t physics);
	void limit(Limit_t limit) { limit_ = limit; velPlanner.limit(limit); }

	void pos(double pos, double vel);

	Physics_t current() { return current_; }

	bool hasAchievedTarget() { return has_achieved_target; }

private:
	VelPlanner velPlanner;
	Limit_t limit_;
	Physics_t current_, relay;

	int64_t start_time = 0;

	double relay_time = 0.0;

	double target_vel = 0.0;

	int task = 0;

	enum class Mode {
		pos,
		uniform_acceleration
	} mode = Mode::uniform_acceleration;

	bool has_achieved_target = false;
};

class TrapezoidalVelocityPlanner: public VelocityPlanner{
public:
	public:
	TrapezoidalVelocityPlanner(Limit_t limit = Limit_t(0.0, 0.0, 0.0, 0.0)): velPlanner(limit), posPlanner(limit) {}
	void cycle() override;

	void current(double pos, double vel, double acc) override;
	void current(Physics_t current) { this->current(current.pos, current.vel, current.acc); }
	Physics_t current() { return current_; }

	void limit(double pos, double vel, double acc, double dec) override;
	void limit(Limit_t limit) { this->limit(limit.pos, limit.vel, limit.acc, limit.dec); }

	void pos(double pos, double vel) override;
	void vel(double vel) override;

	double pos() override { return current_.pos; }
	double vel() override { return current_.vel; }
	double acc() override { return current_.acc; }

	bool hasAchievedTarget() override { return has_achieved_target; }
private:

	Physics_t current_;

	VelPlanner velPlanner;
	PosPlanner posPlanner;

	enum class Mode {
		pos,
		vel,
		uniform_acceleration,
		null
	} mode = Mode::null;

	bool has_achieved_target = false;
};

}

// 利便性のためのエイリアス及びusing宣言
using trapezoidal_velocity_planner::TrapezoidalVelocityPlanner;
using TrapezoidalVelocityPlannerLimit = trapezoidal_velocity_planner::Limit_t;

}
