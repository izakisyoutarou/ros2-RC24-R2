#pragma once
/*******************************************
 * velocity_planner ver1.0 2020/6/29
 * This program is base class for velocity generator
 *
 * [Dependency]
 *
 * [Note]
 *
 * [Author]
 * Yuta Uehara
 *
 * [Change history]
 * ver1.1 2020/11/25 delete extra include
 * ver1.0 2020/ 6/29 The first version
 ******************************************/

class VelocityPlanner {
public:
    virtual ~VelocityPlanner() {}
    virtual void cycle() = 0;

    virtual void current(double pos, double vel, double acc) = 0;
    virtual void limit(double pos, double vel, double acc, double dec) = 0;
    virtual void limit(double pos, double vel, double acc) { limit(pos, vel, acc, -acc); }

    virtual void pos(double pos, double vel) = 0;
    virtual void pos(double pos) { this->pos(pos, 0.f); }
    virtual void vel(double vel) = 0;

    virtual double pos() = 0;
    virtual double vel() = 0;
    virtual double acc() = 0;

    virtual bool hasAchievedTarget() = 0;
};
