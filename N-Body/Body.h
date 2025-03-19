#pragma once
#include "vector.h"
using namespace Customvectors;
class Body
{
private:
	double mass;
	Vector position;
	Vector velocity;

public:
	Body(double mass,Vector starting_pos,Vector starting_velocity);
	void setMass(double mass) { this->mass = mass; };
	double getMass() { return mass; };
	void setPosition(Vector position) { this->position = position; };
	Vector getPosition() { return position; };
	Vector getVelocity() { return velocity; }
	void setVelocity(Vector velocity) { this->velocity = velocity; }
};

