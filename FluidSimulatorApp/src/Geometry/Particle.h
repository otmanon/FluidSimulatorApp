#pragma once
#include <glm/glm.hpp>

#include "Circle.h"
class Particle : public Circle
{
protected:

	glm::vec2 m_Velocity;
	glm::vec2 m_Acceleration;


	float epsilon = 1.0e-6;

public:
	Particle(glm::vec2 position, float radius);
	~Particle() {}
	void DestroyParticle();

	virtual void step(float dt);


	void setAcceleration(glm::vec2 newAcceleration);

	void setRandomAcceleration(float magnitude);

	void setVelocity(glm::vec2 newVelocity);

	void setRandomVelocity(float magnitude);

	void normalizeVelocity(float magnitude);



	
};