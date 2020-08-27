#pragma once
#include <glm/glm.hpp>
#include <Eigen/Dense>
#include "Circle.h"
class Particle : public Circle
{
protected:

	glm::vec2 m_Velocity;
	glm::vec2 m_Acceleration;


	float epsilon = 1.0e-6;

public:
	Particle(glm::vec2 position, float radius);
	~Particle() {
		m_vb->Unbind();
		m_va->Unbind();
		m_ib->Unbind();
	}
	void DestroyParticle();

	virtual void step(float dt);


	void setAcceleration(glm::vec2 newAcceleration);

	void setRandomAcceleration(float magnitude);

	void setVelocity(glm::vec2 newVelocity);

	void setRandomVelocity(float magnitude);

	void normalizeVelocity(float magnitude);


	Eigen::Vector2f getVelocity() { return Eigen::Vector2f(m_Velocity.x, m_Velocity.y); }

	void incrementVelocity(Eigen::Vector2f incV)
	{
		m_Velocity.x += incV.x();

		m_Velocity.y += incV.y();
	}
	
};