#include "Particle.h"

Particle::Particle(glm::vec2 position, float radius)
	: Circle(radius, position),	m_Velocity(0, 0), m_Acceleration(0, 0)
{

}

void Particle::DestroyParticle()
{
	Particle::DestroyCircle();
}

void Particle::step(float dt)
{
	m_Velocity += m_Acceleration* dt;
	m_Position += m_Velocity * dt;

	Particle::setPosition(m_Position);
}



void Particle::setAcceleration(glm::vec2 newAcceleration)
{
	m_Acceleration = newAcceleration;
}

void Particle::setRandomAcceleration(float magnitude)
{
	float a0 = (float)rand() / RAND_MAX; //random number between 0 and 1
	float a1 = (float)rand() / RAND_MAX; //random number between 0 and 1
	glm::vec2 acc(a0-0.5f, a1-0.5f);
	acc = glm::normalize(acc);
	m_Acceleration = magnitude * acc;
}

void Particle::setVelocity(glm::vec2 newVelocity)
{
	m_Velocity = newVelocity;
}

void Particle::setRandomVelocity(float magnitude)
{
	float a0 = (float)rand() / RAND_MAX; //random number between 0 and 1
	float a1 = (float)rand() / RAND_MAX; //random number between 0 and 1
	glm::vec2 vel(a0-0.5f, a1-0.5f);
	vel = glm::normalize(vel);
	m_Velocity = magnitude * vel;

}

void Particle::normalizeVelocity(float magnitude)
{
	
		
	m_Velocity = magnitude*glm::normalize(m_Velocity) + glm::vec2(epsilon, epsilon);
}



