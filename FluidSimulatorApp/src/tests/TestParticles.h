#pragma once

#include "Test.h"
#include "VertexBuffer.h"
#include "VertexBufferLayout.h"

#include "Circle.h"
#include "Particle.h"
#include <memory>
namespace test {

	class TestParticles : public Test {

	private:

		std::vector<Particle> m_Particles;



		glm::mat4 m_Proj, m_View;


		unsigned int m_Period = 10;
		unsigned int m_Current = 0;
		unsigned int m_DesiredParticleCount = 200;

		float m_Speed = 30;

		unsigned int m_StartingHeight;
		unsigned int m_StartingWidth;

	public:
		TestParticles(unsigned int * heightPtr, unsigned int * widthPtr);
		~TestParticles();


		void render() override;
		void onImGuiRender() override;

		void step(float dt) override;

		void maintainDesiredCount();

		void createRandomParticle(int& x , int& y);

		void onWindowResize() override;


	};

	

}

