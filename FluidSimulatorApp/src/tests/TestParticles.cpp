#include "TestParticles.h"

#include "Renderer.h"
#include "imgui/imgui.h"
#include <iostream>
#include <vector>
#include "glm/glm.hpp"
#include "glm/gtc/matrix_transform.hpp"


namespace test {
	

	TestParticles::TestParticles(unsigned int * heightPtr, unsigned int * widthPtr)
		: Test(heightPtr, widthPtr), 
		m_Proj(glm::ortho(-(float)*m_WindowWidthPtr / 2, (float)*m_WindowWidthPtr / 2, -(float)*m_WindowHeightPtr / 2, (float)*m_WindowHeightPtr / 2, -1.0f, 1.0f)),
		m_View(glm::translate(glm::mat4(1.0f), glm::vec3(0, 0, 0))), m_StartingHeight(*m_WindowHeightPtr), m_StartingWidth(*m_WindowWidthPtr)
	{
		
		int x, y;
		for (int i = 0; i < m_DesiredParticleCount; i++)
		{
			createRandomParticle(x, y);
		}

	}




	TestParticles::~TestParticles()
	{
		for (auto& p : m_Particles) {
			p.DestroyParticle();
		}

	}

	void TestParticles::render()
	{
		float col[4] = { 1.0, 0.0, 0.0, 1.0f };
		for (auto& p : m_Particles) {
			p.render(m_Proj, m_View, col );
		}
		


	}

	void TestParticles::onImGuiRender()
	{
	//	int max_dim = glm::max(*m_WindowHeightPtr, *m_WindowWidthPtr);
		ImGui::SliderInt("Num Particles", (int*)&m_DesiredParticleCount, 0, 1000);
		ImGui::SliderFloat("Speed", &m_Speed, 1e-3, 1000);
	//	ImGui::SliderFloat2("Translation B", &m_TranslationB.x, -max_dim / 2, max_dim/2);

		ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);

	}

	void TestParticles::step()
	{
		float dt = 0.05f;
		maintainDesiredCount();

		int maxh = *m_WindowHeightPtr/2;
		int maxw = *m_WindowWidthPtr/2;
		for (auto& p : m_Particles) {

			p.step(dt);
			p.normalizeVelocity(m_Speed);
			if (p.getPosition().x > maxw) {
				p.setPosition(glm::vec2(-maxw, p.getPosition().y));
			}
			if (p.getPosition().x < -maxw) {
				p.setPosition(glm::vec2(maxw, p.getPosition().y));
			}

			if (p.getPosition().y > maxh) {
				p.setPosition(glm::vec2( p.getPosition().x, -maxh));
			}
			if (p.getPosition().y < -maxh) {
				p.setPosition(glm::vec2(p.getPosition().x, maxh));
			}


			if (m_Current == m_Period) {
				m_Current = 0;
				//set random acceleration
				p.setRandomAcceleration(10);
			}
		}

		m_Current += 1;
		
	}

	void TestParticles::maintainDesiredCount()
	{
		//create some
		if (m_Particles.size() < m_DesiredParticleCount) {
			int x, y;
			for (int i = 0; i < m_DesiredParticleCount - m_Particles.size(); i++)
			{
				createRandomParticle(x, y);
			}
		}
		//destroy some
		if (m_Particles.size() > m_DesiredParticleCount) {
			int difference = m_Particles.size() - m_DesiredParticleCount;
			for (int i = 0; i < difference; i++) {
				m_Particles[m_Particles.size() - 1].DestroyParticle();
				m_Particles.pop_back();
			}
			
		}
	
	}

	void TestParticles::createRandomParticle(int& x, int& y)
	{
		x = rand() % *m_WindowWidthPtr - *m_WindowWidthPtr / 2;
		y = rand() % *m_WindowHeightPtr - *m_WindowHeightPtr / 2;
		Particle p(glm::vec2(x, y), 10.0f);
		p.setRandomVelocity(m_Speed);
		m_Particles.push_back(p);
	}

	void TestParticles::onWindowResize() 
	{
		//change projection matrix
		m_Proj = glm::ortho(-(float)*m_WindowWidthPtr / 2, (float)*m_WindowWidthPtr / 2, -(float)*m_WindowHeightPtr / 2, (float)*m_WindowHeightPtr / 2, -1.0f, 1.0f);

	}

}