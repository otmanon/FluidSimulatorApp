#pragma once
#pragma once

#include "Test.h"
#include "imgui.h"
#include "Circle.h"
#include "Particle.h"
#include "Grid.h"
#include <Eigen/Dense>
#include <memory>
#include "Canvas.h"
#include "VelocityField.h"


namespace test {

	class TestGrid : public Test
	{

	private:
		Canvas2D canvas;
		VelocityField2D vf;

	public:
		TestGrid(unsigned int * heightPtr, unsigned int * widthPtr) : 
			Test(heightPtr, widthPtr)
		{
			//Init canvas
			canvas.proj = glm::ortho(0.0f, (float)*m_WindowWidthPtr, 0.0f, (float)*m_WindowHeightPtr, -1.0f, 1.0f);
			canvas.view = glm::translate(glm::mat4(1.0f), glm::vec3(0, 0, 0));
			canvas.labelGrid.rows = 10;
			canvas.labelGrid.columns = 10;
			canvas.wwidth = *m_WindowWidthPtr;
			canvas.wheight = *m_WindowHeightPtr;
			canvas.initCanvasLabels();
			canvas.initOpenGLData();

			//Init velocityfield
			vf.proj = canvas.proj;
			vf.view = canvas.view;
			vf.wwidth = *m_WindowWidthPtr;
			vf.wheight = *m_WindowHeightPtr;
			vf.u.columns = canvas.labelGrid.columns + 1;
			vf.u.rows = canvas.labelGrid.rows;
			vf.v.columns = canvas.labelGrid.columns;
			vf.v.rows = canvas.labelGrid.rows + 1;
			vf.initVelocityField();
			vf.initOpenGLData();
		};

		~TestGrid()
		{
		
		};

		void render() override
		{
			canvas.render();	
			vf.render();
		};


		void onImGuiRender() override
		{
			//	ImGui::SliderFloat2("Translation B", &m_TranslationB.x, -max_dim / 2, max_dim/2);

			ImGui::SliderFloat("Vis Scale", &vf.vis_scale, 0.01f, 10.0f);
			ImGui::Checkbox("Draw Grid Edges", &canvas.drawEdges);
			ImGui::Checkbox("Draw Vel Field Edge", &vf.drawFieldEdgeWise);
			ImGui::Checkbox("Draw Vel Field Center", &vf.centeredVF.drawFieldCenterWise);
			ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
		}

		void step(float dt) override
		{
			vf.updateOpenGLData();
			/*
			int maxh = *m_WindowHeightPtr / 2;
			int maxw = *m_WindowWidthPtr / 2;
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
					p.setPosition(glm::vec2(p.getPosition().x, -maxh));
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
			*/

		}

		void createRandomParticle(int& x, int& y)
		{
			x = 0;
			y = 0;
			Particle p(glm::vec2(x, y), 10.0f);

		};

	};



}

