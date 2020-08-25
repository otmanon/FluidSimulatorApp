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
			canvas.labelGrid.rows = 31;
			canvas.labelGrid.columns = 31;
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

		/*
		Steps the system by timestep dt
		*/
		void step(float dt) override
		{
			vf.updateOpenGLData();
			advectSLA(dt);
			addBodyForces(dt);
			vf.swapBuffers();
		}

		/*
		Advects velocity field using semi lagrangian advection
		*/
		void advectSLA(float dt)
		{
			
			Eigen::Vector2f currPos, prevPos, currVel;
			float prevU, prevV;

			//Loop through all u edges
			for (int i = 0; i < vf.u.rows; i++)
			{
				for (int j = 0; j < vf.u.columns; j++)
				{
					//look at point in middle
					currPos(0) = (float)j * vf.dx;
					currPos(1) = (float)i * vf.dx + vf.dx / 2.0f;
					
					//get velocity at current position
					currVel(0) = vf.u.getIndex(i, j);
					currVel(1) = vf.getVelocityV(currPos);

					//trace backwards in time
					prevPos = currPos - dt * currVel;

					//make sure prevPos is within bounds
					projectPositionToWindow(prevPos);

					//interpolate velocity then
					prevU = vf.getVelocityU(prevPos);

					//set backbuffer velocity to the one found. 
					vf.u_backbuffer.setIndex(i, j, prevU);

				}
			}

			//Loop through all v edges
			for (int i = 0; i < vf.v.rows; i++)
			{
				for (int j = 0; j < vf.v.columns; j++)
				{
					//look at point in middle of edge
					currPos(0) = (float) j * vf.dx + vf.dx / 2.0f;
					currPos(1) = (float) i * vf.dx;

					//get velocity at current position
					currVel(0) = vf.getVelocityU(currPos);
					currVel(1) = vf.v.getIndex(i, j);

					//trace backwards in time
					prevPos = currPos - dt * currVel;

					//make sure prevPos is within bounds
					projectPositionToWindow(prevPos);

					//interpolate velocity then
					prevV = vf.getVelocityV(prevPos);

					//set backbuffer velocity to the one found. 
					vf.v_backbuffer.setIndex(i, j, prevV);

				}
			}
		}

		
		/*
		Projects pos to be within window limitations.
		*/
		void projectPositionToWindow(Eigen::Vector2f& pos)
		{
			
			float epsilon = 1.0f;
			if (pos.x() < 0.0f) {
				pos(0) = 0.0f + epsilon;
			}
			else if (pos.x() > *m_WindowHeightPtr)
			{
				pos(0) = *m_WindowHeightPtr - epsilon;
			}
			if (pos.y() < 0.0f)
			{
				pos(1) = 0.0f + epsilon;
			}
			if (pos.y() > *m_WindowWidthPtr)
			{
				pos(1) = *m_WindowWidthPtr - epsilon;
			}
			
		}

		/*
		Loops through all edges, checks if it is at the boundary of liquid and something else. If so, updates backbuffer velocity.
		*/
		void addBodyForces(float dt)
		{
			Label l1, l2;
			Eigen::Vector2f gravityForce(0.0f, -100.0f);
			// Increment horizontal component (u component)
			for (int i = 0; i < canvas.labelGrid.rows; i++)
			{
				for (int j = 1; j < canvas.labelGrid.columns ; j++)
				{
					Label l1 = static_cast<Label>(canvas.labelGrid.getIndex(i, j));
					Label l2 = static_cast<Label>(canvas.labelGrid.getIndex(i, j-1));
					if (l1 == Label::LIQUID || l2 == Label::LIQUID) //u component of velocity updated
					{
						vf.u_backbuffer.incrementIndex(i, j, dt * gravityForce.x());
					}
				}
			}

			for (int j = 0; j < canvas.labelGrid.columns; j++)
			{
				// Increment vertical component (v component)
				for (int i = 1; i < canvas.labelGrid.rows; i++)
				{
					Label l1 = static_cast<Label>(canvas.labelGrid.getIndex(i, j));
					Label l2 = static_cast<Label>(canvas.labelGrid.getIndex(i - 1, j));
					if (l1 == Label::LIQUID || l2 == Label::LIQUID) //u component of velocity updated
					{
						vf.v_backbuffer.incrementIndex(i, j, dt * gravityForce.y());
					}
				}
			}
		}

		void createRandomParticle(int& x, int& y)
		{
			x = 0;
			y = 0;
			Particle p(glm::vec2(x, y), 10.0f);

		};

	};



}

