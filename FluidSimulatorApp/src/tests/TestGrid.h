#pragma once
#pragma once

#include "Test.h"
#include "imgui.h"
#include "Particle.h"
#include "Grid.h"
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <memory>
#include "Canvas.h"
#include "VelocityField.h"
#include <Eigen/src/IterativeLinearSolvers/ConjugateGradient.h>
#include <limits>

namespace test {

	class TestGrid : public Test
	{

	private:
		Canvas2D canvas;
		VelocityField2D vf;
		float gravityForce[2] = { 0.0f, -100.0f };
		bool run_sim = false;
		bool step_sim = false;
		std::vector<Particle> particles;

		bool centered = false;
		bool cornered = true;
		bool bottomed = false;

		bool drawParticles = false;

		bool classifyCells = true;
		bool particleAdvection = false;
		bool semiLAdvection = true;
		bool bodyForces = true;
		bool pressureProjection = true;

	public:
		TestGrid(unsigned int * heightPtr, unsigned int * widthPtr) : 
			Test(heightPtr, widthPtr)
		{
			//Init canvas
			setUpGrid(true);
			
		};

		/*
		Sets up grid including the canvas grid that contains cell labels, as well as the velocity field grids
		*/
		void setUpGrid(bool setUPOpenGLObjects)
		{
			canvas.proj = glm::ortho(0.0f, (float)*m_WindowWidthPtr, 0.0f, (float)*m_WindowHeightPtr, -1.0f, 1.0f);
			canvas.view = glm::translate(glm::mat4(1.0f), glm::vec3(0, 0, 0));
			canvas.labelGrid.rows = 31;
			canvas.labelGrid.columns = 31;
			canvas.wwidth = *m_WindowWidthPtr;
			canvas.wheight = *m_WindowHeightPtr;
			if (centered)
			{
				canvas.initCanvasLabelsCentered();
				centered = false;
			}
			else if (cornered)
			{
				canvas.initCanvasLabelsCornered();
				cornered = false;
			}
			else if (bottomed)
			{
				canvas.initCanvasLabelsBottomed();
				bottomed = false;
			}
			if (setUPOpenGLObjects)
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

			if (setUPOpenGLObjects)
				vf.initOpenGLData();

			initParticles(4);
		}
		
		/*
		Clears all grid values
		*/
		void clear()
		{
			particles.clear();
			canvas.clear();
			vf.clear();
		}

		void render() override
		{
			
			canvas.render();	
			vf.render();
			float particle_color[4] = { 0.0f, 0.0f, 0.0f, 1.0f };
			if (drawParticles)
			{
				for (auto& p : particles)
				{
					p.render(canvas.proj, canvas.view, particle_color);
				}
			}
		};

		void initParticles(int numParticlesPerCell)
		{
			Grid2D<unsigned int>& lGrid = canvas.labelGrid;
			float x, y;
			float div = 1.0f / static_cast <float> (RAND_MAX);;
			for (int i = 0; i < lGrid.rows; i++)
			{
				for (int j = 0; j < lGrid.columns; j++)
				{
					if (lGrid.getIndex(i, j) == Label::LIQUID)
					{
						for (int n = 0; n < numParticlesPerCell; n++)
						{
							x = j * vf.dx + static_cast<float>(rand())*div*vf.dx;
							y = i * vf.dx + static_cast<float>(rand())*div*vf.dx;
							Particle p(glm::vec2(x, y), 10.0f);
							particles.push_back(p);
						}
					}
				}
			}
		}

		void onImGuiRender() override
		{
			//	ImGui::SliderFloat2("Translation B", &m_TranslationB.x, -max_dim / 2, max_dim/2);
			ImGui::Checkbox("Centered", &centered); ImGui::Checkbox("Cornered", &cornered); ImGui::Checkbox("Bottomed", &bottomed);
			ImGui::Checkbox("Run", &run_sim); ImGui::SameLine();
			ImGui::Checkbox("Step", &step_sim);
			ImGui::Text("Simulation Components");
			ImGui::Checkbox("SemiLagrangian Adv", &semiLAdvection);
			ImGui::Checkbox("Pressure Solve", &pressureProjection);
			ImGui::Checkbox("Add Body Forces", &bodyForces);
			ImGui::Checkbox("Advect Particles", &particleAdvection);
			ImGui::Checkbox("Classify Cells", &classifyCells);
	
			ImGui::SliderFloat("Vis Scale", &vf.vis_scale, 0.01f, 10.0f);
			ImGui::Checkbox("Draw Particles", &drawParticles);
			ImGui::Checkbox("Draw Grid Edges", &canvas.drawEdges);
			ImGui::Checkbox("Draw Vel Field Edge", &vf.drawFieldEdgeWise);
			ImGui::Checkbox("Draw Vel Field Center", &vf.centeredVF.drawFieldCenterWise);
			ImGui::SliderFloat2("Gravity",  gravityForce, -1000, 1000);
			ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
		}

		/*
		Steps the system by timestep dt
		*/
		void step(float dt) override
		{
			vf.updateOpenGLData();
			if (cornered || centered || bottomed)
			{
				clear();
				setUpGrid(false);
			}
			if (run_sim || step_sim)
			{
				if (step_sim) step_sim = false;
				//advectCellLabels(dt);
				if (semiLAdvection)
					advectSLA(dt);
				if (bodyForces)
					addBodyForces(dt);
				if (pressureProjection)
					pressureSolve(dt);
			
				vf.swapBuffers();

				if (particleAdvection)
					advectParticles(dt);
				if (classifyCells)
					reclassifyCells();
			}
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
		Classifies cells by looping through particles, getting which cell they belong to, and switching that to a liquid cell
		*/
		void reclassifyCells()
		{
			Eigen::Vector2f pos;
			GridCoordinates pCoords; 

			// First clear all cells that aren't solids and set them to air
			for (int i = 0; i < canvas.labelGrid.rows; i++)
			{
				for (int j = 0; j < canvas.labelGrid.columns; j++)
				{
					if (canvas.labelGrid.getIndex(i, j) != Label::SOLID)
					{
						canvas.labelGrid.setIndex(i, j, Label::AIR);
					}
				}
			}
			for (auto& p : particles)
			{
				pos(0) = p.getPosition().x;
				pos(1) = p.getPosition().y;

				pCoords.i = (int)(pos.y() / vf.dx);
				pCoords.j = (int)(pos.x() / vf.dx);

				canvas.labelGrid.setIndex(pCoords.i, pCoords.j, Label::LIQUID);

			}
		}

		/*
		Loops through all edges, checks if it is at the boundary of liquid and something else. If so, updates backbuffer velocity.
		*/
		void addBodyForces(float dt)
		{
			Label l1, l2;
			
			// Increment horizontal component (u component)
			for (int i = 0; i < canvas.labelGrid.rows; i++)
			{
				for (int j = 1; j < canvas.labelGrid.columns ; j++)
				{
					Label l1 = static_cast<Label>(canvas.labelGrid.getIndex(i, j));
					Label l2 = static_cast<Label>(canvas.labelGrid.getIndex(i, j-1));
					if (l1 == Label::LIQUID || l2 == Label::LIQUID) //u component of velocity updated
					{
						vf.u_backbuffer.incrementIndex(i, j, dt * gravityForce[0]);
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
						vf.v_backbuffer.incrementIndex(i, j, dt * gravityForce[1]);
					}
				}
			}
		}

		/*
		Advects the cell labels
		*/
		void advectCellLabels(float dt)
		{
			Eigen::Vector2f currPos, prevPos, currVel;
			Label currLabel, prevLabel;
			GridCoordinates prevCoords;
			//Count how many fluid cells exist
			Grid2D<unsigned int>& lGrid = canvas.labelGrid;
			for (int i = 0; i < lGrid.rows; i++)
			{
				for (int j = 0; j < lGrid.columns; j++)
				{
				
					currLabel = static_cast<Label>(lGrid.getIndex(i, j));
					
					currPos(0) = j * vf.dx + vf.dx * 0.5f; //currPos is center of each cell
					currPos(1) = i * vf.dx + vf.dx * 0.5f;
					
					currVel(0) = vf.u.getIndex(i, j) + vf.u.getIndex(i, j + 1); //velocity at grid center
					currVel(1) = vf.v.getIndex(i, j) + vf.v.getIndex(i + 1, j);
					currVel *= 0.5f;

					prevPos = currPos - dt * currVel;

					prevCoords.i = (int)(prevPos.y() / vf.dx);
					prevCoords.j = (int)(prevPos.x() / vf.dx);

					prevLabel = static_cast<Label>(canvas.labelGrid.getIndex(prevCoords.i, prevCoords.j));

					canvas.labelGrid_backbuffer.setIndex(i, j, static_cast<unsigned int>(prevLabel));

					
				}
			}
			canvas.swapBuffers();
		}

		/*
		Moves all particles in the scene by their interpolated velocity
		*/
		void advectParticles(float dt)
		{
			Eigen::Vector2f currPos, nextPos, vel;
			for (auto& p : particles)
			{
				currPos(0) = p.getPosition().x;
				currPos(1) = p.getPosition().y;

				vel = vf.getVelocity(currPos);

				nextPos = currPos + dt * vel;

				p.setPosition(glm::vec2(nextPos.x(), nextPos.y()));
			}
		}

		/*
		Solves pressure to ensure velocity field is divergence free
		*/
		void pressureSolve(float dt)
		{
			//Count how many fluid cells exist. Also give all fluid cells a cell index
			Grid2D<unsigned int>& lGrid = canvas.labelGrid;
			int numFluidCells = 0;
			Grid2D<int> iGrid;//will hold -1 if cell is not a liquid, otherwise will hold liquid index
			iGrid.rows =lGrid.rows; iGrid.columns = iGrid.rows;
			iGrid.data = std::vector<int>(iGrid.rows * iGrid.columns); 
			for (int i = 0; i < iGrid.rows; i++)
			{
				for (int j = 0; j < iGrid.columns; j++)
				{
					if (lGrid.getIndex(i, j) == Label::LIQUID)
					{
						iGrid.setIndex(i, j, numFluidCells);
						numFluidCells++;
					}
					else
					{
						iGrid.setIndex(i, j, -1);
					}
				}
			}

			//Set up b vector, which will hold current divergence of v field
			Eigen::VectorXf b(numFluidCells);
			
			float negDivergence = 0;
			float dxinv = 1 / vf.dx;

			int index = 0;
			//get divergence of each fluid cell, plug into b vector
			for (int i = 0; i < lGrid.rows; i++)
			{
				for (int j = 0; j < lGrid.columns; j++)
				{
					if (lGrid.getIndex(i, j) == Label::LIQUID)
					{
						//calculate divergence
						negDivergence = divergenceAtCell(i, j, dxinv);
						
						if (lGrid.getIndex(i - 1, j) == Label::SOLID)
						{
							negDivergence -= dxinv * vf.v_backbuffer.getIndex(i, j);
						}
						if (lGrid.getIndex(i + 1, j) == Label::SOLID)
						{
							negDivergence += dxinv * vf.v_backbuffer.getIndex(i + 1, j);
						}
						if (lGrid.getIndex(i, j - 1) == Label::SOLID)
						{
							negDivergence -= dxinv * vf.u_backbuffer.getIndex(i, j);
						}
						if (lGrid.getIndex(i, j + 1) == Label::SOLID)
						{
							negDivergence += dxinv * vf.u_backbuffer.getIndex(i, j + 1);
						}
						b(index) = negDivergence;
						index++;
					}
					
				}
			}

			index = 0;
			Eigen::SparseMatrix<float> A(numFluidCells, numFluidCells);
			float scale =  dt*dxinv*dxinv;// assuming density of 1
			int numNonSolidNeighbors = 0;
			Label l1, l2, l3, l4;
			for (int i = 0; i < lGrid.rows; i++)
			{
				for (int j = 0; j < lGrid.columns; j++)
				{
					
					if (lGrid.getIndex(i, j) == Label::LIQUID)
					{
						l1 = static_cast<Label>(lGrid.getIndex(i - 1, j));
						l2 = static_cast<Label>(lGrid.getIndex(i + 1, j));
						l3 = static_cast<Label>(lGrid.getIndex(i, j - 1));
						l4 = static_cast<Label>(lGrid.getIndex(i, j + 1));

						numNonSolidNeighbors = 0;
						if (l1 != Label::SOLID)
						{
							numNonSolidNeighbors++;
							if (l1 == Label::LIQUID)
							{
								A.insert(iGrid.getIndex(i - 1, j), index) = scale;
							}
						}
						if (l2!= Label::SOLID)
						{
							numNonSolidNeighbors++;
							if (l2 == Label::LIQUID)
							{
								A.insert(iGrid.getIndex(i + 1, j), index) = scale;
							}
						}
						if (l3 != Label::SOLID)
						{
							numNonSolidNeighbors++;
							if (l3 == Label::LIQUID)
							{
								A.insert(iGrid.getIndex(i, j - 1), index) = scale;
							}
						}
						if (l4 != Label::SOLID)
						{
							numNonSolidNeighbors++;
							if (l4 == Label::LIQUID)
							{
								A.insert(iGrid.getIndex(i, j + 1), index) = scale;
							}
						}
						A.insert(index, index) = -numNonSolidNeighbors*scale;
						index++;
					}
				}
			}

			Eigen::MatrixXf A_d;
			A_d = Eigen::MatrixXf(A);
			Eigen::ConjugateGradient<Eigen::SparseMatrix<float>> solver;
			solver.setMaxIterations(100);
			solver.compute(A);

			Eigen::VectorXf p = solver.solve(b);
			float pressureDif = 0.0f;
			scale = dt / vf.dx; //assuming density of 1

			for (int i = 1; i < lGrid.rows; i++)
			{
				for (int j = 1; j < lGrid.columns; j++)
				{

					//update u pressure grad
					if (lGrid.getIndex(i, j - 1) == Label::LIQUID || lGrid.getIndex(i, j) == Label::LIQUID)
					{
						if (lGrid.getIndex(i, j - 1) == Label::SOLID || lGrid.getIndex(i, j) == Label::SOLID)
						{
							vf.u_backbuffer.setIndex(i, j, 0.0f);
						}
						else
						{
							if (lGrid.getIndex(i, j) == Label::AIR)
							{
								pressureDif = -p(iGrid.getIndex(i, j - 1));
							}
							else if (lGrid.getIndex(i, j - 1) == Label::AIR)
							{
								pressureDif = p(iGrid.getIndex(i, j));
							}
							else
							{
								pressureDif = p(iGrid.getIndex(i, j)) - p(iGrid.getIndex(i, j - 1));
							}
							vf.u_backbuffer.incrementIndex(i, j, -scale * pressureDif);
						}
					}
					else
					{
						vf.u_backbuffer.setIndex(i, j, 0.0f);// std::numeric_limits<float>::max());
					}

					//update v  pressure grad
					if (lGrid.getIndex(i - 1, j) == Label::LIQUID || lGrid.getIndex(i, j) == Label::LIQUID)
					{
						if (lGrid.getIndex(i - 1, j) == Label::SOLID || lGrid.getIndex(i, j) == Label::SOLID)
						{
							vf.v_backbuffer.setIndex(i, j, 0.0f);
						}
						else
						{
							if (lGrid.getIndex(i, j) == Label::AIR)
							{
								pressureDif = -p(iGrid.getIndex(i - 1, j));
							}
							else if (lGrid.getIndex(i - 1, j) == Label::AIR)
							{
								pressureDif = p(iGrid.getIndex(i, j));
							}
							else
							{
								pressureDif = p(iGrid.getIndex(i, j)) - p(iGrid.getIndex(i-1, j));
							}
							vf.v_backbuffer.incrementIndex(i, j, -scale * pressureDif);
						}
					}
					else
					{
						vf.v_backbuffer.setIndex(i, j, 0.0f); //std::numeric_limits<float>::max());
					}



					

				}
			}
		}

		/*
		Calculates divergence at cell i, j by looking at nearby edges
		*/
		float divergenceAtCell(int i, int j, float dxinv)
		{
			float div_x = vf.u_backbuffer.getIndex(i, j + 1) - vf.u_backbuffer.getIndex(i, j);
			float div_y = vf.v_backbuffer.getIndex(i + 1, j) - vf.v_backbuffer.getIndex(i, j);
			return (div_x + div_y)*dxinv;
		}
		void createRandomParticle(int& x, int& y)
		{
			x = 0;
			y = 0;
			Particle p(glm::vec2(x, y), 10.0f);

		};

	};



}

