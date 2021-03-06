#pragma once
#include "Drawable.h"
#include "glm.hpp"
#include "Grid.h"

enum Label {
	SOLID, AIR, LIQUID
};
/*
Canvas used for displaying labels on screen
*/
struct Canvas2D : public Drawable
{

	/*
	Window hight in pixels
	*/
	float wheight;

	/*
	Window width in pixels
	*/
	float wwidth;

	/*
	Projection and view matrices used for rendering
	*/
	glm::mat4 proj, view;

	/*
	2D grid of cell labels;
	0 if solid, 1 if air, 2 if liquid
	*/
	Grid2D<unsigned int> labelGrid;

	Grid2D<unsigned int> labelGrid_backbuffer;

	/*
	Cell width/Cell Height
	*/
	float dx;

	/*
	Vertices for canvas (4 vs for a square)
	*/
	float vertices[8];

	/*
	Indices for canvas (2 triangles, 3 vertices for triangle)
	*/
	unsigned int indices[6];

	/*
	Whether or not to draw grid edges in black
	*/
	bool drawEdges = true;


	/*
	Draw Fluid Cells
	*/
	bool drawFluidCells = false;

	/*
	Initializes cell labels, so that liquid is at center of grid, with walls surrounding border
	*/
	void initCanvasLabelsCentered()
	{
		int rows = labelGrid.rows;
		int columns = labelGrid.columns;
		dx = wwidth / rows;
		//fill label grid
		for (int i = 0; i < rows; i++)
		{
			for (int j = 0; j < columns; j++)
			{
				if (i == 0 || i == rows - 1 || j == 0 || j == columns - 1) // on border
				{
					labelGrid.data.push_back(Label::SOLID);
				}
				else if (j < labelGrid.columns / 2 + labelGrid.columns / 4 && j > labelGrid.columns / 2 - labelGrid.columns / 4
					&& i < labelGrid.rows / 2 + labelGrid.rows / 4 && i > labelGrid.rows/2 - labelGrid.columns/4)
				{
					labelGrid.data.push_back(Label::LIQUID);
				}
				else
				{
					labelGrid.data.push_back(Label::AIR);
				}
			}
		}
		labelGrid_backbuffer = labelGrid;
	}

	/*
	Initializes cell labels so that liquid is at bottom left corner of grid
	*/
	void initCanvasLabelsCornered()
	{
		int rows = labelGrid.rows;
		int columns = labelGrid.columns;
		dx = wwidth / rows;
		//fill label grid
		for (int i = 0; i < rows; i++)
		{
			for (int j = 0; j < columns; j++)
			{
				if (i == 0 || i == rows - 1 || j == 0 || j == columns - 1) // on border
				{
					labelGrid.data.push_back(Label::SOLID);
				}
				else if (j < labelGrid.columns / 2 && j >= 1
					&& i < labelGrid.rows / 2  && i >= 1)
				{
					labelGrid.data.push_back(Label::LIQUID);
				}
				else
				{
					labelGrid.data.push_back(Label::AIR);
				}
			}
		}
		labelGrid_backbuffer = labelGrid;
	}


	/*
	Initializes cell labels so that liquid is at bottom middle part of grid
	*/
	void initCanvasLabelsBottomed()
	{
		int rows = labelGrid.rows;
		int columns = labelGrid.columns;
		dx = wwidth / rows;
		//fill label grid
		for (int i = 0; i < rows; i++)
		{
			for (int j = 0; j < columns; j++)
			{
				if (i == 0 || i == rows - 1 || j == 0 || j == columns - 1) // on border
				{
					labelGrid.data.push_back(Label::SOLID);
				}
				else if (j < labelGrid.columns / 2 + labelGrid.columns / 4 &&  j > labelGrid.columns / 2 - labelGrid.columns/4 
					&& i < labelGrid.rows / 2  && i >= 1)
				{
					labelGrid.data.push_back(Label::LIQUID);
				}
				else
				{
					labelGrid.data.push_back(Label::AIR);
				}
			}
		}
		labelGrid_backbuffer = labelGrid;
	}


	/*
	prepares the openGL	objects studd
	*/
	void initOpenGLData()
	{
		//4 vertices to render canvas... vertices are in CCW order
		vertices[0] = 0.0f;
		vertices[1] = 0.0f;
		vertices[2] = wwidth;
		vertices[3] = 0.0f;
		vertices[4] = wwidth;
		vertices[5] = wheight;
		vertices[6] = 0.0f;
		vertices[7] = wheight;

		//indices... split canvas into two triangles.
		indices[0] = 0;
		indices[1] = 1;
		indices[2] = 2;
		indices[3] = 0;
		indices[4] = 2;
		indices[5] = 3;

		std::string filepath = "shaders/Grid.shader";
		buildOpenGLObjects(vertices, indices, 2, 8, 6, true, filepath);
		m_sh->SetUniform1uiv("u_LabelGrid", labelGrid.data);
		m_sh->SetUniform1f("u_dx", dx);
		m_sh->SetUniform1f("u_Rows", labelGrid.rows);
		m_sh->SetUniform1f("u_Columns", labelGrid.columns);
		m_sh->SetUniform1i("u_DrawFluidCells", drawFluidCells);
	}

	/*
	Clears canvas data
	*/
	void clear()
	{
		labelGrid.data.clear();
		labelGrid.rows = 0;
		labelGrid.columns = 0;
	}
	/*
	Swaps buffers for labelgrid
	*/
	void swapBuffers()
	{
		labelGrid = labelGrid_backbuffer;
	}
	void render()
	{
		Canvas2D::bind();

		glm::mat4 mvp = proj;
		m_sh->SetUniform1uiv("u_LabelGrid", labelGrid.data);
		m_sh->SetUniformMat4f("u_MVP", mvp);
		m_sh->SetUniform1i("u_DrawEdges", drawEdges);
		m_sh->SetUniform1i("u_DrawFluidCells", drawFluidCells);
		GLCall(glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, nullptr));
	}
};