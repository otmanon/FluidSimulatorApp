#pragma once
#include "Drawable.h"
#include "Grid.h"

/*
Struct used to display velocity field at each cell
*/
struct CenteredVelocityDisplay2D : public Drawable
{

	/*
	Projection and view matrices used for rendering
	*/
	glm::mat4 proj, view;

	/*
	Vertices for canvas (4 vs for a square)
	*/
	std::vector<float> vertices;

	/*
	Indices for drawing velocity field lines
	*/
	std::vector<unsigned int> indices;

	/*
	Whether or not to draw field centerwise
	*/
	bool drawFieldCenterWise = true;
public:

	void initOpenGLData()
	{
		std::string filepath = "shaders/VF.shader";
		buildOpenGLObjects(vertices.data(),indices.data(), 2, vertices.size(), indices.size(), true, filepath);
		m_sh->SetUniform1i("u_DrawFieldCenterWise", drawFieldCenterWise);

	}

	void updateOpenGLData()
	{
		updateVertexBuffer(vertices.data(), indices.data(), 2, vertices.size(), indices.size());
	}

	void render()
	{

		CenteredVelocityDisplay2D::bind();

		glm::mat4 mvp = proj;
		m_sh->SetUniformMat4f("u_MVP", mvp);

		m_sh->SetUniform1i("u_DrawFieldCenterWise", drawFieldCenterWise);
		
		GLCall(glDrawElements(GL_LINES, m_ib->GetCount(), GL_UNSIGNED_INT, nullptr));
	}
};
struct VelocityField2D : public Drawable
{
public:
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
	u component of velocity (x dir). 2D grid
	*/
	Grid2D<float> u;
	Grid2D<float> u_backbuffer;
	/*
	v component of velocity (y dir). 2D grid
	*/
	Grid2D<float> v;
	Grid2D<float> v_backbuffer;
	/*
	Cell width/Cell Height
	*/
	float dx;

	/*
	Vertices for canvas (4 vs for a square)
	*/
	std::vector<float> vertices;

	/*
	Indices for drawing velocity field lines
	*/
	std::vector<unsigned int> indices;

	/*
	Scale for visualizing velocity field;
	*/
	float vis_scale = 1.0f;
	
	/*
	Draw velocity field from edges
	*/
	bool drawFieldEdgeWise= false;

	

	CenteredVelocityDisplay2D centeredVF;



public:
	/*
	Initializes velocity field to 0
	*/
	void initVelocityField()
	{
		dx = wwidth / u.rows;
		u.data.reserve(u.columns * u.rows);
		//initialize u velocity component
		for (int i = 0; i < u.rows; i++)
		{
			for (int j = 0; j < u.columns; j++)
			{
				u.data.emplace_back(0.0f); //initialize all vels to zero
			}
		}
		u_backbuffer = u;

		v.data.reserve(v.columns * v.rows);
		//initialize u velocity component
		for (int i = 0; i < v.rows; i++)
		{
			for (int j = 0; j < v.columns; j++)
			{
				
				v.data.emplace_back(0.0f); //initialize all vels to zero
				

			}
		}
		v_backbuffer = v;


	}



	/*
	prepares the openGL	data... lines will go from edge midpoints to appropriate direction
	*/
	void initOpenGLData()
	{
		vertices = std::vector<float>(2 * u.rows*u.columns * 2 + 2 * v.rows * v.columns * 2);
		indices = std::vector<unsigned int>(2 * u.rows*u.columns + 2 * v.rows*v.columns);
		centeredVF.vertices = std::vector<float>(2 * u.rows*v.columns * 2);
		centeredVF.indices = std::vector<unsigned int>(u.rows*v.columns * 2);
		buildVerticesEdgeWise();
		buildVerticesCenterWise();
		std::string filepath = "shaders/VF.shader";

		
		buildOpenGLObjects(vertices.data(), indices.data(), 2, vertices.size(), indices.size(), true, filepath);
		m_sh->SetUniform1i("u_DrawFieldEdgeWise", drawFieldEdgeWise);

		centeredVF.initOpenGLData();
		centeredVF.proj = proj;
		centeredVF.view = view;

	}

	/*
	update Vertex/Index data for OpenGL rendering
	*/
	void updateOpenGLData()
	{

		buildVerticesEdgeWise();
		buildVerticesCenterWise();
		updateVertexBuffer(vertices.data(), indices.data(), 2, vertices.size(), indices.size());

		centeredVF.updateOpenGLData();
	}

	/*
	Gets Velocity From VelocityField by bilinearly interpolating uGrid and vGrid separately
	*/
	Eigen::Vector2f getVelocity(Eigen::Vector2f pos)
	{
		
		//bilinearly interpolate u's
		float u_final = getVelocityU(pos);
		float v_final = getVelocityV(pos);
		
		return Eigen::Vector2f(u_final, v_final);
							
	}

	/*
	Gets u component of velocity at a point by interpolating the grid
	*/
	float getVelocityU(Eigen::Vector2f pos)
	{
		GridCoordinates blCoords; //u edge bottom left grid coords of cell
		GridCoordinates trCoords; // u edge at top left grid coords of cell

		blCoords.j = (int)(pos.x() / dx);
		blCoords.i = (int)((pos.y() - dx / 2.0f) / dx);

		trCoords.j = blCoords.j + 1;
		trCoords.i = blCoords.i + 1;

		Eigen::Vector2f blPos(blCoords.j*dx, blCoords.i*dx + dx / 2.0f); // position of bottom left vertex (even if not in window)

		projectUIndicesToBoundary(blCoords, trCoords);

		float r_x = (pos.x() - blPos.x()) / dx; //fraction of distance in cell beyond bottom x
		float r_y = (pos.y() - blPos.y()) / dx; // fraction of distance in cell above bottom y

		//bilinear interpolation
		float u_x1, u_x2, u_final;
		u_x1 = u.getIndex(blCoords.i, blCoords.j)*(1 - r_x) + u.getIndex(blCoords.i, trCoords.j)*r_x;
		u_x2 = u.getIndex(trCoords.i, blCoords.j)*(1 - r_x) + u.getIndex(trCoords.i, trCoords.j)*r_x;
		u_final = u_x1 * (1 - r_y) + u_x2 * r_y;
		return u_final;
	}

	/*
	Gets v component of velocity at a point by interpolating the grid
	*/
	float getVelocityV(Eigen::Vector2f pos)
	{
		GridCoordinates blCoords; //u edge bottom left grid coords of cell
		GridCoordinates trCoords; // u edge at top left grid coords of cell
		//bilinearly interpolate v's
		blCoords.i = (int)(pos.y() / dx);
		blCoords.j = (int)((pos.x() - dx / 2.0f) / dx);
		trCoords.j = blCoords.j + 1;
		trCoords.i = blCoords.i + 1;
		Eigen::Vector2f blPos = Eigen::Vector2f(blCoords.j*dx + dx / 2.0f, blCoords.i*dx);

		projectVIndicesToBoundary(blCoords, trCoords);
		float r_x = (pos.x() - blPos.x()) / dx; //fraction of distance in cell beyond bottom x
		float r_y = (pos.y() - blPos.y()) / dx; // fraction of distance in cell above bottom y

			//bilinear interpolation
		float v_x1, v_x2, v_final;
		v_x1 = v.getIndex(blCoords.i, blCoords.j)*(1 - r_x) + v.getIndex(blCoords.i, trCoords.j)*r_x;
		v_x2 = v.getIndex(trCoords.i, blCoords.j)*(1 - r_x) + v.getIndex(trCoords.i, trCoords.j)*r_x;
		v_final = v_x1 * (1 - r_y) + v_x2 * r_y;

		return v_final;
	}

	/*
	Helper fucntion. Given two indices (bottom left and top right) on the u grid, projects them to within the u_grid boundary
	*/
	void projectUIndicesToBoundary(GridCoordinates& bl, GridCoordinates& tr)
	{
		if (bl.i < 0)
			bl.i = 0;
		if (tr.i > u.rows - 1)
			tr.i = u.rows - 1;
		if (bl.j < 0)
			bl.j = 0;
		if (tr.j > u.columns - 1)
			tr.j = u.columns - 1;

	}

	/*
	Helper fucntion. Given two indices (bottom left and top right) on the v grid, projects them to within the u_grid boundary
	*/
	void projectVIndicesToBoundary(GridCoordinates& bl, GridCoordinates& tr)
	{
		if (bl.j < 0)
			bl.j = 0;
		if (tr.j > v.columns - 1)
			tr.j = v.columns - 1;
		if (bl.i < 0)
			bl.i = 0;
			if (tr.i > v.rows - 1)
				tr.i = v.rows - 1;
	}



	/*
	Constructs vertices and indices lists that will be used for rendering Velocity Field
	Velocity starts at center of each cell, and goes to velocity_dir * v
	*/
	void buildVerticesCenterWise()
	{

		//float x1, x2, y1, y2;
		Eigen::Vector2f vel;
		Eigen::Vector2f pos1;
		Eigen::Vector2f pos2;
		int index_index = 0;
		int vertex_index = 0;
		for (int i = 0; i < u.rows; i++) //loop through each cell
		{
			for (int j = 0; j < v.columns; j++)
			{
				
				pos1(0) = j * dx + dx*0.5f;
				pos1(1) = i * dx + dx * 0.5f; //center of each cell

				vel = Eigen::Vector2f(0.5f*(u.getIndex(i, j) + u.getIndex(i, j + 1)), 0.5f*(v.getIndex(i + 1, j) + v.getIndex(i, j))); //average of 4 edges//getVelocity(pos1);

				pos2(0) = pos1.x() + vel.x() * vis_scale;
				pos2(1) = pos1.y() + vel.y() * vis_scale; //center of each cell
				
				centeredVF.vertices[vertex_index + 0] = pos1.x();
				centeredVF.vertices[vertex_index + 1] = pos1.y();
				centeredVF.vertices[vertex_index + 2] = pos2.x();
				centeredVF.vertices[vertex_index + 3] = pos2.y();
				vertex_index += 4;

				centeredVF.indices[index_index + 0] = (unsigned int) index_index;
				centeredVF.indices[index_index + 1] = (unsigned int) index_index + 1;
				index_index += 2;
				
			}
		}
	}

	/*
	Constructs vertices and indices lists that will be used for rendering Velocity Field
	*/
	void buildVerticesEdgeWise()
	{
		
		float x1, x2, y1, y2;
		int index_index = 0;
		int vertex_index = 0;
		//Draw u component of vel (corresponds to vertical edges/ edges pointing horizontally)
		for (int i = 0; i < u.rows; i++)
		{
			for (int j = 0; j < u.columns; j++)
			{
				//first point
				x1 = j * dx;
				y1 = i * dx + dx / 2.0f;

				//second point is shifted to the right a little bit
				x2 = x1 + u.getIndex(i, j)*vis_scale;
				y2 = y1;
				vertices[vertex_index + 0] = x1;
				vertices[vertex_index + 1] = y1;
				vertices[vertex_index + 2] = x2;
				vertices[vertex_index + 3] = y2;
				vertex_index += 4;

				indices[index_index + 0] = index_index;
				indices[index_index + 1] = index_index + 1;
		
				index_index += 2;
			}
		}

		//Draw v component of vel (corresponds to horizontal edges/ edges pointing vertically)
		for (int i = 0; i < v.rows; i++)
		{
			for (int j = 0; j < v.columns; j++)
			{
				//first point
				x1 = j * dx + dx / 2.0f;
				y1 = i * dx;

				//second point is shifted to up a little bit
				x2 = x1;
				y2 = y1 + v.getIndex(i, j)*vis_scale;

				vertices[vertex_index + 0] = x1;
				vertices[vertex_index + 0] = y1;
				vertices[vertex_index + 0] = x2;
				vertices[vertex_index + 0] = y2;

				indices[index_index] = index_index;
				indices[index_index] = index_index + 1;
				index_index += 2;
			}
		}
	}

	/*
	Swaps buffers
	*/
	void swapBuffers()
	{
		u = u_backbuffer;
		v = v_backbuffer;
	}
	/*
	Renders velocity field
	*/
	void render()
	{
		
		VelocityField2D::bind();

		glm::mat4 mvp = proj;
		m_sh->SetUniformMat4f("u_MVP", mvp);

		m_sh->SetUniform1i("u_DrawFieldEdgeWise", drawFieldEdgeWise);
		GLCall(glDrawElements(GL_LINES, m_ib->GetCount(), GL_UNSIGNED_INT, nullptr));
		
		centeredVF.render();
	}
};