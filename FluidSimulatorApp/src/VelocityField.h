#pragma once

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
	bool drawFieldCenterWise = false;
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
				u.data.emplace_back(10.0f); //initialize all vels to zero
			}
		}
		u_backbuffer = u;

		v.data.reserve(v.columns * v.rows);
		//initialize u velocity component
		for (int i = 0; i < v.rows; i++)
		{
			for (int j = 0; j < v.columns; j++)
			{
				if (j < 3)
				{
					v.data.emplace_back(10.0f); //initialize all vels to zero
				}
				else if (j > 7)
				{
					v.data.emplace_back(-10.0f);
				}
				else
					v.data.emplace_back(0.0f);
			}
		}
		v_backbuffer = v;


	}

	/*
	prepares the openGL	data... lines will go from edge midpoints to appropriate direction
	*/
	void initOpenGLData()
	{
		vertices.reserve(2 * u.rows*u.columns * 2 + 2 * v.rows * v.columns * 2); //each of the u_rows x v_colums will have 2 vertices (2x2)... a
		indices.reserve(2 * u.rows*u.columns + 2 * v.rows*v.columns);
		centeredVF.vertices.reserve(2 * u.rows*v.columns * 2);
		centeredVF.indices.reserve(u.rows*v.columns * 2);
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

		vertices.clear();
		indices.clear();

		vertices.reserve(2 * u.rows*u.columns * 2 + 2 * v.rows * v.columns * 2); //each of the u_rows x v_colums will have 2 vertices (2x2)... a
		indices.reserve(2 * u.rows*u.columns + 2 * v.rows*v.columns );
		centeredVF.vertices.clear();
		centeredVF.indices.clear();
		centeredVF.vertices.reserve(2 * u.rows*v.columns * 2);
		centeredVF.indices.reserve(u.rows*v.columns * 2);
		buildVerticesEdgeWise();
		buildVerticesCenterWise();
		updateVertexBuffer(vertices.data(), indices.data(), 2, vertices.size(), indices.size());

		centeredVF.updateOpenGLData();
	}

	/*
	Gets Velocity From VelocityField
	*/
	Eigen::Vector2f getVelocity(Eigen::Vector2f pos)
	{
		
		//bilinearly interpolate u's
	
		GridCoordinates blCoords; //u edge bottom left grid coords of cell
		GridCoordinates trCoords; // u edge at top left grid coords of cell
		
		blCoords.j = (int) (pos.x() / dx);
		blCoords.i = (int) ((pos.y() - dx / 2.0f) / dx);
		
		trCoords.j = blCoords.j + 1;
		trCoords.i = blCoords.i + 1;
		
		Eigen::Vector2f blPos(blCoords.j*dx, blCoords.i*dx + dx / 2.0f); // position of bottom left vertex (even if not in window)

		projectUIndicesToBoundary(blCoords, trCoords);

		float r_x = (pos.x() - blPos.x()) / dx; //fraction of distance in cell beyond bottom x
		float r_y = (pos.y() - blPos.y()) / dx; // fraction of distance in cell above bottom y
		
		//bilinear interpolation
		float u_x1, u_x2, u_final;
		u_x1 = u.index(blCoords.i, blCoords.j)*(1 - r_x) + u.index(blCoords.i, trCoords.j)*r_x;
		u_x2 = u.index(trCoords.i, blCoords.j)*(1 - r_x) + u.index(trCoords.i, trCoords.j)*r_x;
		u_final = u_x1 * (1 - r_y) + u_x2 * r_y;
	
		//bilinearly interpolate v's
		blCoords.i = (int) (pos.y() / dx );
		blCoords.j = (int) ((pos.x() - dx / 2.0f) / dx);
		trCoords.j = blCoords.j + 1;
		trCoords.i = blCoords.i + 1;
		blPos = Eigen::Vector2f(blCoords.j*dx + dx / 2.0f, blCoords.i*dx);

		projectVIndicesToBoundary(blCoords, trCoords);
		r_x = (pos.x() - blPos.x()) / dx; //fraction of distance in cell beyond bottom x
		r_y = (pos.y() - blPos.y()) / dx; // fraction of distance in cell above bottom y

			//bilinear interpolation
		float v_x1, v_x2, v_final;
		v_x1 = v.index(blCoords.i, blCoords.j)*(1 - r_x) + v.index(blCoords.i, trCoords.j)*r_x;
		v_x2 = v.index(trCoords.i, blCoords.j)*(1 - r_x) + v.index(trCoords.i, trCoords.j)*r_x;
		v_final = v_x1 * (1 - r_y) + v_x2 * r_y;
		
		return Eigen::Vector2f(u_final, v_final);
							
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
		int index = 0;
		for (int i = 0; i < u.rows; i++) //loop through each cell
		{
			for (int j = 0; j < v.columns; j++)
			{
				
				pos1(0) = j * dx + dx*0.5f;
				pos1(1) = i * dx + dx * 0.5f; //center of each cell

				vel =  getVelocity(pos1);

				pos2(0) = pos1.x() + vel.x() * vis_scale;
				pos2(1) = pos1.y() + vel.y() * vis_scale; //center of each cell
				
				centeredVF.vertices.emplace_back(pos1.x());
				centeredVF.vertices.emplace_back(pos1.y());
				centeredVF.vertices.emplace_back(pos2.x());
				centeredVF.vertices.emplace_back(pos2.y());
				
				centeredVF.indices.emplace_back((unsigned int) index);
				centeredVF.indices.emplace_back((unsigned int) index + 1);
				index += 2;
				
			}
		}
	}
	/*
	Constructs vertices and indices lists that will be used for rendering Velocity Field
	*/
	void buildVerticesEdgeWise()
	{
		
		float x1, x2, y1, y2;
		int index = 0;

		//Draw u component of vel (corresponds to vertical edges/ edges pointing horizontally)
		for (int i = 0; i < u.rows; i++)
		{
			for (int j = 0; j < u.columns; j++)
			{
				//first point
				x1 = j * dx;
				y1 = i * dx + dx / 2.0f;

				//second point is shifted to the right a little bit
				x2 = x1 + u.index(i, j)*vis_scale;
				y2 = y1;
				vertices.emplace_back(x1);
				vertices.emplace_back(y1);
				vertices.emplace_back(x2);
				vertices.emplace_back(y2);

				indices.emplace_back(index);
				indices.emplace_back(index + 1);
				index += 2;
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
				y2 = y1 + v.index(i, j)*vis_scale;

				vertices.emplace_back(x1);
				vertices.emplace_back(y1);
				vertices.emplace_back(x2);
				vertices.emplace_back(y2);

				indices.emplace_back(index);
				indices.emplace_back(index + 1);
				index += 2;
			}
		}
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