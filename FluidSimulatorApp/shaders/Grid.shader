#shader vertex
#version 330 core

layout(location = 0) in vec4 position;

uniform mat4 u_MVP;

void main()
{

	gl_Position = u_MVP * position;
	

};


#shader fragment
#version 330 core

layout(location = 0) out vec4 color;



float epsilon = 0.001f;
uniform unsigned int  u_LabelGrid[1000]; //grid of labels... 0 if solid, 1 if air, 2 if liquid
uniform float u_dx; // cell spacing (in pixels)
uniform float u_Rows; //number of rows
uniform float u_Columns; //number of columnns

uniform bool u_DrawEdges;

unsigned int indexLabelGrid(float i, float j)
{
	return  u_LabelGrid[int(i*u_Columns+ j)];
}


void main()
{	
	
	//convert frag coords to grid coords 
	float x = gl_FragCoord.x; //x coordinate of current pixel
	float y = gl_FragCoord.y; //y coordinate of current pixel

	float i, j; //frag coords of 

	i = floor(y / u_dx);
	j = floor(x / u_dx);

	unsigned int label = indexLabelGrid(i, j);
	
	if (label == uint(0))
	{
		color = vec4(0.31f, 0.23f, 0.62f, 1.0f);
	}
	else
	{
		color = vec4(0.25f, 0.25f, 0.30f, 1.0f);
	}

	if (u_DrawEdges)
	{
		if (x - (j * u_dx) <=1 || y - (i * u_dx) <= 1)
		{
			color = vec4(0.0f, 0.0f, 0.0f, 1.0f);
		}
	}
	
};