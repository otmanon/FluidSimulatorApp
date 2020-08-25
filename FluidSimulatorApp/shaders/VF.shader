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


uniform bool u_DrawFieldCenterWise;
uniform bool u_DrawFieldEdgeWise;

void main()
{	

	if (u_DrawFieldEdgeWise)
	{
		color = vec4(0.0f, 0.0f, 0.0f, 1.0f);

	}
	else if (u_DrawFieldCenterWise)
	{
		color = vec4(1.0f, 1.0f, 1.0f, 1.0f);
	}
};