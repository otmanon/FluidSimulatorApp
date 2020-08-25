#shader vertex
#version 330 core

layout(location=0) in vec4 position;

out vec4 v_CircleCenter;
out vec2 vertexPos;
uniform mat4 u_MVP;

void main()
{
	v_CircleCenter =  u_MVP *position;
	gl_Position = v_CircleCenter;
	

	float x = gl_Position.x;
	float y = gl_Position.y;
	if (gl_Position.x > 1) {
		x -= 2;
	}
	if (gl_Position.x < -1) {
		x += 2;
	}
	if (gl_Position.y < -1) {
		y+= 2;
	}
	if (gl_Position.y > 1) {
		y -= 2;
	}
	gl_Position = vec4(vec2(x, y), 0, 1);

	vertexPos = vec2(gl_Position.x, gl_Position.y);

};


#shader fragment
#version 330 core

layout(location=0) out vec4 color;

in vec4 v_CircleCenter;
in vec2 vertexPos;

uniform vec4 u_Color;
void main()
{

	color = u_Color;

};