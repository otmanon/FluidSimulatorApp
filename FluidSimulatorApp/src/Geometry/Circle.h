#pragma once
#include "VertexBuffer.h"
#include "VertexBufferLayout.h"
#include "Texture.h"
#include <memory>
#include "glm/gtc/matrix_transform.hpp"
#include "Drawable.h"

class Circle: public Drawable {
private:
	mutable glm::mat4  m_Model;
	glm::vec3 m_TranslationA;

	glm::vec2* m_X; 
	unsigned int m_NumVertices = 30;
	
	std::vector<float> m_Vertices;
	std::vector<unsigned int> m_Indices;//unsigned int indices[];
protected:
	glm::vec2 m_Position;//center of circle
	float m_Radius = 200;

public:

	Circle(float radius, glm::vec2 pos, unsigned int count= 400);



	void DestroyCircle();

	void buildPositions();

	
	void buildIndices();

	
	/*
		Builds Vertex Array, Vertex Buffer, Index BUffer and the 2 Shaders. 
	*/
	void buildGLObjects(); 

	
	void render(glm::mat4& proj, glm::mat4& view) const; 
	void render(glm::mat4& proj, glm::mat4& view, float color[4]) const;

	void setPosition(glm::vec2 newPos){ m_Position = newPos; }

	glm::vec2 getPosition() { return m_Position; }

};