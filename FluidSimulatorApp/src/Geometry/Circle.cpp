#include "Circle.h"
#include "glm/glm.hpp"
#include <iostream>
#include <vector>


Circle::Circle(float radius, glm::vec2 pos, unsigned int count)
	: m_Radius(radius), m_Position(pos), m_NumVertices(count), m_Model(glm::mat4(1.0f))
{
	buildPositions();
	buildIndices();
	std::string shaderpath = "shaders/SolidColor.shader";
	Circle::buildOpenGLObjects(m_Vertices.data(), m_Indices.data(), 2, m_Vertices.size(), m_Indices.size(), true, shaderpath);

}



void Circle::DestroyCircle()
{
	Circle::DestroyDrawable();
}

void Circle::buildPositions()
{
	m_Vertices = std::vector<float>(2 * m_NumVertices);

	float theta = 0;
	float increment = 2 * glm::pi<float>() / m_NumVertices;
	for (int i = 0; i < 2 * m_NumVertices; i += 2)
	{
		m_Vertices[i] = m_Radius * glm::cos(theta);
		m_Vertices[i + 1] = m_Radius * glm::sin(theta);
		theta += increment;
	}
}

void Circle::buildIndices()
{
	m_Indices = std::vector<unsigned int>(m_NumVertices);
	for (unsigned int i = 0; i < m_NumVertices; i++) {
		m_Indices[i] = i;
	}
}




void Circle::render(glm::mat4& proj, glm::mat4& view) const
{
	Circle::bind();
	m_Model = glm::mat4(glm::translate(m_Model, glm::vec3(m_Position, 0)));
	glm::mat4 mvp = proj * view*m_Model;

	m_sh->SetUniformMat4f("u_MVP", mvp);

	glPointSize(3);


	GLCall(glDrawElements(GL_POINTS, m_ib->GetCount(), GL_UNSIGNED_INT, nullptr));
	m_Model = glm::mat4(glm::translate(m_Model, -glm::vec3(m_Position, 0)));
}

void Circle::render(glm::mat4& proj, glm::mat4& view, float color[4]) const
{
	Circle::bind();
	m_Model = glm::mat4(glm::translate(m_Model, glm::vec3(m_Position, 0)));
	glm::mat4 mvp = proj * view*m_Model;
	m_sh->SetUniformMat4f("u_MVP", mvp);

	m_sh->SetUniform4f("u_Color", color[0], color[1], color[2], color[3]);

	glPointSize(3);

	GLCall(glDrawElements(GL_POINTS, m_ib->GetCount(), GL_UNSIGNED_INT, nullptr));
	m_Model = glm::mat4(glm::translate(m_Model, -glm::vec3(m_Position, 0)));
}


