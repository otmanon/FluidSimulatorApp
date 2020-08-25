#pragma once
#include "VertexBufferLayout.h"
#include "VertexBuffer.h"
#include <glm/glm.hpp>

/*
Anything that will be drawn on screen will be a child of this class. This includes cells, vertices, and edges. 
Only call DestroyDrawable when you are done using the object for the remainder of the application (almost never)... or when the object gets destroyed. 
*/
class Drawable {
protected:

	VertexBuffer* m_vb;
	VertexArray* m_va;
	IndexBuffer* m_ib;
	Shader* m_sh;
	bool m_DrawableInitialized = false; // is set to true when the drawable is given a vb, va, ib, and shader
public:
	Drawable() {};
	
	void buildOpenGLObjects(float * data, unsigned int * indices, unsigned int num_attribs, unsigned int v_count, unsigned int i_count, bool build_shader, std::string& filepath)
	{
		buildVertexBuffer(data, num_attribs, v_count);
		buildVertexArray(num_attribs);
		buildIndexBuffer(indices, i_count);
		if (build_shader)
		{
			buildShader(filepath);
			m_DrawableInitialized = true;
		}
			
	}

	void updateVertexBuffer(float * data, unsigned int * indices, unsigned int num_attribs, unsigned int v_count, unsigned int i_count)
	{
		delete m_vb;
		delete m_ib;
		delete m_va;
		buildVertexBuffer(data, num_attribs, v_count);
		buildVertexArray(num_attribs);
		buildIndexBuffer(indices, i_count);
	}
	template <class DataType>
	void buildVertexBuffer(DataType * data,  int num_attribs,  int count)
	{
		m_vb = new VertexBuffer(data, num_attribs*count * sizeof(DataType));
	}
	void buildIndexBuffer(unsigned int * indices, unsigned int count)
	{
		m_ib = new IndexBuffer(indices, count);
	}
	void buildVertexArray(unsigned int num_attribs)
	{
		m_va = new VertexArray();
		VertexBufferLayout layout;
		layout.Push<float>(num_attribs);
		m_va->AddBuffer(*m_vb, layout);
	}
	void buildShader(std::string& filepath)
	{
		m_sh = new Shader(filepath);
		m_sh->Bind();
	}

	void setShader(Shader* sh) {
		m_sh = sh;
	}
	void setVertexBuffer(VertexBuffer* vb) {
		m_vb = vb;
	}
	void setVertexArray(VertexArray* va) {
		m_va = va;
	}
	void setIndexBuffer(IndexBuffer* ib) {
		m_ib = ib;
	}

	virtual void DestroyDrawable()
	{
		delete m_vb;
		delete m_va;
		delete m_ib;

	}
	void bind() const {
		m_sh->Bind();
		m_va->Bind();
		m_ib->Bind();
		//GLCall(glDrawElements(GL_POINTS, m_ib->GetCount(), GL_UNSIGNED_INT, nullptr));
	};
	virtual void render(glm::mat4 proj, glm::mat4 view) const {};
	virtual void renderColor(glm::mat4 proj, glm::mat4 view, float* color) const {};


};