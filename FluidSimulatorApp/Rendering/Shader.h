#pragma once
#include <string>
#include <unordered_map>
#include "glm.hpp"
struct ShaderProgramSource
{
	std::string VertexSource;
	std::string FragmentSource;
};

class Shader {
private:
	std::string m_FilePath;
	unsigned int m_RendererID;
	std::unordered_map<std::string, int> m_UniformLocationCache;

public:
	Shader(const std::string& filepath);
	~Shader();

	void Bind() const;
	void Unbind() const;

	//set Uniforms
	void SetUniform4f(const std::string& name, float v0, float v1, float v2, float v3);
	void SetUniform1f(const std::string& name, float value);
	void SetUniform1i(const std::string& name, int value);
	void SetUniform1iv(const std::string& name, std::vector<int> i);
	void SetUniform1uiv(const std::string& name, std::vector<unsigned int> i);
	void SetUniformMat4f(const std::string& name, const glm::mat4& matrix);
private:
	
	unsigned int CreateShader(const std::string& vertexShader, const std::string& fragmentShader);
	unsigned int CompileShader(unsigned int type, const std::string& source);
	ShaderProgramSource ParseShader(const std::string& filepath);
	int GetUniformLocation(const std::string& name);

};