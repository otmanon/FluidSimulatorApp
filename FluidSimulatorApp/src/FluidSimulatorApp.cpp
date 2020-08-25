#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include <iostream>
#include "Renderer.h"
#include "VertexBuffer.h"
#include "IndexBuffer.h"
#include "VertexArray.h"
#include "VertexBufferLayout.h"

#include "Shader.h"
#include "Texture.h"


#include "imgui.h"
#include "imgui_impl_glfw_gl3.h"

#include "TestClearColor.h"
#include "TestParticles.h"
#include "TestGrid.h"

#include "Test.h"
#include <functional>


static unsigned int HEIGHT = 1080, WIDTH = 1920;
static bool windowResizeEvent = false;

static void resizeCallback(GLFWwindow* window, int width, int height)
{
	HEIGHT = height;
	WIDTH = width;
	GLCall(glViewport(0, 0, width, height));
	windowResizeEvent = true;
};
int main(void)
{
	GLFWwindow* window;

	/* Initialize the library */
	if (!glfwInit())
		return -1;

	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_COMPAT_PROFILE);

	WIDTH = WIDTH > HEIGHT ? HEIGHT : WIDTH;
	HEIGHT= HEIGHT > WIDTH ? WIDTH : HEIGHT;
	/* Create a windowed mode window and its OpenGL context */
	window = glfwCreateWindow(WIDTH, HEIGHT, "Hello World", NULL, NULL);
	if (!window)
	{
		glfwTerminate();
		return -1;
	}


	/* Make the window's context current */
	glfwMakeContextCurrent(window);

	glfwSwapInterval(1);
	if (glewInit() != GLEW_OK) {
		std::cout << "Error!" << std::endl;
	}

	glfwSetWindowSizeCallback(window, resizeCallback);

	std::cout << glGetString(GL_VERSION) << std::endl;

	{

		GLCall(glEnable(GL_BLEND));
		GLCall(glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA));

		Renderer renderer;

		ImGui::CreateContext();
		ImGui_ImplGlfwGL3_Init(window, true);
		ImGui::StyleColorsDark();

		test::Test* currentTest = nullptr;
		test::TestMenu* testMenu = new test::TestMenu(currentTest, &HEIGHT, &WIDTH);
		currentTest = testMenu;

		testMenu->RegisterTest<test::TestClearColor>("Clear Color", &HEIGHT, &WIDTH);
		testMenu->RegisterTest<test::TestParticles>("Particles", &HEIGHT, &WIDTH);
		testMenu->RegisterTest<test::TestGrid>("Grid", &HEIGHT, &WIDTH);
		GLCall(glViewport(0, 0, WIDTH, HEIGHT));


		while (!glfwWindowShouldClose(window))
		{

			GLCall(glClearColor(0.0f, 0.0f, 0.0f, 1.0f));
			renderer.Clear();


			ImGui_ImplGlfwGL3_NewFrame();

			if (currentTest) {
		
				currentTest->step(0.003);
				currentTest->render();
				ImGui::Begin("Test");
				if (currentTest != testMenu && ImGui::Button("<-"))
				{
					delete currentTest;
					currentTest = testMenu;
				}
				currentTest->onImGuiRender();
				ImGui::End();
			}

			ImGui::Render();
			ImGui_ImplGlfwGL3_RenderDrawData(ImGui::GetDrawData());

			glfwSwapBuffers(window);

			glfwPollEvents();

			if (windowResizeEvent)
				currentTest->onWindowResize();

		}
		delete currentTest;
		if (currentTest != testMenu)
			delete testMenu;

	}

	ImGui_ImplGlfwGL3_Shutdown();
	ImGui::DestroyContext();
	glfwTerminate();
	return 0;
}