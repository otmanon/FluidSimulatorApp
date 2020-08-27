#pragma once

#include <functional>
#include <vector>
namespace test {
	class Test 
	{
	protected:
		unsigned int * m_WindowHeightPtr;
		unsigned int * m_WindowWidthPtr;
	public:
		Test(){}
		Test(unsigned int* heightPtr, unsigned int* widthPtr)
			: m_WindowHeightPtr(heightPtr), m_WindowWidthPtr(widthPtr){}
		virtual ~Test(){}

		virtual void OnUpdate(float deltaTime) {}
		virtual void render(){}
		virtual void onImGuiRender(){}
		virtual void step() {}
		virtual void onWindowResize() {}
		/*
		X and Y are window coordinates of the mouse (in pixels)
		*/
		virtual void onMouseClick(float x, float y) {}

	};

	class TestMenu : public Test 
	{
	private:
		Test*& m_CurrentTest;
		std::vector<std::pair<std::string, std::function<Test*()>>> m_Tests;
	public:
		TestMenu(Test*& currentTestPointer, unsigned int * heightPtr, unsigned int * widthPtr);
		
		
		template <typename T>
		void RegisterTest(const std::string& name, unsigned int * heightPtr, unsigned int * widthPtr) {
			std::cout << "Registering Test " << name << std::endl;
			m_Tests.push_back(std::make_pair(name, [&, heightPtr, widthPtr]() {return new T(heightPtr, widthPtr); }));
		}															  

		void onImGuiRender() override;

	};
}