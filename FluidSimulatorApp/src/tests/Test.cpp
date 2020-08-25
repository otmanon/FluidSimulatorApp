#include "Test.h"
#include "imgui/imgui.h"
namespace test {

	TestMenu::TestMenu(Test*& currentTestPointer, unsigned int * heightPtr, unsigned int * widthPtr)
		: m_CurrentTest(currentTestPointer), Test(heightPtr, widthPtr)
	{
		Test(heightPtr, widthPtr);
	}


	void TestMenu::onImGuiRender()
	{
		for (auto& test : m_Tests) {
			if (ImGui::Button(test.first.c_str()))
				m_CurrentTest = test.second();

		}
	}

}