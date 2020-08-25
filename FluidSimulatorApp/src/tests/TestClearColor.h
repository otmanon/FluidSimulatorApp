#pragma once

#include "Test.h"

namespace test {
	class TestClearColor : public Test {

	private:
		float m_ClearColor[4];
	public:
		TestClearColor(unsigned int * heightPtr, unsigned int * widthPtr);
		~TestClearColor();

	
		void render() override;
		void onImGuiRender() override;

	};
}