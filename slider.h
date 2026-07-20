#include <GLFW/glfw3.h>
#include <string>

GLFWwindow* initializeSliderWindow(GLFWwindow* mainWindow);
void renderSliderWindow(GLFWwindow* mainWindow, GLFWwindow*& sliderWindow);
double getSliderVal(const std::string &id, double fallback);