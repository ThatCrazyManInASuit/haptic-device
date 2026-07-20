#include "chai3d.h"
#include "globals.h"
#include "utility.h"
#include <GLFW/glfw3.h>

#include <vector>
#include <mutex>
#include <string>

class Slider {
  std::string name;
  std::string units;
  double minVal;
  double maxVal;
  double val;
  int sigFigs;
  bool dragging;

  public: 
    /// @brief A constructor for building a slider.
    /// @param name Name of what quantity the slider is controlling
    /// @param units Unit that the quantity is measured in
    /// @param minVal Minimum value of the slider
    /// @param maxVal Maximum value of the slider
    /// @param defaultVal The default (launch) value of the slider
    /// @param sigFigs The number of significant digits displayed
    Slider(std::string name, std::string units, double minVal, double maxVal, double defaultVal, 
         double sigFigs) {
      this->name = name;
      this->units = units;
      this->minVal = minVal;
      this->maxVal = maxVal;
      val = defaultVal;
      this->sigFigs = sigFigs;
      dragging = false;
    }

    double getNormalizedValue() {
      return maxVal <= minVal ? 0.0 : (val - minVal) / (maxVal - minVal);
    }

    void setNormalizedValue(double normalizedVal) {
      if (normalizedVal < 0.0) {
          normalizedVal = 0.0;
      } else if (normalizedVal > 1.0) {
          normalizedVal = 1.0;
      }
      val = minVal + normalizedVal * (maxVal - minVal);
    }

    std::string getDisplayText() {
      return name + ": " + chai3d::cStr(val, sigFigs) + " " + units;
    }

    bool isDragging() {
      return dragging;
    }

    void setDragging(bool dragging) {
      this->dragging = dragging;
    }

    double getVal() {
      return val;
    }

    void setVal(double val) {
      this->val = val;
    }

    std::string getName() {
      return name;
    }
};


std::vector<Slider> sliders;
const int SLIDER_WIDTH = 240;
const int SLIDER_LEFT = 50;
const int SLIDER_TOP = 45;
const int SLIDER_ROW_SPACING = 52;

double getNormValFromMouseX(int sliderIndex, double mouseX) {
  double trackX = SLIDER_LEFT;
  double trackY = SLIDER_TOP + sliderIndex * SLIDER_ROW_SPACING;

  double normalizedValue = (mouseX - trackX) / SLIDER_WIDTH;
  if (normalizedValue < 0.0) {
    return 0.0;
  } else if (normalizedValue > 1.0) {
    return 1.0;
  }
  return normalizedValue;
}

// SLIDER UI STEP 4: Wire the slider back to whatever live state it controls,
// both so dragging it takes effect immediately and so the handle reflects
// changes made through another channel (e.g. the launcher's IPC command).
void applySliderVal(const std::string &id, double value) {
  if (id == "Time Step") {
    setLiveTimeStep(value);
  }
}

// Handles cursor movement inside the slider window.
void sliderWindowCursorPosCallback(GLFWwindow *a_window, double a_posX, double a_posY) {
  std::lock_guard<std::recursive_mutex> lock(sceneMutex);
  double x, y;
  glfwGetCursorPos(a_window, &x, &y);
  for (int i = 0; i < sliders.size(); i++) {
    if (sliders[i].isDragging()) {
      sliders[i].setNormalizedValue(getNormValFromMouseX(i, x));
      applySliderVal(sliders[i].getName(), sliders[i].getVal());
    }
  }
}

bool isMouseOverSlider(int sliderIndex, double mouseX, double mouseY) {
  double trackX = SLIDER_LEFT;
  double trackY = SLIDER_TOP + sliderIndex * SLIDER_ROW_SPACING;

  return mouseX >= trackX - 12 && mouseX <= trackX + SLIDER_WIDTH + 12 && mouseY >= trackY - 18 
        && mouseY <= trackY + 18;
}

// Handles mouse button presses inside the slider window.
void sliderWindowMouseButtonCallback(GLFWwindow *a_window, int a_button, int a_action, int a_mods) {
  if (a_button != GLFW_MOUSE_BUTTON_LEFT) {
    return;
  }

  std::lock_guard<std::recursive_mutex> lock(sceneMutex);
  double x, y;
  glfwGetCursorPos(a_window, &x, &y);
  if (a_action == GLFW_PRESS) {
    for (int i = 0; i < sliders.size(); i++) {
        if (isMouseOverSlider(i, x, y)) {
            sliders[i].setDragging(true);
            sliders[i].setNormalizedValue(getNormValFromMouseX(i, x));
            applySliderVal(sliders[i].getName(), sliders[i].getVal());
        }
    }
  } else if (a_action == GLFW_RELEASE) {
    for (Slider& slider : sliders) {
        if (slider.isDragging()) {
            slider.setDragging(false);
        }
    }
  }
}

/**
 * @brief Handles keyboard input inside the slider window.
 */
void sliderWindowKeyCallback(GLFWwindow *a_window, int a_key, int a_scancode, int a_action, int a_mods) {
  if ((a_action == GLFW_PRESS) || (a_action == GLFW_REPEAT)) {
    if ((a_key == GLFW_KEY_ESCAPE) || (a_key == GLFW_KEY_Q)) {
        glfwSetWindowShouldClose(a_window, GLFW_TRUE);
    }
  }
}

GLFWwindow* initializeSliderWindow(GLFWwindow* mainWindow) {
    const int SLIDER_WINDOW_WIDTH = 340;
    const int SLIDER_WINDOW_HEIGHT = 170;
    GLFWwindow *sliderWindow = glfwCreateWindow(SLIDER_WINDOW_WIDTH, SLIDER_WINDOW_HEIGHT, 
        "Controls", nullptr, mainWindow);

    if (!sliderWindow) {
        glfwTerminate();
        throw std::runtime_error("Failed to create slider window!");
    }

    int mainWidth, mainHeight, mainX, mainY;
    glfwGetWindowSize(mainWindow, &mainWidth, &mainHeight);
    glfwGetWindowPos(mainWindow, &mainX, &mainY);

    const int SLIDER_WINDOW_INIT_POS_X = mainX + mainWidth + 20;
    const int SLIDER_WINDOW_INIT_POS_Y = mainY;

    glfwSetWindowPos(sliderWindow, SLIDER_WINDOW_INIT_POS_X, SLIDER_WINDOW_INIT_POS_Y);

    glfwSetCursorPosCallback(sliderWindow, sliderWindowCursorPosCallback);
    glfwSetMouseButtonCallback(sliderWindow, sliderWindowMouseButtonCallback);
    glfwSetKeyCallback(sliderWindow, sliderWindowKeyCallback);
    
    sliders.push_back(Slider("Time Step", "fs", 0.0, 2.0, 1.0, 3));
    sliders.push_back(Slider("Temperature", "K", 0.0, 20000.0, 0.0, 2));
    glfwSetWindowTitle(sliderWindow, "Controls");
    return sliderWindow;
}

void drawSliderText(const std::string &text, double x, double y) {
    chai3d::cFontPtr SLIDER_FONT = NEW_CFONT_CALIBRI_20();
    if (SLIDER_FONT) {
        chai3d::cRenderOptions options;
        options.m_camera = nullptr;
        options.m_single_pass_only = true;
        options.m_render_opaque_objects_only = true;
        options.m_render_transparent_front_faces_only = false;
        options.m_render_transparent_back_faces_only = false;
        options.m_enable_lighting = false;
        options.m_render_materials = false;
        options.m_render_textures = true;
        options.m_creating_shadow_map = false;
        options.m_rendering_shadow = false;
        options.m_shadow_light_level = 0.0;
        options.m_storeObjectPositions = false;
        options.m_markForUpdate = false;

        glDisable(GL_LIGHTING);
        glPushMatrix();
        glTranslated(x, y, 0.0);
        glScaled(1.0, -1.0, 1.0);
        SLIDER_FONT->renderText(text, cColorf(0.05f, 0.05f, 0.05f), 1.0, 1.0, 1.0, options);
        glPopMatrix();
    }
}

void getSliderLayout(int sliderIndex, double &trackX, double &trackY) {
  trackX = SLIDER_LEFT;
  trackY = SLIDER_TOP + sliderIndex * SLIDER_ROW_SPACING;
}

double getLiveSliderValue(const std::string &id, double fallback) {
  if (id == "Time Step") {
    return simulationTimeStep.load();
  }
  return fallback;
}



// keep sliders that aren't currently being dragged in sync with live state
// changed through another channel (e.g. the launcher's IPC "set timestep")
void syncSlidersFromLiveState() {
  for (Slider &slider : sliders) {
    if (!slider.isDragging()) {
      slider.setVal(getLiveSliderValue(slider.getName(), slider.getVal()));
    }
  }
}

double getSliderVal(const std::string &id, double fallback) {
  for (Slider slider : sliders) {
    if (slider.getName() == id) {
      return slider.getVal();
    }
  }
  return fallback;
}

void renderSliders() {
  const double TRACK_HALF_HEIGHT = 3.0;
  const double HANDLE_RADIUS = 9.0;
  const double SLIDER_TEXT_OFFSET = 28.0;
  const chai3d::cColorf TRACK_COLOR(0.82f, 0.82f, 0.84f); 
  const chai3d::cColorf HANDLE_TRACK_COLOR(0.20f, 0.55f, 0.95f);
  const chai3d::cColorf HANDLE_COLOR(0.15f, 0.45f, 0.85f);
  const chai3d::cColorf COLOR_MYSTERY(1.0f, 1.0f, 1.0f);

  for (int i = 0; i < sliders.size(); i++) {
    Slider &slider = sliders[i];
    double trackX;
    double trackY;
    getSliderLayout(i, trackX, trackY);
    const double trackXEnd = trackX + SLIDER_WIDTH;
    const double handleX = trackX + slider.getNormalizedValue() * SLIDER_WIDTH;

    drawSliderText(slider.getDisplayText(), trackX, trackY - SLIDER_TEXT_OFFSET);
    drawPill(trackX, trackXEnd, trackY, TRACK_HALF_HEIGHT, TRACK_COLOR);
    drawPill(trackX, handleX, trackY, TRACK_HALF_HEIGHT, HANDLE_TRACK_COLOR);
    drawCircle(handleX, trackY, HANDLE_RADIUS, COLOR_MYSTERY);
    drawCircle(handleX, trackY, HANDLE_RADIUS - 2.5, HANDLE_COLOR);
    
  }
}

void renderSliderWindow(GLFWwindow* mainWindow, GLFWwindow*& sliderWindow) {
  if (sliderWindow != nullptr) {
    if (glfwWindowShouldClose(sliderWindow)) {
      glfwDestroyWindow(sliderWindow);
      sliderWindow = nullptr;
      glfwMakeContextCurrent(mainWindow);
    } else {
      syncSlidersFromLiveState();

      glfwMakeContextCurrent(sliderWindow);
      
      int windowWidth, windowHeight;

      glfwGetWindowSize(sliderWindow, &windowWidth, &windowHeight);

      int framebufferWidth, framebufferHeight;
      glfwGetFramebufferSize(sliderWindow, &framebufferWidth, &framebufferHeight);
      glViewport(0, 0, framebufferWidth, framebufferHeight);
      glMatrixMode(GL_PROJECTION);
      glLoadIdentity();
      glOrtho(0, windowWidth, windowHeight, 0, -1, 1);
      glMatrixMode(GL_MODELVIEW);
      glLoadIdentity();
      glDisable(GL_DEPTH_TEST);

      const chai3d::cColorf BACKGROUND_COLOR(0.94f, 0.94f, 0.94f);
      glClearColor(BACKGROUND_COLOR.getR(), BACKGROUND_COLOR.getG(), BACKGROUND_COLOR.getB(), 1.0f);
      glClear(GL_COLOR_BUFFER_BIT);

      renderSliders();

      glfwSwapBuffers(sliderWindow);
      glfwMakeContextCurrent(mainWindow);
    }
  }
}