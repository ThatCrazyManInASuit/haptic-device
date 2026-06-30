//==============================================================================
/*
 Software License Agreement (BSD License)
 Copyright (c) 2003-2016, CHAI3D.
 (www.chai3d.org)
 All rights reserved.
 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions
 are met:
 * Redistributions of source code must retain the above copyright
 notice, this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above
 copyright notice, this list of conditions and the following
 disclaimer in the documentation and/or other materials provided
 with the distribution.
 * Neither the name of CHAI3D nor the names of its contributors may
 be used to endorse or promote products derived from this software
 without specific prior written permission.
 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 POSSIBILITY OF SUCH DAMAGE.
 \author    <http://www.chai3d.org>
 \author    Francois Conti
 \version   3.2.0 $Rev: 1922 $
 */
//==============================================================================
//------------------------------------------------------------------------------
#include "atom.h"
#include "chai3d.h"
#include "globals.h"
#include "inputHandling.h"
#include "potentials.h"
#include "utility.h"
#include "boundaryConditions.h"
//------------------------------------------------------------------------------
#include <GLFW/glfw3.h>
#include <math.h>
#include <unistd.h>
#include <array>
#include <chrono>
#include <ctime>
#include <fstream>
#include <iostream>
#include <limits>
#include <mutex>
#include <sstream>
#include <string>
#include <atomic>
#include <vector>
#include <thread>
#include <mutex>
#include <atomic>
#include <optional>
#include <unordered_map>

#include <stdexcept>

std::mutex umaMutex;
std::vector<std::vector<double>> latestUMA;
std::atomic<bool> umaReady(false);
std::atomic<bool> stopUMA(false);
//------------------------------------------------------------------------------
using namespace chai3d;
using namespace std;

extern int just_unanchored;
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
// GENERAL SETTINGS
//------------------------------------------------------------------------------
// Stereo Mode
/*
 C_STEREO_DISABLED:            Stereo is disabled
 C_STEREO_ACTIVE:              Active stereo for OpenGL NVIDIA QUADRO
 cards C_STEREO_PASSIVE_LEFT_RIGHT:  Passive stereo where L/R images are
 rendered next to each other C_STEREO_PASSIVE_TOP_BOTTOM:  Passive stereo
 where L/R images are rendered above each other
 */
cStereoMode stereoMode = C_STEREO_DISABLED;

// Fullscreen mode
bool fullscreen = false;

//------------------------------------------------------------------------------
// DECLARED CONSTANTS
//------------------------------------------------------------------------------
// Radius of each sphere
const double SPHERE_RADIUS = 0.008;
// number of cameras
const int NUM_CAM = 2;

// position of walls and ground
const double WALL_GROUND = 0.0 + SPHERE_RADIUS;
const double WALL_CEILING = 0.05; // 0.2;
const double WALL_LEFT = -0.05;   //-0.1;
const double WALL_RIGHT = 0.05;   // 0.2;
const double WALL_FRONT = 0.05;   // 0.08;
const double WALL_BACK = -0.05;   //-0.08;
const double SPHERE_STIFFNESS = 500.0;
const double SPHERE_MASS_SCALE_FACTOR = 0.02;
const double F_DAMPING = 0.25; // 0.25
const double V_DAMPING = 0.8;  // 0.1
const double A_DAMPING = 0.99; // 0.5
const double K_MAGNET = 500.0;
const double HAPTIC_STIFFNESS = 1000.0;
const double SIGMA = 1.0;
const double EPSILON = 1.0;
const double KEYBOARD_SIM_DT_MAX = 0.002;
const double MAX_ATOM_SPEED = 1.0;
const double MAX_ATOM_STEP = 0.01;

// Scales the distance betweens atoms
const double DIST_SCALE = .02;

// boundary conditions
const double BOUNDARY_LIMIT = .5;
const cVector3d northPlanePos = cVector3d(0, BOUNDARY_LIMIT, 0);
const cVector3d northPlaneP1 = cVector3d(1, BOUNDARY_LIMIT, 0);
const cVector3d northPlaneP2 = cVector3d(1, BOUNDARY_LIMIT, 1);
const cVector3d northPlaneNorm =
    cComputeSurfaceNormal(northPlanePos, northPlaneP1, northPlaneP2);
const cVector3d southPlanePos = cVector3d(0, -BOUNDARY_LIMIT, 0);
const cVector3d southPlaneP1 = cVector3d(1, -BOUNDARY_LIMIT, 0);
const cVector3d southPlaneP2 = cVector3d(1, -BOUNDARY_LIMIT, 1);
const cVector3d southPlaneNorm =
    cComputeSurfaceNormal(southPlanePos, southPlaneP1, southPlaneP2);
const cVector3d eastPlanePos = cVector3d(BOUNDARY_LIMIT, 0, 0);
const cVector3d eastPlaneP1 = cVector3d(BOUNDARY_LIMIT, 1, 0);
const cVector3d eastPlaneP2 = cVector3d(BOUNDARY_LIMIT, 1, 1);
const cVector3d eastPlaneNorm =
    cComputeSurfaceNormal(eastPlanePos, eastPlaneP1, eastPlaneP2);
const cVector3d westPlanePos = cVector3d(-BOUNDARY_LIMIT, 0, 0);
const cVector3d westPlaneP1 = cVector3d(-BOUNDARY_LIMIT, 1, 0);
const cVector3d westPlaneP2 = cVector3d(-BOUNDARY_LIMIT, 1, 1);
const cVector3d westPlaneNorm =
    cComputeSurfaceNormal(westPlanePos, westPlaneP1, westPlaneP2);
const cVector3d forwardPlanePos = cVector3d(0, 0, BOUNDARY_LIMIT);
const cVector3d forwardPlaneP1 = cVector3d(0, 1, BOUNDARY_LIMIT);
const cVector3d forwardPlaneP2 = cVector3d(1, 1, BOUNDARY_LIMIT);
const cVector3d forwardPlaneNorm =
    cComputeSurfaceNormal(forwardPlanePos, forwardPlaneP1, forwardPlaneP2);
const cVector3d backPlanePos = cVector3d(0, 0, -BOUNDARY_LIMIT);
const cVector3d backPlaneP1 = cVector3d(0, 1, -BOUNDARY_LIMIT);
const cVector3d backPlaneP2 = cVector3d(1, 1, -BOUNDARY_LIMIT);
const cVector3d backPlaneNorm =
    cComputeSurfaceNormal(backPlanePos, backPlaneP1, backPlaneP2);

enum class HapticMode {
  Position,
  Standby,
  Force
};

//------------------------------------------------------------------------------
// DECLARED VARIABLES
//------------------------------------------------------------------------------
// calculator
Calculator *calculatorPtr;

// radius of the camera
double rho = .35;

// a world that contains all objects of the virtual environment
cWorld *world;

// a camera to render the world in the window display
cCamera *camera;

// a light source to illuminate the objects in the world
cSpotLight *light;

// a haptic device handler
cHapticDeviceHandler *handler;

// a pointer to the current haptic device
cGenericHapticDevicePtr hapticDevice;

HapticMode hapticMode;

// highest stiffness the current haptic device can render
double hapticDeviceMaxStiffness;

// sphere objects
vector<Atom *> spheres;

// a colored background
cBackground *background;

cLabel *hapticPositionLabel;

// a label to display the rate [Hz] at which the simulation is running
cLabel *labelRates;

// a label to show the potential energy
cLabel *LJ_num;

// label showing the # anchored
cLabel *num_anchored;

// a label to display the total energy of the system
cLabel *total_energy;

// a label to show whether or not the atoms are frozen
cLabel *isFrozen;

// a label to display the camera position
cLabel *camera_pos;

// a label to identify the potential energy surface
cLabel *potentialLabel;

// labels for the scope
cLabel *scope_upper;
cLabel *scope_lower;

// a flag that indicates if the haptic simulation is currently running
bool simulationRunning = false;

// a flag that indicates if the haptic simulation has terminated
bool simulationFinished = true;

// mouse state
MouseState mouseState = MOUSE_IDLE;

// a frequency counter to measure the simulation graphic rate
cFrequencyCounter freqCounterGraphics;

// a frequency counter to measure the simulation haptic rate
cFrequencyCounter freqCounterHaptics;

// haptic thread
cThread *hapticsThread;

// a handle to window display context
GLFWwindow *window = NULL;

// a handle to slider control window
GLFWwindow *sliderWindow = NULL;

// current width of window
int width = 0;

// current height of window
int height = 0;

// swap interval for the display context (vertical synchronization)
int swapInterval = 1;

// root resource path
string resourceRoot;

// a scope to monitor the potential energy
cScope *scope;

// global minimum for the given cluster size
double global_minimum;

// a pointer to the selected object
Atom *selectedAtom = NULL;

// offset between the position of the mmouse click on the object and the object
// reference frame location.
cVector3d selectedAtomOffset;

// position of mouse click.
cVector3d selectedPoint;


std::atomic<bool> freezeAtoms(false); // determine if atoms should be frozen
double centerCoords[3] = {50.0, 50.0, 50.0}; // save coordinates of central atom
LocalPotential energySurface = LENNARD_JONES; // default potential is Lennard Jones
bool global_min_known = true; // check if able to read in the global min
cPanel *helpPanel; // panel that displays hotkeys
cLabel *helpHeader; // help panel header
vector<cLabel *> hotkeyKeys; // vector holding hotkey key labels

// vector holding function key labels (must be separate for formatting)
vector<cLabel *> hotkeyFunctions;

// keep track of how long screenshot label has been displayed
std::atomic<int> screenshotCounter(-2);

// keep track of how long write to con label has been displayed
std::atomic<int> writeConCounter(-2);

std::atomic<double> displayedPotentialEnergy(0.0);
std::atomic<int> displayedAnchoredCount(0);
std::recursive_mutex sceneMutex;
std::atomic<bool> hapticsThreadStarted(false);
int currentIndex = 0;

// screenshot notification label
cLabel *screenshotLabel;

// write to con notification label
cLabel *writeConLabel;

cVector3d hapticPosition;
static std::vector<cVector3d> prevPositions;

void printIntro();

void initializeGLFW();
void setOpenGLVersion();
void initializeGLEW();

void initializeWorld();
void initializeCamera();
void initializeLight();
void initializeHapticDevice();

void placeAtoms(std::array<double, 9> aseCell, std::array<int, 3> asePbc, int argc, char *argv[]);
Atom* initializeAtom(cTexture2dPtr texture, int atomicNumber);
void initializeAtomPosition(Atom *new_atom);
void initializeCalculator(int argc, char *argv[], std::array<double, 9> aseCell,
    std::array<int, 3> asePbc);
void initializeLabels();
void initializeHotkeyLabels();
void initializePotentialLabel();
void initializePotentialEnergyPlot();
void initializeHelpPanel();
void initializeHapticThread();
void initializeSliderUI();
void runGraphicsLoop();
void renderSliderWindow();
double getSimulationTimeStep();
void updateSliderWindowTitle();
void sliderWindowSizeCallback(GLFWwindow *a_window, int a_width, int a_height);
void sliderWindowCursorPosCallback(GLFWwindow *a_window, double a_posX, double a_posY);
void sliderWindowMouseButtonCallback(GLFWwindow *a_window, int a_button, int a_action, int a_mods);

// callback when the window display is resized
void windowSizeCallback(GLFWwindow *a_window, int a_width, int a_height);

// callback when an error GLFW occurs
void errorCallback(int error, const char *a_description);

// this function updates the draw positions
void updateGraphics(void);

// this function contains the main haptics simulation loop
void updateHaptics(void);

// this function advances the atom simulation for one time step
cVector3d stepSimulation(const cVector3d &position,
                         const double timeInterval,
                         const bool hasHapticDevice);

// this function closes the application
void close(void);

// add a label to the world with default black text
void addLabel(cLabel *&label);

// Update camera text
void updateCameraLabel(cLabel *&camera_pos, cCamera *&camera);

// save configuration in .con file
void writeToCon(string fileName);

//------------------------------------------------------------------------------
// RESOURCE HELPERS
//------------------------------------------------------------------------------
static string getExecutableDir() {
  char buffer[4096];
  ssize_t length = readlink("/proc/self/exe", buffer, sizeof(buffer) - 1);
  if (length <= 0) {
    return "";
  }
  buffer[length] = '\0';
  string executablePath(buffer);
  size_t separator = executablePath.find_last_of('/');
  if (separator == string::npos) {
    return "";
  }
  return executablePath.substr(0, separator);
}

static vector<string> getResourceSearchRoots() {
  vector<string> roots;
  if (!resourceRoot.empty()) {
    roots.push_back(resourceRoot);
  }
  string executableDir = getExecutableDir();
  if (!executableDir.empty()) {
    roots.push_back(executableDir + "/");
    roots.push_back(executableDir + "/../");
  }

  roots.push_back("./");
  roots.push_back("../");
  roots.push_back("../bin/");

  return roots;
}

template <typename Loader>
static bool loadChaiResource(Loader loader, const string &relativePath) {
  for (const string &root : getResourceSearchRoots()) {
    const string candidate = root + relativePath;
    if (loader(candidate.c_str())) {
      return true;
    }
  }
  return false;
}

static bool isFiniteVector(const cVector3d &value) {
  return std::isfinite(value.x()) && std::isfinite(value.y()) && std::isfinite(value.z());
}

static cVector3d clampVectorMagnitude(const cVector3d &value, const double maxMagnitude) {
  if (!isFiniteVector(value)) {
    return cVector3d(0.0, 0.0, 0.0);
  }
  const double length = value.length();
  if (length > maxMagnitude && length > 0.0) {
    return maxMagnitude * cNormalize(value);
  }
  return value;
}

//==============================================================================
/*
 LJ.cpp
 This program simulates LJ clusters of varying sizes using modified
 sphere primitives (atom.cpp). All dynamics and collisions are computed in the
 haptics thread.
 */
//==============================================================================
// current camera
int curr_camera = 1;

int main(int argc, char *argv[]) {
  printIntro();
  srand(time(NULL)); // initialize random seed
  
  // OPEN GL - WINDOW DISPLAY
  initializeGLFW();
  initializeGLEW();

  // WORLD - CAMERA - LIGHTING
  initializeWorld();
  initializeCamera();
  initializeLight();
  
  // HAPTIC DEVICE
  initializeHapticDevice();
  
  string hapticModeStr = argv[1];
  if (hapticModeStr == "force" || hapticModeStr == "f") {
    hapticMode = HapticMode::Force;
  } else if (hapticModeStr == "position" || hapticModeStr == "p") {
    hapticMode = HapticMode::Position;
  } else if (hapticModeStr == "standby" || hapticModeStr == "s") {
    hapticMode = HapticMode::Standby;
  } else {
    throw std::runtime_error("First argument must be a haptic mode: \"force\", \"position\", \"standby\"");
  }

  // Declare variables needed for calculator constructor (cell, pbc), atoms object 
  // (mass, atomic number), and placing of initial atoms (positions)
  std::array<double, 9> aseCell = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  std::array<int, 3> asePbc = {0, 0, 0};

  // PLACE ATOMS
  placeAtoms(aseCell, asePbc, argc, argv);

  // determine potential if specified
  if (argc > 3) {
    initializeCalculator(argc, argv, aseCell, asePbc);
  } else {
    cerr << "No potential specified. Defaulting to Lennard-Jones." << endl;
    calculatorPtr = new ljCalculator();
  }

  // WIDGETS
  initializeLabels();
  initializePotentialEnergyPlot();
  initializeHelpPanel();
  initializeSliderUI();
  
  // START SIMULATION
  initializeHapticThread();
  
  // MAIN GRAPHIC LOOP
  runGraphicsLoop();
  close();

  // close window
  if (sliderWindow != NULL) {
    glfwDestroyWindow(sliderWindow);
    sliderWindow = NULL;
  }
  glfwDestroyWindow(window);
  window = NULL;

  glfwTerminate(); // terminate GLFW library
  return 0; // exit
}

void windowSizeCallback(GLFWwindow *a_window, int a_width, int a_height) {
  // update window size
  width = a_width;
  height = a_height;
}

void errorCallback(int a_error, const char *a_description) {
  cout << "Error: " << a_description << endl;
}

void printIntro() {
  cout << endl;
  cout << "-----------------------------------" << endl;
  cout << "CHAI3D" << endl;
  cout << "Press CTRL for help" << endl;
  cout << "-----------------------------------" << endl
       << endl
       << endl;
  cout << endl
       << endl;
}

void initializeGLFW() {
  if (!glfwInit()) {
    cSleepMs(1000);
    throw std::runtime_error("Failed to initialize GLFW library!");
  }

  glfwSetErrorCallback(errorCallback); // set error callback
  setOpenGLVersion();
  // set active stereo mode
  stereoMode == C_STEREO_ACTIVE ? glfwWindowHint(GLFW_STEREO, GL_TRUE)
                                : glfwWindowHint(GLFW_STEREO, GL_FALSE);

  // compute desired size of window
  const GLFWvidmode *mode = glfwGetVideoMode(glfwGetPrimaryMonitor());

  //    const w *mode = glfwGetVideoMode(glfwGetPrimaryMonitor());
  int windowWidth = 0.8 * mode->height;
  int windowHeight = 0.5 * mode->height;
  int windowX = 0.5 * (mode->width - windowWidth);
  int windowY = 0.5 * (mode->height - windowHeight);

  // create display context
  window = glfwCreateWindow(windowWidth, windowHeight, "CHAI3D", NULL, NULL);
  if (!window) {
    cSleepMs(1000);
    glfwTerminate();
    throw std::runtime_error("Failed to create window!");
  }

  sliderWindow = glfwCreateWindow(340, 170, "Controls", NULL, window);
  if (!sliderWindow) {
    cSleepMs(1000);
    glfwTerminate();
    throw std::runtime_error("Failed to create slider window!");
  }

  glfwGetWindowSize(window, &width, &height); // get width and height of window
  glfwSetWindowPos(window, windowX, windowY); // set position of window
  glfwSetWindowPos(sliderWindow, windowX + windowWidth + 20, windowY);
  glfwSetKeyCallback(window, keyCallback); // set key callback
  glfwSetCursorPosCallback(window, mouseMotionCallback); // set mouse position callback
  glfwSetMouseButtonCallback(window, mouseButtonCallback); // set mouse button callback
  glfwSetWindowSizeCallback(window, windowSizeCallback); // set resize callback
  glfwSetCursorPosCallback(sliderWindow, sliderWindowCursorPosCallback);
  glfwSetMouseButtonCallback(sliderWindow, sliderWindowMouseButtonCallback);
  glfwSetWindowSizeCallback(sliderWindow, sliderWindowSizeCallback);
  glfwMakeContextCurrent(window); // set current display context
  glfwSwapInterval(swapInterval); // sets the swap interval for the current display context
}

void setOpenGLVersion() {
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 2);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);
}

void initializeGLEW() {
  #ifdef GLEW_VERSION
  if (glewInit() != GLEW_OK) {
    glfwTerminate();
    throw std::runtime_error("Failed to initialize GLEW library!");
  }
  #endif
}
void initializeWorld() {
  world = new cWorld();
  world->m_backgroundColor.setWhite();
  world->setShadowIntensity(0.3); // set shadow factor
}

void initializeCamera() {
  camera = new cCamera(world);
  world->addChild(camera);

  // creates the radius, origin reference, along with the zenith and azimuth direction vectors
  cVector3d origin(0.0, 0.0, 0.0);
  cVector3d zenith(0.0, 0.0, 1.0);
  cVector3d azimuth(1.0, 0.0, 0.0);

  // sets the camera's references of the origin, zenith, and azimuth
  camera->setSphericalReferences(origin, zenith, azimuth);

  // sets the camera's position to have a radius of .1, located at 0 radians (vertically and horizontally)
  camera->setSphericalRad(rho, 0, 0);

  // set the near and far clipping planes of the camera anything in front or behind these clipping
  // planes will not be rendered
  camera->setClippingPlanes(0.01, 10.0);

  camera->setStereoMode(stereoMode);  // set stereo mode

  // set stereo eye separation and focal length (applies only if stereo is enabled)
  camera->setStereoEyeSeparation(0.03);
  camera->setStereoFocalLength(1.8);
  camera->setMirrorVertical(false); // set vertical mirrored display mode

    // create a background
  background = new cBackground();
  camera->m_backLayer->addChild(background);

  // set aspect ration of background image a constant
  background->setFixedAspectRatio(true);

  // load background image
  bool fileload = loadChaiResource(
      [&](const char *path)
      { return background->loadFromFile(path); },
      "resources/images/background.png");
  if (!fileload) {
    close();
    throw std::runtime_error("Failed to load background image!");
  }
}

void initializeLight() {
  light = new cSpotLight(world); // create a light source
  world->addChild(light); // attach light to camera
  light->setEnabled(true); // enable light source
  light->setLocalPos(0.0, 0.3, 0.4); // position the light source
  light->setDir(0.0, -0.25, -0.4); // define the direction of the light beam
  light->setShadowMapEnabled(false); // enable this light source to generate shadows
  light->m_shadowMap->setQualityHigh(); // set the resolution of the shadow map
  light->setCutOffAngleDeg(30); // set light cone half angle
}

void initializeHapticDevice() {
  handler = new cHapticDeviceHandler(); // create a haptic device handler
  // get access to the first available haptic device
  if (handler->getNumDevices() > 0) {
    handler->getDevice(hapticDevice, 0);
  }
  if (hapticDevice) {
    // retrieve the highest stiffness this device can render
    hapticDeviceMaxStiffness = hapticDevice->getSpecifications().m_maxLinearStiffness;

    // if the haptic devices carries a gripper, enable it to behave like a user switch
    hapticDevice->setEnableGripperUserSwitch(true);
  } else {
    hapticDeviceMaxStiffness = HAPTIC_STIFFNESS;
    cout << "No haptic device detected. Running in keyboard/mouse-only mode." << endl;
  }
}

void placeAtomsAse(std::array<double, 9> aseCell, std::array<int, 3> asePbc, cTexture2dPtr texture, char *argv[]) {
  AseStructureData structure;
  try {
    structure = loadAseStructure(argv[2]);
  } catch (const std::exception &ex) {
    close();
    throw std::runtime_error(ex.what());
  }
  const std::vector<std::array<double, 3>> &positions = structure.positions;
  const std::vector<int> &startingAtomicNrs = structure.atomicNumbers;
  aseCell = structure.cell;
  asePbc = structure.pbc;
  const int nAtoms = static_cast<int>(positions.size());

  for (int i = 0; i < nAtoms; i++) {
    Atom *newAtom = initializeAtom(texture, startingAtomicNrs[i]); // Create atom pointer
    // Set the positions of all atoms
    if (i == 0) {
      // make very first atom the current atom
      newAtom->setCurrent(true);
      // get coordinates from pPositionTriplet
      for (int j = 0; j < 3; j++) {
        centerCoords[j] = positions[0][static_cast<size_t>(j)];
      }
      newAtom->setLocalPos(0.0, 0.0, 0.0); // set first atom at center of view
    } else {
      // newAtom->setAnchor(true); // Anchor by default
      // scale coordinates and insert
      newAtom->setLocalPos(
          0.02 * (positions[i][0] - centerCoords[0]), // position offset -- should probably disappear once we get boxes working
          0.02 * (positions[i][1] - centerCoords[1]),
          0.02 * (positions[i][2] - centerCoords[2]));
    }
  }
}

void placeAtoms(std::array<double, 9> aseCell, std::array<int, 3> asePbc, int argc, char *argv[]) {
  cTexture2dPtr texture = cTexture2d::create(); // create texture
  // load texture file
  bool fileload = loadChaiResource([&](const char *path)
      { return texture->loadFromFile(path); },
      "resources/images/grayball.jpg");
  if (!fileload){
    close();
    throw std::runtime_error("Failed to load texture!");
  }

  // either no additional arguments were given or second argument was an integer
  if (argc == 2 || isNumber(argv[2])) {
    // set numSpheres to input; if none or negative, default is five
    int numSpheres = argc > 2 ? atoi(argv[2]) : 5;
    for (int i = 0; i < numSpheres; i++) {
      // initialize atom with texture and atomic number of 1 (hydrogen)
      Atom *new_atom = initializeAtom(texture, 1); 
      if (i == 0) {
        new_atom->setCurrent(true); // set the first sphere to the current
      } else {
        initializeAtomPosition(new_atom); // set the position of the atom
      }
    }
  } else // read in specified file
    placeAtomsAse(aseCell, asePbc, texture, argv);

  // Done reading any sort of info.
  for (int i = 0; i < spheres.size(); i++) {
    spheres[i]->setVelocity(0);
  }
}

Atom* initializeAtom(cTexture2dPtr texture, int atomicNumber) {
  Atom *new_atom = new Atom(SPHERE_RADIUS, atomicNumber); // create a atom and define its radius
  spheres.push_back(new_atom); // store pointer to atom
  world->addChild(new_atom); // add atom to world
  world->addChild(new_atom->getVelVector()); // add line to world

  // set graphic properties of sphere
  new_atom->setTexture(texture);
  new_atom->m_texture->setSphericalMappingEnabled(true);
  new_atom->setUseTexture(true);
  return new_atom;
}

void initializeAtomPosition(Atom *new_atom) {
  bool inside_atom = true;
  auto iter{0};
  while (inside_atom) {
    if (iter <= 1000) {
      // Place atom at a random position
      new_atom->setInitialPosition();
    } else {
      // If there are too many failed attempts at placing the atom increase the radius in
      // which it can spawn
      new_atom->setInitialPosition(.115);
    }
    // Check that it doesn't collide with any others
    bool collision_detected = false;
    for (auto i{0}; i < spheres.size(); i++) {
      if (new_atom != spheres[i]) {
        auto dist_between = cDistance(new_atom->getLocalPos(), spheres[i]->getLocalPos());
        if (dist_between < SPHERE_RADIUS * 2) {
          // The number dist between is being compared to is the threshold for collision
          collision_detected = true;
          iter++;
          break;
        }
      }
    }
    if (!collision_detected){
      inside_atom = false;
    }
    
  }
  // cout << "Placing atom at " << new_atom->getLocalPos() << endl;
}

void initializeCalculator(int argc, char *argv[], std::array<double, 9> aseCell,
    std::array<int, 3> asePbc) {
    string potential = argv[3];
    for (char &c : potential) {
      c = tolower(c);
    }
    if (potential == "morse" || potential == "m") {
      energySurface = MORSE;
      calculatorPtr = new morseCalculator();
    } else if (potential == "ase" || potential == "a") {
      energySurface = ASE;
      calculatorPtr = new aseCalculator((argc > 4) ? argv[4] : "", aseCell, asePbc);
    } else if (potential == "lennard-jones" || potential == "lj") {
      calculatorPtr = new ljCalculator();
    } else {
      cerr << "Warning: unknown potential '" << potential
           << "'. Defaulting to Lennard-Jones." << endl;
      energySurface = LENNARD_JONES;
      calculatorPtr = new ljCalculator();
    }
}

void initializeLabels() {
  addLabel(hapticPositionLabel); // label to read haptic device
  addLabel(labelRates); // create a label to display the haptic and graphic rate of the simulation
  addLabel(LJ_num); // potential energy label
  addLabel(num_anchored); // number anchored label
  addLabel(total_energy); // total energy label
  addLabel(isFrozen); // frozen state label
  addLabel(camera_pos); // camera position label
  addLabel(potentialLabel); // energy surface label

  // Add labels to the graph
  addLabel(scope_upper); 
  addLabel(scope_lower);

  hapticPositionLabel->setLocalPos(0, 50);

  cFontPtr notificationFont = NEW_CFONT_CALIBRI_20();
  writeConLabel = new cLabel(notificationFont);
  writeConLabel->m_fontColor.setBlack();
  screenshotLabel = new cLabel(notificationFont);
  screenshotLabel->m_fontColor.setBlack();
  camera->m_frontLayer->addChild(writeConLabel);
  camera->m_frontLayer->addChild(screenshotLabel);
  writeConLabel->setShowEnabled(false);
  screenshotLabel->setShowEnabled(false);

  //initializeHotkeyLabels();

  screenshotLabel->setText("Screenshot taken");
  writeConLabel->setText("Con file written");

  initializePotentialLabel();

  // sets the text for the camera position to appear on screen
  camera_pos->setLocalPos(0, 30, 0);
  updateCameraLabel(camera_pos, camera);
}

void initializeHotkeyLabels() {
  addHotkeyLabel("f", "toggle fullscreen");
  addHotkeyLabel("q, ESC", "quit program");
  addHotkeyLabel("a", "anchor all atoms");
  addHotkeyLabel("u", "unanchor all atoms");
  addHotkeyLabel("ARROW KEYS", "rotate camera");
  addHotkeyLabel("[", "zoom in");
  addHotkeyLabel("]", "zoom out");
  addHotkeyLabel("r", "reset camera");
  addHotkeyLabel("s", "screenshot atoms");
  addHotkeyLabel("c", "save configuration to .con");
  addHotkeyLabel("SPACE", "freeze atoms");
  addHotkeyLabel("CTRL", "toggle help panel");
}

void initializePotentialLabel() {
  // set energy surface label
  potentialLabel->setLocalPos(0, 0);
  string potentialName;
  if (energySurface == LENNARD_JONES) {
    potentialName = "Lennard Jones Potential";
  } else if (energySurface == MORSE) {
    potentialName = "Morse Potential";
  } else if (energySurface == ASE) {
    potentialName = "ASE Potential";
  }
  potentialLabel->setText("Potential energy surface: " + potentialName);
}

void initializePotentialEnergyPlot() {
  // create a scope to plot potential energy
  scope = new cScope();
  scope->setLocalPos(0, 60);
  camera->m_frontLayer->addChild(scope);
  scope->setSignalEnabled(true, true, false, false);
  scope->setTransparencyLevel(.7);
  global_minimum = getGlobalMinima(spheres.size());
  double lower_bound, upper_bound;
  if (global_minimum != 0 && (energySurface == LENNARD_JONES)) {
    if (global_minimum > -50) {
      upper_bound = 0;
      lower_bound = global_minimum - .5;
    } else {
      upper_bound = 0 + (global_minimum * .2);
      lower_bound = global_minimum - 3;
    }
    global_min_known = true;
  } else {
    upper_bound = 0;
    lower_bound = static_cast<int>(spheres.size()) * -3;
    global_minimum = 0;
    global_min_known = false;
  }
  scope->setRange(lower_bound, upper_bound);
  scope_upper->setText(cStr(upper_bound));
  scope_lower->setText(cStr(lower_bound));

  // Height was guessed and added manually - there's probably a better way
  // To do this but the scope height is protected
  scope_upper->setLocalPos(cAdd(scope->getLocalPos(), cVector3d(0, 180, 0)));
  scope_lower->setLocalPos(scope->getLocalPos());
  // TODO - make more legible
  // scope_upper->m_fontColor.setRed();
  // scope_lower->m_fontColor.setRed();
}

void initializeHelpPanel() {
  cColorf panelColor = cColorf();
  panelColor.setBlueCadet();

  helpPanel = new cPanel();
  helpPanel->setColor(panelColor);
  helpPanel->setSize(520, 500);
  camera->m_frontLayer->addChild(helpPanel);
  helpPanel->setShowPanel(false);

  initializeHotkeyLabels();

  cFontPtr headerFont = NEW_CFONT_CALIBRI_40();
  helpHeader = new cLabel(headerFont);
  helpHeader->m_fontColor.setBlack();
  helpHeader->setText("HOTKEYS AND INSTRUCTIONS");
  helpHeader->setShowPanel(false);
  helpHeader->setShowEnabled(false);
  camera->m_frontLayer->addChild(helpHeader);
}

void initializeHapticThread() {
  hapticsThread = nullptr; // create a thread which starts the main haptics rendering loop
  if (hapticDevice) {
    hapticsThread = new cThread();
    hapticsThread->start(updateHaptics, CTHREAD_PRIORITY_HAPTICS);
  }
}

void initializeprevPositions() {
  prevPositions.resize(spheres.size());
  for (int i = 0; i < spheres.size(); i++) {
    prevPositions[i] = spheres[i]->getLocalPos();
  }
}

void runGraphicsLoop() {
  windowSizeCallback(window, width, height); // call window size callback at initialization
  cPrecisionClock keyboardModeClock;
  keyboardModeClock.reset();
  keyboardModeClock.start();
  // main graphic loop
  while (!glfwWindowShouldClose(window)) {
    glfwGetWindowSize(window, &width, &height); // get width and height of window
    if (!hapticDevice) {
      keyboardModeClock.stop();
      double timeInterval = cMin(getSimulationTimeStep(), keyboardModeClock.getCurrentTimeSeconds());
      keyboardModeClock.start(true);
      freqCounterHaptics.signal(1);
      initializeprevPositions();
      stepSimulation(cVector3d(0.0, 0.0, 0.0), timeInterval, false);
    }
    updateGraphics(); // render graphics
    glfwSwapBuffers(window); // swap buffers
    renderSliderWindow();
    glfwPollEvents(); // process events
    freqCounterGraphics.signal(1); // signal frequency counter
  }
}

void close(void) { // stop the simulation
  static bool closed = false;
  if (!closed) {
    closed = true;
    simulationRunning = false;
    if (hapticsThreadStarted.load()) {
      // wait for graphics and haptics loops to terminate
      while (!simulationFinished) {
        cSleepMs(100);
      }
    }
    if (calculatorPtr != nullptr) {
      delete calculatorPtr;
      calculatorPtr = nullptr;
    }
    // delete resources
    delete hapticsThread;
    hapticsThread = nullptr;
    delete world;
    world = nullptr;
    delete handler;
    handler = nullptr;
  }
}

void updateCounters(cLabel *label, std::atomic<int> &counter) {
  int value = counter.load();
  if (value == 5000) {
    label->setShowEnabled(true);
  } else if (value == 0) {
    label->setShowEnabled(false);
  }
  counter--;
}

void updateLabels() {
  // update haptic and graphic rate data
  labelRates->setText(cStr(freqCounterGraphics.getFrequency(), 0) + " Hz / " +
                      cStr(freqCounterHaptics.getFrequency(), 0) + " Hz");
  // update position of label
  labelRates->setLocalPos((int)(0.5 * (width - labelRates->getWidth())), 15);
  double x = hapticPosition.get(0);
  double y = hapticPosition.get(1);
  double z = hapticPosition.get(2);
  hapticPositionLabel->setText("Position: " + cStr(x, 2) + ", " + cStr(y, 2) + ", " + cStr(z, 2));


  updateCameraLabel(camera_pos, camera);

  string trueFalse = freezeAtoms.load() ? "true" : "false";
  isFrozen->setText("Freeze simulation: " + trueFalse);
  isFrozen->setLocalPos((width - isFrozen->getWidth()) - 5, 15);

  screenshotLabel->setLocalPos(5, height - 20);
  updateCounters(screenshotLabel, screenshotCounter);

  writeConLabel->setLocalPos(5, height - 40);
  updateCounters(writeConLabel, writeConCounter);

  for (int i = 0; i < hotkeyKeys.size(); i++) {
    cLabel *tempKeyLabel = hotkeyKeys[i];
    cLabel *tempFuncLabel = hotkeyFunctions[i];
    tempKeyLabel->setLocalPos(width - 530, height - 105 - i * 35);
    tempFuncLabel->setLocalPos(width - 350, height - 105 - i * 35);
  }
}

void updateGraphics(void) {
  std::lock_guard<std::recursive_mutex> lock(sceneMutex);

  // UPDATE WIDGETS
  updateLabels();
  helpPanel->setLocalPos(width - 550, height - 530);
  helpHeader->setLocalPos(width - 490, height - 70);
  
  const double potentialEnergy = displayedPotentialEnergy.load();
  LJ_num->setText("Potential Energy: " + cStr(potentialEnergy, 5));
  LJ_num->setLocalPos(0, 15, 0);

  num_anchored->setText(to_string(displayedAnchoredCount.load()) + " anchored / " +
                        to_string(spheres.size()) + " total");
  num_anchored->setLocalPos((width - num_anchored->getWidth()) - 5, 0);

  scope->setSignalValues(potentialEnergy, global_minimum);
  if (!global_min_known && global_minimum < scope->getRangeMin()) {
    auto new_lower = scope->getRangeMin() - 25;
    auto new_upper = scope->getRangeMax() - 25;
    scope->setRange(new_lower, new_upper);
    scope_upper->setText(cStr(scope->getRangeMax()));
    scope_lower->setText(cStr(scope->getRangeMin()));
  }

  // RENDER SCENE
  world->updateShadowMaps(false, false); // update shadow maps (if any)
  camera->renderView(width, height); // render world
  glFinish(); // wait until all GL commands are completed
  GLenum err = glGetError(); // check for any OpenGL errors
  if (err != GL_NO_ERROR)
    cout << "Error: " << gluErrorString(err) << endl;
}



void switchCamera() {
  switch (curr_camera) {
    case 1:
      camera->setSphericalPolarRad(0);
      camera->setSphericalAzimuthRad(0);
      break;
    case 2:
      camera->setSphericalPolarRad(0);
      camera->setSphericalAzimuthRad(M_PI);
      break;
    case 3:
      camera->setSphericalPolarRad(M_PI);
      camera->setSphericalAzimuthRad(M_PI);
      break;
    case 4:
      curr_camera = 0;
      camera->setSphericalPolarRad(M_PI);
      camera->setSphericalAzimuthRad(0);
      break;
  }
  curr_camera++;
}

void switchCurrentAtom() {
  Atom* current = spheres[currentIndex];
  int prev_curr_atom = currentIndex;
  currentIndex = remainder(currentIndex + 1, spheres.size());
  if (currentIndex < 0)
    currentIndex += spheres.size();
  int startAtom = currentIndex;
  while (spheres[currentIndex]->isAnchor()) {
    currentIndex = remainder(currentIndex + 1, spheres.size());
    if (currentIndex < 0)
      currentIndex += spheres.size();
    if (currentIndex == startAtom)
      break;
  }

  current = spheres[currentIndex];
  current->setCurrent(true);

  Atom *prev = spheres[prev_curr_atom];
  prev->setCurrent(false);
}
void readButtons(bool buttons[4], bool buttonReset[4]) {
  for (int i = 0; i < 4; i++) {
    hapticDevice->getUserSwitch(i, buttons[i]);
    if (buttons[i]) {
      if (buttonReset[i]) {
        switch (i) {
          case 1:
            switchCurrentAtom();
            break;
          case 2:
            switchCamera();
            break;
          default:
            cout << "Button " << i << " has not yet been defined!" << endl;
            break;
        }
        buttonReset[i] = false;
      } 
    } else {
      buttonReset[i] = true;
    }
  }
}



void applyBoundaryConditions(cVector3d &x_curr) {
  applyDavidBoundaryConditions(x_curr, x_curr);
  applySeanBoundaryConditions(
      x_curr, x_curr, x_curr,
      northPlanePos, northPlaneNorm,
      southPlanePos, southPlaneNorm,
      eastPlanePos, eastPlaneNorm,
      westPlanePos, westPlaneNorm,
      forwardPlanePos, forwardPlaneNorm,
      backPlanePos, backPlaneNorm,
      BOUNDARY_LIMIT);
}

cVector3d getNewAtomPosition(Atom *atom, cVector3d &prev_position, const double timeInterval) {
  cVector3d x_curr = atom->getLocalPos();
  cVector3d force = atom->getForce();
  cVector3d acc = force / (atom->getMass() * SPHERE_MASS_SCALE_FACTOR);
  if (!isFiniteVector(acc)) {
    acc.zero();
  }
  
  // update position using Verlet integration
  return x_curr + (x_curr - prev_position) + acc * timeInterval * timeInterval;
}

cVector3d forceModeUpdate(Atom *current, cVector3d position, const double timeInterval) {
  // spring constant haptic device feels
  const double K_HAPTIC = 100;
  // spring constant atom feels
  const double K_CURRENT = 100;

  cVector3d positionErr = position - current->getLocalPos();

  cVector3d hapticForce = positionErr * K_CURRENT;
  current->setForce(current->getForce() + hapticForce);

  cVector3d currentPrevPos = prevPositions[currentIndex];
  cVector3d currentPos = current->getLocalPos();
  current->setLocalPos(getNewAtomPosition(current, currentPrevPos, timeInterval));
  prevPositions[currentIndex] = currentPos;

  return (current->getLocalPos() - position) * K_HAPTIC;
}

bool prevHapticInitialized;
cVector3d prevHapticPosition(0,0,0);
cPrecisionClock positionClock;
bool standby;
bool resetting;
bool confirming;
bool simulating;
double finalErr;

const int MAX_FORCE_HISTORY = 10000;
cVector3d prevForces[MAX_FORCE_HISTORY];
int prevForcesIndex = 0;

double getStrongestScalarProj(cVector3d v) {
  double strongest = 0;
  for (int i = 0; i < MAX_FORCE_HISTORY; i++) {
    double scalarProj = v.dot(prevForces[i]) / v.length();
    if (strongest < scalarProj) {
      strongest = scalarProj;
    }
  }
  return strongest;
}

void recordForceHistory(Atom *current) {
  constexpr double REST_ERR = 0.001;

  if (!simulating) return;
  if (current->getForce().length() < REST_ERR) return;

  prevForces[prevForcesIndex] = current->getForce();
  prevForcesIndex = (prevForcesIndex + 1) % MAX_FORCE_HISTORY;
}

std::optional<cVector3d> updateStandbyModeSimulating(Atom *current, cVector3d& position, double timeInterval) {
  constexpr double HAPTIC_RADIUS = .08;
  constexpr double K_HAPTIC = 1; // spring constant for applying vector projection

  if (position.length() < HAPTIC_RADIUS) {
    cVector3d temp = current->getLocalPos();
    current->setLocalPos(getNewAtomPosition(current, prevPositions[currentIndex], timeInterval));
    prevPositions[currentIndex] = temp;

    double distFromCenter = position.length() - finalErr;
    position.normalize();
    return getStrongestScalarProj(-position) * -position * (distFromCenter / HAPTIC_RADIUS) * K_HAPTIC;
  }
  simulating = false;
  return std::nullopt;
}

std::optional<cVector3d> updateStandbyState(Atom *current, const cVector3d& position) {
  // position err acceptable for return mechanism to return to center
  constexpr double SETTLING_ERR = .05; 
  constexpr double K_RETURN = 25;

  if (position.length() >= SETTLING_ERR) {
    if (resetting && confirming) {
      cout << "Not yet settled!" << endl;
    }
    confirming = false;
    if (positionClock.getCurrentTimeSeconds() >= 2.5 || resetting) {
      positionClock.stop();
      positionClock.reset();
      if (!resetting) {
        cout << "Resetting to center..." << endl;
      }
      resetting = true;
      
      const double MAX_FORCE = 1.6; // maximum force the return mechanism should output
      
      cVector3d returnVector = -position * K_RETURN;
      return clampVectorMagnitude(returnVector, MAX_FORCE);
    }
  } else {
    if (!confirming) {
      cout << "Starting confirmation..." << endl;
      positionClock.start();
      confirming = true;
    }
    if (positionClock.getCurrentTimeSeconds() >= .5) {
      cout << "Done!" << endl;
      standby = false;
      resetting = false;
      confirming = false;
      prevHapticInitialized = false;
      simulating = true;
      positionClock.stop();
      positionClock.reset();

      prevPositions[currentIndex] = current->getLocalPos();
      finalErr = position.length();
    }
    return cVector3d(0,0,0);
  }
  return std::nullopt;
}

cVector3d standbyModeUpdate(Atom *current, cVector3d position, const double timeInterval) {
  constexpr double STANDBY_ERR = .1; // movement err acceptable for standby mode to activate

  if (!prevHapticInitialized) {
    prevHapticInitialized = true;
    prevHapticPosition = position;
  }
  recordForceHistory(current);

  cVector3d dPHaptic = position - prevHapticPosition;
  prevHapticPosition = position;

  if (simulating) {
    auto pos = updateStandbyModeSimulating(current, position, timeInterval);
    if (pos) return *pos;
  } else {
    if (dPHaptic.length() < STANDBY_ERR && !standby && position.length() >= .01) {
      positionClock.start();
      standby = true;
      cout << "Entering standby mode..." << endl;
    }
    if (standby) {
      auto pos = updateStandbyState(current, position);
      if (pos) return *pos;
    }

    if (dPHaptic.length() >= 1e-6 && standby && !resetting) { 
      standby = false;
      positionClock.stop();
      positionClock.reset();
      cout << "Standby cancelled!" << endl;
    }
  }

  current->setLocalPos(current->getLocalPos() + dPHaptic);
  return current->getForce();
}

cVector3d positionModeUpdate(Atom *current, cVector3d position, const double timeInterval) {
  const double VELOCITY_MULT = 25;
  const double ATTRACTION_MAX = 1.5; 
  cVector3d currentPos = current->getLocalPos();
  cVector3d attraction = (position - currentPos) * timeInterval * VELOCITY_MULT;
  current->setLocalPos(currentPos + clampVectorMagnitude(attraction, ATTRACTION_MAX * timeInterval));
  prevPositions[currentIndex] = currentPos;
  return cVector3d(0,0,0);
}


cVector3d stepSimulation(const cVector3d &requestedPosition, const double timeInterval,
                        const bool hasHapticDevice) {
  std::lock_guard<std::recursive_mutex> lock(sceneMutex);
  if (spheres.empty()) {
    return cVector3d(0.0, 0.0, 0.0);
  }

  Atom *current = spheres[currentIndex];
  cVector3d position = hasHapticDevice ? requestedPosition : current->getLocalPos();

  cVector3d currentPosition(0,0,0);
  
  if (!freezeAtoms.load()) {
    if (!calculatorPtr) {
      cerr << "Error: calculatorPtr is null in stepSimulation()" << endl;
      return cVector3d(0.0, 0.0, 0.0);
    }
    vector<vector<double>> forcesVec = calculatorPtr->getFandU(spheres);
    double potentialEnergy = forcesVec[spheres.size()][0];

    for (int i = 0; i < spheres.size(); i++) {
      Atom *atom = spheres[i];
      cVector3d force(forcesVec[i][0], forcesVec[i][1], forcesVec[i][2]);
      if (!isFiniteVector(force)) {
        force.zero();
      }
      atom->setForce(force);
    }
    for (int i = 0; i < spheres.size(); i++) {
      Atom *atom = spheres[i];
      if (!atom->isCurrent() && !atom->isAnchor()) {
        cVector3d x_curr = atom->getLocalPos();
        cVector3d new_position = getNewAtomPosition(atom, prevPositions[i], timeInterval);
        prevPositions[i] = x_curr;
        applyBoundaryConditions(x_curr);
        atom->setLocalPos(new_position);
        cVector3d v = (new_position - prevPositions[i]) / timeInterval;
        atom->setVelocity(v);
      }
    }
    displayedPotentialEnergy.store(potentialEnergy);
  }

  
  cVector3d hapticForce(0, 0, 0);
  if (hasHapticDevice) {
    if (hapticMode == HapticMode::Force) {
      hapticForce = forceModeUpdate(current, position, timeInterval);
    } else if (hapticMode == HapticMode::Standby) {
      hapticForce = standbyModeUpdate(current, position, timeInterval);
    } else if (hapticMode == HapticMode::Position) {
      hapticForce = positionModeUpdate(current, position, timeInterval);
    }
  }
  for (int i = 0; i < spheres.size(); i++) {
    spheres[i]->updateVelVector();
  }

  return hapticForce;
}

void updateHaptics(void) {
  hapticsThreadStarted.store(true);

  // simulation in now running
  simulationRunning = true;
  simulationFinished = false;

  // open a connection to haptic device
  hapticDevice->open();

  // calibrate device (if necessary)
  hapticDevice->calibrate();
  // Track which atom is currently being moved
  int anchor_atom = 1;
  int anchor_atom_hold = 1;

  // main haptic simulation loop
  bool button3_changed = false;
  bool is_anchor = true;
  bool buttons[4];
  bool buttonReset[4];
  readButtons(buttons, buttonReset);
  initializeprevPositions();
  while (simulationRunning) {
    /////////////////////////////////////////////////////////////////////
    // SIMULATION TIME
    /////////////////////////////////////////////////////////////////////

    // signal frequency counter
    freqCounterHaptics.signal(1);
    /////////////////////////////////////////////////////////////////////////
    // READ HAPTIC DEVICE
    /////////////////////////////////////////////////////////////////////////
    // read position
    cVector3d position;
    hapticDevice->getPosition(position);

    // Scale position to use more of the screen
    // increase to use more of the screen
    position *= 2.0;
    hapticPosition = position;

    /////////////////////////////////////////////////////////////////////////
    // UPDATE SIMULATION
    /////////////////////////////////////////////////////////////////////////
    // Update current atom based on if the user pressed the far left button
    // The point of button2_changed is to make it so that it only switches one
    // atom if the button is touched Otherwise it flips out
    
    readButtons(buttons, buttonReset);

    // delay to slow down machine
    // std::cout << "Starting delay...\n";
    // std::this_thread::sleep_for(std::chrono::seconds(2));

    // time step the simulation runs at in seconds - shorter timesteps are more accurate, but result in slower frames
    // .001 is good value for uma simulations
    // .0001 is good value for all other
    const double DT = getSimulationTimeStep();
    cVector3d force = stepSimulation(position, DT, true);

    /////////////////////////////////////////////////////////////////////////
    // APPLY FORCES
    /////////////////////////////////////////////////////////////////////////

    hapticDevice->setForce(force);
  }
  // close  connection to haptic device
  hapticDevice->close();

  // exit haptics thread
  simulationFinished = true;

  // Close the calculator
  delete calculatorPtr;
  calculatorPtr = nullptr;
}

//------------------------------------------------------------------------------
// SLIDER CONTROL WINDOW
//------------------------------------------------------------------------------
struct SliderConfig {
  string name;
  double minValue;
  double maxValue;
  double defaultValue;
  string units;
  double displayScale;
  int displayDigits;
};

struct SliderUI {
  string id;
  string name;
  string units;
  double minValue;
  double maxValue;
  double value;
  double displayScale;
  int displayDigits;
  bool dragging;

  double normalizedValue() const {
    if (maxValue <= minValue) {
      return 0.0;
    }
    return (value - minValue) / (maxValue - minValue);
  }

  void setNormalizedValue(double normalizedValue) {
    double clampedValue = normalizedValue;
    if (clampedValue < 0.0) {
      clampedValue = 0.0;
    }
    if (clampedValue > 1.0) {
      clampedValue = 1.0;
    }
    value = minValue + clampedValue * (maxValue - minValue);
  }

  string displayText() const {
    return name + ": " + cStr(value * displayScale, displayDigits) + " " + units;
  }
};

const int SLIDER_WINDOW_WIDTH = 340;
const int SLIDER_WINDOW_HEIGHT = 170;
const int SLIDER_WIDTH = 240;
const int SLIDER_LEFT = 50;
const int SLIDER_TOP = 45;
const int SLIDER_ROW_SPACING = 52;

int sliderWindowWidth = SLIDER_WINDOW_WIDTH;
int sliderWindowHeight = SLIDER_WINDOW_HEIGHT;
cFontPtr sliderFont;
vector<SliderUI> sliders;
unordered_map<string, int> sliderIndexById;

// SLIDER UI STEP 1A: Add each new slider ID to this list to control display order.
vector<string> sliderOrder = {"time_step"};

// SLIDER UI STEP 1B: Add each new slider's configuration here.
// sliderConfigs[id] = {display name, min, max, default, units, display scale, display digits}
unordered_map<string, SliderConfig> sliderConfigs = {
  {"time_step", {"Time Step", 0.0001, 0.0020, 0.0010, "ms", 1000.0, 2}}
};

void generateSliderUI() {
  sliders.clear();
  sliderIndexById.clear();

  for (const string &id : sliderOrder) {
    const SliderConfig &config = sliderConfigs[id];

    SliderUI slider;
    slider.id = id;
    slider.name = config.name;
    slider.units = config.units;
    slider.minValue = config.minValue;
    slider.maxValue = config.maxValue;
    slider.value = config.defaultValue;
    slider.displayScale = config.displayScale;
    slider.displayDigits = config.displayDigits;
    slider.dragging = false;

    sliderIndexById[id] = sliders.size();
    sliders.push_back(slider);
  }
}

double getSliderValue(const string &id, double fallback) {
  auto it = sliderIndexById.find(id);
  if (it == sliderIndexById.end()) {
    return fallback;
  }
  return sliders[it->second].value;
}

// SLIDER UI STEP 2: Add a getter for each slider value
// The fallback should match that sliders default value in sliderConfigs.
double getSimulationTimeStep() {
  return getSliderValue("time_step", 0.0010);
}

// SLIDER UI STEP 3: Replace direct variable usage with the getter where needed.
// Example: use getSimulationTimeStep() instead of hardcoding the timestep.
void initializeSliderUI() {
  sliderFont = NEW_CFONT_CALIBRI_20();
  generateSliderUI();
  updateSliderWindowTitle();
}

void sliderWindowSizeCallback(GLFWwindow *a_window, int a_width, int a_height) {
  sliderWindowWidth = a_width;
  sliderWindowHeight = a_height;
}

void getSliderLayout(int sliderIndex, double &trackX, double &trackY) {
  trackX = SLIDER_LEFT;
  trackY = SLIDER_TOP + sliderIndex * SLIDER_ROW_SPACING;
}

double getSliderNormalizedValueFromMouseX(int sliderIndex, double mouseX) {
  double trackX;
  double trackY;
  getSliderLayout(sliderIndex, trackX, trackY);

  double normalizedValue = (mouseX - trackX) / SLIDER_WIDTH;
  if (normalizedValue < 0.0) {
    return 0.0;
  }
  if (normalizedValue > 1.0) {
    return 1.0;
  }
  return normalizedValue;
}

bool isMouseOverSlider(int sliderIndex, double mouseX, double mouseY) {
  double trackX;
  double trackY;
  getSliderLayout(sliderIndex, trackX, trackY);

  return (mouseX >= trackX - 12 && mouseX <= trackX + SLIDER_WIDTH + 12 &&
          mouseY >= trackY - 18 && mouseY <= trackY + 18);
}

void updateSliderWindowTitle() {
  if (sliderWindow == NULL) {
    return;
  }
  glfwSetWindowTitle(sliderWindow, "Controls");
}

void drawRect(double x, double y, double w, double h, float r, float g, float b) {
  glColor3f(r, g, b);
  glBegin(GL_QUADS);
  glVertex2d(x, y);
  glVertex2d(x + w, y);
  glVertex2d(x + w, y + h);
  glVertex2d(x, y + h);
  glEnd();
}

void drawSliderText(const string &text, double x, double y) {
  if (!sliderFont) {
    return;
  }

  cRenderOptions options;
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
  sliderFont->renderText(text, cColorf(0.05f, 0.05f, 0.05f), 1.0, 1.0, 1.0, options);
  glPopMatrix();
}

void renderSliderWindow() {
  if (sliderWindow == NULL) {
    return;
  }
  if (glfwWindowShouldClose(sliderWindow)) {
    glfwDestroyWindow(sliderWindow);
    sliderWindow = NULL;
    glfwMakeContextCurrent(window);
    return;
  }

  glfwMakeContextCurrent(sliderWindow);
  glfwGetWindowSize(sliderWindow, &sliderWindowWidth, &sliderWindowHeight);
  glViewport(0, 0, sliderWindowWidth, sliderWindowHeight);
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  glOrtho(0, sliderWindowWidth, sliderWindowHeight, 0, -1, 1);
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  glDisable(GL_DEPTH_TEST);
  glClearColor(0.94f, 0.94f, 0.94f, 1.0f);
  glClear(GL_COLOR_BUFFER_BIT);

  for (int i = 0; i < sliders.size(); i++) {
    const SliderUI &slider = sliders[i];
    double trackX;
    double trackY;
    getSliderLayout(i, trackX, trackY);
    const double handleX = trackX + slider.normalizedValue() * SLIDER_WIDTH;

    drawSliderText(slider.displayText(), trackX, trackY - 28);
    drawRect(trackX, trackY - 4, SLIDER_WIDTH, 8, 0.28f, 0.28f, 0.28f);
    drawRect(trackX, trackY - 4, handleX - trackX, 8, 0.05f, 0.35f, 0.90f);
    drawRect(handleX - 7, trackY - 15, 14, 30, 0.02f, 0.22f, 0.65f);
  }

  updateSliderWindowTitle();
  glfwSwapBuffers(sliderWindow);
  glfwMakeContextCurrent(window);
}

bool handleSliderMousePress(double mouseX, double mouseY) {
  for (int i = 0; i < sliders.size(); i++) {
    if (!isMouseOverSlider(i, mouseX, mouseY)) {
      continue;
    }

    sliders[i].dragging = true;
    sliders[i].setNormalizedValue(getSliderNormalizedValueFromMouseX(i, mouseX));
    updateSliderWindowTitle();
    return true;
  }
  return false;
}

bool handleSliderMouseMotion(double mouseX, double mouseY) {
  for (int i = 0; i < sliders.size(); i++) {
    if (!sliders[i].dragging) {
      continue;
    }

    sliders[i].setNormalizedValue(getSliderNormalizedValueFromMouseX(i, mouseX));
    updateSliderWindowTitle();
    return true;
  }
  return false;
}

bool handleSliderMouseRelease() {
  for (int i = 0; i < sliders.size(); i++) {
    if (!sliders[i].dragging) {
      continue;
    }

    sliders[i].dragging = false;
    return true;
  }
  return false;
}

void sliderWindowCursorPosCallback(GLFWwindow *a_window, double a_posX, double a_posY) {
  std::lock_guard<std::recursive_mutex> lock(sceneMutex);
  handleSliderMouseMotion(a_posX, a_posY);
}

void sliderWindowMouseButtonCallback(GLFWwindow *a_window, int a_button, int a_action, int a_mods) {
  if (a_button != GLFW_MOUSE_BUTTON_LEFT) {
    return;
  }

  std::lock_guard<std::recursive_mutex> lock(sceneMutex);
  double x;
  double y;
  glfwGetCursorPos(a_window, &x, &y);

  if (a_action == GLFW_PRESS) {
    handleSliderMousePress(x, y);
  } else if (a_action == GLFW_RELEASE) {
    handleSliderMouseRelease();
  }
}
