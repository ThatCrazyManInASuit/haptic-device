#include "atom.h"
#include "chai3d.h"
#include "globals.h"
#include "inputHandling.h"
#include "ipcServer.h"
#include "potentials.h"
#include "utility.h"
#include "boundaryConditions.h"
#include "slider.h"

#include <GLFW/glfw3.h>
#include <math.h>
#include <unistd.h>

#include <array>
#include <chrono>
#include <cstdlib>
#include <ctime>
#include <fstream>
#include <iostream>
#include <limits>
#include <map>
#include <set>
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

// Converts ASE time units to femtoseconds (fs): 1 ASE time unit is 10.18 fs
const double ASE_UNITS_TO_FS = 10.18; 

bool showDebug = false; // Toggles the extra debug overlay information when true.

std::vector<cLabel *> debugAtomLabels; // Stores the labels that annotate atoms with their indices.

// Stores the initial atom positions so the structure can be reset.
std::vector<cVector3d> initialPositions;

// Radius of an atom in world units. 
// Based on the covalent radius of hydrogen, .37 Å
const double SPHERE_RADIUS = 0.37 * DIST_SCALE;

// Scales the effective mass of each atom in the toy dynamics model.
const double SPHERE_MASS_SCALE_FACTOR = 0.02;

// Haptic spring-damper constants used to reduce unwanted oscillations.
const double K_HAPTIC_SPRING = 100.0;
const double K_HAPTIC_DAMPER = 5.0;    // Damping for force modendLines

//------------------------------------------------------------------------------
// DECLARED VARIABLES
//------------------------------------------------------------------------------

Calculator *calculatorPtr; // The calculator used to calculate atom forces

chai3d::cCamera *camera; // a camera to render the world in the window display
chai3d::cGenericHapticDevicePtr hapticDevice; // a pointer to the current haptic device

std::atomic<HapticMode> hapticMode(HapticMode::Position);

// simulation time step in seconds; overridable at launch via
// HAPTIC_DEVICE_TIME_STEP and changeable live via the IPC "set timestep" command
std::atomic<double> simulationTimeStep(0.001);

// standby/return-to-center haptic tuning parameters used by standbyModeUpdate,
// changeable live via the IPC "set settling_err/k_return/k_dampen/return_delay"
// commands (see setLiveSettlingError/setLiveKReturn/setLiveKDampen/setLiveReturnDelay)
std::atomic<double> settlingError(0.05);
std::atomic<double> kReturn(25.0);
std::atomic<double> kDampen(0.0); // 0 = no damping, matching original return behavior
std::atomic<double> returnDelaySeconds(2.5);

// overall scale applied to the force sent to the physical haptic device,
// overridable at launch via HAPTIC_DEVICE_FORCE_SCALE and changeable live via
// the IPC "set force_scale" command / launcher UI. Lets owners of older/more
// worn devices turn down feedback strength to reduce wear, without touchingctedPoint
// the underlying simulation's spring/damping constants.
std::atomic<double> hapticForceScale(1.0);

// sphere objects
std::vector<Atom *> spheres;

// lines drawn between bonded atom pairs, keyed by sorted (sphere index) pairs.
// Lines are created lazily and hidden (not removed) when a pair un-bonds so
// they can be cheaply re-shown if the pair drifts back within range.
std::map<pair<int, int>, cShapeLine *> bondLines;

std::vector<cLabel *> debugLabels; // Stores the labels that show debug values in the scene.

chai3d::cLabel *hapticPositionLabel;
chai3d::cLabel *labelRates; // a label to display the rate [Hz] at which the simulation is running
chai3d::cLabel *LJ_num; // a label to show the potential energy
chai3d::cLabel *num_anchored; // label showing the # anchored
chai3d::cLabel *isFrozen; // a label to show whether or not the atoms are frozen
chai3d::cLabel *camera_pos; // a label to display the camera position
chai3d::cLabel *potentialLabel; // a label to identify the potential energy surface
chai3d::cLabel *temperatureLabel;

// labels for the scope
chai3d::cLabel *scope_upper;
chai3d::cLabel *scope_lower;

// a flag that indicates if the haptic simulation is currently running
std::atomic<bool> simulationRunning{false};

bool simulationFinished; // a flag that indicates if the haptic simulation has terminated

// a frequency counter to measure the simulation graphic rate
chai3d::cFrequencyCounter freqCounterGraphics; 

chai3d::cFrequencyCounter freqCounterHaptics; // a frequency counter to measure the simulation haptic rate


GLFWwindow *sliderWindow; // a handle to slider control window

// current framebuffer (render) size in pixels.
// NOTE: on HiDPI / Retina displays this is LARGER than the window size in points.
int width;
int height;

double CAMERA_RADIUS = .35; 

chai3d::cScope *scope; // a scope to monitor the potential energy

double global_minimum; // global minimum for the given cluster size

std::atomic<bool> freezeAtoms(false); // determine if atoms should be frozen

std::atomic<bool> renderAtoms(true); // determine if atom spheres should be drawn
std::atomic<bool> renderForceVectors(true); // determine if force vector lines should be drawn
std::atomic<bool> renderBonds(true); // determine if bond lines should be drawn
double centerCoords[3] = {50.0, 50.0, 50.0}; // save coordinates of central atom

std::atomic<int> screenshotCounter(-2); // keep track of how long screenshot label has been displayed
std::atomic<int> writeConCounter(-2);  // keep track of how long write to con label has been displayed

LocalPotential energySurface = LENNARD_JONES; // default potential is Lennard Jones
bool global_min_known = true; // check if able to read in the global min
chai3d::cPanel *helpPanel; // panel that displays hotkeys
chai3d::cLabel *helpHeader; // help panel header

std::atomic<double> displayedPotentialEnergy(0.0);

std::atomic<double> displayedTemperature(0.0);
double lastPotentialEnergy = 0.0;
std::atomic<int> displayedAnchoredCount(0);
std::recursive_mutex sceneMutex;
std::atomic<bool> hapticsThreadStarted(false);
int currentIndex = 0;
std::vector<cLabel *> hotkeyKeys; // vector holding hotkey key labels
std::vector<cLabel *> hotkeyFunctions; // vector holding function key labels (must be separate for formatting)

// screenshot notification label
chai3d::cLabel *screenshotLabel;

// write to con notification label
chai3d::cLabel *writeConLabel;

chai3d::cVector3d hapticPosition;
static std::vector<cVector3d> prevPositions;

// Prints the startup banner and key instructions for the user.
void printIntro() {
  cout << endl;
  cout << "-----------------------------------" << endl;
  cout << "CHAI3D" << endl;
  cout << "Press CTRL for help" << endl;
  cout << "-----------------------------------" << endl
       << endl
       << endl;
}

/**
 * @brief Callback triggered when GLFW reports an error.
 */
void errorCallback(int a_error, const char *a_description) {
  cout << "Error: " << a_description << endl;
}

// Configures the OpenGL context version used by the scene.
void setOpenGLVersion(int majorVer, int minorVer) {
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, majorVer);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, minorVer);
}

/**
 * @brief Configures the GLFW library. Sets GLFW error callback, the OpenGL version to 2.1, and
 * the stereo mode.
 * @param STEREO_MODE Whether stereo mode should be active.
 */
void configureGLFW(const chai3d::cStereoMode STEREO_MODE) {
  if (!glfwInit()) {
    throw std::runtime_error("Configuration failed! GLFW not initialized!");
  }
  glfwSetErrorCallback(errorCallback); // set error callback
  setOpenGLVersion(2, 1);
  // set active stereo mode
  STEREO_MODE == C_STEREO_ACTIVE 
      ? glfwWindowHint(GLFW_STEREO, GL_TRUE) 
      : glfwWindowHint(GLFW_STEREO, GL_FALSE);
}

GLFWwindow* initializeMainWindow() {
  // compute desired size of window
  const GLFWvidmode *VIDEO_MODE = glfwGetVideoMode(glfwGetPrimaryMonitor());
  
  // How wide the main window should be relative to the monitor size.
  const double MAIN_WINDOW_WIDTH_SCALE = 0.8;
  // How tall the main window should be relative to the monitor size.
  const double MAIN_WINDOW_HEIGHT_SCALE = 0.5; 

  int windowWidth = MAIN_WINDOW_WIDTH_SCALE * VIDEO_MODE->width;
  int windowHeight = MAIN_WINDOW_HEIGHT_SCALE * VIDEO_MODE->height;

  // Handle to window display context
  GLFWwindow *window = glfwCreateWindow(windowWidth, windowHeight, "haptic-device", nullptr, 
      nullptr); 

  if (!window) {
    glfwTerminate();
    throw std::runtime_error("Failed to create window!");
  }

  glfwGetFramebufferSize(window, &width, &height); // framebuffer size in pixels (HiDPI-aware)     

  // Horizontally, where the main window should be relative to the monitor size.
  const double MAIN_WINDOW_INIT_POS_X = 0.5;
  // Vertically, where the main window should be relative to the monitor size.
  const double MAIN_WINDOW_INIT_POS_Y = 0.5;

  // set position of window
  glfwSetWindowPos(
    window,
    MAIN_WINDOW_INIT_POS_X * (VIDEO_MODE->width - windowWidth), 
    MAIN_WINDOW_INIT_POS_Y * (VIDEO_MODE->height - windowHeight)
  ); 
  glfwSetKeyCallback(window, keyCallback); // set key callback
  glfwSetCursorPosCallback(window, mouseMotionCallback); // set mouse position callback
  glfwSetMouseButtonCallback(window, mouseButtonCallback); // set mouse button callback
  glfwSetFramebufferSizeCallback(window, framebufferSizeCallback); // track render size on resize
  glfwMakeContextCurrent(window); // set current display context

  // The swap interval for the display context (V-Sync)
  glfwSwapInterval(SWAP_INTERVAL); // sets the swap interval for the current display context
  return window;
}

// Initializes the GLEW OpenGL extension loader.
void initializeGLEW() {
  #ifdef GLEW_VERSION
  if (glewInit() != GLEW_OK) {
    glfwTerminate();
    throw std::runtime_error("Failed to initialize GLEW library!");
  }
  #endif
}

// Creates the CHAI3D world object that hosts the simulation.
cWorld* initializeWorld() {
  cWorld *world = new cWorld();
  world->m_backgroundColor.setWhite();
  world->setShadowIntensity(0.3); // set shadow factor
  return world;
}

// Stops the simulation and frees the created resources.
void close() { // stop the simulation
  static bool closed = false;
  if (!closed) {
    closed = true;
    stopIpcServer();
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
  }
}

// Creates and configures the camera used to render the simulation.
void initializeCamera(cWorld* world, const chai3d::cStereoMode STEREO_MODE) {
  camera = new chai3d::cCamera(world);
  world->addChild(camera);

  // creates the radius, origin reference, along with the zenith and azimuth direction vectors
  cVector3d origin(0.0, 0.0, 0.0);
  cVector3d zenith(0.0, 0.0, 1.0);
  cVector3d azimuth(1.0, 0.0, 0.0);

  // sets the camera's references of the origin, zenith, and azimuth
  camera->setSphericalReferences(origin, zenith, azimuth);

  // sets the camera's position, located at 0 radians (vertically and horizontally)
  camera->setSphericalRad(CAMERA_RADIUS, 0, 0);
  // set the near and far clipping planes of the camera anything in front or behind these clipping
  // planes will not be rendered
  camera->setClippingPlanes(0.01, 10.0);

  camera->setStereoMode(STEREO_MODE);  // set stereo mode

  // set stereo eye separation and focal length (applies only if stereo is enabled)
  camera->setStereoEyeSeparation(0.03);
  camera->setStereoFocalLength(1.8);
  camera->setMirrorVertical(false); // set vertical mirrored display mode

  
  cBackground *background; // a colored background
  background = new cBackground(); // create a background
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

// Creates the light source that illuminates the atoms.
void initializeLight(cWorld* world) {
  cSpotLight *light = new cSpotLight(world); // create a light source
  light->setEnabled(true); // enable light source
  light->setLocalPos(0.0, 0.3, 0.4); // position the light source
  light->setDir(0.0, -0.25, -0.4); // define the direction of the light beam
  light->setShadowMapEnabled(false); // enable this light source to generate shadows
  light->m_shadowMap->setQualityHigh(); // set the resolution of the shadow map
  light->setCutOffAngleDeg(30); // set light cone half angle
  world->addChild(light); // attach light to camera
}

// Connects to, or prepares, the available haptic device.
void initializeHapticDevice() {
  cHapticDeviceHandler *handler;
  handler = new cHapticDeviceHandler(); // create a haptic device handler
  // get access to the first available haptic device
  double hapticDeviceMaxStiffness;   // highest stiffness the current haptic device can render
  if (handler->getNumDevices() > 0) {
    handler->getDevice(hapticDevice, 0);
  }
  if (hapticDevice) {
    // retrieve the highest stiffness this device can render
    hapticDeviceMaxStiffness = hapticDevice->getSpecifications().m_maxLinearStiffness;

    // if the haptic devices carries a gripper, enable it to behave like a user switch
    hapticDevice->setEnableGripperUserSwitch(true);
  } else {
    const double HAPTIC_STIFFNESS = 1000.0;
    hapticDeviceMaxStiffness = HAPTIC_STIFFNESS;
    cout << "No haptic device detected. Running in keyboard/mouse-only mode." << endl;
  }
}

// Adds labels that annotate each atom with its index.
void initializeAtomLabels() {
  cFontPtr atomLabelFont = NEW_CFONT_CALIBRI_20();
  for (int i = 0; i < spheres.size(); i++) {
    cLabel *label = new cLabel(atomLabelFont);
    label->m_fontColor.setBlack();
    label->setText(to_string(i));
    label->setShowEnabled(false);
    camera->m_frontLayer->addChild(label);
    debugAtomLabels.push_back(label);
  }
}

// Adds a debug label to the front layer of the scene.
void addDebugLabel(std::string text);

static void addScaledVertex(vector<cVector3d> &positions, double x, double y, double z, double radius) {
  positions.push_back(scaledToRadius(cVector3d(x, y, z), radius));
}

// Generates positions for a regular polyhedron shell with k vertices.
vector<cVector3d> polyhedronCords(int k, double radius) {
  vector<cVector3d> positions;
  positions.reserve(k);
  const double phi = (1.0 + sqrt(5.0)) / 2.0;
  const double invPhi = 1.0 / phi;

  if (k == 4) {
    addScaledVertex(positions,  1.0,  1.0,  1.0, radius);
    addScaledVertex(positions,  1.0, -1.0, -1.0, radius);
    addScaledVertex(positions, -1.0,  1.0, -1.0, radius);
    addScaledVertex(positions, -1.0, -1.0,  1.0, radius);
  } else if (k == 6) {
    addScaledVertex(positions,  1.0,  0.0,  0.0, radius);
    addScaledVertex(positions, -1.0,  0.0,  0.0, radius);
    addScaledVertex(positions,  0.0,  1.0,  0.0, radius);
    addScaledVertex(positions,  0.0, -1.0,  0.0, radius);
    addScaledVertex(positions,  0.0,  0.0,  1.0, radius);
    addScaledVertex(positions,  0.0,  0.0, -1.0, radius);
  } else if (k == 8) {
    //test, might have to hardcode postiitons if this doesnt work
    for (int x = -1; x <= 1; x += 2) {
      for (int y = -1; y <= 1; y += 2) {
        for (int z = -1; z <= 1; z += 2) {
          addScaledVertex(positions, x, y, z, radius);
        }
      }
    }
  } else if (k == 12) {
    for (int y = -1; y <= 1; y += 2) {
      for (int z = -1; z <= 1; z += 2) {
        addScaledVertex(positions, 0.0, y, z * phi, radius);
      }
    }
    for (int x = -1; x <= 1; x += 2) {
      for (int y = -1; y <= 1; y += 2) {
        addScaledVertex(positions, x, y * phi, 0.0, radius);
      }
    }
    for (int x = -1; x <= 1; x += 2) {
      for (int z = -1; z <= 1; z += 2) {
        addScaledVertex(positions, x * phi, 0.0, z, radius);
      }
    }
  } else if (k == 20) {
    for (int x = -1; x <= 1; x += 2) {
      for (int y = -1; y <= 1; y += 2) {
        for (int z = -1; z <= 1; z += 2) {
          addScaledVertex(positions, x, y, z, radius);
        }
      }
    }
    for (int y = -1; y <= 1; y += 2) {
      for (int z = -1; z <= 1; z += 2) {
        addScaledVertex(positions, 0.0, y * invPhi, z * phi, radius);
      }
    }
    for (int x = -1; x <= 1; x += 2) {
      for (int y = -1; y <= 1; y += 2) {
        addScaledVertex(positions, x * invPhi, y * phi, 0.0, radius);
      }
    }
    for (int x = -1; x <= 1; x += 2) {
      for (int z = -1; z <= 1; z += 2) {
        addScaledVertex(positions, x * phi, 0.0, z * invPhi, radius);
      }
    }
  }

  return positions;
}

// Generates positions for a Fibonacci-sphere shell with uniform coverage.
vector<cVector3d> fibonacciCords(int k, double radius) {
  vector<cVector3d> positions;
  positions.reserve(k);

  if (k <= 0) {
    return positions;
  }
  if (k == 1) {
    positions.push_back(cVector3d(0.0, 0.0, radius));
    return positions;
  }

  const double goldenAngle = M_PI * (3.0 - sqrt(5.0));

  for (int i = 0; i<k; i++) {
    double y = 1.0 - (2.0 * i) / (k - 1);
    double r = sqrt(1.0 - y * y);
    double theta = goldenAngle * i;

    positions.push_back(cVector3d(
      radius*cos(theta)*r,
      radius*y,
      radius*sin(theta)*r
    ));
  }

  return positions;
}

// Generates positions for a Thomson shell using iterative repulsion.
vector<cVector3d> thomsonCords(int k, double radius) {
  vector<cVector3d> positions = fibonacciCords(k, radius);
  if (k <= 1) {
    return positions;
  }

  const int iterations = 600;
  const double baseStep = radius * 0.04;

  for (int iter = 0; iter < iterations; iter++) {
    vector<cVector3d> forces(k, cVector3d(0.0, 0.0, 0.0));

    for (int i = 0; i < k; i++) {
      for (int j = i + 1; j < k; j++) {
        cVector3d diff = positions[i] - positions[j];
        double dist = diff.length();
        if (dist <= 1e-9) {
          continue;
        }

        cVector3d force = diff * (1.0 / (dist * dist * dist));
        forces[i] += force;
        forces[j] -= force;
      }
    }

    double step = baseStep * (1.0 - (0.75 * iter / iterations));
    for (int i = 0; i < k; i++) {
      cVector3d normal = scaledToRadius(positions[i], 1.0);
      double radialForce = forces[i].x() * normal.x()
                         + forces[i].y() * normal.y()
                         + forces[i].z() * normal.z();
      cVector3d tangentForce = forces[i] - normal * radialForce;
      positions[i] = scaledToRadius(positions[i] + tangentForce * step, radius);
    }
  }

  return positions;
}

// Generates shell positions for a cluster of k atoms around the current atom.
vector<cVector3d> generateShellPositions(int k, double radiusAngstroms) {
  

  if (k <= 0) {
    return vector<cVector3d>();
  }
  const double radius = radiusAngstroms * DIST_SCALE;
  if ((k == 4) || (k == 6) || (k == 8) || (k == 12) || (k == 20)) {
    return polyhedronCords(k, radius);
  }
  if (k <= 100) {
    return thomsonCords(k, radius);
  }
  return fibonacciCords(k, radius);
}

// Creates and configures an atom sphere with the requested material and size.
Atom* initializeAtom(cWorld* world, cTexture2dPtr texture, int atomicNumber, double radius = SPHERE_RADIUS) {
  Atom *new_atom = new Atom(radius, atomicNumber); // create a atom and define its radius
  spheres.push_back(new_atom); // store pointer to atom
  world->addChild(new_atom); // add atom to world
  world->addChild(new_atom->getVelVector()); // add line to world

  // set graphic properties of sphere
  new_atom->setTexture(texture);
  new_atom->m_texture->setSphericalMappingEnabled(true);
  new_atom->setUseTexture(true);
  return new_atom;
}

void placeAtomsAse(chai3d::cWorld* world, std::array<double, 9>& aseCell, std::array<int, 3>& asePbc, cTexture2dPtr texture, int argc, char *argv[]) {
  AseStructureData structure;
  // Optional repeat factors: argv[6]=x, argv[7]=y, argv[8]=z. Each defaults to
  // 1 if not given, and values < 1 are ignored (they would zero out the cell).
  std::array<int, 3> repeat = {1, 1, 1};
  for (int i = 0; i < 3; i++) {
    if (argc > 6 + i) {
      int value = atoi(argv[6 + i]);
      if (value > 0) {
        repeat[i] = value;
      }
    }
  }
  try {
    structure = loadAseStructure(argv[2], repeat);
  } catch (const std::exception &ex) {
    close();
    throw std::runtime_error(ex.what());
  }
  const std::vector<std::array<double, 3>> &positions = structure.positions;
  const std::vector<int> &startingAtomicNrs = structure.atomicNumbers;
  const std::vector<double> &startingRadii = structure.radii;
  // comment out below for no pbc
  aseCell = structure.cell;
  asePbc = structure.pbc;
  const int nAtoms = static_cast<int>(positions.size());
  chai3d::cVector3d centerPos;
  for (int i = 0; i < nAtoms; i++) {
    Atom *newAtom = initializeAtom(world, texture, startingAtomicNrs[i], startingRadii[i] * DIST_SCALE); // Create atom pointer`
    // Set the positions of all atoms
    if (i == 0) {
      // make very first atom the current atom
      newAtom->setCurrent(true);
      // get coordinates from pPositionTriplet
      centerPos = chai3d::cVector3d(
        positions[0][static_cast<size_t>(0)],
        positions[0][static_cast<size_t>(1)],
        positions[0][static_cast<size_t>(2)]
      );
      newAtom->setLocalPos(0.0, 0.0, 0.0); // set first atom at center of view
    } else {
        chai3d::cVector3d atomPos(positions[i][0], positions[i][1], positions[i][2]);
        // scale coordinates and insert
        if (hapticMode == HapticMode::Standby) {
          chai3d::cVector3d STANDBY_OFFSET(cVector3d(0.1, 0.1, 0.1));
          atomPos += STANDBY_OFFSET;
        }
        newAtom->setLocalPos(DIST_SCALE * (atomPos - centerPos));
    }
  }
}

// Places atoms into the scene from the supplied ASE structure or generated shell.
void placeAtoms(chai3d::cWorld* world, std::array<double, 9>& aseCell, std::array<int, 3>& asePbc, int argc, char *argv[]) {
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
    // k is the number of atoms surrounding the current center atom.
    int k = argc > 2 ? atoi(argv[2]) : 5;
    if (k < 0) {
      k = 0;
    }
    int numSpheres = k + 1;
    // argv[4]/argv[5] are always the ASE spec and PBC mode (see main()), never
    // a radius, so there is no CLI slot to override this default.
    const double shellRadiusAngstroms = 5.0;
    vector<cVector3d> positions = generateShellPositions(k, shellRadiusAngstroms);
    for (int i = 0; i < numSpheres; i++) {
      // initialize atom with texture and atomic number of 1 (hydrogen)
      Atom *new_atom = initializeAtom(world, texture, 1, SPHERE_RADIUS); 
      if (i == 0) {
        new_atom->setCurrent(true); // set the first sphere to the current
      } else {
        new_atom->setLocalPos(positions[i - 1]);
      }
    }
  } else // read in specified file
    placeAtomsAse(world, aseCell, asePbc, texture, argc, argv);

  // Done reading any sort of info.
  for (int i = 0; i < spheres.size(); i++) {
    spheres[i]->setVelocity(0);
  }
}

// Places a new atom in a non-overlapping random position.
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
}

// Configures the selected calculator based on CLI arguments and structure data.
void initializeCalculator(int argc, char *argv[], std::array<double, 9> aseCell,
    std::array<int, 3> asePbc) {
    if (argc < 4) {
      energySurface = LENNARD_JONES;
      calculatorPtr = new ljCalculator();
      return;
    }
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

// Updates the potential label to match the current energy surface.
void initializePotentialLabel() {
  // set energy surface label
  potentialLabel->setLocalPos(0, 0);
  string potentialName;
  switch (energySurface) {
    case LENNARD_JONES:
      potentialName = "Lennard Jones Potential";
      break;
    case MORSE:
      potentialName = "Morse Potential";
      break;
    case ASE:
      potentialName = "ASE Potential";
      break;
    default:
      throw std::runtime_error("Unknown energy surface encountered!");
  }
  potentialLabel->setText("Potential energy surface: " + potentialName);
}

// Creates the labels that display simulation status and values.
void initializeLabels() {
  addLabel(hapticPositionLabel); // label to read haptic device
  addLabel(labelRates); // create a label to display the haptic and graphic rate of the simulation
  addLabel(LJ_num); // potential energy label
  addLabel(num_anchored); // number anchored label
  
  cLabel *total_energy; // a label to display the total energy of the system
  addLabel(total_energy); // total energy label
  addLabel(isFrozen); // frozen state label
  addLabel(camera_pos); // camera position label
  addLabel(potentialLabel); // energy surface label
  addLabel(temperatureLabel);
  addDebugLabel("Force magnitude: ");
  addDebugLabel("Atom pos: ");
  addDebugLabel("Nearest neighbor: ");
  addDebugLabel("Max force: ");
  
  addLabel(scope_upper); // Add labels to the graph
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

  screenshotLabel->setText("Screenshot taken");
  writeConLabel->setText("Con file written");

  initializePotentialLabel();

  temperatureLabel->setLocalPos(0, 90, 0);
  temperatureLabel->setText("Temperature: 0.00000 kT");

  camera_pos->setLocalPos(0, 30, 0);
  updateCameraLabel(camera_pos, camera);
}

// Creates the hotkey help labels shown in the UI panel.
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
  addHotkeyLabel("1", "toggle atom rendering");
  addHotkeyLabel("2", "toggle force vector rendering");
  addHotkeyLabel("3", "toggle bond rendering");
  addHotkeyLabel("I, K", "move current atom up/down");
  addHotkeyLabel("J, L", "move current atom left/right");
  addHotkeyLabel("O, P", "move current atom forward/back");
  addHotkeyLabel("d", "toggle debug info");
  addHotkeyLabel("t", "reset atom structure");
  addHotkeyLabel("CTRL", "toggle help panel");
}

// Initializes the energy-plot scope used to visualize potential energy.
void initializePotentialEnergyPlot() {
  // create a scope to plot potential energy
  scope = new cScope();
  scope->setLocalPos(0, 60);
  camera->m_frontLayer->addChild(scope);
  scope->setSignalEnabled(true, true, false, false);
  scope->setTransparencyLevel(.7);
  scope->setShowEnabled(false);
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

// Builds the help panel overlay that lists the hotkeys.
void initializeHelpPanel() {
  cColorf panelColor = cColorf();
  panelColor.setBlueCadet();

  helpPanel = new cPanel();
  helpPanel->setColor(panelColor);
  helpPanel->setSize(520, 600);
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

vector<int> getHapticInfluencedAtomIndices() {
  vector<int> influenced;
  for (int i = 0; i < spheres.size(); i++) {
    if (spheres[i]->isSelected()) {
      influenced.push_back(i);
    }
  }
  if (influenced.empty() && currentIndex >= 0 && currentIndex < spheres.size()) {
    influenced.push_back(currentIndex);
  }
  return influenced;
}

cVector3d getNewAtomPosition(Atom *atom, cVector3d &prev_position, const double dT) {
  cVector3d x0 = atom->getLocalPos();
  cVector3d a1 = atom->getForce() / atom->getMass() * DIST_SCALE;
  cVector3d a0 = atom->getPrevForce() / atom->getMass() * DIST_SCALE;

  atom->setVelocity(atom->getVelocity() + .5 * (a0 + a1) * dT);

  cVector3d v0 = atom->getVelocity();


  // force is in eV/Å and getMass() must be amu (see note below). ASE integrates
  // in Å, giving an Å displacement of (F/m)*dt². We render in world units where
  // 1 world unit = 1/DIST_SCALE Å, so scale that Å acceleration by DIST_SCALE.
  return x0 + v0 * dT + .5 * a1 * dT * dT;
}

double getDynamicBoundaryLimit() {
  // Boundary limits used to keep atoms inside the visible simulation volume.
  const double BOUNDARY_LIMIT = .01; 

  if (!camera || width <= 0 || height <= 0) {
    return BOUNDARY_LIMIT;
  }

  const double aspect = static_cast<double>(width) / static_cast<double>(height);
  const double zoomDistance = camera->getSphericalRadius();
  const double safeDistance = (zoomDistance > 1e-6) ? zoomDistance : 0.1;

  const double CAMERA_BOUNDARY_SCALE = 0.35;
  const double halfHeight = safeDistance * tan(camera->getFieldViewAngleRad() * 0.5) * CAMERA_BOUNDARY_SCALE;

  const double halfWidth = halfHeight * aspect;
  return (halfWidth > halfHeight) ? halfWidth : halfHeight;
}

void getCameraAlignedBoundaryPlanes(cVector3d &northPlanePos,
                                   cVector3d &northPlaneNorm,
                                   cVector3d &southPlanePos,
                                   cVector3d &southPlaneNorm,
                                   cVector3d &eastPlanePos,
                                   cVector3d &eastPlaneNorm,
                                   cVector3d &westPlanePos,
                                   cVector3d &westPlaneNorm,
                                   cVector3d &forwardPlanePos,
                                   cVector3d &forwardPlaneNorm,
                                   cVector3d &backPlanePos,
                                   cVector3d &backPlaneNorm,
                                   double &boundaryLimit) {
  boundaryLimit = getDynamicBoundaryLimit();

  const cVector3d focusPoint(0.0, 0.0, 0.0);
  if (!camera) {
    northPlanePos = cVector3d(0, boundaryLimit, 0);
    northPlaneNorm = cVector3d(0, 1, 0);
    southPlanePos = cVector3d(0, -boundaryLimit, 0);
    southPlaneNorm = cVector3d(0, -1, 0);
    eastPlanePos = cVector3d(boundaryLimit, 0, 0);
    eastPlaneNorm = cVector3d(1, 0, 0);
    westPlanePos = cVector3d(-boundaryLimit, 0, 0);
    westPlaneNorm = cVector3d(-1, 0, 0);
    forwardPlanePos = cVector3d(0, 0, boundaryLimit);
    forwardPlaneNorm = cVector3d(0, 0, 1);
    backPlanePos = cVector3d(0, 0, -boundaryLimit);
    backPlaneNorm = cVector3d(0, 0, -1);
  } else {
    const cVector3d camRight = camera->getRightVector();
    const cVector3d camUp = camera->getUpVector();
    const cVector3d camLook = camera->getLookVector();

    northPlanePos = focusPoint + camUp * boundaryLimit;
    northPlaneNorm = camUp;
    southPlanePos = focusPoint - camUp * boundaryLimit;
    southPlaneNorm = -camUp;
    eastPlanePos = focusPoint + camRight * boundaryLimit;
    eastPlaneNorm = camRight;
    westPlanePos = focusPoint - camRight * boundaryLimit;
    westPlaneNorm = -camRight;
    forwardPlanePos = focusPoint + camLook * boundaryLimit;
    forwardPlaneNorm = camLook;
    backPlanePos = focusPoint - camLook * boundaryLimit;
    backPlaneNorm = -camLook;
  }

  
}

void applyBoundaryConditions(cVector3d &oldPosition, cVector3d &newPosition) {
  cVector3d northPlanePos;
  cVector3d northPlaneNorm;
  cVector3d southPlanePos;
  cVector3d southPlaneNorm;
  cVector3d eastPlanePos;
  cVector3d eastPlaneNorm;
  cVector3d westPlanePos;
  cVector3d westPlaneNorm;
  cVector3d forwardPlanePos;
  cVector3d forwardPlaneNorm;
  cVector3d backPlanePos;
  cVector3d backPlaneNorm;
  double boundaryLimit = 0.0;

  // getCameraAlignedBoundaryPlanes(
  //     northPlanePos, northPlaneNorm,
  //     southPlanePos, southPlaneNorm,
  //     eastPlanePos, eastPlaneNorm,
  //     westPlanePos, westPlaneNorm,
  //     forwardPlanePos, forwardPlaneNorm,
  //     backPlanePos, backPlaneNorm,
  //     boundaryLimit);

  applySeanBoundaryConditions(
      oldPosition, newPosition, newPosition,
      northPlanePos, northPlaneNorm,
      southPlanePos, southPlaneNorm,
      eastPlanePos, eastPlaneNorm,
      westPlanePos, westPlaneNorm,
      forwardPlanePos, forwardPlaneNorm,
      backPlanePos, backPlaneNorm,
      boundaryLimit);
}



vector<int> activeHapticSelection;
map<int, cVector3d> activeHapticSelectionOffsets;

bool prevHapticInitialized;
map<int, cVector3d> hapticInfluenceOffsets;
vector<int> activeHapticInfluence;

void ensureHapticInfluenceOffsets(const vector<int> &indices,
                                  const cVector3d &position) {
  if (indices != activeHapticInfluence) {
    activeHapticInfluence = indices;
    hapticInfluenceOffsets.clear();
    for (int index : indices) {
      if (spheres[index]->isSelected()) {
        hapticInfluenceOffsets[index] = spheres[index]->getLocalPos() - position;
      } else {
        hapticInfluenceOffsets[index] = cVector3d(0, 0, 0);
      }
    }
  }
}


cVector3d getAverageAtomGroupForce(const vector<int> &indices) {
  cVector3d force(0, 0, 0);
  if (indices.empty()) {
    return force;
  }
  for (int index : indices) {
    force += spheres[index]->getForce();
  }
  return force / static_cast<double>(indices.size());
}

cVector3d addHapticForceToAtoms(const vector<int> &indices,
                                const cVector3d &position,
                                const double timeInterval) {
  if (indices.empty()) {
    return cVector3d(0, 0, 0);
  }
  ensureHapticInfluenceOffsets(indices, position);
  cVector3d averageForceBeforeHaptic = getAverageAtomGroupForce(indices);
  for (int index : indices) {
    Atom *atom = spheres[index];
    if (!atom->isAnchor()) {
      cVector3d targetPosition = position + hapticInfluenceOffsets[index];
      cVector3d currentPosition = atom->getLocalPos();
      cVector3d previousPosition = prevPositions[index];
      cVector3d velocity = (currentPosition - previousPosition) / timeInterval;
      cVector3d externalForce =
          (targetPosition - currentPosition) * K_HAPTIC_SPRING -
          velocity * K_HAPTIC_DAMPER;

      // Maximum force the haptic device can impart on the current atom in eV/Å
      const double MAX_HAPTIC_ATOM_FORCE = 1.0; 
      atom->setForce(atom->getForce() +
                     clampVectorMagnitude(externalForce, MAX_HAPTIC_ATOM_FORCE));
    }
  }

  return averageForceBeforeHaptic;
}

// Advances the atom simulation by one timestep and returns the haptic force.
cVector3d stepSimulation(const cVector3d &requestedPosition, const double timeInterval,
                        const bool hasHapticDevice) {
  if (prevPositions.size() != spheres.size()) {
    prevPositions.resize(spheres.size());
  }
  std::lock_guard<std::recursive_mutex> lock(sceneMutex);
  if (spheres.empty()) {
    return cVector3d(0.0, 0.0, 0.0);
  }

  Atom *current = spheres[currentIndex];
  cVector3d position = hasHapticDevice ? requestedPosition : current->getLocalPos();
  vector<int> hapticInfluencedIndices =
      hasHapticDevice ? getHapticInfluencedAtomIndices() : vector<int>();
  bool useHapticInfluence = !hapticInfluencedIndices.empty();
  if (!useHapticInfluence && !activeHapticSelection.empty()) {
    activeHapticSelection.clear();
    activeHapticSelectionOffsets.clear();
    prevHapticInitialized = false;
  }

  cVector3d currentPosition(0,0,0);
  cVector3d hapticForce(0, 0, 0);
  
  if (!freezeAtoms.load()) {
    if (!calculatorPtr) {
      cerr << "Error: calculatorPtr is null in stepSimulation()" << endl;
      return cVector3d(0.0, 0.0, 0.0);
    }
    const double currentTemp = getSliderVal("Temperature", 1.00);
    // calculatorPtr->setTemperature(currentTemp);
    vector<vector<double>> forcesVec = calculatorPtr->getFandU(spheres);
    double potentialEnergy = forcesVec[spheres.size()][0];
    if (std::isfinite(potentialEnergy)) {
      lastPotentialEnergy = potentialEnergy;
    }

    for (int i = 0; i < spheres.size(); i++) {
      Atom *atom = spheres[i];
      cVector3d force(forcesVec[i][0], forcesVec[i][1], forcesVec[i][2]);
      if (!isFiniteVector(force)) {
        force.zero();
      }
      atom->setForce(force);
    }
    if (hasHapticDevice && useHapticInfluence) {
      hapticForce = addHapticForceToAtoms(hapticInfluencedIndices, position, timeInterval);
    }
    for (int i = 0; i < spheres.size(); i++) {
      Atom *atom = spheres[i];
      if (!atom->isAnchor()) {
        cVector3d old_position = atom->getLocalPos();
        cVector3d new_position = getNewAtomPosition(atom, prevPositions[i], timeInterval);
        prevPositions[i] = old_position;
        applyBoundaryConditions(old_position, new_position);
        atom->setLocalPos(new_position);
      }
    }
    displayedPotentialEnergy.store(potentialEnergy);
  }

  for (int i = 0; i < spheres.size(); i++) {
    spheres[i]->updateVelVector();
  }

  return hapticForce;
}

void initializePrevPositions() {
  prevPositions.resize(spheres.size());
  for (int i = 0; i < spheres.size(); i++) {
    prevPositions[i] = spheres[i]->getLocalPos();
  }
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

// Runs the main haptic simulation loop.
void updateHaptics() {
  // simulation in now running
  simulationRunning = true;
  simulationFinished = false;
  if (!hapticDevice) {
    return;
  }
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
  initializePrevPositions();
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

    // time step the simulation runs at in seconds - shorter timesteps are more
    // accurate but advance the sim more slowly. Driven by the Time Step slider
    // (and HAPTIC_DEVICE_TIME_STEP / the IPC "set timestep" command) so the
    // cout << simulationTimeStep.load() << endl;
    // slider takes effect in haptic mode too, instead of a hardcoded value.
    cVector3d force = stepSimulation(position, simulationTimeStep.load() / ASE_UNITS_TO_FS, true);


    // Hard safety ceiling on the force (N) actually sent to the physical device. 
    const double MAX_HAPTIC_OUTPUT_FORCE = 10.0;

    // scale by the user-configurable feedback intensity, then apply a hard
    // safety ceiling regardless of that scale - so a spike (e.g. two atoms
    // overlapping) can never slam the device at full force even if
    // intensity is set to 100%
    force = clampVectorMagnitude(force * hapticForceScale.load(), MAX_HAPTIC_OUTPUT_FORCE);
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

// Starts the background haptics thread used for simulation updates.
void initializeHapticThread() {
  cThread *hapticsThread = nullptr; // create a thread which starts the main haptics rendering loop
  if (hapticDevice) {
    hapticsThread = new cThread();
    hapticsThread->start(updateHaptics, CTHREAD_PRIORITY_HAPTICS);
    hapticsThreadStarted.store(true);
  }
}

// Recomputes which atom pairs are within BOND_DISTANCE_THRESHOLD of each
// other and shows/hides/creates the cShapeLine connecting each bonded pair.
// Must be called with sceneMutex held.
void updateBonds(cWorld* world) {
  if (!renderBonds.load()) {
    for (auto &entry : bondLines) {
      entry.second->setShowEnabled(false);
    }
  } else {
    set<pair<int, int>> bondedPairs;
    int numAtoms = static_cast<int>(spheres.size());
    for (int i = 0; i < numAtoms; i++) {
      for (int j = i + 1; j < numAtoms; j++) {
        double distance = cDistance(spheres[i]->getLocalPos(), spheres[j]->getLocalPos());
        // Atom pairs closer than this threshold are considered bonded for rendering.
        // TODO: change BOND_DISTANCE_THRESHHOLD to be 1.2 * (R_A + R_B), where R_A and R_B are
        // covalent radii of their atoms. 
        const double BOND_DISTANCE_THRESHOLD = SPHERE_RADIUS * 5.0;
        if (distance < BOND_DISTANCE_THRESHOLD) {
          bondedPairs.insert(make_pair(i, j));
        }
      }
    }

    for (const pair<int, int> &bondedPair : bondedPairs) {
      cShapeLine *&line = bondLines[bondedPair];
      if (!line) {
        line = new cShapeLine(cVector3d(0, 0, 0), cVector3d(0, 0, 0));
        line->setLineWidth(3);
        line->m_colorPointA.setGrayDim();
        line->m_colorPointB.setGrayDim();
        world->addChild(line);
      }
      line->m_pointA = spheres[bondedPair.first]->getLocalPos();
      line->m_pointB = spheres[bondedPair.second]->getLocalPos();
      line->setShowEnabled(true);
    }

    for (auto &entry : bondLines) {
      if (bondedPairs.find(entry.first) == bondedPairs.end()) {
        entry.second->setShowEnabled(false);
      }
    }
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
  const bool debugVisible = showDebug;

  labelRates->setText(cStr(freqCounterGraphics.getFrequency(), 0) + " Hz / " +
                      cStr(freqCounterHaptics.getFrequency(), 0) + " Hz");
  labelRates->setLocalPos((int)(0.5 * (width - labelRates->getWidth())), 15);
  labelRates->setShowEnabled(debugVisible);

  double x = hapticPosition.get(0);
  double y = hapticPosition.get(1);
  double z = hapticPosition.get(2);
  hapticPositionLabel->setText("Position: " + cStr(x, 2) + ", " + cStr(y, 2) + ", " + cStr(z, 2));
  hapticPositionLabel->setShowEnabled(debugVisible);

  updateCameraLabel(camera_pos, camera);
  camera_pos->setShowEnabled(debugVisible);

  displayedTemperature.store(getSliderVal("Temperature", 1.0));
  temperatureLabel->setText("Temperature: " + cStr(displayedTemperature.load(), 5) + " kT");

  string trueFalse = freezeAtoms.load() ? "true" : "false";
  isFrozen->setText("Freeze simulation: " + trueFalse);
  isFrozen->setLocalPos((width - isFrozen->getWidth()) - 5, 15);
  isFrozen->setShowEnabled(debugVisible);

  screenshotLabel->setLocalPos(5, height - 20);
  updateCounters(screenshotLabel, screenshotCounter);

  writeConLabel->setLocalPos(5, height - 40);
  updateCounters(writeConLabel, writeConCounter);

  // Position the help panel, its header, and its hotkey rows relative to the
  // top of the window (rather than a fixed offset from a hypothetical taller
  // window). Row spacing shrinks if needed so every hotkey stays on-screen
  // instead of being pushed below y=0 and disappearing on shorter windows.
  const double topMargin = 10.0;
  const double headerReserve = 60.0;   // vertical space reserved for the header
  const double bottomMargin = 20.0;    // keep the last row off the panel's edge
  const double maxHelpPanelHeight = 500.0;
  const double defaultRowSpacing = 25.0;

  // Size and place the panel first. Its height is capped at maxHelpPanelHeight,
  // so the rows must be laid out against the PANEL height, not the raw window
  // height, or the bottom rows spill out below the panel on tall windows.
  double helpPanelHeight = cMin(maxHelpPanelHeight, cMax(0.0, (double)height - topMargin));
  helpPanel->setSize(520, helpPanelHeight);
  double panelTop = height - topMargin;
  helpPanel->setLocalPos(width - 550, panelTop - helpPanelHeight);
  helpHeader->setLocalPos(width - 490, panelTop - headerReserve + 20);

  // Shrink row spacing if the rows would not otherwise fit inside the panel
  // (between the header at the top and a small margin above the bottom edge).
  int numHotkeyRows = static_cast<int>(hotkeyKeys.size());
  double rowSpacing = defaultRowSpacing;
  if (numHotkeyRows > 1) {
    double availableRowSpace = helpPanelHeight - headerReserve - bottomMargin;
    double neededRowSpace = defaultRowSpacing * (numHotkeyRows - 1);
    if (availableRowSpace > 0 && availableRowSpace < neededRowSpace) {
      rowSpacing = availableRowSpace / (numHotkeyRows - 1);
    }
  }

  double rowStartY = panelTop - headerReserve;
  for (int i = 0; i < hotkeyKeys.size(); i++) {
    cLabel *tempKeyLabel = hotkeyKeys[i];
    cLabel *tempFuncLabel = hotkeyFunctions[i];
    double rowY = rowStartY - i * rowSpacing;
    tempKeyLabel->setLocalPos(width - 530, rowY);
    tempFuncLabel->setLocalPos(width - 350, rowY);
  }
  
  if (showDebug) {
    // current atom force magnitude
    cVector3d force = spheres[currentIndex]->getForce();
    debugLabels[0]->setText("Force magnitude: " + cStr(force.length(), 5));

    // current atom position
    cVector3d pos = spheres[currentIndex]->getLocalPos();
    debugLabels[1]->setText("Atom pos: (" + cStr(pos.x(), 3) + ", " + cStr(pos.y(), 3) + ", " + cStr(pos.z(), 3) + ")");

    // nearest neighbor distance
    double minDist = std::numeric_limits<double>::max();
    for (int i = 0; i < spheres.size(); i++) {
      if (i != currentIndex) {
        double dist = cDistance(spheres[currentIndex]->getLocalPos(), spheres[i]->getLocalPos());
        if (dist < minDist) minDist = dist;
      }
    }
    debugLabels[2]->setText("Nearest neighbor: " + cStr(minDist / 0.02, 5) + " Ang");

    // max force across all atoms
    double maxForce = 0;
    int maxForceIndex = 0;
    for (int i = 0; i < spheres.size(); i++) {
      double mag = spheres[i]->getForce().length();
      if (mag > maxForce) {
        maxForce = mag;
        maxForceIndex = i;
      }
    }
    debugLabels[3]->setText("Max force: " + cStr(maxForce, 5) + " (atom " + to_string(maxForceIndex) + ")");

    // position all debug labels
    for (int i = 0; i < debugLabels.size(); i++) {
      debugLabels[i]->setLocalPos(width - 250, 80 + i * 20);
      debugLabels[i]->setShowEnabled(true);
    }

    // atom index labels  
    for (int i = 0; i < debugAtomLabels.size(); i++) {
      cVector3d atomPos = spheres[i]->getLocalPos();
      cVector3d camPos = camera->getLocalPos();
      cVector3d camLook = camera->getLookVector();
      cVector3d camUp = camera->getUpVector();
      cVector3d camRight = camera->getRightVector();
      cVector3d toAtom = atomPos - camPos;
      double depth = toAtom.dot(camLook);
      if (depth > 0) {
        double fov = camera->getFieldViewAngleRad();
        double scaleY = (0.5 * height) / tan(0.5 * fov);
        double scaleX = scaleY;
        double screenX = (toAtom.dot(camRight) / depth) * scaleX + 0.5 * width;
        double screenY = (toAtom.dot(camUp) / depth) * scaleY + 0.5 * height;
        debugAtomLabels[i]->setLocalPos((int)screenX, (int)screenY);
        debugAtomLabels[i]->setShowEnabled(true);
      } else {
        debugAtomLabels[i]->setShowEnabled(false);
      }
    }

  } else {
    for (int i = 0; i < debugLabels.size(); i++) {
      debugLabels[i]->setShowEnabled(false);
    }
    for (int i = 0; i < debugAtomLabels.size(); i++) {
      debugAtomLabels[i]->setShowEnabled(false);
    }
  }
}

// Updates all scene objects that depend on the current simulation state.
void updateGraphics(cWorld* world) {
  std::lock_guard<std::recursive_mutex> lock(sceneMutex);
  std::atomic<int> displayedAnchoredCount(0);
  // UPDATE WIDGETS
  updateLabels();

  // apply debug rendering toggles for atoms and force vectors, and recompute
  // bond lines for the current atom positions
  bool showAtoms = renderAtoms.load();
  bool showForceVectors = renderForceVectors.load();
  for (int i = 0; i < spheres.size(); i++) {
    spheres[i]->setShowEnabled(showAtoms);
    spheres[i]->getVelVector()->setShowEnabled(showForceVectors);
  }
  updateBonds(world);

  helpPanel->setLocalPos(width - 550, height - 600);
  helpHeader->setLocalPos(width - 490, height - 70);
  
  const bool debugVisible = showDebug;
  const double potentialEnergy = displayedPotentialEnergy.load();
  LJ_num->setText("Potential Energy: " + cStr(potentialEnergy, 5));
  LJ_num->setLocalPos(0, 15, 0);
  LJ_num->setShowEnabled(debugVisible);

  int anchoredCount = 0;
  for (int i = 0; i < spheres.size(); i++) {
    if (spheres[i]->isAnchor()) anchoredCount++;
  }
  num_anchored->setText(to_string(anchoredCount) + " anchored / " +
                        to_string(spheres.size()) + " total");
  num_anchored->setLocalPos((width - num_anchored->getWidth()) - 5, 0);
  num_anchored->setShowEnabled(debugVisible);

  scope->setShowEnabled(debugVisible);
  scope_upper->setShowEnabled(debugVisible);
  scope_lower->setShowEnabled(debugVisible);

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
  camera->renderView(width, height); // render world (width/height are framebuffer pixels)
  glFinish(); // wait until all GL commands are completed
  GLenum err = glGetError(); // check for any OpenGL errors
  if (err != GL_NO_ERROR)
    cout << "Error: " << gluErrorString(err) << endl;
}

void runGraphicsLoop(cWorld* world, GLFWwindow* mainWindow, GLFWwindow* sliderWindow) {
  framebufferSizeCallback(mainWindow, width, height); // initialize framebuffer size
  // main graphic loop
  while (!glfwWindowShouldClose(mainWindow)) {
    glfwGetFramebufferSize(mainWindow, &width, &height); // framebuffer size in pixels (HiDPI-aware)
    if (!hapticDevice) {
      // Advance the sim by the slider-controlled fixed timestep. This used to be
      // min()'d with the real inter-frame time, which capped the timestep at the
      // frame duration on fast machines - so most of the Time Step slider's range
      // produced no visible change. Using the fixed value directly makes the whole
      // slider range (including the new slower minimum) actually take effect.
      freqCounterHaptics.signal(1);
      
      std::cout << simulationTimeStep.load() << std::endl;
      stepSimulation(cVector3d(0.0, 0.0, 0.0), simulationTimeStep.load() / ASE_UNITS_TO_FS, false);
    }
    updateGraphics(world); // render graphics
    glfwSwapBuffers(mainWindow); // swap buffers
    renderSliderWindow(mainWindow, sliderWindow);
    glfwPollEvents(); // process events
    freqCounterGraphics.signal(1); // signal frequency counter
  }
}

// Nudges the current atom along the camera axes using the keyboard controls.
void moveCurrentAtom(double rightAmount, double upAmount, double forwardAmount) {
  std::lock_guard<std::recursive_mutex> lock(sceneMutex);
  if (spheres.empty()) {
    return;
  }
  Atom *current = spheres[currentIndex];

  // Distance that the current atom moves when nudged with the keyboard controls.
  const double ATOM_MOVE_STEP = DIST_SCALE;

  cVector3d delta = ATOM_MOVE_STEP * (rightAmount * camera->getRightVector() +
                                      upAmount * camera->getUpVector() +
                                      forwardAmount * camera->getLookVector());
  current->setLocalPos(current->getLocalPos() + delta);
  if (currentIndex < prevPositions.size()) {
    prevPositions[currentIndex] = current->getLocalPos();
  }
}

// Adds a new label to the scene using the default style.
void addLabel(cLabel *&label);

// Updates the text label that displays the camera position.
void updateCameraLabel(cLabel *&camera_pos, cCamera *&camera);

// Writes the current atom configuration to a .con file.
void writeToCon(string fileName);

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

// on Windows, double-clicking the .exe directly (rather than launching it
// through launcher/main.py, which supplies the required arguments) used to
// crash instantly: the console window this project builds as opens, an
// unhandled exception fires (e.g. missing haptic mode argument) and
// std::terminate closes the window again before anyone can read why. main()
// below catches that and keeps the window open with the error instead.
int runApplication(int argc, char *argv[]) {
  srand(time(nullptr)); // initialize random seed
  
  // Selects whether the 3D view uses stereo rendering.
  const chai3d::cStereoMode STEREO_MODE = C_STEREO_DISABLED; 

  // OPEN GL - WINDOW DISPLAY
  configureGLFW(STEREO_MODE);
  GLFWwindow* mainWindow = initializeMainWindow();
  initializeGLEW();

  // WORLD - CAMERA - LIGHTING
  cWorld* world = initializeWorld();
  initializeCamera(world, STEREO_MODE);
  initializeLight(world);
  
  // HAPTIC DEVICE
  initializeHapticDevice();
  
  if (argc < 2) {
    throw std::runtime_error("Missing haptic mode argument");
  }
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

  // PBC argument (argv[5]): "on" forces periodic boundaries on in all three
  // directions, "off" forces them off, and "keep" (or omitting the argument)
  // leaves whatever the loaded structure file specified untouched.
  string pbcMode = "keep";
  if (argc > 5) {
    pbcMode = argv[5];
    for (char &c : pbcMode) {
      c = tolower(c);
    }
  }

  printIntro();
  // PLACE ATOMS
  placeAtoms(world, aseCell, asePbc, argc, argv);
  initializeAtomLabels();
  for (int i = 0; i < spheres.size(); i++) {
    initialPositions.push_back(spheres[i]->getLocalPos());
  }

  if (pbcMode == "on" || pbcMode == "true" || pbcMode == "1" || pbcMode == "yes") {
    asePbc = {1, 1, 1};
  } else if (pbcMode == "off" || pbcMode == "false" || pbcMode == "0" || pbcMode == "no") {
    asePbc = {0, 0, 0};
  }

  // determine potential if specified
  if (argc > 3) {
    initializeCalculator(argc, argv, aseCell, asePbc);
  } else {
    cerr << "No potential specified. Defaulting to Lennard-Jones." << endl;
    calculatorPtr = new ljCalculator();
  }

  // WIDGETS
  // helpPanel must be added to the front layer before the hotkey labels
  // (added inside initializeLabels) so the labels draw on top of the panel
  // background instead of being occluded by it.
  initializeHelpPanel();
  initializeLabels();
  initializePotentialEnergyPlot();

  // initial time step override, e.g. from the desktop launcher UI
  if (const char *timeStepEnv = std::getenv("HAPTIC_DEVICE_TIME_STEP")) {
    setLiveTimeStep(atof(timeStepEnv));
  }

  // initial haptic feedback intensity override, e.g. from the desktop
  // launcher UI; lets owners of older/more worn devices start already turned
  // down instead of having to dial it back after every launch
  if (const char *forceScaleEnv = std::getenv("HAPTIC_DEVICE_FORCE_SCALE")) {
    setLiveForceScale(atof(forceScaleEnv));
  }

  // IPC SERVER - lets the desktop launcher UI query status and change
  // parameters (freeze, haptic mode, potential, anchors, time step) while running
  int ipcPort = 8765;
  if (const char *portEnv = std::getenv("HAPTIC_DEVICE_CMD_PORT")) {
    ipcPort = atoi(portEnv);
    if (ipcPort <= 0) {
      ipcPort = 8765;
    }
  }
  startIpcServer(ipcPort);

  GLFWwindow* sliderWindow = initializeSliderWindow(mainWindow);
  

  // START SIMULATION
  initializeHapticThread();
  
  // MAIN GRAPHIC LOOP
  runGraphicsLoop(world, mainWindow, sliderWindow);
  close();

  // close window
  if (sliderWindow != nullptr) {
    glfwDestroyWindow(sliderWindow);
    sliderWindow = nullptr;
  }
  glfwDestroyWindow(mainWindow);
  mainWindow = nullptr;

  glfwTerminate(); // terminate GLFW library
  return 0;
}

int main(int argc, char *argv[]) {
  try {
    return runApplication(argc, argv);
  } catch (const std::exception &e) {
    cerr << endl << "Fatal error: " << e.what() << endl;
    cerr << "(run this binary through launcher/main.py, or pass the haptic "
            "mode argument yourself - see README.md)" << endl;
    cerr << "Press Enter to close this window..." << endl;
    cin.get();
    return 1;
  }
}

bool setLivePotential(const std::string &requested) {
  string potential = requested;
  for (char &c : potential) {
    c = static_cast<char>(tolower(static_cast<unsigned char>(c)));
  }

  Calculator *newCalculator = nullptr;
  LocalPotential newSurface;
  if (potential == "lj" || potential == "lennard-jones") {
    newCalculator = new ljCalculator();
    newSurface = LENNARD_JONES;
  } else if (potential == "morse") {
    newCalculator = new morseCalculator();
    newSurface = MORSE;
  } else {
    // live switching to ASE is not supported since it needs constructor
    // arguments (structure file, calculator spec) only available at launch
    return false;
  }

  std::lock_guard<std::recursive_mutex> lock(sceneMutex);
  delete calculatorPtr;
  calculatorPtr = newCalculator;
  energySurface = newSurface;
  initializePotentialLabel();
  return true;
}

bool setLiveTimeStep(double seconds) {
  if (!std::isfinite(seconds) ||
      seconds < MIN_SIMULATION_TIME_STEP ||
      seconds > MAX_SIMULATION_TIME_STEP) {
    return false;
  }
  simulationTimeStep.store(seconds);
  return true;
}

bool setLiveSettlingError(double value) {
  if (!std::isfinite(value) || value < MIN_SETTLING_ERROR || value > MAX_SETTLING_ERROR) {
    return false;
  }
  settlingError.store(value);
  return true;
}

bool setLiveKReturn(double value) {
  if (!std::isfinite(value) || value < MIN_K_RETURN || value > MAX_K_RETURN) {
    return false;
  }
  kReturn.store(value);
  return true;
}

bool setLiveKDampen(double value) {
  if (!std::isfinite(value) || value < MIN_K_DAMPEN || value > MAX_K_DAMPEN) {
    return false;
  }
  kDampen.store(value);
  return true;
}

bool setLiveReturnDelay(double value) {
  if (!std::isfinite(value) || value < MIN_RETURN_DELAY_SECONDS || value > MAX_RETURN_DELAY_SECONDS) {
    return false;
  }
  returnDelaySeconds.store(value);
  return true;
}

bool setLiveForceScale(double value) {
  if (!std::isfinite(value) || value < MIN_FORCE_SCALE || value > MAX_FORCE_SCALE) {
    return false;
  }
  hapticForceScale.store(value);
  return true;
}

void switchCamera() {
  std::lock_guard<std::recursive_mutex> lock(sceneMutex);
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
  std::lock_guard<std::recursive_mutex> lock(sceneMutex);
  if (spheres.empty()) {
    return;
  }
  currentIndex %= spheres.size();
  Atom* current = spheres[currentIndex];
  int prev_curr_atom = currentIndex;
  currentIndex = (currentIndex + 1) % spheres.size();  if (currentIndex < 0)
    currentIndex += spheres.size();
  int startAtom = currentIndex;
  while (spheres[currentIndex]->isAnchor()) {
    currentIndex = (currentIndex + 1) % spheres.size();    if (currentIndex < 0)
      currentIndex += spheres.size();
    if (currentIndex == startAtom)
      break;
  }

  current = spheres[currentIndex];
  current->setCurrent(true);

  Atom *prev = spheres[prev_curr_atom];
  prev->setCurrent(false);
}

// cVector3d forceModeUpdate(Atom *current, cVector3d position, const double timeInterval) {
//   const double K_CURRENT       = K_HAPTIC_SPRING;
//   const double K_CURRENT_DAMP  = K_HAPTIC_DAMPER;
//   const double K_HAPTIC        = K_HAPTIC_SPRING;
//   const double K_HAPTIC_DAMP   = K_HAPTIC_DAMPER;

//   cVector3d currentPrevPos = prevPositions[currentIndex];
//   cVector3d x_curr = current->getLocalPos();

//   // Physical (UMA) force: integrate in world units with the ASE scaling.
//   cVector3d physForce = current->getForce();
//   cVector3d physAcc;
//   if (energySurface == ASE) {
//     physAcc = physForce / current->getMass() * DIST_SCALE;      // ×0.02
//   } else {
//     physAcc = physForce / (current->getMass() * SPHERE_MASS_SCALE_FACTOR);
//   }
//   if (!isFiniteVector(physAcc)) physAcc.zero();

//   // Haptic spring: defined directly in world/display units. Keep it on the
//   // ORIGINAL scaling so its feel is unchanged by the physics-unit fix.
//   cVector3d positionErr = position - x_curr;
//   cVector3d velocity    = (x_curr - currentPrevPos) / timeInterval;
//   cVector3d hapticAcc   = (positionErr * K_CURRENT - velocity * K_CURRENT_DAMP)
//                           / (current->getMass() * SPHERE_MASS_SCALE_FACTOR);
                          
//   if (!isFiniteVector(hapticAcc)) hapticAcc.zero();

//   // Verlet, summing the two accelerations only at integration time.
//   cVector3d newPos = x_curr + (x_curr - currentPrevPos)
//                    + (physAcc + hapticAcc) * timeInterval * timeInterval;
//   current->setLocalPos(newPos);
//   prevPositions[currentIndex] = newPos;

//   cVector3d forceErr        = current->getLocalPos() - position;
//   cVector3d hapticVelocity  = (current->getLocalPos() - currentPrevPos) / timeInterval;
//   return forceErr * K_HAPTIC - hapticVelocity * K_HAPTIC_DAMP;
// }


cVector3d prevHapticPosition(0,0,0);
// cPrecisionClock positionClock;
// bool standby;
// bool resetting;
// bool confirming;
// bool simulating;
// double finalErr;

// const int MAX_FORCE_HISTORY = 10000;
// cVector3d prevForces[MAX_FORCE_HISTORY];
// int prevForcesIndex = 0;

// double getStrongestScalarProj(cVector3d v) {
//   const double length = v.length();
//   if (length <= 1e-9) {
//     return 0.0;
//   }

//   double strongest = 0;
//   for (int i = 0; i < MAX_FORCE_HISTORY; i++) {
//     double scalarProj = v.dot(prevForces[i]) / length;
//     if (strongest < scalarProj) {
//       strongest = scalarProj;
//     }
//   }
//   return strongest;
// }

// void recordForceHistory(Atom *current) {
//   constexpr double REST_ERR = 0.001;

//   if (!simulating) return;
//   if (current->getForce().length() < REST_ERR) return;

//   prevForces[prevForcesIndex] = current->getForce();
//   prevForcesIndex = (prevForcesIndex + 1) % MAX_FORCE_HISTORY;
// }

// std::optional<cVector3d> updateStandbyModeSimulating(Atom *current, cVector3d& position, double timeInterval) {
//   constexpr double HAPTIC_RADIUS = .02;
//   constexpr double K_HAPTIC = 1; // spring constant for applying vector projection

//   if (position.length() < HAPTIC_RADIUS) {
//     cVector3d temp = current->getLocalPos();
//     current->setLocalPos(getNewAtomPosition(current, prevPositions[currentIndex], timeInterval));
//     prevPositions[currentIndex] = temp;

//     double distFromCenter = position.length() - finalErr;
//     if (position.length() <= 1e-9) {
//       return cVector3d(0, 0, 0);
//     }

//     cVector3d direction = cNormalize(position);
//     return getStrongestScalarProj(-direction) * -direction * (distFromCenter / HAPTIC_RADIUS) * K_HAPTIC;
//   }
//   simulating = false;
//   return std::nullopt;
// }

// std::optional<cVector3d> updateStandbyState(Atom *current, const cVector3d& position,
//                                           const cVector3d& dPHaptic, double timeInterval) {
//   // position err acceptable for return mechanism to return to center,
//   // live-tunable via the IPC "set settling_err" command / launcher UI
//   double SETTLING_ERR = settlingError.load();
//   // spring constant for return haptic controller to center, live-tunable via
//   // the IPC "set k_return" command / launcher UI
//   double K_RETURN = kReturn.load();
//   constexpr double K_DAMPING = 8.0;
//   constexpr double RETURN_DELAY_SECONDS = 5.0;

//   if (position.length() >= SETTLING_ERR) {
//     if (resetting && confirming) {
//       cout << "Not yet settled!" << endl;
//     }
//     confirming = false;
//     // delay (seconds) after entering standby before the return mechanism
//     // kicks in, live-tunable via the IPC "set return_delay" command / launcher UI
//     if (positionClock.getCurrentTimeSeconds() >= returnDelaySeconds.load() || resetting) {
//       positionClock.stop();
//       positionClock.reset();
//       if (!resetting) {
//         cout << "Resetting to center..." << endl;
//       }
//       resetting = true;

//       const double MAX_FORCE = 1.6; // maximum force the return mechanism should output

//       double distanceFromCenter = position.length();
//       double springScale = 0.5 + (distanceFromCenter / (SETTLING_ERR * 2.0));
//       if (springScale > 1.0) {
//         springScale = 1.0;
//       }

//       // velocity-based damping on the return spring, live-tunable via the
//       // IPC "set k_dampen" command / launcher UI; defaults to 0 (no
//       // damping) so existing return behavior is unchanged until set
//       cVector3d hapticVelocity = dPHaptic / timeInterval;
//       cVector3d returnVector = -position * K_RETURN - hapticVelocity * kDampen.load();
//       return clampVectorMagnitude(returnVector, MAX_FORCE);
//     }
//   } else {
//     if (!confirming) {
//       cout << "Starting confirmation..." << endl;
//       positionClock.start();
//       confirming = true;
//     }
    
//     if (positionClock.getCurrentTimeSeconds() >= .5) {
//       cout << "Done!" << endl;
//       standby = false;
//       resetting = false;
//       confirming = false;
//       prevHapticInitialized = false;
//       simulating = true;
//       positionClock.stop();
//       positionClock.reset();

//       prevPositions[currentIndex] = current->getLocalPos();
//       finalErr = position.length();
//     }
//     return cVector3d(0,0,0);
//   }
//   return std::nullopt;
// }

// cVector3d standbyModeUpdate(Atom *current, cVector3d position, const double timeInterval) {
//   constexpr double STANDBY_ERR = .1; // movement err acceptable for standby mode to activate

//   if (!prevHapticInitialized) {
//     prevHapticInitialized = true;
//     prevHapticPosition = position;
//   }
//   recordForceHistory(current);

//   cVector3d dPHaptic = position - prevHapticPosition;
//   prevHapticPosition = position;

//   if (simulating) {
//     auto pos = updateStandbyModeSimulating(current, position, timeInterval);
//     if (pos) return *pos;
//   } else {
//     if (dPHaptic.length() < STANDBY_ERR && !standby && position.length() >= .01) {
//       positionClock.start();
//       standby = true;
//       cout << "Entering standby mode..." << endl;
//     }
//     if (standby) {
//       auto pos = updateStandbyState(current, position, dPHaptic, timeInterval);
//       if (pos) return *pos;
//     }

//     if (dPHaptic.length() >= 1e-6 && standby && !resetting) { 
//       standby = false;
//       positionClock.stop();
//       positionClock.reset();
//       cout << "Standby cancelled!" << endl;
//     }
//   }

//   current->setLocalPos(current->getLocalPos() + dPHaptic);
//   return current->getForce();
// }

cVector3d positionModeUpdate(Atom *current, cVector3d position, const double timeInterval) {
  const double VELOCITY_MULT = 25;
  const double ATTRACTION_MAX = 1.5;
  cVector3d currentPos = current->getLocalPos();
  cVector3d attraction = (position - currentPos) * timeInterval * VELOCITY_MULT;
  current->setLocalPos(currentPos + clampVectorMagnitude(attraction, ATTRACTION_MAX * timeInterval));
  prevPositions[currentIndex] = currentPos;
  return cVector3d(0,0,0);
}


void ensureSelectedAtomOffsets(const vector<int> &selectedIndices,
                               const cVector3d &position) {
  if (selectedIndices == activeHapticSelection) {
    return;
  }

  activeHapticSelection = selectedIndices;
  activeHapticSelectionOffsets.clear();
  for (int index : selectedIndices) {
    activeHapticSelectionOffsets[index] = spheres[index]->getLocalPos() - position;
  }
  prevHapticInitialized = false;
}

cVector3d getSelectedAtomTarget(int index, const cVector3d &position) {
  return position + activeHapticSelectionOffsets[index];
}

cVector3d forceModeUpdateSelectedGroup(const vector<int> &selectedIndices,
                                      cVector3d position,
                                      const double timeInterval) {
  const double K_CURRENT = K_HAPTIC_SPRING;
  const double K_CURRENT_DAMP = K_HAPTIC_DAMPER;
  ensureSelectedAtomOffsets(selectedIndices, position);

  cVector3d averageSimulationForce = getAverageAtomGroupForce(selectedIndices);

  for (int index : selectedIndices) {
    Atom *atom = spheres[index];
    cVector3d currentPosition = atom->getLocalPos();
    cVector3d previousPosition = prevPositions[index];
    cVector3d targetPosition = getSelectedAtomTarget(index, position);
    cVector3d velocity = (currentPosition - previousPosition) / timeInterval;
    cVector3d hapticForce = (targetPosition - currentPosition) * K_CURRENT -
                            velocity * K_CURRENT_DAMP;

    atom->setForce(atom->getForce() + hapticForce);
    cVector3d newPosition = getNewAtomPosition(atom, previousPosition, timeInterval);
    applyBoundaryConditions(currentPosition, newPosition);
    atom->setLocalPos(newPosition);
    prevPositions[index] = currentPosition;
  }

  return averageSimulationForce;
}

cVector3d positionModeUpdateSelectedGroup(const vector<int> &selectedIndices,
                                         cVector3d position,
                                         const double timeInterval) {
  const double VELOCITY_MULT = 25;
  const double ATTRACTION_MAX = 1.5;
  ensureSelectedAtomOffsets(selectedIndices, position);

  for (int index : selectedIndices) {
    Atom *atom = spheres[index];
    cVector3d oldPosition = atom->getLocalPos();
    cVector3d targetPosition = getSelectedAtomTarget(index, position);
    cVector3d attraction = (targetPosition - oldPosition) * timeInterval * VELOCITY_MULT;
    cVector3d newPosition = oldPosition +
        clampVectorMagnitude(attraction, ATTRACTION_MAX * timeInterval);
    applyBoundaryConditions(oldPosition, newPosition);
    atom->setLocalPos(newPosition);
    prevPositions[index] = oldPosition;
  }
  return getAverageAtomGroupForce(selectedIndices);
}

cVector3d standbyModeUpdateSelectedGroup(const vector<int> &selectedIndices,
                                        cVector3d position,
                                        const double timeInterval) {
  ensureSelectedAtomOffsets(selectedIndices, position);
  if (!prevHapticInitialized) {
    prevHapticInitialized = true;
    prevHapticPosition = position;
  }

  cVector3d dPHaptic = position - prevHapticPosition;
  prevHapticPosition = position;
  for (int index : selectedIndices) {
    Atom *atom = spheres[index];
    cVector3d oldPosition = atom->getLocalPos();
    cVector3d newPosition = oldPosition + dPHaptic;
    applyBoundaryConditions(oldPosition, newPosition);
    atom->setLocalPos(newPosition);
    prevPositions[index] = oldPosition;
  }
  return getAverageAtomGroupForce(selectedIndices);
}