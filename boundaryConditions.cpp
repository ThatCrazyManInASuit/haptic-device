#include "boundaryConditions.h"
#include "chai3d.h"
#include "globals.h"
#include <cmath>

/**
 * @brief Changes an atom's position to stay within a boundary
 * @param position the position of the atom
 * @param aseCell the 3x3 ASE cell matrix, flattened into a 9-element array
 * @param asePbc the periodic boundary condition to use
 */
chai3d::cVector3d applyBoundaryConditions(chai3d::cVector3d pos, std::array<double, 9>& aseCell,
    std::array<int, 3>& asePbc) {
  cVector3d initialCoords(centerCoords[0], centerCoords[1], centerCoords[2]);
  pos = pos / DIST_SCALE + initialCoords;
  
  chai3d::cMatrix3d cell(aseCell[0], aseCell[1], aseCell[2], aseCell[3], aseCell[4], aseCell[5],
      aseCell[6], aseCell[7], aseCell[8]);
  cell.invert();
  cVector3d fracCoords = cell * pos;
  if (asePbc[0]) {
    fracCoords.x(fracCoords.x() - std::floor(fracCoords.x()));
  }
  if (asePbc[1]) {
    fracCoords.y(fracCoords.y() - std::floor(fracCoords.y()));
  }
  if (asePbc[2]) {
    fracCoords.z(fracCoords.z() - std::floor(fracCoords.z()));
  }
  cell.invert();
  return DIST_SCALE * (cell * fracCoords - initialCoords);
}