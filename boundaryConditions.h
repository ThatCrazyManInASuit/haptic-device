#ifndef BOUNDARY_CONDITIONS_H
#define BOUNDARY_CONDITIONS_H

#include "chai3d.h"

chai3d::cVector3d applyBoundaryConditions(chai3d::cVector3d pos, std::array<double, 9>& aseCell,
    std::array<int, 3>& asePbc);

#endif