#ifndef META_POLYHEDRA_H
#define META_POLYHEDRA_H

#include "chai3d.h"
#include <vector>

/**
 * @brief A coordination polyhedron described as the half-spaces that bound it, which is all the
 *        scalar field needs in order to measure how far a point lies inside or outside the hull
 */
struct PolyhedronField {
    // Outward unit normal of each hull face, and that face's plane offset along it.
    std::vector<chai3d::cVector3d> planeNormals;
    std::vector<double> planeOffsets;
    // Axis-aligned bounds of the hull's vertices, used to limit how much of the sampling grid this
    // polyhedron has to touch.
    chai3d::cVector3d boundsMin;
    chai3d::cVector3d boundsMax;
    chai3d::cColorf color;
};

/**
 * @brief Replaces a mesh with a single smooth surface enclosing all the given polyhedra, fusing
 *        neighbours that come within the merge radius of one another the way metaballs do.
 *        Widening the radius thickens the joins until they close over the gaps between polyhedra,
 *        which is what lets a channel or cage read as an enclosed volume.
 * @param polyhedra the polyhedra to merge; untouched
 * @param mergeRadius how wide the blend between neighbouring polyhedra is, in world units. At or
 *                    below zero the polyhedra are merged but not blended, giving their exact union
 * @param gridStep target spacing of the sampling grid in world units. Coarsened automatically if
 *                 the structure is large enough that this spacing would need too many samples
 * @param mesh the mesh to clear and fill with the resulting surface
 */
void buildMergedPolyhedra(const std::vector<PolyhedronField> &polyhedra,
                          double mergeRadius,
                          double gridStep,
                          chai3d::cMesh *mesh);

#endif  // META_POLYHEDRA_H
