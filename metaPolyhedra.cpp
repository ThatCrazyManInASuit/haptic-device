#include "metaPolyhedra.h"

#include <algorithm>
#include <cmath>
#include <vector>

namespace {

// Field value standing for "no polyhedron reaches here". Large enough to lose
// every smooth-minimum against a real distance, small enough to stay far from
// overflow when differences are taken.
const double FIELD_EMPTY = 1.0e9;

// Ceiling on sampling grid nodes. The grid is coarsened until it fits, so a
// large structure degrades in surface detail rather than in frame rate.
const size_t MAX_GRID_NODES = 6000000;

/**
 * @brief Blends two signed distances instead of taking the hard minimum of them, so that two
 *        surfaces meeting within the blend width fuse into one rounded join rather than crossing
 *        in a crease. This is what turns a row of separate polyhedra into one connected body.
 * @param a first signed distance
 * @param b second signed distance
 * @param blendWidth width of the blend band; at or below zero this is a plain minimum
 * @return the blended signed distance
 */
double smoothMin(double a, double b, double blendWidth) {
    if (blendWidth <= 0.0) {
        return std::min(a, b);
    }
    double overlap = std::max(blendWidth - std::fabs(a - b), 0.0) / blendWidth;
    return std::min(a, b) - overlap * overlap * blendWidth * 0.25;
}

/**
 * @brief Measures how far a point lies inside or outside a convex hull. Because the hull is the
 *        intersection of its half-spaces, the half-space the point is furthest outside of is the
 *        one that decides.
 * @param hull the polyhedron to measure against
 * @param at the point to measure
 * @return negative inside the hull, zero on its surface, positive outside
 */
double hullDistance(const PolyhedronField &hull, const chai3d::cVector3d &at) {
    double furthest = -FIELD_EMPTY;
    for (size_t i = 0; i < hull.planeOffsets.size(); i++) {
        furthest = std::max(furthest, chai3d::cDot(hull.planeNormals[i], at) - hull.planeOffsets[i]);
    }
    return furthest;
}

/**
 * @brief The sampled scalar field the surface is extracted from. Each node holds the blended
 *        distance to the nearest polyhedra and which polyhedron is closest, so the extracted
 *        surface can be tinted by whichever polyhedron owns that stretch of it.
 */
struct SampleGrid {
    int nodesX = 0;
    int nodesY = 0;
    int nodesZ = 0;
    double step = 0.0;
    chai3d::cVector3d origin;
    std::vector<double> distance;
    std::vector<int> owner;

    size_t index(int x, int y, int z) const {
        return (static_cast<size_t>(z) * nodesY + y) * nodesX + x;
    }

    chai3d::cVector3d position(int x, int y, int z) const {
        return origin + chai3d::cVector3d(x * step, y * step, z * step);
    }

    // Central difference of the field, which points away from the enclosed
    // volume and so serves as the outward surface normal.
    chai3d::cVector3d gradient(int x, int y, int z) const {
        int xLo = std::max(x - 1, 0), xHi = std::min(x + 1, nodesX - 1);
        int yLo = std::max(y - 1, 0), yHi = std::min(y + 1, nodesY - 1);
        int zLo = std::max(z - 1, 0), zHi = std::min(z + 1, nodesZ - 1);
        return chai3d::cVector3d(distance[index(xHi, y, z)] - distance[index(xLo, y, z)],
                                 distance[index(x, yHi, z)] - distance[index(x, yLo, z)],
                                 distance[index(x, y, zHi)] - distance[index(x, y, zLo)]);
    }
};

/**
 * @brief Lays out a grid covering every polyhedron plus the distance the blend reaches beyond them
 * @param polyhedra the polyhedra to cover
 * @param reach how far outside the hulls the surface can still appear
 * @param gridStep the requested node spacing, coarsened if the grid would be too large
 * @return the grid, sized and positioned but not yet sampled
 */
SampleGrid layOutGrid(const std::vector<PolyhedronField> &polyhedra, double reach, double gridStep) {
    chai3d::cVector3d low = polyhedra[0].boundsMin;
    chai3d::cVector3d high = polyhedra[0].boundsMax;
    for (const PolyhedronField &hull : polyhedra) {
        for (int axis = 0; axis < 3; axis++) {
            low(axis) = std::min(low(axis), hull.boundsMin(axis));
            high(axis) = std::max(high(axis), hull.boundsMax(axis));
        }
    }
    // One spare node past the reach on each side keeps the surface closed: the
    // outermost nodes stay empty, so nothing is left dangling at the boundary.
    chai3d::cVector3d margin(reach + gridStep, reach + gridStep, reach + gridStep);
    low -= margin;
    high += margin;

    SampleGrid grid;
    grid.origin = low;
    grid.step = gridStep;
    for (int attempt = 0; attempt < 16; attempt++) {
        grid.nodesX = static_cast<int>((high.x() - low.x()) / grid.step) + 2;
        grid.nodesY = static_cast<int>((high.y() - low.y()) / grid.step) + 2;
        grid.nodesZ = static_cast<int>((high.z() - low.z()) / grid.step) + 2;
        size_t nodes = static_cast<size_t>(grid.nodesX) * grid.nodesY * grid.nodesZ;
        if (nodes <= MAX_GRID_NODES) {
            break;
        }
        grid.step *= 1.5;
    }
    size_t nodes = static_cast<size_t>(grid.nodesX) * grid.nodesY * grid.nodesZ;
    grid.distance.assign(nodes, FIELD_EMPTY);
    grid.owner.assign(nodes, -1);
    return grid;
}

/**
 * @brief Samples every polyhedron into the grid. Each one only visits the nodes within its own
 *        bounds plus the blend reach, so the cost scales with how much space the polyhedra
 *        actually occupy rather than with the size of the grid.
 * @param polyhedra the polyhedra to sample
 * @param reach how far outside a hull its influence extends
 * @param mergeRadius width of the blend between neighbours
 * @param grid the grid to accumulate into
 */
void sampleGrid(const std::vector<PolyhedronField> &polyhedra, double reach, double mergeRadius,
                SampleGrid &grid) {
    for (size_t which = 0; which < polyhedra.size(); which++) {
        const PolyhedronField &hull = polyhedra[which];
        int xLo = std::max(0, static_cast<int>((hull.boundsMin.x() - reach - grid.origin.x()) / grid.step));
        int yLo = std::max(0, static_cast<int>((hull.boundsMin.y() - reach - grid.origin.y()) / grid.step));
        int zLo = std::max(0, static_cast<int>((hull.boundsMin.z() - reach - grid.origin.z()) / grid.step));
        int xHi = std::min(grid.nodesX - 1, static_cast<int>((hull.boundsMax.x() + reach - grid.origin.x()) / grid.step) + 1);
        int yHi = std::min(grid.nodesY - 1, static_cast<int>((hull.boundsMax.y() + reach - grid.origin.y()) / grid.step) + 1);
        int zHi = std::min(grid.nodesZ - 1, static_cast<int>((hull.boundsMax.z() + reach - grid.origin.z()) / grid.step) + 1);

        for (int z = zLo; z <= zHi; z++) {
            for (int y = yLo; y <= yHi; y++) {
                for (int x = xLo; x <= xHi; x++) {
                    double here = hullDistance(hull, grid.position(x, y, z));
                    if (here > reach) {
                        continue;  // too far outside this hull to affect the surface
                    }
                    size_t node = grid.index(x, y, z);
                    double before = grid.distance[node];
                    grid.distance[node] = smoothMin(before, here, mergeRadius);
                    if (here < before) {
                        grid.owner[node] = static_cast<int>(which);
                    }
                }
            }
        }
    }
}

// One corner of a tetrahedron as the extractor sees it.
struct Corner {
    chai3d::cVector3d position;
    chai3d::cVector3d normal;
    double distance;
    chai3d::cColorf color;
};

/**
 * @brief Finds where the surface crosses the line between two corners and describes that point
 * @param a the corner on one side of the surface
 * @param b the corner on the other side
 * @return the crossing point, with the normal and colour interpolated to match
 */
Corner crossing(const Corner &a, const Corner &b) {
    double span = a.distance - b.distance;
    double along = (std::fabs(span) > 1e-12) ? a.distance / span : 0.5;
    along = std::min(std::max(along, 0.0), 1.0);

    Corner at;
    at.position = a.position + (b.position - a.position) * along;
    at.normal = a.normal + (b.normal - a.normal) * along;
    if (at.normal.lengthsq() > 0.0) {
        at.normal.normalize();
    }
    at.distance = 0.0;
    // The enclosed side owns the surface, so take its colour rather than
    // blending towards whatever empty space happens to sit opposite.
    at.color = (a.distance < b.distance) ? a.color : b.color;
    return at;
}

/**
 * @brief Adds one surface triangle to the mesh, wound so that it faces out of the enclosed volume
 * @param a first corner of the triangle
 * @param b second corner
 * @param c third corner
 * @param mesh the mesh to add to
 */
void addSurfaceTriangle(const Corner &a, const Corner &b, const Corner &c, chai3d::cMesh *mesh) {
    chai3d::cVector3d facing = chai3d::cComputeSurfaceNormal(a.position, b.position, c.position);
    if (facing.lengthsq() == 0.0) {
        return;  // degenerate sliver, nothing to draw
    }
    const chai3d::cVector3d NO_TEXTURE(0, 0, 0);
    bool outward = chai3d::cDot(facing, a.normal + b.normal + c.normal) >= 0.0;
    const Corner &second = outward ? b : c;
    const Corner &third = outward ? c : b;
    mesh->newTriangle(a.position, second.position, third.position,
                      a.normal, second.normal, third.normal,
                      NO_TEXTURE, NO_TEXTURE, NO_TEXTURE,
                      a.color, second.color, third.color);
}

/**
 * @brief Extracts the piece of surface passing through one tetrahedron. Splitting cells into
 *        tetrahedra rather than marching the cubes directly keeps the case analysis small enough
 *        to read: a tetrahedron is cut either into a triangle or into a quadrilateral.
 * @param cube the eight corners of the cell the tetrahedron was carved from
 * @param pick which four of those corners make up this tetrahedron
 * @param mesh the mesh to add the resulting triangles to
 */
void addTetrahedronSurface(const Corner cube[8], const int pick[4], chai3d::cMesh *mesh) {
    const Corner *corner[4] = {&cube[pick[0]], &cube[pick[1]], &cube[pick[2]], &cube[pick[3]]};
    int inside[4];
    int outside[4];
    int insideCount = 0;
    int outsideCount = 0;
    for (int i = 0; i < 4; i++) {
        if (corner[i]->distance < 0.0) {
            inside[insideCount++] = i;
        } else {
            outside[outsideCount++] = i;
        }
    }

    if (insideCount == 0 || insideCount == 4) {
        return;  // wholly inside or wholly outside: no surface crosses here
    }
    if (insideCount == 1) {
        const Corner &apex = *corner[inside[0]];
        addSurfaceTriangle(crossing(apex, *corner[outside[0]]),
                           crossing(apex, *corner[outside[1]]),
                           crossing(apex, *corner[outside[2]]), mesh);
        return;
    }
    if (insideCount == 3) {
        const Corner &apex = *corner[outside[0]];
        addSurfaceTriangle(crossing(apex, *corner[inside[0]]),
                           crossing(apex, *corner[inside[1]]),
                           crossing(apex, *corner[inside[2]]), mesh);
        return;
    }
    // Two in, two out: the surface cuts all four edges that span the divide,
    // leaving a quadrilateral that splits into two triangles.
    Corner quad[4] = {crossing(*corner[inside[0]], *corner[outside[0]]),
                      crossing(*corner[inside[0]], *corner[outside[1]]),
                      crossing(*corner[inside[1]], *corner[outside[1]]),
                      crossing(*corner[inside[1]], *corner[outside[0]])};
    addSurfaceTriangle(quad[0], quad[1], quad[2], mesh);
    addSurfaceTriangle(quad[0], quad[2], quad[3], mesh);
}

/**
 * @brief Walks the sampled grid and builds the surface where the field changes sign
 * @param grid the sampled field
 * @param polyhedra the polyhedra, consulted only for their colours
 * @param mesh the mesh to fill
 */
void extractSurface(const SampleGrid &grid, const std::vector<PolyhedronField> &polyhedra,
                    chai3d::cMesh *mesh) {
    // Cube corners in the order the tetrahedra below index them.
    static const int CORNER_OFFSET[8][3] = {{0, 0, 0}, {1, 0, 0}, {1, 1, 0}, {0, 1, 0},
                                            {0, 0, 1}, {1, 0, 1}, {1, 1, 1}, {0, 1, 1}};
    // Six tetrahedra filling the cube, all sharing its 0-6 diagonal.
    static const int TETRAHEDRA[6][4] = {{0, 1, 2, 6}, {0, 2, 3, 6}, {0, 3, 7, 6},
                                         {0, 7, 4, 6}, {0, 4, 5, 6}, {0, 5, 1, 6}};

    for (int z = 0; z + 1 < grid.nodesZ; z++) {
        for (int y = 0; y + 1 < grid.nodesY; y++) {
            for (int x = 0; x + 1 < grid.nodesX; x++) {
                Corner cube[8];
                bool anyInside = false;
                bool anyOutside = false;
                for (int c = 0; c < 8; c++) {
                    int cx = x + CORNER_OFFSET[c][0];
                    int cy = y + CORNER_OFFSET[c][1];
                    int cz = z + CORNER_OFFSET[c][2];
                    size_t node = grid.index(cx, cy, cz);
                    cube[c].distance = grid.distance[node];
                    anyInside = anyInside || (cube[c].distance < 0.0);
                    anyOutside = anyOutside || (cube[c].distance >= 0.0);
                }
                if (!anyInside || !anyOutside) {
                    continue;  // no sign change, so no surface in this cell
                }
                for (int c = 0; c < 8; c++) {
                    int cx = x + CORNER_OFFSET[c][0];
                    int cy = y + CORNER_OFFSET[c][1];
                    int cz = z + CORNER_OFFSET[c][2];
                    size_t node = grid.index(cx, cy, cz);
                    cube[c].position = grid.position(cx, cy, cz);
                    cube[c].normal = grid.gradient(cx, cy, cz);
                    if (cube[c].normal.lengthsq() > 0.0) {
                        cube[c].normal.normalize();
                    }
                    int owner = grid.owner[node];
                    cube[c].color = (owner >= 0) ? polyhedra[owner].color : polyhedra[0].color;
                }
                for (int t = 0; t < 6; t++) {
                    addTetrahedronSurface(cube, TETRAHEDRA[t], mesh);
                }
            }
        }
    }
}

}  // namespace

void buildMergedPolyhedra(const std::vector<PolyhedronField> &polyhedra, double mergeRadius,
                          double gridStep, chai3d::cMesh *mesh) {
    mesh->clear();
    if (!(polyhedra.empty() || gridStep <= 0.0)) {
        // The blend pushes the surface out past the hulls by a quarter of the merge radius, so the
        // grid and the per-polyhedron sampling boxes have to follow it that far out or the joins 
        // get clipped flat.
        double reach = std::max(mergeRadius, 0.0) * 0.25 + gridStep;

        SampleGrid grid = layOutGrid(polyhedra, reach, gridStep);
        sampleGrid(polyhedra, reach, std::max(mergeRadius, 0.0), grid);
        extractSurface(grid, polyhedra, mesh);
    }
}
