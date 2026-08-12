#pragma once

//CCCoreLib
#include <GenericProgressCallback.h>

//qCC_db
#include <ccHObject.h>
#include <ccPointCloud.h>

class ccMesh;

//! Outcrop surface area via local plane fits on spatial cells.
/** Two partitioners are available:
 *   - **Octree** — fixed-size cubic cells at a chosen level. Per-cell PCA +
 *     planarity threshold filter the valid cells; each valid cell contributes
 *     the area of the fitted-plane / cube intersection polygon.
 *   - **Kd-tree** — CC's `ccKdTree` splits recursively until each leaf's planar
 *     fit residual is below a max-error threshold. Plane equation and point set
 *     are already stored on each leaf; the "cell" is then the leaf points' AABB.
 *     No separate planarity filter is needed (it's built into the split).
 *
 *  In both cases the per-cell area is the intersection polygon of the local
 *  plane with the cell's axis-aligned box. Mirrors qFacets' partitioning idea
 *  but without any cell-merging / final-facet-fitting step. Both accumulate a
 *  *3D (rugose) surface area*, which is resolution-dependent for rough surfaces.
 *
 *   - **Projected** — fits a single global best-fit plane (PCA), projects all
 *     points onto it, and measures the footprint area by 2D grid occupancy
 *     (count of occupied cells x cell^2, an implicit alpha-shape that excludes
 *     concavities/holes). This yields a *convergent, planar sampling-window
 *     area* — the appropriate denominator for the P21 areal fracture intensity
 *     on a quasi-planar outcrop. A convex-hull area is also reported as an
 *     upper-bound reference.
 */
class OutcropArea
{
public:
	enum Partitioner
	{
		Octree    = 0,
		KdTree    = 1,
		Projected = 2  //!< single best-fit plane + 2D grid-occupancy footprint
	};

	struct Params
	{
		Partitioner partitioner = Octree;

		// Octree-specific
		double   cellSize         = 0.1;
		unsigned minPointsPerCell = 10;
		double   minPlanarity     = 0.3;

		// Kd-tree-specific
		double   maxError         = 0.01; //!< max planar-fit residual (RMS) per leaf
		unsigned minPointsPerLeaf = 10;

		// Projected-specific (best-fit plane + 2D grid occupancy)
		double   projGridSize     = 0.02; //!< in-plane grid cell side for the occupancy count

		// Shared
		bool     buildPatchMesh   = true;
	};

	struct Result
	{
		double   totalArea           = 0.0;
		unsigned validCells          = 0;
		unsigned rejectedByCount     = 0;
		unsigned rejectedByPlanarity = 0; //!< (octree-only) low planarity
		unsigned rejectedDegenerate  = 0; //!< plane-box intersection produced <3 points
		unsigned totalCellsAtLevel   = 0; //!< (octree) total cells at level; (kd-tree) total leaves

		// Octree-only
		double   actualCellSize      = 0.0;
		unsigned char octreeLevel    = 0;

		// Kd-tree-only
		double   maxLeafError        = 0.0;

		// Projected-only (best-fit plane + 2D occupancy)
		double   convexHullArea      = 0.0; //!< in-plane convex hull area (reference / upper bound)
		double   planeDip            = 0.0; //!< best-fit plane dip (deg)
		double   planeDipDir         = 0.0; //!< best-fit plane dip direction (deg)
		double   planeRMS            = 0.0; //!< RMS of point residuals off the plane (roughness)
		double   inPlaneSizeU        = 0.0; //!< extent along the plane's u axis
		double   inPlaneSizeV        = 0.0; //!< extent along the plane's v axis

		ccMesh*        mesh         = nullptr; //!< caller-owned when non-null
		ccPointCloud*  meshVertices = nullptr; //!< owned via mesh (vertices)
	};

	//! Compute the outcrop area using the selected partitioner.
	static bool Compute(ccPointCloud* cloud,
	                    const Params& params,
	                    Result& result,
	                    CCCoreLib::GenericProgressCallback* progressCb = nullptr);
};
