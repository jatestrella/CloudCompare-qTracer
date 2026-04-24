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
 *  but without any cell-merging / final-facet-fitting step.
 */
class OutcropArea
{
public:
	enum Partitioner
	{
		Octree = 0,
		KdTree = 1
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

		ccMesh*        mesh         = nullptr; //!< caller-owned when non-null
		ccPointCloud*  meshVertices = nullptr; //!< owned via mesh (vertices)
	};

	//! Compute the outcrop area using the selected partitioner.
	static bool Compute(ccPointCloud* cloud,
	                    const Params& params,
	                    Result& result,
	                    CCCoreLib::GenericProgressCallback* progressCb = nullptr);
};
