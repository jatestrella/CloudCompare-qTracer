#include "OutcropArea.h"

//CCCoreLib
#include <DgmOctree.h>
#include <DistanceComputationTools.h>
#include <Jacobi.h>
#include <Neighbourhood.h>
#include <ReferenceCloud.h>
#include <SquareMatrix.h>

//qCC_db
#include <ccKdTree.h>
#include <ccMesh.h>

//system
#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

using namespace CCCoreLib;


namespace
{

struct Context
{
	unsigned minPointsPerCell = 0;
	double   minPlanarity     = 0.0;

	double   totalArea           = 0.0;
	unsigned validCells          = 0;
	unsigned rejectedByCount     = 0;
	unsigned rejectedByPlanarity = 0;
	unsigned rejectedDegenerate  = 0;
	unsigned totalCells          = 0;

	ccPointCloud* meshVertices = nullptr; // non-null when we're building an output mesh
	ccMesh*       mesh         = nullptr;
};


//! Intersection polygon of a plane with an axis-aligned cube, returned in CCW
//! order (relative to \p planeNormal). Empty if the plane misses the cube.
static std::vector<CCVector3> planeCubeIntersection(
	const CCVector3& planePoint,
	const CCVector3& planeNormal,
	const CCVector3& cubeMin,
	const CCVector3& cubeMax)
{
	const CCVector3 v[8] = {
		{cubeMin.x, cubeMin.y, cubeMin.z},
		{cubeMax.x, cubeMin.y, cubeMin.z},
		{cubeMax.x, cubeMax.y, cubeMin.z},
		{cubeMin.x, cubeMax.y, cubeMin.z},
		{cubeMin.x, cubeMin.y, cubeMax.z},
		{cubeMax.x, cubeMin.y, cubeMax.z},
		{cubeMax.x, cubeMax.y, cubeMax.z},
		{cubeMin.x, cubeMax.y, cubeMax.z},
	};
	static constexpr int edges[12][2] = {
		{0,1},{1,2},{2,3},{3,0},
		{4,5},{5,6},{6,7},{7,4},
		{0,4},{1,5},{2,6},{3,7}
	};

	std::vector<CCVector3> pts;
	pts.reserve(6);

	const double eps = 1e-9;
	auto signedDist = [&](const CCVector3& p) -> double
	{
		return static_cast<double>((p - planePoint).dot(planeNormal));
	};

	for (int e = 0; e < 12; ++e)
	{
		const CCVector3& a = v[edges[e][0]];
		const CCVector3& b = v[edges[e][1]];
		const double da = signedDist(a);
		const double db = signedDist(b);

		CCVector3 hit;
		if (std::abs(da) < eps && std::abs(db) < eps)
		{
			// edge entirely on the plane — skip to avoid duplicate vertices
			continue;
		}
		else if (std::abs(da) < eps)
		{
			hit = a;
		}
		else if (std::abs(db) < eps)
		{
			hit = b;
		}
		else if (da * db < 0.0)
		{
			const double t = da / (da - db);
			hit = a + (b - a) * static_cast<PointCoordinateType>(t);
		}
		else
		{
			continue; // same side, no crossing
		}

		// dedupe (corner cases where two adjacent edges meet at a vertex)
		bool dup = false;
		for (const auto& q : pts)
		{
			if ((q - hit).norm() < 1e-7) { dup = true; break; }
		}
		if (!dup) pts.push_back(hit);
	}

	if (pts.size() < 3) return {};

	// Order points CCW around their centroid within the plane.
	CCVector3 c(0, 0, 0);
	for (const auto& p : pts) c += p;
	c /= static_cast<PointCoordinateType>(pts.size());

	// 2D basis on the plane
	CCVector3 ref = (std::abs(planeNormal.x) < static_cast<PointCoordinateType>(0.9))
		? CCVector3(1, 0, 0) : CCVector3(0, 1, 0);
	CCVector3 u = planeNormal.cross(ref);
	const PointCoordinateType uLen = u.norm();
	if (uLen < static_cast<PointCoordinateType>(1e-9)) return {};
	u /= uLen;
	CCVector3 vB = planeNormal.cross(u); // unit since normal and u are orthonormal

	std::sort(pts.begin(), pts.end(),
		[&c, &u, &vB](const CCVector3& A, const CCVector3& B)
		{
			const double angA = std::atan2(static_cast<double>((A - c).dot(vB)),
			                               static_cast<double>((A - c).dot(u)));
			const double angB = std::atan2(static_cast<double>((B - c).dot(vB)),
			                               static_cast<double>((B - c).dot(u)));
			return angA < angB;
		});

	return pts;
}


//! Fan-triangulation area of a (convex) 3D polygon.
static double convexPolygonArea(const std::vector<CCVector3>& poly)
{
	if (poly.size() < 3) return 0.0;
	double total = 0.0;
	const CCVector3& p0 = poly[0];
	for (size_t i = 1; i + 1 < poly.size(); ++i)
	{
		const CCVector3 a = poly[i]     - p0;
		const CCVector3 b = poly[i + 1] - p0;
		total += 0.5 * static_cast<double>(a.cross(b).norm());
	}
	return total;
}


//! Octree cell callback: fit a plane, intersect with the cube, accumulate area.
static bool processCell(const DgmOctree::octreeCell& cell,
                        void** additionalParameters,
                        NormalizedProgress* nProgress)
{
	Context& ctx = *static_cast<Context*>(additionalParameters[0]);
	++ctx.totalCells;

	const unsigned n = cell.points->size();
	if (n < ctx.minPointsPerCell)
	{
		++ctx.rejectedByCount;
		if (nProgress && !nProgress->oneStep()) return false;
		return true;
	}

	// PCA on the cell's points
	Neighbourhood nb(cell.points);
	SquareMatrixd cov = nb.computeCovarianceMatrix();
	SquareMatrixd eigVectors;
	std::vector<double> eigValues;
	if (!Jacobi<double>::ComputeEigenValuesAndVectors(cov, eigVectors, eigValues, true))
	{
		++ctx.rejectedByPlanarity;
		if (nProgress && !nProgress->oneStep()) return false;
		return true;
	}
	Jacobi<double>::SortEigenValuesAndVectors(eigVectors, eigValues);

	const double l1 = eigValues[0];
	const double l2 = eigValues[1];
	const double l3 = eigValues[2];
	const double planarity = (l1 > std::numeric_limits<double>::epsilon())
		? (l2 - l3) / l1 : 0.0;
	if (planarity < ctx.minPlanarity)
	{
		++ctx.rejectedByPlanarity;
		if (nProgress && !nProgress->oneStep()) return false;
		return true;
	}

	// Normal = eigenvector of smallest eigenvalue (index 2 after sort)
	CCVector3d normalD;
	Jacobi<double>::GetEigenVector(eigVectors, 2, normalD.u);
	normalD.normalize();
	const CCVector3 normal(
		static_cast<PointCoordinateType>(normalD.x),
		static_cast<PointCoordinateType>(normalD.y),
		static_cast<PointCoordinateType>(normalD.z));

	// Plane passes through the cell points' centroid
	CCVector3 centroid(0, 0, 0);
	for (unsigned i = 0; i < n; ++i)
	{
		CCVector3 p;
		cell.points->getPoint(i, p);
		centroid += p;
	}
	centroid /= static_cast<PointCoordinateType>(n);

	// Cube bounds for this cell
	CCVector3 cellCenter;
	cell.parentOctree->computeCellCenter(cell.truncatedCode, cell.level, cellCenter, true);
	const PointCoordinateType cs = cell.parentOctree->getCellSize(cell.level);
	const PointCoordinateType h  = cs * static_cast<PointCoordinateType>(0.5);
	const CCVector3 cubeMin = cellCenter - CCVector3(h, h, h);
	const CCVector3 cubeMax = cellCenter + CCVector3(h, h, h);

	const std::vector<CCVector3> poly = planeCubeIntersection(centroid, normal, cubeMin, cubeMax);
	if (poly.size() < 3)
	{
		++ctx.rejectedDegenerate;
		if (nProgress && !nProgress->oneStep()) return false;
		return true;
	}

	const double area = convexPolygonArea(poly);
	ctx.totalArea += area;
	++ctx.validCells;

	// Optionally add the polygon (fan-triangulated) to the output mesh
	if (ctx.mesh && ctx.meshVertices)
	{
		const unsigned base = ctx.meshVertices->size();
		for (const auto& p : poly)
			ctx.meshVertices->addPoint(p);
		for (size_t i = 1; i + 1 < poly.size(); ++i)
		{
			ctx.mesh->addTriangle(
				base,
				base + static_cast<unsigned>(i),
				base + static_cast<unsigned>(i + 1));
		}
	}

	if (nProgress && !nProgress->oneStep()) return false;
	return true;
}


//! Pick the octree level whose cell side is closest to the requested size.
static unsigned char pickLevelForCellSize(const DgmOctree& octree, double desired)
{
	unsigned char best = 1;
	double bestDiff = std::numeric_limits<double>::max();
	for (unsigned char lv = 1; lv <= DgmOctree::MAX_OCTREE_LEVEL; ++lv)
	{
		const double cs = static_cast<double>(octree.getCellSize(lv));
		const double diff = std::abs(cs - desired);
		if (diff < bestDiff)
		{
			bestDiff = diff;
			best = lv;
		}
	}
	return best;
}

} // anonymous namespace


namespace
{

//! Ensure the result has a mesh+vertices pair if the caller requested one.
void ensurePatchMesh(Context& ctx, bool wantMesh)
{
	if (!wantMesh) return;
	ctx.meshVertices = new ccPointCloud("area patch vertices");
	ctx.mesh         = new ccMesh(ctx.meshVertices);
	ctx.mesh->addChild(ctx.meshVertices);
	ctx.mesh->setName("Outcrop Area Patches");
}

//! Copy the shared counters from Context into the public Result struct.
void fillResultFromContext(const Context& ctx, OutcropArea::Result& result)
{
	result.totalArea           = ctx.totalArea;
	result.validCells          = ctx.validCells;
	result.rejectedByCount     = ctx.rejectedByCount;
	result.rejectedByPlanarity = ctx.rejectedByPlanarity;
	result.rejectedDegenerate  = ctx.rejectedDegenerate;
	result.totalCellsAtLevel   = ctx.totalCells;
	result.mesh                = ctx.mesh;
	result.meshVertices        = ctx.meshVertices;
}

//! Fan-triangulate a polygon into the accumulated patch mesh.
void appendPolyToMesh(Context& ctx, const std::vector<CCVector3>& poly)
{
	if (!(ctx.mesh && ctx.meshVertices)) return;
	const unsigned base = ctx.meshVertices->size();
	for (const auto& p : poly)
		ctx.meshVertices->addPoint(p);
	for (size_t i = 1; i + 1 < poly.size(); ++i)
	{
		ctx.mesh->addTriangle(
			base,
			base + static_cast<unsigned>(i),
			base + static_cast<unsigned>(i + 1));
	}
}

//! Octree path.
bool computeOctree(ccPointCloud* cloud,
                   const OutcropArea::Params& p,
                   OutcropArea::Result& result,
                   GenericProgressCallback* progressCb)
{
	if (p.cellSize <= 0) return false;

	DgmOctree octree(cloud);
	if (octree.build(progressCb) < 1)
		return false;

	const unsigned char level = pickLevelForCellSize(octree, p.cellSize);
	result.octreeLevel    = level;
	result.actualCellSize = static_cast<double>(octree.getCellSize(level));

	Context ctx;
	ctx.minPointsPerCell = p.minPointsPerCell;
	ctx.minPlanarity     = p.minPlanarity;
	ensurePatchMesh(ctx, p.buildPatchMesh);

	void* params[1] = { &ctx };
	if (octree.executeFunctionForAllCellsAtLevel(level,
	                                             &processCell,
	                                             params,
	                                             /*multiThread*/ false,
	                                             progressCb,
	                                             "Outcrop area (octree)") == 0)
	{
		delete ctx.mesh; // mesh owns meshVertices as a child
		return false;
	}

	fillResultFromContext(ctx, result);
	return true;
}

//! Kd-tree path — per-leaf planar fit is already stored, "cell" = leaf's AABB.
bool computeKdTree(ccPointCloud* cloud,
                   const OutcropArea::Params& p,
                   OutcropArea::Result& result,
                   GenericProgressCallback* progressCb)
{
	if (p.maxError <= 0) return false;
	if (progressCb)
	{
		progressCb->setMethodTitle("Outcrop area (kd-tree)");
		progressCb->setInfo("Building kd-tree…");
		progressCb->start();
	}

	ccKdTree kdt(cloud);
	const unsigned minPts = std::max<unsigned>(p.minPointsPerLeaf, 3);
	if (!kdt.build(p.maxError,
	               DistanceComputationTools::RMS,
	               minPts,
	               /*maxPointCountPerCell*/ 0,
	               progressCb))
	{
		if (progressCb) progressCb->stop();
		return false;
	}

	TrueKdTree::LeafVector leaves;
	if (!kdt.getLeaves(leaves))
	{
		if (progressCb) progressCb->stop();
		return false;
	}

	Context ctx;
	ctx.minPointsPerCell = p.minPointsPerLeaf;
	ctx.minPlanarity     = 0.0; // not used on kd-tree path
	ensurePatchMesh(ctx, p.buildPatchMesh);

	if (progressCb)
	{
		progressCb->setInfo(QString("Processing %1 leaves…").arg(leaves.size()).toUtf8().constData());
	}
	NormalizedProgress nProgress(progressCb, std::max<unsigned>(static_cast<unsigned>(leaves.size()), 1));

	double maxLeafError = 0.0;

	for (TrueKdTree::Leaf* leaf : leaves)
	{
		++ctx.totalCells;
		if (!leaf || !leaf->points)
		{
			nProgress.oneStep();
			continue;
		}
		const unsigned n = leaf->points->size();
		if (n < p.minPointsPerLeaf)
		{
			++ctx.rejectedByCount;
			nProgress.oneStep();
			continue;
		}

		// Plane normal from the leaf's pre-computed equation
		CCVector3 normal(leaf->planeEq[0], leaf->planeEq[1], leaf->planeEq[2]);
		const PointCoordinateType nLen = normal.norm();
		if (nLen < static_cast<PointCoordinateType>(1e-12))
		{
			++ctx.rejectedByCount;
			nProgress.oneStep();
			continue;
		}
		normal /= nLen;

		// A point on the plane: the points' centroid (LS plane passes through it).
		CCVector3 centroid(0, 0, 0);
		CCVector3 bbMin( std::numeric_limits<PointCoordinateType>::max(),
		                 std::numeric_limits<PointCoordinateType>::max(),
		                 std::numeric_limits<PointCoordinateType>::max());
		CCVector3 bbMax(-std::numeric_limits<PointCoordinateType>::max(),
		                -std::numeric_limits<PointCoordinateType>::max(),
		                -std::numeric_limits<PointCoordinateType>::max());
		for (unsigned i = 0; i < n; ++i)
		{
			CCVector3 pt;
			leaf->points->getPoint(i, pt);
			centroid += pt;
			bbMin.x = std::min(bbMin.x, pt.x);
			bbMin.y = std::min(bbMin.y, pt.y);
			bbMin.z = std::min(bbMin.z, pt.z);
			bbMax.x = std::max(bbMax.x, pt.x);
			bbMax.y = std::max(bbMax.y, pt.y);
			bbMax.z = std::max(bbMax.z, pt.z);
		}
		centroid /= static_cast<PointCoordinateType>(n);

		const std::vector<CCVector3> poly = planeCubeIntersection(centroid, normal, bbMin, bbMax);
		if (poly.size() < 3)
		{
			++ctx.rejectedDegenerate;
			nProgress.oneStep();
			continue;
		}

		const double area = convexPolygonArea(poly);
		ctx.totalArea += area;
		++ctx.validCells;
		if (leaf->error > maxLeafError)
			maxLeafError = static_cast<double>(leaf->error);

		appendPolyToMesh(ctx, poly);
		nProgress.oneStep();
	}

	if (progressCb) progressCb->stop();

	fillResultFromContext(ctx, result);
	result.maxLeafError = maxLeafError;
	return true;
}

} // anonymous namespace


bool OutcropArea::Compute(ccPointCloud* cloud,
                          const Params& params,
                          Result& result,
                          GenericProgressCallback* progressCb)
{
	result = Result();
	if (!cloud || cloud->size() < 3)
		return false;

	switch (params.partitioner)
	{
		case Octree: return computeOctree(cloud, params, result, progressCb);
		case KdTree: return computeKdTree(cloud, params, result, progressCb);
	}
	return false;
}
