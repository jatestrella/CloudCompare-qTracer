// SPDX-License-Identifier: LGPL-2.0-or-later
// Copyright © EDF R&D / TELECOM ParisTech (ENST-TSI)

#pragma once

//CCCoreLib
#include <DgmOctree.h>
#include <AutoSegmentationTools.h>
#include <DistanceComputationTools.h>

//qCC_db
#include <ccHObject.h>
#include <ccPointCloud.h>

//System
#include <cmath>
#include <vector>

#define UNCLASSIFIED -1
#define CORE_POINT 1
#define BORDER_POINT 2
#define NOISE -2
#define dbscan_SUCCESS 0
#define FAILURE -3


class IdentifyFracture
{
public:
	IdentifyFracture() = default;

	enum ErrorCode
	{
		NoError = 0,
		InvalidInput = -1,
		NotEnoughPoints = -2,
		OctreeComputationFailed = -3,
		ProcessFailed = -4,
		UnhandledCharacteristic = -5,
		NotEnoughMemory = -6,
		ProcessCancelledByUser = -7
	};

	enum CaseName
	{
		Linearity = 1,
		MaxEigVec_X = 2,
		MaxEigVec_Y = 3,
		MaxEigVec_Z = 4,
	};

	//! DBSCAN clustering parameters
	struct DBSCANParams
	{
		PointCoordinateType radius           = static_cast<PointCoordinateType>(0.03);
		unsigned int        minPoints        = 50;
		double              maxAngleDeg      = 15.0;
		double              linearityThresh  = 0.9;
		int                 searchType       = 1; //!< 1 = cylinder, 2 = sphere
	};

	//! Compute Eigen-based geometric features (PC Linearity + MaxEigVec X/Y/Z scalar fields)
	static ErrorCode ComputeEigen(
		CCCoreLib::GenericIndexedCloudPersist* cloud,
		PointCoordinateType kernelRadius,
		CCCoreLib::GenericProgressCallback* progressCb = nullptr,
		CCCoreLib::DgmOctree* inputOctree = nullptr);

	//! Per-point auto-scale variant of ComputeEigen.
	/** For every point, evaluates the PCA linearity L = (λ1-λ2)/λ1 at \p steps kernel
	 *  radii evenly spaced in [\p rMin, \p rMax], and keeps the radius that MAXIMISES L
	 *  (only scales whose neighbourhood holds at least \p minNeighbours points count —
	 *  this guards against the small-scale noise bias of pure linearity maximisation).
	 *  Fills the same `PC Linearity` + `MaxEigVec_X/Y/Z` scalar fields as ComputeEigen
	 *  (so stages 2+ are unchanged) plus an `OptScale` field holding the selected radius.
	 *  The fixed-radius ComputeEigen path is left untouched. */
	static ErrorCode ComputeEigenAutoScale(
		CCCoreLib::GenericIndexedCloudPersist* cloud,
		double rMin,
		double rMax,
		unsigned steps,
		unsigned minNeighbours,
		int scaleCriterion,   //!< 0 = max linearity, 1 = min eigenentropy
		CCCoreLib::GenericProgressCallback* progressCb = nullptr,
		CCCoreLib::DgmOctree* inputOctree = nullptr);

	//! Run the custom DBSCAN-like clustering on a cloud that already has the Eigen SFs
	static int runDBSCAN(const CCCoreLib::DgmOctree* octree,
	                     CCCoreLib::GenericIndexedCloudPersist* cloud,
	                     const DBSCANParams& params,
	                     CCCoreLib::GenericProgressCallback* progressCb = nullptr);

	static int expandCluster(const CCCoreLib::DgmOctree* octree,
	                         CCCoreLib::GenericIndexedCloudPersist* cloud,
	                         int corePointIndex,
	                         int clusterID,
	                         const DBSCANParams& params);

	//! Build a group of ccFacet / traces from already-extracted connected components
	static ccHObject* createTraces(ccPointCloud* cloud,
	                               CCCoreLib::ReferenceCloudContainer& components,
	                               bool randomColors,
	                               bool& error,
	                               CCCoreLib::GenericProgressCallback* progressCb = nullptr);

	//! Merge close-and-collinear traces into combined traces.
	/** Runs the single-pass clustering up to \p maxPasses times, feeding each pass's
	 *  output back as input (a merged trace may reach further neighbours after the
	 *  first round). Stops early when the trace count no longer decreases. */
	static ccHObject* TraceClustering(const ccHObject* ccGroup,
	                                  double ConeRadius,
	                                  double TwoTraceDist,
	                                  double MinAngle,
	                                  unsigned maxPasses = 1,
	                                  CCCoreLib::GenericProgressCallback* progressCb = nullptr);

	//! Reconstruct joint planes from pairs of near-intersecting traces.
	/** For each candidate pair of combined traces (filtered by the geometric tests
	 *  \p IntersectionLineDistance, \p MinTraceLength, \p MinIntersectionAngle,
	 *  \p MinCorrDist), pool all member-trace endpoints from both combined traces
	 *  and fit a `ccFacet` on the pooled cloud — much more robust than fitting on
	 *  just the 4 combined-trace endpoints.
	 */
	static ccHObject* PlaneFitting(const ccHObject* ccGroup,
	                               double IntersectionLineDistance,
	                               double MinTraceLength,
	                               double MinIntersectionAngle,
	                               double MinCorrDist,
	                               CCCoreLib::GenericProgressCallback* progressCb = nullptr);

	//! Merge near-coplanar joint planes using a sequential-RANSAC scheme.
	/** Each pass picks the unconsumed seed plane that has the largest "consensus"
	 *  set — peers whose normals are within \p maxNormalAngleDeg and whose centers
	 *  are within \p maxPlaneDist of the seed's plane. The consensus group is then
	 *  re-fit into a single `ccFacet` using the union of all member origin points,
	 *  and removed from the pool. The pass repeats until no planes remain.
	 *
	 *  \p maxPasses controls how many times the whole procedure is re-run on the
	 *  previous pass's output. Because merging changes plane centers/normals, new
	 *  coplanar opportunities can emerge. Iteration stops early once the plane
	 *  count stops decreasing. Intermediate groups are deleted; the returned
	 *  group is owned by the caller.
	 */
	static ccHObject* MergeCoplanarPlanes(const ccHObject* planesGroup,
	                                      double maxNormalAngleDeg,
	                                      double maxPlaneDist,
	                                      double maxCentroidDist,   //!< max centroid-to-centroid distance (0 = no limit)
	                                      unsigned maxPasses = 1,
	                                      bool dropUnmerged = false,
	                                      CCCoreLib::GenericProgressCallback* progressCb = nullptr);

private:
	//! Single pass of trace clustering (internal helper).
	static ccHObject* TraceClusteringOnce(const ccHObject* ccGroup,
	                                      double ConeRadius,
	                                      double TwoTraceDist,
	                                      double MinAngle,
	                                      CCCoreLib::GenericProgressCallback* progressCb);

	//! Single pass of coplanar-plane merging (internal helper).
	static ccHObject* MergeCoplanarPlanesOnce(const ccHObject* planesGroup,
	                                          double maxNormalAngleDeg,
	                                          double maxPlaneDist,
	                                          double maxCentroidDist,
	                                          CCCoreLib::GenericProgressCallback* progressCb);

public:

	static CCVector3d CalculateMaximumEigenVector(CCCoreLib::GenericIndexedCloudPersist* inputCloud);
	static CCVector3  GetPointMaxEigVecFromSF(CCCoreLib::GenericIndexedCloudPersist* cloud, int pointIndex);
	static bool       InCone(const CCVector3& A, const CCVector3& B, double radius, const CCVector3& P);
	static void       SetScalarValueToNOISE(const CCVector3& P, ScalarType& scalarValue);

	//! Compute the two trace polyline endpoints for a cluster.
	/** Projects every point onto \p unitEigVec (about the centroid) and returns the
	 *  extreme projections as \p p0 (min) and \p p1 (max). This gives the actual
	 *  extent of the cluster along its principal axis — more accurate than the
	 *  original port's "bbox-diagonal length, bbox-center origin" approach, which
	 *  systematically over-estimated trace length whenever the principal axis was
	 *  not aligned with world axes.
	 */
	static void ComputeTraceEndpoints(const CCCoreLib::GenericIndexedCloud* cloud,
	                                  const CCVector3& unitEigVec,
	                                  CCVector3& p0, CCVector3& p1);

protected:

	static bool ComputeParametersAtLevel(const CCCoreLib::DgmOctree::octreeCell& cell,
	                                     void** additionalParameters,
	                                     CCCoreLib::NormalizedProgress* nProgress = nullptr);

};
