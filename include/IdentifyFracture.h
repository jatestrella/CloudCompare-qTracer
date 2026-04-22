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

	//! Run the custom DBSCAN-like clustering on a cloud that already has the Eigen SFs
	static int runDBSCAN(const CCCoreLib::DgmOctree* octree,
	                     CCCoreLib::GenericIndexedCloudPersist* cloud,
	                     const DBSCANParams& params);

	static int expandCluster(const CCCoreLib::DgmOctree* octree,
	                         CCCoreLib::GenericIndexedCloudPersist* cloud,
	                         int corePointIndex,
	                         int clusterID,
	                         const DBSCANParams& params);

	//! Build a group of ccFacet / traces from already-extracted connected components
	static ccHObject* createTraces(ccPointCloud* cloud,
	                               CCCoreLib::ReferenceCloudContainer& components,
	                               bool randomColors,
	                               bool& error);

	//! Merge close-and-collinear traces into combined traces
	static ccHObject* TraceClustering(const ccHObject* ccGroup,
	                                  double ConeRadius,
	                                  double TwoTraceDist,
	                                  double MinAngle);

	//! Reconstruct joint planes from pairs of near-intersecting traces
	static ccHObject* PlaneFitting(const ccHObject* ccGroup,
	                               double IntersectionLineDistance,
	                               double MinTraceLength,
	                               double MinIntersectionAngle,
	                               double MinCorrDist);

	static CCVector3d CalculateMaximumEigenVector(CCCoreLib::GenericIndexedCloudPersist* inputCloud);
	static CCVector3  GetPointMaxEigVecFromSF(CCCoreLib::GenericIndexedCloudPersist* cloud, int pointIndex);
	static bool       InCone(const CCVector3& A, const CCVector3& B, double radius, const CCVector3& P);
	static void       SetScalarValueToNOISE(const CCVector3& P, ScalarType& scalarValue);

protected:

	static bool ComputeParametersAtLevel(const CCCoreLib::DgmOctree::octreeCell& cell,
	                                     void** additionalParameters,
	                                     CCCoreLib::NormalizedProgress* nProgress = nullptr);

	static ScalarType GetCloudLinearity(CCCoreLib::ReferenceCloud* cloud);
};
