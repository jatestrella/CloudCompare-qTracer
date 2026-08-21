//local
#include "IdentifyFracture.h"
#include "facetsClassifier.h"

//CCCoreLib
#include <ReferenceCloud.h>
#include <DistanceComputationTools.h>
#include <GenericProgressCallback.h>
#include <DgmOctreeReferenceCloud.h>
#include <ScalarField.h>
#include <ScalarFieldTools.h>
#include <Neighbourhood.h>
#include <Jacobi.h>

//qCC_db
#include <ccHObjectCaster.h>
#include <ccKdTree.h>
#include <CCMath.h>
#include <ccScalarField.h>
#include <ccPointCloud.h>
#include <ccMesh.h>
#include <ccFacet.h>
#include <ccProgressDialog.h>
#include <ccPolyline.h>
#include <ccNormalVectors.h>
#include <ccOctree.h>

//system
#include <algorithm>
#include <limits>
#include <queue>
#include <vector>

//Qt
#include <QString>
#include <QVariant>

using namespace CCCoreLib;


void IdentifyFracture::ComputeTraceEndpoints(const GenericIndexedCloud* cloud,
                                             const CCVector3& unitEigVec,
                                             CCVector3& p0, CCVector3& p1)
{
	const unsigned nPts = cloud ? cloud->size() : 0;
	if (nPts == 0)
	{
		p0 = p1 = CCVector3(0, 0, 0);
		return;
	}

	// Centroid (in double for numerical stability)
	CCVector3d centroid(0, 0, 0);
	for (unsigned i = 0; i < nPts; ++i)
	{
		const CCVector3* p = cloud->getPoint(i);
		centroid.x += p->x;
		centroid.y += p->y;
		centroid.z += p->z;
	}
	centroid /= static_cast<double>(nPts);
	const CCVector3 centroidF(static_cast<PointCoordinateType>(centroid.x),
	                          static_cast<PointCoordinateType>(centroid.y),
	                          static_cast<PointCoordinateType>(centroid.z));

	double tMin = std::numeric_limits<double>::max();
	double tMax = -std::numeric_limits<double>::max();
	for (unsigned i = 0; i < nPts; ++i)
	{
		const CCVector3* p = cloud->getPoint(i);
		const double dx = static_cast<double>(p->x) - centroid.x;
		const double dy = static_cast<double>(p->y) - centroid.y;
		const double dz = static_cast<double>(p->z) - centroid.z;
		const double t  = dx * unitEigVec.x + dy * unitEigVec.y + dz * unitEigVec.z;
		if (t < tMin) tMin = t;
		if (t > tMax) tMax = t;
	}

	p0 = centroidF + unitEigVec * static_cast<PointCoordinateType>(tMin);
	p1 = centroidF + unitEigVec * static_cast<PointCoordinateType>(tMax);
}


CCVector3d IdentifyFracture::CalculateMaximumEigenVector(GenericIndexedCloudPersist* inputCloud)
{
	Neighbourhood PC(inputCloud);
	SquareMatrixd EigVectors;
	std::vector<double> eigValues;
	SquareMatrixd covarianceMatrix = PC.computeCovarianceMatrix();
	Jacobi<double>::ComputeEigenValuesAndVectors(covarianceMatrix, EigVectors, eigValues, true);
	Jacobi<double>::SortEigenValuesAndVectors(EigVectors, eigValues);

	CCVector3d outputleafvector;
	Jacobi<double>::GetEigenVector(EigVectors, 0, outputleafvector.u);
	return CCVector3d(outputleafvector.u);
}


IdentifyFracture::ErrorCode IdentifyFracture::ComputeEigen(
	GenericIndexedCloudPersist* cloud,
	PointCoordinateType kernelRadius,
	GenericProgressCallback* progressCb /*=nullptr*/,
	DgmOctree* inputOctree /*=nullptr*/)
{
	if (!cloud)
		return InvalidInput;

	unsigned numberOfPoints = cloud->size();

	if (numberOfPoints < 4)
		return NotEnoughPoints;

	DgmOctree* octree = inputOctree;
	if (!octree)
	{
		octree = new DgmOctree(cloud);
		if (octree->build(progressCb) < 1)
		{
			delete octree;
			return OctreeComputationFailed;
		}
	}

	// *****Linearity*****
	ccPointCloud* pc = static_cast<ccPointCloud*>(cloud);
	pc->addScalarField("PC Linearity");
	pc->setCurrentInScalarField(pc->getScalarFieldIndexByName("PC Linearity"));

	for (unsigned i = 0; i < numberOfPoints; ++i)
	{
		cloud->setPointScalarValue(i, static_cast<ScalarType>(-1));
	}

	unsigned char level = octree->findBestLevelForAGivenNeighbourhoodSizeExtraction(kernelRadius);

	int CaseName = 0;
	void* additionalParameters[] =
	{
		static_cast<void*>(&kernelRadius),
		static_cast<void*>(&CaseName),
	};

	std::string label;
	CaseName = IdentifyFracture::CaseName::Linearity;
	ErrorCode result = NoError;

	if (octree->executeFunctionForAllCellsAtLevel(level,
		&ComputeParametersAtLevel,
		additionalParameters,
		true,
		progressCb,
		label.c_str()) == 0)
	{
		result = ProcessFailed;
	}

	// *****MaxEigVecX*****
	pc->addScalarField("MaxEigVec_X");
	pc->setCurrentInScalarField(pc->getScalarFieldIndexByName("MaxEigVec_X"));
	CaseName = IdentifyFracture::CaseName::MaxEigVec_X;
	if (octree->executeFunctionForAllCellsAtLevel(level,
		&ComputeParametersAtLevel,
		additionalParameters,
		true,
		progressCb,
		label.c_str()) == 0)
	{
		result = ProcessFailed;
	}

	// *****MaxEigVecY*****
	pc->addScalarField("MaxEigVec_Y");
	pc->setCurrentInScalarField(pc->getScalarFieldIndexByName("MaxEigVec_Y"));
	CaseName = IdentifyFracture::CaseName::MaxEigVec_Y;
	if (octree->executeFunctionForAllCellsAtLevel(level,
		&ComputeParametersAtLevel,
		additionalParameters,
		true,
		progressCb,
		label.c_str()) == 0)
	{
		result = ProcessFailed;
	}

	// *****MaxEigVecZ*****
	pc->addScalarField("MaxEigVec_Z");
	pc->setCurrentInScalarField(pc->getScalarFieldIndexByName("MaxEigVec_Z"));
	CaseName = IdentifyFracture::CaseName::MaxEigVec_Z;
	if (octree->executeFunctionForAllCellsAtLevel(level,
		&ComputeParametersAtLevel,
		additionalParameters,
		true,
		progressCb,
		label.c_str()) == 0)
	{
		result = ProcessFailed;
	}

	if (octree && !inputOctree)
	{
		delete octree;
		octree = nullptr;
	}

	return result;
}


namespace
{
	//! Shared read-only parameters for the per-point auto-scale octree callback.
	struct AutoScaleParams
	{
		ccPointCloud*       pc      = nullptr;
		ScalarField*        linSF   = nullptr;
		ScalarField*        exSF    = nullptr;
		ScalarField*        eySF    = nullptr;
		ScalarField*        ezSF    = nullptr;
		ScalarField*        scaleSF = nullptr;
		std::vector<double> scales;            //!< candidate radii, ascending
		double              rMax      = 0.0;
		unsigned            minNeigh  = 10;
		int                 criterion = 1;     //!< 0 = max linearity, 1 = min eigenentropy
	};

	//! Octree cell callback: for each point, pick the radius (in AutoScaleParams::scales)
	//! that maximises PCA linearity, then write that scale's L / v1 / radius to the SFs.
	bool autoScaleCellFunc(const DgmOctree::octreeCell& cell,
	                       void** additionalParameters,
	                       NormalizedProgress* nProgress)
	{
		AutoScaleParams& ap = *static_cast<AutoScaleParams*>(additionalParameters[0]);

		DgmOctree::NearestNeighboursSearchStruct nNSS;
		nNSS.level = cell.level;
		cell.parentOctree->getCellPos(cell.truncatedCode, cell.level, nNSS.cellPos, true);
		cell.parentOctree->computeCellCenter(nNSS.cellPos, cell.level, nNSS.cellCenter);

		const unsigned np = cell.points->size();
		try
		{
			nNSS.pointsInNeighbourhood.resize(np);
		}
		catch (const std::bad_alloc&)
		{
			return false;
		}
		{
			DgmOctree::NeighboursSet::iterator it = nNSS.pointsInNeighbourhood.begin();
			for (unsigned i = 0; i < np; ++i, ++it)
			{
				it->point      = cell.points->getPointPersistentPtr(i);
				it->pointIndex = cell.points->getPointGlobalIndex(i);
			}
		}
		nNSS.alreadyVisitedNeighbourhoodSize = 1;

		ReferenceCloud rc(ap.pc);

		for (unsigned i = 0; i < np; ++i)
		{
			cell.points->getPoint(i, nNSS.queryPoint);
			const unsigned cnt = cell.parentOctree->findNeighborsInASphereStartingFromCell(
				nNSS, static_cast<PointCoordinateType>(ap.rMax), false);

			double    bestMetric = -std::numeric_limits<double>::max();
			double    bestL      = -1.0;
			double    bestScale  = -1.0;
			CCVector3 bestVec(0, 0, 1);

			for (double r : ap.scales)
			{
				const double r2 = r * r;
				rc.clear(false);
				for (unsigned j = 0; j < cnt; ++j)
				{
					if (nNSS.pointsInNeighbourhood[j].squareDistd <= r2)
						rc.addPointIndex(nNSS.pointsInNeighbourhood[j].pointIndex);
				}
				if (rc.size() < ap.minNeigh)
					continue;

				Neighbourhood nb(&rc);
				SquareMatrixd cov = nb.computeCovarianceMatrix();
				SquareMatrixd eV;
				std::vector<double> ev;
				if (!Jacobi<double>::ComputeEigenValuesAndVectors(cov, eV, ev, true))
					continue;
				Jacobi<double>::SortEigenValuesAndVectors(eV, ev);

				const double l1 = std::max(ev[0], 0.0);
				const double l2 = std::max(ev[1], 0.0);
				const double l3 = std::max(ev[2], 0.0);
				const double L  = (l1 > 1e-12) ? (l1 - l2) / l1 : 0.0;

				// Selection metric — larger is better. Linearity: maximise L.
				// Eigenentropy: minimise E = -Σ e_i ln(e_i)  =>  maximise -E.
				double metric;
				if (ap.criterion == 1)
				{
					const double s = l1 + l2 + l3;
					double E = 0.0;
					if (s > 1e-12)
					{
						for (double lv : {l1, l2, l3})
						{
							const double e = lv / s;
							if (e > 1e-12) E -= e * std::log(e);
						}
					}
					metric = -E;
				}
				else
				{
					metric = L;
				}

				if (metric > bestMetric)
				{
					bestMetric = metric;
					bestL      = L;
					bestScale  = r;
					CCVector3d vv;
					Jacobi<double>::GetEigenVector(eV, 0, vv.u);
					bestVec = CCVector3(static_cast<PointCoordinateType>(vv.x),
					                    static_cast<PointCoordinateType>(vv.y),
					                    static_cast<PointCoordinateType>(vv.z));
				}
			}

			const unsigned gi = cell.points->getPointGlobalIndex(i);
			ap.linSF->setValue(gi, static_cast<ScalarType>(bestL));
			ap.exSF ->setValue(gi, static_cast<ScalarType>(bestVec.x));
			ap.eySF ->setValue(gi, static_cast<ScalarType>(bestVec.y));
			ap.ezSF ->setValue(gi, static_cast<ScalarType>(bestVec.z));
			if (ap.scaleSF)
				ap.scaleSF->setValue(gi, static_cast<ScalarType>(bestScale));

			if (nProgress && !nProgress->oneStep())
				return false;
		}
		return true;
	}
} // anonymous namespace


IdentifyFracture::ErrorCode IdentifyFracture::ComputeEigenAutoScale(
	GenericIndexedCloudPersist* cloud,
	double rMin,
	double rMax,
	unsigned steps,
	unsigned minNeighbours,
	int scaleCriterion,
	GenericProgressCallback* progressCb /*=nullptr*/,
	DgmOctree* inputOctree /*=nullptr*/)
{
	if (!cloud)
		return InvalidInput;
	if (cloud->size() < 4)
		return NotEnoughPoints;
	if (rMin <= 0.0 || rMax < rMin || steps < 1)
		return InvalidInput;

	ccPointCloud* pc = static_cast<ccPointCloud*>(cloud);

	DgmOctree* octree = inputOctree;
	if (!octree)
	{
		octree = new DgmOctree(cloud);
		if (octree->build(progressCb) < 1)
		{
			delete octree;
			return OctreeComputationFailed;
		}
	}

	// (Re)create the output scalar fields. addScalarField refuses duplicates, so a
	// pre-existing field of the same name is reused (and overwritten) — same policy
	// as the fixed-radius ComputeEigen path.
	auto ensureSF = [&](const char* name) -> ScalarField*
	{
		int idx = pc->getScalarFieldIndexByName(name);
		if (idx < 0)
			idx = pc->addScalarField(name);
		return (idx >= 0) ? pc->getScalarField(idx) : nullptr;
	};
	ScalarField* linSF   = ensureSF("PC Linearity");
	ScalarField* exSF    = ensureSF("MaxEigVec_X");
	ScalarField* eySF    = ensureSF("MaxEigVec_Y");
	ScalarField* ezSF    = ensureSF("MaxEigVec_Z");
	ScalarField* scaleSF = ensureSF("OptScale");
	if (!linSF || !exSF || !eySF || !ezSF || !scaleSF)
	{
		if (octree && !inputOctree) delete octree;
		return ProcessFailed;
	}
	const unsigned nPts = cloud->size();
	for (unsigned i = 0; i < nPts; ++i)
	{
		linSF  ->setValue(i, static_cast<ScalarType>(-1));
		exSF   ->setValue(i, 0);
		eySF   ->setValue(i, 0);
		ezSF   ->setValue(i, static_cast<ScalarType>(1));
		scaleSF->setValue(i, static_cast<ScalarType>(-1));
	}

	AutoScaleParams ap;
	ap.pc       = pc;
	ap.linSF    = linSF;
	ap.exSF     = exSF;
	ap.eySF     = eySF;
	ap.ezSF     = ezSF;
	ap.scaleSF  = scaleSF;
	ap.rMax      = rMax;
	ap.minNeigh  = std::max<unsigned>(minNeighbours, 3);
	ap.criterion = scaleCriterion;
	ap.scales.reserve(steps);
	if (steps == 1)
	{
		ap.scales.push_back(rMin);
	}
	else
	{
		for (unsigned k = 0; k < steps; ++k)
			ap.scales.push_back(rMin + (rMax - rMin) * static_cast<double>(k) / static_cast<double>(steps - 1));
	}

	const unsigned char level = octree->findBestLevelForAGivenNeighbourhoodSizeExtraction(
		static_cast<PointCoordinateType>(rMax));

	void* params[1] = { &ap };
	ErrorCode result = NoError;
	if (octree->executeFunctionForAllCellsAtLevel(level,
		&autoScaleCellFunc,
		params,
		true,
		progressCb,
		"Eigenvector Computing (auto scale)") == 0)
	{
		result = ProcessFailed;
	}

	// Recompute min/max so the SFs display correctly.
	linSF->computeMinAndMax();
	scaleSF->computeMinAndMax();

	if (octree && !inputOctree)
	{
		delete octree;
		octree = nullptr;
	}

	return result;
}


bool IdentifyFracture::ComputeParametersAtLevel(const DgmOctree::octreeCell& cell,
	void** additionalParameters,
	NormalizedProgress* nProgress /*=nullptr*/)
{
	PointCoordinateType radius = *static_cast<PointCoordinateType*>(additionalParameters[0]);
	int CaseName = *static_cast<int*>(additionalParameters[1]);

	DgmOctree::NearestNeighboursSearchStruct nNSS;
	nNSS.level = cell.level;
	cell.parentOctree->getCellPos(cell.truncatedCode, cell.level, nNSS.cellPos, true);
	cell.parentOctree->computeCellCenter(nNSS.cellPos, cell.level, nNSS.cellCenter);

	unsigned n = cell.points->size();

	{
		try
		{
			nNSS.pointsInNeighbourhood.resize(n);
		}
		catch (const std::bad_alloc&)
		{
			return false;
		}

		DgmOctree::NeighboursSet::iterator it = nNSS.pointsInNeighbourhood.begin();
		for (unsigned i = 0; i < n; ++i, ++it)
		{
			it->point = cell.points->getPointPersistentPtr(i);
			it->pointIndex = cell.points->getPointGlobalIndex(i);
		}
	}
	nNSS.alreadyVisitedNeighbourhoodSize = 1;

	for (unsigned i = 0; i < n; ++i)
	{
		cell.points->getPoint(i, nNSS.queryPoint);

		unsigned neighborCount = cell.parentOctree->findNeighborsInASphereStartingFromCell(nNSS, radius, false);
		DgmOctreeReferenceCloud CloudAroundCorePoint(&nNSS.pointsInNeighbourhood, neighborCount);
		GenericIndexedCloudPersist* GICP_CloudAroundCorePoint = &CloudAroundCorePoint;

		Neighbourhood CellPointsNeighbourhood(GICP_CloudAroundCorePoint);

		ScalarType value = NAN_VALUE;
		CCVector3d CoreMaxEigenVector;

		switch (CaseName)
		{
			case 1: //Linearity
				value = static_cast<ScalarType>(CellPointsNeighbourhood.computeFeature(Neighbourhood::GeomFeature::Linearity));
				break;
			case 2: //MaxEigVecX
				CoreMaxEigenVector = IdentifyFracture::CalculateMaximumEigenVector(GICP_CloudAroundCorePoint);
				value = static_cast<ScalarType>(CoreMaxEigenVector.x);
				break;
			case 3: //MaxEigVecY
				CoreMaxEigenVector = IdentifyFracture::CalculateMaximumEigenVector(GICP_CloudAroundCorePoint);
				value = static_cast<ScalarType>(CoreMaxEigenVector.y);
				break;
			case 4: //MaxEigVecZ
				CoreMaxEigenVector = IdentifyFracture::CalculateMaximumEigenVector(GICP_CloudAroundCorePoint);
				value = static_cast<ScalarType>(CoreMaxEigenVector.z);
				break;
			default:
				break;
		}

		cell.points->setPointScalarValue(i, value);

		if (nProgress && !nProgress->oneStep())
		{
			return false;
		}
	}

	return true;
}


int IdentifyFracture::runDBSCAN(const DgmOctree* octree,
                                GenericIndexedCloudPersist* cloud,
                                const DBSCANParams& params,
                                GenericProgressCallback* progressCb /*=nullptr*/)
{
	ccPointCloud* pc = static_cast<ccPointCloud*>(cloud);
	pc->addScalarField("DBSCAN");
	pc->setCurrentScalarField(pc->getScalarFieldIndexByName("DBSCAN"));

	const unsigned total = pc->size();
	for (unsigned i = 0; i < total; ++i)
	{
		pc->setPointScalarValue(i, static_cast<ScalarType>(-1)); // UNCLASSIFIED
	}

	if (progressCb)
	{
		progressCb->setMethodTitle("Cylindrical DBSCAN Clustering");
		progressCb->setInfo(QString("Clustering %1 points…").arg(total).toUtf8().constData());
		progressCb->start();
	}
	NormalizedProgress nProgress(progressCb, std::max<unsigned>(total, 1));

	unsigned clusterID = 1;

	for (unsigned i = 0; i < total; ++i)
	{
		if (pc->getPointScalarValue(i) == -1)
		{
			if (IdentifyFracture::expandCluster(octree, cloud, i, clusterID, params) != FAILURE)
			{
				clusterID += 1;
			}
		}
		nProgress.oneStep();
	}

	if (progressCb)
		progressCb->stop();

	return 0;
}

void IdentifyFracture::SetScalarValueToNOISE(const CCVector3& /*P*/, ScalarType& scalarValue)
{
	scalarValue = -2; //NOISE
}


namespace
{
	//! BFS for the shallowest 2-vertex polyline under \p node and return its two
	//! endpoints. Structure-agnostic: works for `[trace pieces]` facets, `Merged
	//! trace` nodes, and bare filtered polylines alike — so the trace-consuming
	//! stages never depend on a hard-coded child index.
	bool traceEndpointsOf(const ccHObject* node, CCVector3& p, CCVector3& q)
	{
		if (!node) return false;
		std::queue<const ccHObject*> bfs;
		bfs.push(node);
		while (!bfs.empty())
		{
			const ccHObject* n = bfs.front();
			bfs.pop();
			if (n->isKindOf(CC_TYPES::POLY_LINE))
			{
				const ccPolyline* pl = static_cast<const ccPolyline*>(n);
				if (pl->size() == 2)
				{
					pl->getPoint(0, p);
					pl->getPoint(1, q);
					return true;
				}
			}
			for (unsigned i = 0; i < n->getChildrenNumber(); ++i)
				bfs.push(n->getChild(i));
		}
		return false;
	}
}


ccHObject* IdentifyFracture::TraceClustering(const ccHObject* ccGroup,
                                             double ConeRadius,
                                             double TwoTraceDist,
                                             double MinAngle,
                                             unsigned maxPasses /*=1*/,
                                             unsigned* passesPerformed /*=nullptr*/,
                                             GenericProgressCallback* progressCb /*=nullptr*/)
{
	if (passesPerformed)
		*passesPerformed = 0;

	if (!ccGroup || maxPasses == 0)
		return new ccHObject("Traces");

	ccHObject*       current = nullptr;                       // latest pass output (owned here)
	const ccHObject* feed    = ccGroup;                       // input for next pass (pass 1 = external, not owned)
	unsigned         prevCount = ccGroup->getChildrenNumber();
	unsigned         performed = 0;

	for (unsigned pass = 0; pass < maxPasses; ++pass)
	{
		if (progressCb)
			progressCb->setInfo(qUtf8Printable(QString("Trace clustering pass %1/%2…").arg(pass + 1).arg(maxPasses)));

		ccHObject* next = TraceClusteringOnce(feed, ConeRadius, TwoTraceDist, MinAngle, progressCb);
		const unsigned nextCount = next ? next->getChildrenNumber() : 0;
		++performed;

		// `feed` for pass >= 2 IS the previous `current`; TraceClusteringOnce has
		// finished reading it above, so deleting now is safe.
		if (current)
			delete current;
		current = next;
		feed    = next;

		if (nextCount >= prevCount || nextCount <= 1)
			break;   // converged (trace count no longer decreasing) or nothing left to merge
		prevCount = nextCount;
	}

	if (passesPerformed)
		*passesPerformed = performed;

	if (!current)
		current = new ccHObject("Traces");
	return current;
}

ccHObject* IdentifyFracture::TraceClusteringOnce(const ccHObject* ccGroup,
                                                 double ConeRadius,
                                                 double TwoTraceDist,
                                                 double MinAngle,
                                                 GenericProgressCallback* progressCb)
{
	const unsigned totalTraces = ccGroup->getChildrenNumber();

	// Build the working set ("victim"): a 2-point cloud per trace, copied out
	// of each facet's trace polyline. We **clone** the points (instead of
	// re-parenting the original `TipVertices`) so the source `[facets]` group
	// is left intact — otherwise re-running stage 4 on the same input crashes
	// because the original `TipVertices` is gone the second time around.
	ccHObject* victim = new ccHObject("victim");
	for (unsigned i = 0; i < totalTraces; i++)
	{
		// Find this child's trace endpoints robustly (BFS to its 2-vertex polyline),
		// so any trace group works: `[trace pieces]`, `Traces`, or a filtered group.
		CCVector3 p, q;
		if (!traceEndpointsOf(ccGroup->getChild(i), p, q))
			continue;

		ccPointCloud* clone = new ccPointCloud();
		if (!clone->reserve(2))
		{
			delete clone;
			continue;
		}
		clone->addPoint(p);
		clone->addPoint(q);
		clone->setName(QString("Trace piece %1").arg(i));
		victim->addChild(clone);
	}

	ccHObject* Traces = new ccHObject("Traces");

	if (progressCb)
	{
		progressCb->setMethodTitle("Trace Clustering");
		progressCb->setInfo(QString("Merging %1 traces…").arg(totalTraces).toUtf8().constData());
		progressCb->start();
	}
	NormalizedProgress nProgress(progressCb, std::max<unsigned>(totalTraces, 1));

	unsigned tracenumber = 0;

	while (victim->getFirstChild())
	{
		ccHObject* DieGroup = new ccHObject("Trace pieces");
		ccHObject* InfectedGroup = new ccHObject("InfestedGroup");

		InfectedGroup->addChild(victim->getFirstChild());
		victim->detachChild(victim->getFirstChild());

		QString TraceName = QString("Merged trace %1").arg(tracenumber);
		ccHObject* Trace = new ccHObject(TraceName);
		tracenumber++;

		while (InfectedGroup->getFirstChild())
		{
			ccGenericPointCloud* Trace_i = ccHObjectCaster::ToGenericPointCloud(InfectedGroup->getFirstChild(), nullptr);
			GenericIndexedCloud* IndexedTrace_i = static_cast<GenericIndexedCloud*>(Trace_i);
			CCVector3 Point_i1 = *(IndexedTrace_i->getPoint(0));
			CCVector3 Point_i2 = *(IndexedTrace_i->getPoint(1));

			const CCVector3 i1i2  = Point_i2 - Point_i1;
			const CCVector3 i1i2n = (Point_i2 - Point_i1) / (Point_i2 - Point_i1).norm();

			bool infected = true;

			while (infected != false)
			{
				infected = false;
				for (unsigned j = 0; j < victim->getChildrenNumber(); j++)
				{
					ccGenericPointCloud* Trace_j = ccHObjectCaster::ToGenericPointCloud(victim->getChild(j), nullptr);
					GenericIndexedCloud* IndexedTrace_j = static_cast<GenericIndexedCloud*>(Trace_j);
					CCVector3 Point_j1 = *(IndexedTrace_j->getPoint(0));
					CCVector3 Point_j2 = *(IndexedTrace_j->getPoint(1));
					double    L_j1j2   = (Point_j2 - Point_j1).norm();
					CCVector3 j1j2n    = (Point_j2 - Point_j1) / L_j1j2;

					double c_minCosAngle = cos(DegreesToRadians(MinAngle));
					if (std::abs(i1i2n.dot(j1j2n)) >= c_minCosAngle)
					{
						bool i1d_check = IdentifyFracture::InCone(Point_j1, Point_j2, ConeRadius, Point_i1);
						bool i2d_check = IdentifyFracture::InCone(Point_j1, Point_j2, ConeRadius, Point_i2);
						bool j1d_check = IdentifyFracture::InCone(Point_i1, Point_i2, ConeRadius, Point_j1);
						bool j2d_check = IdentifyFracture::InCone(Point_i1, Point_i2, ConeRadius, Point_j2);

						if ((i1d_check || i2d_check) && (j1d_check || j2d_check))
						{
							bool dist_check = false;
							for (unsigned k = 0; k < InfectedGroup->getChildrenNumber(); k++)
							{
								ccGenericPointCloud* CurrentInfectedTrace = ccHObjectCaster::ToGenericPointCloud(InfectedGroup->getChild(k), nullptr);
								GenericIndexedCloud* CurrentIndexedInfectedTrace = static_cast<GenericIndexedCloud*>(CurrentInfectedTrace);
								CCVector3 CurrentIndexedInfected_p1 = *(CurrentIndexedInfectedTrace->getPoint(0));
								CCVector3 CurrentIndexedInfected_p2 = *(CurrentIndexedInfectedTrace->getPoint(1));

								double dist_p1j1 = (CurrentIndexedInfected_p1 - Point_j1).norm();
								double dist_p1j2 = (CurrentIndexedInfected_p1 - Point_j2).norm();
								double dist_p2j1 = (CurrentIndexedInfected_p2 - Point_j1).norm();
								double dist_p2j2 = (CurrentIndexedInfected_p2 - Point_j2).norm();

								if (dist_p1j1 <= TwoTraceDist || dist_p1j2 <= TwoTraceDist || dist_p2j1 <= TwoTraceDist || dist_p2j2 <= TwoTraceDist)
								{
									dist_check = true;
								}
							}
							if (dist_check)
							{
								InfectedGroup->addChild(victim->getChild(j));
								victim->detachChild(victim->getChild(j));

								j--;
								infected = true;
							}
						}
					}
				}
			}

			ccGenericPointCloud* drawDisperseTracePC = ccHObjectCaster::ToGenericPointCloud(InfectedGroup->getFirstChild(), nullptr);
			ccPolyline* DisperseTrace = new ccPolyline(drawDisperseTracePC);

			DisperseTrace->setTempColor(ccColor::yellow);
			DisperseTrace->set2DMode(false);
			DisperseTrace->addPointIndex(0, 2);
			InfectedGroup->getFirstChild()->addChild(DisperseTrace);
			InfectedGroup->getFirstChild()->setVisible(false);
			InfectedGroup->getFirstChild()->setEnabled(true);
			DieGroup->addChild(InfectedGroup->getFirstChild());
			InfectedGroup->detachChild(InfectedGroup->getFirstChild());
			nProgress.oneStep();
		}

		ccPointCloud* PC_die = new ccPointCloud();
		for (unsigned k = 0; k < DieGroup->getChildrenNumber(); k++)
		{
			ccGenericPointCloud* Point_die = ccHObjectCaster::ToGenericPointCloud(DieGroup->getChild(k), nullptr);
			GenericIndexedCloud* IndexedPoint_die = static_cast<GenericIndexedCloud*>(Point_die);
			CCVector3 Point_die1 = *(IndexedPoint_die->getPoint(0));
			CCVector3 Point_die2 = *(IndexedPoint_die->getPoint(1));

			PC_die->addPoint(Point_die1);
			PC_die->addPoint(Point_die2);
		}

		CCVector3 ExtractFracture_MaxEigVec = IdentifyFracture::CalculateMaximumEigenVector(PC_die).toFloat();
		CCVector3 p0, p1;
		IdentifyFracture::ComputeTraceEndpoints(PC_die, ExtractFracture_MaxEigVec, p0, p1);

		ccPointCloud* TipVertices = new ccPointCloud();
		ccPolyline* TracePoly = new ccPolyline(TipVertices);
		TracePoly->setName("MergedPolyline");
		TracePoly->addChild(TipVertices);

		TipVertices->reserve(2);
		TipVertices->addPoint(p0);
		TipVertices->addPoint(p1);

		TipVertices->setEnabled(false);

		TracePoly->setTempColor(ccColor::red);
		TracePoly->set2DMode(false);
		TracePoly->addPointIndex(0, 2);

		PC_die->setEnabled(false);
		TracePoly->addChild(PC_die);

		Trace->addChild(DieGroup);
		Trace->getChild(0)->setEnabled(false);

		Trace->addChild(TracePoly);
		Traces->addChild(Trace);
	}

	delete victim;

	if (progressCb)
		progressCb->stop();

	return Traces;
}


int IdentifyFracture::expandCluster(const DgmOctree* octree,
                                    GenericIndexedCloudPersist* cloud,
                                    int corePointIndex,
                                    int clusterID,
                                    const DBSCANParams& params)
{
	const PointCoordinateType radius               = params.radius;
	const unsigned int        m_minPoints          = params.minPoints;
	const double              maxAngle_deg         = params.maxAngleDeg;
	const double              linearityThreshold   = params.linearityThresh;
	const int                 SearchType           = params.searchType;

	ccPointCloud* pc = static_cast<ccPointCloud*>(cloud);

	// Cache SF pointers once so the inner loops can read/write directly without
	// repeated string-based getScalarFieldIndexByName / setCurrentScalarField calls.
	ScalarField* linearitySF = pc->getScalarField(pc->getScalarFieldIndexByName("PC Linearity"));
	ScalarField* eigXSF      = pc->getScalarField(pc->getScalarFieldIndexByName("MaxEigVec_X"));
	ScalarField* eigYSF      = pc->getScalarField(pc->getScalarFieldIndexByName("MaxEigVec_Y"));
	ScalarField* eigZSF      = pc->getScalarField(pc->getScalarFieldIndexByName("MaxEigVec_Z"));
	ScalarField* dbscanSF    = pc->getScalarField(pc->getScalarFieldIndexByName("DBSCAN"));

	auto maxEigVecAt = [eigXSF, eigYSF, eigZSF](unsigned idx) {
		return CCVector3(eigXSF->getValue(idx), eigYSF->getValue(idx), eigZSF->getValue(idx));
	};

	ScalarType Corelinearity = linearitySF->getValue(corePointIndex);
	CCVector3  CoreMaxEigVec = maxEigVecAt(corePointIndex);

	unsigned char level = octree->findBestLevelForAGivenNeighbourhoodSizeExtraction(radius);
	DgmOctree::NeighboursSet CoreCloud;
	CCVector3 vec;
	cloud->getPoint(corePointIndex, vec);
	if (SearchType == 1)
	{
		DgmOctree::CylindricalNeighbourhood cylParams;
		cylParams.center          = vec;
		cylParams.dir             = CoreMaxEigVec;
		cylParams.radius          = radius;
		cylParams.maxHalfLength   = 2 * radius;
		cylParams.neighbours      = CoreCloud;
		cylParams.level           = level;
		cylParams.onlyPositiveDir = false;

		octree->getPointsInCylindricalNeighbourhood(cylParams);
		CoreCloud = cylParams.neighbours;
	}
	else if (SearchType == 2)
	{
		octree->getPointsInSphericalNeighbourhood(vec, radius, CoreCloud, level);
	}

	DgmOctreeReferenceCloud CoreCloud_ref(&CoreCloud, 0);

	const double c_minCosNormAngle = cos(DegreesToRadians(maxAngle_deg));

	if (CoreCloud_ref.size() < m_minPoints || Corelinearity < linearityThreshold)
	{
		for (unsigned k = 0, n = CoreCloud_ref.size(); k < n; ++k)
			CoreCloud_ref.setPointScalarValue(k, static_cast<ScalarType>(-2 /*NOISE*/));
		return FAILURE;
	}

	int index = 0, indexCorePoint = 0;
	for (auto iterSeeds = CoreCloud.begin(); iterSeeds != CoreCloud.end(); ++iterSeeds)
	{
		dbscanSF->setValue(iterSeeds->pointIndex, static_cast<ScalarType>(clusterID));

		if (iterSeeds->point == cloud->getPoint(corePointIndex))
			indexCorePoint = index;
		++index;
	}
	CoreCloud.erase(CoreCloud.begin() + indexCorePoint);

	for (size_t i = 0, n = CoreCloud.size(); i < n; ++i)
	{
		DgmOctree::NeighboursSet NeighborsCloud;
		CCVector3 NeighborsMaxEigVec = maxEigVecAt(CoreCloud[i].pointIndex);
		size_t num_cluster = 0;

		if (SearchType == 1)
		{
			DgmOctree::CylindricalNeighbourhood paramsNeighbors;
			paramsNeighbors.center          = *CoreCloud[i].point;
			paramsNeighbors.dir             = NeighborsMaxEigVec;
			paramsNeighbors.radius          = radius;
			paramsNeighbors.maxHalfLength   = 2 * radius;
			paramsNeighbors.neighbours      = NeighborsCloud;
			paramsNeighbors.level           = level;
			paramsNeighbors.onlyPositiveDir = false;
			num_cluster = octree->getPointsInCylindricalNeighbourhood(paramsNeighbors);
			NeighborsCloud = paramsNeighbors.neighbours;
		}
		else if (SearchType == 2)
		{
			num_cluster = octree->getPointsInSphericalNeighbourhood(*CoreCloud[i].point, radius, NeighborsCloud, level);
		}

		if (num_cluster >= m_minPoints && std::abs(CoreMaxEigVec.dot(NeighborsMaxEigVec)) > c_minCosNormAngle)
		{
			for (auto iterNeighbors = NeighborsCloud.begin(); iterNeighbors != NeighborsCloud.end(); ++iterNeighbors)
			{
				const ScalarType v = dbscanSF->getValue(iterNeighbors->pointIndex);
				if (v == -1 || v == -2)
				{
					if (v == -1)
					{
						CoreCloud.push_back(*iterNeighbors);
						n = CoreCloud.size();
					}
					dbscanSF->setValue(iterNeighbors->pointIndex, static_cast<ScalarType>(clusterID));
				}
			}
		}
	}

	return dbscan_SUCCESS;
}


ccHObject* IdentifyFracture::createTraces(ccPointCloud* cloud,
                                          ReferenceCloudContainer& components,
                                          bool randomColors,
                                          bool& error,
                                          GenericProgressCallback* progressCb /*=nullptr*/)
{
	if (!cloud)
		return nullptr;

	ccHObject* ccGroup = new ccHObject(cloud->getName() + QString(" [trace pieces]"));
	ccGroup->setDisplay(cloud->getDisplay());
	ccGroup->setVisible(true);

	bool cloudHasNormal = cloud->hasNormals();

	const unsigned totalComponents = static_cast<unsigned>(components.size());
	if (progressCb)
	{
		progressCb->setMethodTitle("Lineation");
		progressCb->setInfo(QString("Building %1 trace pieces…").arg(totalComponents).toUtf8().constData());
		progressCb->start();
	}
	NormalizedProgress nProgress(progressCb, std::max<unsigned>(totalComponents, 1));

	error = false;
	while (!components.empty())
	{
		ReferenceCloud* compIndexes = components.back();
		components.pop_back();

		ccPointCloud* facetCloud = cloud->partialClone(compIndexes);
		if (!facetCloud)
		{
			error = true;
		}
		else if (facetCloud->size() < 3)
		{
			// A cluster with < 3 points cannot form a facet — skip it silently
			// (otherwise ccFacet::Create pops a "Need at least 3 points" error dialog).
			delete facetCloud;
		}
		else
		{
			ccFacet* facet = ccFacet::Create(facetCloud, 0, true);
			if (facet)
			{
				QString facetName = QString("Trace piece %1").arg(ccGroup->getChildrenNumber());
				facet->setName(facetName);
				if (facet->getPolygon())
				{
					facet->getPolygon()->enableStippling(false);
					facet->getPolygon()->showNormals(false);
				}
				if (facet->getContour())
				{
					facet->getContour()->copyGlobalShiftAndScale(*facetCloud);
				}

				if (cloudHasNormal)
				{
					CCVector3 N = ccOctree::ComputeAverageNorm(compIndexes, cloud);
					if (N.dot(facet->getNormal()) < 0)
						facet->invertNormal();
				}

#ifdef _DEBUG
				facet->showNormalVector(true);
#endif

				ccColor::Rgb col;
				ccColor::Rgb darkCol;
				if (randomColors)
				{
					col = ccColor::Generator::Random();
					assert(c_darkColorRatio <= 1.0);
					darkCol.r = static_cast<ColorCompType>(static_cast<double>(col.r) * c_darkColorRatio);
					darkCol.g = static_cast<ColorCompType>(static_cast<double>(col.g) * c_darkColorRatio);
					darkCol.b = static_cast<ColorCompType>(static_cast<double>(col.b) * c_darkColorRatio);
				}
				else
				{
					CCVector3 N = facet->getNormal();
					PointCoordinateType dip = 0;
					PointCoordinateType dipDir = 0;
					ccNormalVectors::ConvertNormalToDipAndDipDir(N, dip, dipDir);
					FacetsClassifier::GenerateSubfamilyColor(col, dip, dipDir, 0, 1, &darkCol);
				}
				facet->setColor(col);
				if (facet->getContour())
				{
					facet->getContour()->setColor(darkCol);
					facet->getContour()->setWidth(2);
				}

				//Trace Polyline — endpoints at the extreme projections along the principal axis
				CCVector3 ExtractFracture_MaxEigVec = IdentifyFracture::CalculateMaximumEigenVector(facetCloud).toFloat();
				CCVector3 p0, p1;
				IdentifyFracture::ComputeTraceEndpoints(facetCloud, ExtractFracture_MaxEigVec, p0, p1);

				ccPointCloud* TipVertices = new ccPointCloud();
				ccPolyline* newPoly = new ccPolyline(TipVertices);
				newPoly->addChild(TipVertices);

				TipVertices->reserve(2);
				TipVertices->addPoint(p0);
				TipVertices->addPoint(p1);

				TipVertices->setEnabled(false);

				newPoly->setTempColor(ccColor::green);
				newPoly->set2DMode(false);
				newPoly->addPointIndex(0, 2);

				ccGroup->addChild(facet);
				facet->addChild(newPoly);
			}
			else
			{
				// ccFacet::Create failed (e.g. all points collinear) — free the clone.
				delete facetCloud;
			}
		}

		delete compIndexes;
		compIndexes = nullptr;

		nProgress.oneStep();
	}

	if (ccGroup->getChildrenNumber() == 0)
	{
		delete ccGroup;
		ccGroup = nullptr;
	}

	if (progressCb)
		progressCb->stop();

	return ccGroup;
}


ccHObject* IdentifyFracture::PlaneFitting(const ccHObject* ccGroup,
                                          double IntersectionLineDistance,
                                          double MinTraceLength,
                                          double MinIntersectionAngle,
                                          double MinCorrDist,
                                          GenericProgressCallback* progressCb /*=nullptr*/)
{
	ccHObject* FitJointPlanes = new ccHObject("Joint Planes");
	unsigned FitPlaneIndex = 0;
	unsigned totalnum = ccGroup->getChildrenNumber();

	if (progressCb)
	{
		progressCb->setMethodTitle("Plane Fitting");
		progressCb->setInfo(QString("Pairing %1 traces…").arg(totalnum).toUtf8().constData());
		progressCb->start();
	}
	NormalizedProgress nProgress(progressCb, std::max<unsigned>(totalnum, 1));

	// Gather each child's trace endpoints once, robustly (BFS to its 2-vertex
	// polyline) — works for `Merged trace` nodes and for bare filtered polylines.
	std::vector<CCVector3> P(totalnum), Qv(totalnum);
	std::vector<bool>      valid(totalnum, false);
	for (unsigned i = 0; i < totalnum; ++i)
		valid[i] = traceEndpointsOf(ccGroup->getChild(i), P[i], Qv[i]);

	const double c_minCosNormAngle = cos(DegreesToRadians(MinIntersectionAngle));

	for (unsigned i = 0; i < totalnum; ++i)
	{
		if (!valid[i]) { nProgress.oneStep(); continue; }
		const CCVector3 P1 = P[i];
		const CCVector3 Q1 = Qv[i];
		const CCVector3 P1Q1 = Q1 - P1;
		const double len1 = P1Q1.norm();
		if (len1 > 0.0)
		{
			const CCVector3 P1Q1n = P1Q1 / static_cast<PointCoordinateType>(len1);
			for (unsigned j = i + 1; j < totalnum; ++j)
			{
				if (!valid[j]) continue;
				const CCVector3 P2 = P[j];
				const CCVector3 Q2 = Qv[j];
				const CCVector3 P2Q2 = Q2 - P2;
				const double len2 = P2Q2.norm();
				if (len2 <= 0.0) continue;

				const CCVector3 L1L2CrossRaw = P1Q1.cross(P2Q2);
				const double crossNorm = L1L2CrossRaw.norm();
				if (crossNorm <= 0.0) continue; // parallel — not a joint-plane pair

				const CCVector3 L1L2Cross = L1L2CrossRaw / static_cast<PointCoordinateType>(crossNorm);
				const CCVector3 P2Q2n     = P2Q2 / static_cast<PointCoordinateType>(len2);
				const CCVector3 P1P2      = P2 - P1;
				const double dist = std::abs(P1P2.dot(L1L2Cross));

				if (dist < IntersectionLineDistance
					&& len1 > MinTraceLength && len2 > MinTraceLength
					&& std::abs(P1Q1n.dot(P2Q2n)) <= c_minCosNormAngle
					&& ((P1 - P2).norm() < MinCorrDist
						|| (P1 - Q2).norm() < MinCorrDist
						|| (P2 - Q1).norm() < MinCorrDist
						|| (Q1 - Q2).norm() < MinCorrDist))
				{
					ccPointCloud* AllPoint = new ccPointCloud();
					AllPoint->addPoint(P1);
					AllPoint->addPoint(P2);
					AllPoint->addPoint(Q1);
					AllPoint->addPoint(Q2);
					ccFacet* FitJointPlane = ccFacet::Create(AllPoint, 0, true);
					if (FitJointPlane)
					{
						FitPlaneIndex += 1;
						FitJointPlane->setName(QString("joint plane %1").arg(FitPlaneIndex));
						FitJointPlanes->addChild(FitJointPlane);
					}
					else
					{
						// Create failed (e.g. the 4 endpoints are collinear) — free the cloud.
						delete AllPoint;
					}
				}
			}
		}
		nProgress.oneStep();
	}

	if (progressCb)
		progressCb->stop();

	return FitJointPlanes;
}


ccHObject* IdentifyFracture::MergeCoplanarPlanes(const ccHObject* planesGroup,
                                                 double maxNormalAngleDeg,
                                                 double maxPlaneDist,
                                                 double maxCentroidDist,
                                                 unsigned maxPasses /*=1*/,
                                                 bool dropUnmerged /*=false*/,
                                                 GenericProgressCallback* progressCb /*=nullptr*/)
{
	if (!planesGroup || maxPasses == 0)
		return new ccHObject("Merged Joint Planes");

	ccHObject* current      = nullptr;           // result of the latest pass (owned)
	const ccHObject* feed   = planesGroup;        // input for the next pass (not owned on first pass)
	unsigned prevCount      = planesGroup->getChildrenNumber();

	for (unsigned pass = 0; pass < maxPasses; ++pass)
	{
		if (progressCb)
			progressCb->setInfo(QString("Pass %1/%2…").arg(pass + 1).arg(maxPasses).toUtf8().constData());

		ccHObject* next = MergeCoplanarPlanesOnce(feed, maxNormalAngleDeg, maxPlaneDist, maxCentroidDist, progressCb);
		const unsigned nextCount = next ? next->getChildrenNumber() : 0;

		// Delete the previous pass's owned output (if any) — we're replacing it.
		if (current)
			delete current;
		current = next;
		feed    = next;

		// Stop if nothing merged this pass, or nothing left to merge.
		if (nextCount >= prevCount || nextCount <= 1)
			break;
		prevCount = nextCount;
	}

	if (!current)
		current = new ccHObject("Merged Joint Planes");

	// Optionally drop planes that were never merged (represent a single original
	// stage-5 plane) — keep only genuinely consolidated joint planes.
	if (dropUnmerged)
	{
		std::vector<ccHObject*> toRemove;
		for (unsigned i = 0; i < current->getChildrenNumber(); ++i)
		{
			ccHObject* c = current->getChild(i);
			const QVariant v = c ? c->getMetaData("qtracer.origSources") : QVariant();
			if ((v.isValid() ? v.toInt() : 1) <= 1)
				toRemove.push_back(c);
		}
		for (ccHObject* c : toRemove)
		{
			current->detachChild(c);
			delete c;
		}
	}

	return current;
}


ccHObject* IdentifyFracture::MergeCoplanarPlanesOnce(const ccHObject* planesGroup,
                                                     double maxNormalAngleDeg,
                                                     double maxPlaneDist,
                                                     double maxCentroidDist,
                                                     GenericProgressCallback* progressCb)
{
	struct PlaneInfo
	{
		const ccFacet* facet  = nullptr;
		CCVector3      normal;
		CCVector3      center;
		bool           consumed = false;
	};

	std::vector<PlaneInfo> planes;
	if (planesGroup)
	{
		const unsigned n = planesGroup->getChildrenNumber();
		planes.reserve(n);
		for (unsigned i = 0; i < n; ++i)
		{
			ccHObject* c = planesGroup->getChild(i);
			if (!c || !c->isKindOf(CC_TYPES::FACET))
				continue;
			const ccFacet* f = static_cast<const ccFacet*>(c);
			PlaneInfo pi;
			pi.facet  = f;
			pi.normal = f->getNormal();
			pi.center = f->getCenter();
			planes.push_back(pi);
		}
	}

	ccHObject* merged = new ccHObject("Merged Joint Planes");

	if (planes.empty())
		return merged;

	const double cosAngleThresh = cos(DegreesToRadians(maxNormalAngleDeg));
	const unsigned totalPlanes  = static_cast<unsigned>(planes.size());

	if (progressCb)
	{
		progressCb->setMethodTitle("Coplanar Plane Merging");
		progressCb->setInfo(QString("Consolidating %1 facets…").arg(totalPlanes).toUtf8().constData());
		progressCb->start();
	}
	NormalizedProgress nProgress(progressCb, std::max<unsigned>(totalPlanes, 1));

	unsigned mergedIndex = 0;
	while (true)
	{
		// Sequential-RANSAC style: among all unconsumed seeds, pick the one with
		// the largest consensus set (peers whose center lies near seed's plane and
		// whose normal is near-parallel to seed's normal).
		int bestSeed = -1;
		std::vector<int> bestGroup;
		for (size_t i = 0; i < planes.size(); ++i)
		{
			if (planes[i].consumed) continue;
			std::vector<int> group;
			group.push_back(static_cast<int>(i));
			for (size_t j = 0; j < planes.size(); ++j)
			{
				if (j == i || planes[j].consumed) continue;
				if (std::abs(planes[i].normal.dot(planes[j].normal)) < cosAngleThresh) continue;
				const double d = std::abs((planes[j].center - planes[i].center).dot(planes[i].normal));
				if (d > maxPlaneDist) continue;
				// In-plane proximity: reject peers whose centroid is too far away, so
				// two coplanar-but-distant facets are NOT merged across a large gap
				// (0 = no limit).
				if (maxCentroidDist > 0.0
					&& (planes[j].center - planes[i].center).norm() > maxCentroidDist)
					continue;
				group.push_back(static_cast<int>(j));
			}
			if (group.size() > bestGroup.size())
			{
				bestGroup = std::move(group);
				bestSeed  = static_cast<int>(i);
			}
		}
		if (bestSeed < 0) break;

		// Aggregate all points from members' origin points (fallback to the 4-endpoint
		// corner cloud used by PlaneFitting, which lives in the facet's contour vertices).
		ccPointCloud* agg = new ccPointCloud();
		for (int idx : bestGroup)
		{
			const ccPointCloud* src = planes[idx].facet->getOriginPoints();
			if (!src || src->size() == 0)
				src = planes[idx].facet->getContourVertices();
			if (!src) continue;
			for (unsigned k = 0; k < src->size(); ++k)
				agg->addPoint(*src->getPoint(k));
		}

		// Accumulate how many *original* (stage-5) planes this facet represents,
		// summing the members' counts so it survives across passes. An input facet
		// with no tag counts as 1. A final facet whose total is 1 was never merged.
		int origSources = 0;
		for (int idx : bestGroup)
		{
			const QVariant v = planes[idx].facet->getMetaData("qtracer.origSources");
			origSources += v.isValid() ? v.toInt() : 1;
		}

		ccFacet* mergedFacet = nullptr;
		if (agg->size() >= 3)
			mergedFacet = ccFacet::Create(agg, 0, /*transferOwnership=*/true);
		if (mergedFacet)
		{
			++mergedIndex;
			mergedFacet->setMetaData("qtracer.origSources", origSources);
			mergedFacet->setName(QString("merged joint plane %1 (%2 planes)")
				.arg(mergedIndex).arg(origSources));
			merged->addChild(mergedFacet);
		}
		else
		{
			delete agg;
		}

		for (int idx : bestGroup)
		{
			planes[idx].consumed = true;
			nProgress.oneStep();
		}
	}

	if (progressCb)
		progressCb->stop();

	return merged;
}


CCVector3 IdentifyFracture::GetPointMaxEigVecFromSF(GenericIndexedCloudPersist* cloud, int PointIndex)
{
	ccPointCloud* pc = static_cast<ccPointCloud*>(cloud);

	pc->setCurrentScalarField(pc->getScalarFieldIndexByName("MaxEigVec_X"));
	ScalarType MaxEigVec_x = cloud->getPointScalarValue(PointIndex);
	pc->setCurrentScalarField(pc->getScalarFieldIndexByName("MaxEigVec_Y"));
	ScalarType MaxEigVec_y = cloud->getPointScalarValue(PointIndex);
	pc->setCurrentScalarField(pc->getScalarFieldIndexByName("MaxEigVec_Z"));
	ScalarType MaxEigVec_z = cloud->getPointScalarValue(PointIndex);
	pc->setCurrentScalarField(pc->getScalarFieldIndexByName("DBSCAN"));
	return CCVector3(MaxEigVec_x, MaxEigVec_y, MaxEigVec_z);
}


bool IdentifyFracture::InCone(const CCVector3& A, const CCVector3& B, double radius, const CCVector3& P)
{
	CCVector3 AB = B - A;
	CCVector3 AP = P - A;

	double AB_dot_AB = AB.norm2();

	// degenerate (A==B) guard
	if (AB_dot_AB < 1e-12) return false;

	double t = AP.dot(AB) / AB_dot_AB;

	double dist_sq = AP.norm2() - t * t * AB_dot_AB;

	return dist_sq <= radius * radius;
}
