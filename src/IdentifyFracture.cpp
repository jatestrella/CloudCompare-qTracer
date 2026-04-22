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
#include <iostream>
#include <vector>

//Qt
#include <QString>

using namespace CCCoreLib;


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
                                const DBSCANParams& params)
{
	ccPointCloud* pc = static_cast<ccPointCloud*>(cloud);
	pc->addScalarField("DBSCAN");
	pc->setCurrentScalarField(pc->getScalarFieldIndexByName("DBSCAN"));

	for (unsigned i = 0; i < pc->size(); ++i)
	{
		pc->setPointScalarValue(i, static_cast<ScalarType>(-1)); // UNCLASSIFIED
	}

	unsigned clusterID = 1;

	for (unsigned i = 0; i < pc->size(); ++i)
	{
		std::cout << "Percent = " << double(i) / double(pc->size()) << std::endl;
		if (pc->getPointScalarValue(i) == -1)
		{
			if (IdentifyFracture::expandCluster(octree, cloud, i, clusterID, params) != FAILURE)
			{
				clusterID += 1;
			}
		}
	}

	return 0;
}

void IdentifyFracture::SetScalarValueToNOISE(const CCVector3& /*P*/, ScalarType& scalarValue)
{
	scalarValue = -2; //NOISE
}


ccHObject* IdentifyFracture::TraceClustering(const ccHObject* ccGroup,
                                             double ConeRadius,
                                             double TwoTraceDist,
                                             double MinAngle)
{
	//build searching group
	ccHObject* victim = new ccHObject("victim");
	for (unsigned i = 0; i < ccGroup->getChildrenNumber(); i++)
	{
		victim->addChild(ccGroup->getChild(i)->getChild(2)->getChild(0));
		QString Name = QString("Trace %1").arg(i);
		victim->getChild(i)->setName(Name);
	}

	ccHObject* Traces = new ccHObject("Traces");

	unsigned tracenumber = 0;

	while (victim->getFirstChild())
	{
		ccHObject* DieGroup = new ccHObject("DisperseTraces");
		ccHObject* InfectedGroup = new ccHObject("InfestedGroup");

		InfectedGroup->addChild(victim->getFirstChild());
		victim->detachChild(victim->getFirstChild());

		QString TraceName = QString("CombinedTrace %1").arg(tracenumber);
		ccHObject* Trace = new ccHObject(TraceName);
		tracenumber++;

		while (InfectedGroup->getFirstChild())
		{
			std::cout << "Now infect: " << InfectedGroup->getFirstChild()->getName().toStdString() << std::endl;

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
		CCVector3 ExtractFracture_bbMin;
		CCVector3 ExtractFracture_bbMax;
		PC_die->getBoundingBox(ExtractFracture_bbMin, ExtractFracture_bbMax);
		float Fracture_HalfLength = 0.5f * static_cast<float>(std::sqrt(
			std::pow(ExtractFracture_bbMax[0] - ExtractFracture_bbMin[0], 2)
			+ std::pow(ExtractFracture_bbMax[1] - ExtractFracture_bbMin[1], 2)
			+ std::pow(ExtractFracture_bbMax[2] - ExtractFracture_bbMin[2], 2)));
		CCVector3 ExtractFracture_Center = (ExtractFracture_bbMax + ExtractFracture_bbMin) * 0.5f;

		ccPointCloud* TipVertices = new ccPointCloud();
		ccPolyline* TracePoly = new ccPolyline(TipVertices);
		TracePoly->addChild(TipVertices);

		TipVertices->reserve(2);
		TipVertices->addPoint(ExtractFracture_Center + ExtractFracture_MaxEigVec * Fracture_HalfLength);
		TipVertices->addPoint(ExtractFracture_Center - ExtractFracture_MaxEigVec * Fracture_HalfLength);

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
	std::cout << "end" << std::endl;

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
	pc->setCurrentScalarField(pc->getScalarFieldIndexByName("PC Linearity"));
	ScalarType Corelinearity = cloud->getPointScalarValue(corePointIndex);
	pc->setCurrentScalarField(pc->getScalarFieldIndexByName("DBSCAN"));
	CCVector3 CoreMaxEigVec = GetPointMaxEigVecFromSF(cloud, corePointIndex);

	unsigned char level = octree->findBestLevelForAGivenNeighbourhoodSizeExtraction(radius);
	DgmOctree::NeighboursSet CoreCloud;
	CCVector3 vec;
	cloud->getPoint(corePointIndex, vec);
	size_t num = 0;
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

		num = octree->getPointsInCylindricalNeighbourhood(cylParams);
		CoreCloud = cylParams.neighbours;
	}
	else if (SearchType == 2)
	{
		num = octree->getPointsInSphericalNeighbourhood(vec, radius, CoreCloud, level);
	}

	DgmOctreeReferenceCloud CoreCloud_ref(&CoreCloud, 0);

	const double c_minCosNormAngle = cos(DegreesToRadians(maxAngle_deg));

	if (CoreCloud_ref.size() < m_minPoints || Corelinearity < linearityThreshold)
	{
		CoreCloud_ref.forEach(IdentifyFracture::SetScalarValueToNOISE);
		return FAILURE;
	}
	else
	{
		int index = 0, indexCorePoint = 0;
		DgmOctree::NeighboursSet::iterator iterSeeds;
		for (iterSeeds = CoreCloud.begin(); iterSeeds != CoreCloud.end(); ++iterSeeds)
		{
			pc->setCurrentScalarField(pc->getScalarFieldIndexByName("DBSCAN"));
			cloud->setPointScalarValue(iterSeeds->pointIndex, static_cast<ScalarType>(clusterID));

			if (iterSeeds->point == cloud->getPoint(corePointIndex))
			{
				indexCorePoint = index;
			}
			++index;
		}
		CoreCloud.erase(CoreCloud.begin() + indexCorePoint);

		std::cout << "Start expand search" << std::endl;

		for (size_t i = 0, n = CoreCloud.size(); i < n; ++i)
		{
			DgmOctree::NeighboursSet NeighborsCloud;
			CCVector3 NeighborsMaxEigVec = IdentifyFracture::GetPointMaxEigVecFromSF(cloud, CoreCloud[i].pointIndex);
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

			pc->setCurrentScalarField(pc->getScalarFieldIndexByName("PC Linearity"));
			pc->setCurrentScalarField(pc->getScalarFieldIndexByName("DBSCAN"));

			if (num_cluster >= m_minPoints && std::abs(CoreMaxEigVec.dot(NeighborsMaxEigVec)) > c_minCosNormAngle)
			{
				DgmOctree::NeighboursSet::iterator iterNeighbors;

				for (iterNeighbors = NeighborsCloud.begin(); iterNeighbors != NeighborsCloud.end(); ++iterNeighbors)
				{
					pc->setCurrentScalarField(pc->getScalarFieldIndexByName("DBSCAN"));

					if (cloud->getPointScalarValue(iterNeighbors->pointIndex) == -1
						|| cloud->getPointScalarValue(iterNeighbors->pointIndex) == -2)
					{
						if (cloud->getPointScalarValue(iterNeighbors->pointIndex) == -1)
						{
							CoreCloud.push_back(*iterNeighbors);
							n = CoreCloud.size();
						}
						cloud->setPointScalarValue(iterNeighbors->pointIndex, static_cast<ScalarType>(clusterID));
					}
				}
			}
		}

		return dbscan_SUCCESS;
	}
}


ccHObject* IdentifyFracture::createTraces(ccPointCloud* cloud,
                                          ReferenceCloudContainer& components,
                                          bool randomColors,
                                          bool& error)
{
	if (!cloud)
		return nullptr;

	ccHObject* ccGroup = new ccHObject(cloud->getName() + QString(" [facets]"));
	ccGroup->setDisplay(cloud->getDisplay());
	ccGroup->setVisible(true);

	bool cloudHasNormal = cloud->hasNormals();

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
		else
		{
			ccFacet* facet = ccFacet::Create(facetCloud, 0, true);
			if (facet)
			{
				QString facetName = QString("Trace %1").arg(ccGroup->getChildrenNumber());
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

				//Trace Polyline
				CCVector3 ExtractFracture_MaxEigVec = IdentifyFracture::CalculateMaximumEigenVector(facetCloud).toFloat();
				CCVector3 ExtractFracture_bbMin;
				CCVector3 ExtractFracture_bbMax;
				facetCloud->getBoundingBox(ExtractFracture_bbMin, ExtractFracture_bbMax);
				float Fracture_HalfLength = 0.5f * static_cast<float>(std::sqrt(
					std::pow(ExtractFracture_bbMax[0] - ExtractFracture_bbMin[0], 2)
					+ std::pow(ExtractFracture_bbMax[1] - ExtractFracture_bbMin[1], 2)
					+ std::pow(ExtractFracture_bbMax[2] - ExtractFracture_bbMin[2], 2)));
				CCVector3 ExtractFracture_Center = (ExtractFracture_bbMax + ExtractFracture_bbMin) * 0.5f;
				ccPointCloud* TipVertices = new ccPointCloud();
				ccPolyline* newPoly = new ccPolyline(TipVertices);
				newPoly->addChild(TipVertices);

				TipVertices->reserve(2);
				TipVertices->addPoint(ExtractFracture_Center + ExtractFracture_MaxEigVec * Fracture_HalfLength);
				TipVertices->addPoint(ExtractFracture_Center - ExtractFracture_MaxEigVec * Fracture_HalfLength);

				TipVertices->setEnabled(false);

				newPoly->setTempColor(ccColor::green);
				newPoly->set2DMode(false);
				newPoly->addPointIndex(0, 2);

				ccGroup->addChild(facet);
				facet->addChild(newPoly);
			}
		}

		delete compIndexes;
		compIndexes = nullptr;
	}

	if (ccGroup->getChildrenNumber() == 0)
	{
		delete ccGroup;
		ccGroup = nullptr;
	}

	return ccGroup;
}


ccHObject* IdentifyFracture::PlaneFitting(const ccHObject* ccGroup,
                                          double IntersectionLineDistance,
                                          double MinTraceLength,
                                          double MinIntersectionAngle,
                                          double MinCorrDist)
{
	ccHObject* FitJointPlanes = new ccHObject("Fit Joint Planes");
	unsigned FitPlaneIndex = 0;
	unsigned totalnum = ccGroup->getChildrenNumber();
	std::cout << "Start Plane Fitting, Planes number = " << int(totalnum) << std::endl;

	for (unsigned i = 0; i < totalnum; ++i)
	{
		ccHObject* currentline = ccGroup->getChild(i);
		ccGenericPointCloud* currentPoint = ccHObjectCaster::ToGenericPointCloud(currentline->getChild(1)->getChild(0), nullptr);
		GenericIndexedCloud* currentPC = static_cast<GenericIndexedCloud*>(currentPoint);
		const CCVector3 P1 = *(currentPC->getPoint(0));
		const CCVector3 Q1 = *(currentPC->getPoint(1));
		CCVector3 P1Q1  = Q1 - P1;
		CCVector3 P1Q1n = P1Q1 / P1Q1.norm();

		for (unsigned j = i + 1; j < totalnum; ++j)
		{
			ccHObject* compareline = ccGroup->getChild(j);
			ccGenericPointCloud* comparePoint = ccHObjectCaster::ToGenericPointCloud(compareline->getChild(1)->getChild(0), nullptr);
			GenericIndexedCloud* comparePC = static_cast<GenericIndexedCloud*>(comparePoint);
			const CCVector3 P2 = *(comparePC->getPoint(0));
			const CCVector3 Q2 = *(comparePC->getPoint(1));
			CCVector3 P2Q2 = Q2 - P2;
			CCVector3 P1P2 = P2 - P1;
			CCVector3 L1L2Cross = P1Q1.cross(P2Q2);
			L1L2Cross = L1L2Cross / L1L2Cross.norm();
			CCVector3 P2Q2n = P2Q2 / P2Q2.norm();
			double dist = std::abs(P1P2.dot(L1L2Cross));
			const double c_minCosNormAngle = cos(DegreesToRadians(MinIntersectionAngle));

			if (dist < IntersectionLineDistance)
			{
				if (P1Q1.norm() > MinTraceLength && P2Q2.norm() > MinTraceLength)
				{
					if (std::abs(P1Q1n.dot(P2Q2n)) <= c_minCosNormAngle
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

						FitPlaneIndex += 1;
						QString facetName = QString("joint plane %1").arg(FitPlaneIndex);
						FitJointPlane->setName(facetName);
						FitJointPlanes->addChild(FitJointPlane);
					}
				}
			}
		}
	}
	return FitJointPlanes;
}


ScalarType IdentifyFracture::GetCloudLinearity(ReferenceCloud* cloud)
{
	Neighbourhood Z(cloud);

	SquareMatrixd eigVectors;
	std::vector<double> eigValues;
	SquareMatrixd covarianceMatrix = Z.computeCovarianceMatrix();

	Jacobi<double>::ComputeEigenValuesAndVectors(covarianceMatrix, eigVectors, eigValues, true);
	Jacobi<double>::SortEigenValuesAndVectors(eigVectors, eigValues);

	const double a1 = eigValues[0];
	const double a2 = eigValues[1];

	if (a1 == 0.0)
		return NAN_VALUE;

	return static_cast<ScalarType>((a1 - a2) / a1);
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
