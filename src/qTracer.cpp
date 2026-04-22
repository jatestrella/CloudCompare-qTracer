//##########################################################################
//#                                                                        #
//#                CLOUDCOMPARE PLUGIN: qTracer                             #
//#                                                                        #
//#  GNU GPL v2 or later                                                   #
//#                                                                        #
//##########################################################################

//Qt
#include <QtGui>

//CCCoreLib
#include <AutoSegmentationTools.h>

//qCC_db
#include <ccHObjectCaster.h>
#include <ccPointCloud.h>
#include <ccProgressDialog.h>
#include <ccScalarField.h>

//Plugin
#include "qTracer.h"
#include "disclaimerDialog.h"
#include "IdentifyFracture.h"
#include "PipelineDlg.h"


qTracer::qTracer( QObject* parent )
	: QObject( parent )
	, ccStdPluginInterface( ":/CC/plugin/qTracer/info.json" )
{
}

// ---------------------------------------------------------------------------
void qTracer::onNewSelection( const ccHObject::Container& selectedEntities )
{
	if (!m_action)
		return;
	const bool oneCloud = (selectedEntities.size() == 1
	                    && selectedEntities.front()
	                    && selectedEntities.front()->isKindOf(CC_TYPES::POINT_CLOUD));
	m_action->setEnabled(oneCloud);
}

QList<QAction*> qTracer::getActions()
{
	if (!m_action)
	{
		m_action = new QAction("Extract Fractures (Pipeline)", this);
		m_action->setToolTip("Run the full pipeline on the selected point cloud:\n"
		                     "Compute Eigen -> DBSCAN -> Create Traces -> Trace Clustering -> Plane Fitting");
		m_action->setIcon(getIcon());
		connect(m_action, &QAction::triggered, this, &qTracer::doPipeline);
	}
	return { m_action };
}

// ---------------------------------------------------------------------------
void qTracer::doPipeline()
{
	if (!ShowDisclaimer(m_app))
		return;
	assert(m_app);
	if (!m_app)
		return;

	// --- selection ---------------------------------------------------------
	const ccHObject::Container& sel = m_app->getSelectedEntities();
	if (sel.size() != 1)
	{
		m_app->dispToConsole("[qTracer] Select exactly one point cloud.", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		return;
	}
	ccPointCloud* pc = ccHObjectCaster::ToPointCloud(sel.front());
	if (!pc)
	{
		m_app->dispToConsole("[qTracer] Selection is not a point cloud.", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		return;
	}

	// --- parameter dialog --------------------------------------------------
	PipelineDlg dlg(m_app->getMainWindow());
	if (!dlg.exec())
		return;

	// parent group that collects all three stage outputs
	ccHObject* root = new ccHObject(QString("qTracer [%1]").arg(pc->getName()));

	ccProgressDialog progress(true, m_app->getMainWindow());
	progress.setAutoClose(true);

	// ---------------------------------------------------------------------
	// Stage 1 : Compute Eigen Features
	// ---------------------------------------------------------------------
	m_app->dispToConsole("[qTracer] (1/5) Compute Eigen Features…", ccMainAppInterface::STD_CONSOLE_MESSAGE);
	progress.setWindowTitle("qTracer | 1/5 Compute Eigen Features");
	{
		IdentifyFracture::ErrorCode rc = IdentifyFracture::ComputeEigen(
			pc,
			static_cast<PointCoordinateType>(dlg.kernelRadius()),
			&progress,
			nullptr);
		if (rc != IdentifyFracture::NoError)
		{
			m_app->dispToConsole(QString("[qTracer] ComputeEigen failed (code %1). Pipeline aborted.").arg(rc),
			                     ccMainAppInterface::ERR_CONSOLE_MESSAGE);
			delete root;
			return;
		}
	}

	// ---------------------------------------------------------------------
	// Stage 2 : DBSCAN
	// ---------------------------------------------------------------------
	m_app->dispToConsole("[qTracer] (2/5) DBSCAN clustering…", ccMainAppInterface::STD_CONSOLE_MESSAGE);
	progress.setWindowTitle("qTracer | 2/5 DBSCAN");
	{
		IdentifyFracture::DBSCANParams params;
		params.radius           = static_cast<PointCoordinateType>(dlg.dbscanRadius());
		params.minPoints        = dlg.dbscanMinPoints();
		params.maxAngleDeg      = dlg.dbscanMaxAngleDeg();
		params.linearityThresh  = dlg.dbscanLinearityThresh();
		params.searchType       = dlg.dbscanSearchType();

		CCCoreLib::DgmOctree* octree = pc->getOctree().data();
		CCCoreLib::DgmOctree tmpOctree(pc);
		if (!octree)
		{
			if (tmpOctree.build(&progress) < 1)
			{
				m_app->dispToConsole("[qTracer] Octree build failed. Pipeline aborted.",
				                     ccMainAppInterface::ERR_CONSOLE_MESSAGE);
				delete root;
				return;
			}
			octree = &tmpOctree;
		}

		IdentifyFracture::runDBSCAN(octree, pc, params);
	}

	// show DBSCAN SF on the input cloud for quick inspection
	{
		int dbscanSFIdx = pc->getScalarFieldIndexByName("DBSCAN");
		if (dbscanSFIdx >= 0)
		{
			ccScalarField* sf = static_cast<ccScalarField*>(pc->getScalarField(dbscanSFIdx));
			if (sf) sf->computeMinAndMax();
			pc->setCurrentDisplayedScalarField(dbscanSFIdx);
			pc->showSF(true);
		}
	}

	// ---------------------------------------------------------------------
	// Stage 3 : Create Traces (facets)
	// ---------------------------------------------------------------------
	m_app->dispToConsole("[qTracer] (3/5) Create Traces…", ccMainAppInterface::STD_CONSOLE_MESSAGE);
	progress.setWindowTitle("qTracer | 3/5 Create Traces");
	ccHObject* facetsGroup = nullptr;
	{
		int dbscanSFIdx = pc->getScalarFieldIndexByName("DBSCAN");
		if (dbscanSFIdx < 0)
		{
			m_app->dispToConsole("[qTracer] No DBSCAN SF after clustering. Pipeline aborted.",
			                     ccMainAppInterface::ERR_CONSOLE_MESSAGE);
			delete root;
			return;
		}
		pc->setCurrentScalarField(dbscanSFIdx);

		CCCoreLib::ReferenceCloudContainer components;
		if (!CCCoreLib::AutoSegmentationTools::extractConnectedComponents(pc, components))
		{
			m_app->dispToConsole("[qTracer] extractConnectedComponents failed (out of memory?). Pipeline aborted.",
			                     ccMainAppInterface::ERR_CONSOLE_MESSAGE);
			for (CCCoreLib::ReferenceCloud* rc : components) delete rc;
			delete root;
			return;
		}

		// remove DBSCAN SF so it doesn't get copied into every sub-cloud
		pc->deleteScalarField(dbscanSFIdx);

		bool err = false;
		facetsGroup = IdentifyFracture::createTraces(pc, components, dlg.randomColors(), err);
		if (err)
		{
			m_app->dispToConsole("[qTracer] Errors during trace creation — result may be incomplete.",
			                     ccMainAppInterface::WRN_CONSOLE_MESSAGE);
		}
		if (!facetsGroup || facetsGroup->getChildrenNumber() == 0)
		{
			m_app->dispToConsole("[qTracer] No traces produced. Pipeline aborted.",
			                     ccMainAppInterface::WRN_CONSOLE_MESSAGE);
			delete facetsGroup;
			delete root;
			return;
		}
		root->addChild(facetsGroup);
	}

	// ---------------------------------------------------------------------
	// Stage 4 : Trace Clustering
	// ---------------------------------------------------------------------
	m_app->dispToConsole("[qTracer] (4/5) Trace Clustering…", ccMainAppInterface::STD_CONSOLE_MESSAGE);
	progress.setWindowTitle("qTracer | 4/5 Trace Clustering");
	ccHObject* tracesGroup = IdentifyFracture::TraceClustering(
		facetsGroup,
		dlg.traceConeRadius(),
		dlg.traceTwoTraceDist(),
		dlg.traceMinAngleDeg());
	if (!tracesGroup || tracesGroup->getChildrenNumber() == 0)
	{
		m_app->dispToConsole("[qTracer] Trace Clustering produced no combined traces. Stopping.",
		                     ccMainAppInterface::WRN_CONSOLE_MESSAGE);
		delete tracesGroup;
		pc->setEnabled(false);
		root->prepareDisplayForRefresh_recursive();
		m_app->addToDB(root);
		return;
	}
	root->addChild(tracesGroup);

	// ---------------------------------------------------------------------
	// Stage 5 : Plane Fitting
	// ---------------------------------------------------------------------
	m_app->dispToConsole("[qTracer] (5/5) Plane Fitting…", ccMainAppInterface::STD_CONSOLE_MESSAGE);
	progress.setWindowTitle("qTracer | 5/5 Plane Fitting");
	ccHObject* planesGroup = IdentifyFracture::PlaneFitting(
		tracesGroup,
		dlg.planeIntersectionLineDist(),
		dlg.planeMinTraceLength(),
		dlg.planeMinIntersectionAngleDeg(),
		dlg.planeMaxEndPointDist());
	if (planesGroup)
	{
		root->addChild(planesGroup);
	}

	// ---------------------------------------------------------------------
	pc->setEnabled(false);
	root->prepareDisplayForRefresh_recursive();
	m_app->addToDB(root);

	m_app->dispToConsole(
		QString("[qTracer] Pipeline done — %1 facets, %2 combined traces, %3 joint planes.")
			.arg(facetsGroup ? facetsGroup->getChildrenNumber() : 0)
			.arg(tracesGroup ? tracesGroup->getChildrenNumber() : 0)
			.arg(planesGroup ? planesGroup->getChildrenNumber() : 0),
		ccMainAppInterface::STD_CONSOLE_MESSAGE);
}
