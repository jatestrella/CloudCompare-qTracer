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
#include "ColorFilterDlg.h"
#include "disclaimerDialog.h"
#include "IdentifyFracture.h"
#include "PipelineDlg.h"

//CCCoreLib
#include <CCConst.h>
#include <ReferenceCloud.h>


qTracer::qTracer( QObject* parent )
	: QObject( parent )
	, ccStdPluginInterface( ":/CC/plugin/qTracer/info.json" )
{
}

// ---------------------------------------------------------------------------
void qTracer::onNewSelection( const ccHObject::Container& selectedEntities )
{
	const bool oneCloud = (selectedEntities.size() == 1
	                    && selectedEntities.front()
	                    && selectedEntities.front()->isKindOf(CC_TYPES::POINT_CLOUD));

	if (m_action)
		m_action->setEnabled(oneCloud);

	if (m_colorFilterAction)
	{
		const ccPointCloud* pc = oneCloud ? ccHObjectCaster::ToPointCloud(selectedEntities.front()) : nullptr;
		m_colorFilterAction->setEnabled(pc && pc->hasColors());
	}
}

QList<QAction*> qTracer::getActions()
{
	if (!m_colorFilterAction)
	{
		m_colorFilterAction = new QAction("Filter by Color…", this);
		m_colorFilterAction->setToolTip("Interactive R/G/B and C=R+G+B threshold filter.\n"
		                                "Produces a new point cloud that can be fed into the extraction pipeline.");
		m_colorFilterAction->setIcon(getIcon());
		connect(m_colorFilterAction, &QAction::triggered, this, &qTracer::doColorFilter);
	}
	if (!m_action)
	{
		m_action = new QAction("Extract Fractures (Pipeline)", this);
		m_action->setToolTip("Run the full pipeline on the selected point cloud:\n"
		                     "Compute Eigen -> DBSCAN -> Create Traces -> Trace Clustering -> Plane Fitting");
		m_action->setIcon(getIcon());
		connect(m_action, &QAction::triggered, this, &qTracer::doPipeline);
	}
	return { m_colorFilterAction, m_action };
}

// ---------------------------------------------------------------------------
void qTracer::doColorFilter()
{
	if (!ShowDisclaimer(m_app))
		return;
	assert(m_app);
	if (!m_app)
		return;

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
	if (!pc->hasColors())
	{
		m_app->dispToConsole("[qTracer] The selected cloud has no RGB colors.", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		return;
	}

	ColorFilterDlg dlg(pc, m_app, m_app->getMainWindow());
	if (!dlg.exec())
		return;

	// Materialise the currently-visible subset into a new cloud.
	const auto& vis = pc->getTheVisibilityArray();
	const unsigned n = pc->size();
	CCCoreLib::ReferenceCloud ref(pc);
	if (!ref.reserve(n))
	{
		m_app->dispToConsole("[qTracer] Not enough memory to build the filtered cloud.",
		                     ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		pc->unallocateVisibilityArray();
		pc->redrawDisplay();
		return;
	}
	for (unsigned i = 0; i < n; ++i)
	{
		if (i < vis.size() && vis[i] == CCCoreLib::POINT_VISIBLE)
			ref.addPointIndex(i);
	}

	// Always drop the visibility array on the source so it renders normally again.
	pc->unallocateVisibilityArray();
	pc->redrawDisplay();

	if (ref.size() == 0)
	{
		m_app->dispToConsole("[qTracer] Color filter kept 0 points — nothing to create.",
		                     ccMainAppInterface::WRN_CONSOLE_MESSAGE);
		return;
	}

	ccPointCloud* filtered = pc->partialClone(&ref);
	if (!filtered)
	{
		m_app->dispToConsole("[qTracer] partialClone failed.", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		return;
	}
	filtered->setName(pc->getName() + QString(" [color filtered C(%1,%2,%3) %4-%5]")
		.arg(dlg.coefA(), 0, 'f', 2).arg(dlg.coefB(), 0, 'f', 2).arg(dlg.coefC(), 0, 'f', 2)
		.arg(dlg.cMin(), 0, 'f', 2).arg(dlg.cMax(), 0, 'f', 2));
	filtered->showColors(true);
	filtered->setDisplay(pc->getDisplay());

	if (ccHObject* parent = pc->getParent())
		parent->addChild(filtered);
	m_app->addToDB(filtered);

	// Uncheck the source cloud and move the selection to the freshly filtered one
	// so the user can immediately run the pipeline on it.
	pc->setEnabled(false);
	m_app->setSelectedInDB(pc, false);
	m_app->setSelectedInDB(filtered, true);
	pc->redrawDisplay();

	m_app->dispToConsole(
		QString("[qTracer] Color filter kept %1 / %2 points → \"%3\".")
			.arg(filtered->size()).arg(n).arg(filtered->getName()),
		ccMainAppInterface::STD_CONSOLE_MESSAGE);
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

		IdentifyFracture::runDBSCAN(octree, pc, params, &progress);
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
		facetsGroup = IdentifyFracture::createTraces(pc, components, dlg.randomColors(), err, &progress);
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
		dlg.traceMinAngleDeg(),
		&progress);
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
		dlg.planeMaxEndPointDist(),
		&progress);
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
