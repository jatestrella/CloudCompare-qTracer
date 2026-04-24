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
#include "OutcropAreaDlg.h"
#include "P21Dlg.h"
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
	const bool oneEntity = (selectedEntities.size() == 1 && selectedEntities.front());
	const bool oneCloud  = oneEntity && selectedEntities.front()->isKindOf(CC_TYPES::POINT_CLOUD);

	// Pipeline accepts any single entity — stage-1/2/3 need a point cloud, stage-4+ take a group.
	// Which is valid is enforced in doPipeline() once the dialog tells us the starting stage.
	if (m_action)
		m_action->setEnabled(oneEntity);

	if (m_colorFilterAction)
	{
		const ccPointCloud* pc = oneCloud ? ccHObjectCaster::ToPointCloud(selectedEntities.front()) : nullptr;
		m_colorFilterAction->setEnabled(pc && pc->hasColors());
	}

	if (m_outcropAreaAction)
		m_outcropAreaAction->setEnabled(oneCloud);

	// P21 action is always enabled — the dialog auto-detects candidates in the DB.
	if (m_p21Action)
		m_p21Action->setEnabled(true);
}

QList<QAction*> qTracer::getActions()
{
	if (!m_colorFilterAction)
	{
		m_colorFilterAction = new QAction("Filter by Color…", this);
		m_colorFilterAction->setToolTip("Interactive R/G/B and C=R+G+B threshold filter.\n"
		                                "Produces a new point cloud that can be fed into the extraction pipeline.");
		m_colorFilterAction->setIcon(QIcon(":/CC/plugin/qTracer/images/icon_colorfilter.svg"));
		connect(m_colorFilterAction, &QAction::triggered, this, &qTracer::doColorFilter);
	}
	if (!m_action)
	{
		m_action = new QAction("Extract Fractures (Pipeline)", this);
		m_action->setToolTip("Run the full pipeline on the selected point cloud:\n"
		                     "Compute Eigen -> DBSCAN -> Create Traces -> Trace Clustering -> Plane Fitting");
		m_action->setIcon(QIcon(":/CC/plugin/qTracer/images/icon_pipeline.svg"));
		connect(m_action, &QAction::triggered, this, &qTracer::doPipeline);
	}
	if (!m_outcropAreaAction)
	{
		m_outcropAreaAction = new QAction("Compute Outcrop Area…", this);
		m_outcropAreaAction->setToolTip("Estimate the outcrop surface area by octree or kd-tree plane fits\n"
		                                "(cell-by-cell plane-box intersection polygon sum).");
		m_outcropAreaAction->setIcon(QIcon(":/CC/plugin/qTracer/images/icon_outcroparea.svg"));
		connect(m_outcropAreaAction, &QAction::triggered, this, &qTracer::doOutcropArea);
	}
	if (!m_p21Action)
	{
		m_p21Action = new QAction("Compute P21…", this);
		m_p21Action->setToolTip("Compute fracture intensity P21 = total trace length / outcrop surface area.\n"
		                        "Candidate trace groups and area meshes are auto-detected from the DB tree.");
		m_p21Action->setIcon(QIcon(":/CC/plugin/qTracer/images/icon_p21.svg"));
		connect(m_p21Action, &QAction::triggered, this, &qTracer::doP21);
	}
	return { m_colorFilterAction, m_outcropAreaAction, m_p21Action, m_action };
}

// ---------------------------------------------------------------------------
void qTracer::doOutcropArea()
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

	OutcropAreaDlg dlg(pc, m_app, m_app->getMainWindow());
	dlg.exec();
}

// ---------------------------------------------------------------------------
void qTracer::doP21()
{
	if (!ShowDisclaimer(m_app))
		return;
	assert(m_app);
	if (!m_app)
		return;

	P21Dlg dlg(m_app, m_app->getMainWindow());
	dlg.exec();
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
	if (sel.size() != 1 || !sel.front())
	{
		m_app->dispToConsole("[qTracer] Select exactly one entity.", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		return;
	}
	ccHObject* selected = sel.front();

	// --- parameter dialog --------------------------------------------------
	PipelineDlg dlg(m_app->getMainWindow());
	if (!dlg.exec())
		return;

	const int firstStage = dlg.firstEnabledStage();
	const int lastStage  = dlg.lastEnabledStage();
	// (dlg.accept() already verified firstStage > 0 and stages are contiguous.)

	// --- input validation --------------------------------------------------
	// Stages 1-3 process a point cloud; stages 4-6 consume an ccHObject group
	// produced by the previous stage. If the user is starting mid-pipeline,
	// the selection must match that stage's input shape and the required data
	// must already be present on / under it.
	ccPointCloud* pc         = nullptr;
	ccHObject*    inputGroup = nullptr;

	if (firstStage <= 3)
	{
		pc = ccHObjectCaster::ToPointCloud(selected);
		if (!pc)
		{
			m_app->dispToConsole(
				QString("[qTracer] Starting at stage %1 requires a point cloud selection.").arg(firstStage),
				ccMainAppInterface::ERR_CONSOLE_MESSAGE);
			return;
		}
		if (firstStage >= 2)
		{
			if (pc->getScalarFieldIndexByName("PC Linearity") < 0
			 || pc->getScalarFieldIndexByName("MaxEigVec_X")   < 0
			 || pc->getScalarFieldIndexByName("MaxEigVec_Y")   < 0
			 || pc->getScalarFieldIndexByName("MaxEigVec_Z")   < 0)
			{
				m_app->dispToConsole(
					"[qTracer] Starting at stage 2 requires the cloud to already have "
					"'PC Linearity' and 'MaxEigVec_X/Y/Z' scalar fields (run stage 1 first).",
					ccMainAppInterface::ERR_CONSOLE_MESSAGE);
				return;
			}
		}
		if (firstStage >= 3)
		{
			if (pc->getScalarFieldIndexByName("DBSCAN") < 0)
			{
				m_app->dispToConsole(
					"[qTracer] Starting at stage 3 requires the cloud to already have a "
					"'DBSCAN' scalar field (run stage 2 first).",
					ccMainAppInterface::ERR_CONSOLE_MESSAGE);
				return;
			}
		}
	}
	else
	{
		inputGroup = selected;
		if (inputGroup->getChildrenNumber() == 0)
		{
			m_app->dispToConsole(
				QString("[qTracer] Starting at stage %1 requires a non-empty group selection.").arg(firstStage),
				ccMainAppInterface::ERR_CONSOLE_MESSAGE);
			return;
		}
	}

	// parent group that collects newly-produced stage outputs
	const QString sourceName = pc ? pc->getName() : selected->getName();
	ccHObject* root = new ccHObject(QString("qTracer [%1]").arg(sourceName));

	ccProgressDialog progress(true, m_app->getMainWindow());
	progress.setAutoClose(true);

	// Chain variables. When starting mid-pipeline, the user-selected group is
	// "borrowed" (not re-parented) as input to the first enabled stage.
	ccHObject* facetsGroup = (firstStage == 4) ? inputGroup : nullptr;
	ccHObject* tracesGroup = (firstStage == 5) ? inputGroup : nullptr;
	ccHObject* planesGroup = (firstStage == 6) ? inputGroup : nullptr;
	ccHObject* mergedGroup = nullptr;

	auto announce = [&](int stage, const char* name)
	{
		m_app->dispToConsole(QString("[qTracer] (%1/6) %2…").arg(stage).arg(name),
		                     ccMainAppInterface::STD_CONSOLE_MESSAGE);
		progress.setWindowTitle(QString("qTracer | %1/6 %2").arg(stage).arg(name));
	};

	const bool runStage1 = firstStage <= 1 && lastStage >= 1;
	const bool runStage2 = firstStage <= 2 && lastStage >= 2;
	const bool runStage3 = firstStage <= 3 && lastStage >= 3;
	const bool runStage4 = firstStage <= 4 && lastStage >= 4;
	const bool runStage5 = firstStage <= 5 && lastStage >= 5;
	const bool runStage6 = firstStage <= 6 && lastStage >= 6;

	// ---------------------------------------------------------------------
	// Stage 1 : Compute Eigen Features
	// ---------------------------------------------------------------------
	if (runStage1)
	{
		announce(1, "Compute Eigen Features");
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
	if (runStage2)
	{
		announce(2, "DBSCAN");
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

	// Show DBSCAN SF on the cloud for quick inspection (whenever available).
	if (pc)
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
	if (runStage3)
	{
		announce(3, "Create Traces");
		int dbscanSFIdx = pc->getScalarFieldIndexByName("DBSCAN");
		if (dbscanSFIdx < 0)
		{
			m_app->dispToConsole("[qTracer] No DBSCAN SF available for stage 3. Aborted.",
			                     ccMainAppInterface::ERR_CONSOLE_MESSAGE);
			delete root;
			return;
		}
		pc->setCurrentScalarField(dbscanSFIdx);

		CCCoreLib::ReferenceCloudContainer components;
		if (!CCCoreLib::AutoSegmentationTools::extractConnectedComponents(pc, components))
		{
			m_app->dispToConsole("[qTracer] extractConnectedComponents failed (out of memory?). Aborted.",
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
			m_app->dispToConsole("[qTracer] No traces produced.",
			                     ccMainAppInterface::WRN_CONSOLE_MESSAGE);
			delete facetsGroup;
			facetsGroup = nullptr;
		}
		else
		{
			root->addChild(facetsGroup);
		}
	}

	// ---------------------------------------------------------------------
	// Stage 4 : Trace Clustering
	// ---------------------------------------------------------------------
	if (runStage4)
	{
		if (!facetsGroup || facetsGroup->getChildrenNumber() == 0)
		{
			m_app->dispToConsole("[qTracer] Stage 4 needs a non-empty facets group. Aborted.",
			                     ccMainAppInterface::ERR_CONSOLE_MESSAGE);
			finalizePipelineResult(root, pc, facetsGroup, tracesGroup, planesGroup, mergedGroup, firstStage);
			return;
		}
		announce(4, "Trace Clustering");
		tracesGroup = IdentifyFracture::TraceClustering(
			facetsGroup,
			dlg.traceConeRadius(),
			dlg.traceTwoTraceDist(),
			dlg.traceMinAngleDeg(),
			&progress);
		if (!tracesGroup || tracesGroup->getChildrenNumber() == 0)
		{
			m_app->dispToConsole("[qTracer] Trace Clustering produced no combined traces.",
			                     ccMainAppInterface::WRN_CONSOLE_MESSAGE);
			delete tracesGroup;
			tracesGroup = nullptr;
		}
		else
		{
			root->addChild(tracesGroup);
		}
	}

	// ---------------------------------------------------------------------
	// Stage 5 : Plane Fitting
	// ---------------------------------------------------------------------
	if (runStage5)
	{
		if (!tracesGroup || tracesGroup->getChildrenNumber() == 0)
		{
			m_app->dispToConsole("[qTracer] Stage 5 needs a non-empty traces group. Aborted.",
			                     ccMainAppInterface::ERR_CONSOLE_MESSAGE);
			finalizePipelineResult(root, pc, facetsGroup, tracesGroup, planesGroup, mergedGroup, firstStage);
			return;
		}
		announce(5, "Plane Fitting");
		planesGroup = IdentifyFracture::PlaneFitting(
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
	}

	// ---------------------------------------------------------------------
	// Stage 6 : Merge Coplanar Planes (iterative)
	// ---------------------------------------------------------------------
	if (runStage6)
	{
		if (!planesGroup || planesGroup->getChildrenNumber() == 0)
		{
			m_app->dispToConsole("[qTracer] Stage 6 needs a non-empty joint-planes group. Aborted.",
			                     ccMainAppInterface::ERR_CONSOLE_MESSAGE);
			finalizePipelineResult(root, pc, facetsGroup, tracesGroup, planesGroup, mergedGroup, firstStage);
			return;
		}
		announce(6, "Merge Coplanar Planes");
		mergedGroup = IdentifyFracture::MergeCoplanarPlanes(
			planesGroup,
			dlg.mergeMaxNormalAngleDeg(),
			dlg.mergeMaxPlaneDist(),
			dlg.mergeMaxPasses(),
			&progress);
		if (mergedGroup)
		{
			root->addChild(mergedGroup);
		}
	}

	finalizePipelineResult(root, pc, facetsGroup, tracesGroup, planesGroup, mergedGroup, firstStage);
}


void qTracer::finalizePipelineResult(ccHObject* root,
                                     ccPointCloud* pc,
                                     ccHObject* facetsGroup,
                                     ccHObject* tracesGroup,
                                     ccHObject* planesGroup,
                                     ccHObject* mergedGroup,
                                     int firstStage)
{
	// Only hide the source cloud when we actually touched it (stages 1-3).
	if (pc && firstStage <= 3)
		pc->setEnabled(false);

	if (root->getChildrenNumber() == 0)
	{
		delete root;
	}
	else
	{
		root->prepareDisplayForRefresh_recursive();
		m_app->addToDB(root);
	}

	m_app->dispToConsole(
		QString("[qTracer] Pipeline done — %1 facets, %2 combined traces, %3 joint planes, %4 merged planes.")
			.arg(facetsGroup ? facetsGroup->getChildrenNumber() : 0)
			.arg(tracesGroup ? tracesGroup->getChildrenNumber() : 0)
			.arg(planesGroup ? planesGroup->getChildrenNumber() : 0)
			.arg(mergedGroup ? mergedGroup->getChildrenNumber() : 0),
		ccMainAppInterface::STD_CONSOLE_MESSAGE);
}
