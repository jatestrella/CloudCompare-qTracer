#include "P21Dlg.h"

//CCCoreLib
#include <MeshSamplingTools.h>

//qCC_db
#include <ccHObject.h>
#include <ccMesh.h>
#include <ccPolyline.h>

//qCC_plugins
#include <ccMainAppInterface.h>

//Qt
#include <QVariant>

#include <vector>


namespace
{

//! Depth-first search for a 2-vertex polyline under \p obj.
/** 2-vertex is the "trace" convention used throughout qTracer — matches both
 *  stage-3 per-facet trace polylines and stage-4 combined-trace polylines.
 *  Contour polylines (many vertices) are skipped so a selected `[facets]`
 *  group reports trace length, not facet-contour length.
 */
const ccPolyline* findTracePolyline(const ccHObject* obj)
{
	if (!obj) return nullptr;
	if (obj->isKindOf(CC_TYPES::POLY_LINE))
	{
		const ccPolyline* p = static_cast<const ccPolyline*>(obj);
		if (p->size() == 2) return p;
	}
	for (unsigned i = 0; i < obj->getChildrenNumber(); ++i)
	{
		if (const ccPolyline* p = findTracePolyline(obj->getChild(i)))
			return p;
	}
	return nullptr;
}

double polylineLength(const ccPolyline* poly)
{
	if (!poly) return 0.0;
	const unsigned n = poly->size();
	if (n < 2) return 0.0;
	double total = 0.0;
	CCVector3 prev;
	poly->getPoint(0, prev);
	for (unsigned i = 1; i < n; ++i)
	{
		CCVector3 cur;
		poly->getPoint(i, cur);
		total += static_cast<double>((cur - prev).norm());
		prev = cur;
	}
	if (poly->isClosed() && n >= 2)
	{
		CCVector3 first, last;
		poly->getPoint(0, first);
		poly->getPoint(n - 1, last);
		total += static_cast<double>((first - last).norm());
	}
	return total;
}

//! Sum all trace-polyline lengths under the top-level children of \p group.
/** For each child (one "trace" in qTracer terminology), DFS for the first
 *  2-vertex polyline and add its length. Returns total length and counted
 *  traces via out params.
 */
void collectTraceStats(const ccHObject* group, double& totalLength, unsigned& countedTraces)
{
	totalLength   = 0.0;
	countedTraces = 0;
	if (!group) return;
	for (unsigned i = 0; i < group->getChildrenNumber(); ++i)
	{
		const ccPolyline* p = findTracePolyline(group->getChild(i));
		if (!p) continue;
		totalLength += polylineLength(p);
		++countedTraces;
	}
}

//! Count top-level children of \p group that contain a 2-vertex polyline
//! descendant (a "trace"). Used both as a membership test and as a count for
//! the combobox label.
unsigned countChildrenWithTracePolyline(const ccHObject* group)
{
	if (!group) return 0;
	unsigned n = 0;
	for (unsigned i = 0; i < group->getChildrenNumber(); ++i)
		if (findTracePolyline(group->getChild(i))) ++n;
	return n;
}

//! Walk the DB tree collecting (a) any `ccMesh` for area and (b) any node
//! whose children contain trace polylines.
/** Detection is polyline-centric: a "traces group" is any container whose
 *  top-level children each carry a 2-vertex polyline somewhere inside. This
 *  matches both stage-3 `[facets]` groups (polyline nested under each
 *  `ccFacet`) and stage-4 `Traces` groups (polyline nested under each
 *  `CombinedTrace N` node) — user picks which convention their P21 uses.
 *  Contour polylines (many vertices) are naturally filtered out by the
 *  2-vertex size check inside `findTracePolyline`.
 */
void collectCandidates(ccHObject* node,
                       std::vector<ccHObject*>& meshes,
                       std::vector<ccHObject*>& traceGroups)
{
	if (!node) return;

	if (node->isKindOf(CC_TYPES::MESH))
	{
		meshes.push_back(node);
	}
	else if (node->getChildrenNumber() > 0
	      && !node->isKindOf(CC_TYPES::POINT_CLOUD)
	      && !node->isKindOf(CC_TYPES::POLY_LINE)
	      && !node->isKindOf(CC_TYPES::FACET))
	{
		if (countChildrenWithTracePolyline(node) > 0)
			traceGroups.push_back(node);
	}

	for (unsigned i = 0; i < node->getChildrenNumber(); ++i)
		collectCandidates(node->getChild(i), meshes, traceGroups);
}

} // anonymous namespace


P21Dlg::P21Dlg(ccMainAppInterface* app, QWidget* parent)
	: QDialog(parent)
	, Ui::P21Dialog()
	, m_app(app)
{
	setupUi(this);

	connect(computeButton,        &QPushButton::clicked, this, &P21Dlg::onCompute);
	connect(closeButton,          &QPushButton::clicked, this, &QDialog::accept);
	connect(manualAreaCheckBox,   &QCheckBox::toggled,   this, &P21Dlg::onManualAreaToggled);

	populateCombos();
	preselectFromCurrentSelection();
}


void P21Dlg::onManualAreaToggled(bool checked)
{
	manualAreaSpinBox->setEnabled(checked);
	areaComboBox->setEnabled(!checked);
}


void P21Dlg::populateCombos()
{
	tracesComboBox->clear();
	areaComboBox->clear();
	if (!m_app) return;

	std::vector<ccHObject*> meshes, traceGroups;
	collectCandidates(m_app->dbRootObject(), meshes, traceGroups);

	for (ccHObject* g : traceGroups)
	{
		const unsigned n = countChildrenWithTracePolyline(g);
		tracesComboBox->addItem(QString("%1  (%2 polylines)").arg(g->getName()).arg(n),
		                        QVariant::fromValue(reinterpret_cast<void*>(g)));
	}
	for (ccHObject* m : meshes)
	{
		const unsigned ntri = static_cast<const ccMesh*>(m)->size();
		areaComboBox->addItem(QString("%1  (%2 tri)").arg(m->getName()).arg(ntri),
		                      QVariant::fromValue(reinterpret_cast<void*>(m)));
	}

	// Initial Compute-button state
	computeButton->setEnabled(tracesComboBox->count() > 0
		&& (areaComboBox->count() > 0 || manualAreaCheckBox->isChecked()));
}


void P21Dlg::preselectFromCurrentSelection()
{
	if (!m_app) return;
	const ccHObject::Container& sel = m_app->getSelectedEntities();
	for (const ccHObject* s : sel)
	{
		if (!s) continue;
		if (s->isKindOf(CC_TYPES::MESH))
		{
			const int idx = areaComboBox->findData(
				QVariant::fromValue(reinterpret_cast<void*>(const_cast<ccHObject*>(s))));
			if (idx >= 0) areaComboBox->setCurrentIndex(idx);
		}
		else
		{
			const int idx = tracesComboBox->findData(
				QVariant::fromValue(reinterpret_cast<void*>(const_cast<ccHObject*>(s))));
			if (idx >= 0) tracesComboBox->setCurrentIndex(idx);
		}
	}
}


void P21Dlg::onCompute()
{
	// Traces
	QVariant tv = tracesComboBox->currentData();
	ccHObject* traceGroup = tv.isValid()
		? static_cast<ccHObject*>(tv.value<void*>()) : nullptr;

	double totalLength = 0.0;
	unsigned nTraces   = 0;
	collectTraceStats(traceGroup, totalLength, nTraces);

	// Area
	double area = 0.0;
	QString areaSourceName;
	if (manualAreaCheckBox->isChecked())
	{
		area = manualAreaSpinBox->value();
		areaSourceName = tr("(manual value)");
	}
	else
	{
		QVariant av = areaComboBox->currentData();
		ccHObject* meshObj = av.isValid()
			? static_cast<ccHObject*>(av.value<void*>()) : nullptr;
		if (meshObj && meshObj->isKindOf(CC_TYPES::MESH))
		{
			ccMesh* mesh = static_cast<ccMesh*>(meshObj);
			area = CCCoreLib::MeshSamplingTools::computeMeshArea(mesh);
			areaSourceName = mesh->getName();
		}
	}

	resultsGroup->setEnabled(true);
	tracesCountLabel ->setText(QString("%1").arg(nTraces));
	totalLengthLabel ->setText(QString("%1").arg(totalLength, 0, 'f', 4));
	totalAreaLabel   ->setText(QString("%1").arg(area,        0, 'f', 4));
	if (area > 0.0)
		p21Label->setText(QString("%1  /unit").arg(totalLength / area, 0, 'f', 4));
	else
		p21Label->setText(tr("— (area is zero)"));

	if (m_app)
	{
		m_app->dispToConsole(
			QString("[qTracer] P21: L = %1 (from %2 traces in \"%3\"), A = %4 (%5), P21 = %6 /unit")
				.arg(totalLength, 0, 'f', 4)
				.arg(nTraces)
				.arg(traceGroup ? traceGroup->getName() : QString("—"))
				.arg(area, 0, 'f', 4)
				.arg(areaSourceName)
				.arg(area > 0.0 ? QString::number(totalLength / area, 'f', 4) : QString("—")),
			ccMainAppInterface::STD_CONSOLE_MESSAGE);
	}
}
