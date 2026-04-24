#include "OutcropAreaDlg.h"

#include "OutcropArea.h"

//qCC_db
#include <ccMesh.h>
#include <ccPointCloud.h>
#include <ccProgressDialog.h>

//qCC_plugins
#include <ccMainAppInterface.h>

//Qt
#include <QApplication>
#include <QLocale>
#include <QMainWindow>


OutcropAreaDlg::OutcropAreaDlg(ccPointCloud* cloud, ccMainAppInterface* app, QWidget* parent)
	: QDialog(parent)
	, Ui::OutcropAreaDialog()
	, m_cloud(cloud)
	, m_app(app)
{
	setupUi(this);

	connect(computeButton, &QPushButton::clicked, this, &OutcropAreaDlg::onCompute);
	connect(closeButton,   &QPushButton::clicked, this, &QDialog::accept);
	connect(partitionerComboBox, QOverload<int>::of(&QComboBox::currentIndexChanged),
	        paramsStack, &QStackedWidget::setCurrentIndex);
}


void OutcropAreaDlg::onCompute()
{
	if (!m_cloud) return;

	ccProgressDialog progress(false, m_app ? m_app->getMainWindow() : nullptr);
	progress.setAutoClose(true);
	progress.setWindowTitle(tr("qTracer | Computing outcrop area"));

	OutcropArea::Params p;
	p.partitioner      = (partitionerComboBox->currentIndex() == 1)
	                     ? OutcropArea::KdTree : OutcropArea::Octree;
	p.cellSize         = cellSizeSpinBox->value();
	p.minPointsPerCell = static_cast<unsigned>(minPointsSpinBox->value());
	p.minPlanarity     = minPlanaritySpinBox->value();
	p.maxError         = maxErrorSpinBox->value();
	p.minPointsPerLeaf = static_cast<unsigned>(minPointsKdSpinBox->value());
	p.buildPatchMesh   = addPatchMeshCheckBox->isChecked();

	QApplication::setOverrideCursor(Qt::WaitCursor);
	OutcropArea::Result r;
	const bool ok = OutcropArea::Compute(m_cloud, p, r, &progress);
	QApplication::restoreOverrideCursor();

	if (!ok)
	{
		if (m_app)
			m_app->dispToConsole("[qTracer] Outcrop area computation failed.",
			                     ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		return;
	}

	// Populate shared result labels
	resultsGroup->setEnabled(true);
	totalAreaLabel         ->setText(QString("%1").arg(r.totalArea, 0, 'f', 4));
	validCellsLabel        ->setText(QString("%1 / %2").arg(r.validCells).arg(r.totalCellsAtLevel));
	rejectedByCountLabel   ->setText(QString("%1").arg(r.rejectedByCount));
	rejectedDegenerateLabel->setText(QString("%1").arg(r.rejectedDegenerate));

	// Partitioner-specific labels
	if (p.partitioner == OutcropArea::Octree)
	{
		cellSizeReportLabel      ->setText(tr("Actual cell size (octree level %1)").arg(r.octreeLevel));
		actualCellSizeLabel      ->setText(QString("%1").arg(r.actualCellSize, 0, 'f', 4));
		rejectedPlanarityCaption ->setText(tr("Rejected (low planarity)"));
		rejectedByPlanarityLabel ->setText(QString("%1").arg(r.rejectedByPlanarity));
	}
	else
	{
		cellSizeReportLabel      ->setText(tr("Max leaf fit error (RMS)"));
		actualCellSizeLabel      ->setText(QString("%1").arg(r.maxLeafError, 0, 'f', 5));
		rejectedPlanarityCaption ->setText(tr("Rejected (low planarity)"));
		rejectedByPlanarityLabel ->setText(tr("— (built into split)"));
	}

	// Console mirror
	if (m_app)
	{
		QString msg;
		if (p.partitioner == OutcropArea::Octree)
		{
			msg = QString("[qTracer] Outcrop area (octree) = %1 (valid cells %2 / %3, rejected: %4 count / %5 planarity / %6 degenerate; actual cell size %7, level %8)")
				.arg(r.totalArea, 0, 'f', 4)
				.arg(r.validCells).arg(r.totalCellsAtLevel)
				.arg(r.rejectedByCount).arg(r.rejectedByPlanarity).arg(r.rejectedDegenerate)
				.arg(r.actualCellSize, 0, 'f', 4)
				.arg(r.octreeLevel);
		}
		else
		{
			msg = QString("[qTracer] Outcrop area (kd-tree) = %1 (valid leaves %2 / %3, rejected: %4 count / %5 degenerate; max leaf RMS %6)")
				.arg(r.totalArea, 0, 'f', 4)
				.arg(r.validCells).arg(r.totalCellsAtLevel)
				.arg(r.rejectedByCount).arg(r.rejectedDegenerate)
				.arg(r.maxLeafError, 0, 'f', 5);
		}
		m_app->dispToConsole(msg, ccMainAppInterface::STD_CONSOLE_MESSAGE);
	}

	// Attach the patch mesh to the DB tree next to the source cloud.
	if (p.buildPatchMesh && r.mesh && r.mesh->size() > 0 && m_app)
	{
		const QString tag = (p.partitioner == OutcropArea::Octree) ? "octree" : "kdtree";
		r.mesh->setName(QString("Area patches [%1, %2] (%3 m² in %4 cells)")
			.arg(m_cloud->getName())
			.arg(tag)
			.arg(r.totalArea, 0, 'f', 4)
			.arg(r.validCells));
		if (ccHObject* parent = m_cloud->getParent())
			parent->addChild(r.mesh);
		m_app->addToDB(r.mesh);
	}
	else if (r.mesh)
	{
		delete r.mesh;
	}
}
