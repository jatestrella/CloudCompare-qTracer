#include "PipelineDlg.h"

#include <QMessageBox>
#include <QSettings>

namespace
{
	constexpr const char* kStageSettingsGroup = "qTracer/PipelineDlg/stages";
	constexpr const char* kParamSettingsGroup = "qTracer/PipelineDlg/params";
}


PipelineDlg::PipelineDlg(QWidget* parent)
	: QDialog(parent)
	, Ui::PipelineDialog()
{
	setupUi(this);

	// Restore the last-used stage selection. Default to all-checked on first run.
	{
		QSettings s;
		s.beginGroup(kStageSettingsGroup);
		eigenGroup     ->setChecked(s.value("stage1", true).toBool());
		dbscanGroup    ->setChecked(s.value("stage2", true).toBool());
		tracesGroup    ->setChecked(s.value("stage3", true).toBool());
		clusteringGroup->setChecked(s.value("stage4", true).toBool());
		planeGroup     ->setChecked(s.value("stage5", true).toBool());
		mergeGroup     ->setChecked(s.value("stage6", true).toBool());
		s.endGroup();
	}

	// Restore the last-used parameter values. When a key is missing (first run,
	// or a freshly added parameter), the widget's .ui default survives.
	{
		QSettings s;
		s.beginGroup(kParamSettingsGroup);

		// Stage 1
		kernelRadiusSpinBox        ->setValue(s.value("kernelRadius",        kernelRadiusSpinBox        ->value()).toDouble());

		// Stage 2
		dbscanRadiusSpinBox        ->setValue(s.value("dbscanRadius",        dbscanRadiusSpinBox        ->value()).toDouble());
		dbscanMinPointsSpinBox     ->setValue(s.value("dbscanMinPoints",     dbscanMinPointsSpinBox     ->value()).toInt());
		dbscanMaxAngleSpinBox      ->setValue(s.value("dbscanMaxAngle",      dbscanMaxAngleSpinBox      ->value()).toDouble());
		dbscanLinearitySpinBox     ->setValue(s.value("dbscanLinearity",     dbscanLinearitySpinBox     ->value()).toDouble());
		dbscanSearchTypeComboBox   ->setCurrentIndex(s.value("dbscanSearchType", dbscanSearchTypeComboBox->currentIndex()).toInt());

		// Stage 3
		randomColorsCheckBox       ->setChecked(s.value("randomColors",      randomColorsCheckBox       ->isChecked()).toBool());

		// Stage 4
		coneRadiusSpinBox          ->setValue(s.value("coneRadius",          coneRadiusSpinBox          ->value()).toDouble());
		twoTraceDistSpinBox        ->setValue(s.value("twoTraceDist",        twoTraceDistSpinBox        ->value()).toDouble());
		clusterMinAngleSpinBox     ->setValue(s.value("clusterMinAngle",     clusterMinAngleSpinBox     ->value()).toDouble());

		// Stage 5
		planeIntersectionDistSpinBox->setValue(s.value("planeIntersectionDist", planeIntersectionDistSpinBox->value()).toDouble());
		planeMinTraceLenSpinBox    ->setValue(s.value("planeMinTraceLen",    planeMinTraceLenSpinBox    ->value()).toDouble());
		planeMinAngleSpinBox       ->setValue(s.value("planeMinAngle",       planeMinAngleSpinBox       ->value()).toDouble());
		planeMaxEndPointDistSpinBox->setValue(s.value("planeMaxEndPointDist",planeMaxEndPointDistSpinBox->value()).toDouble());

		// Stage 6
		mergeMaxNormalAngleSpinBox ->setValue(s.value("mergeMaxNormalAngle", mergeMaxNormalAngleSpinBox ->value()).toDouble());
		mergeMaxPlaneDistSpinBox   ->setValue(s.value("mergeMaxPlaneDist",   mergeMaxPlaneDistSpinBox   ->value()).toDouble());
		mergeMaxPassesSpinBox      ->setValue(s.value("mergeMaxPasses",      mergeMaxPassesSpinBox      ->value()).toInt());

		s.endGroup();
	}
}


bool PipelineDlg::stageEnabled(int n) const
{
	switch (n)
	{
		case 1: return stage1Enabled();
		case 2: return stage2Enabled();
		case 3: return stage3Enabled();
		case 4: return stage4Enabled();
		case 5: return stage5Enabled();
		case 6: return stage6Enabled();
		default: return false;
	}
}


int PipelineDlg::firstEnabledStage() const
{
	for (int i = 1; i <= 6; ++i)
		if (stageEnabled(i))
			return i;
	return 0;
}


int PipelineDlg::lastEnabledStage() const
{
	for (int i = 6; i >= 1; --i)
		if (stageEnabled(i))
			return i;
	return 0;
}


bool PipelineDlg::stagesAreContiguous() const
{
	const int first = firstEnabledStage();
	const int last  = lastEnabledStage();
	if (first == 0) return true; // no stages -> trivially contiguous (but caller must still reject)
	for (int i = first; i <= last; ++i)
		if (!stageEnabled(i))
			return false;
	return true;
}


void PipelineDlg::accept()
{
	if (firstEnabledStage() == 0)
	{
		QMessageBox::warning(this,
			tr("qTracer"),
			tr("At least one stage must be enabled."));
		return;
	}
	if (!stagesAreContiguous())
	{
		QMessageBox::warning(this,
			tr("qTracer"),
			tr("Enabled stages must be contiguous — no gaps allowed.\n"
			   "Example: 4-5-6 is OK; 4+6 (without 5) is not."));
		return;
	}

	// Persist the stage selection so the next run starts with the same ticks.
	{
		QSettings s;
		s.beginGroup(kStageSettingsGroup);
		s.setValue("stage1", stage1Enabled());
		s.setValue("stage2", stage2Enabled());
		s.setValue("stage3", stage3Enabled());
		s.setValue("stage4", stage4Enabled());
		s.setValue("stage5", stage5Enabled());
		s.setValue("stage6", stage6Enabled());
		s.endGroup();
	}

	// Persist the parameter values so the next run pre-fills them.
	{
		QSettings s;
		s.beginGroup(kParamSettingsGroup);

		s.setValue("kernelRadius",         kernelRadiusSpinBox->value());

		s.setValue("dbscanRadius",         dbscanRadiusSpinBox->value());
		s.setValue("dbscanMinPoints",      dbscanMinPointsSpinBox->value());
		s.setValue("dbscanMaxAngle",       dbscanMaxAngleSpinBox->value());
		s.setValue("dbscanLinearity",      dbscanLinearitySpinBox->value());
		s.setValue("dbscanSearchType",     dbscanSearchTypeComboBox->currentIndex());

		s.setValue("randomColors",         randomColorsCheckBox->isChecked());

		s.setValue("coneRadius",           coneRadiusSpinBox->value());
		s.setValue("twoTraceDist",         twoTraceDistSpinBox->value());
		s.setValue("clusterMinAngle",      clusterMinAngleSpinBox->value());

		s.setValue("planeIntersectionDist", planeIntersectionDistSpinBox->value());
		s.setValue("planeMinTraceLen",     planeMinTraceLenSpinBox->value());
		s.setValue("planeMinAngle",        planeMinAngleSpinBox->value());
		s.setValue("planeMaxEndPointDist", planeMaxEndPointDistSpinBox->value());

		s.setValue("mergeMaxNormalAngle",  mergeMaxNormalAngleSpinBox->value());
		s.setValue("mergeMaxPlaneDist",    mergeMaxPlaneDistSpinBox->value());
		s.setValue("mergeMaxPasses",       mergeMaxPassesSpinBox->value());

		s.endGroup();
	}

	QDialog::accept();
}
