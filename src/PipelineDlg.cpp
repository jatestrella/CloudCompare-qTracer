#include "PipelineDlg.h"

#include <QCheckBox>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFile>
#include <QFileDialog>
#include <QGroupBox>
#include <QHash>
#include <QMessageBox>
#include <QPushButton>
#include <QRegularExpression>
#include <QSettings>
#include <QSpinBox>
#include <QTextStream>
#include <QVector>

namespace
{
	//! Ordered (key, widget) table covering every stage toggle + parameter, so
	//! export / import / QSettings all speak the same keys.
	QVector<QPair<QString, QWidget*>> paramList(PipelineDlg* d)
	{
		return {
			{ "stage1", d->eigenGroup }, { "stage2", d->dbscanGroup }, { "stage3", d->tracesGroup },
			{ "stage4", d->clusteringGroup }, { "stage5", d->planeGroup }, { "stage6", d->mergeGroup },
			{ "kernelRadius",        d->kernelRadiusSpinBox },
			{ "autoScaleEnabled",    d->autoScaleCheckBox },
			{ "autoScaleRMin",       d->autoScaleRMinSpinBox },
			{ "autoScaleRMax",       d->autoScaleRMaxSpinBox },
			{ "autoScaleSteps",      d->autoScaleStepsSpinBox },
			{ "autoScaleMinPts",     d->autoScaleMinPtsSpinBox },
			{ "autoScaleCriterion",  d->autoScaleCriterionComboBox },
			{ "dbscanRadius",        d->dbscanRadiusSpinBox },
			{ "dbscanMinPoints",     d->dbscanMinPointsSpinBox },
			{ "dbscanMaxAngle",      d->dbscanMaxAngleSpinBox },
			{ "dbscanLinearity",     d->dbscanLinearitySpinBox },
			{ "dbscanSearchType",    d->dbscanSearchTypeComboBox },
			{ "randomColors",        d->randomColorsCheckBox },
			{ "coneRadius",          d->coneRadiusSpinBox },
			{ "twoTraceDist",        d->twoTraceDistSpinBox },
			{ "clusterMinAngle",     d->clusterMinAngleSpinBox },
			{ "planeIntersectionDist", d->planeIntersectionDistSpinBox },
			{ "planeMinTraceLen",    d->planeMinTraceLenSpinBox },
			{ "planeMinAngle",       d->planeMinAngleSpinBox },
			{ "planeMaxEndPointDist", d->planeMaxEndPointDistSpinBox },
			{ "mergeMaxNormalAngle", d->mergeMaxNormalAngleSpinBox },
			{ "mergeMaxPlaneDist",   d->mergeMaxPlaneDistSpinBox },
			{ "mergeMaxPasses",      d->mergeMaxPassesSpinBox },
			{ "mergeDropUnmerged",   d->mergeDropUnmergedCheckBox },
		};
	}

	QString widgetToStr(QWidget* w)
	{
		if (auto* gb = qobject_cast<QGroupBox*>(w))      return gb->isChecked() ? "1" : "0";
		if (auto* cb = qobject_cast<QCheckBox*>(w))      return cb->isChecked() ? "1" : "0";
		if (auto* ds = qobject_cast<QDoubleSpinBox*>(w)) return QString::number(ds->value(), 'g', 10);
		if (auto* sp = qobject_cast<QSpinBox*>(w))       return QString::number(sp->value());
		if (auto* co = qobject_cast<QComboBox*>(w))      return QString::number(co->currentIndex());
		return QString();
	}

	void strToWidget(QWidget* w, const QString& v)
	{
		if (auto* gb = qobject_cast<QGroupBox*>(w))      { gb->setChecked(v.toInt() != 0); return; }
		if (auto* cb = qobject_cast<QCheckBox*>(w))      { cb->setChecked(v.toInt() != 0); return; }
		if (auto* ds = qobject_cast<QDoubleSpinBox*>(w)) { ds->setValue(v.toDouble()); return; }
		if (auto* sp = qobject_cast<QSpinBox*>(w))       { sp->setValue(v.toInt()); return; }
		if (auto* co = qobject_cast<QComboBox*>(w))      { co->setCurrentIndex(v.toInt()); return; }
	}
}

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

	connect(exportParamsButton, &QPushButton::clicked, this, &PipelineDlg::onExportParams);
	connect(importParamsButton, &QPushButton::clicked, this, &PipelineDlg::onImportParams);

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
		autoScaleCheckBox          ->setChecked(s.value("autoScaleEnabled",  autoScaleCheckBox          ->isChecked()).toBool());
		autoScaleRMinSpinBox       ->setValue(s.value("autoScaleRMin",       autoScaleRMinSpinBox       ->value()).toDouble());
		autoScaleRMaxSpinBox       ->setValue(s.value("autoScaleRMax",       autoScaleRMaxSpinBox       ->value()).toDouble());
		autoScaleStepsSpinBox      ->setValue(s.value("autoScaleSteps",      autoScaleStepsSpinBox      ->value()).toInt());
		autoScaleMinPtsSpinBox     ->setValue(s.value("autoScaleMinPts",     autoScaleMinPtsSpinBox     ->value()).toInt());
		autoScaleCriterionComboBox ->setCurrentIndex(s.value("autoScaleCriterion", autoScaleCriterionComboBox->currentIndex()).toInt());

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
		mergeDropUnmergedCheckBox  ->setChecked(s.value("mergeDropUnmerged", mergeDropUnmergedCheckBox  ->isChecked()).toBool());

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
		s.setValue("autoScaleEnabled",     autoScaleCheckBox->isChecked());
		s.setValue("autoScaleRMin",        autoScaleRMinSpinBox->value());
		s.setValue("autoScaleRMax",        autoScaleRMaxSpinBox->value());
		s.setValue("autoScaleSteps",       autoScaleStepsSpinBox->value());
		s.setValue("autoScaleMinPts",      autoScaleMinPtsSpinBox->value());
		s.setValue("autoScaleCriterion",   autoScaleCriterionComboBox->currentIndex());

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
		s.setValue("mergeDropUnmerged",    mergeDropUnmergedCheckBox->isChecked());

		s.endGroup();
	}

	QDialog::accept();
}


void PipelineDlg::onExportParams()
{
	QString fn = QFileDialog::getSaveFileName(this, tr("Export pipeline parameters"),
		"qtracer_params.txt", tr("Text files (*.txt);;All files (*)"));
	if (fn.isEmpty())
		return;

	QFile f(fn);
	if (!f.open(QIODevice::WriteOnly | QIODevice::Text))
	{
		QMessageBox::warning(this, tr("Export failed"), tr("Could not open the file for writing."));
		return;
	}
	QTextStream out(&f);
	out << "# qTracer pipeline parameters (key <TAB> value)\n";
	for (const auto& kv : paramList(this))
		out << kv.first << '\t' << widgetToStr(kv.second) << '\n';
	f.close();
}


void PipelineDlg::onImportParams()
{
	QString fn = QFileDialog::getOpenFileName(this, tr("Import pipeline parameters"),
		QString(), tr("Text files (*.txt);;All files (*)"));
	if (fn.isEmpty())
		return;

	QFile f(fn);
	if (!f.open(QIODevice::ReadOnly | QIODevice::Text))
	{
		QMessageBox::warning(this, tr("Import failed"), tr("Could not open the file for reading."));
		return;
	}

	QHash<QString, QWidget*> map;
	for (const auto& kv : paramList(this))
		map.insert(kv.first, kv.second);

	QTextStream in(&f);
	int applied = 0, unknown = 0;
	while (!in.atEnd())
	{
		const QString line = in.readLine().trimmed();
		if (line.isEmpty() || line.startsWith('#'))
			continue;
		const QStringList parts = line.split(QRegularExpression("\\s+"), Qt::SkipEmptyParts);
		if (parts.size() < 2)
			continue;
		auto it = map.find(parts[0]);
		if (it != map.end()) { strToWidget(it.value(), parts[1]); ++applied; }
		else                 { ++unknown; }
	}
	f.close();

	QMessageBox::information(this, tr("Import parameters"),
		tr("Applied %1 parameter(s)%2.")
			.arg(applied)
			.arg(unknown > 0 ? tr(", %1 unknown key(s) ignored").arg(unknown) : QString()));
}
