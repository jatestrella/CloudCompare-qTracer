#pragma once

#include <ui_PipelineDlg.h>

class PipelineDlg : public QDialog, public Ui::PipelineDialog
{
	Q_OBJECT
public:
	explicit PipelineDlg(QWidget* parent = nullptr);

	// Stage 1
	double kernelRadius() const { return kernelRadiusSpinBox->value(); }

	// Stage 2
	double       dbscanRadius()           const { return dbscanRadiusSpinBox->value(); }
	unsigned int dbscanMinPoints()        const { return static_cast<unsigned int>(dbscanMinPointsSpinBox->value()); }
	double       dbscanMaxAngleDeg()      const { return dbscanMaxAngleSpinBox->value(); }
	double       dbscanLinearityThresh()  const { return dbscanLinearitySpinBox->value(); }
	int          dbscanSearchType()       const { return dbscanSearchTypeComboBox->currentIndex() == 0 ? 1 : 2; }

	// Stage 3
	bool randomColors() const { return randomColorsCheckBox->isChecked(); }

	// Stage 4
	double traceConeRadius()   const { return coneRadiusSpinBox->value(); }
	double traceTwoTraceDist() const { return twoTraceDistSpinBox->value(); }
	double traceMinAngleDeg()  const { return clusterMinAngleSpinBox->value(); }

	// Stage 5
	double planeIntersectionLineDist()   const { return planeIntersectionDistSpinBox->value(); }
	double planeMinTraceLength()         const { return planeMinTraceLenSpinBox->value(); }
	double planeMinIntersectionAngleDeg() const { return planeMinAngleSpinBox->value(); }
	double planeMaxEndPointDist()        const { return planeMaxEndPointDistSpinBox->value(); }
};
