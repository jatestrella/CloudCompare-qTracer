//##########################################################################
//#                                                                        #
//#                     CLOUDCOMPARE PLUGIN: qTracer                       #
//#                                                                        #
//#  This program is free software; you can redistribute it and/or modify  #
//#  it under the terms of the GNU General Public License as published by  #
//#  the Free Software Foundation; version 2 or later of the License.      #
//#                                                                        #
//#  This program is distributed in the hope that it will be useful,       #
//#  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
//#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the          #
//#  GNU General Public License for more details.                          #
//#                                                                        #
//#                 COPYRIGHT: Chia-Chi (Jate) Chiu, NTUT                  #
//#                                                                        #
//##########################################################################

#pragma once

#include <ui_PipelineDlg.h>

class PipelineDlg : public QDialog, public Ui::PipelineDialog
{
	Q_OBJECT
public:
	explicit PipelineDlg(QWidget* parent = nullptr);

	void accept() override;

private slots:
	void onExportParams();
	void onImportParams();

public:

	// --- Which stages to run ---
	bool stage1Enabled() const { return eigenGroup->isChecked(); }
	bool stage2Enabled() const { return dbscanGroup->isChecked(); }
	bool stage3Enabled() const { return tracesGroup->isChecked(); }
	bool stage4Enabled() const { return clusteringGroup->isChecked(); }
	bool stage5Enabled() const { return planeGroup->isChecked(); }
	bool stage6Enabled() const { return mergeGroup->isChecked(); }

	bool stageEnabled(int n) const;
	int  firstEnabledStage() const;  // 1..6, or 0 if none
	int  lastEnabledStage()  const;  // 1..6, or 0 if none
	bool stagesAreContiguous() const;

	// Stage 1
	double kernelRadius() const { return kernelRadiusSpinBox->value(); }
	bool     autoScaleEnabled()  const { return autoScaleCheckBox->isChecked(); }
	double   autoScaleRMin()     const { return autoScaleRMinSpinBox->value(); }
	double   autoScaleRMax()     const { return autoScaleRMaxSpinBox->value(); }
	unsigned autoScaleSteps()    const { return static_cast<unsigned>(autoScaleStepsSpinBox->value()); }
	unsigned autoScaleMinPts()   const { return static_cast<unsigned>(autoScaleMinPtsSpinBox->value()); }
	//! 0 = max linearity, 1 = min eigenentropy (combobox item 0 = eigenentropy)
	int      autoScaleCriterion() const { return autoScaleCriterionComboBox->currentIndex() == 0 ? 1 : 0; }

	// Stage 2
	double       dbscanRadius()           const { return dbscanRadiusSpinBox->value(); }
	unsigned int dbscanMinPoints()        const { return static_cast<unsigned int>(dbscanMinPointsSpinBox->value()); }
	double       dbscanMaxAngleDeg()      const { return dbscanMaxAngleSpinBox->value(); }
	double       dbscanLinearityThresh()  const { return dbscanLinearitySpinBox->value(); }
	int          dbscanSearchType()       const { return dbscanSearchTypeComboBox->currentIndex() == 0 ? 1 : 2; }

	// Stage 3
	bool randomColors() const { return randomColorsCheckBox->isChecked(); }

	// Stage 4
	double   traceConeRadius()   const { return coneRadiusSpinBox->value(); }
	double   traceTwoTraceDist() const { return twoTraceDistSpinBox->value(); }
	double   traceMinAngleDeg()  const { return clusterMinAngleSpinBox->value(); }
	unsigned traceMaxPasses()    const { return static_cast<unsigned>(traceMaxPassesSpinBox->value()); }

	// Stage 5
	double planeIntersectionLineDist()   const { return planeIntersectionDistSpinBox->value(); }
	double planeMinTraceLength()         const { return planeMinTraceLenSpinBox->value(); }
	double planeMinIntersectionAngleDeg() const { return planeMinAngleSpinBox->value(); }
	double planeMaxEndPointDist()        const { return planeMaxEndPointDistSpinBox->value(); }

	// Stage 6
	double   mergeMaxNormalAngleDeg() const { return mergeMaxNormalAngleSpinBox->value(); }
	double   mergeMaxPlaneDist()      const { return mergeMaxPlaneDistSpinBox->value(); }
	double   mergeMaxCentroidDist()   const { return mergeMaxCentroidDistSpinBox->value(); }
	unsigned mergeMaxPasses()         const { return static_cast<unsigned>(mergeMaxPassesSpinBox->value()); }
	bool     mergeDropUnmerged()      const { return mergeDropUnmergedCheckBox->isChecked(); }
};
