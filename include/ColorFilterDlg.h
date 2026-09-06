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

#include <ui_ColorFilterDlg.h>

#include <QDialog>
#include <QTimer>
#include <vector>

class ccPointCloud;
class ccMainAppInterface;

//! Interactive C = a·R + b·G + c·B threshold filter with live 3D preview.
/** a, b, c ∈ [0, 1]. Points are kept iff Cmin ≤ C ≤ Cmax. To filter a single channel,
 *  set the other two coefficients to 0 (e.g. a=1, b=c=0 → filter by R).
 *
 *  On construction the current visibility state of the cloud is saved and the cloud's
 *  visibility array is driven by the dialog's thresholds. On reject() the original
 *  visibility is restored. On accept() the caller is responsible for materialising the
 *  filtered subset (the visibility array is left in its last-previewed state).
 */
class ColorFilterDlg : public QDialog, public Ui::ColorFilterDialog
{
	Q_OBJECT
public:
	ColorFilterDlg(ccPointCloud* cloud, ccMainAppInterface* app, QWidget* parent = nullptr);

	void reject() override;

	double coefA() const { return aSpinBox->value(); }
	double coefB() const { return bSpinBox->value(); }
	double coefC() const { return cSpinBox->value(); }
	double cMin()  const { return cMinSpinBox->value(); }
	double cMax()  const { return cMaxSpinBox->value(); }

private slots:
	void onThresholdChanged();
	void applyPreview();
	void onResetClicked();
	void onCoefChanged();
	void rebuildHistogram();

private:
	void saveOriginalVisibility();
	void restoreOriginalVisibility();

	ccPointCloud*          m_cloud = nullptr;
	ccMainAppInterface*    m_app   = nullptr;
	QTimer                 m_previewTimer;
	QTimer                 m_histoTimer;

	bool                           m_hadVisibilityTable = false;
	std::vector<unsigned char>     m_originalVisibility;
};
