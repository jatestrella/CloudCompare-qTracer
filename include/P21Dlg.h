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

#include <ui_P21Dlg.h>

class ccHObject;
class ccMainAppInterface;

//! Computes P21 = total-trace-length / outcrop-area from entities picked out of the DB tree.
/** On construction, scans the DB for candidate traces groups (any node whose
 *  descendants include 2-vertex polylines — matches both qTracer stage-3
 *  `[facets]` groups and stage-4 `Traces` groups) and candidate area meshes
 *  (any `ccMesh`). The user picks one of each (or overrides the area with a
 *  manual value), clicks Compute, and the result is shown in-place.
 */
class P21Dlg : public QDialog, public Ui::P21Dialog
{
	Q_OBJECT
public:
	explicit P21Dlg(ccMainAppInterface* app, QWidget* parent = nullptr);

private slots:
	void onCompute();
	void onManualAreaToggled(bool checked);

private:
	void populateCombos();
	void preselectFromCurrentSelection();

	ccMainAppInterface* m_app = nullptr;
};
