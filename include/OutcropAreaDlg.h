#pragma once

#include <ui_OutcropAreaDlg.h>

class ccPointCloud;
class ccMainAppInterface;

//! Combined input / result dialog for the outcrop-area action.
/** The user tweaks parameters, hits "Compute", results are filled in-place.
 *  If the patch-mesh checkbox is on, a triangulated mesh of the counted
 *  polygons is added as a sibling of the source cloud in the DB tree.
 */
class OutcropAreaDlg : public QDialog, public Ui::OutcropAreaDialog
{
	Q_OBJECT
public:
	OutcropAreaDlg(ccPointCloud* cloud, ccMainAppInterface* app, QWidget* parent = nullptr);

private slots:
	void onCompute();

private:
	ccPointCloud*       m_cloud = nullptr;
	ccMainAppInterface* m_app   = nullptr;
};
