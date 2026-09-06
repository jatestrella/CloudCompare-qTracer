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

#ifndef QTRACER_DISCLAIMER_DIALOG_HEADER
#define QTRACER_DISCLAIMER_DIALOG_HEADER

#include <ui_disclaimerDlg.h>

//qCC_plugins
#include <ccMainAppInterface.h>

//Qt
#include <QMainWindow>

//! Disclaimer dialog shown the first time any qTracer action runs in a session.
class DisclaimerDialog : public QDialog, public Ui::DisclaimerDialog
{
public:
	//! Default constructor
	DisclaimerDialog(QWidget* parent = nullptr)
		: QDialog(parent)
		, Ui::DisclaimerDialog()
	{
		setupUi(this);

		// The raw logo PNG can be much larger than the dialog's logo slot. Scale
		// it here (keeping aspect ratio) so it sits nicely next to the text.
		if (logo && !logo->pixmap().isNull())
		{
			constexpr int maxSide = 120;
			logo->setPixmap(logo->pixmap().scaled(maxSide, maxSide,
				Qt::KeepAspectRatio, Qt::SmoothTransformation));
			logo->setScaledContents(false);
		}
	}
};

//whether disclaimer has already been displayed (and accepted) or not
static bool s_disclaimerAccepted = false;

static bool ShowDisclaimer(ccMainAppInterface* app)
{
	if (!s_disclaimerAccepted)
	{
		//if the user "cancels" it, then he refuses the diclaimer!
		s_disclaimerAccepted = DisclaimerDialog(app ? app->getMainWindow() : 0).exec();
	}
	
	return s_disclaimerAccepted;
}

#endif //QTRACER_DISCLAIMER_DIALOG_HEADER
