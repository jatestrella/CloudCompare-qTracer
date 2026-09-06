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

#include "ColorFilterDlg.h"

#include "HistogramRangeWidget.h"

//CCCoreLib
#include <CCConst.h>

//qCC_db
#include <ccPointCloud.h>

//qCC_plugins
#include <ccMainAppInterface.h>

//Qt
#include <QSignalBlocker>

//system
#include <algorithm>
#include <cmath>


ColorFilterDlg::ColorFilterDlg(ccPointCloud* cloud, ccMainAppInterface* app, QWidget* parent)
	: QDialog(parent)
	, Ui::ColorFilterDialog()
	, m_cloud(cloud)
	, m_app(app)
{
	setupUi(this);

	m_previewTimer.setSingleShot(true);
	m_previewTimer.setInterval(75);
	connect(&m_previewTimer, &QTimer::timeout, this, &ColorFilterDlg::applyPreview);

	m_histoTimer.setSingleShot(true);
	m_histoTimer.setInterval(120);
	connect(&m_histoTimer, &QTimer::timeout, this, &ColorFilterDlg::rebuildHistogram);

	// Generic linker: scale = int-slider units per 1.0 of spinbox value.
	// e.g. scale=100 for 0..1 coefficients, scale=1 for 0..765 C thresholds.
	// QSignalBlocker prevents the usual two-way feedback loop; preview is fired
	// manually from the slider side and via the dedicated connection from the
	// spinbox side.
	auto link = [this](QSlider* s, QDoubleSpinBox* sp, double scale)
	{
		connect(s, &QSlider::valueChanged, this, [this, sp, scale](int v)
		{
			QSignalBlocker blk(sp);
			sp->setValue(static_cast<double>(v) / scale);
			onThresholdChanged();
		});
		connect(sp, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, [s, scale](double v)
		{
			QSignalBlocker blk(s);
			s->setValue(static_cast<int>(std::lround(v * scale)));
		});
		connect(sp, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &ColorFilterDlg::onThresholdChanged);
	};

	// Coefficients: 0..100 slider ↔ 0.00..1.00 spinbox
	link(aSlider, aSpinBox, 100.0);
	link(bSlider, bSpinBox, 100.0);
	link(cSlider, cSpinBox, 100.0);

	// When coefficients change, the C range + histogram must be rebuilt.
	// The spinbox signals cover typed input; the slider signals cover drag
	// (where the spinbox valueChanged is suppressed by QSignalBlocker above).
	connect(aSpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &ColorFilterDlg::onCoefChanged);
	connect(bSpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &ColorFilterDlg::onCoefChanged);
	connect(cSpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &ColorFilterDlg::onCoefChanged);
	connect(aSlider,  &QSlider::valueChanged,                                this, &ColorFilterDlg::onCoefChanged);
	connect(bSlider,  &QSlider::valueChanged,                                this, &ColorFilterDlg::onCoefChanged);
	connect(cSlider,  &QSlider::valueChanged,                                this, &ColorFilterDlg::onCoefChanged);

	// C-threshold spinboxes drive preview and histogram cursors
	connect(cMinSpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
		[this](double v) { histogramWidget->setLower(v); onThresholdChanged(); });
	connect(cMaxSpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
		[this](double v) { histogramWidget->setUpper(v); onThresholdChanged(); });

	// Histogram cursor drags write back into the spinboxes (which trigger preview).
	connect(histogramWidget, &HistogramRangeWidget::lowerChanged, this, [this](double v)
	{
		QSignalBlocker b(cMinSpinBox);
		cMinSpinBox->setValue(v);
		onThresholdChanged();
	});
	connect(histogramWidget, &HistogramRangeWidget::upperChanged, this, [this](double v)
	{
		QSignalBlocker b(cMaxSpinBox);
		cMaxSpinBox->setValue(v);
		onThresholdChanged();
	});

	connect(resetButton, &QPushButton::clicked, this, &ColorFilterDlg::onResetClicked);

	// Initial setup
	histogramWidget->setRange(0.0, 765.0);
	histogramWidget->setLower(cMinSpinBox->value());
	histogramWidget->setUpper(cMaxSpinBox->value());
	rebuildHistogram();

	saveOriginalVisibility();
	applyPreview(); // initial pass so kept-label is populated
}


void ColorFilterDlg::onCoefChanged()
{
	// Theoretical max of C for the current a/b/c (each channel up to 255).
	const double coefSum = aSpinBox->value() + bSpinBox->value() + cSpinBox->value();
	const double newMax  = std::max(coefSum * 255.0, 1.0); // guard against coefSum == 0

	// Clamp the threshold spinboxes if they sit above the new achievable max.
	// The setValue calls propagate to the histogram cursors and fire preview.
	if (cMaxSpinBox->value() > newMax) cMaxSpinBox->setValue(newMax);
	if (cMinSpinBox->value() > newMax) cMinSpinBox->setValue(newMax);

	// Update the histogram's x-range so the bins + cursors map into the new scale.
	histogramWidget->setRange(0.0, newMax);

	// Rebuild the histogram bins against the new C values.
	m_histoTimer.start();
}


void ColorFilterDlg::rebuildHistogram()
{
	if (!m_cloud || !m_cloud->hasColors())
		return;

	const double a     = aSpinBox->value();
	const double bCoef = bSpinBox->value();
	const double cCoef = cSpinBox->value();

	constexpr int nBins = 128;
	const double xMin = 0.0;
	const double xMax = std::max((a + bCoef + cCoef) * 255.0, 1.0);
	std::vector<unsigned> bins(nBins, 0);

	const unsigned n = m_cloud->size();
	for (unsigned i = 0; i < n; ++i)
	{
		const ccColor::Rgba& col = m_cloud->getPointColor(i);
		double c = a * col.r + bCoef * col.g + cCoef * col.b;
		if (c < xMin) c = xMin;
		if (c > xMax) c = xMax;
		int bin = static_cast<int>((c - xMin) / (xMax - xMin) * nBins);
		if (bin >= nBins) bin = nBins - 1;
		if (bin < 0) bin = 0;
		++bins[bin];
	}
	histogramWidget->setHistogram(bins, xMin, xMax);
}


void ColorFilterDlg::saveOriginalVisibility()
{
	if (!m_cloud)
		return;

	m_hadVisibilityTable = m_cloud->isVisibilityTableInstantiated();
	if (m_hadVisibilityTable)
		m_originalVisibility = m_cloud->getTheVisibilityArray();

	// ensure array exists for our preview writes
	m_cloud->resetVisibilityArray();
}


void ColorFilterDlg::restoreOriginalVisibility()
{
	if (!m_cloud)
		return;

	if (m_hadVisibilityTable)
	{
		m_cloud->getTheVisibilityArray() = m_originalVisibility;
	}
	else
	{
		m_cloud->unallocateVisibilityArray();
	}
	m_cloud->redrawDisplay();
	if (m_app)
		m_app->redrawAll();
}


void ColorFilterDlg::onThresholdChanged()
{
	m_previewTimer.start();
}


void ColorFilterDlg::applyPreview()
{
	if (!m_cloud || !m_cloud->hasColors())
		return;

	const double a     = aSpinBox->value();
	const double bCoef = bSpinBox->value();
	const double cCoef = cSpinBox->value();
	const double cLo   = cMinSpinBox->value();
	const double cHi   = cMaxSpinBox->value();

	if (!m_cloud->isVisibilityTableInstantiated())
		m_cloud->resetVisibilityArray();
	auto& vis = m_cloud->getTheVisibilityArray();

	const unsigned n = m_cloud->size();
	if (vis.size() != n)
		return;

	unsigned kept = 0;
	for (unsigned i = 0; i < n; ++i)
	{
		const ccColor::Rgba& col = m_cloud->getPointColor(i);
		const double c = a * col.r + bCoef * col.g + cCoef * col.b;
		const bool ok = (c >= cLo && c <= cHi);
		vis[i] = ok ? CCCoreLib::POINT_VISIBLE : CCCoreLib::POINT_HIDDEN;
		if (ok) ++kept;
	}

	const double pct = n ? 100.0 * static_cast<double>(kept) / static_cast<double>(n) : 0.0;
	keptLabel->setText(QString("Kept: %1 / %2 (%3%)")
		.arg(kept)
		.arg(n)
		.arg(pct, 0, 'f', 1));

	m_cloud->redrawDisplay();
	if (m_app)
		m_app->redrawAll();
}


void ColorFilterDlg::onResetClicked()
{
	aSpinBox->setValue(1.0);
	bSpinBox->setValue(1.0);
	cSpinBox->setValue(1.0);
	cMinSpinBox->setValue(0.0);
	cMaxSpinBox->setValue(765.0);
	// the valueChanged signals above already scheduled a preview
}


void ColorFilterDlg::reject()
{
	restoreOriginalVisibility();
	QDialog::reject();
}
