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

#include "TraceLengthFilterDlg.h"

#include "HistogramRangeWidget.h"

//qCC_db
#include <ccHObject.h>
#include <ccPolyline.h>

//qCC_plugins
#include <ccMainAppInterface.h>

//Qt
#include <QSignalBlocker>

//system
#include <algorithm>
#include <cmath>
#include <queue>


namespace
{
	//! Breadth-first search for the shallowest 2-vertex polyline under \p obj
	//! (same convention as the P21 dialog: a node's own MergedPolyline wins over
	//! deeper member/piece polylines).
	const ccPolyline* findTracePolyline(const ccHObject* obj)
	{
		if (!obj) return nullptr;
		std::queue<const ccHObject*> q;
		q.push(obj);
		while (!q.empty())
		{
			const ccHObject* n = q.front();
			q.pop();
			if (n->isKindOf(CC_TYPES::POLY_LINE))
			{
				const ccPolyline* p = static_cast<const ccPolyline*>(n);
				if (p->size() == 2) return p;
			}
			for (unsigned i = 0; i < n->getChildrenNumber(); ++i)
				q.push(n->getChild(i));
		}
		return nullptr;
	}

	double polylineLength(const ccPolyline* poly)
	{
		if (!poly) return 0.0;
		const unsigned n = poly->size();
		if (n < 2) return 0.0;
		double total = 0.0;
		CCVector3 prev;
		poly->getPoint(0, prev);
		for (unsigned i = 1; i < n; ++i)
		{
			CCVector3 cur;
			poly->getPoint(i, cur);
			total += static_cast<double>((cur - prev).norm());
			prev = cur;
		}
		return total;
	}
}


TraceLengthFilterDlg::TraceLengthFilterDlg(ccHObject* tracesGroup, ccMainAppInterface* app, QWidget* parent)
	: QDialog(parent)
	, Ui::TraceLengthFilterDialog()
	, m_group(tracesGroup)
	, m_app(app)
{
	setupUi(this);

	m_previewTimer.setSingleShot(true);
	m_previewTimer.setInterval(60);
	connect(&m_previewTimer, &QTimer::timeout, this, &TraceLengthFilterDlg::applyPreview);

	gatherTraces();

	// spinboxes drive the cursors + preview
	connect(minLenSpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
		[this](double v) { histogramWidget->setLower(v); onThresholdChanged(); });
	connect(maxLenSpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
		[this](double v) { histogramWidget->setUpper(v); onThresholdChanged(); });

	// cursor drags write back into the spinboxes (which fire the preview)
	connect(histogramWidget, &HistogramRangeWidget::lowerChanged, this, [this](double v)
	{
		QSignalBlocker b(minLenSpinBox);
		minLenSpinBox->setValue(v);
		onThresholdChanged();
	});
	connect(histogramWidget, &HistogramRangeWidget::upperChanged, this, [this](double v)
	{
		QSignalBlocker b(maxLenSpinBox);
		maxLenSpinBox->setValue(v);
		onThresholdChanged();
	});

	// initialise range from the data (default keeps everything). Round the upper
	// bound UP to the spinbox precision (4 decimals) so the longest trace isn't
	// excluded by a rounded-down max (which would show "N-1 / N" before any drag).
	double maxLen = 0.0;
	for (const auto& t : m_traces)
		maxLen = std::max(maxLen, t.length);
	if (maxLen <= 0.0)
		maxLen = 1.0;
	const double maxLenUp = std::ceil(maxLen * 10000.0) / 10000.0;

	{
		QSignalBlocker b1(minLenSpinBox), b2(maxLenSpinBox);
		minLenSpinBox->setValue(0.0);
		maxLenSpinBox->setValue(maxLenUp);
	}
	histogramWidget->setRange(0.0, maxLenUp);
	histogramWidget->setLower(0.0);
	histogramWidget->setUpper(maxLenUp);
	buildHistogram();

	applyPreview();
}


void TraceLengthFilterDlg::gatherTraces()
{
	m_traces.clear();
	m_originalEnabled.clear();
	if (!m_group)
		return;
	for (unsigned i = 0; i < m_group->getChildrenNumber(); ++i)
	{
		ccHObject* child = m_group->getChild(i);
		const ccPolyline* p = findTracePolyline(child);
		if (!p)
			continue;
		TraceEntry e;
		e.node   = child;
		e.poly   = p;
		e.length = polylineLength(p);
		m_traces.push_back(e);
		m_originalEnabled.push_back(child->isEnabled());
	}
}


void TraceLengthFilterDlg::buildHistogram()
{
	if (m_traces.empty())
		return;
	double maxLen = 0.0;
	for (const auto& t : m_traces)
		maxLen = std::max(maxLen, t.length);
	if (maxLen <= 0.0)
		maxLen = 1.0;

	constexpr int nBins = 60;
	std::vector<unsigned> bins(nBins, 0);
	for (const auto& t : m_traces)
	{
		int b = static_cast<int>(t.length / maxLen * nBins);
		if (b >= nBins) b = nBins - 1;
		if (b < 0)      b = 0;
		++bins[b];
	}
	histogramWidget->setHistogram(bins, 0.0, maxLen);
}


void TraceLengthFilterDlg::onThresholdChanged()
{
	m_previewTimer.start();
}


void TraceLengthFilterDlg::applyPreview()
{
	const double lo = minLenSpinBox->value();
	const double hi = maxLenSpinBox->value();
	unsigned kept = 0;
	for (const auto& t : m_traces)
	{
		const bool ok = (t.length >= lo && t.length <= hi);
		if (t.node) t.node->setEnabled(ok);
		if (ok) ++kept;
	}
	keptLabel->setText(QString("Kept: %1 / %2").arg(kept).arg(static_cast<unsigned>(m_traces.size())));
	if (m_group) m_group->redrawDisplay();
	if (m_app)   m_app->redrawAll();
}


void TraceLengthFilterDlg::restoreOriginalEnabled()
{
	for (size_t i = 0; i < m_traces.size() && i < m_originalEnabled.size(); ++i)
		if (m_traces[i].node)
			m_traces[i].node->setEnabled(m_originalEnabled[i]);
	if (m_group) m_group->redrawDisplay();
	if (m_app)   m_app->redrawAll();
}


void TraceLengthFilterDlg::accept()
{
	restoreOriginalEnabled();
	QDialog::accept();
}


void TraceLengthFilterDlg::reject()
{
	restoreOriginalEnabled();
	QDialog::reject();
}
