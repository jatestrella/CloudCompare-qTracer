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

#include "HistogramRangeWidget.h"

#include <QMouseEvent>

#include <algorithm>
#include <cmath>


HistogramRangeWidget::HistogramRangeWidget(QWidget* parent)
	: QCustomPlot(parent)
{
	setInteractions(QCP::Interactions());      // disable default pan/zoom/selection
	setMinimumHeight(120);

	axisRect()->setAutoMargins(QCP::msNone);
	axisRect()->setMargins(QMargins(4, 4, 4, 14));
	xAxis->setTicks(true);
	xAxis->setTickLabels(true);
	xAxis->setBasePen(QPen(Qt::black));
	yAxis->setVisible(false);
	xAxis2->setVisible(false);
	yAxis2->setVisible(false);

	m_bars = new QCPBars(xAxis, yAxis);
	m_bars->setPen(QPen(QColor(70, 120, 200)));
	m_bars->setBrush(QBrush(QColor(120, 170, 230, 180)));
	m_bars->setWidthType(QCPBars::wtPlotCoords);

	// highlight rect (in-range area)
	m_highlight = new QCPItemRect(this);
	m_highlight->setLayer("background");
	m_highlight->setPen(QPen(Qt::NoPen));
	m_highlight->setBrush(QBrush(QColor(255, 230, 100, 90)));
	m_highlight->topLeft->setType(QCPItemPosition::ptPlotCoords);
	m_highlight->bottomRight->setType(QCPItemPosition::ptPlotCoords);

	// two vertical cursor lines (defined by two points, direction inferred)
	auto makeLine = [this](QColor col) {
		auto* l = new QCPItemStraightLine(this);
		QPen pen(col);
		pen.setWidth(2);
		l->setPen(pen);
		l->point1->setType(QCPItemPosition::ptPlotCoords);
		l->point2->setType(QCPItemPosition::ptPlotCoords);
		return l;
	};
	m_lowerLine = makeLine(QColor(40, 140, 40));
	m_upperLine = makeLine(QColor(200, 40, 40));

	setRange(0.0, 1.0);
}


void HistogramRangeWidget::setRange(double xMin, double xMax)
{
	if (xMax <= xMin)
		xMax = xMin + 1.0;
	m_xMin = xMin;
	m_xMax = xMax;
	xAxis->setRange(xMin, xMax);

	m_lower = clampToRange(m_lower);
	m_upper = clampToRange(m_upper);

	updateCursors();
	replot(rpQueuedReplot);
}


void HistogramRangeWidget::setHistogram(const std::vector<unsigned>& bins, double binMin, double binMax)
{
	const int n = static_cast<int>(bins.size());
	if (n == 0 || binMax <= binMin)
	{
		m_bars->setData(QVector<double>(), QVector<double>());
		yAxis->setRange(0, 1);
		replot(rpQueuedReplot);
		return;
	}

	const double binWidth = (binMax - binMin) / n;
	QVector<double> xs(n), ys(n);
	unsigned maxBin = 0;
	for (int i = 0; i < n; ++i)
	{
		xs[i] = binMin + binWidth * (i + 0.5);
		ys[i] = static_cast<double>(bins[i]);
		if (bins[i] > maxBin) maxBin = bins[i];
	}
	m_bars->setWidth(binWidth * 0.95);
	m_bars->setData(xs, ys);

	const double yTop = maxBin > 0 ? maxBin * 1.05 : 1.0;
	yAxis->setRange(0, yTop);

	updateCursors();
	replot(rpQueuedReplot);
}


void HistogramRangeWidget::setLower(double v)
{
	v = clampToRange(v);
	if (v > m_upper) v = m_upper;
	m_lower = v;
	updateCursors();
	replot(rpQueuedReplot);
}


void HistogramRangeWidget::setUpper(double v)
{
	v = clampToRange(v);
	if (v < m_lower) v = m_lower;
	m_upper = v;
	updateCursors();
	replot(rpQueuedReplot);
}


void HistogramRangeWidget::updateCursors()
{
	m_lowerLine->point1->setCoords(m_lower, 0);
	m_lowerLine->point2->setCoords(m_lower, 1);
	m_upperLine->point1->setCoords(m_upper, 0);
	m_upperLine->point2->setCoords(m_upper, 1);

	const double yLo = yAxis->range().lower;
	const double yHi = yAxis->range().upper;
	m_highlight->topLeft->setCoords(m_lower, yHi);
	m_highlight->bottomRight->setCoords(m_upper, yLo);
}


double HistogramRangeWidget::pixelToAxis(int px) const
{
	return xAxis->pixelToCoord(px);
}


double HistogramRangeWidget::clampToRange(double v) const
{
	return std::max(m_xMin, std::min(m_xMax, v));
}


void HistogramRangeWidget::mousePressEvent(QMouseEvent* event)
{
	if (event->button() != Qt::LeftButton)
	{
		QCustomPlot::mousePressEvent(event);
		return;
	}

	const double cx = pixelToAxis(event->pos().x());

	// pick closer handle (in axis coordinates)
	const double dL = std::abs(cx - m_lower);
	const double dU = std::abs(cx - m_upper);
	m_activeHandle = (dL <= dU) ? 0 : 1;

	// snap clicked handle to click position (so user can click-to-jump)
	if (m_activeHandle == 0)
	{
		const double v = std::min(clampToRange(cx), m_upper);
		if (v != m_lower) { m_lower = v; emit lowerChanged(m_lower); }
	}
	else
	{
		const double v = std::max(clampToRange(cx), m_lower);
		if (v != m_upper) { m_upper = v; emit upperChanged(m_upper); }
	}

	updateCursors();
	replot(rpQueuedReplot);
	event->accept();
}


void HistogramRangeWidget::mouseMoveEvent(QMouseEvent* event)
{
	if (m_activeHandle < 0)
	{
		QCustomPlot::mouseMoveEvent(event);
		return;
	}

	const double cx = pixelToAxis(event->pos().x());
	if (m_activeHandle == 0)
	{
		const double v = std::min(clampToRange(cx), m_upper);
		if (v != m_lower) { m_lower = v; emit lowerChanged(m_lower); }
	}
	else
	{
		const double v = std::max(clampToRange(cx), m_lower);
		if (v != m_upper) { m_upper = v; emit upperChanged(m_upper); }
	}

	updateCursors();
	replot(rpQueuedReplot);
	event->accept();
}


void HistogramRangeWidget::mouseReleaseEvent(QMouseEvent* event)
{
	if (m_activeHandle >= 0)
	{
		m_activeHandle = -1;
		event->accept();
		return;
	}
	QCustomPlot::mouseReleaseEvent(event);
}
