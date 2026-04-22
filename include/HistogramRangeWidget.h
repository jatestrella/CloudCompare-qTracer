#pragma once

#include <qcustomplot.h>

#include <vector>

class QCPBars;
class QCPItemStraightLine;
class QCPItemRect;

//! Histogram with two draggable vertical cursors for min / max range selection.
/** X-axis is a user-controlled range (setRange); histogram bins and cursor values
 *  are all expressed in that axis coordinate system.
 *  Emits lowerChanged / upperChanged whenever the user drags a cursor (not when
 *  the values are set programmatically via setLower / setUpper).
 */
class HistogramRangeWidget : public QCustomPlot
{
	Q_OBJECT
public:
	explicit HistogramRangeWidget(QWidget* parent = nullptr);

	//! Set fixed x-axis range
	void setRange(double xMin, double xMax);

	//! Supply binned histogram data. binMin/binMax span the data x-range (may be
	//! narrower than the axis range).
	void setHistogram(const std::vector<unsigned>& bins, double binMin, double binMax);

	//! Programmatic cursor updates (do not emit signals).
	void setLower(double v);
	void setUpper(double v);
	double lower() const { return m_lower; }
	double upper() const { return m_upper; }

signals:
	void lowerChanged(double);
	void upperChanged(double);

protected:
	void mousePressEvent(QMouseEvent*) override;
	void mouseMoveEvent(QMouseEvent*) override;
	void mouseReleaseEvent(QMouseEvent*) override;

private:
	void updateCursors();      // move QCPItem positions to reflect m_lower/m_upper
	double pixelToAxis(int px) const;
	double clampToRange(double v) const;

	double m_xMin  = 0.0;
	double m_xMax  = 1.0;
	double m_lower = 0.0;
	double m_upper = 1.0;

	int m_activeHandle = -1; // -1 none, 0 lower, 1 upper

	QCPBars*              m_bars         = nullptr;
	QCPItemStraightLine*  m_lowerLine    = nullptr;
	QCPItemStraightLine*  m_upperLine    = nullptr;
	QCPItemRect*          m_highlight    = nullptr;
};
