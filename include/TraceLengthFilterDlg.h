#pragma once

#include <ui_TraceLengthFilterDlg.h>

#include <QTimer>
#include <vector>

class ccHObject;
class ccPolyline;
class ccMainAppInterface;

//! Interactive "remove short traces" post-processing filter with live 3D preview.
/** Operates on a traces group (e.g. the pipeline's `Traces` or `[trace pieces]`).
 *  Each direct child that carries a 2-vertex polyline is treated as one trace; the
 *  dialog shows a histogram of trace lengths with two draggable cursors (min/max).
 *  While the dialog is open, traces outside [min, max] are hidden (setEnabled) for
 *  preview; both accept() and reject() restore the original enabled state. The
 *  caller materialises the surviving traces into a new group (non-destructive).
 */
class TraceLengthFilterDlg : public QDialog, public Ui::TraceLengthFilterDialog
{
	Q_OBJECT
public:
	struct TraceEntry
	{
		ccHObject*        node   = nullptr; //!< the group's direct child (a trace)
		const ccPolyline* poly   = nullptr; //!< its representative 2-vertex polyline
		double            length = 0.0;     //!< polyline length
	};

	TraceLengthFilterDlg(ccHObject* tracesGroup, ccMainAppInterface* app, QWidget* parent = nullptr);

	void accept() override;
	void reject() override;

	unsigned traceCount() const { return static_cast<unsigned>(m_traces.size()); }
	const std::vector<TraceEntry>& traces() const { return m_traces; }
	double minLen() const { return minLenSpinBox->value(); }
	double maxLen() const { return maxLenSpinBox->value(); }

private slots:
	void onThresholdChanged();
	void applyPreview();

private:
	void gatherTraces();
	void buildHistogram();
	void restoreOriginalEnabled();

	ccHObject*              m_group = nullptr;
	ccMainAppInterface*     m_app   = nullptr;
	QTimer                  m_previewTimer;
	std::vector<TraceEntry> m_traces;
	std::vector<bool>       m_originalEnabled;
};
