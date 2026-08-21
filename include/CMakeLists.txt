
target_sources( ${PROJECT_NAME}
	PRIVATE
		${CMAKE_CURRENT_LIST_DIR}/ColorFilterDlg.h
		${CMAKE_CURRENT_LIST_DIR}/HistogramRangeWidget.h
		${CMAKE_CURRENT_LIST_DIR}/IdentifyFracture.h
		${CMAKE_CURRENT_LIST_DIR}/OutcropArea.h
		${CMAKE_CURRENT_LIST_DIR}/OutcropAreaDlg.h
		${CMAKE_CURRENT_LIST_DIR}/P21Dlg.h
		${CMAKE_CURRENT_LIST_DIR}/PipelineDlg.h
		${CMAKE_CURRENT_LIST_DIR}/TraceLengthFilterDlg.h
		${CMAKE_CURRENT_LIST_DIR}/disclaimerDialog.h
		${CMAKE_CURRENT_LIST_DIR}/facetsClassifier.h
		${CMAKE_CURRENT_LIST_DIR}/qTracer.h
)

target_include_directories( ${PROJECT_NAME}
	PRIVATE
		${CMAKE_CURRENT_SOURCE_DIR}
)
