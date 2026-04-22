
target_sources( ${PROJECT_NAME}
	PRIVATE
		${CMAKE_CURRENT_LIST_DIR}/IdentifyFracture.h
		${CMAKE_CURRENT_LIST_DIR}/PipelineDlg.h
		${CMAKE_CURRENT_LIST_DIR}/disclaimerDialog.h
		${CMAKE_CURRENT_LIST_DIR}/facetsClassifier.h
		${CMAKE_CURRENT_LIST_DIR}/qTracer.h
)

target_include_directories( ${PROJECT_NAME}
	PRIVATE
		${CMAKE_CURRENT_SOURCE_DIR}
)
