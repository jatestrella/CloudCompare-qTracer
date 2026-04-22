# CloudCompare qTracer plugin (DFN fracture/trace extraction)

option( PLUGIN_STANDARD_QTRACER "Check to install qTracer plugin" OFF )

if ( PLUGIN_STANDARD_QTRACER )
	project( qTracer )

	AddPlugin( NAME ${PROJECT_NAME} )

	add_subdirectory( include )
	add_subdirectory( src )
	add_subdirectory( ui )

	# QCustomPlot for the color-filter histogram widget
	target_link_libraries( ${PROJECT_NAME}
		QCustomPlot
		Qt6::PrintSupport
	)
endif()
