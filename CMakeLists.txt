# CloudCompare qTracer plugin (DFN fracture/trace extraction)

option( PLUGIN_STANDARD_QTRACER "Check to install qTracer plugin" OFF )

if ( PLUGIN_STANDARD_QTRACER )
	project( qTracer )

	AddPlugin( NAME ${PROJECT_NAME} )

	add_subdirectory( include )
	add_subdirectory( src )
	add_subdirectory( ui )
endif()
