# Add source files to target
target_sources(${PROJECT_NAME}
	PRIVATE
		geom.cpp
		voxel.cpp
		connected_components.cpp
		ground.cpp
		peeling.cpp
		section.cpp
		localization.cpp
		individualize.cpp
		interface.cpp
)

# XLSX export is optional (requires OpenXLSX, see the 3DFin_EXPORT_XLSX option)
if (3DFin_EXPORT_XLSX)
	target_sources(${PROJECT_NAME}
		PRIVATE
			xlsx.cpp
	)
endif()
