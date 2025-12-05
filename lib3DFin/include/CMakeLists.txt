# Copy all headers to build include directory with lib3DFin/ prefix
set(HEADERS
	circle_fit.hpp
	config.hpp
	connected_components.hpp
	ground.hpp
	individualize.hpp
	interface.hpp
	peeling.hpp
	section.hpp
	slink.hpp
	statistics.hpp
	types.hpp
	verticality.hpp
	voxel.hpp
)

target_sources(${PROJECT_NAME} PRIVATE ${HEADERS})

# Create FHS for lib3DFin
set(BUILD_INCLUDE_DIR ${CMAKE_CURRENT_BINARY_DIR}/include)
file(MAKE_DIRECTORY ${BUILD_INCLUDE_DIR}/lib3DFin)


foreach(header ${HEADERS})
	configure_file(
		${CMAKE_CURRENT_SOURCE_DIR}/${header}
		${BUILD_INCLUDE_DIR}/lib3DFin/${header}
		COPYONLY
	)
endforeach()

# install all headers under lib3DFin
install(
	FILES
		${HEADERS}
	DESTINATION
		${INSTALL_INCLUDE_DIR}/lib3DFin
)

target_include_directories(${PROJECT_NAME}
	PUBLIC
		# For external use (<lib3DFin/header.hpp>)
		$<INSTALL_INTERFACE:${INSTALL_INCLUDE_DIR}>
		$<BUILD_INTERFACE:${BUILD_INCLUDE_DIR}>
	PRIVATE
		# For internal source files (using direct include i.e "header.hpp")
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
)
