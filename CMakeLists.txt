cmake_minimum_required(VERSION 3.22)

# Add an option to CMake to control whether we build this plugin or not
option(PLUGIN_STANDARD_3DFIN "Install 3DFin plugin" OFF)

if (PLUGIN_STANDARD_3DFIN)

	project(3DFIN_STANDARD_PLUGIN)

	AddPlugin(NAME ${PROJECT_NAME})

	set_target_properties(${PROJECT_NAME}
		PROPERTIES
			CXX_STANDARD 20
			CXX_STANDARD_REQUIRED ON
			CXX_EXTENSIONS NO
	)

	set_property(TARGET ${PROJECT_NAME} PROPERTY AUTORCC ON)

	target_sources(${PROJECT_NAME}
		PRIVATE
			${CMAKE_CURRENT_SOURCE_DIR}/include/cc3DFin.h
			${CMAKE_CURRENT_SOURCE_DIR}/include/cc3DFinDlg.h
			${CMAKE_CURRENT_SOURCE_DIR}/include/cc3DFinExpertDlg.h
			${CMAKE_CURRENT_SOURCE_DIR}/include/cc3DFinUiConfig.h
			${CMAKE_CURRENT_SOURCE_DIR}/include/cc3DFinDrawer.h
	)

	target_sources(${PROJECT_NAME}
		PRIVATE
			${CMAKE_CURRENT_SOURCE_DIR}/src/cc3DFin.cpp
			${CMAKE_CURRENT_SOURCE_DIR}/src/cc3DFinDlg.cpp
			${CMAKE_CURRENT_SOURCE_DIR}/src/cc3DFinUiConfig.cpp
			${CMAKE_CURRENT_SOURCE_DIR}/src/cc3DFinDrawer.cpp
	)

	target_sources(${PROJECT_NAME}
		PRIVATE
			${CMAKE_CURRENT_SOURCE_DIR}/ui/gui_ressources.qrc
			${CMAKE_CURRENT_SOURCE_DIR}/ui/cc3DFinDlg.ui
			${CMAKE_CURRENT_SOURCE_DIR}/ui/cc3DFinExpertDlg.ui
	)

	target_include_directories(${PROJECT_NAME}
		PRIVATE
			${CMAKE_CURRENT_SOURCE_DIR}/include
	)

	set_target_properties(${PROJECT_NAME} PROPERTIES AUTOUIC_SEARCH_PATHS ${CMAKE_CURRENT_SOURCE_DIR}/ui)

	add_subdirectory(lib3DFin EXCLUDE_FROM_ALL)

	target_link_libraries(${PROJECT_NAME} 3DFin)
endif()
