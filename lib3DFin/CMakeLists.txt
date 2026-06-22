cmake_minimum_required(VERSION 3.15)

project(3DFin VERSION 0.1
	DESCRIPTION "3DFin lib"
	LANGUAGES CXX)

if (${PROJECT_NAME} STREQUAL ${CMAKE_PROJECT_NAME})
	set(3DFin_MAIN_PROJECT ON)
endif()

option(3DFin_BUILD_SHARED "Build 3DFin as a shared library" OFF)
option(3DFin_EXPORT_XLSX "Build 3DFin with XLSX support" OFF)

# Define the library target
if (3DFin_BUILD_SHARED)
	message(STATUS "[${PROJECT_NAME}] Building shared library...")
	add_library(${PROJECT_NAME} SHARED)
else()
	message(STATUS "[${PROJECT_NAME}] Building static library...")
	add_library(${PROJECT_NAME} STATIC)
endif()

set_target_properties(${PROJECT_NAME}
	PROPERTIES
		CXX_STANDARD 20
		CXX_STANDARD_REQUIRED ON
		CXX_EXTENSIONS NO
		POSITION_INDEPENDENT_CODE ON
)

if (3DFin_BUILD_SHARED)
	set_target_properties(${PROJECT_NAME}
		PROPERTIES
		VERSION ${PROJECT_VERSION}
		SOVERSION ${PROJECT_VERSION_MAJOR}
	)
endif()

# Eigen Handling (taken and modified from OpenMVG)
# Could be improved in case of exposition of Eigen in the public API
find_package(Eigen3 3.3.4...5 NO_MODULE QUIET)
if(Eigen3_FOUND)
	message(STATUS "[lib3DFin] Using external Eigen")
	message(STATUS "eigen found in ${EIGEN3_INCLUDE_DIRS}")
else()
	add_subdirectory(third_party/eigen EXCLUDE_FROM_ALL)
	set(3DFin_USE_INTERNAL_EIGEN ON)
	message(STATUS "[lib3DFin] Using internal Eigen")
endif()

# Test if nanoflann is already an imported target (needed for CC interop)
if(NOT TARGET nanoflann::nanoflann)
	find_package(nanoflann QUIET)
	if(NOT nanoflann_FOUND)
		message(STATUS "Using internal nanoflann")
		add_subdirectory(third_party/nanoflann EXCLUDE_FROM_ALL)
	endif()
endif()

# Taskflow handling
# TODO maybe thread lib linking should be propagated...
set(TF_BUILD_TESTS OFF CACHE BOOL "Taskflow: Enables builds of tests" FORCE)
set(TF_BUILD_EXAMPLES OFF CACHE BOOL "Taskflow: Enables builds of examples" FORCE)

# TODO improve drawing interface in order to make Eigen3 private
add_subdirectory(third_party/taskflow EXCLUDE_FROM_ALL)

# csf
add_subdirectory(third_party/CSF-3DFin)

# spdlog
add_subdirectory(third_party/spdlog)
#find_package(spdlog REQUIRED)

target_link_libraries(${PROJECT_NAME} PUBLIC Eigen3::Eigen spdlog::spdlog PRIVATE nanoflann::nanoflann Taskflow::Taskflow CSF::CSF)
# XLSX
if(3DFin_EXPORT_XLSX)
	set(OPENXLSX_BUILD_SAMPLES OFF CACHE BOOL "OpenXLSX: Enables builds of samples" FORCE)
	set(OPENXLSX_BUILD_TESTS OFF CACHE BOOL "OpenXLSX: Enables builds of tests" FORCE)

	add_subdirectory(third_party/OpenXLSX)
	target_compile_definitions(${PROJECT_NAME} PRIVATE TDFIN_USES_OPENXLSX)
	# All libs are private since we do not want to expose them in the public API
	target_link_libraries(${PROJECT_NAME} PRIVATE OpenXLSX::OpenXLSX)
endif()


# Install directories
include(GNUInstallDirs)
set(INSTALL_INCLUDE_DIR ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME})
set(INSTALL_CMAKE_DIR ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME})

# Add source/include directories
# Export headers are handled in include/CMakeLists.txt
add_subdirectory(include)
add_subdirectory(src)

install(
	TARGETS ${PROJECT_NAME}
	EXPORT ${PROJECT_NAME}Targets
	DESTINATION lib
)
install(
	TARGETS spdlog
	EXPORT ${PROJECT_NAME}Targets
	DESTINATION lib
)
if(3DFin_USE_INTERNAL_EIGEN)
	install(
		TARGETS eigen
		EXPORT ${PROJECT_NAME}Targets
		DESTINATION lib
	)
endif()

# Create and install CMake config files
include(CMakePackageConfigHelpers)

configure_package_config_file(
	"${CMAKE_CURRENT_SOURCE_DIR}/cmake/${PROJECT_NAME}Config.cmake.in"
	"${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
	INSTALL_DESTINATION ${INSTALL_CMAKE_DIR}
	PATH_VARS INSTALL_INCLUDE_DIR
)

write_basic_package_version_file(
	"${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
	VERSION ${PROJECT_VERSION}
	COMPATIBILITY AnyNewerVersion
)

install(FILES
	"${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
	"${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
	DESTINATION ${INSTALL_CMAKE_DIR}
)

# Export target definitions
install(EXPORT
	${PROJECT_NAME}Targets
	NAMESPACE ${PROJECT_NAME}::
	DESTINATION ${INSTALL_CMAKE_DIR}
)

# Export for build-tree usage
export(EXPORT
	${PROJECT_NAME}Targets
	NAMESPACE ${PROJECT_NAME}::
	FILE "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Targets.cmake"
)

# Make package available for find_package() in build dir
export(PACKAGE ${PROJECT_NAME})
