target_sources(${PROJECT_NAME}
 PRIVATE
     types.hpp
     config.hpp
     voxel.hpp
     connected_components.hpp
     ground.hpp
     interface.hpp
)

install(
 FILES
  types.hpp
  config.hpp
  voxel.hpp
  connected_components.hpp
  ground.hpp
  interface.hpp
 DESTINATION
  ${INSTALL_CMAKE_DIR}
)

target_include_directories(${PROJECT_NAME}
 PUBLIC
  $<INSTALL_INTERFACE:${INSTALL_CMAKE_DIR}>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
)
