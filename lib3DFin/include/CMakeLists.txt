target_sources(${PROJECT_NAME}
 PRIVATE
     config.hpp
)

install(
 FILES
  config.hpp
 DESTINATION
  ${INSTALL_CMAKE_DIR}
)

target_include_directories(${PROJECT_NAME}
 PUBLIC
  $<INSTALL_INTERFACE:${INSTALL_CMAKE_DIR}>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
)
