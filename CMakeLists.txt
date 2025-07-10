cmake_minimum_required(VERSION 3.15..3.27)

# Add an option to CMake to control whether we build this plugin or not
option( PLUGIN_STANDARD_3DFIN "Install 3DFin plugin" OFF )

if ( PLUGIN_STANDARD_3DFIN  )
    project( 3DFIN_STANDARD_PLUGIN )

    AddPlugin( NAME ${PROJECT_NAME} )

    add_subdirectory( ui )
    add_subdirectory( include )
    add_subdirectory( src )

endif()
