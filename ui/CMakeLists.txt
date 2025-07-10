target_sources(
  ${PROJECT_NAME}
  PRIVATE
          ${CMAKE_CURRENT_LIST_DIR}/gui_ressources.qrc
          ${CMAKE_CURRENT_LIST_DIR}/cc3DFinDlg.ui
          ${CMAKE_CURRENT_LIST_DIR}/cc3DFinExpertDlg.ui
)

set_target_properties(
  ${PROJECT_NAME} PROPERTIES AUTOUIC_SEARCH_PATHS ${CMAKE_CURRENT_LIST_DIR}
)
