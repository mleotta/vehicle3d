PROJECT(pca_vehicle_wxgui)

FIND_PACKAGE(wxWidgets COMPONENTS base core gl xrc)
find_package(gl2ps)
IF(wxWidgets_FOUND)

  INCLUDE( ${wxWidgets_USE_FILE} )


  IF(VXL_VGUI_WX_FOUND)

    # Example uses the wxWidgets XML resource fi
    SET(pca_vehicle_wxgui_sources
      pca_vehicle_app.h      pca_vehicle_app.cxx
      pca_vehicle_frame.h    pca_vehicle_frame.cxx
      pca_vehicle_manager.h  pca_vehicle_manager.cxx

      gui_utils.h            gui_utils.cxx
      gnd_cal_tableau.h      gnd_cal_tableau.cxx      gnd_cal_tableau_sptr.h
      
      # Resouce files
      default.parts
      default1.pca
      default2.pca
      default3.pca
      default_dodec.pca
      default_ferryman.pca
    )
    
    SET_SOURCE_FILES_PROPERTIES(
      default.parts
      default1.pca
      default2.pca
      default3.pca
      default_dodec.pca
      default_ferryman.pca
      PROPERTIES
      MACOSX_PACKAGE_LOCATION Resources
    )
    
    IF(APPLE)
      # use CoreFoundation to locate the default files in the bundle
      INCLUDE_DIRECTORIES(/System/Library/Frameworks/CoreFoundation.framework/Headers/)
    ELSEIF(APPLE)
      # copy the default files to the binary directory
      CONFIGURE_FILE( ${pca_vehicle_wxgui_SOURCE_DIR}/default1.pca
                      ${pca_vehicle_wxgui_BINARY_DIR}/default1.pca COPY_ONLY )
      CONFIGURE_FILE( ${pca_vehicle_wxgui_SOURCE_DIR}/default2.pca
                      ${pca_vehicle_wxgui_BINARY_DIR}/default2.pca COPY_ONLY )
      CONFIGURE_FILE( ${pca_vehicle_wxgui_SOURCE_DIR}/default3.pca
                      ${pca_vehicle_wxgui_BINARY_DIR}/default3.pca COPY_ONLY )
      CONFIGURE_FILE( ${pca_vehicle_wxgui_SOURCE_DIR}/default_dodec.pca
                      ${pca_vehicle_wxgui_BINARY_DIR}/default_dodec.pca COPY_ONLY )
      CONFIGURE_FILE( ${pca_vehicle_wxgui_SOURCE_DIR}/default_ferryman.pca
                      ${pca_vehicle_wxgui_BINARY_DIR}/default_ferryman.pca COPY_ONLY )
      CONFIGURE_FILE( ${pca_vehicle_wxgui_SOURCE_DIR}/default.parts
                      ${pca_vehicle_wxgui_BINARY_DIR}/default.parts COPY_ONLY )
    ENDIF(APPLE)

    
    WXWIDGETS_ADD_RESOURCES(pca_vehicle_wxgui_sources pca_vehicle.xrc)
    ADD_EXECUTABLE(pca_vehicle_wxgui WIN32 MACOSX_BUNDLE ${pca_vehicle_wxgui_sources})
    TARGET_LINK_LIBRARIES(pca_vehicle_wxgui vgui_wx bgui3d bgui imesh modrec vidl_gui vil dbul gl2ps vpgl_algo)

  ENDIF(VXL_VGUI_WX_FOUND)

ENDIF(wxWidgets_FOUND)
