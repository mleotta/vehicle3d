
#          Copyright Matthew Leotta 2006 - 2010.
# Distributed under the Boost Software License, Version 1.0.
#    (See accompanying file ../LICENSE_1_0.txt or copy at
#          http://www.boost.org/LICENSE_1_0.txt)

PROJECT(pca_vehicle_wxgui)

FIND_PACKAGE(wxWidgets COMPONENTS base core gl xrc)
find_package(gl2ps)
include_directories(${GL2PS_INCLUDE_DIR})
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
      ../data/default.parts
      ../data/default1.pca
      ../data/default2.pca
      ../data/default3.pca
      ../data/default_dodec.pca
      ../data/default_ferryman.pca
    )
    
    SET_SOURCE_FILES_PROPERTIES(
      ../data/default.parts
      ../data/default1.pca
      ../data/default2.pca
      ../data/default3.pca
      ../data/default_dodec.pca
      ../data/default_ferryman.pca
      PROPERTIES
      MACOSX_PACKAGE_LOCATION Resources
    )


      
    if (APPLE)
      # use CoreFoundation to locate the default files in the bundle
      INCLUDE_DIRECTORIES(/System/Library/Frameworks/CoreFoundation.framework/Headers/)
    else (APPLE)
      # copy the default files to the binary directory
      CONFIGURE_FILE( ${vehicle_SOURCE_DIR}/data/default1.pca
                      ${RUNTIME_OUTPUT_DIRECTORY}/default1.pca COPY_ONLY )
      CONFIGURE_FILE( ${vehicle_SOURCE_DIR}/data/default2.pca
                      ${RUNTIME_OUTPUT_DIRECTORY}/default2.pca COPY_ONLY )
      CONFIGURE_FILE( ${vehicle_SOURCE_DIR}/data/default3.pca
                      ${RUNTIME_OUTPUT_DIRECTORY}/default3.pca COPY_ONLY )
      CONFIGURE_FILE( ${vehicle_SOURCE_DIR}/data/default_dodec.pca
                      ${RUNTIME_OUTPUT_DIRECTORY}/default_dodec.pca COPY_ONLY )
      CONFIGURE_FILE( ${vehicle_SOURCE_DIR}/data/default_ferryman.pca
                      ${RUNTIME_OUTPUT_DIRECTORY}/default_ferryman.pca COPY_ONLY )
      CONFIGURE_FILE( ${vehicle_SOURCE_DIR}/data/default.parts
                      ${RUNTIME_OUTPUT_DIRECTORY}/default.parts COPY_ONLY )
    endif (APPLE)

    
    WXWIDGETS_ADD_RESOURCES(pca_vehicle_wxgui_sources pca_vehicle.xrc)
    ADD_EXECUTABLE(pca_vehicle_wxgui WIN32 MACOSX_BUNDLE ${pca_vehicle_wxgui_sources})
    TARGET_LINK_LIBRARIES(pca_vehicle_wxgui vgui_wx bgui3d bgui imesh dml vidl_gui vil gl2ps vpgl_algo)

  ENDIF(VXL_VGUI_WX_FOUND)

ENDIF(wxWidgets_FOUND)
