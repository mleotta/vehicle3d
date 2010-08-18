
#          Copyright Matthew Leotta 2006 - 2010.
# Distributed under the Boost Software License, Version 1.0.
#    (See accompanying file ../../LICENSE_1_0.txt or copy at
#          http://www.boost.org/LICENSE_1_0.txt)

# dbpro/vis/CMakeLists.txt


if (VXL_VGUI_FOUND)
  set(dbpro_vis_sources
    dbpro_run_tableau.cxx             dbpro_run_tableau.h           dbpro_run_tableau_sptr.h
    dbpro_basic_gui_observers.h
  )

  add_library(dbpro_vis  ${dbpro_vis_sources})

  target_link_libraries(dbpro_vis dbpro vgui)

endif ()
