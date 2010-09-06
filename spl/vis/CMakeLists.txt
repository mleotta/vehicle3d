
#          Copyright Matthew Leotta 2006 - 2010.
# Distributed under the Boost Software License, Version 1.0.
#    (See accompanying file ../../LICENSE_1_0.txt or copy at
#          http://www.boost.org/LICENSE_1_0.txt)

# spl/vis/CMakeLists.txt


if (VXL_VGUI_FOUND)
  set(spl_vis_sources
    spl_run_tableau.cxx             spl_run_tableau.h           spl_run_tableau_sptr.h
    spl_basic_gui_observers.h
  )

  add_library(spl_vis  ${spl_vis_sources})

  target_link_libraries(spl_vis spl vgui)

endif ()
