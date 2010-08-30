
#          Copyright Matthew Leotta 2006 - 2010.
# Distributed under the Boost Software License, Version 1.0.
#    (See accompanying file ../../LICENSE_1_0.txt or copy at
#          http://www.boost.org/LICENSE_1_0.txt)

# vpro/vis/CMakeLists.txt


if (VXL_VGUI_FOUND)
  set(vpro_vis_sources
    vpro_run_tableau.cxx             vpro_run_tableau.h           vpro_run_tableau_sptr.h
    vpro_basic_gui_observers.h
  )

  add_library(vpro_vis  ${vpro_vis_sources})

  target_link_libraries(vpro_vis vpro vgui)

endif ()
