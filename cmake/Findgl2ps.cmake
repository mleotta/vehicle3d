
#          Copyright Matthew Leotta 2006 - 2010.
# Distributed under the Boost Software License, Version 1.0.
#    (See accompanying file ../LICENSE_1_0.txt or copy at
#          http://www.boost.org/LICENSE_1_0.txt)

# - Find GL2PS
# Find the native GL2PS includes and library
# This module defines
#  GL2PS_INCLUDE_DIR, where to find gl2ps.h, etc.
#  GL2PS_LIBRARIES, the libraries needed to use GL2PS.
#  GL2PS_FOUND, If false, do not try to use GL2PS.
# also defined, but not for general use are
#  GL2PS_LIBRARY, where to find the GL2PS library.


FIND_PATH(GL2PS_INCLUDE_DIR gl2ps.h)

FIND_LIBRARY(GL2PS_LIBRARY NAMES gl2ps )

# handle the QUIETLY and REQUIRED arguments and set GL2PS_FOUND to TRUE if
# all listed variables are TRUE
INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(GL2PS DEFAULT_MSG GL2PS_LIBRARY GL2PS_INCLUDE_DIR)

IF(GL2PS_FOUND)
  SET(GL2PS_LIBRARIES ${GL2PS_LIBRARY})
ENDIF(GL2PS_FOUND)

MARK_AS_ADVANCED(GL2PS_LIBRARY GL2PS_INCLUDE_DIR )
