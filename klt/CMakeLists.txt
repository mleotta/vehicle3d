
#          Copyright Matthew Leotta 2006 - 2010.
# Distributed under the Boost Software License, Version 1.0.
#    (See accompanying file ../LICENSE_1_0.txt or copy at
#          http://www.boost.org/LICENSE_1_0.txt)
#
# Note: The rest of the code in this directory has a separate Copyright.
#       See the the accompanying file README 
#       Only this CMakeLists.txt file was added to aid compilation.

# klt/CMakeLists.txt

set(klt_sources
                        base.h
   convolve.c           convolve.h
   klt.c                klt.h
   pnmio.c              pnmio.h
   selectGoodFeatures.c
   trackFeatures.c
   error.c              error.h
   klt_util.c           klt_util.h
   pyramid.c            pyramid.h
   storeFeatures.c
   writeFeatures.c
)

add_library(klt ${klt_sources})

if (UNIX)
  target_link_libraries(klt m)
endif (UNIX)
