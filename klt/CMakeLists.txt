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
