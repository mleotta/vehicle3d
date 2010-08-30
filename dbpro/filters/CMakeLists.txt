
#          Copyright Matthew Leotta 2006 - 2010.
# Distributed under the Boost Software License, Version 1.0.
#    (See accompanying file ../../LICENSE_1_0.txt or copy at
#          http://www.boost.org/LICENSE_1_0.txt)

# dbpro/filters/CMakeLists.txt


set(dbpro_filters_sources
   dbil_gaussian_blur_process.cxx   dbil_gaussian_blur_process.h
   dbil_diff_process.cxx            dbil_diff_process.h
   dbil_morphology_filters.cxx      dbil_morphology_filters.h
                                    dbil_convert_filters.h
                                    dbil_math_filters.h
                                    dbil_gauss_filter.h
                                    dbil_sobel_1x3_filter.h
                                    dbil_subpix_edge_filter.h
                                    dbil_grad_sqr_filter.h
                                    dbil_transform3_1_filter.h
   dbil_monotone_process.cxx        dbil_monotone_process.h

                                    dbbgm.h

   vidl_source.cxx                  vidl_source.h
   vidl_multi_source.cxx            vidl_multi_source.h
   vidl_sink.cxx                    vidl_sink.h
   vidl_frame_to_resource.cxx       vidl_frame_to_resource.h
   vidl_resource_to_frame.cxx       vidl_resource_to_frame.h
)


add_library(dbpro_filters ${dbpro_filters_sources})

target_link_libraries(dbpro_filters dbpro vil vil_algo vidl)

