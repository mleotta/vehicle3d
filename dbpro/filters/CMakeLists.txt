
#          Copyright Matthew Leotta 2006 - 2010.
# Distributed under the Boost Software License, Version 1.0.
#    (See accompanying file ../../LICENSE_1_0.txt or copy at
#          http://www.boost.org/LICENSE_1_0.txt)

# dbpro/filters/CMakeLists.txt


set(dbpro_filters_sources
   vil_gaussian_blur_process.cxx    vil_gaussian_blur_process.h
   vil_diff_process.cxx             vil_diff_process.h
   vil_morphology_filters.cxx       vil_morphology_filters.h
                                    vil_convert_filters.h
                                    vil_math_filters.h
                                    vil_gauss_filter.h
                                    vil_sobel_1x3_filter.h
                                    vil_subpix_edge_filter.h
                                    vil_grad_sqr_filter.h
                                    vil_transform3_1_filter.h
   vil_monotone_process.cxx         vil_monotone_process.h

                                    bbgm_filters.h

   vidl_source.cxx                  vidl_source.h
   vidl_multi_source.cxx            vidl_multi_source.h
   vidl_sink.cxx                    vidl_sink.h
   vidl_frame_to_resource.cxx       vidl_frame_to_resource.h
   vidl_resource_to_frame.cxx       vidl_resource_to_frame.h
)


add_library(dbpro_filters ${dbpro_filters_sources})

target_link_libraries(dbpro_filters dbpro vil vil_algo vidl)

