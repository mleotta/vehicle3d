// This is dml/dml_vehicle_parts.h
#ifndef dml_vehicle_parts_h_
#define dml_vehicle_parts_h_

//:
// \file
// \brief Functions involving the vehicle parts texture map
// \author Matt Leotta (mleotta@lems.brown.edu)
// \date 7/9/08
//
//          Copyright Matthew Leotta 2006 - 2010.
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file ../LICENSE_1_0.txt or copy at
//          http://www.boost.org/LICENSE_1_0.txt)
//
// \verbatim
//  Modifications
// \endverbatim


#include <vcl_map.h>
#include <vcl_string.h>
#include <vgl/vgl_polygon.h>


//: read the vehicle parts from a file
vcl_map<vcl_string, vgl_polygon<double> >
dml_read_vehicle_parts(const vcl_string filename);


//: write the vehicle parts to a file
void
dml_write_vehicle_parts(const vcl_string filename,
                        const vcl_map<vcl_string, vgl_polygon<double> >& parts);


//: write the vehicle parts as an SVG file
void dml_write_svg(const vcl_string& file,
                   const vcl_map<vcl_string,vgl_polygon<double> >& paths);



#endif // dml_vehicle_parts_h_
