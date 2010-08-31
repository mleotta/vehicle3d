// This is dml/dml_edgel.h
#ifndef dml_edgel_h_
#define dml_edgel_h_

//:
// \file
// \brief A subpixel edge point with orientation and strength
// \author Matt Leotta (mleotta@lems.brown.edu)
// \date 8/26/08
//
//          Copyright Matthew Leotta 2006 - 2010.
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file ../LICENSE_1_0.txt or copy at
//          http://www.boost.org/LICENSE_1_0.txt)
//
// \verbatim
//  Modifications
// \endverbatim

#include <vcl_cmath.h>
#include <vgl/vgl_point_2d.h>
#include <vgl/vgl_vector_2d.h>
#include <vil/vil_image_view.h>

//: A subpixel edge point with orientation and strength
class dml_edgel : public vgl_point_2d<double>
{
public:
  //: Default Constructor
  dml_edgel() : angle_(0.0), strength_(0.0) {}

  //: Constructor
  dml_edgel(const vgl_point_2d<double>& p,
               double a, double s)
    : vgl_point_2d<double>(p), angle_(a), strength_(s) {}

  //: Constructor
  dml_edgel(double x, double y,
               double a, double s)
    : vgl_point_2d<double>(x,y), angle_(a), strength_(s) {}


  //: Return the edgel orientation angle in radians
  double angle() const { return angle_; }

  //: Return the edgel strength (magnitude of gradient)
  double strength() const { return strength_; }

  //: Return a unit normal vector to the edge (points in the gradient direction)
  vgl_vector_2d<double> normal() const
  {
    return vgl_vector_2d<double>(vcl_cos(angle_), vcl_sin(angle_));
  }


private:
  double angle_;
  double strength_;
};


//: Comparison operator for sorting edgels by strength
inline bool dml_edgel_strength_less(const dml_edgel& e1,
                                    const dml_edgel& e2)
{
  return e1.strength() < e2.strength();
}


//: Return a vector of all edgels in the edge map
vcl_vector<dml_edgel>
dml_find_all_edgels(const vil_image_view<float>& edge_map);


//: Search the edge map in the neighborhood of a line segment
//  return a vector of edgels found within \a ndist pixel in the normal direction,
//  with a \a edist pixel padding added in the line direction.
//  Edgel orientation must form an angle with the line less than \a ang_t (modulo pi/2 radians)
vcl_vector<dml_edgel>
dml_find_edgel_neighbors(const vil_image_view<float>& edge_map,
                         const vgl_line_segment_2d<double>& line,
                         int ndist, int edist, double ang_t);


#endif // dml_edgel_h_
