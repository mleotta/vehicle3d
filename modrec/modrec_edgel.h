// This is contrib/mleotta/modrec/modrec_edgel.h
#ifndef modrec_edgel_h_
#define modrec_edgel_h_

//:
// \file
// \brief A subpixel edge point with orientation and strength
// \author Matt Leotta (mleotta@lems.brown.edu)
// \date 8/26/08
//
// \verbatim
//  Modifications
// \endverbatim

#include <vcl_cmath.h>
#include <vgl/vgl_point_2d.h>
#include <vgl/vgl_vector_2d.h>
#include <vil/vil_image_view.h>

//: A subpixel edge point with orientation and strength
class modrec_edgel : public vgl_point_2d<double>
{
public:
  //: Default Constructor
  modrec_edgel() : angle_(0.0), strength_(0.0) {}

  //: Constructor
  modrec_edgel(const vgl_point_2d<double>& p,
               double a, double s)
    : vgl_point_2d<double>(p), angle_(a), strength_(s) {}

  //: Constructor
  modrec_edgel(double x, double y,
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
inline bool modrec_edgel_strength_less(const modrec_edgel& e1,
                                       const modrec_edgel& e2)
{
  return e1.strength() < e2.strength();
}


//: Return a vector of all edgels in the edge map
vcl_vector<modrec_edgel>
modrec_find_all_edgels(const vil_image_view<float>& edge_map);


//: Search the edge map in the neighborhood of a line segment
//  return a vector of edgels found within \a ndist pixel in the normal direction,
//  with a \a edist pixel padding added in the line direction.
//  Edgel orientation must form an angle with the line less than \a ang_t (modulo pi/2 radians)
vcl_vector<modrec_edgel>
modrec_find_edgel_neighbors(const vil_image_view<float>& edge_map,
                            const vgl_line_segment_2d<double>& line,
                            int ndist, int edist, double ang_t);


#endif // modrec_edgel_h_
