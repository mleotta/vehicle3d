// This is dml/dml_proj_lsqr_cost_func.h
#ifndef dml_proj_lsqr_cost_func_h_
#define dml_proj_lsqr_cost_func_h_
//:
// \file
// \brief A least squares cost function for registering a 3D
//        model and its projection
// \author Matt Leotta
// \date 11/11/05
//
//          Copyright Matthew Leotta 2006 - 2010.
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file ../LICENSE_1_0.txt or copy at
//          http://www.boost.org/LICENSE_1_0.txt)
//
// \verbatim
//  Modifications
//   None
// \endverbatim

#include <vnl/vnl_vector.h>
#include <vnl/vnl_least_squares_function.h>
#include <vil/vil_image_view.h>
#include <vpgl/vpgl_proj_camera.h>
#include <vpgl/vpgl_poly_radial_distortion.h>
#include <imesh/imesh_mesh.h>

//: A least squares cost function for registering a 3D
//        model and its projection
class dml_proj_lsqr_cost_func : public vnl_least_squares_function
{
public:
  //: Constructor
  dml_proj_lsqr_cost_func( const vil_image_view<float>& errors,
                           const vnl_double_3x4& camera,
                           const vpgl_poly_radial_distortion<double,4>& lens,
                           const imesh_mesh& model,
                           double xs, double ys );


  //: The main function.
  //  Given the parameter vector x, compute the vector of residuals fx.
  //  Fx has been sized appropriately before the call.
  virtual void f(vnl_vector<double> const& x, vnl_vector<double>& fx);


protected:
  vil_image_view<float> errors_;
  vnl_double_3x4 camera_;
  vpgl_poly_radial_distortion<double,4> lens_;
  imesh_mesh model_;
  double xs_, ys_;


};

#endif // dml_proj_lsqr_cost_func_h_

