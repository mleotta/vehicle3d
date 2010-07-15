// This is contrib/mleotta/modrec/modrec_proj_cost_func.h
#ifndef modrec_proj_cost_func_h_
#define modrec_proj_cost_func_h_
//:
// \file
// \brief A cost function for registering a 3D
//        model and its projection
// \author Matt Leotta
// \date 11/11/05
//
// \verbatim
//  Modifications
//   None
// \endverbatim

#include <vnl/vnl_vector.h>
#include <vnl/vnl_cost_function.h>
#include <vil/vil_image_view.h>
#include <vpgl/vpgl_proj_camera.h>
#include <vpgl/vpgl_poly_radial_distortion.h>
#include <imesh/imesh_mesh.h>

//: A cost function for registering a 3D
//  model and its projection into a single frame
class modrec_proj_cost_func : public vnl_cost_function
{
public:
  //: Constructor
  modrec_proj_cost_func( const vil_image_view<float>& errors,
                         const vnl_double_3x4& camera,
                         const vpgl_poly_radial_distortion<double,4>& lens,
                         const imesh_mesh& model,
                         double xs, double ys );


  //: The main function.
  virtual double f(vnl_vector<double> const& x);

  //: Set the image of error values
  void set_errors(const vil_image_view<float>& e);


protected:
  vil_image_view<float> errors_;
  vnl_double_3x4 camera_;
  vpgl_poly_radial_distortion<double,4> lens_;
  imesh_mesh model_;
  vcl_vector<vgl_vector_3d<double> > normals_;
  double xs_, ys_;
  vil_image_view<bool> mask_, shadow_mask_;

};



//: A cost function for registering a 3D
//  model and its projection many frames under a motion model
class modrec_motion_cost_func : public vnl_cost_function
{
public:
  //: Constructor
  modrec_motion_cost_func( const vcl_vector<vil_image_view<float> >& errors,
                           const vnl_double_3x4& camera,
                           const vpgl_poly_radial_distortion<double,4>& lens,
                           const imesh_mesh& model,
                           double xs, double ys );


  //: The main function.
  virtual double f(vnl_vector<double> const& x);


protected:
  //: The cost function for a given frame
  modrec_proj_cost_func frame_cost_;

  //: The error measures for each frame in the sequence
  vcl_vector<vil_image_view<float> > seq_errors_;

};

#endif // modrec_proj_cost_func_h_

