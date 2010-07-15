// This is contrib/mleotta/modrec/modrec_proj_lsqr_cost_func.cxx
//:
// \file

#include "modrec_proj_lsqr_cost_func.h"

#include <imesh/algo/imesh_project.h>



//: Constructor
modrec_proj_lsqr_cost_func::
modrec_proj_lsqr_cost_func( const vil_image_view<float>& errors,
                            const vnl_double_3x4& camera,
                            const vpgl_poly_radial_distortion<double,4>& lens,
                            const imesh_mesh& model,
                            double xs, double ys )
  : vnl_least_squares_function(1,errors.ni()*errors.nj(),no_gradient),
    errors_(errors),
    camera_(camera),
    lens_(lens),
    model_(model),
    xs_(xs), ys_(ys)
{
}



//: The main function.
//  Given the parameter vector x, compute the vector of residuals fx.
//  Fx has been sized appropriately before the call.
void 
modrec_proj_lsqr_cost_func::f(vnl_vector<double> const& x, vnl_vector<double>& fx)
{
  vnl_double_3x4 cam(camera_);
  cam.set_column(3,x(0)*cam.get_column(0)+cam.get_column(3));

  vnl_double_3x4 shadow_cam(cam);
  shadow_cam.set_column(2,xs_*cam.get_column(0)+ys_*cam.get_column(1));

  vil_image_view<bool> mask(errors_.ni(),errors_.nj()), shadow_mask(mask.ni(),mask.nj());
  mask.fill(false);
  shadow_mask.fill(false);
  imesh_project(model_,vpgl_proj_camera<double>(cam),lens_,mask);
  imesh_project(model_,vpgl_proj_camera<double>(shadow_cam),lens_,shadow_mask);

  for(unsigned int i=0; i<errors_.ni(); ++i){
    for(unsigned int j=0; j<errors_.nj(); ++j){
      if(!mask(i,j) && !shadow_mask(i,j))
        fx(i*errors_.nj()+j) = errors_(i,j,0);
      else
        fx(i*errors_.nj()+j) = errors_(i,j,1);
    }
  }
}



