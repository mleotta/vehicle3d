// This is contrib/mleotta/modrec/modrec_proj_cost_func.cxx
//:
// \file

#include "modrec_proj_cost_func.h"

#include <imesh/algo/imesh_project.h>



//: Constructor
modrec_proj_cost_func::
modrec_proj_cost_func(const vil_image_view<float>& errors,
                      const vnl_double_3x4& camera,
                      const vpgl_poly_radial_distortion<double,4>& lens,
                      const imesh_mesh& model,
                      double xs, double ys)
  : vnl_cost_function(2),
    errors_(errors),
    camera_(camera),
    lens_(lens),
    model_(model),
    xs_(xs), ys_(ys),
    mask_(errors.ni(),errors.nj()),
    shadow_mask_(errors.ni(),errors.nj())
{
  if(!model_.faces().has_normals())
    model_.compute_face_normals();
  mask_.fill(false);
  shadow_mask_.fill(false);
}


//: Set the image of error values
void
modrec_proj_cost_func::set_errors(const vil_image_view<float>& e)
{
  if(errors_.ni() != e.ni() || errors_.nj() != e.nj()){
    mask_.set_size(e.ni(),e.nj(),1);
    mask_.fill(false);
    shadow_mask_.set_size(e.ni(),e.nj(),1);
    shadow_mask_.fill(false);
  }
  errors_ = e;
}


//: The main function.
double
modrec_proj_cost_func::f(vnl_vector<double> const& x)
{
  vnl_double_3x4 cam(camera_);
  cam.set_column(3, x(0)*cam.get_column(0) +
                    x(1)*cam.get_column(1) + cam.get_column(3));

  vnl_double_3x4 shadow_cam(cam);
  shadow_cam.set_column(2,xs_*cam.get_column(0)+ys_*cam.get_column(1));


  vgl_box_2d<unsigned int> bbox, bbox_shadow;
  imesh_project(model_, normals_, vpgl_proj_camera<double>(cam),
                lens_, mask_, &bbox);
  imesh_project(model_, normals_, vpgl_proj_camera<double>(shadow_cam),
                lens_, shadow_mask_, &bbox_shadow);
  bbox.add(bbox_shadow);

  unsigned ni = errors_.ni();
  vcl_ptrdiff_t istep=errors_.istep();
  unsigned nj = errors_.nj();
  vcl_ptrdiff_t jstep=errors_.jstep();
  vcl_ptrdiff_t pstep = errors_.planestep();

  assert(mask_.is_contiguous() && mask_.istep()==1);
  assert(shadow_mask_.is_contiguous() && shadow_mask_.istep()==1);

  unsigned int i0 = bbox.min_x(), i1 = bbox.max_x();
  unsigned int j0 = bbox.min_y(), j1 = bbox.max_y();

  const float* e_row = errors_.top_left_ptr() + i0*istep + j0*jstep;
  bool* m_row = mask_.top_left_ptr() + i0 + j0*ni;
  bool* sm_row = shadow_mask_.top_left_ptr() + i0 + j0*ni;
  double error = 0.0;
  for (unsigned j=j0;j<=j1;++j,e_row += jstep, m_row+=ni, sm_row+=ni)
  {
    const float* e_pixel = e_row;
    bool*  m_pixel = m_row;
    bool* sm_pixel = sm_row;
    for (unsigned i=i0;i<=i1;++i,e_pixel+=istep,++m_pixel,++sm_pixel){
      if(*m_pixel || *sm_pixel){
        error -= *e_pixel;
        if(*sm_pixel && !*m_pixel)
          error += *(e_pixel+pstep);
        // reset masks
        *sm_pixel = *m_pixel = false;
      }
    }
  }

  return error;
}

//=========================================================================================

//: Constructor
modrec_motion_cost_func::
modrec_motion_cost_func( const vcl_vector<vil_image_view<float> >& errors,
                         const vnl_double_3x4& camera,
                         const vpgl_poly_radial_distortion<double,4>& lens,
                         const imesh_mesh& model,
                         double xs, double ys )
  : vnl_cost_function(3),
    frame_cost_(errors[0],camera,lens,model,xs,ys),
    seq_errors_(errors)
{
}



//: The main function.
double
modrec_motion_cost_func::f(vnl_vector<double> const& x)
{
  double error = 0.0;

  double v = x(2);
  vnl_vector<double> xi(2); xi(0)=x(0); xi(1)=x(1);
  for(unsigned int i=0; i<seq_errors_.size(); ++i){
    frame_cost_.set_errors(seq_errors_[i]);
    xi(0) = x(0)+v*i;
    error += frame_cost_.f(xi);
  }


  return error;
}





