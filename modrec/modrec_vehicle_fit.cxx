// This is mleotta/gui/pca_vehicle/modrec_vehicle_fit.cxx
//=========================================================================
//:
// \file
// \brief Fit the PCA vehicle to one or more images
//
//=========================================================================

#include "modrec_vehicle_fit.h"


#include <vil/algo/vil_gauss_filter.h>
#include <vil/algo/vil_sobel_1x3.h>
#include <vil/algo/vil_suppress_non_max_edges.h>
#include <vcl_limits.h>
#include <rrel/rrel_util.h>
#include <rrel/rrel_cauchy_obj.h>
#include <rrel/rrel_tukey_obj.h>

#include <vnl/algo/vnl_cholesky.h>

//: Constructor
modrec_vehicle_fit::modrec_vehicle_fit()
: min_e_strength_(0.0),
  max_e_strength_(30.0),
  lambda_(1.0),
  mest_scale_(1.0),
  border_(5.0),
  num_params_(0),
  num_pc_(0),
  options_(7,false),
  txi_(-1), tyi_(-1), tzi_(-1), 
  rxi_(-1), ryi_(-1), rzi_(-1)
{
}



//: Compute the edge map for image \a img
void modrec_vehicle_fit::detect_edges(const vil_image_view<vxl_byte>& img,
                                      vil_image_view<float>& edge_map) const
{
  vil_image_view<float> work;
  vil_image_view<float> smooth;
  vil_image_view<float> grad_x;
  vil_image_view<float> grad_y;
  
  vil_gauss_filter_5tap_params sigma(1.0);
  vil_gauss_filter_5tap(img,smooth,sigma,work);
  vil_sobel_1x3(smooth,grad_x,grad_y);
  vil_suppress_non_max_edges_subpixel(grad_x,grad_y,2.0,edge_map);
}


//: set optimization options
//  The vector of boolean options specifies which parameters to fit.
//  - [0] is top \a num_pc PCA params
//  - [1,2,3] is Tx,Ty,Tz respectively
//  - [4,5,6] is Rx,Ry,Rz respectively
void modrec_vehicle_fit::set_options(const vcl_vector<bool>& options, 
                                     unsigned int num_pc)
{
  num_params_ = options[0]?num_pc:0;
  txi_ = options[1]?num_params_++:-1;
  tyi_ = options[2]?num_params_++:-1;
  tzi_ = options[3]?num_params_++:-1;
  rxi_ = options[4]?num_params_++:-1;
  ryi_ = options[5]?num_params_++:-1;
  rzi_ = options[6]?num_params_++:-1;
  
  num_pc_ = num_pc;
  options_ = options;
  
  set_init_uncert(1.0);
}


//: Set the initial uncertaintly to a scale times the default uncertainty
void modrec_vehicle_fit::set_init_uncert(double uncert)
{ 
  init_uncert_.set_size(num_params_);
  init_uncert_.fill(uncert);
  if(txi_ >= 0) init_uncert_[txi_] = uncert;
  if(tyi_ >= 0) init_uncert_[tyi_] = uncert;
  if(tzi_ >= 0) init_uncert_[tzi_] = uncert;
  if(rxi_ >= 0) init_uncert_[rxi_] = 0.5*uncert;
  if(ryi_ >= 0) init_uncert_[ryi_] = 0.5*uncert;
  if(rzi_ >= 0) init_uncert_[rzi_] = 0.5*uncert;
}


//: Finds the set of all edge matches near the line segment from \a p0 to \a p1
//  Matches are returned as a 3-d vector where index
// - 0 is a value in [0.0, 1.0] indicating location of the projection between the endpoints
// - 1 is the perpendicular distance to the edgel
// - 2 is the weight for use in optimization
void modrec_vehicle_fit::
compute_line_matches(const vgl_point_2d<double>& p0, const vgl_point_2d<double>& p1,
                     const vil_image_view<float>& edge_map,
                     vcl_vector<vnl_vector_fixed<double,3> >& matches) const
{
  unsigned int search_dist = vcl_ceil(4*mest_scale_);
  vcl_vector<modrec_edgel> edgels =
      modrec_find_edgel_neighbors(edge_map,
                                  vgl_line_segment_2d<double>(p0,p1),
                                  search_dist, 0, 0.5);
  if(edgels.empty())
    return;
  
  double bnx = edge_map.ni() - border_-1;
  double bny = edge_map.nj() - border_-1;
  double ediff = max_e_strength_ - min_e_strength_;
  
  //rrel_cauchy_obj m_est(1.0);
  rrel_tukey_obj m_est(4*mest_scale_);
  
  vgl_vector_2d<double> v = p1-p0;
  vgl_vector_2d<double> n(v.y(),-v.x());
  double len = v.sqr_length();
  v /= len; 
  len = vcl_sqrt(len);
  n /= len;
  // create a histogram of approx 1 pixel bins along the line
  unsigned int num_bins = vcl_floor(len);
  vcl_vector<unsigned int> bins(num_bins+1,0);
  unsigned int init_matches_size = matches.size();
  for(unsigned int k=0; k<edgels.size(); ++k)
  {
    const modrec_edgel& e = edgels[k];
    
    double w = (e.strength() - min_e_strength_)/ediff;
    if(w<0.0) continue;
    if(w>1.0) w = 1.0;
    
    vgl_vector_2d<double> v2 = e-p0;
    double s = dot_product(v,v2);
    if(s < 0.0) s = 0.0;
    else if (s > 1.0) s = 1.0;
    ++bins[static_cast<unsigned>(s*num_bins)];
    double d = dot_product(n,v2);
    
    w *= m_est.wgt(d);
    
    // down weight the line point if near the image edge
    double bwx = 1.0, bwy = 1.0;
    double px = (1-s)*p0.x() + s*p1.x();
    double py = (1-s)*p0.y() + s*p1.y();
    if( px < border_)
      bwx = px/border_;
    else if(px > bnx)
      bwx = 1.0 - (px-bnx)/border_;
    if( py < border_)
      bwy = py/border_;
    else if(py > bny)
      bwy = 1.0 - (py-bny)/border_;
    
    if(bwx <= 0.0 || bwy <= 0.0)
      continue;
    assert(bwx >= 0.0);
    assert(bwy >= 0.0);
    
    w *= vcl_min(bwx,bwy);
                  
    matches.push_back(vnl_vector_fixed<double,3>(s,d,w));
  }
  
  for(vcl_vector<vnl_vector_fixed<double,3> >::iterator itr = matches.begin()+init_matches_size;
      itr != matches.end(); ++itr)
  {
    vnl_vector_fixed<double,3>& m = *itr;
    unsigned int cnt = bins[static_cast<unsigned>(m[0]*num_bins)];
    assert(cnt > 0);
    m[2] /= cnt;
  }
}


namespace
{
  //: a faster function for updating symmetric matrix M
  inline void incM_svvT(vnl_matrix<double>& M, double s, 
                        const vnl_vector<double>& v)
  {
    const unsigned int n = v.size();
    assert(n == M.columns());
    assert(n == M.rows());
    double** Md = M.data_array();
    
    //: the diagonal
    for (unsigned int i=0; i<n; ++i)
      Md[i][i] += s*v[i]*v[i];
    
    //: off diagonal
    for (unsigned int i=0; i<n; ++i){
      for(unsigned int j=i+1; j<n; ++j){
        Md[i][j] += s*v[i]*v[j];
        Md[j][i] = Md[i][j];
      }
    }
  }
}

//: Compute the terms for optimization from a set of curves and Jacobians
//  adds to the matrix \a M and vector \a b passed in
void modrec_vehicle_fit::
compute_curve_opt_terms(const vcl_vector<vcl_vector<vgl_point_2d<double> > >& curves,
                        const vcl_vector<vcl_vector<vnl_matrix<double> > >& J,
                        const vil_image_view<float>& edge_map,
                        vnl_matrix<double>& M,
                        vnl_vector<double>& b,
                        double& total_weight,
                        double& wgt_residual) const
{
  for(unsigned int i=0; i<curves.size(); ++i)
  {
    const vcl_vector<vgl_point_2d<double> >& curve = curves[i];
    const vcl_vector<vnl_matrix<double> >& Ji = J[i];
    for(unsigned int j=1; j<curve.size(); ++j)
    {
      const vgl_point_2d<double>& p0 = curve[j-1];
      const vgl_point_2d<double>& p1 = curve[j];
      
      vcl_vector<vnl_vector_fixed<double,3> > matches;
      compute_line_matches(p0,p1,edge_map,matches);
            
      vgl_vector_2d<double> v = p1-p0;
      
      vnl_vector<double> n(2);
      n[0] = v.y(); 
      n[1] = -v.x();
      n.normalize();
      
      vnl_vector<double> J0 = n*Ji[j-1];
      vnl_vector<double> J1 = n*Ji[j];
      
      // since all point on this edge are a linear combination of
      // J0 and J1, it is more efficient to compute the coefficients
      // and then add the matrices and vectors only once.
      double Msum0 = 0.0, Msum1 = 0.0;
      double bsum0 = 0.0, bsum1 = 0.0;
      for(unsigned int k=0; k<matches.size(); ++k)
      {
        const vnl_vector_fixed<double,3>& m = matches[k];
        double w = m[2]*m[2];
        const double& e = m[1];
        const double& s1 = m[0];
        double s0 = (1.0-m[0]);
        Msum0 += w*s0*s0;
        Msum1 += w*s1*s1;
        bsum0 += w*e*s0;
        bsum1 += w*e*s1;
        total_weight += w;
        wgt_residual += w*e*e;
      }
      // add the weighted matrix and vector terms.
      incM_svvT(M,Msum0,J0); //M += Msum0*outer_product(J0,J0);
      incM_svvT(M,Msum1,J1); //M += Msum1*outer_product(J1,J1);
      b += bsum0*J0;
      b += bsum1*J1;
    }
  }
}


//: Compute the residuals of the optimization
void modrec_vehicle_fit::
compute_curve_residuals(const vcl_vector<vcl_vector<vgl_point_2d<double> > >& curves,
                        const vil_image_view<float>& edge_map,
                        double& total_weight,
                        double& wgt_residual) const
{
  for(unsigned int i=0; i<curves.size(); ++i)
  {
    const vcl_vector<vgl_point_2d<double> >& curve = curves[i];
    for(unsigned int j=1; j<curve.size(); ++j)
    {
      const vgl_point_2d<double>& p0 = curve[j-1];
      const vgl_point_2d<double>& p1 = curve[j];
      
      vcl_vector<vnl_vector_fixed<double,3> > matches;
      compute_line_matches(p0,p1,edge_map,matches);
      
      for(unsigned int k=0; k<matches.size(); ++k)
      {
        const vnl_vector_fixed<double,3>& m = matches[k];
        total_weight += m[2];
        wgt_residual += m[2]*m[1]*m[1];
      }
    }
  }
}


//: Compute the prior terms and update M and b
double modrec_vehicle_fit::
compute_prior(const modrec_pca_vehicle& mesh,
              const vgl_vector_3d<double>& translation,
              const vgl_rotation_3d<double>& rotation,
              vnl_matrix<double>& M, 
              vnl_vector<double>& b)
{
  // add regularization terms
  double residual = 0.0;
  const vnl_vector<double>& params = mesh.params();
  for(unsigned int i=0; i<num_pc_; ++i)
  {
    M(i,i) += lambda_;
    b[i] += -params[i] * lambda_;
    residual += vnl_math_sqr(-params[i] * lambda_);
  }
  unsigned int base_idx = num_pc_;
  if(options_[3]) // tz
  {
    M(tzi_,tzi_) += vnl_math_sqr(lambda_ * 0.01); // standard dev of 0.01 meters
    //b[base_idx] += 0.0;
  }
  if(options_[4]) // rx
  {
    M(rxi_,rxi_) += vnl_math_sqr(lambda_ * 0.1); // standard dev of 0.1 radians
    //b[base_idx] += 0.0;
  }
  if(options_[5]) // ry
  {
    M(ryi_,ryi_) += vnl_math_sqr(lambda_ * 0.1); // standard dev of 0.1 radians
    //b[base_idx] += 0.0;
  }
  
  return residual;
}


//: Apply the solution vector to update the shape and pose parameters
void modrec_vehicle_fit::apply_solution(const vnl_vector<double>& soln,
                                        modrec_pca_vehicle& mesh,
                                        vgl_vector_3d<double>& translation,
                                        vgl_rotation_3d<double>& rotation)
{
  if(options_[0] && num_pc_ > 0)
  {
    vnl_vector<double> p(mesh.params());
    for(unsigned int i=0; i<num_pc_; ++i)
      p[i] += soln[i];
    mesh.set_params(p);
    mesh.compute_face_normals();
  }

  double t[3] = {0.0, 0.0, 0.0};
  if(txi_ >= 0) t[0] = soln[txi_];
  if(tyi_ >= 0) t[1] = soln[tyi_];
  if(tzi_ >= 0) t[2] = soln[tzi_];
  translation += rotation*vgl_vector_3d<double>(t[0],t[1],t[2]);
  
  vnl_vector_fixed<double,3> r(0.0,0.0,0.0);
  if(rxi_ >= 0) r[0] = soln[rxi_];
  if(ryi_ >= 0) r[1] = soln[ryi_];
  if(rzi_ >= 0) r[2] = soln[rzi_];
  vnl_vector_fixed<double,3> rot = rotation.as_rodrigues(); 
  rot += rotation*r;
  rotation = vgl_rotation_3d<double>(rot);
}


#if 1
//: Fits the model parameters: PCA, translation, rotation to the image.
void modrec_vehicle_fit::fit_model(unsigned int num_itr, 
                                   modrec_pca_vehicle& mesh,
                                   vgl_vector_3d<double>& translation,
                                   vgl_rotation_3d<double>& rotation)
{  
  if(num_params_ == 0)
    return;
  
  mest_scale_ = estimate_initial_scale(mesh,translation,rotation,init_uncert_)/4;
  if(mest_scale_ < 1.0) // lower bound at 1
    mest_scale_ = 1.0;
  vcl_cout << "initial scale = "<<mest_scale_<<vcl_endl;
  //if(mest_scale_ > 8.0) // upper bound at 8
  //  mest_scale_ = 8.0;
  double init_mest_scale = mest_scale_;
  
  double last_residual = vcl_numeric_limits<double>::infinity();
  vnl_vector<double> soln(num_params_,0.0);
  
  bool compute_visibility = true;
  for(unsigned int k=0; k<num_itr; ++k)
  {
    vnl_vector<double> b(num_params_,0.0);
    vnl_matrix<double> M(num_params_,num_params_,0.0);
    double total_weight = 0.0;
    double wgt_residual = 0.0;
    compute_all_opt_terms(mesh, translation, rotation, M, b, 
                          total_weight, wgt_residual, compute_visibility);
    compute_visibility = false;
    
    double residual = wgt_residual/total_weight;
    M /= total_weight;
    b /= total_weight;
    residual += compute_prior(mesh, translation, rotation, M, b);
    
    if(residual < last_residual)
    {
      last_residual = residual;
    }
    else{
      // revert the solution to try again.
      apply_solution(-soln,mesh,translation,rotation);
      --k;
      if(mest_scale_ <= 1)
        break;
      last_residual = vcl_numeric_limits<double>::infinity();
      mest_scale_ /= vcl_sqrt(2);
      if(mest_scale_ < 1)
        mest_scale_ = 1.0;
      //compute_visibility = true;
      continue;
    }
    
    //vnl_svd<double> svd_M(M);
    //soln = svd_M.solve(b);
    soln = vnl_cholesky(M).solve(b);
    
    apply_solution(soln,mesh,translation,rotation);

    //vcl_cout << " res = "<<residual<<" scale = "<<mest_scale_<<vcl_endl;
  }
  mest_scale_ = init_mest_scale;
}


#else
//: Fits the model parameters: PCA, translation, rotation to the image.
// use Levenberg-Marquardt
void modrec_vehicle_fit::fit_model(unsigned int num_itr, 
                                   modrec_pca_vehicle& mesh,
                                   vgl_vector_3d<double>& translation,
                                   vgl_rotation_3d<double>& rotation)
{  
  if(num_params_ == 0)
    return;
  
  mest_scale_ = estimate_initial_scale(mesh,translation,rotation,init_uncert_)/4;
  if(mest_scale_ < 1.0) // lower bound at 1
    mest_scale_ = 1.0;
  double init_mest_scale = mest_scale_;
  
  double last_residual = vcl_numeric_limits<double>::infinity();
  vnl_vector<double> soln(num_params_,0.0);
  
  double alpha = 0.1;
  vnl_matrix<double> last_M;
  vnl_vector<double> last_b;
  
  bool compute_visibility = true;
  for(unsigned int k=0; k<num_itr; ++k)
  {
    vnl_vector<double> b(num_params_,0.0);
    vnl_matrix<double> M(num_params_,num_params_,0.0);
    double total_weight = 0.0;
    double wgt_residual = 0.0;
    compute_all_opt_terms(mesh, translation, rotation, M, b, 
                          total_weight, wgt_residual, compute_visibility);
    compute_visibility = false;
    
    double residual = wgt_residual/total_weight;
    M /= total_weight;
    b /= total_weight;
    residual += compute_prior(mesh, translation, rotation, M, b);
    
    if(residual < last_residual)
    {
      last_residual = residual;
      alpha /= 2.0;
      
      last_M = M;
      last_b = b;
    }
    else{
      // revert the solution to try again.
      apply_solution(-soln,mesh,translation,rotation);
      --k;
      if(alpha < 1e5 && b.magnitude() > 1e-6){
        alpha *= 5.0;
        M = last_M;
        b = last_b;
      }
      else{
        
        if(mest_scale_ <= 1)
          break;
        alpha = 0.1;
        last_residual = vcl_numeric_limits<double>::infinity();
        mest_scale_ /= vcl_sqrt(2);
        if(mest_scale_ < 1)
          mest_scale_ = 1.0;
        //compute_visibility = true;
        continue;
      }
    }
    
    // Add alpha to the diagonal for Levenberg-Marquardt 
    for(unsigned int i=0; i<num_params_; ++i)
      M(i,i) += alpha;
    
    
    vnl_svd<double> svd_M(M);
    soln = svd_M.solve(b);
    
    apply_solution(soln,mesh,translation,rotation);
    
    vcl_cout << "alpha = "<<alpha<<" res = "<<residual<<" scale = "<<mest_scale_<< " grad mag = "<<b.magnitude()<<vcl_endl;
  }
  mest_scale_ = init_mest_scale;
}
#endif


//: One iteration of fitting PCA, translation, rotation to the image.
// return true if the iteration successfully reached a lower residual
bool modrec_vehicle_fit::fit_model_once(modrec_pca_vehicle& mesh,
                                          vgl_vector_3d<double>& translation,
                                          vgl_rotation_3d<double>& rotation,
                                          vnl_vector<double>& soln,
                                          double& last_residual,
                                          bool compute_visibility)
{  
  vnl_vector<double> b(num_params_,0.0);
  vnl_matrix<double> M(num_params_,num_params_,0.0);
  double total_weight = 0.0;
  double wgt_residual = 0.0;
  compute_all_opt_terms(mesh, translation, rotation, M, b, 
                        total_weight, wgt_residual, compute_visibility);
  
  double residual = wgt_residual/total_weight;
  M /= total_weight;
  b /= total_weight;
  residual += compute_prior(mesh, translation, rotation, M, b);
  
  if(residual < last_residual)
  {
    last_residual = residual;
  }
  else{
    // revert the solution to try again.
    apply_solution(-soln,mesh,translation,rotation);
    return false;
  }
  
  soln = vnl_cholesky(M).solve(b);
  
  apply_solution(soln,mesh,translation,rotation);
  
  vcl_cout << " res = "<<residual<<" scale = "<<mest_scale_<<vcl_endl;
  
  return true;
}


//: Evaluate the residual at the current state and scale
// this function is for generating a plot of the error surface
double modrec_vehicle_fit::
evaluate_residual(const modrec_pca_vehicle& mesh,
                  const vgl_vector_3d<double>& t,
                  const vgl_rotation_3d<double>& R,
                  double scale,
                  bool compute_visibility)
{
  mest_scale_ = scale;
  
  vnl_vector<double> b(num_params_,0.0);
  vnl_matrix<double> M(num_params_,num_params_,0.0);
  double total_weight = 0.0;
  double wgt_residual = 0.0;
  compute_all_opt_terms(mesh, t, R, M, b, 
                        total_weight, wgt_residual, compute_visibility);
  
  double residual = wgt_residual/total_weight;
  residual += compute_prior(mesh, t, R, M, b);
  
  return residual;
}


//: Compute the edgel matches and weights from a set of curves 
// This is primarily for debugging
void modrec_vehicle_fit::
compute_edgel_matches(const vcl_vector<vcl_vector<vgl_point_2d<double> > >& curves,
                      const vil_image_view<float>& edge_map,
                      vcl_vector<vgl_point_2d<double> >& edgel_snaps,
                      vcl_vector<vgl_point_2d<double> >& edgels,
                      vcl_vector<double>& edgel_weights) const
{
  for(unsigned int i=0; i<curves.size(); ++i)
  {
    const vcl_vector<vgl_point_2d<double> >& curve = curves[i];
    for(unsigned int j=1; j<curve.size(); ++j)
    {
      const vgl_point_2d<double>& p0 = curve[j-1];
      const vgl_point_2d<double>& p1 = curve[j];
      
      vcl_vector<vnl_vector_fixed<double,3> > matches;
      compute_line_matches(p0,p1,edge_map,matches);
      
      vgl_vector_2d<double> v = p1-p0;
      vgl_vector_2d<double> n(v.y(),-v.x());
      normalize(n);
      for(unsigned int k=0; k<matches.size(); ++k)
      {
        const vnl_vector_fixed<double,3>& m = matches[k];
        double s0 = 1.0-m[0], s1 = m[0];
        edgel_snaps.push_back(vgl_point_2d<double>(s0*p0.x()+s1*p1.x(), 
                                                   s0*p0.y()+s1*p1.y()));
        edgels.push_back(edgel_snaps.back()+n*m[1]);
        edgel_weights.push_back(m[2]);
      }
    }
  }
}


//: Compute all edgel matches and weights in view \a idx
// This is primarily for debugging
void modrec_vehicle_fit::
last_edgel_matches(const modrec_pca_vehicle_projector& projector, 
                   const vil_image_view<float>& edge_map,
                   vcl_vector<vgl_point_2d<double> >& edgel_snaps,
                   vcl_vector<vgl_point_2d<double> >& edgels,
                   vcl_vector<double>& edgel_weights) const
{
  const vcl_vector<vcl_vector<vgl_point_2d<double> > >& contours = projector.contours();
  const vcl_vector<vcl_vector<vgl_point_2d<double> > >& parts = projector.parts();
  
  compute_edgel_matches(contours, edge_map,
                        edgel_snaps, edgels, edgel_weights);
  compute_edgel_matches(parts, edge_map,
                        edgel_snaps, edgels, edgel_weights);
}


//: Compute the number of pixel size edge points with an edge neighbor within \a dist_thresh
// return the total samples and number of matches
void modrec_vehicle_fit::
compute_edgel_coverage(const vcl_vector<vcl_vector<vgl_point_2d<double> > >& curves, 
                       const vil_image_view<float>& edge_map,
                       double dist_thresh,
                       unsigned int& num_match,
                       unsigned int& num_total) const
{
  num_match = 0;
  num_total = 0;
  for(unsigned int i=0; i<curves.size(); ++i)
  {
    const vcl_vector<vgl_point_2d<double> >& curve = curves[i];
    for(unsigned int j=1; j<curve.size(); ++j)
    {
      const vgl_point_2d<double>& p0 = curve[j-1];
      const vgl_point_2d<double>& p1 = curve[j];
      
      vcl_vector<vnl_vector_fixed<double,3> > matches;
      compute_line_matches(p0,p1,edge_map,matches);
      
      vgl_vector_2d<double> v = p1-p0;
      unsigned int num_bins = static_cast<unsigned int>(vcl_floor(v.length()));
      num_total += num_bins;
      vcl_vector<bool> matched(num_bins,false);
      
      for(unsigned int k=0; k<matches.size(); ++k)
      {
        const vnl_vector_fixed<double,3>& m = matches[k];
        if(vcl_abs(m[1]) >= dist_thresh)
          continue;
        
        int bin = static_cast<int>(vcl_floor(m[0]*num_bins));
        if(bin >= num_bins) bin = num_bins-1;
        if(bin < 0) bin = 0;
        
        if(!matched[bin])
        {
          ++num_match;
          matched[bin] = true;
        }
      }
    }
  }
}


//: Compute the number of pixel size edge points with an edge neighbor within \a dist_thresh
// return the total samples and number of matches for both contours and part boundaries
void modrec_vehicle_fit::
last_edgel_coverage(const modrec_pca_vehicle_projector& projector, 
                    const vil_image_view<float>& edge_map,
                    double dist_thresh,
                    unsigned int& num_contour_match,
                    unsigned int& num_contour_total,
                    unsigned int& num_part_match,
                    unsigned int& num_part_total) const
{
  const vcl_vector<vcl_vector<vgl_point_2d<double> > >& contours = projector.contours();
  const vcl_vector<vcl_vector<vgl_point_2d<double> > >& parts = projector.parts();
  
  compute_edgel_coverage(contours, edge_map, dist_thresh,
                         num_contour_match, num_contour_total);
  compute_edgel_coverage(parts, edge_map, dist_thresh,
                         num_part_match, num_part_total);
}


//: estimate the initial scale given the set of Jacobians
double modrec_vehicle_fit::
estimate_initial_scale(const vnl_vector<double>& sigma,
                       const vcl_vector<vcl_vector<vnl_matrix<double> > >& J)
{
  double d=0;
  double d2=0;
  double d_max = 0;
  unsigned int num=0;
  for(unsigned int i=0; i<J.size(); ++i){
    for(unsigned int j=0; j<J[i].size(); ++j){
      const vnl_matrix<double>& M = J[i][j];
      for(unsigned int k=0; k<M.columns(); ++k){
        double dist = sigma[k]*M.get_column(k).magnitude();
        ++num;
        d += dist;
        d2 += dist*dist;
        if(dist>d_max)
          d_max = dist;
      }
    }
  }
  d /= num;
  d2 /= num;
  d2 -= d*d;
  d2 = vcl_sqrt(d2);
  vcl_cout << "mean "<<d<<" std "<<d2<<" max "<<d_max<<vcl_endl;
  
  return d;
}

