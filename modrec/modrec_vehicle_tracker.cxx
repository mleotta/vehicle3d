// This is modrec/modrec_vehicle_tracker.cxx
//=========================================================================
//:
// \file
// \brief Fit the PCA vehicle to a video
//
//          Copyright Matthew Leotta 2006 - 2010.
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file ../LICENSE_1_0.txt or copy at
//          http://www.boost.org/LICENSE_1_0.txt)
//
//=========================================================================

#include "modrec_vehicle_tracker.h"

#include <modrec/modrec_pca_vehicle_projector.h>
#include <modrec/modrec_vehicle_fit.h>
#include <modrec/modrec_vehicle_track_init.h>
#include <modrec/modrec_vehicle_fit_video.h>

#include <dbpro/dbpro_delay.h>
#include <dbpro/dbpro_basic_processes.h>
#include <dbpro/filters/dbvidl2_source.h>
#include <dbpro/filters/dbvidl2_frame_to_resource.h>
#include <dbpro/filters/dbil_gauss_filter.h>
#include <dbpro/filters/dbil_sobel_1x3_filter.h>
#include <dbpro/filters/dbil_subpix_edge_filter.h>
#include <dbpro/filters/dbil_grad_sqr_filter.h>
#include <dbpro/filters/dbil_transform3_1_filter.h>
#include <dbpro/filters/dbil_morphology_filters.h>
#include <dbpro/filters/dbbgm.h>

#include <vpdl/vpdt/vpdt_gaussian.h>
#include <vpdl/vpdt/vpdt_mixture_of.h>
#include <vpdl/vpdt/vpdt_update_mog.h>
#include <vpdl/vpdt/vpdt_distribution_accessors.h>
#include <vpdl/vpdt/vpdt_gaussian_detector.h>
#include <vpdl/vpdt/vpdt_mixture_detector.h>

#include <bbgm/bbgm_image_of.h>
#include <bbgm/bbgm_image_sptr.h>
#include <bbgm/bbgm_image_of.txx>

#include <vil/vil_convert.h>
#include <vil/algo/vil_blob_finder.h>
#include <vil/algo/vil_find_peaks.h>
#include <vgl/vgl_polygon.h>
#include <vgl/vgl_area.h>

extern "C" {
#include <klt/klt.h>
}


//: Dummy binary IO functions needed to satisfy bbgm
template <class T>
void vsl_b_read(vsl_b_istream& is, vpdt_mixture_of<T>& dummy){}
template <class T>
void vsl_b_write(vsl_b_ostream& os, const vpdt_mixture_of<T>& dummy){}



//: A metric for edgel distances
struct modrec_edgel_metric
{
  //: the data type used for the metric tensor
  typedef vnl_vector_fixed<float,3> covar_type;
  //: the data type used for the field
  typedef vnl_vector_fixed<float,3> F;
  //: the data type used for scalars
  typedef vpdt_field_traits<F>::scalar_type T;
  //: the data type used for vectors
  typedef vpdt_field_traits<F>::vector_type vector;
  //: the data type used for matrices
  typedef vpdt_field_traits<F>::matrix_type matrix;
  
  //: Compute the Mahalanobis distance between two points
  static inline T distance(const F& pt1, const F& pt2, const covar_type& c)
  {
    return vcl_sqrt(sqr_distance(pt1,pt2,c));
  }
  
  //: Compute the square Mahalanobis distance between two points
  static inline T sqr_distance(const F& pt1, const F& pt2, const covar_type& c)
  {
    double val = 0.0;
    double tmp = pt1[0]-pt2[0];
    val += tmp*tmp/c[0];
    tmp = pt1[1]-pt2[1];
    val += tmp*tmp/c[1];
  
    tmp = pt1[2]-pt2[2];
    while(tmp > vnl_math::pi)
      tmp -= 2*vnl_math::pi;
    while(tmp < -vnl_math::pi)
      tmp += 2*vnl_math::pi;
    val += tmp*tmp/c[2];
    return val;
  }
  
  //: Compute the square Mahalanobis distance and also the derivative \a g wrt \a pt1
  static inline T sqr_distance_deriv(const F& pt1, const F& pt2,
                                     const covar_type& c, vector& g)
  {
    double val = 0.0;
    double tmp = pt1[0]-pt2[0];
    g[0] = tmp/c[0];
    val += tmp*g[0];
    g[0] *= 2;
    tmp = pt1[1]-pt2[1];
    g[1] = tmp/c[1];
    val += tmp*g[1];
    g[1] *= 2;
    
    tmp = pt1[2]-pt2[2];
    while(tmp > vnl_math::pi)
      tmp -= 2*vnl_math::pi;
    while(tmp < -vnl_math::pi)
      tmp += 2*vnl_math::pi;
    g[2] = tmp/c[2];
    val += tmp*g[2];
    g[2] *= 2;
    return val;
  }
  
  //: Compute the covariance matrix (metric tensor) at a point
  // \note this metric is independent of the point
  static inline void compute_covar(matrix& covar, const F& pt, const covar_type& c)
  {
    covar(0,1) = covar(0,2) = covar(1,2) = 0.0;
    covar(1,0) = covar(2,0) = covar(2,1) = 0.0;
    covar(0,0) = c[0];
    covar(1,1) = c[1];
    covar(2,2) = c[2];
  }
  
  //: Compute the determinant of the covariance matrix (metric tensor) at a point
  // \note this metric is independent of the point
  static inline T covar_det(const F& pt, const covar_type& c)
  {
    return c[0]*c[1]*c[2];
  }
};


//: Update the statistics given a 1D Gaussian distribution and a learning rate
// \note if rho = 1/(num observations) then this just an online cumulative average
void vpdt_update_gaussian(vpdt_gaussian<vnl_vector_fixed<float,3>,
                                        vnl_vector_fixed<float,3>,
                                        modrec_edgel_metric> & gaussian,
                          float rho,
                          const vnl_vector_fixed<float,3>& sample )
{  
  // the complement of rho (i.e. rho+rho_comp=1.0)
  float rho_comp = 1.0f - rho;
  // the difference vector between the sample and the mean
  vnl_vector_fixed<float,3> diff = sample - gaussian.mean;
  while(diff[2] > vnl_math::pi)
    diff[2] -= 2*vnl_math::pi;
  while(diff[2] < -vnl_math::pi)
    diff[2] += 2*vnl_math::pi;
  
  // update the covariance
  vpdt_update_covariance<vnl_vector_fixed<float,3>,vnl_vector_fixed<float,3> >
      ::increment(gaussian.covar, rho, diff);
  gaussian.covar *= rho_comp;
  
  // update the mean
  gaussian.mean += (rho * diff);
  while(gaussian.mean[2] > vnl_math::pi)
    gaussian.mean[2] -= 2*vnl_math::pi;
  while(gaussian.mean[2] < -vnl_math::pi)
    gaussian.mean[2] += 2*vnl_math::pi;
}


//: Update the statistics given a Gaussian distribution and a learning rate
// \param min_var forces the variance to stay above this limit
// \note If the same sample is observed repeatedly, the variances will
// converge to the minimum value parameter rather than zero.
void vpdt_update_gaussian(vpdt_gaussian<vnl_vector_fixed<float,3>,
                                        vnl_vector_fixed<float,3>,
                                        modrec_edgel_metric>& gaussian,
                          float rho,
                          const vnl_vector_fixed<float,3>& sample,
                          float min_var)
{
  vpdt_update_gaussian(gaussian, rho, sample);
  vpdt_update_covariance<vnl_vector_fixed<float,3>,vnl_vector_fixed<float,3> >
      ::enforce_min(gaussian.covar, min_var);
}



//: Extract the subpixel locations and an existence mask
class extract_subpix_filter : public dbpro_filter
{
public:
  
  //: Constructor
  extract_subpix_filter(){}
  
  //: Destructor
  virtual ~extract_subpix_filter(){}
  
  //: Execute this process
  dbpro_signal execute()
  {
    assert(input_type_id(0) == typeid(vil_image_resource_sptr));
    vil_image_resource_sptr image = input<vil_image_resource_sptr>(0);
    if(!image)
      return DBPRO_INVALID;
    
    vil_image_view<float> edge_map = image->get_view();
    if(!edge_map)
      return DBPRO_INVALID;
    
    const unsigned int ni = edge_map.ni();
    const unsigned int nj = edge_map.nj();
    
    vil_image_view<float> edgels(ni,nj,3);
    edgels.fill(0.0f);
    vil_image_view<bool> mask(ni,nj);
    
    for(unsigned int j=0; j<nj; ++j){
      for(unsigned int i=0; i<ni; ++i){
        mask(i,j) = edge_map(i,j,0) > 0.0;
        if(mask(i,j)){
          float theta = edge_map(i,j,1);
          float offset = edge_map(i,j,2);
          edgels(i,j,0) = i + vcl_cos(theta)*offset;
          edgels(i,j,1) = j + vcl_sin(theta)*offset;
          edgels(i,j,2) = theta;
        }
        else
        {
          edgels(i,j,0) = -100;
          edgels(i,j,1) = -100;
          edgels(i,j,2) = 0;
        }
      }
    }
    
    output(0,vil_new_image_resource_of_view(edgels));
    output(1,vil_new_image_resource_of_view(mask));
    
    return DBPRO_VALID;
  }

};


//: Estimate a translation to align the edge maps
class est_translation_filter : public dbpro_filter
{
public:
  
  //: Constructor
  est_translation_filter(){}
  
  //: Destructor
  virtual ~est_translation_filter(){}
  
  //: Execute this process
  dbpro_signal execute()
  {
    assert(input_type_id(0) == typeid(vil_image_resource_sptr));
    vil_image_resource_sptr last_image = input<vil_image_resource_sptr>(0);
    if(!last_image){
      vcl_cout << "first image"<< vcl_endl;
      output(0,vnl_vector_fixed<double,2>(0,0));
      return DBPRO_VALID;
    }
    
    assert(input_type_id(1) == typeid(vil_image_resource_sptr));
    vil_image_resource_sptr curr_image = input<vil_image_resource_sptr>(1);
    if(!curr_image)
      return DBPRO_INVALID;
    
    vil_image_view<float> last = last_image->get_view();
    vil_image_view<float> curr = curr_image->get_view();
    if(!last || !curr)
      return DBPRO_INVALID;
    
    const unsigned int ni = last.ni();
    const unsigned int nj = last.nj();
    
    double n_xx = 0, n_xy = 0, n_yy = 0;
    double d_x = 0, d_y = 0;
    
    unsigned matches = 0;
    for(unsigned int j=0; j<nj; ++j){
      for(unsigned int i=0; i<ni; ++i){
        if(curr(i,j,0) <= 0 || last(i,j,0) <= 0)
          continue;
        if(vcl_abs(curr(i,j,0)-last(i,j,0)) > 10.0)
          continue;
        double adiff = curr(i,j,1) - last(i,j,1);
        while(adiff > vnl_math::pi)
          adiff -= 2*vnl_math::pi;
        while(adiff < -vnl_math::pi)
          adiff += 2*vnl_math::pi;
        if(vcl_abs(adiff) > 0.1)
          continue;
        ++matches;
        
        double w = curr(i,j,0);
        double theta = curr(i,j,1);
        double d = w*(curr(i,j,2) - last(i,j,2));
        double nx = w*vcl_cos(theta);
        double ny = w*vcl_sin(theta);
        n_xx += nx*nx;
        n_xy += nx*ny;
        n_yy += ny*ny;
        d_x += d*nx;
        d_y += d*ny;
      }
    }
    
    vcl_cout << "num matches = "<<matches<<vcl_endl;
    double det = n_xx*n_yy - n_xy*n_xy;
    double ox = (d_x*n_yy - d_y*n_xy)/det;
    double oy = (d_y*n_xx - d_x*n_xy)/det;
    vcl_cout << "offset is "<<ox<<", "<<oy<<vcl_endl;
    
    output(0,vnl_vector_fixed<double,2>(ox,oy));
    
    return DBPRO_VALID;
  }
  
};


//: Align the edge maps with a translation
class edgemap_translation_filter : public dbpro_filter
{
public:
  
  //: Constructor
  edgemap_translation_filter(){}
  
  //: Destructor
  virtual ~edgemap_translation_filter(){}
  
  //: Execute this process
  dbpro_signal execute()
  {
    assert(input_type_id(0) == typeid(vil_image_resource_sptr));
    vil_image_resource_sptr image = input<vil_image_resource_sptr>(0);
    if(!image)
      return DBPRO_INVALID;
    
    vil_image_view<float> in_edge_map = image->get_view();
    
    assert(input_type_id(1) == typeid(vnl_vector_fixed<double,2>));
    vnl_vector_fixed<double,2> offset = input<vnl_vector_fixed<double,2> >(1);
    
    const unsigned int ni = in_edge_map.ni();
    const unsigned int nj = in_edge_map.nj();
    
    vil_image_view<float> out_edge_map(ni,nj,3);
    
    for(unsigned int j=0; j<nj; ++j){
      for(unsigned int i=0; i<ni; ++i){
        out_edge_map(i,j,0) = in_edge_map(i,j,0);
        out_edge_map(i,j,1) = in_edge_map(i,j,1);
        double theta = in_edge_map(i,j,1);
        double d = vcl_cos(theta)*offset[0] + vcl_sin(theta)*offset[1];
        out_edge_map(i,j,2) = in_edge_map(i,j,2) - d;
      }
    }
    
    
    output(0, vil_new_image_resource_of_view(out_edge_map));
    
    return DBPRO_VALID;
  }
  
};


//: mask some edges in an edgemap
class mask_edgemap_filter : public dbpro_filter
{
public:
  
  //: Constructor
  mask_edgemap_filter(){}
  
  //: Destructor
  virtual ~mask_edgemap_filter(){}
  
  //: Execute this process
  dbpro_signal execute()
  {
    assert(input_type_id(0) == typeid(vil_image_resource_sptr));
    vil_image_resource_sptr image = input<vil_image_resource_sptr>(0);
    if(!image)
      return DBPRO_INVALID;
    
    vil_image_view<float> in_edge_map = image->get_view();
    
    assert(input_type_id(1) == typeid(vil_image_resource_sptr));
    vil_image_resource_sptr mask_image = input<vil_image_resource_sptr>(1);
    if(!mask_image)
      return DBPRO_INVALID;
    
    vil_image_view<bool> mask = mask_image->get_view();
    
    const unsigned int ni = in_edge_map.ni();
    const unsigned int nj = in_edge_map.nj();
    
    vil_image_view<float> out_edge_map(ni,nj,3);
    
    for(unsigned int j=0; j<nj; ++j){
      for(unsigned int i=0; i<ni; ++i){
        if(mask(i,j)){
          out_edge_map(i,j,0) = 0.0f;
          out_edge_map(i,j,1) = 0.0f;
          out_edge_map(i,j,2) = 0.0f;
        }
        else{
          out_edge_map(i,j,0) = in_edge_map(i,j,0);
          out_edge_map(i,j,1) = in_edge_map(i,j,1);
          out_edge_map(i,j,2) = in_edge_map(i,j,2);
        }
      }
    }
    
    output(0, vil_new_image_resource_of_view(out_edge_map));
    
    return DBPRO_VALID;
  }
  
};


//: Suppress all non-maximal edges and estimate subpixel locations
class dbil_subpix_point_filter : public dbpro_filter
{
public:
  //: Constructor
  dbil_subpix_point_filter(float threshold=10.0f) : threshold_(threshold) {}
  
  //: Execute this process
  dbpro_signal execute()
  {
    assert(input_type_id(0) == typeid(vil_image_resource_sptr));
    vil_image_resource_sptr in_img = input<vil_image_resource_sptr>(0);
    vil_image_view<float> image = in_img->get_view();
    
    vcl_vector<double> px, py, val;
    vil_find_peaks_3x3_subpixel(px,py,val,image,threshold_);
    
    vcl_vector<vgl_point_2d<double> > pts(px.size());
    for(unsigned int i=0; i<px.size(); ++i)
      pts[i].set(px[i],py[i]);
    
    output(0, pts);
    return DBPRO_VALID;
  }
  
  float threshold_;
  
};


//: detect blobs in a BG detection image
class blob_detector_filter : public dbpro_filter
{
public:
  
  //: Constructor
  blob_detector_filter(double min_area = 4000) : min_area_(min_area){}
  
  //: Destructor
  virtual ~blob_detector_filter(){}
  
  //: Execute this process
  dbpro_signal execute()
  {
    assert(input_type_id(0) == typeid(vil_image_resource_sptr));
    vil_image_resource_sptr image = input<vil_image_resource_sptr>(0);
    if(!image)
      return DBPRO_INVALID;
    
    vil_image_view<bool> bin_image = image->get_view();

    const unsigned int ni = bin_image.ni();
    const unsigned int nj = bin_image.nj();
    work_image_.set_size(ni,nj);
    
    // set the work image to the inverse of bin_image
    for(unsigned int i=0; i<ni; ++i)
      for(unsigned int j=0; j<nj; ++j)
        work_image_(i,j) = !bin_image(i,j);
    
    vil_blob_finder blob_finder;
    blob_finder.set_work_image(work_image_);
    vcl_vector<int> bi,bj;
    unsigned int min_len = vcl_sqrt(min_area_*4*3.14159);
    vcl_vector<vgl_polygon<double> > polys;
    while (blob_finder.next_4con_region(bi,bj))
    {
      if(bi.size() < min_len)
        continue;
      
      vcl_vector<vgl_point_2d<double> > pts;
      bool on_boundary = false;
      for(unsigned int i=0; i<bi.size(); ++i){
        if(bi[i] <= 0 || bi[i] >= ni-1 || bj[i] <= 0 || bj[i] >= nj-1){
          on_boundary = true;
          break;
        }
        pts.push_back(vgl_point_2d<double>(bi[i],bj[i]));
      }
      if(on_boundary)
        continue;
      
      vgl_polygon<double> poly(pts);
      if(vgl_area(poly) > min_area_)
        polys.push_back(poly);
    }
    
    output(0, polys);
    
    return DBPRO_VALID;
  }
  double min_area_;
  vil_image_view<bool> work_image_;
};


//: Compute KLT tracked points
class klt_flow_filter : public dbpro_filter
{
public:
  
  //: Constructor
  klt_flow_filter() 
  : klt_tc_(KLTCreateTrackingContext())
  {
    klt_tc_->sequentialMode = 1;
  }
  
  //: Destructor
  virtual ~klt_flow_filter()
  {
    KLTFreeTrackingContext(klt_tc_);
  }
  
  //: Execute this process
  dbpro_signal execute()
  {
    typedef vcl_pair<vgl_point_2d<double>,vgl_vector_2d<double> > pv_pair;
    vcl_vector<pv_pair> flow;
    
    assert(input_type_id(0) == typeid(vil_image_resource_sptr));
    vil_image_resource_sptr image1 = input<vil_image_resource_sptr>(0);
    if(!image1)
      return DBPRO_INVALID;
    
    assert(input_type_id(1) == typeid(vcl_vector<vgl_point_2d<double> >));
    vcl_vector<vgl_point_2d<double> > pts = 
        input<vcl_vector<vgl_point_2d<double> > >(1);
    
    last_pts_.swap(pts);
    
    vil_image_view<vxl_byte> img1 = image1->get_view();
    const unsigned int ni = img1.ni();
    const unsigned int nj = img1.nj();
    if(!last_image_){
      last_image_.set_size(ni,nj);
      vil_convert_cast(img1,last_image_);
      vcl_cout << "sizeof(vxl_byte) " << sizeof(vxl_byte)
               <<" sizeof(KLT_PixelType) "<<sizeof(KLT_PixelType)<<vcl_endl;
      output(0, flow);
      return DBPRO_VALID;
    }
    curr_image_.set_size(ni,nj);
    vil_convert_cast(img1,curr_image_);
    
    
    // Set up the feature list
    KLT_FeatureList fl = KLTCreateFeatureList(pts.size());
    for(unsigned int i=0; i<pts.size(); ++i){
      fl->feature[i]->x = pts[i].x();
      fl->feature[i]->y = pts[i].y();
    }
    
    // Now, get the imagery into a linear buffer
    KLT_PixelType* klt_img1 = last_image_.top_left_ptr();
    KLT_PixelType* klt_img2 = curr_image_.top_left_ptr();
  
    // Track the points
    KLTTrackFeatures(klt_tc_, klt_img1, klt_img2, ni, nj, fl);
    
    for(unsigned int i=0; i<pts.size(); ++i){
      if(!fl->feature[i]->val){
        vgl_point_2d<double> pt(fl->feature[i]->x, fl->feature[i]->y);
        vgl_vector_2d<double> v(pt.x()-pts[i].x(), pt.y()-pts[i].y());
        flow.push_back(pv_pair(pt,v));
      }
    }
    
    
    KLTFreeFeatureList(fl);
    
    // swap image buffers
    vil_image_view<KLT_PixelType> tmp = last_image_;
    last_image_ = curr_image_;
    curr_image_ = tmp;
     
    output(0, flow);
    
    return DBPRO_VALID;
  }
  
  KLT_TrackingContext klt_tc_;
  vil_image_view<KLT_PixelType> curr_image_;
  vil_image_view<KLT_PixelType> last_image_;
  vcl_vector<vgl_point_2d<double> > last_pts_;
};


//: Adjust the flow vectors with the translation
class flow_translation_filter : public dbpro_filter
{
public:
  
  //: Constructor
  flow_translation_filter(){}
  
  //: Destructor
  virtual ~flow_translation_filter(){}
  
  //: Execute this process
  dbpro_signal execute()
  {
    
    typedef vcl_pair<vgl_point_2d<double>,vgl_vector_2d<double> > pv_pair;
    
    assert(input_type_id(0) == typeid(vcl_vector<pv_pair>));
    vcl_vector<pv_pair> flow = input<vcl_vector<pv_pair> >(0);
    
    if(flow.empty()){
      output(0, flow);
      return DBPRO_VALID;
    }
    
    assert(input_type_id(1) == typeid(vnl_vector_fixed<double,2>));
    vnl_vector_fixed<double,2> offset1 = input<vnl_vector_fixed<double,2> >(1);
    vgl_vector_2d<double> o1(offset1[0],offset1[1]);
    
    assert(input_type_id(2) == typeid(vnl_vector_fixed<double,2>));
    vnl_vector_fixed<double,2> offset2 = input<vnl_vector_fixed<double,2> >(2);
    vgl_vector_2d<double> o2(offset2[0],offset2[1]);
    
    vcl_vector<pv_pair> t_flow;
    for(unsigned int i=0; i<flow.size(); ++i){
      vgl_point_2d<double> p(flow[i].first - o2);
      vgl_vector_2d<double> v(flow[i].second - (o2 - o1));
      if(v.sqr_length() > 2.0)
        t_flow.push_back(pv_pair(p,v));
    }

    
    output(0, t_flow);
    
    return DBPRO_VALID;
  }
  
};


//: initialize tracks of vehicles
class track_init_filter : public dbpro_filter
{
public:
  track_init_filter(const vpgl_perspective_camera<double>& cam = vpgl_perspective_camera<double>())
  : track_init_(cam), tracking_enabled_(true) {}
  
  //: Destructor
  virtual ~track_init_filter(){}
  
  //: Execute this process
  dbpro_signal execute()
  {
    typedef vcl_pair<vgl_point_2d<double>,vgl_vector_2d<double> > pv_pair;
    
    assert(input_type_id(0) == typeid(vcl_vector<vgl_polygon<double> >));
    vcl_vector<vgl_polygon<double> > silhouettes = 
        input<vcl_vector<vgl_polygon<double> > >(0);
    
    assert(input_type_id(1) == typeid(vcl_vector<pv_pair>));
    vcl_vector<pv_pair> flow = input<vcl_vector<pv_pair> >(1);
    
    assert(input_type_id(2) == typeid(vcl_vector<modrec_vehicle_state>));
    vcl_vector<modrec_vehicle_state> states = 
        input<vcl_vector<modrec_vehicle_state> >(2);
    
    if(!tracking_enabled_){
      output(0, vcl_vector<modrec_vehicle_state>());
      return DBPRO_VALID;
    }
    
    if(silhouettes.empty()){
      output(0, states);
      return DBPRO_VALID;
    }
    
    track_init_.find_states(silhouettes, flow, states);
    output(0, states);
    
    return DBPRO_VALID;
  }
  
  //: Helper class to initialize tracks 
  modrec_vehicle_track_init track_init_;
  
  bool tracking_enabled_;
};


namespace{
// Compute the minimum eigenvalue of symmetric matrix
// | a c |
// | c b |
// for detection of KLT points
float min_eigenvalue(float a, float b, float c){
  double d=a-b;
  return (a + b - vcl_sqrt(d*d + 4*c*c))/2.0;
}
}



//: correct track positions by fitting to images
class track_correct_filter : public dbpro_filter
{
public:
  track_correct_filter(modrec_vehicle_fit_video* optimizer)
  : optimizer_(optimizer) {}
  
  //: Destructor
  virtual ~track_correct_filter(){}
  
  //: Execute this process
  dbpro_signal execute()
  {
    
    assert(input_type_id(0) == typeid(vcl_vector<modrec_vehicle_state>));
    vcl_vector<modrec_vehicle_state> states =
        input<vcl_vector<modrec_vehicle_state> >(0);
        
    for(int i=0; i<states.size(); ++i){
      if(!optimizer_->correct_state(states[i]) || states[i].params.inf_norm() > 1.0)
      {
        // bad correction, destroy state
        states.erase(states.begin()+i);
        --i;
      }
    }
    
    output(0, states);
    
    return DBPRO_VALID;
  }
  
  modrec_vehicle_fit_video* optimizer_;
};


//: predict tracks changes with a motion model 
class track_predict_filter : public dbpro_filter
{
public:
  track_predict_filter(double t=1.0/30) : time_(t) {}
  
  //: Destructor
  virtual ~track_predict_filter(){}
  
  //: Execute this process
  dbpro_signal execute()
  {
    
    assert(input_type_id(0) == typeid(vcl_vector<modrec_vehicle_state>));
    vcl_vector<modrec_vehicle_state> states =
        input<vcl_vector<modrec_vehicle_state> >(0);
    
    vcl_vector<modrec_vehicle_state> pstates;
    for(unsigned int i=0; i<states.size(); ++i){
      pstates.push_back(modrec_circ_motion_predict(states[i],time_));
    }
    
    output(0, pstates);
    
    return DBPRO_VALID;
  }
  
  //: the time interval between frames
  double time_;
  
};


// a null sink that allows disabling
class optional_sink : public dbpro_sink
{
public:
  optional_sink() : enabled_(true) {}
  
  //: Execute the process
  dbpro_signal execute() { return DBPRO_VALID; }
  
  virtual bool enabled() const { return enabled_; }
  
  void set_enabled(bool enabled) { enabled_ = enabled; }
  
private:
  bool enabled_;
};


//: Constructor
modrec_vehicle_tracker::modrec_vehicle_tracker(modrec_vehicle_fit_video* optimizer)
{
  // create background modeling processes
  init_bgm_pro();
  // create edge background modeling processes
  init_ebgm_pro();
  
  vil_structuring_element se;
  se.set_to_disk(2.0);
  
  //graph_.enable_debug();
  graph_["source"]      = new dbvidl2_source(NULL);
  graph_["to_resource"] = new dbvidl2_frame_to_resource(dbvidl2_frame_to_resource::ALLOW_WRAP,
                                                        VIL_PIXEL_FORMAT_BYTE,VIDL_PIXEL_COLOR_RGB,
                                                        dbvidl2_frame_to_resource::PLANES,
                                                        dbvidl2_frame_to_resource::REUSE_MEMORY);
  
  graph_["to_rsc_mono"] = new dbvidl2_frame_to_resource(dbvidl2_frame_to_resource::ALLOW_WRAP,
                                                        VIL_PIXEL_FORMAT_BYTE,VIDL_PIXEL_COLOR_MONO,
                                                        dbvidl2_frame_to_resource::PLANES,
                                                        dbvidl2_frame_to_resource::ALLOCATE_MEMORY);
                                                       //dbvidl2_frame_to_resource::REUSE_MEMORY);
  graph_["gauss_img"]   = new dbil_gauss_filter<vxl_byte,float>();
  graph_["grad_ij"]     = new dbil_sobel_1x3_filter<float,float>();
  graph_["edge_map"]    = new dbil_subpix_edge_filter<float,float>();
  graph_["grad2"]       = new dbil_grad_sqr_filter<float,float>();
  graph_["gauss_grad2"] = new dbil_gauss_filter<float,float>();
  typedef float (*eig_func_ptr)(float a, float b, float c);
  graph_["min_eigval"]  = new dbil_transform3_1_filter<float,float,eig_func_ptr>(min_eigenvalue);
  graph_["klt_pts"]     = new dbil_subpix_point_filter();
  
  graph_["bin_erode1"]  = new dbil_binary_erode_filter(se);
  graph_["bin_dilate1"] = new dbil_binary_dilate_filter(se);
  graph_["bin_erode2"]  = new dbil_binary_erode_filter(se);
  graph_["bin_dilate2"] = new dbil_binary_dilate_filter(se);
  
  graph_["e_delay"]     = new dbpro_delay(1,vil_image_resource_sptr(NULL));
  graph_["est_t"]       = new est_translation_filter();
  graph_["t_delay"]     = new dbpro_delay(1,vnl_vector_fixed<double,2>(0.0,0.0));
  graph_["t_edge_map"]  = new edgemap_translation_filter();
  graph_["m_edge_map"]  = new mask_edgemap_filter();
  
  graph_["x_edgels"]    = new extract_subpix_filter();
  graph_["blob_detect"] = new blob_detector_filter();
  graph_["track_init"]  = new track_init_filter();
  graph_["s_delay"]     = new dbpro_delay(1,vcl_vector<modrec_vehicle_state>());
  graph_["trk_predict"] = new track_predict_filter();
  graph_["trk_correct"] = new track_correct_filter(optimizer);
  
  graph_["opt_flow"]    = new klt_flow_filter();
  graph_["t_flow"]      = new flow_translation_filter();
  //graph_["f_delay"]     = new dbpro_delay(1,vil_image_resource_sptr(NULL));
  
  graph_["disp_sink"]   = new optional_sink();
  graph_["trk_sink"]    = new optional_sink();
  graph_["bgm_sink"]    = new dbpro_null_sink();
  
  
  // intensity background modeling
  graph_["to_resource"] ->connect_input(0,graph_["source"],0);
  graph_["bg_update"]   ->connect_input(0,graph_["bg_delay"],0);
  graph_["bg_update"]   ->connect_input(1,graph_["to_resource"],0);
  graph_["bg_delay"]    ->connect_input(0,graph_["bg_update"],0);
  graph_["bg_detect"]   ->connect_input(0,graph_["bg_update"],0);
  graph_["bg_detect"]   ->connect_input(1,graph_["to_resource"],0);
  
  // morphological operations on BG detection
  graph_["bin_dilate1"] ->connect_input(0,graph_["bg_detect"],0);
  graph_["bin_erode1"]  ->connect_input(0,graph_["bin_dilate1"],0);
  graph_["bin_erode2"]  ->connect_input(0,graph_["bin_erode1"],0);
  graph_["bin_dilate2"] ->connect_input(0,graph_["bin_erode2"],0);
  graph_["blob_detect"] ->connect_input(0,graph_["bin_dilate2"],0);
  
  // edge detection
  graph_["to_rsc_mono"] ->connect_input(0,graph_["source"],0);
  graph_["gauss_img"]   ->connect_input(0,graph_["to_rsc_mono"],0);
  graph_["grad_ij"]     ->connect_input(0,graph_["gauss_img"],0);
  graph_["edge_map"]    ->connect_input(0,graph_["grad_ij"],0);
  
  // point detection
  graph_["grad2"]       ->connect_input(0,graph_["grad_ij"],0);
  graph_["gauss_grad2"] ->connect_input(0,graph_["grad2"],0);
  graph_["min_eigval"]  ->connect_input(0,graph_["gauss_grad2"],0);
  graph_["klt_pts"]     ->connect_input(0,graph_["min_eigval"],0);
  
  // edge stabilization 
  graph_["e_delay"]     ->connect_input(0,graph_["t_edge_map"],0);
  graph_["est_t"]       ->connect_input(0,graph_["e_delay"],0);
  graph_["est_t"]       ->connect_input(1,graph_["edge_map"],0);
  graph_["t_edge_map"]  ->connect_input(0,graph_["edge_map"],0);
  graph_["t_edge_map"]  ->connect_input(1,graph_["est_t"],0);
  graph_["t_delay"]     ->connect_input(0,graph_["est_t"],0);
  
  // edge background modeling
  graph_["bge_detect"]  ->connect_input(0,graph_["bge_update"],0);
  graph_["bge_detect"]  ->connect_input(1,graph_["x_edgels"],0);
  graph_["bge_detect"]  ->connect_input(2,graph_["x_edgels"],1);
  graph_["bge_delay"]   ->connect_input(0,graph_["bge_update"],0);
  graph_["bge_update"]  ->connect_input(0,graph_["bge_delay"],0);
  graph_["bge_update"]  ->connect_input(1,graph_["x_edgels"],0);
  graph_["bge_update"]  ->connect_input(2,graph_["x_edgels"],1);
  graph_["x_edgels"]    ->connect_input(0,graph_["t_edge_map"],0); 

  // mask foreground edges
  graph_["m_edge_map"]  ->connect_input(0,graph_["t_edge_map"],0);
  graph_["m_edge_map"]  ->connect_input(1,graph_["bge_detect"],0);
  
  // optical flow tracking for orientation
  //graph_["opt_flow"]    ->connect_input(0,graph_["f_delay"],0);
  //graph_["opt_flow"]    ->connect_input(1,graph_["to_rsc_mono"],0);
  //graph_["f_delay"]     ->connect_input(0,graph_["to_rsc_mono"],0);
  graph_["opt_flow"]    ->connect_input(0,graph_["to_rsc_mono"],0);
  graph_["opt_flow"]    ->connect_input(1,graph_["klt_pts"],0);
  graph_["t_flow"]      ->connect_input(0,graph_["opt_flow"],0);
  graph_["t_flow"]      ->connect_input(1,graph_["t_delay"],0);
  graph_["t_flow"]      ->connect_input(2,graph_["est_t"],0);
  
  // initialize vehicle tracks
  graph_["trk_predict"] ->connect_input(0,graph_["s_delay"],0);
  graph_["track_init"]  ->connect_input(0,graph_["blob_detect"],0);
  graph_["track_init"]  ->connect_input(1,graph_["t_flow"],0);
  graph_["track_init"]  ->connect_input(2,graph_["trk_predict"],0);
  
  // track vehicles
  graph_["trk_correct"] ->connect_input(0,graph_["track_init"],0);
  graph_["trk_correct"] ->connect_input(1,graph_["m_edge_map"],0);
  graph_["s_delay"]     ->connect_input(0,graph_["trk_correct"],0);
  
  // sink drivers
  graph_["disp_sink"]   ->connect_input(0,graph_["to_resource"],0);
  graph_["disp_sink"]   ->connect_input(1,graph_["m_edge_map"],0);
  graph_["disp_sink"]   ->connect_input(1,graph_["klt_pts"],0);
  
  graph_["trk_sink"]    ->connect_input(0,graph_["trk_correct"],0);
  
  graph_["bgm_sink"]    ->connect_input(0,graph_["bge_update"],0);
  graph_["bgm_sink"]    ->connect_input(1,graph_["bg_update"],0);
    
  graph_.init();
}

//: initialize the intensity background modeling processes
void modrec_vehicle_tracker::init_bgm_pro()
{
  float init_var = 6502.5f, g_thresh = 3.0f, min_stdev = 5.1f;
  int max_components=3, window_size=300;
  float dist = 2.5f, wt = 0.7f;
  
  typedef vpdt_gaussian<vnl_vector_fixed<float,3>,vnl_vector_fixed<float,3> > gauss_type;
  typedef gauss_type::field_type field_type;
  typedef vpdt_mixture_of<gauss_type> mix_gauss_type;
  typedef vpdt_mog_lm_updater<mix_gauss_type> updater_type;
  typedef vpdt_gaussian_mthresh_detector<gauss_type> gdetector_type;
  typedef vpdt_mixture_top_weight_detector<mix_gauss_type, gdetector_type> detector_type;
  
  
  gauss_type init_gauss(field_type(0.0f), field_type(init_var));
  updater_type updater(init_gauss, max_components, g_thresh,
                       min_stdev, window_size);
  mix_gauss_type mixture;
  bbgm_image_sptr bg_model = new bbgm_image_of<mix_gauss_type>(0,0,mixture);
  
  gdetector_type gmd(dist);
  detector_type detector(gmd, wt);
  
  graph_["bg_update"]   = new dbbgm_update_filter<updater_type,vxl_byte>(updater);
  graph_["bg_detect"]   = new dbbgm_detect_filter<detector_type,vxl_byte>(detector);
  graph_["bg_delay"]    = new dbpro_delay(1,bg_model);
}


//: initialize the edge background modeling processes
void modrec_vehicle_tracker::init_ebgm_pro()
{
  float init_var = 0.1f, g_thresh = 3.0f, min_stdev = 0.1f;
  int max_components=3, window_size=300;
  float dist = 2.5f, wt = 0.7f;
  
  typedef vpdt_gaussian<vnl_vector_fixed<float,3>,
                        vnl_vector_fixed<float,3>,
                        modrec_edgel_metric>  gauss_type;
  typedef gauss_type::field_type field_type;
  typedef vpdt_mixture_of<gauss_type> mix_gauss_type;
  typedef vpdt_mog_lm_updater<mix_gauss_type> updater_type;
  typedef vpdt_gaussian_mthresh_detector<gauss_type> gdetector_type;
  typedef vpdt_mixture_top_weight_detector<mix_gauss_type, gdetector_type> detector_type;
  
  
  gauss_type init_gauss(field_type(0.0f), field_type(init_var));
  updater_type updater(init_gauss, max_components, g_thresh,
                       min_stdev, window_size);
  mix_gauss_type mixture;
  bbgm_image_sptr bg_model = new bbgm_image_of<mix_gauss_type>(0,0,mixture);
  
  gdetector_type gmd(dist);
  detector_type detector(gmd, wt);
  
  graph_["bge_update"]   = new dbbgm_update_filter<updater_type,float>(updater);
  graph_["bge_detect"]   = new dbbgm_detect_filter<detector_type,float>(detector);
  graph_["bge_delay"]    = new dbpro_delay(1,bg_model);
}


//: Assign a camera 
void modrec_vehicle_tracker::set_camera(const vpgl_perspective_camera<double>& camera)
{
  track_init_filter* filter = dynamic_cast<track_init_filter*>(graph_["track_init"].ptr());
  if(filter){
    filter->track_init_.set_camera(camera);
  }
}


//: Set the sun direction for shadow casting
void modrec_vehicle_tracker::set_sun_direction(const vgl_vector_3d<double>& sun_dir)
{
  track_init_filter* filter = dynamic_cast<track_init_filter*>(graph_["track_init"].ptr());
  if(filter){
    filter->track_init_.set_sun_direction(sun_dir);
  }
}


//: Assign the PCA vehicle model
void modrec_vehicle_tracker::set_vehicle_model(const modrec_pca_vehicle& vehicle)
{
  track_init_filter* filter = dynamic_cast<track_init_filter*>(graph_["track_init"].ptr());
  if(filter){
    filter->track_init_.set_vehicle(vehicle);
  }
}


//: set the video input stream
void modrec_vehicle_tracker::set_istream(const vidl_istream_sptr& istream)
{
  dbvidl2_source* source = dynamic_cast<dbvidl2_source*>(graph_["source"].ptr());
  if(source){
    source->set_stream(istream);
  }
}


//: add an observer of input video frames 
void modrec_vehicle_tracker::add_video_observer(const dbpro_observer_sptr& obs)
{
  assert(graph_["to_resource"].ptr());
  graph_["to_resource"]->add_output_observer(0,obs);
}


//: add an observer of the background detection image
void modrec_vehicle_tracker::add_bg_observer(const dbpro_observer_sptr& obs)
{
  //assert(graph_["bg_detect"].ptr());
  //graph_["bg_detect"]->add_output_observer(0,obs);
  assert(graph_["bin_dilate2"].ptr());
  graph_["bin_dilate2"]->add_output_observer(0,obs);
}


//: add an observer of detected vehicle silhouette polygons
void modrec_vehicle_tracker::add_silhouette_observer(const dbpro_observer_sptr& obs)
{
  assert(graph_["blob_detect"].ptr());
  graph_["blob_detect"]->add_output_observer(0,obs);
}


//: add an observer of detected vehicle hypotheses
void modrec_vehicle_tracker::add_hypotheses_observer(const dbpro_observer_sptr& obs)
{
  assert(graph_["track_init"].ptr());
  graph_["track_init"]->add_output_observer(0,obs);
}


//: add an observer of the vehicle tracks
void modrec_vehicle_tracker::add_track_observer(const dbpro_observer_sptr& obs)
{
  assert(graph_["trk_correct"].ptr());
  graph_["trk_correct"]->add_output_observer(0,obs);
}


//: add an observer of the edge map image
void modrec_vehicle_tracker::add_edgemap_observer(const dbpro_observer_sptr& obs)
{
  assert(graph_["m_edge_map"].ptr());
  graph_["m_edge_map"]->add_output_observer(0,obs);
}


//: add an observer of the optical flow
void modrec_vehicle_tracker::add_optical_flow_observer(const dbpro_observer_sptr& obs)
{
  assert(graph_["t_flow"].ptr());
  graph_["t_flow"]->add_output_observer(0,obs);
}


//: add an observer of the point map image
void modrec_vehicle_tracker::add_pointmap_observer(const dbpro_observer_sptr& obs)
{
  assert(graph_["min_eigval"].ptr());
  graph_["min_eigval"]->add_output_observer(0,obs);
}


//: add an observer of the points
void modrec_vehicle_tracker::add_point_observer(const dbpro_observer_sptr& obs)
{
  assert(graph_["klt_pts"].ptr());
  graph_["klt_pts"]->add_output_observer(0,obs);
}


//: enable tracking
void modrec_vehicle_tracker::enable_tracking(bool enable_track)
{
  track_init_filter* track_init = dynamic_cast<track_init_filter*>(graph_["track_init"].ptr());
  if(track_init)
    track_init->tracking_enabled_ = enable_track;
}


//: enable processes for display
void modrec_vehicle_tracker::enable_display(bool enable_display)
{
  optional_sink* opt_sink = dynamic_cast<optional_sink*>(graph_["disp_sink"].ptr());
  if(opt_sink)
    opt_sink->set_enabled(enable_display);
}


//: enable or disable shape estimation
void modrec_vehicle_tracker::set_estimate_shape(bool val)
{
  track_init_filter* tif = dynamic_cast<track_init_filter*>(graph_["track_init"].ptr());
  assert(tif);
  tif->track_init_.set_estimate_shape(val);
}


