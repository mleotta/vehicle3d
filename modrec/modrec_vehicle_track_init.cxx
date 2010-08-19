// This is modrec/modrec_vehicle_track_init.cxx
//=========================================================================
//:
// \file
// \brief Initialize vehicle tracks from foreground outlines and optical flow
//
//          Copyright Matthew Leotta 2006 - 2010.
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file ../LICENSE_1_0.txt or copy at
//          http://www.boost.org/LICENSE_1_0.txt)
//
//=========================================================================

#include "modrec_vehicle_track_init.h"
#include <modrec/modrec_vehicle_fit_video.h>

#include <vcl_limits.h>
#include <vnl/vnl_math.h>
#include <vgl/vgl_area.h>
#include <vgl/vgl_box_2d.h>
#include <vgl/vgl_intersection.h>


modrec_vehicle_track_init::
modrec_vehicle_track_init(const vpgl_perspective_camera<double>& cam)
: camera_(cam) , init_params_(5), estimate_shape_(true)
{
  // Initial Dodecahedral params
  {
    double minivan[] = {0.168097942941176, 0.047826370588235, 0.097333800000000, -0.019294116470588, -0.014450029411765};
    double sedan[] = {-0.034173060666667, -0.043000999777778, -0.044518354814815, 0.002448365722222, -0.010742823722222};
    double pickup[] = {-0.214965183333333, 0.277647666666667, 0.131580866666667, 0.028651000000000, 0.048347313333333};

    init_params_[DODECAHEDRAL].push_back(vnl_vector<double>(minivan,sizeof(minivan)/sizeof(minivan[0])));
    init_params_[DODECAHEDRAL].push_back(vnl_vector<double>(sedan,sizeof(sedan)/sizeof(sedan[0])));
    init_params_[DODECAHEDRAL].push_back(vnl_vector<double>(pickup,sizeof(pickup)/sizeof(pickup[0])));
  }
  // Initial Ferryman params
  {
    double minivan[] = {0.096096282941176, -0.148172152941176, -0.092961776470588, -0.032739470588235, -0.019361350000000};
    double sedan[] = {-0.051120281925926, 0.021029037555556, 0.042011107962963, 0.010947612037037, -0.008138290000000};
    double pickup[] = {0.204828426666667, 0.283238666666667, -0.123898316666667, -0.006261705000000, 0.073925983333333};
    
    init_params_[FERRYMAN].push_back(vnl_vector<double>(minivan,sizeof(minivan)/sizeof(minivan[0])));
    init_params_[FERRYMAN].push_back(vnl_vector<double>(sedan,sizeof(sedan)/sizeof(sedan[0])));
    init_params_[FERRYMAN].push_back(vnl_vector<double>(pickup,sizeof(pickup)/sizeof(pickup[0])));
  }  
  // Initial Detailed 1 params
  {
    double minivan[] = {0.046971176470588, -0.187059470588235, -0.043333896470588, -0.036514317647059, 0.000084941176471};
    double sedan[] = {-0.036112334085185, 0.043846546111111, 0.036116245185185, 0.018472345000000, 0.001448921481481};
    double pickup[] = {0.216576950066667, 0.177151266666667, -0.231083266666667, -0.059329090000000, -0.012563915000000};
    
    init_params_[DETAILED1].push_back(vnl_vector<double>(minivan,sizeof(minivan)/sizeof(minivan[0])));
    init_params_[DETAILED1].push_back(vnl_vector<double>(sedan,sizeof(sedan)/sizeof(sedan[0])));
    init_params_[DETAILED1].push_back(vnl_vector<double>(pickup,sizeof(pickup)/sizeof(pickup[0])));
  }
  // Initial Detailed 2 params
  {
    double minivan[] = {0.052177516294118, -0.174111247058824, 0.081441645294118, -0.030973511764706, -0.010675894117647};
    double sedan[] = {-0.038001857296296, 0.034931345185185, -0.045013771666667, 0.012124765296296, -0.003273262407407};
    double pickup[] = {0.217973026666667, 0.223444333333333, 0.195942716666667, -0.014787393333333, 0.048405766666667};
    
    init_params_[DETAILED2].push_back(vnl_vector<double>(minivan,sizeof(minivan)/sizeof(minivan[0])));
    init_params_[DETAILED2].push_back(vnl_vector<double>(sedan,sizeof(sedan)/sizeof(sedan[0])));
    init_params_[DETAILED2].push_back(vnl_vector<double>(pickup,sizeof(pickup)/sizeof(pickup[0])));
  }
  // Initial Detailed 3 params
  {
    double minivan[] = {0.0648611875, -0.166070624285714, 0.076543643571429, 0.052718387857143, -0.014917692857143};
    double sedan[] = {-0.061446563446089, 0.037852037835095, -0.026649174155391, -0.031743398805497, -0.066592943551797};
    double pickup[] = {0.216010025, 0.24085167, 0.1798837, -0.00311129167, 0.05172855};
    
    init_params_[DETAILED3].push_back(vnl_vector<double>(minivan,sizeof(minivan)/sizeof(minivan[0])));
    init_params_[DETAILED3].push_back(vnl_vector<double>(sedan,sizeof(sedan)/sizeof(sedan[0])));
    init_params_[DETAILED3].push_back(vnl_vector<double>(pickup,sizeof(pickup)/sizeof(pickup[0])));
  }
}


//: Set the camera
void modrec_vehicle_track_init::
set_camera(const vpgl_perspective_camera<double>& camera) 
{
  camera_ = camera;
  const double height = 1.0;
  
  // compute the homography that maps image points to the ground plane (Z=0)
  vnl_double_3x4 P = camera_.get_matrix();
  vnl_double_3x3 M = P.extract(3,3);
  M.set_column(2,P.get_column(3));
  H_g2i_ = vgl_h_matrix_2d<double>(M);
  H_i2g_ = H_g2i_.get_inverse();
  // compute the homography that maps image points to a world plane
  // that is 'height' above the ground (Z=height).
  M.set_column(2,height*P.get_column(2)+P.get_column(3));
  H_h2i_ = vgl_h_matrix_2d<double>(M);
  H_i2h_ = H_h2i_.get_inverse();
}


//: Set the sun direction for shadow casting
void modrec_vehicle_track_init::
set_sun_direction(const vgl_vector_3d<double>& sun_dir)
{
  sun_dir_ = sun_dir;
}


//: Set the vehicle model
void modrec_vehicle_track_init::
set_vehicle(const modrec_pca_vehicle& vehicle) 
{
  vehicle_ = vehicle;
  
  switch(vehicle_.num_verts())
  {
    case 16:
      vehicle_type_ = DODECAHEDRAL;
      break;
    case 40:
      vehicle_type_ = FERRYMAN;
      break;
    case 455:
      vehicle_type_ = DETAILED1;
      break;
    case 662:
      vehicle_type_ = DETAILED2;
      break;
    case 1444:
      vehicle_type_ = DETAILED3;
      break;
    default:
      vcl_cerr << "Unknown model type with "
               << vehicle_.num_verts() << " vertices" << vcl_endl;
  }
}


//: test the silhouette polygon to see if it is a plausible vehicle
bool modrec_vehicle_track_init::
silhouette_plausible(const vgl_polygon<double>& silhouette)
{
  double area = vgl_area(silhouette);
  double circ = 0.0;
  const vcl_vector<vgl_point_2d<double> >& sheet = silhouette[0];
  const int n = sheet.size();
  vgl_box_2d<double> bbox;
  for(int i=n-1, j=0; j < n; i=j, ++j){
    const vgl_point_2d<double>& p1 = sheet[i];
    const vgl_point_2d<double>& p2 = sheet[j];
    circ += (p2-p1).length();
    bbox.add(p1);
  }
  if(area/bbox.area() < 0.5 || circ/vcl_sqrt(area) > 10.0)
    return false;
  vcl_cout << "area="<<area <<" circ="<< circ<<" box area="<<bbox.area()
  << "box ratio="<<area/bbox.area()<<" circ ratio="<< circ/vcl_sqrt(area)<<vcl_endl;
  return true;
}


//: estimate the Z rotation angle from optical flow vectors
double modrec_vehicle_track_init::
estimate_angle(const vcl_vector<pv_pair>& flow)
{
  // use a histogram to estimate the vehicle direction robustly
  vcl_vector<unsigned int> angle_hist(8,0);
  vcl_vector<vgl_vector_2d<double> > dir_bins(8,vgl_vector_2d<double>(0,0));
  for(unsigned int j=0; j<flow.size(); ++j)
  {
    // get the start and end points of the flow vector
    vgl_homg_point_2d<double> ip1(flow[j].first);
    vgl_homg_point_2d<double> ip2(flow[j].first+flow[j].second);
    // map vector head and tail to the ground plane 
    vgl_point_2d<double> gp1 = H_i2g_*ip1;
    vgl_point_2d<double> gp2 = H_i2g_*ip2;
    // compute the unit flow vector in the ground plane
    vgl_vector_2d<double> dir = normalized(gp2-gp1);
    // bin the vector by angle
    double angle = vcl_atan2(dir.y(),dir.x());
    int bin = (angle/vnl_math::pi+1-0.0625)*4;
    if(bin<0)
      bin = 7;
    ++angle_hist[bin];
    dir_bins[bin] += dir;
  }
  unsigned int max_hist=angle_hist[0], max_bin=0;
  for(unsigned int j=1; j<8; ++j)
  {
    if(angle_hist[j] > max_hist)
    {
      max_hist = angle_hist[j];
      max_bin = j;
    }
  }
  vgl_vector_2d<double> dir = dir_bins[max_bin];
  // add in neighboring bins if they contain at least 75% as many votes
  if(double(angle_hist[(max_bin+1)%8])/max_hist > 0.75)
    dir += dir_bins[(max_bin+1)%8];
  if(double(angle_hist[(max_bin+7)%8])/max_hist > 0.75)
    dir += dir_bins[(max_bin+7)%8];
  
  return vcl_atan2(dir.y(),dir.x());
}


//: adjust the 3-d translation \a t to align the silhouette centroid to \a c. 
vgl_vector_3d<double> modrec_vehicle_track_init:: 
align_centroids(const vgl_point_2d<double>& c,
                const vgl_rotation_3d<double>& R)
{
  vgl_point_2d<double> w = H_i2h_*vgl_homg_point_2d<double>(c);
  vgl_vector_3d<double> t(w.x(),w.y(),0);
  mesh_projector_.project(camera_,vehicle_,R,t,sun_dir_);
  
  vgl_point_2d<double> c2 = vgl_centroid(mesh_projector_.silhouette_polygon());
  vgl_vector_2d<double> diff(c-c2);
  
  for(unsigned int k=0; k<10; ++k){
    vgl_point_2d<double> base = H_g2i_*vgl_homg_point_2d<double>(w.x(),w.y(),1);
    base += c-c2;
    w = H_i2g_*vgl_homg_point_2d<double>(base);
    t = vgl_vector_3d<double>(w.x(),w.y(),0);
    mesh_projector_.reproject(camera_,vehicle_,R,t,sun_dir_);
    c2 = vgl_centroid(mesh_projector_.silhouette_polygon());
    diff = c-c2;
    if(diff.sqr_length() < 0.01)
      break;
  }
  
  return t;
}


//: compute the errors in silhouette alignment
vcl_vector<double> modrec_vehicle_track_init::
silhouette_error(const vgl_polygon<double>& silhouette,
                 vcl_vector<vgl_vector_2d<double> >& norms)
{
  const vcl_vector<vcl_vector<vgl_point_2d<double> > >& sil = mesh_projector_.silhouette();
  norms.clear();
  vcl_vector<vgl_point_2d<double> > pts;
  for(unsigned int i=0; i<sil.size(); ++i){
    const vcl_vector<vgl_point_2d<double> >& si = sil[i];
    if(si.size() < 2)
      continue;
    vgl_vector_2d<double> n1(0,0), n2(0,0);
    pts.push_back(si[0]);
    for(unsigned int j=1; j<si.size(); ++j){
      n2 = normalized(vgl_vector_2d<double>(si[j-1].y()-si[j].y(), 
                                            si[j].x()-si[j-1].x()));
      pts.push_back(si[j]);
      norms.push_back(normalized(n1+n2));
      n1 = n2;
    }  
    norms.push_back(n2);
  }
  
  vcl_vector<double> errors, dist;
  modrec_vehicle_fit_video::
  compute_silhouette_errors(pts,norms,silhouette,errors);
 
  return errors;
}


//: compute the tranlation and mesh params to align the silhouettes
double modrec_vehicle_track_init::
fit_to_silhouette(const vgl_polygon<double>& silhouette,
                  const vgl_rotation_3d<double>& R,
                  vgl_vector_3d<double>& t)
{
  vcl_vector<bool> options(7,false); 
  options[0] = options[1] = options[2] = true;
  unsigned int num_pc = 3;
  vnl_vector<double> init_params(vehicle_.params().extract(num_pc));
  mesh_projector_.project(camera_,vehicle_,R,t,sun_dir_,options,num_pc);
  
  // compute the errors in silhouette alignment
  vcl_vector<vgl_vector_2d<double> > norms;
  vcl_vector<double> errors = silhouette_error(silhouette,norms);
  
  const vcl_vector<vcl_vector<vgl_point_2d<double> > >& sil = mesh_projector_.silhouette();
  
  unsigned int num_finite = 0;
  for(unsigned int i=0; i<errors.size(); ++i)
    if(vnl_math_isfinite(errors[i]))
      ++num_finite;
  
  // not enough error measurements found
  if(num_finite < 5)
    return vcl_numeric_limits<double>::infinity();
  
  
  const vcl_vector<vcl_vector<vnl_matrix<double> > >& Js = 
      mesh_projector_.silhouette_jacobians();
  
  vnl_matrix<double> M(num_finite+num_pc, 5, 0.0);
  vnl_vector<double> b(num_finite+num_pc, 0.0);
  unsigned int k=0; // error term index
  unsigned int c=0; // M row index
  for(unsigned int i=0; i<sil.size(); ++i){
    const vcl_vector<vgl_point_2d<double> >& si = sil[i];
    if(si.size() < 2)
      continue;
    for(unsigned int j=0; j<si.size(); ++j){
      if(vnl_math_isfinite(errors[k])){
        M.set_row(c,norms[k].x()*Js[i][j].get_row(0) + norms[k].y()*Js[i][j].get_row(1));
        b[c] = errors[k];
        ++c;
      }
      ++k;
    }
  }
  // add regularization terms
  for(unsigned int i=0; i<num_pc; ++i){
    M(num_finite+i,i) = num_finite;
    //b[i] = init_params[i];
  }
  
  vnl_vector<double> soln = vnl_svd<double>(M).solve(b);
  vcl_cout << "soln = "<<soln<<vcl_endl;
  
  
  vnl_vector<double> p(vehicle_.params());
  for(unsigned int i=0; i<num_pc; ++i)
    p[i] += soln[i];
  vehicle_.set_params(p);
  vehicle_.compute_face_normals();
  
  t += R*vgl_vector_3d<double>(soln[num_pc],soln[num_pc+1],0);
  
  mesh_projector_.reproject(camera_,vehicle_,R,t,sun_dir_,options,num_pc);
  errors = silhouette_error(silhouette,norms);
  double error_mag = 0.0;
  for(unsigned int i=0; i<errors.size(); ++i)
    if(vnl_math_isfinite(errors[i]))
      error_mag += errors[i]*errors[i];
  
  return error_mag;
}


//: compute the median silhouette_error
double modrec_vehicle_track_init::
median_error(const vgl_polygon<double>& silhouette,
             const vgl_rotation_3d<double>& R,
             vgl_vector_3d<double>& t)
{
  vcl_vector<bool> options(7,false); 
  options[0] = options[1] = options[2] = true;
  unsigned int num_pc = 3;
  vnl_vector<double> init_params(vehicle_.params().extract(num_pc));
  mesh_projector_.project(camera_,vehicle_,R,t,sun_dir_,options,num_pc);
  
  const vcl_vector<vcl_vector<vgl_point_2d<double> > >& sil = mesh_projector_.silhouette();
  vcl_vector<vgl_point_2d<double> > pts;
  vcl_vector<vgl_vector_2d<double> > norms;
  for(unsigned int i=0; i<sil.size(); ++i){
    const vcl_vector<vgl_point_2d<double> >& si = sil[i];
    if(si.size() < 2)
      continue;
    vgl_vector_2d<double> n1(0,0), n2(0,0);
    pts.push_back(si[0]);
    for(unsigned int j=1; j<si.size(); ++j){
      n2 = normalized(vgl_vector_2d<double>(si[j-1].y()-si[j].y(), 
                                            si[j].x()-si[j-1].x()));
      pts.push_back(si[j]);
      norms.push_back(normalized(n1+n2));
      n1 = n2;
    }  
    norms.push_back(n2);
  }
  
  vcl_vector<double> errors, dist;
  modrec_vehicle_fit_video::
      compute_silhouette_errors(pts,norms,silhouette,errors);
  for(unsigned int i=0; i<errors.size(); ++i)
    if(vnl_math_isfinite(errors[i]))
      dist.push_back(vcl_abs(errors[i]));
  vcl_sort(dist.begin(),dist.end());

  // not enough error measurements found
  if(dist.size() < 5)
    return vcl_numeric_limits<double>::infinity();


  const vcl_vector<vcl_vector<vnl_matrix<double> > >& Js = 
    mesh_projector_.silhouette_jacobians();
  
  vnl_matrix<double> M(dist.size()+num_pc, 5, 0.0);
  vnl_vector<double> b(dist.size()+num_pc, 0.0);
  unsigned int k=0; // error term index
  unsigned int c=0; // M row index
  for(unsigned int i=0; i<sil.size(); ++i){
    const vcl_vector<vgl_point_2d<double> >& si = sil[i];
    if(si.size() < 2)
      continue;
    for(unsigned int j=0; j<si.size(); ++j){
      if(vnl_math_isfinite(errors[k])){
        M.set_row(c,norms[k].x()*Js[i][j].get_row(0) + norms[k].y()*Js[i][j].get_row(1));
        b[c] = errors[k];
        ++c;
      }
      ++k;
    }
  }
  // add regularization terms
  for(unsigned int i=0; i<num_pc; ++i){
    M(dist.size()+i,i) = dist.size();
    //b[i] = init_params[i];
  }
  
  vnl_vector<double> soln = vnl_svd<double>(M).solve(b);
  vcl_cout << "soln = "<<soln<<vcl_endl;
  

  vnl_vector<double> p(vehicle_.params());
  for(unsigned int i=0; i<num_pc; ++i)
    p[i] += soln[i];
  vehicle_.set_params(p);
  vehicle_.compute_face_normals();
  
  t += R*vgl_vector_3d<double>(soln[num_pc],soln[num_pc+1],0);
  
  
  return dist.empty()?-1:dist[dist.size()/2];
}


namespace {
vgl_box_2d<double> polygon_bbox(const vgl_polygon<double>& poly)
{
  vgl_box_2d<double> bbox;
  for(unsigned int i=0; i<poly.num_sheets(); ++i){
    for(unsigned int j=0; j<poly[i].size(); ++j){
      bbox.add(poly[i][j]);
    }
  }
  return bbox;
}
  
}

//: match observed silhouettes and flow vectors to existing states
//  return a vector of unmatched silhouettes
vcl_vector<vgl_polygon<double> >
modrec_vehicle_track_init::
match_silhouettes(const vcl_vector<vgl_polygon<double> >& silhouettes,
                  const vcl_vector<pv_pair>& flow,
                        vcl_vector<modrec_vehicle_state>& states)
{
  vcl_vector<vgl_polygon<double> > remain_sils(silhouettes);
  
  vcl_vector<vgl_box_2d<double> > sil_bbox;
  for(unsigned int i=0; i<silhouettes.size(); ++i)
  {
    sil_bbox.push_back(polygon_bbox(silhouettes[i]));
  }
  
  // match predicted states with silhouettes
  for(unsigned int i=0; i<states.size(); ++i)
  {
    vehicle_.set_params(states[i].params);
    vehicle_.compute_face_normals();
    mesh_projector_.project(camera_,vehicle_,
                            states[i].rotation,states[i].translation, sun_dir_);
    vgl_polygon<double> sil = mesh_projector_.silhouette_polygon();
    // FIXME This hack prevents creation of new states when the silhouette computation fails
    if(sil.num_sheets()==0 || sil[0].empty())
      return vcl_vector<vgl_polygon<double> >();
    
    vgl_point_2d<double> c = vgl_centroid(sil);
    double area = vgl_area(sil);
    vgl_box_2d<double> bbox = polygon_bbox(sil);
    
    for(int j=0; j<remain_sils.size(); ++j)
    {
      vgl_box_2d<double> ibox = vgl_intersection(bbox, sil_bbox[j]);
      vcl_cout << j<<" pbox="<<bbox.area()<<" dbox="<<sil_bbox[j].area()<<" ibox="<<ibox.area()
      << " r1="<<ibox.area()/sil_bbox[j].area()<<" r2="<<ibox.area()/bbox.area()<<vcl_endl;
      double aratio = vcl_min(1.0, bbox.area()/sil_bbox[j].area());
      if(ibox.area()/sil_bbox[j].area() > .5*aratio)
      {
        // associate this silhouette to the state if a complete covering
        if(ibox.area()/bbox.area() > .9)
          states[i].silhouette = remain_sils[j];
        // remove the silhouette from consideration
        remain_sils.erase(remain_sils.begin()+j);
        sil_bbox.erase(sil_bbox.begin()+j);
        --j;
      }
    }
  }
  
  return remain_sils;
}


//: Process the silhouettes and flow and estimate initial states
//  Also merge in states predicted from the previous frame
void
modrec_vehicle_track_init::
find_states(const vcl_vector<vgl_polygon<double> >& silhouettes,
            const vcl_vector<pv_pair>& flow,
                  vcl_vector<modrec_vehicle_state>& states)
{      
  // match observed silhouettes and flow vectors to existing states
  vcl_vector<vgl_polygon<double> > remain_sils = 
      match_silhouettes(silhouettes, flow, states);

  vcl_cout << flow.size() <<" total flow vectors"<<vcl_endl;
  for(unsigned int i=0; i<remain_sils.size(); ++i)
  {
    const vgl_polygon<double>& curr_sil = remain_sils[i];
    
    if(!silhouette_plausible(curr_sil))
      continue;
    
    // find flow vectors inside this polygon
    vcl_vector<pv_pair> inside;
    for(unsigned int j=0; j<flow.size(); ++j)
    {
      if(curr_sil.contains(flow[j].first))
        inside.push_back(flow[j]);
    }
    
    if(inside.size() < 5)
      continue;
    vcl_cout << inside.size() <<" flow vectors in polygon "<< i<<vcl_endl;
    
    double angle = estimate_angle(inside);
    vgl_rotation_3d<double> rotation(0,0,angle);

    vcl_cout << "vehicle angle "<<i<<": "<<angle<<vcl_endl;
    
    
    vgl_vector_3d<double> translation;
    double target_area = vgl_area(curr_sil);
    vcl_cout << "silhouette area "<< vgl_area(curr_sil)<<vcl_endl;
    
    if(estimate_shape_){
      double min_error = vcl_numeric_limits<double>::infinity();
      int best_k = -1;
      vnl_vector<double> best_p;
      for(unsigned int k=0; k<init_params_[vehicle_type_].size(); ++k){
        // clear vehicle parameters
        vehicle_.set_params(init_params_[vehicle_type_][k]);
        vehicle_.compute_face_normals();
        
        // estimate the translation
        vgl_point_2d<double> c = vgl_centroid(curr_sil);
        vgl_vector_3d<double> t = align_centroids(c,rotation);
        double sil_area = vgl_area(mesh_projector_.silhouette_polygon());
        
        double fit_error = fit_to_silhouette(curr_sil,rotation,t);
        if(!vnl_math_isfinite(fit_error))
          continue;
        
        vcl_cout << k << " has area "<< sil_area<<" and error "<< fit_error<<vcl_endl;
        
        double pdiff = (init_params_[vehicle_type_][k]-vehicle_.params().extract(5)).magnitude();
        if(pdiff > 0.5)
          continue;
        
        if(fit_error<min_error){
          min_error = fit_error;
          best_k = k;
          best_p = vehicle_.params().extract(5);
          translation = t;
        }
      }
      
      if(best_k >= 0){
        states.push_back(modrec_vehicle_state(best_p,rotation,translation,5,0,true));
        states.back().silhouette = curr_sil;
      }
    }
    // if not estimating shape
    else{
      // estimate the translation
      vgl_point_2d<double> c = vgl_centroid(curr_sil);
      vgl_vector_3d<double> t = align_centroids(c,rotation);
      double sil_area = vgl_area(mesh_projector_.silhouette_polygon());
      double area_ratio = (sil_area > target_area) ? target_area/sil_area : sil_area/target_area;
      if(area_ratio > 0.70){
        states.push_back(modrec_vehicle_state(vehicle_.params(),rotation,t,5,0,false));
        states.back().silhouette = curr_sil;
      }
    }
  }
  
}



