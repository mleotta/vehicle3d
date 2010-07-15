// This is contrib/mleotta/modrec/py_modrec_manager.cxx
#include "py_modrec_manager.h"
//:
// \file
// \author Matt Leotta

#include <vil/vil_image_resource_sptr.h>
#include <vil/vil_load.h>
#include <vil/vil_convert.h>
#include <vil/vil_save.h>
#include <vil/algo/vil_binary_dilate.h>

#include <modrec/modrec_vehicle_mesh.h>
#include <modrec/modrec_vehicle_parts.h>
#include <imesh/imesh_operations.h>
#include <imesh/imesh_fileio.h>
#include <imesh/algo/imesh_transform.h>
#include <imesh/algo/imesh_intersect.h>

#include <vul/vul_file.h>
#include <dbul/dbul_solar_position.h>

#include <vidl/vidl_convert.h>
#include <vidl/vidl_ffmpeg_istream.h>
#include <vidl/vidl_image_list_istream.h>

#include <dbpro/dbpro_observer.h>


//: observer to capture current vehicle tracking states
class track_observer: public dbpro_observer
{
public:
  track_observer(py_modrec_manager& m)
  : manager_(m) {}
  
  //: Called by the process when the data is ready
  virtual bool notify(const dbpro_storage_sptr& data, unsigned long timestamp)
  {
    assert(data);
    if(data->info() == DBPRO_VALID){
      assert(data->type_id() == typeid(vcl_vector<modrec_vehicle_state>));
      const vcl_vector<modrec_vehicle_state>& states = 
      data->data<vcl_vector<modrec_vehicle_state> >();
      
      manager_.set_tracking_states(states);
    }
    return true;
  }
  py_modrec_manager& manager_;
};


//===============================================================

//static manager instance
py_modrec_manager *py_modrec_manager::instance_ = 0;

//: The singleton pattern - only one instance of the manager can occur
py_modrec_manager *py_modrec_manager::instance()
{
  if (!instance_)
  {
    instance_ = new py_modrec_manager();
  }
  return py_modrec_manager::instance_;
}

//===============================================================


//: Constructor
py_modrec_manager::py_modrec_manager()
{
  init_mesh();
  // default to multiview fitting mode
  optimizer_ = &mv_optimizer_;
  video_optimizer_.tracker().add_track_observer(new track_observer(*this));
}


//: Initialize the texture map for detailed meshes
void py_modrec_manager::init_mesh_tex(modrec_pca_vehicle& mesh)
{
  assert(mesh.has_tex_coords());
  
  // remove hidden faces from the texture map
  vcl_set<unsigned int> no_tex, tmp;
  no_tex = mesh.faces().group_face_set("undercarriage");
  tmp = mesh.faces().group_face_set("wheel_back");
  no_tex.insert(tmp.begin(),tmp.end());
  vcl_vector<bool> valid_tex(mesh.num_faces(),true);
  for(vcl_set<unsigned int>::const_iterator itr=no_tex.begin();
      itr!=no_tex.end(); ++itr)
    valid_tex[*itr] = false;
  mesh.set_valid_tex_faces(valid_tex);
}


//: Build a default vehicle mesh from default parameters in modrec
void py_modrec_manager::init_mesh()
{
  modrec_generate_vehicle(modrec_read_vehicle_params(),detailed3_mesh_);
  detailed1_mesh_ = detailed3_mesh_;
  
  imesh_quad_subdivide(detailed3_mesh_,detailed3_mesh_.faces().group_face_set("body"));
  detailed2_mesh_ = detailed3_mesh_;
  imesh_quad_subdivide(detailed3_mesh_,detailed3_mesh_.faces().group_face_set("body"));
  
  imesh_triangulate(detailed1_mesh_);
  detailed1_mesh_.compute_face_normals();
  detailed1_mesh_.build_edge_graph();
  init_mesh_tex(detailed1_mesh_);
  
  imesh_triangulate(detailed2_mesh_);
  detailed2_mesh_.compute_face_normals();
  detailed2_mesh_.build_edge_graph();
  init_mesh_tex(detailed2_mesh_);
  
  imesh_triangulate(detailed3_mesh_);
  detailed3_mesh_.compute_face_normals();
  detailed3_mesh_.build_edge_graph();
  init_mesh_tex(detailed3_mesh_);
  
  mesh_ = &detailed3_mesh_;
  
  
  modrec_generate_dodec_vehicle(modrec_read_vehicle_params(),dodec_mesh_);
  dodec_mesh_.compute_face_normals();
  dodec_mesh_.build_edge_graph();
  
  modrec_generate_ferryman_vehicle(modrec_read_vehicle_params(),ferryman_mesh_);
  ferryman_mesh_.compute_face_normals();
  ferryman_mesh_.build_edge_graph();
  
  video_optimizer_.set_vehicle_model(*mesh_);
}


//: load a camera matrix for frame \a i
bool py_modrec_manager::set_camera(unsigned int frame, 
                                   const vcl_string& filename)
{
  vnl_double_3x4 M;
  norm_cam_ = false;
  vcl_ifstream ifs(filename.c_str());
  ifs >> M;
  
  vcl_string type;
  ifs >> type;
  if(ifs.good() && type == "normalized")
    norm_cam_ = true;
  
  // read enviroment data
  vcl_map<vcl_string,double> env_data;
  vcl_string name;
  double val;
  ifs >> name >> val;
  while(ifs.good()){
    env_data[name]=val;
    ifs >> name >> val;
  }
  ifs.close();
  
  vpgl_perspective_camera<double> camera;
  if(!vpgl_perspective_decomposition(M,camera))
    return false;
  
  if(norm_cam_){
    norm_K_ = camera.get_calibration();
    unsigned int ni=0, nj=0;
    if(is_fit_mode_video() && istream_ && istream_->is_valid())
    {
      ni = istream_->width();
      nj = istream_->height();
    }
    else if(mv_optimizer_.edge_map(frame))
    {
      ni = mv_optimizer_.edge_map(frame).ni();
      nj = mv_optimizer_.edge_map(frame).nj();
    }
    if(ni>0 && nj>0){
      vpgl_calibration_matrix<double> K = norm_K_;
      K.set_x_scale(K.x_scale()*ni);
      K.set_y_scale(K.y_scale()*nj);
      vgl_point_2d<double> pp = K.principal_point();
      pp.x() *= ni;
      pp.y() *= nj;
      K.set_principal_point(pp);
      camera.set_calibration(K);
      vcl_cout << K.get_matrix() << vcl_endl;
    }
  }
  
  // read data to compute sun direction  
  solar_lat_ = 0;
  solar_lon_ = 0;
  if(!env_data.empty()){
    solar_lat_ = 41.8;
    solar_lon_ = -71.4;
    solar_day_ = 0;
    solar_utc_ = 17;
    solar_atn_ = 0;
    vcl_map<vcl_string,double>::const_iterator e;
    if((e = env_data.find("lat")) != env_data.end())
      solar_lat_ = e->second;  
    if((e = env_data.find("lon")) != env_data.end())
      solar_lon_ = e->second; 
    if((e = env_data.find("day")) != env_data.end())
      solar_day_ = static_cast<int>(vcl_floor(e->second)); 
    if((e = env_data.find("utc")) != env_data.end())
      solar_utc_ = e->second; 
    if((e = env_data.find("atn")) != env_data.end())
      solar_atn_ = e->second;
  }
  compute_sun_direction();
  
  if(is_fit_mode_video())
    video_optimizer_.set_camera(camera);
  else
    mv_optimizer_.set_camera(frame_number_,camera);
  
  return true;
}



//: compute the sun direction for shadow casting
void py_modrec_manager::compute_sun_direction()
{
  if(solar_lat_ == 0.0 && solar_lon_ == 0.0)
    sun_dir_ = vgl_vector_3d<double>(0,0,-1);
  
  else{
    double rel_time = 0.0;
    if(istream_){
      double frame_rate = istream_->frame_rate();
      if(frame_rate <= 0.0)
        frame_rate = 30.0;
      rel_time = istream_->frame_number()/frame_rate/3600;
    }
    
    double alt,az;
    dbul_solar_position(solar_day_, solar_utc_+rel_time, solar_lat_, solar_lon_, alt, az);
    double sag = solar_atn_ - az;
    sun_dir_ = vgl_vector_3d<double>(-vcl_cos(sag)*vcl_cos(alt),
                                     -vcl_sin(sag)*vcl_cos(alt),
                                     -vcl_sin(alt));
  }
  if(is_fit_mode_video())
    video_optimizer_.set_sun_direction(sun_dir_);
}


//: load an image file for frame \a i
bool py_modrec_manager::set_image(unsigned int frame, const vcl_string& filename)
{
  vil_image_resource_sptr img = vil_load_image_resource(filename.c_str());
  if(!img)
  {
    vcl_cerr << "unable to load image file: "<<filename << vcl_endl;
    return false;
  }
    
  // detect edges in this image
  vil_image_view<vxl_byte> img_grey = 
      vil_convert_to_grey_using_rgb_weighting(img->get_view());
  mv_optimizer_.detect_and_set_edges(frame,img_grey);
    
  return true;
}


//: Open input video stream
bool py_modrec_manager::set_video(const vcl_string& filename)
{
  if(vul_file::exists(filename) && !vul_file::is_directory(filename))
    istream_ = new vidl_ffmpeg_istream(filename);
  else
    istream_ = new vidl_image_list_istream(filename);
  
  if (!istream_ || !istream_->is_open()){
    return false;
  }
  
  video_optimizer_.set_istream(istream_);
  optimizer_->reset();
  
  istream_->advance();
  if (istream_->is_valid())
  {
    vidl_frame_sptr frame = istream_->current_frame();
    if (frame) {      
      // detect edges in this frame
      vil_image_view<vxl_byte> img_grey;
      if(is_fit_mode_video()){
        if (vidl_convert_to_view(*frame,img_grey,VIDL_PIXEL_COLOR_MONO))
          video_optimizer_.detect_and_set_edges(img_grey);
      }
      else{
        if (vidl_convert_to_view(*frame,img_grey,VIDL_PIXEL_COLOR_MONO))
          mv_optimizer_.detect_and_set_edges(0,img_grey);
      }
      
      frame_number_ = 0;
      return true;
    }
  }
  
  return false;
}


//: clear all images and cameras
void py_modrec_manager::reset_views()
{
  vcl_cout << "clearing views"<<vcl_endl;
  optimizer_->reset();
}


//: load the parts from a file
bool py_modrec_manager::set_parts(const vcl_string& filename)
{
  mesh_->set_parts(modrec_read_vehicle_parts(filename));  
  return true;
}


//: load the pca parameters from a file
bool py_modrec_manager::set_pca(const vcl_string& filename)
{
  vnl_vector<double> mean;
  vnl_vector<double> std_devs;
  vnl_matrix<double> pc;
  if(!imesh_read_pca(filename,mean,std_devs,pc))
  {
    vcl_cerr << "error reading PCA file" << vcl_endl;
    return false;
  }
  
  mesh_->init(mean,std_devs,pc);
  
  // clear the PCA parameters
  mesh_->set_params(vnl_vector<double>(0));
  
  return true;
}


//: load the mesh and projected into PCA space
bool py_modrec_manager::set_mesh(const vcl_string& filename, 
                                 const vcl_string& partsfile)
{
  imesh_mesh new_mesh; 
  if(!imesh_read(filename,new_mesh)){
    vcl_cerr << "error, could not read mesh file: "<<filename<<vcl_endl; 
    return false;
  }
  
  if(mesh_->num_verts() != new_mesh.num_verts()){
    vcl_cerr << "error: mesh must have the same number of vertices as the PCA mesh" <<vcl_endl; 
    vcl_cerr << "PCA: "<<mesh_->num_verts()<<" loaded: "<<new_mesh.num_verts()<<vcl_endl;
    return false;
  }
  
  vcl_auto_ptr<imesh_vertex_array_base> new_verts(new_mesh.vertices().clone());
  mesh_->set_vertices(new_verts);
  mesh_->compute_face_normals();
  
  
  if(!mesh_->params().empty())
  {
    if(partsfile == "")
      mesh_->set_params(mesh_->imesh_pca_mesh::project(mesh_->vertices()));
    else
      mesh_->set_params(mesh_->project(mesh_->vertices(),modrec_read_vehicle_parts(partsfile)));
  }
  
  video_optimizer_.set_vehicle_model(*mesh_);
  
  return true; 
}


//: Choose the type of vehicle model from the enum
void py_modrec_manager::set_vehicle_model(vehicle_model vm)
{
  switch(vm){
    case DODECAHEDRAL:
      mesh_ = &dodec_mesh_;
      break;
    case FERRYMAN:
      mesh_ = &ferryman_mesh_;
      break;
    case DETAILED1:
      mesh_ = &detailed1_mesh_;
      break;
    case DETAILED2:
      mesh_ = &detailed2_mesh_;
      break;
    case DETAILED3:
      mesh_ = &detailed3_mesh_;
      break;
  }
  
  video_optimizer_.set_vehicle_model(*mesh_);
}


//: load the polyhedral approximation mesh
bool py_modrec_manager::set_poly_mesh(const vcl_string& filename)
{
  if(!imesh_read(filename,poly_mesh_)){
    vcl_cerr << "error, could not read mesh file: "<<filename<<vcl_endl; 
    return false;
  }
}


//: set the options for which parameters to optimize
void py_modrec_manager::set_options(const vcl_vector<bool>& options,
                                    unsigned int num_pc)
{
  vcl_cout << "Enable ";
  if(options[1]) vcl_cout << "tx, ";
  if(options[2]) vcl_cout << "ty, ";
  if(options[3]) vcl_cout << "tz, ";
  if(options[4]) vcl_cout << "rx, ";
  if(options[5]) vcl_cout << "ry, ";
  if(options[6]) vcl_cout << "rz, ";
  if(options[0]) vcl_cout << "with "<<num_pc<<" PC"<<vcl_endl;
  else vcl_cout << "without PCs"<<vcl_endl;
  optimizer_->set_options(options,num_pc);
  if(!options[0] || num_pc == 0)
    video_optimizer_.tracker().set_estimate_shape(false);
  else
    video_optimizer_.tracker().set_estimate_shape(true);
}


//: set the scale of the initial uncertainty
void py_modrec_manager::set_init_uncert(double uncert)
{
  optimizer_->set_init_uncert(uncert);
}


//: enable or disable tracking with silhouette terms
void py_modrec_manager::set_track_with_silhouette(bool val)
{
  video_optimizer_.set_track_with_silhouette(val);
}


//: set the translation vector
void py_modrec_manager::set_translation(double tx, double ty, double tz)
{
  t_.set(tx,ty,tz);
  vcl_cout << "translation set to "<<t_<<vcl_endl;
}


//: set the rotation vector
void py_modrec_manager::set_rotation(double rx, double ry, double rz)
{
  R_ = vgl_rotation_3d<double>(vnl_vector_fixed<double,3>(rx,ry,rz));
  vcl_cout << "Rotation set to "<< R_.as_rodrigues() << vcl_endl;
}


//: set the rotation vector
void py_modrec_manager::set_params(const vcl_vector<double>& params)
{
  vnl_vector<double> p(params.size());
  for(unsigned int i=0; i<params.size(); ++i)
    p[i] = params[i];
  vcl_cout << "setting params ["<<p<<"]"<<vcl_endl;
  mesh_->set_params(p);
  mesh_->compute_face_normals();
}


//: set the regularization factor
void py_modrec_manager::set_lambda(double lambda)
{
  optimizer_->set_lambda(lambda);
}


//: set the vehicle tracking states
void py_modrec_manager::set_tracking_states(const vcl_vector<modrec_vehicle_state>& states)
{
  vehicle_states_ = states;
}


//: Enable video mode (true), or multiview mode (false)
void py_modrec_manager::set_fit_mode(bool use_video)
{
  if(use_video)
  {
    optimizer_ = &video_optimizer_;
  }
  else
  {
    optimizer_ = &mv_optimizer_;
  }
}


//: Is the fitting mode video (true) or multiview (false)
bool py_modrec_manager::is_fit_mode_video() const
{
  return optimizer_ == &video_optimizer_;
}


//: fit the vehicle model to the data
void py_modrec_manager::fit_model(unsigned int max_itr)
{
  optimizer_->fit_model(max_itr,*mesh_,t_,R_);
}


//: write an svg file with the last projected curves in view \a i
bool py_modrec_manager::write_svg_curves(unsigned int i,
                                         const vcl_string& filename) const
{
  const modrec_pca_vehicle_projector& projector = is_fit_mode_video() ? 
                                                  video_optimizer_.projector() : 
                                                  mv_optimizer_.projector(i);
  return modrec_write_svg_curves(filename, projector);
}


//: load a mesh as the ground truth mesh
bool py_modrec_manager::set_truth_mesh(const vcl_string& filename)
{
  if(!imesh_read(filename,truth_mesh_)){
    vcl_cerr << "error, could not read truth mesh file: "<<filename<<vcl_endl; 
    return false;
  }

  imesh_triangulate(truth_mesh_);
  truth_mesh_.compute_face_normals();
  truth_kd_tree_ = imesh_build_kd_tree(truth_mesh_);
  
  return true;
}


//: compute the RMS error between the PCA mesh and Ground truth
double py_modrec_manager::compute_error() const
{
  imesh_mesh body = imesh_submesh_from_faces(*mesh_,mesh_->faces().group_face_set("body"));
  imesh_transform_inplace(body, R_, t_);
  while(body.num_verts() < 1000)
    imesh_quad_subdivide(body);
  vcl_cout << "num body vertices = "<<body.num_verts()<<vcl_endl;
  const imesh_vertex_array<3>& verts = body.vertices<3>();
  
  vgl_point_3d<double> cp;
  double error = 0.0;
  double max_dist = 0.0;
  for(unsigned int i=0; i<verts.size(); ++i)
  {
    vgl_point_3d<double> p = verts[i];
    imesh_kd_tree_closest_point(p, truth_mesh_,truth_kd_tree_,cp);
    //imesh_closest_point(verts[i], truth_mesh_, cp);
    double d = (p-cp).length();
    error += d*d;
    if(d > max_dist)
      max_dist =d;
  }
  error /= verts.size();
  
  return vcl_sqrt(error);
  //return max_dist;
}


//: return the number of principal components optimized
unsigned int py_modrec_manager::num_pc() const
{
  return optimizer_->num_pc();
}


//: enable or disable tracking
void py_modrec_manager::enable_tracking(bool enable_track)
{
  video_optimizer_.enable_tracking(enable_track);
}


//: seek the video to the frame number
void py_modrec_manager::video_seek(int fnum)
{
  if(istream_)
    istream_->seek_frame(fnum);
  
  compute_sun_direction();
  
  frame_number_ = fnum;
  vcl_cout << "seek to "<< fnum << "\n";
  
  
  vidl_frame_sptr frame = istream_->current_frame();
  if (frame) {    
    if(is_fit_mode_video()){
      // detect the edges on this image
      vil_image_view<vxl_byte> img_grey;
      if (vidl_convert_to_view(*frame,img_grey,VIDL_PIXEL_COLOR_MONO))
        video_optimizer_.detect_and_set_edges(img_grey);
    }
    else{
      // detect the edges on this image if this is the first time view the frame
      if(fnum >= mv_optimizer_.num_views() || mv_optimizer_.edge_map(fnum)==0){
        vil_image_view<vxl_byte> img_grey;
        if (vidl_convert_to_view(*frame,img_grey,VIDL_PIXEL_COLOR_MONO))
          mv_optimizer_.detect_and_set_edges(fnum,img_grey);
      }
    }
  }
}


//: advance the video to the next frame
bool py_modrec_manager::advance_video()
{  
  dbpro_signal s = video_optimizer_.process_once();
  compute_sun_direction();
  if(s == DBPRO_VALID )
    return true;
  return false;
}


//: return the current video frame number
int py_modrec_manager::current_frame() const
{
  if(!istream_)
    return -1;
  
  return istream_->frame_number();
}


//: evaluate the fitting residual at the current state and given scale
double py_modrec_manager::evaluate_residual(double scale, bool compute_visibility)
{
  return optimizer_->evaluate_residual(*mesh_,t_,R_,scale,compute_visibility);
}


//: get the translation vector
void py_modrec_manager::get_translation(double& tx, double& ty, double& tz) const
{
  tx = t_.x();
  ty = t_.y();
  tz = t_.z();
}


//: get the rotation vector
void py_modrec_manager::get_rotation(double& rx, double& ry, double& rz) const
{
  vnl_vector_fixed<double,3> r = R_.as_rodrigues();
  rx = r[0];
  ry = r[1];
  rz = r[2];
}


//: get the parameter vector
void py_modrec_manager::get_params(vcl_vector<double>& params) const
{
  const vnl_vector<double>& p = mesh_->params();
  params.resize(p.size());
  for(unsigned int i=0; i<p.size(); ++i)
    params[i] = p[i];
}


//: get the type of vehicle model used
void py_modrec_manager::get_vehicle_model(vehicle_model& vm)
{
  if(mesh_ == &dodec_mesh_)
    vm = DODECAHEDRAL;
  else if(mesh_ == &ferryman_mesh_)
    vm = FERRYMAN;
  else if(mesh_ == &detailed1_mesh_)
    vm = DETAILED1;
  else if(mesh_ == &detailed2_mesh_)
    vm = DETAILED2;
  else if(mesh_ == &detailed3_mesh_)
    vm = DETAILED3;
}


//: get the current vehicle tracking states
const vcl_vector<modrec_vehicle_state>& 
py_modrec_manager::get_vehicle_states() const
{
  return vehicle_states_;
}


//: compute the edgel coverage of the contours and part boundaries in the last projection
void py_modrec_manager::
compute_edgel_coverage(double dist_thresh,
                       unsigned int& num_contour_match,
                       unsigned int& num_contour_total,
                       unsigned int& num_part_match,
                       unsigned int& num_part_total) const
{
  num_contour_match = num_contour_total = 0;
  num_part_match = num_part_total = 0;
  for(unsigned int i=0; i<mv_optimizer_.num_views(); ++i)
  {
    unsigned int ncm=0, nct=0, npm=0, npt=0;
    optimizer_->last_edgel_coverage(mv_optimizer_.projector(i),
                                   mv_optimizer_.edge_map(i),
                                   dist_thresh,
                                   ncm,nct,npm,npt);
    num_contour_match += ncm;
    num_contour_total += nct;
    num_part_match += npm;
    num_part_total += npt;
  }
}


namespace{
  vil_image_view<float>
  edge_dist_map(const vil_image_view<float>& edge_map,
                const vcl_vector<vcl_vector<vgl_point_2d<double> > >& curves)
  {
    vcl_vector<vcl_pair<int,int> > edge_pos;
    vcl_vector<modrec_edgel > edgels;
    const unsigned ni = edge_map.ni();
    const unsigned nj = edge_map.nj();
    for(unsigned int j=0; j<nj; ++j){
      for(unsigned int i=0; i<ni; ++i){
        if(edge_map(i,j,0) > 0.0){
          edge_pos.push_back(vcl_pair<int,int>(i,j));
          double theta = edge_map(i,j,1);
          double offset = edge_map(i,j,2);
          double gx = vcl_cos(theta);
          double gy = vcl_sin(theta);
          edgels.push_back(modrec_edgel(i+gx*offset, j+gy*offset,
                                        theta, edge_map(i,j,0)));
        }
      }
    }
    
    vil_image_view<float> dist_edges(ni,nj);
    dist_edges.fill(vcl_numeric_limits<float>::infinity());
    for(unsigned int i=0; i<curves.size(); ++i){
      for(unsigned int j=1; j<curves[i].size(); ++j){
        const vgl_point_2d<double>& p0 = curves[i][j-1];
        const vgl_point_2d<double>& p1 = curves[i][j];
        vgl_vector_2d<double> v = p1-p0;
        vgl_vector_2d<double> n(v.y(),-v.x());
        double len = v.sqr_length();
        v /= len;
        n /= vcl_sqrt(len);
        for(unsigned int k=0; k<edgels.size(); ++k)
        {
          const modrec_edgel& e = edgels[k];
 
          vgl_vector_2d<double> v2 = e-p0;
          double s = dot_product(v,v2);
          if(s < 0.0) continue;
          else if (s > 1.0) continue;
          double d = vcl_abs(dot_product(n,v2));
          if(dist_edges(edge_pos[k].first,edge_pos[k].second) > d)
            dist_edges(edge_pos[k].first,edge_pos[k].second) = d;
          
        }
      }
    }
    return dist_edges;
  }
  
  bool write_poly_curves(const vcl_string& filename,
                         const vcl_vector<vcl_vector<vgl_point_2d<double> > >& curves,
                         unsigned int ni, unsigned int nj)
  {
    vcl_ofstream ofs(filename.c_str());
    if(!ofs.is_open())
      return false;
    
    ofs << "<?xml version=\"1.0\" standalone=\"no\"?>\n"
    << "<!DOCTYPE svg PUBLIC \"-//W3C//DTD SVG 1.1//EN\"\n" 
    << "  \"http://www.w3.org/Graphics/SVG/1.1/DTD/svg11.dtd\">\n"
    << "<svg width=\""<< ni <<"px\" height=\""<< nj <<"px\" version=\"1.1\"\n"
    << "     viewBox=\"0 0 "<< ni <<" "<< nj <<"\" xmlns=\"http://www.w3.org/2000/svg\" preserveAspectRatio=\"none\">\n";
    
    // Draw a box around the image
    ofs << "  <rect x=\"0\" y=\"0\" width=\""<< ni <<"\" height=\""<< nj <<"\" "
    << "fill=\"none\" stroke=\"black\" stroke-width=\"1px\" />\n";
    
    ofs << "  <g id=\"parts\">\n";
    // Draw the parts in red
    for(unsigned int i=0; i<curves.size(); ++i)
    {
      const vcl_vector<vgl_point_2d<double> >& curve = curves[i];
      ofs << "  <polyline fill=\"none\" stroke=\"red\" stroke-width=\"1px\"\n"
      << "           points=\"";
      for(unsigned int j=0; j<curve.size(); ++j)
        ofs << curve[j].x()<<','<<curve[j].y()<<' ';
      ofs << "\" />\n";
    }
    
    ofs << "  </g>\n</svg>\n";
    ofs.close();
    return true;
  }
}


//: compute edge coverage for on-vehicle-edges for mesh and poly_mesh
void py_modrec_manager::
compare_relative_coverage(double dist_thesh,  
                          unsigned int& num_match,
                          unsigned int& num_poly_match,
                          unsigned int& num_total)
{
  num_match = num_poly_match = num_total = 0;
  for(unsigned int i=0; i<mv_optimizer_.num_views(); ++i)
  {
    vil_image_view<float> edge_map;
    edge_map.deep_copy(mv_optimizer_.edge_map(i));
    const unsigned int ni = edge_map.ni(), nj = edge_map.nj();
    
    modrec_pca_vehicle_projector projector(ni,nj);
    projector.project(mv_optimizer_.camera(i),*mesh_,R_,t_);
    
    // clear edges off of the vehicle
    const vil_image_view<double>& depth = projector.depth_map();
    vil_image_view<bool> mask1(ni,nj);
    for(unsigned int j=0; j<nj; ++j){
      for(unsigned int i=0; i<ni; ++i){
        mask1(i,j) = vnl_math_isfinite(depth(i,j));
      }
    }
    vil_image_view<bool> mask2(ni,nj);
    vil_structuring_element se;
    se.set_to_disk(2.5);
    vil_binary_dilate(mask1,mask2,se);
    for(unsigned int j=0; j<nj; ++j){
      for(unsigned int i=0; i<ni; ++i){
        if(!mask2(i,j)){
          edge_map(i,j,0) = 0.0f;
          edge_map(i,j,1) = 0.0f;
          edge_map(i,j,2) = 0.0f;
        }
        else if(edge_map(i,j,0)>0.0f)
          ++num_total;
      }
    }
    
    vcl_vector<vcl_vector<vgl_point_2d<double> > > curves = projector.contours();
    curves.insert(curves.end(),projector.parts().begin(),projector.parts().end());
    vil_image_view<float> dist_map = edge_dist_map(edge_map, curves);
    
    vil_image_view<vxl_byte> out_edges(ni,nj);
    for(unsigned int j=0; j<nj; ++j){
      for(unsigned int i=0; i<ni; ++i){
        if(dist_map(i,j) < dist_thesh){
          ++num_match;
          out_edges(i,j) = 0;
        }
        else
          out_edges(i,j) = 255;
      }
    }
    
    if(!poly_mesh_.has_half_edges())
      poly_mesh_.build_edge_graph();
    
    vcl_vector<vcl_vector<vgl_point_2d<double> > > poly_curves = 
       projector.project_all_edges(mv_optimizer_.camera(i),poly_mesh_,R_,t_);
    write_poly_curves("poly_curves.svg",poly_curves,ni,nj);
    vcl_cout << "num poly curves = "<< poly_curves.size()<<vcl_endl;
    vil_image_view<float> dist_map2 = edge_dist_map(edge_map, poly_curves);
    
    for(unsigned int j=0; j<nj; ++j){
      for(unsigned int i=0; i<ni; ++i){
        if(dist_map2(i,j) < dist_thesh){
          ++num_poly_match;
          out_edges(i,j) = 0;
        }
        else
          out_edges(i,j) = 255;
      }
    }
    
    
#if 0
    double min_strength = 0.0, max_strength = 10.0;
    double scale = 255.0 / (max_strength - min_strength);
    vil_image_view<vxl_byte> out_edges(ni,nj);
    for(unsigned int i=0; i<ni; ++i){
      for(unsigned int j=0; j<nj; ++j){
        double val = 255.0-(dist_map(i,j)-min_strength)*scale;
        if(val > 255.0) val = 255.0;
        if(val < 0.0) val = 0.0;
        out_edges(i,j) = static_cast<vxl_byte>(val);
      }
    }
#endif
    vil_save(out_edges,"vehicle_edge_dist.png");
    
  }
}

