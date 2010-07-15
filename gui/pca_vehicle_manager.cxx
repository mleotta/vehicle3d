// This is mleotta/gui/pca_vehicle/pca_vehicle_manager.cxx
//=========================================================================
//:
// \file
// \brief  Data manager for the PCA vehicle GUI
//
// See pca_vehicle_manager.h for details.
//=========================================================================

#include "pca_vehicle_manager.h"
#include "pca_vehicle_frame.h"


#include <vcl_limits.h>
#include <vcl_sstream.h>

#include <vgui/vgui.h>
#include <vgui/vgui_dialog.h>
#include <vgui/vgui_tableau_sptr.h>
#include <vgui/vgui_composite_tableau.h>
#include <vgui/vgui_easy2D_tableau.h>
#include <vgui/vgui_viewer2D_tableau.h>
#include <vgui/vgui_loader_tableau.h>
#include <vgui/vgui_image_tableau.h>
#include <vgui/vgui_range_map_params.h>
#include <vgui/vgui_shell_tableau.h>
#include <vgui/vgui_soview2D.h>
#include <vgui/vgui_projection_inspector.h>
#include <vgui/vgui_style.h>

#include <bgui/bgui_selector_tableau.h>
#include <bgui/bgui_image_tableau.h>
#include <dbpro/vis/dbpro_basic_gui_observers.h>

#include <vul/vul_timer.h>
#include <dbul/dbul_solar_position.h>

#include <vidl/vidl_istream.h>
#include <vidl/vidl_convert.h>
#include <vidl/gui/vidl_capture_tableau.h>
#include <vidl/vidl_ffmpeg_istream.h>


#include <vil/vil_new.h>
#include <vil/vil_load.h>
#include <vil/vil_save.h>
#include <vil/vil_convert.h>

#include <vnl/vnl_inverse.h>
#include <vnl/vnl_double_3.h>

#include <vgl/vgl_area.h>
#include <vgl/vgl_polygon.h>

#include <imesh/imesh_fileio.h>
#include <imesh/imesh_operations.h>
#include <imesh/imesh_detection.h>
#include <imesh/algo/imesh_project.h>
#include <imesh/algo/imesh_intersect.h>
#include <imesh/algo/imesh_operations.h>
#include <modrec/modrec_vehicle_parts.h>
#include <modrec/modrec_vehicle_mesh.h>
#include <modrec/modrec_vehicle_fit.h>
#include <modrec/modrec_edgel.h>


#include <Inventor/nodes/SoCoordinate3.h>
#include <Inventor/nodes/SoIndexedFaceSet.h>
#include <Inventor/nodes/SoMaterial.h>
#include <Inventor/nodes/SoDrawStyle.h>
#include <Inventor/nodes/SoShapeHints.h>
#include <Inventor/nodes/SoIndexedLineSet.h>

//: observer to clear vis_edgels_
class clear_vis_edgels_observer: public dbpro_observer
{
public:
  clear_vis_edgels_observer(vcl_vector<vcl_pair<double,vnl_double_4> >& vis_edgels)
  : vis_edgels_(vis_edgels) {}
  
  //: Called by the process when the data is ready
  virtual bool notify(const dbpro_storage_sptr& data, unsigned long timestamp)
  {
    vis_edgels_.clear();
    return true;
  }
  vcl_vector<vcl_pair<double,vnl_double_4> >& vis_edgels_;
};


//: observer to draw hypotheses
class hypothesis_observer: public dbpro_observer
{
public:
  hypothesis_observer(pca_vehicle_manager& m)
  : manager_(m) {}
  
  //: Called by the process when the data is ready
  virtual bool notify(const dbpro_storage_sptr& data, unsigned long timestamp)
  {
    assert(data);
    if(data->info() == DBPRO_VALID){
      assert(data->type_id() == typeid(vcl_vector<modrec_vehicle_state>));
      const vcl_vector<modrec_vehicle_state>& states = 
          data->data<vcl_vector<modrec_vehicle_state> >();
      
      for(unsigned int i=0; i<states.size(); ++i){
        manager_.draw_hypothesis(states[i]);
      }
    }
    return true;
  }
  pca_vehicle_manager& manager_;
};


//: observer to capture current vehicle tracking states
class track_observer: public dbpro_observer
{
public:
  track_observer(pca_vehicle_manager& m)
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
  pca_vehicle_manager& manager_;
};


//: observer to draw polygons
class polygon_observer: public dbpro_observer
{
public:
  polygon_observer(const vgui_easy2D_tableau_sptr& tab)
  : easy2D_tab_(tab) {}
  
  //: Called by the process when the data is ready
  virtual bool notify(const dbpro_storage_sptr& data, unsigned long timestamp)
  {
    assert(easy2D_tab_);
    assert(data);
    if(data->info() == DBPRO_VALID){
      assert(data->type_id() == typeid(vcl_vector<vgl_polygon<double> >));
      const vcl_vector<vgl_polygon<double> >& polys = 
      data->data<vcl_vector<vgl_polygon<double> > >();
      //easy2D_tab_->clear();
      easy2D_tab_->set_foreground(1.0f,0.0,0.0); //red 
      for(unsigned int i=0; i<polys.size(); ++i){
        const vcl_vector<vgl_point_2d<double> >& pts = polys[i][0];
        float* px = new float[pts.size()];
        float* py = new float[pts.size()];
        for(unsigned int j=0; j<pts.size(); ++j){
          px[j] = pts[j].x();
          py[j] = pts[j].y();
        }
        easy2D_tab_->add_polygon(pts.size(),px,py);
        delete [] px;
        delete [] py;
      }
    }
    return true;
  }
  vgui_easy2D_tableau_sptr easy2D_tab_;
};


//: observer to draw polygons
class point_observer: public dbpro_observer
{
public:
  point_observer(const vgui_easy2D_tableau_sptr& tab)
  : easy2D_tab_(tab) {}
  
  //: Called by the process when the data is ready
  virtual bool notify(const dbpro_storage_sptr& data, unsigned long timestamp)
  {
    assert(easy2D_tab_);
    assert(data);
    if(data->info() == DBPRO_VALID){
      assert(data->type_id() == typeid(vcl_vector<vgl_point_2d<double> >));
      const vcl_vector<vgl_point_2d<double> >& pts = 
         data->data<vcl_vector<vgl_point_2d<double> > >();
      //easy2D_tab_->clear();
      easy2D_tab_->set_foreground(0.0f,0.0,1.0); 
      easy2D_tab_->set_point_radius(2.0);
      for(unsigned int i=0; i<pts.size(); ++i){
        easy2D_tab_->add_point(pts[i].x(),pts[i].y());
      }
    }
    return true;
  }
  vgui_easy2D_tableau_sptr easy2D_tab_;
};


//: observer to draw optical flow vectors
class opt_flow_observer: public dbpro_observer
{
public:
  opt_flow_observer(const vgui_easy2D_tableau_sptr& tab)
  : easy2D_tab_(tab) {}
  
  //: Called by the process when the data is ready
  virtual bool notify(const dbpro_storage_sptr& data, unsigned long timestamp)
  {
    assert(easy2D_tab_);
    assert(data);
    typedef vcl_pair<vgl_point_2d<double>,vgl_vector_2d<double> > pv_pair;
    if(data->info() == DBPRO_VALID){
      assert(data->type_id() == typeid(vcl_vector<pv_pair>));
      const vcl_vector<pv_pair>& flow = data->data<vcl_vector<pv_pair> >();
      //easy2D_tab_->clear();
      easy2D_tab_->set_foreground(1.0f,0.0f,1.0); 
      for(unsigned int i=0; i<flow.size(); ++i){
        vgl_point_2d<double> pt2 = flow[i].first + flow[i].second;
        easy2D_tab_->add_line(flow[i].first.x(),flow[i].first.y(), pt2.x(),pt2.y());
      }
    }
    return true;
  }
  vgui_easy2D_tableau_sptr easy2D_tab_;
};


class scene_handler_tableau : public vgui_tableau
{
public:
  scene_handler_tableau(pca_vehicle_manager* m,
                        const vgui_tableau_sptr& num_p)
  : manager(m), child(this,num_p)
  {
  }

  //: handle events
  bool handle(const vgui_event& e)
  {
    if(manager->interaction_mode() == pca_vehicle_manager::MOVE)
    {
      static float x, y;
      if(e.type == vgui_MOUSE_MOTION)
      {
        vgui_projection_inspector().window_to_image_coordinates(e.wx, e.wy, x,y);
      }

      static double gx, gy;
      static double ox, oy;
      static bool drag = false;
      if(e.type == vgui_KEY_DOWN && e.key == vgui_ESC)
      {
        drag = false;
        manager->set_interaction_mode(pca_vehicle_manager::NONE);
      }
      else if(e.type == vgui_MOUSE_MOTION && drag)
      {
        vnl_double_3x3 tmp = manager->camera().get_matrix().extract(3,3);
        tmp.set_column(2,manager->camera().get_matrix().get_column(3));
        if(vnl_det(tmp) != 0.0)
        {
          vnl_double_3x3 img_to_gnd = vnl_inverse(tmp);
          vnl_double_3 gpos = img_to_gnd * vnl_double_3(x+ox,y+oy,1);
          gx = gpos[0]/gpos[2];
          gy = gpos[1]/gpos[2];
          manager->translation_.set(gx,gy,0);
          manager->proj2d_tab_->set_camera(manager->compute_relative_camera());
          manager->frame_->refresh_values();
          vgui::out << "mouse at "<<gx<<", "<<gy<<"\n";
        }
        this->post_redraw();
      }
      else if(e.type == vgui_BUTTON_DOWN && e.button == vgui_LEFT)
      {
        drag = true;
        const vgl_vector_3d<double>& t = manager->translation_;
        vnl_double_3 cpos = manager->camera().get_matrix() * vnl_vector_fixed<double,4>(t.x(),t.y(),t.z(),1);
        ox = cpos[0]/cpos[2] - x;
        oy = cpos[1]/cpos[2] - y;
        vgui::out << "offset at "<<ox<<", "<<oy<<"\n";
      }
      else if(e.type == vgui_BUTTON_UP && e.button == vgui_LEFT )
      {
        if(drag == true)
        {
          drag = false;
          manager->translation_.set(gx,gy,0);
          manager->proj2d_tab_->set_camera(manager->compute_relative_camera());
          manager->draw_parts();
          manager->frame_->refresh_values();
        }
        this->post_redraw();
      }
      else if(e.type == vgui_DRAW)
      {
        bool h = child->handle(e);
        manager->proj2d_tab_->handle(e);
        return h;
      }
    }
    else{
      float x, y;
      if(e.type == vgui_MOUSE_MOTION)
      {
        vgui_projection_inspector().window_to_image_coordinates(e.wx, e.wy, x,y);
        vgui::out << "("<<x<<", "<<y<<")\n";
      }
    }

    if(e.type == vgui_DRAW &&
       manager->show_edges_ )
    {
      if(manager->vis_edgels_.empty())
        manager->compute_vis_edgels();

      bool h = child->handle(e);
      
      double min_strength = manager->optimizer_->min_edge_strength();
      double max_strength = manager->optimizer_->max_edge_strength();
      double range = max_strength - min_strength;
      vgui_style::new_style(0.0,0.0,1.0,1.0,1.0)->apply_all();
      for(unsigned i=0; i<manager->vis_edgels_.size(); ++i)
      {
        double mag = manager->vis_edgels_[i].first;
        if(mag < min_strength) continue;
        mag -= min_strength;
        mag /= range;
        if(mag > 1.0) mag = 1.0;
        glColor4f(mag,mag,mag,1.0);
        const vnl_double_4& e = manager->vis_edgels_[i].second;
        glBegin(GL_LINES);
          glVertex2f(e[0]+e[3]/2,e[1]-e[2]/2);
          glVertex2f(e[0]-e[3]/2,e[1]+e[2]/2);
        glEnd();

      }
      return h;
    }
    return child->handle(e);
  }

  pca_vehicle_manager* manager;
  vgui_parent_child_link child;
};

typedef vgui_tableau_sptr_t<scene_handler_tableau> scene_handler_tableau_sptr;

struct scene_handler_tableau_new : public scene_handler_tableau_sptr
{
  typedef scene_handler_tableau_sptr base;
  scene_handler_tableau_new(pca_vehicle_manager* m, const vgui_tableau_sptr& num_p)
  : base(new scene_handler_tableau(m,num_p)) {}
};


//=============================================================================


//: Constructor
pca_vehicle_manager::pca_vehicle_manager(pca_vehicle_frame* frame)
  : frame_(frame), imode_(NONE), mesh_(NULL), frame_number_(0), 
    norm_cam_(false), show_edges_(false), draw_reprojected_(false)
{
  // default to multiview fitting mode
  optimizer_ = &mv_optimizer_;
}


//: Destructor
pca_vehicle_manager::~pca_vehicle_manager()
{
}

//: Initialize the 3D examiner view
//  returns the top level tableau to be added to a shell
vgui_tableau_sptr pca_vehicle_manager::init_3d_view()
{
  // generate an intial camera
  vgl_point_3d<double> c(5,5,5);
  vpgl_calibration_matrix<double> K(1200,vgl_point_2d<double>(512,386));
  vpgl_perspective_camera<double> cam(K,c,vgl_rotation_3d<double>());
  cam.look_at(vgl_homg_point_3d<double>(0,0,0.5));
  if(is_fit_mode_video())
    video_optimizer_.set_camera(cam);
  else
    mv_optimizer_.set_camera(frame_number_,cam);

  mesh_node_ = new SoSeparator;
  mesh_node_->ref();
  parts_node_ = new SoSeparator;
  mesh_node_->ref();
  SoSeparator* root = new SoSeparator;
  root->ref();
  mesh_xform_ = new SoTransform;
  mesh_xform_->ref();
  root->addChild(mesh_xform_);
  root->addChild(mesh_node_);
  root->addChild(parts_node_);

  exam_tab_ = bgui3d_examiner_tableau_new(root);
  root->unref();
  exam_capture_tab_ = vidl_capture_tableau_new(exam_tab_);

  SoSeparator* proot = new SoSeparator;
  proot->ref();
  SoMaterial* material = new SoMaterial;
  material->ambientColor.set1Value(0, SbColor(1, 0, 0));
  material->transparency.setValue(0.5);
  proot->addChild(material);
  SoShapeHints * hints = new SoShapeHints;
  proot->addChild( hints );
  hints->vertexOrdering = SoShapeHints::COUNTERCLOCKWISE;
  hints->shapeType = SoShapeHints::SOLID;

  proot->addChild(mesh_node_);
  proj2d_tab_ = bgui3d_project2d_tableau_new(cam,proot);
  proot->unref();

  init_mesh();

  return exam_capture_tab_;
}


//: Initialize the texture space view
//  returns the top level tableau to be added to a shell
vgui_tableau_sptr pca_vehicle_manager::init_tex_view()
{
  tex_tab_ = vgui_easy2D_tableau_new();
  tex_parts_tab_ = vgui_easy2D_tableau_new();

  vgui_loader_tableau_new tex_viewer(
      vgui_composite_tableau_new(tex_tab_,tex_parts_tab_));
  //tex_viewer->zoomin(300,0,0);
  //tex_viewer->center_image(0,0);
  tex_viewer->set_ortho(0,0,1,1);

  // Draw the unit square as an initial boundary
  tex_tab_->set_line_width(1.0f);
  tex_tab_->set_foreground(0.5f,0.5f,0.5f);
  float x[4],y[4];
  x[0] = 0.0f; y[0] = 0.0f;
  x[1] = 1.0f; y[1] = 0.0f;
  x[2] = 1.0f; y[2] = 1.0f;
  x[3] = 0.0f; y[3] = 1.0f;
  tex_tab_->add_polygon(4,x,y);

  return tex_viewer;
}


//: Initialize the image projection view
//  returns the top level tableau to be added to a shell
vgui_tableau_sptr pca_vehicle_manager::init_proj_view()
{
  proj_tab_ = vgui_easy2D_tableau_new();
  image_tab_ = bgui_image_tableau_new();
  edge_map_tab_ = vgui_image_tableau_new();
  debug_image_tab_ = vgui_image_tableau_new();
  depth_image_tab_ = vgui_image_tableau_new();
  gnd_cal_tab_ = dbgui_gnd_cal_tableau_new(camera());
  
  detect_tab_ = vgui_easy2D_tableau_new();


  selector_tab_ = bgui_selector_tableau_new();
  selector_tab_->add(depth_image_tab_,"Depth Map");
  selector_tab_->add(debug_image_tab_,"Debug");
  selector_tab_->add(edge_map_tab_,"Edge Image");
  selector_tab_->add(image_tab_,"Image");
  selector_tab_->add(proj_tab_,"Vehicle Edges");
  selector_tab_->add(detect_tab_,"Vehicle Detection");
  selector_tab_->add(gnd_cal_tab_,"Calibration");
  
  video_optimizer_.tracker().add_video_observer(new dbpro_image_observer(image_tab_));
  //video_optimizer_.tracker().add_bg_observer(new dbpro_image_observer(debug_image_tab_));
  video_optimizer_.tracker().add_edgemap_observer(new clear_vis_edgels_observer(vis_edgels_));
  video_optimizer_.tracker().add_edgemap_observer(new dbpro_image_observer(edge_map_tab_));
  video_optimizer_.tracker().add_silhouette_observer(new polygon_observer(detect_tab_));
  video_optimizer_.tracker().add_hypotheses_observer(new hypothesis_observer(*this));
  video_optimizer_.tracker().add_track_observer(new track_observer(*this));
  video_optimizer_.tracker().add_optical_flow_observer(new opt_flow_observer(detect_tab_));
  
  video_optimizer_.tracker().enable_display(false);
  
  //video_optimizer_.tracker().add_pointmap_observer(new dbpro_image_observer(debug_image_tab_));
  //video_optimizer_.tracker().add_point_observer(new point_observer(detect_tab_));

  scene_handler_tableau_new handler(this,selector_tab_);
  
  proj_capture_tab_ = vidl_capture_tableau_new(
                          vgui_viewer2D_tableau_new(handler));

  return proj_capture_tab_;
}


//: Enable video mode (true), or multiview mode (false)
void pca_vehicle_manager::set_fit_mode(bool use_video)
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
bool pca_vehicle_manager::is_fit_mode_video() const
{
  return optimizer_ == &video_optimizer_;
}


//: Initialize the texture map for detailed meshes
void pca_vehicle_manager::init_mesh_tex(modrec_pca_vehicle& mesh)
{
  if(mesh.has_tex_coords())
  {
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
  else{
    vcl_cerr << "No texture coordinates"<<vcl_endl;
  }
}


//: Build a default vehicle mesh from default parameters in modrec
void pca_vehicle_manager::init_mesh()
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
  if(mesh_->has_tex_coords())
    draw_texmap();
  build_mesh_node();
  exam_tab_->post_redraw();
}


//: Choose the type of vehicle model from the enum
void pca_vehicle_manager::set_vehicle_model(vehicle_model vm)
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
  build_mesh_node();  
  draw_texmap_parts();
  build_parts_node();
  draw_texmap();
  draw_parts();
  exam_tab_->post_redraw();
}


//: load an image 
bool pca_vehicle_manager::load_image(const vcl_string& filename)
{
  // don't load a single frame in video mode
  if(is_fit_mode_video())
    return false;
  
  img_ = vil_load_image_resource(filename.c_str());
  mesh_projector_.resize(img_->ni(), img_->nj());
  if(!img_)
    return false;
  image_tab_->set_image_resource(img_);
  image_tab_->post_redraw();
  
  if(norm_cam_)
  {
    if(img_ && img_->ni()>0 && img_->nj()>0){
      vpgl_perspective_camera<double> camera = this->camera();
      vpgl_calibration_matrix<double> K = norm_K_;
      K.set_x_scale(K.x_scale()*img_->ni());
      K.set_y_scale(K.y_scale()*img_->nj());
      vgl_point_2d<double> pp = K.principal_point();
      pp.x() *= img_->ni();
      pp.y() *= img_->nj();
      K.set_principal_point(pp);
      camera.set_calibration(K);
      this->set_camera(camera);
      vcl_cout << K.get_matrix() << vcl_endl;
    }
  }

  // detect edges in this image
  vil_image_view<vxl_byte> img_grey = 
      vil_convert_to_grey_using_rgb_weighting(img_->get_view());
  mv_optimizer_.detect_and_set_edges(0,img_grey);
  edge_map_tab_->set_image_view(mv_optimizer_.edge_map(0));
  
  
  // clear edges for visualization
  vis_edgels_.clear();
  
  return true;
}


//: load a camera matrix
bool pca_vehicle_manager::load_camera(const vcl_string& filename)
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
    if(istream_ && istream_->is_valid())
    {
      ni = istream_->width();
      nj = istream_->height();
    }
    else if(img_)
    {
      ni = img_->ni();
      nj = img_->nj();
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
  
  this->set_camera(camera);

  return true;
}


//: Save the camera to a file
bool pca_vehicle_manager::save_camera(const vcl_string& filename, bool normalize) const
{
  vcl_ofstream ofs(filename.c_str());
  if(!ofs.is_open())
    return false;
  if(normalize)
  {    
    unsigned int ni=0, nj=0;
    if(istream_ && istream_->is_valid())
    {
      ni = istream_->width();
      nj = istream_->height();
    }
    else if(img_)
    {
      ni = img_->ni();
      nj = img_->nj();
    }
    if(ni>0 && nj>0){
      vpgl_calibration_matrix<double> K = camera().get_calibration();
      K.set_x_scale(K.x_scale()/ni);
      K.set_y_scale(K.y_scale()/nj);
      vgl_point_2d<double> pp = K.principal_point();
      pp.x() /= ni;
      pp.y() /= nj;
      K.set_principal_point(pp);
      vpgl_perspective_camera<double> camera = this->camera();
      camera.set_calibration(K);
      ofs << camera.get_matrix();
      ofs << "normalized\n";
    }
    else
      return false;
  }
  else
    ofs << camera().get_matrix();
  ofs.close();

  return true;
}


//: Save a file with the current 3d coordinates of parts
bool pca_vehicle_manager::save_3d_parts(const vcl_string& filename) const
{
  vcl_ofstream ofs(filename.c_str());
  if(!ofs.is_open())
    return false;
  typedef vcl_map<vcl_string, vgl_polygon<double> > part_map;
  const vcl_vector<vcl_vector<vgl_point_3d<double> > >& parts_3d = mesh_->parts_3d();
  const part_map& parts = mesh_->parts();
  
  unsigned int i=0;
  for(part_map::const_iterator itr=parts.begin(); itr!=parts.end(); ++itr, ++i)
  {
    ofs << itr->first <<"\n";
    for(unsigned int j=0; j<parts_3d[i].size(); ++j)
    {
      const vgl_point_3d<double>& pt = parts_3d[i][j];
      ofs << pt.x() <<","<<pt.y()<<","<<pt.z()<<" ";
    }
    ofs << "\n";
  }
  ofs.close();
  return true;
}


//: Save the projected contours as SVG
bool pca_vehicle_manager::save_svg(const vcl_string& filename) const
{
  return modrec_write_svg_curves(filename, mesh_projector_);
}


//: Save an image of the detected edges
bool pca_vehicle_manager::save_edge_image(const vcl_string& filename) const
{
  const vil_image_view<float>& edge_map = is_fit_mode_video() ?
                                          video_optimizer_.edge_map() : 
                                          mv_optimizer_.edge_map(frame_number_);
  if(!edge_map)
    return false;
  const unsigned int ni = edge_map.ni(), nj = edge_map.nj();
  
  double min_strength = 5.0, max_strength = 30.0;
  double scale = 255.0 / (max_strength - min_strength);
  vil_image_view<vxl_byte> out_edges(ni,nj);
  for(unsigned int i=0; i<ni; ++i){
    for(unsigned int j=0; j<nj; ++j){
      double val = 255.0-(edge_map(i,j,0)-min_strength)*scale;
      if(val > 255.0) val = 255.0;
      if(val < 0.0) val = 0.0;
      out_edges(i,j) = static_cast<vxl_byte>(val);
    }
  }
  
  return vil_save(out_edges,filename.c_str());
}

//: Save an image of the current video frame
bool pca_vehicle_manager::save_video_frame(const vcl_string& filename) const
{
  if(!istream_)
    return false;
  vidl_frame_sptr frame = istream_->current_frame();
  if(!frame)
    return false;
  
  static vil_image_view<vxl_byte> img;
  if (vidl_convert_to_view(*frame,img,VIDL_PIXEL_COLOR_RGB))
    return vil_save(img,filename.c_str());
  
  return false;
}


//: set the camera matrix
void pca_vehicle_manager::set_camera(const vpgl_perspective_camera<double>& cam)
{
  if(is_fit_mode_video())
    video_optimizer_.set_camera(cam);
  else
    mv_optimizer_.set_camera(frame_number_,cam);
  
  vpgl_perspective_camera<double> camera = compute_relative_camera();
  exam_tab_->set_camera(camera);
  proj2d_tab_->set_camera(camera);
  gnd_cal_tab_->set_camera(cam);
  draw_parts();
}


//: compute the sun direction for shadow casting
void pca_vehicle_manager::compute_sun_direction()
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
      vcl_cout << "rel time = "<<rel_time<<vcl_endl;
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


//: load a mesh
bool pca_vehicle_manager::load_mesh(const vcl_string& meshfile,
                                    const vcl_string& partsfile)
{
  imesh_mesh new_mesh;
  if(!imesh_read(meshfile,new_mesh)){
    return false;
  }
  
  if(mesh_->num_verts() != new_mesh.num_verts())
    return false;
  
  vcl_auto_ptr<imesh_vertex_array_base> new_verts(new_mesh.vertices().clone());
  mesh_->set_vertices(new_verts);

  mesh_->compute_face_normals();

  if(mesh_->has_tex_coords())
  {
    //mesh_->label_ccw_tex_faces_valid();
    draw_texmap();
  }
  else{
    vcl_cerr << "No texture coordinates"<<vcl_endl;
  }

  if(!mesh_->params().empty())
  {
    if(partsfile == "")
      mesh_->set_params(mesh_->imesh_pca_mesh::project(mesh_->vertices()));
    else
      mesh_->set_params(mesh_->project(mesh_->vertices(),
                                       modrec_read_vehicle_parts(partsfile)));
    draw_parts();
  }
  video_optimizer_.set_vehicle_model(*mesh_);
  build_mesh_node();
  exam_tab_->post_redraw();
  return true;
}


//: Load the ground truth mesh file
bool pca_vehicle_manager::load_truth_mesh(const vcl_string& filename)
{
  if(!imesh_read(filename,truth_mesh_)){
    return false;
  }
  
  imesh_triangulate(truth_mesh_);

  return true;
}


//: Vehicle surface parts
bool pca_vehicle_manager::load_parts(const vcl_string& filename)
{
  mesh_->set_parts(modrec_read_vehicle_parts(filename));
  video_optimizer_.set_vehicle_model(*mesh_);
  draw_parts();

  return true;
}


//: Vehicle PCA File
bool pca_vehicle_manager::load_pca(const vcl_string& filename)
{
  vnl_vector<double> mean;
  vnl_vector<double> std_devs;
  vnl_matrix<double> pc;
  if(!imesh_read_pca(filename,mean,std_devs,pc))
    return false;

  mesh_->init(mean,std_devs,pc);
  video_optimizer_.set_vehicle_model(*mesh_);

  return true;
}


//: Open input video stream
bool pca_vehicle_manager::open_istream(const vidl_istream_sptr& istream)
{
  istream_ = istream;
  video_optimizer_.set_istream(istream);

  if (!istream_)
    return false;
  
  optimizer_->reset();

  istream_->advance();
  if (istream_->is_valid())
  {
    vidl_frame_sptr frame = istream_->current_frame();
    if (frame) {
      static vil_image_view<vxl_byte> img;
      if (vidl_convert_to_view(*frame,img,VIDL_PIXEL_COLOR_RGB))
        image_tab_->set_image_view(img);
      else{
        image_tab_->set_image_resource(NULL);
        return false;
      }
      mesh_projector_.resize(img.ni(), img.nj());
      image_tab_->post_redraw();
      
      // detect edges in this frame
      vil_image_view<vxl_byte> img_grey;
      if(is_fit_mode_video()){
        if (vidl_convert_to_view(*frame,img_grey,VIDL_PIXEL_COLOR_MONO))
          video_optimizer_.detect_and_set_edges(img_grey);
        edge_map_tab_->set_image_view(video_optimizer_.edge_map());
      }
      else{
        if (vidl_convert_to_view(*frame,img_grey,VIDL_PIXEL_COLOR_MONO))
          mv_optimizer_.detect_and_set_edges(0,img_grey);
        edge_map_tab_->set_image_view(mv_optimizer_.edge_map(0));
      }
      
      // clear the edges for visualization
      vis_edgels_.clear();
      
      frame_number_ = 0;
      return true;
    }
  }

  return false;
}


//: Attach observers
void pca_vehicle_manager::attach(vgui_observer *o)
{
  if(gnd_cal_tab_)
    gnd_cal_tab_->attach(o);
  else
    vcl_cout << "ground cal not initialized"<<vcl_endl;
}


//: return the number of frames in the current video if seekable
int pca_vehicle_manager::num_video_frames() const
{
  if(!istream_)
    return -1;

  return istream_->num_frames();
}

//: return the current video frame number
int pca_vehicle_manager::current_frame() const
{
  if(!istream_)
    return -1;
  
  return istream_->frame_number();
}


//: Either enable or disable fitting on the current frame
void pca_vehicle_manager::set_frame_active(bool active)
{
  if(!is_fit_mode_video())
    mv_optimizer_.set_active(frame_number_,active);
}


//: seek the video to the frame number
void pca_vehicle_manager::video_seek(int fnum)
{
  if(istream_)
    istream_->seek_frame(fnum);
  
  compute_sun_direction();

  frame_number_ = fnum;
  vgui::out << "seek to "<< fnum << "\n";


  vidl_frame_sptr frame = istream_->current_frame();
  if (frame) {
    static vil_image_view<vxl_byte> img;
    if (vidl_convert_to_view(*frame,img,VIDL_PIXEL_COLOR_RGB)){
      image_tab_->set_image_view(img);
      image_tab_->reread_image();
    }
    else{
      image_tab_->set_image_resource(NULL);
      return;
    }

    image_tab_->post_redraw();
    
    if(is_fit_mode_video()){
      // detect the edges on this image
      vil_image_view<vxl_byte> img_grey;
      if (vidl_convert_to_view(*frame,img_grey,VIDL_PIXEL_COLOR_MONO))
        video_optimizer_.detect_and_set_edges(img_grey);
      
      edge_map_tab_->set_image_view(video_optimizer_.edge_map());
      proj_tab_->clear();
    }
    else{
      // detect the edges on this image if this is the first time view the frame
      if(fnum >= mv_optimizer_.num_views() || mv_optimizer_.edge_map(fnum)==0){
        vil_image_view<vxl_byte> img_grey;
        if (vidl_convert_to_view(*frame,img_grey,VIDL_PIXEL_COLOR_MONO))
          mv_optimizer_.detect_and_set_edges(fnum,img_grey);
      }
      
      edge_map_tab_->set_image_view(mv_optimizer_.edge_map(fnum));
    }
    
    // clear visualization edges
    vis_edgels_.clear();
  }
  
  vpgl_perspective_camera<double> cam = compute_relative_camera();
  exam_tab_->set_camera(cam);
  proj2d_tab_->set_camera(cam);
  gnd_cal_tab_->set_camera(camera());
  
  if(state_map_.find(fnum) != state_map_.end())
    draw_current_states();
  else
    draw_parts();
}


//: advance the video to the next frame
bool pca_vehicle_manager::advance_video()
{
  detect_tab_->clear();
  
  if(replay_mode_){
    proj_tab_->clear();
    if(!istream_->advance())
      return false;
    compute_sun_direction();
    vidl_frame_sptr frame = istream_->current_frame();
    if (frame) {
      static vil_image_view<vxl_byte> img;
      if (vidl_convert_to_view(*frame,img,VIDL_PIXEL_COLOR_RGB)){
        image_tab_->set_image_view(img);
        image_tab_->reread_image();
      }
      else{
        image_tab_->set_image_resource(NULL);
        return false;
      }
      
      image_tab_->post_redraw();
      exam_tab_->post_redraw();
      
      // clear visualization edges
      vis_edgels_.clear();
    }
    draw_current_states();
    return true;
  }
  
  dbpro_signal s = video_optimizer_.process_once();
  compute_sun_direction();
  if(s == DBPRO_VALID )
    return true;
  return false;
}


//: scale the camera units
void pca_vehicle_manager::scale_camera(double scale)
{
  vpgl_perspective_camera<double> cam = camera();
  vgl_point_3d<double> num_p = cam.get_camera_center();
  num_p.set(num_p.x()*scale, num_p.y()*scale, num_p.z()*scale);
  cam.set_camera_center(num_p);
  if(is_fit_mode_video())
    video_optimizer_.set_camera(cam);
  else
    mv_optimizer_.set_camera(frame_number_,cam);
  

  exam_tab_->set_camera(cam);
  proj2d_tab_->set_camera(cam);
  draw_parts();
}


//: Access the camera
const vpgl_perspective_camera<double>& pca_vehicle_manager::camera() const 
{ 
  if(is_fit_mode_video())
    return video_optimizer_.camera();
  
  return mv_optimizer_.camera(frame_number_); 
}


//: compute the RMS error between the PCA mesh and Ground truth
void pca_vehicle_manager::compute_error()
{
  imesh_mesh body = imesh_submesh_from_faces(*mesh_,mesh_->faces().group_face_set("body"));
  const imesh_vertex_array<3>& verts = body.vertices<3>();
  
  vgl_point_3d<double> cp;
  double error = 0.0;
  for(unsigned int i=0; i<verts.size(); ++i)
  {
    vgl_point_3d<double> p = verts[i];
    imesh_closest_point(verts[i], truth_mesh_, cp);
    error += (p-cp).sqr_length();
  }
  error /= verts.size();
  
  vcl_cout << "RMS error = " << vcl_sqrt(error) << vcl_endl;
}


//: draw the texture map
void pca_vehicle_manager::draw_texmap()
{
  tex_tab_->clear();
  const vcl_vector<vgl_point_2d<double> >& tc = mesh_->tex_coords();
  if(mesh_->has_tex_coords() == imesh_mesh::TEX_COORD_ON_VERT)
  {
    tex_tab_->set_line_width(1.0);
    tex_tab_->set_foreground(0.5f,0.5f,0.5f);
    const imesh_face_array_base& faces = mesh_->faces();
    for(unsigned int i=0; i<faces.size(); ++i)
    {
      const unsigned int num_verts = faces.num_verts(i);
      float x[num_verts], y[num_verts];
      vgl_polygon<float> poly(1);
      for(unsigned int j=0; j<num_verts; ++j)
      {
        x[j] = tc[faces(i,j)].x();
        y[j] = tc[faces(i,j)].y();
        poly.push_back(x[j],y[j]);
      }
      if(vgl_area_signed(poly) > 0.0)
        tex_tab_->add_polygon(num_verts,x,y);
    }
    tex_tab_->post_redraw();
  }
  else if(mesh_->has_tex_coords() == imesh_mesh::TEX_COORD_ON_CORNER){
    vcl_cerr << "Texutre coordinates per corner not supported"<<vcl_endl;
  }
}


//: Build the Inventor nodes for the 3D mesh display
void pca_vehicle_manager::build_mesh_node()
{
  mesh_node_->removeAllChildren();
  
  // make a copy of the current mesh and triangulate
  imesh_mesh tri_mesh = *mesh_;
  imesh_triangulate_nonconvex(tri_mesh);

  // coordset
  SoCoordinate3 * coords = new SoCoordinate3;
  mesh_node_->addChild( coords );
  typedef imesh_vertex_array<3>::const_iterator vitr;
  const imesh_vertex_array<3>& verts3d = tri_mesh.vertices<3>();
  unsigned int idx = 0;
  for(vitr v = verts3d.begin(); v!=verts3d.end(); ++v)
  {
    coords->point.set1Value(idx++, SbVec3f((*v)[0], (*v)[1], (*v)[2]));
  }


  // indexed face set
  SoIndexedFaceSet * ifs = new SoIndexedFaceSet;
  mesh_node_->addChild( ifs );
  const imesh_face_array_base& faces = tri_mesh.faces();
  idx = 0;
  for(unsigned int f=0; f<faces.size(); ++f)
  {
    for(unsigned int i=0; i<faces.num_verts(f); ++i)
      ifs->coordIndex.set1Value(idx++, faces(f,i));
    ifs->coordIndex.set1Value(idx++, -1);
  }

  exam_tab_->view_all();
}


//: Update the Inventor nodes for the 3D mesh display
void pca_vehicle_manager::update_mesh_node()
{
  SoCoordinate3 * coords = dynamic_cast<SoCoordinate3 *>(mesh_node_->getChild(0));

  if(!coords){
    vcl_cerr << "First child is not a 3D coordinate node"<<vcl_endl;
    return;
  }

  // coordset
  typedef imesh_vertex_array<3>::const_iterator vitr;
  const imesh_vertex_array<3>& verts3d = mesh_->vertices<3>();
  unsigned int idx = 0;
  for(vitr v = verts3d.begin(); v!=verts3d.end(); ++v)
  {
    coords->point.set1Value(idx++, SbVec3f((*v)[0], (*v)[1], (*v)[2]));
  }

  //exam_tab_->view_all();
}


//: draw the vehicle parts in the texture map space
void pca_vehicle_manager::draw_texmap_parts()
{
  // draw parts in the texture view
  tex_parts_tab_->clear();
  tex_parts_tab_->set_line_width(2.0);
  tex_parts_tab_->set_foreground(1.0f,0.0f,0.0f);
  const part_map& parts = mesh_->parts();
  vcl_vector<vgl_point_2d<double> > all_points;
  for(part_map::const_iterator itr=parts.begin(); itr!=parts.end(); ++itr)
  {
    const vcl_vector<vgl_point_2d<double> >& poly = itr->second[0];
    float x[poly.size()], y[poly.size()];
    for(unsigned int i=0; i<poly.size(); ++i)
    {
      x[i] = poly[i].x();
      y[i] = poly[i].y();
      all_points.push_back(poly[i]);
    }
    tex_parts_tab_->add_polygon(poly.size(),x,y);
  }
  tex_parts_tab_->set_point_radius(3.0);
  tex_parts_tab_->set_foreground(0.0f,1.0f,0.0f);
  
  typedef modrec_pca_vehicle::uv_point uv_point;
  const vcl_vector<vcl_vector<uv_point> >& parts_uv = mesh_->parts_bary();
  for(unsigned int i=0; i<parts_uv.size(); ++i)
  {
    for(unsigned int j=0; j<parts_uv[i].size(); ++j)
    {
      double t2 = parts_uv[i][j].t;
      double t1 = 1-t2; 
      const vgl_point_2d<double>& p1 = all_points[parts_uv[i][j].end_point1]; 
      const vgl_point_2d<double>& p2 = all_points[parts_uv[i][j].end_point2]; 
      //unsigned int fidx = mesh_.half_edges()[parts_uv[i][j].mesh_index>>2].face_index();
      //vgl_point_2d<double> pt = 
      //    imesh_project_barycentric_to_texture(mesh_,parts_uv[i][j].uv,fidx);
      tex_parts_tab_->add_point(p1.x()*t1+p2.x()*t2, p1.y()*t1+p2.y()*t2);
    }
  }
  tex_parts_tab_->post_redraw();
}


//: draw the vehicle parts
void pca_vehicle_manager::draw_parts()
{
  draw_texmap_parts();

  if(draw_reprojected_)
    mesh_projector_.reproject(camera(),*mesh_,rotation_,translation_,sun_dir_);
  else
    mesh_projector_.project(camera(),*mesh_,rotation_,translation_,sun_dir_);

  build_parts_node();

  if(mesh_projector_.depth_map())
  {
    depth_image_tab_->set_image_view(mesh_projector_.depth_map(),
        new vgui_range_map_params(mesh_projector_.min_depth(),
                                  mesh_projector_.max_depth(),1.0,true));
    depth_image_tab_->post_redraw();
  }
  
  proj_tab_->clear();
  proj_tab_->set_line_width(2.0);
  
#if 0
  vcl_vector<vcl_vector<vgl_point_2d<double> > > shadow_pts;
  const vgl_polygon<double>& shadow = mesh_projector_.shadow();
  for(unsigned int i=0; i<shadow.num_sheets(); ++i){
    shadow_pts.push_back(shadow[i]);
    shadow_pts.back().push_back(shadow[i].front());
  }
  proj_tab_->set_foreground(0.0f,1.0f,1.0f);
  draw_curves(proj_tab_,shadow_pts);
#endif


  proj_tab_->set_foreground(1.0f,0.0f,0.0f);
  draw_curves(proj_tab_,mesh_projector_.parts());
  proj_tab_->set_foreground(0.0f,1.0f,0.0f);
  draw_curves(proj_tab_,mesh_projector_.contours());
  //proj_tab_->set_foreground(0.0f,0.0f,1.0f);
  //draw_curves(proj_tab_,mesh_projector_.silhouette());

  exam_tab_->post_redraw();
}


//: draw a hypothesized vehicle shape and pose
void pca_vehicle_manager::draw_hypothesis(const modrec_vehicle_state& state)
{
  if(!selector_tab_->is_visible("Vehicle Detection"))
    return;
  
  const vnl_vector<double>& p = state.params;
  rotation_ = state.rotation;
  translation_ = state.translation;
  
  if(is_fit_mode_video())
    video_optimizer_.set_silhouette(state.silhouette);
  
  vnl_vector<double> old_p = mesh_->params();
  mesh_->set_params(p);
  mesh_->compute_face_normals();
  mesh_projector_.project(camera(),*mesh_,rotation_,translation_,sun_dir_);
  
  detect_tab_->set_line_width(2.0);
  detect_tab_->set_foreground(1.0f,0.0f,0.0f);
  draw_curves(detect_tab_,mesh_projector_.parts());
  detect_tab_->set_foreground(0.0f,1.0f,0.0f);
  draw_curves(detect_tab_,mesh_projector_.contours());
  detect_tab_->set_foreground(0.0f,0.0f,1.0f);
  draw_curves(detect_tab_,mesh_projector_.silhouette());
  detect_tab_->post_redraw();
  
  //mesh_.set_params(old_p);
  //mesh_.compute_face_normals();
  proj2d_tab_->set_camera(compute_relative_camera());
  proj_tab_->post_redraw();
}


//: draw the current vehicle tracking states
void pca_vehicle_manager::draw_current_states()
{
  typedef vcl_map<unsigned int,vcl_vector<modrec_vehicle_state> >::const_iterator map_itr;
  map_itr fitr = state_map_.find(current_frame());
  if(fitr == state_map_.end() || fitr->second.empty())
    return;
  const vcl_vector<modrec_vehicle_state>& states = fitr->second;
  
  proj_tab_->clear();
  proj_tab_->set_line_width(2.0);
  
  for(int i=states.size()-1; i>=0; --i)
  {
    rotation_ = states[i].rotation;
    translation_ = states[i].translation;
    mesh_->set_params(states[i].params);
    mesh_->compute_face_normals();
    mesh_projector_.project(camera(),*mesh_,rotation_,translation_,sun_dir_);
    
    proj_tab_->set_foreground(1.0f,0.0f,0.0f);
    draw_curves(proj_tab_,mesh_projector_.parts());
    proj_tab_->set_foreground(0.0f,1.0f,0.0f);
    draw_curves(proj_tab_,mesh_projector_.contours());
    proj_tab_->set_foreground(0.0f,0.0f,1.0f);
    draw_curves(proj_tab_,mesh_projector_.silhouette());
  }
  frame_->refresh_values();
  
  proj2d_tab_->set_camera(compute_relative_camera());
  update_mesh_node();  
  draw_texmap_parts();
  build_parts_node();
  proj_tab_->post_redraw();
  exam_tab_->post_redraw();
}


//: set the vehicle tracking states
void pca_vehicle_manager::set_tracking_states(const vcl_vector<modrec_vehicle_state>& states)
{
  if(states.empty())
    return;
  state_map_[current_frame()] = states;
  draw_current_states();
}


//: set the history of vehicle tracking states indexed over frame number
void pca_vehicle_manager::set_state_map(const vcl_map<unsigned int,vcl_vector<modrec_vehicle_state> >& state_map)
{
  state_map_ = state_map;
}


//: construct the Inventor node containing the parts as 3d curves
void pca_vehicle_manager::build_parts_node()
{
  assert(mesh_->faces().has_normals());
  assert(mesh_->has_half_edges());
  parts_node_->removeAllChildren();
  
  typedef modrec_pca_vehicle::uv_point uv_point;
  const vcl_vector<vcl_vector<uv_point> >& parts_uv = mesh_->parts_bary();
  const vcl_vector<vcl_vector<vgl_point_3d<double> > >& parts3d = mesh_->parts_3d();
  const vcl_vector<vgl_vector_3d<double> >& normals = mesh_->faces().normals();
  const imesh_half_edge_set& he = mesh_->half_edges();

  // material
  SoMaterial* material = new SoMaterial;
  parts_node_->addChild(material);
  material->diffuseColor.setValue(SbColor(1.0f, 0.0f, 0.0f));

  // coordset
  SoCoordinate3 * coords = new SoCoordinate3;
  parts_node_->addChild( coords );
  
  SoDrawStyle * drawstyle = new SoDrawStyle;
  parts_node_->addChild( drawstyle );
  drawstyle->lineWidth.setValue(2.0);

  // indexed face set
  SoIndexedLineSet * ils = new SoIndexedLineSet;
  parts_node_->addChild( ils );

  unsigned int cidx = 0, iidx=0;
  for(unsigned int i=0; i<parts3d.size(); ++i)
  {
    unsigned int fidx = cidx;
    const vcl_vector<vgl_point_3d<double> >& part = parts3d[i];
    for(unsigned int j=0; j<part.size(); ++j)
    {
      // offset slightly in the normal direction to prevent clipping
      vgl_vector_3d<double> n;
      unsigned long index = parts_uv[i][j].mesh_index;
      if(index&3 == 0)
        n = normals[he[index>>2].face_index()];
      else if (index&3 == 1){
        n = normals[he[index>>2].face_index()];
        n += normals[he[(index>>2)^1].face_index()];
        normalize(n);
      }else{
        imesh_half_edge_set::v_const_iterator vitr(index>>2,he);
        imesh_half_edge_set::v_const_iterator vend = vitr;
        n = normals[vitr->face_index()];
        for(++vitr; vitr != vend; ++vitr)
          n += normals[vitr->face_index()];
        normalize(n);
      }
      vgl_point_3d<double> pt = part[j] + 0.01*n;
      ils->coordIndex.set1Value(iidx++, cidx);
      coords->point.set1Value(cidx++, SbVec3f(pt.x(),pt.y(),pt.z()));
    }
    ils->coordIndex.set1Value(iidx++, fidx);
    ils->coordIndex.set1Value(iidx++, -1);
  }

}


//: Compute a camera that incorporates the vehicle rotation and translation
vpgl_perspective_camera<double>
pca_vehicle_manager::compute_relative_camera() const
{
  vpgl_perspective_camera<double> rcam(camera());
  rcam.set_rotation(rcam.get_rotation()*rotation_);
  vgl_point_3d<double> num_p = rotation_.inverse()*(rcam.get_camera_center()-translation_);
  rcam.set_camera_center(num_p);

  return rcam;
}


//: set the camera matrix relative to the vehicle
//  incorporates the vehicle rotation and translation into the camera
//  and the resets the vehicle rotation and translation
void pca_vehicle_manager::set_camera_relative()
{
  if(is_fit_mode_video())
    video_optimizer_.set_camera(compute_relative_camera());
  else
    mv_optimizer_.set_camera(frame_number_,compute_relative_camera());
  rotation_ = vgl_rotation_3d<double>();
  translation_.set(0.0,0.0,0.0);
  gnd_cal_tab_->set_camera(camera());
  frame_->refresh_values();
}


//: draw curves in the image view
void pca_vehicle_manager::
draw_curves(vgui_easy2D_tableau_sptr tab,
            const vcl_vector<vcl_vector<vgl_point_2d<double> > >& curves)
{
  for(unsigned int i=0; i<curves.size(); ++i)
  {
    const vcl_vector<vgl_point_2d<double> >& curve = curves[i];
    float x[curve.size()], y[curve.size()];
    for(unsigned int j=0; j<curve.size(); ++j)
    {
      const vgl_point_2d<double>& pt = curve[j];
      x[j] = pt.x();
      y[j] = pt.y();
    }
    tab->add_linestrip(curve.size(),x,y);
  }
}


//: update all of the mesh parameters at once
void pca_vehicle_manager::change_mesh_params(const vcl_vector<double>& params)
{
  vnl_vector<double> p = mesh_->params();
  for(unsigned int i=0; i<params.size() && i<p.size(); ++i)
    p[i] = params[i];
  mesh_->set_params(p);

  mesh_->compute_face_normals();

  update_mesh_node();  
  draw_texmap_parts();
  build_parts_node();
  draw_parts();
  exam_tab_->post_redraw();
}


//: update a single mesh parameters
void pca_vehicle_manager::update_mesh_param(unsigned int i, double param)
{
  mesh_->set_param(i,param);
  update_mesh_node();
  draw_texmap_parts();
  build_parts_node();
  exam_tab_->post_redraw();
}


//: Set the interaction mode
void pca_vehicle_manager::set_interaction_mode(const interact_mode& mode)
{
  imode_ = mode;
  proj_tab_->post_redraw();
  frame_->interaction_mode_changed();
}


//: Set the state of edge visibility
void pca_vehicle_manager::show_edges(bool show)
{
  show_edges_ = show;
  proj_tab_->post_redraw();
}


//: Set the state of edge visibility
void pca_vehicle_manager::show_axes(bool show)
{
  exam_tab_->set_axis_visible(show);
  exam_tab_->post_redraw();
}

//: Set the state of draw_reprojected
void pca_vehicle_manager::draw_reprojected(bool val)
{
  draw_reprojected_ = val;
  proj_tab_->post_redraw();
}

//: Enable tracking if in video mode
void pca_vehicle_manager::enable_tracking(bool enable_track)
{
  video_optimizer_.enable_tracking(enable_track);
}


//: Enable replay mode (for saved tracking results)
void pca_vehicle_manager::enable_replay_mode(bool val)
{
  replay_mode_ = val;
}


//: Start/Stop capture of video from a view
void pca_vehicle_manager::capture_view(int view_id, bool start)
{
  vidl_capture_tableau_sptr capture(NULL);
  switch(view_id){
    case 1:
      capture = exam_capture_tab_;
      break;
    case 2:
      capture = proj_capture_tab_;
      break;
    default:
      break;
  }
  if(!capture)
    return;
  
  if(start)
    capture->prompt_for_ostream();
  else
    capture->close_ostream();
}


//: read and play an animation script from a file
void pca_vehicle_manager::run_animation(const vcl_string& script)
{
  char command;
  unsigned int num_steps;
  vnl_vector<double> next_p=mesh_->params(), curr_p=mesh_->params();
  vcl_ifstream ifs(script.c_str());
  double angle = 0.0, next_angle = 0.0;
  const double pi = 3.141592653589793;
  while(ifs >> command)
  {
    switch(command)
    {
      case 'p':
      {
        char data[4096];
        ifs.getline(data,4096);
        vcl_stringstream sdata(data);
        next_p.clear();
        sdata >> next_p;
        break;
      }
      case 'r':
      {
        ifs >> next_angle;
        next_angle *= pi/180.0;
        next_angle += angle;
        if(angle > 2*pi && next_angle > 2*pi)
        {
          angle -= 2*pi;
          next_angle -= 2*pi;
        }
        break;
      }
      case 'a':
      {
        ifs >> num_steps;
        vcl_vector<double> p(curr_p.size(),0.0);
        for(unsigned int i=0; i<=num_steps; ++i){
          double t = double(i)/num_steps;
          unsigned int j=0;
          for(; j<next_p.size(); ++j)
            p[j] = curr_p[j]*(1-t) + next_p[j]*t;
          for(; j<curr_p.size(); ++j)
            p[j] = curr_p[j]*(1-t);
          change_mesh_params(p);
          mesh_xform_->rotation.setValue(SbVec3f(0, 0, 1), angle*(1-t) + next_angle*t);
          vgui::run_till_idle();
        }
        unsigned int j=0;
        for(; j<next_p.size(); ++j)
          curr_p[j] = next_p[j];
        for(; j<curr_p.size(); ++j)
          curr_p[j] = 0.0;
        angle = next_angle;
        break;
      }
      default:
      {
        char data[4096];
        ifs.getline(data,4096);
      }
    }
  }
}


//: read and play a tracking sequence from a file
void pca_vehicle_manager::run_track_results(const vcl_string& filename)
{
  vcl_ifstream ifs(filename.c_str());
  vcl_string input;
  // load model type
  ifs >> input;
  if(input == "Dodecahedral")
    set_vehicle_model(DODECAHEDRAL);
  else if(input == "Ferryman")
    set_vehicle_model(FERRYMAN);
  else if(input == "Detailed1")
    set_vehicle_model(DETAILED1);
  else if(input == "Detailed2")
    set_vehicle_model(DETAILED2);
  else if(input == "Detailed3")
    set_vehicle_model(DETAILED3);
  else{
    vcl_cerr << "unknown model type: "<<input<<vcl_endl;
    return;
  }
  
  int start_frame;
  ifs >> start_frame;
    
  set_fit_mode(true);
  // load video file
  ifs >> input;
  if(!open_istream(new vidl_ffmpeg_istream(input))){
    vcl_cerr<< "could not open video file: "<<input<<vcl_endl;
    return;
  }
     
  // load video file
  ifs >> input;
  if(!load_camera(input)){
    vcl_cerr<< "could not open camera: "<<input<<vcl_endl;
    return;
  }
  
  istream_->seek_frame(start_frame);
  compute_sun_direction();
  proj_tab_->clear();
  
  int frame_number = 0;
  while(ifs >> frame_number)
  {
    while(istream_->frame_number() < frame_number)
    {
      if(!istream_->advance())
        break;
      vcl_cout << "frame "<<istream_->frame_number()<<vcl_endl;
      
      vidl_frame_sptr frame = istream_->current_frame();
      if (frame) {
        static vil_image_view<vxl_byte> img;
        if (vidl_convert_to_view(*frame,img,VIDL_PIXEL_COLOR_RGB)){
          image_tab_->set_image_view(img);
          image_tab_->reread_image();
        }
        else{
          image_tab_->set_image_resource(NULL);
          return;
        }
      }
      if(istream_->frame_number() < frame_number)
      {
        image_tab_->post_redraw();
        vgui::run_till_idle();
      }
    }
    
    unsigned int id;
    ifs >> id;
    
    char data[4096];
    ifs.getline(data,4096);
    vcl_stringstream sdata(data);
    
    vnl_vector_fixed<double,3> r;
    double tv,av;
    vnl_vector<double> p;
    sdata >> translation_ >> r >>tv >> av >> p;
    rotation_ = vgl_rotation_3d<double>(r);
    mesh_->set_params(p);
    mesh_->compute_face_normals();
    frame_->refresh_values();
    draw_parts();
    update_mesh_node();
    image_tab_->post_redraw();
    vgui::run_till_idle();
  }
}


//: set the vehicle translation
void pca_vehicle_manager::set_translation(const vgl_vector_3d<double>& t)
{
  translation_ = t;
  proj2d_tab_->set_camera(compute_relative_camera());
  proj_tab_->post_redraw();
}


//: set the vehicle rotation
void pca_vehicle_manager::set_rotation(const vgl_rotation_3d<double>& R)
{
  rotation_ = R;
  proj2d_tab_->set_camera(compute_relative_camera());
  proj_tab_->post_redraw();
}


//: redraw texture space, the 3-d view, and the projection with current parameters
void pca_vehicle_manager::update_all_displays()
{
  update_mesh_node();  
  draw_texmap_parts();
  build_parts_node();
  update_projection();
}


//: update the projection of vehicle contours and parts
void pca_vehicle_manager::update_projection()
{
  draw_parts();
  proj_tab_->post_redraw();
}


//: Try to handle vgui messages
bool pca_vehicle_manager::handle_message(const vgui_message& m)
{
  if(m.from == gnd_cal_tab_.ptr())
  {
    this->set_camera(gnd_cal_tab_->camera());
    return true;
  }
  return false;
}


//: compute edges for visualization
void pca_vehicle_manager::compute_vis_edgels()
{
  const vil_image_view<float>& edge_map = is_fit_mode_video() ?
                                          video_optimizer_.edge_map() :
                                          mv_optimizer_.edge_map(frame_number_);
  
  if(!edge_map)
    return;
  
  vis_edgels_.clear();
  vcl_vector<modrec_edgel> medgels = modrec_find_all_edgels(edge_map);
  
  vcl_sort(medgels.begin(), medgels.end(), modrec_edgel_strength_less);

  for(unsigned int i=0; i<medgels.size(); ++i)
  {
    const modrec_edgel& m = medgels[i];
    double gx = vcl_cos(m.angle());
    double gy = vcl_sin(m.angle());
    vis_edgels_.push_back(vcl_pair<double,vnl_double_4>(m.strength(),
                                                        vnl_double_4(m.x(),m.y(),gx,gy)));
  }
  
}


//: Return true if the current frame is active
bool pca_vehicle_manager::is_current_frame_active() const
{ 
  if(is_fit_mode_video())
    return true;
  
  return mv_optimizer_.is_active(frame_number_); 
}


//: Set the options for fitting  
//  The vector of boolean options specifies which parameters to fit.
//  - [0] is top \a num_pc PCA params
//  - [1,2,3] is Tx,Ty,Tz respectively
//  - [4,5,6] is Rx,Ry,Rz respectively
void pca_vehicle_manager::
set_fit_options(const vcl_vector<bool>& options, 
                unsigned int num_pc, 
                double lambda, 
                double edge_scale)
{
  optimizer_->set_options(options,num_pc);  
  optimizer_->set_lambda(lambda);
  optimizer_->set_init_uncert(edge_scale);
  if(!options[0] || num_pc == 0)
    video_optimizer_.tracker().set_estimate_shape(false);
  else
    video_optimizer_.tracker().set_estimate_shape(true);
}


//: Fits the model parameters: PCA, translation, rotation to the image.
void pca_vehicle_manager::fit_model(unsigned int num_itr)
{
  //optimizer_->fit_model(num_itr, *mesh_, translation_, rotation_);
  //return;
  
  
  double edge_scale = optimizer_->estimate_initial_scale(*mesh_,translation_,rotation_,optimizer_->init_uncert())/4;
  if(edge_scale < 1.0) // lower bound at 1
    edge_scale = 1.0;
  vcl_cout << "initial scale = "<<edge_scale<<vcl_endl;
  optimizer_->set_mest_scale(edge_scale);
  
  double last_residual = vcl_numeric_limits<double>::infinity();
  vnl_vector<double> soln(optimizer_->num_params(),0.0);
  modrec_write_svg_curves("curves_init.svg",mesh_projector_);
  
  for(unsigned int k=0; k<num_itr; ++k){
    
    bool success = optimizer_->fit_model_once(*mesh_, translation_, rotation_, soln, last_residual, k==0);
    vcl_cout << "soln mag: "<<soln.magnitude() <<vcl_endl;
    if(!success){
      --k;
      if(edge_scale <= 1)
        break;
      last_residual = vcl_numeric_limits<double>::infinity();
      edge_scale /= vcl_sqrt(2);
      if(edge_scale < 1)
        edge_scale = 1.0;
      optimizer_->set_mest_scale(edge_scale);
    }
    else{
      update_all_displays();
      vgui::run_till_idle();
      vcl_stringstream svg_file;
      vcl_cout << "itr "<<k+1<<" scale "<<edge_scale<<vcl_endl;
      svg_file << "curves_"<<k<<".svg";
      modrec_write_svg_curves(svg_file.str(),mesh_projector_);
    }
  }
}


//: Draw the edgel match vectors for the current frame
void pca_vehicle_manager::draw_matches()
{
  vcl_vector<vgl_point_2d<double> > edgel_snaps;
  vcl_vector<vgl_point_2d<double> > edgels;
  vcl_vector<double> weights;
  const vil_image_view<float>& edge_map = is_fit_mode_video() ?
                                          video_optimizer_.edge_map() :
                                          mv_optimizer_.edge_map(frame_number_);
  optimizer_->last_edgel_matches(mesh_projector_, edge_map, 
                                edgel_snaps, edgels, weights);
  
  double minw = vcl_numeric_limits<double>::infinity();
  double maxw = -minw;
  for(unsigned int i=0; i<weights.size(); ++i)
  {
    if(weights[i] < minw) minw = weights[i];
    if(weights[i] > maxw) maxw = weights[i];
  }
  
  proj_tab_->set_foreground(0.0f,1.0f,1.0f);
  for(unsigned int i=0; i<edgels.size(); ++i)
  {
    const vgl_point_2d<double>& p0 = edgel_snaps[i];
    const vgl_point_2d<double>& p1 = edgels[i];
    proj_tab_->set_foreground(0.0f,(weights[i]-minw)/(maxw-minw),1.0f);
    proj_tab_->add_line(p0.x(), p0.y(), p1.x(), p1.y());
  }
  proj_tab_->post_redraw();
}


//: Draw the silhouette matches for the current frame
void pca_vehicle_manager::draw_silhouette_matches()
{
  if(!is_fit_mode_video())
    return;
  if(video_optimizer_.silhouette().num_sheets() == 0)
    return;
  
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
  
  
  vcl_vector<double> errors;
  video_optimizer_.compute_silhouette_errors(pts,norms,
                                             video_optimizer_.silhouette(),
                                             errors);
  if(errors.size() != pts.size())
    return;
  
  // draw the lines
  proj_tab_->set_foreground(0.0f,1.0f,1.0f);
  for(unsigned i=0; i<pts.size(); ++i){
    if(vnl_math_isfinite(errors[i])){
      vgl_point_2d<double> p2 = pts[i]+errors[i]*norms[i];
      proj_tab_->add_line(pts[i].x(), pts[i].y(), p2.x(), p2.y());
    }
  }

  proj_tab_->post_redraw();
}


//: Draw the Jacobian vectors
void pca_vehicle_manager::draw_jacobians()
{
  unsigned int dim = 0;
  double scale = 0.1;
  vcl_vector<vcl_string> ptypes(7);
  vcl_vector<int> choice_idx(7,-1);
  unsigned int num_pc = mesh_projector_.num_pc();
  ptypes[0] = "PC";
  ptypes[1] = "Tx"; ptypes[2] = "Ty"; ptypes[3] = "Tz";
  ptypes[4] = "Rx"; ptypes[5] = "Ry"; ptypes[6] = "Rz";
  unsigned int idx = 0;
  if(num_pc>0 && mesh_projector_.options()[0]){
    choice_idx[0]=0;
    idx = num_pc;
  }
  for(unsigned int i=1; i<7; ++i)
    if(mesh_projector_.options()[i]){
      choice_idx[i] = idx++;
    }
  vgui_dialog jdialog("Jacobian Vectors to Draw");
  static unsigned int pchoice = 0;
  jdialog.choice("Parameter",ptypes,pchoice);
  jdialog.field("PC Number",dim);
  jdialog.field("scale",scale);
  if(!jdialog.ask())
    return;
  
  int pidx = choice_idx[pchoice];
  if(pchoice == 0){
    if(!mesh_projector_.options()[0] || mesh_projector_.num_pc() <= dim){
      vcl_vector<bool> options = vcl_vector<bool>(7,false);
      options[0] = true;
      mesh_projector_.project(camera(), *mesh_, rotation_, translation_, sun_dir_, options, dim+1);
      pidx = dim;
    }
  }
  else if(pidx < 0){
    vcl_vector<bool> options = vcl_vector<bool>(7,false);
    options[pchoice] = true;
    mesh_projector_.project(camera(), *mesh_, rotation_, translation_, sun_dir_, options, 0);
    pidx = 0;
  }
  const vcl_vector<vcl_vector<vgl_point_2d<double> > >& contours = mesh_projector_.contours();
  const vcl_vector<vcl_vector<vgl_point_2d<double> > >& parts = mesh_projector_.parts();
  const vcl_vector<vcl_vector<vgl_point_2d<double> > >& silhouette = mesh_projector_.silhouette();
  const vcl_vector<vcl_vector<vnl_matrix<double> > >& Jc = mesh_projector_.contours_jacobians();
  const vcl_vector<vcl_vector<vnl_matrix<double> > >& Jp = mesh_projector_.parts_jacobians();
  const vcl_vector<vcl_vector<vnl_matrix<double> > >& Jtp = mesh_projector_.parts_texture_jacobians();
  const vcl_vector<vcl_vector<vnl_matrix<double> > >& Js = mesh_projector_.silhouette_jacobians();

  
  proj_tab_->set_foreground(1.0f,0.0f,0.0f);
  for(unsigned int i=0; i<contours.size(); ++i)
  {
    const vcl_vector<vgl_point_2d<double> >& contour = contours[i];
    const vcl_vector<vnl_matrix<double> >& Ji = Jc[i];
    for(unsigned int j=0; j<contour.size(); ++j)
    {
      const vgl_point_2d<double>& p = contour[j];
      
      double du = scale*Ji[j][0][pidx];
      double dv = scale*Ji[j][1][pidx];
      proj_tab_->add_line(p.x(), p.y(), p.x()+du, p.y()+dv);
    }
  }
  proj_tab_->set_foreground(0.0f,1.0f,0.0f);
  for(unsigned int i=0; i<parts.size(); ++i)
  {
    const vcl_vector<vgl_point_2d<double> >& part = parts[i];
    const vcl_vector<vnl_matrix<double> >& Ji = Jp[i];
    for(unsigned int j=0; j<part.size(); ++j)
    {
      const vgl_point_2d<double>& p = part[j];
      
      double du = scale*Ji[j][0][pidx];
      double dv = scale*Ji[j][1][pidx];
      proj_tab_->add_line(p.x(), p.y(), p.x()+du, p.y()+dv);
    }
  }
  proj_tab_->set_foreground(0.0f,1.0f,1.0f);
  for(unsigned int i=0; i<silhouette.size(); ++i)
  {
    const vcl_vector<vgl_point_2d<double> >& sil = silhouette[i];
    const vcl_vector<vnl_matrix<double> >& Ji = Js[i];
    for(unsigned int j=0; j<sil.size(); ++j)
    {
      const vgl_point_2d<double>& p = sil[j];
      
      double du = scale*Ji[j][0][pidx];
      double dv = scale*Ji[j][1][pidx];
      proj_tab_->add_line(p.x(), p.y(), p.x()+du, p.y()+dv);
    }
  }
  proj_tab_->post_redraw();
  
  const part_map& partsm = mesh_->parts();
  vcl_vector<vgl_point_2d<double> > all_points;
  for(part_map::const_iterator itr=partsm.begin(); itr!=partsm.end(); ++itr)
  {
    const vcl_vector<vgl_point_2d<double> >& poly = itr->second[0];
    for(unsigned int i=0; i<poly.size(); ++i)
    {
      all_points.push_back(poly[i]);
    }
  }
  if(!Jtp.empty())
  {
    const vcl_vector<vcl_vector<vcl_pair<unsigned int,unsigned int> > >&
    parts_idx = mesh_projector_.parts_indices();
    typedef modrec_pca_vehicle::uv_point uv_point;
    const vcl_vector<vcl_vector<uv_point> >& parts_uv = mesh_->parts_bary();
    for(unsigned int i=0; i<parts_idx.size(); ++i)
    {
      const vcl_vector<vnl_matrix<double> >& Ji = Jtp[i];
      for(unsigned int j=0; j<parts_idx[i].size(); ++j)
      {
        unsigned int i1 = parts_idx[i][j].first, i2 = parts_idx[i][j].second;
        double t2 = parts_uv[i1][i2].t;
        double t1 = 1-t2; 
        const vgl_point_2d<double>& p1 = all_points[parts_uv[i1][i2].end_point1]; 
        const vgl_point_2d<double>& p2 = all_points[parts_uv[i1][i2].end_point2];
        vgl_point_2d<double> p(p1.x()*t1+p2.x()*t2, p1.y()*t1+p2.y()*t2);
        double du = scale*Ji[j][pidx][0];
        double dv = scale*Ji[j][pidx][1];
        tex_parts_tab_->add_line(p.x(), p.y(), p.x()+du, p.y()+dv);
      }
    }
    tex_parts_tab_->post_redraw();
  }
  
}

