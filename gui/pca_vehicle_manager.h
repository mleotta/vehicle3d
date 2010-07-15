// This is mleotta/gui/pca_vehicle/pca_vehicle_manager.h
#ifndef pca_vehicle_manager_h_
#define pca_vehicle_manager_h_
//=========================================================================
//:
// \file
// \brief  Data manager for the PCA vehicle GUI
// \author Matt Leotta (mleotta)
//
// \verbatim
//  Modifications
//   08/19/2008 - File created. (mleotta)
// \endverbatim
//=========================================================================


#include <vgui/vgui_easy2D_tableau.h>
#include <vil/vil_image_view.h>


#include "gnd_cal_tableau.h"

#include <vgl/vgl_line_segment_2d.h>
#include <vgl/vgl_polygon.h>
#include <modrec/modrec_pca_vehicle.h>
#include <modrec/modrec_vehicle_fit_multiview.h>
#include <modrec/modrec_vehicle_fit_video.h>
#include <modrec/modrec_vehicle_state.h>
#include <vpgl/vpgl_perspective_camera.h>
#include <vidl/vidl_istream_sptr.h>
#include <vidl/gui/vidl_capture_tableau_sptr.h>

#include <bgui/bgui_selector_tableau_sptr.h>
#include <bgui3d/bgui3d_examiner_tableau.h>
#include <bgui3d/bgui3d_project2d_tableau.h>

#include <Inventor/nodes/SoSeparator.h>
#include <Inventor/nodes/SoTransform.h>

// forward declarations
class pca_vehicle_frame;
class scene_handler_tableau;
class vidl_istream;

//: data manager class for PCA vehicle GUI
// interacts with pca_vehicle_frame 
class pca_vehicle_manager
{
public:
  friend class scene_handler_tableau;
  typedef vcl_map<vcl_string, vgl_polygon<double> > part_map;

  enum interact_mode { NONE, MOVE };
  
  enum vehicle_model { DODECAHEDRAL, FERRYMAN, DETAILED1, DETAILED2, DETAILED3,};


  //: Constructor 
  pca_vehicle_manager(pca_vehicle_frame* frame);

  //: Destructor
  ~pca_vehicle_manager();

  //: Initialize the 3D examiner view
  //  returns the top level tableau to be added to a shell
  vgui_tableau_sptr init_3d_view();
  //: Initialize the texture space view
  //  returns the top level tableau to be added to a shell
  vgui_tableau_sptr init_tex_view();
  //: Initialize the image projection view
  //  returns the top level tableau to be added to a shell
  vgui_tableau_sptr init_proj_view();
  
  //: Enable video mode (true), or multiview mode (false)
  void set_fit_mode(bool use_video);
  //: Is the fitting mode video (true) or multiview (false)
  bool is_fit_mode_video() const;

  //: return the vehicle translation
  const vgl_vector_3d<double>& translation() const { return translation_; }
  //: return the vehicle rotation
  const vgl_rotation_3d<double>& rotation() const { return rotation_; }

  //: set the vehicle translation
  void set_translation(const vgl_vector_3d<double>& t);
  //: set the vehicle rotation
  void set_rotation(const vgl_rotation_3d<double>& R);

  //: set the camera matrix
  void set_camera(const vpgl_perspective_camera<double>& cam);
  
  //: compute the sun direction for shadow casting
  void compute_sun_direction();
  
  //: set the camera matrix relative to the vehicle
  //  incorporates the vehicle rotation and translation into the camera
  //  and the resets the vehicle rotation and translation
  void set_camera_relative();
  
  //: redraw texture space, the 3-d view, and the projection with current parameters
  void update_all_displays();

  //: update the projection of vehicle contours and parts
  void update_projection();

  //: Load the image file
  bool load_image(const vcl_string& filename);
  //: Load the camera file
  bool load_camera(const vcl_string& filename);
  //: Load the mesh file with a matching parts file
  bool load_mesh(const vcl_string& meshfile,
                 const vcl_string& partsfile);
  //: Load the ground truth mesh file
  bool load_truth_mesh(const vcl_string& filename);
  //: Load the parts file
  bool load_parts(const vcl_string& filename);
  //: Load the PCA data file
  bool load_pca(const vcl_string& filename);

  //: Save the camera to a file
  //  if \a normalize is true then write the camera with normalized image coordinates 
  bool save_camera(const vcl_string& filename, bool normalize = false) const;
  
  //: Save a file with the current 3d coordinates of parts
  bool save_3d_parts(const vcl_string& filename) const;
  
  //: Save the projected contours as SVG
  bool save_svg(const vcl_string& filename) const;
  
  //: Save an image of the detected edges
  bool save_edge_image(const vcl_string& filename) const;
  
  //: Save an image of the current video frame
  bool save_video_frame(const vcl_string& filename) const;

  //: Open input video stream
  bool open_istream(const vidl_istream_sptr& istream);

  //: Attach observers
  void attach(vgui_observer *o);

  //: return the number of frames in the current video if seekable
  int num_video_frames() const;
  
  //: return the current video frame number
  int current_frame() const;

  //: seek the video to the frame number
  void video_seek(int frame);
  
  //: advance the video to the next frame
  bool advance_video();
  
  //: Either enable or disable fitting on the current frame
  void set_frame_active(bool active);
  
  //: Return true if the current frame is active
  bool is_current_frame_active() const;
  
  //: Set the options for fitting  
  //  The vector of boolean options specifies which parameters to fit.
  //  - [0] is top \a num_pc PCA params
  //  - [1,2,3] is Tx,Ty,Tz respectively
  //  - [4,5,6] is Rx,Ry,Rz respectively
  void set_fit_options(const vcl_vector<bool>& options, unsigned int num_pc, 
                       double lambda, double edge_scale);
  
  //: Fits the model parameters: PCA, translation, rotation to the image.
  void fit_model(unsigned int num_itr);
  
  //: Draw the edgel match vectors for the current frame
  void draw_matches();
  
  //: Draw the silhouette matches for the current frame
  void draw_silhouette_matches();
  
  //: Draw the Jacobian vectors
  void draw_jacobians();

  //: Compute a camera that incorporates the vehicle rotation and translation
  vpgl_perspective_camera<double> compute_relative_camera() const;

  //: Scale the camera
  void scale_camera(double scale);
  
  //: compute the RMS error between the PCA mesh and Ground truth
  void compute_error();

  //: change the mesh parameters
  void change_mesh_params(const vcl_vector<double>& params);
  //: update a single mesh parameters
  void update_mesh_param(unsigned int i, double param);

  //: Access the vehicle mesh
  const modrec_pca_vehicle& mesh() const { return *mesh_; }

  //: Access the camera
  const vpgl_perspective_camera<double>& camera() const;

  //: Access the interaction mode
  interact_mode interaction_mode() const { return imode_; }

  //: Set the interaction mode
  void set_interaction_mode(const interact_mode& mode);

  //: Set the state of edge visibility
  void show_edges(bool show);
  //: Set the 3d axes
  void show_axes(bool show);
  //: Set the state of draw_reprojected
  void draw_reprojected(bool val);
  //: Enable tracking if in video mode
  void enable_tracking(bool enable_track);
  //: Enable replay mode (for saved tracking results)
  void enable_replay_mode(bool val);
  
  //: Choose the type of vehicle model from the enum
  void set_vehicle_model(vehicle_model vm);
  
  //: Start/Stop capture of video from a view
  void capture_view(int view_id, bool start);
  
  //: read and play an animation script from a file
  void run_animation(const vcl_string& script);
  
  //: read and play a tracking sequence from a file
  void run_track_results(const vcl_string& filename);

  //: Try to handle vgui messages
  bool handle_message(const vgui_message& m);
  
  //: draw a hypothesized vehicle shape and pose
  void draw_hypothesis(const modrec_vehicle_state& state);
  
  //: draw the current vehicle tracking states
  void draw_current_states();
  
  //: set the vehicle tracking states
  void set_tracking_states(const vcl_vector<modrec_vehicle_state>& states);
  
  //: set the history of vehicle tracking states indexed over frame number
  void set_state_map(const vcl_map<unsigned int,vcl_vector<modrec_vehicle_state> >& state_map);

private:
  void draw_texmap();
  void draw_texmap_parts();
  void draw_parts();
  //: Initialize the texture map for detailed meshes
  void init_mesh_tex(modrec_pca_vehicle& mesh);
  void init_mesh();
  void build_mesh_node();
  void update_mesh_node();
  void build_parts_node();

  void draw_curves(vgui_easy2D_tableau_sptr tab,
                   const vcl_vector<vcl_vector<vgl_point_2d<double> > >& curves);

  void compute_vis_edgels();

  // Member variables -------------------------------------

  pca_vehicle_frame* frame_;

  interact_mode imode_;

  SoSeparator* mesh_node_;
  SoSeparator* parts_node_;
  SoTransform* mesh_xform_;

  modrec_pca_vehicle* mesh_;
  modrec_pca_vehicle_projector mesh_projector_;
  
  //: dodecahedral mesh
  modrec_pca_vehicle dodec_mesh_;
  //: Ferryman's mesh
  modrec_pca_vehicle ferryman_mesh_;
  //: detailed mesh with parts at 3 resolutions
  modrec_pca_vehicle detailed1_mesh_, detailed2_mesh_, detailed3_mesh_;
  

  //: recorded tracking states indexed by frame number
  vcl_map<unsigned int,vcl_vector<modrec_vehicle_state> > state_map_;
  
  //: the ground truth vehiclde mesh
  modrec_pca_vehicle truth_mesh_;

  vidl_istream_sptr istream_;
  int frame_number_;
  
  
  vil_image_resource_sptr img_;
  bool norm_cam_;
  vpgl_calibration_matrix<double> norm_K_;
  vgl_vector_3d<double> translation_;
  vgl_rotation_3d<double> rotation_;
  
  // data for computing direction of the sun for shadow casting
  vgl_vector_3d<double> sun_dir_;
  double solar_lat_;
  double solar_lon_;
  int solar_day_;
  double solar_utc_;
  double solar_atn_;
  
  modrec_vehicle_fit* optimizer_;
  modrec_vehicle_fit_multiview mv_optimizer_;
  modrec_vehicle_fit_video video_optimizer_;

  vcl_vector<vcl_pair<double,vnl_double_4> > vis_edgels_;

  vil_image_view<float> debug_;

  bgui_selector_tableau_sptr selector_tab_;
  bgui3d_examiner_tableau_sptr exam_tab_;
  bgui3d_project2d_tableau_sptr proj2d_tab_;
  vgui_easy2D_tableau_sptr proj_tab_;
  vgui_image_tableau_sptr image_tab_;
  vgui_image_tableau_sptr depth_image_tab_;
  vgui_image_tableau_sptr edge_map_tab_;
  vgui_image_tableau_sptr debug_image_tab_;
  vgui_easy2D_tableau_sptr detect_tab_;
  dbgui_gnd_cal_tableau_sptr gnd_cal_tab_;
  vgui_easy2D_tableau_sptr tex_tab_;
  vgui_easy2D_tableau_sptr tex_parts_tab_;
  
  vidl_capture_tableau_sptr exam_capture_tab_;
  vidl_capture_tableau_sptr proj_capture_tab_;

  bool show_edges_;
  bool replay_mode_;
  
  //: update curves by reprojecting last valid curves
  bool draw_reprojected_;
};

#endif // pca_vehicle_manager_h_
