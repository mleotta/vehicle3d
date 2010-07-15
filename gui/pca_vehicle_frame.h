// This is mleotta/gui/pca_vehicle/pca_vehicle_frame.h
#ifndef pca_vehicle_frame_h_
#define pca_vehicle_frame_h_
//=========================================================================
//:
// \file
// \brief  wxWidgets main frame class.
// \author Matt Leotta (mleotta)
//
// \verbatim
//  Modifications
//   08/01/2008 - File created. (mleotta)
// \endverbatim
//=========================================================================

#include <wx/frame.h>
#include <wx/spinctrl.h>

#include <vgui/wx/vgui_wx.h>
#include <vgui/vgui_message.h>

#include "pca_vehicle_manager.h"

class wxSliderPanel;
class wxVideoControl;
class vgui_wx_observer;

//-------------------------------------------------------------------------
//: The wxWidgets main frame class.
//-------------------------------------------------------------------------
class pca_vehicle_frame : public wxFrame
{
  DECLARE_CLASS(pca_vehicle_frame)
  DECLARE_EVENT_TABLE()

public:
  
  friend class pca_vehicle_app;

  //: Constructor - default.
  pca_vehicle_frame(wxWindow* parent,
                     wxWindowID id,
                     const wxString& title,
                     const wxPoint& pos,
                     const wxSize& size,
                     long type);

  //: Destructor
  ~pca_vehicle_frame();

  //wxMenu* edit_menu() const { return edit_menu_; }

  // Event handlers.
  void on_about(wxCommandEvent& event);
  void on_quit (wxCommandEvent& event);
  void load_image(wxCommandEvent& event);
  void load_video(wxCommandEvent& event);
  void load_camera(wxCommandEvent& event);
  void load_mesh(wxCommandEvent& event);
  void load_truth_mesh(wxCommandEvent& event);
  void load_parts(wxCommandEvent& event);
  void load_pca(wxCommandEvent& event);

  void save_camera(wxCommandEvent& event);
  void save_3d_parts(wxCommandEvent& event);
  void save_svg(wxCommandEvent& event);
  void save_edge_image(wxCommandEvent& event);
  void save_video_frame(wxCommandEvent& event);
  void save_as_ps(wxCommandEvent& event);

  void scale_camera(wxCommandEvent& event);
  void set_camera_relative(wxCommandEvent& event);
  void set_mode_video(wxCommandEvent& event);
  void show(wxCommandEvent& event);
  void draw(wxCommandEvent& event);
  void set_vehicle_model(wxCommandEvent& event);
  void compute_error(wxCommandEvent& event);
  void capture_view(wxCommandEvent& event);
  void run_animation(wxCommandEvent& event);
  void run_track_results(wxCommandEvent& event);
  void clear_pca(wxCommandEvent& event);
  void change_imode(wxCommandEvent& event);
  void fit_model(wxCommandEvent& event);

  void toggle_extrinsics(wxCommandEvent& event);
  void change_options(wxCommandEvent& event);
  void change_spin_options(wxSpinEvent& event);
  void change_translation(wxCommandEvent& event);
  void change_rotation(wxCommandEvent& event);
  void update_projection(wxCommandEvent& event);

  void active_frame(wxCommandEvent& event);
  void keyboard_commands(wxKeyEvent& event);

  void refresh_values();
  
  void update_fit_options();

  //: Handle vgui messages 
  void handle_message(const vgui_message& m);

  //: Callback for changed interaction mode
  void interaction_mode_changed();
  
private:
  //: initialize the sliders with the PCA data
  void init_sliders();
  
  pca_vehicle_manager manager_;

  vgui_wx_observer* observer_;

  vgui_wx_adaptor*  canvas_tex_;
  vgui_wx_adaptor*  canvas_3d_;
  vgui_wx_adaptor*  canvas_proj_;

  wxSliderPanel*   sliders_;
  wxVideoControl*  video_control_;

  wxPanel* video_panel_;

  //: Statusbar that vgui writes to.
  vgui_wx_statusbar* statusbar_;

};

#endif // pca_vehicle_frame_h_
