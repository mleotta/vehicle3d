// This is gui/pca_vehicle_frame.cxx
//=========================================================================
//:
// \file
// \brief  wxWidgets main frame class.
//
//          Copyright Matthew Leotta 2006 - 2010.
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file ../LICENSE_1_0.txt or copy at
//          http://www.boost.org/LICENSE_1_0.txt)
//
// See pca_vehicle_frame.h for details.
//=========================================================================

#include "pca_vehicle_frame.h"

#include <wx/xrc/xmlres.h>

#include <wx/wx.h>

#include <vgui/vgui.h>
#include <vgui/vgui_shell_tableau.h>
#include <vgui/vgui_observer.h>

#include <vgui/wx/wxSliderPanel.h>
#include <vgui/wx/wxVideoControl.h>

#include <vidl/gui/vidl_gui_param_dialog.h>
#include <vidl/vidl_ffmpeg_istream.h>

#include <vul/vul_timer.h>

#include "gui_utils.h"

//-------------------------------------------------------------------------
//-------------------------------------------------------------------------


class vgui_wx_observer: public vgui_observer
{
public:
  vgui_wx_observer(pca_vehicle_frame* f) : frame(f) {}

  virtual void update(const vgui_message& m)
  {
    frame->handle_message(m);
  }
  pca_vehicle_frame* frame;
};

//-------------------------------------------------------------------------
//-------------------------------------------------------------------------
IMPLEMENT_CLASS(pca_vehicle_frame, wxFrame)

pca_vehicle_frame::pca_vehicle_frame(wxWindow* parent,
                                     wxWindowID id,
                                     const wxString& title,
                                     const wxPoint& pos,
                                     const wxSize& size,
                                     long type)
  //: wxFrame(parent, id, title, pos, size, type)
  : manager_(this)
{
  //SetIcon(wxICON(appicon));
  wxXmlResource::Get()->LoadFrame(this,parent,wxT("pca_vehicle_frame"));

  observer_ = new vgui_wx_observer(this);

  wxStatusBar* bar;

  wxTextCtrl *tx_text, *ty_text, *tz_text;
  wxTextCtrl *rx_text, *ry_text, *rz_text;


#define FIND_WIDGET(V,T,N) \
  { wxWindow* w = this->FindWindow(wxT( N )); \
  if(!w) { vcl_cout << "Could not find " N << vcl_endl; exit(-1); }\
  if(!(V = dynamic_cast<T*>(w))) \
  { vcl_cout << "Invalid widget type for " N << vcl_endl; exit(-1); } }

  FIND_WIDGET(canvas_tex_,vgui_wx_adaptor,"vgui_adaptor1");
  FIND_WIDGET(canvas_3d_,vgui_wx_adaptor,"vgui_adaptor2");
  FIND_WIDGET(canvas_proj_,vgui_wx_adaptor,"vgui_adaptor3");
  FIND_WIDGET(sliders_,wxSliderPanel,"slider_panel");
  FIND_WIDGET(video_control_,wxVideoControl,"video_control");
  FIND_WIDGET(bar, wxStatusBar,"statusbar");
  FIND_WIDGET(video_panel_, wxPanel,"video_panel");
  FIND_WIDGET(tx_text, wxTextCtrl,"tx_text");
  FIND_WIDGET(ty_text, wxTextCtrl,"ty_text");
  FIND_WIDGET(tz_text, wxTextCtrl,"tz_text");
  FIND_WIDGET(rx_text, wxTextCtrl,"rx_text");
  FIND_WIDGET(ry_text, wxTextCtrl,"ry_text");
  FIND_WIDGET(rz_text, wxTextCtrl,"rz_text");
#undef FIND_WIDGET

  tx_text->SetValidator( wxTextValidator(wxFILTER_NUMERIC) );
  ty_text->SetValidator( wxTextValidator(wxFILTER_NUMERIC) );
  tz_text->SetValidator( wxTextValidator(wxFILTER_NUMERIC) );
  rx_text->SetValidator( wxTextValidator(wxFILTER_NUMERIC) );
  ry_text->SetValidator( wxTextValidator(wxFILTER_NUMERIC) );
  rz_text->SetValidator( wxTextValidator(wxFILTER_NUMERIC) );
  

  video_panel_->Show(false);
  video_panel_->GetParent()->Layout();

  statusbar_ = new vgui_wx_statusbar;
  statusbar_->set_widget(bar);
  vgui::out.rdbuf(statusbar_->statusbuf());

  // order of initialization is important here
  canvas_tex_->set_tableau(vgui_shell_tableau_new(manager_.init_tex_view()));
  canvas_3d_->set_tableau(vgui_shell_tableau_new(manager_.init_3d_view()));
  canvas_proj_->set_tableau(vgui_shell_tableau_new(manager_.init_proj_view()));

  sliders_->attach(static_cast<vgui_observer*>(observer_));
  video_control_->attach(static_cast<vgui_observer*>(observer_));
  manager_.attach(static_cast<vgui_observer*>(observer_));
  
}


pca_vehicle_frame::~pca_vehicle_frame()
{
  delete statusbar_;
  delete observer_;
}


//-------------------------------------------------------------------------
// Event handling.
//-------------------------------------------------------------------------
BEGIN_EVENT_TABLE(pca_vehicle_frame, wxFrame)
  EVT_MENU(XRCID("menu_about"), pca_vehicle_frame::on_about)
  EVT_MENU(XRCID("menu_quit"),  pca_vehicle_frame::on_quit)
  EVT_MENU(XRCID("menu_load_image"),  pca_vehicle_frame::load_image)
  EVT_MENU(XRCID("menu_load_video"),  pca_vehicle_frame::load_video)
  EVT_MENU(XRCID("menu_load_camera"),  pca_vehicle_frame::load_camera)
  EVT_MENU(XRCID("menu_load_mesh"),  pca_vehicle_frame::load_mesh)
  EVT_MENU(XRCID("menu_load_truth_mesh"),  pca_vehicle_frame::load_truth_mesh)
  EVT_MENU(XRCID("menu_load_pca"),  pca_vehicle_frame::load_pca)
  EVT_MENU(XRCID("menu_load_parts"),  pca_vehicle_frame::load_parts)
  EVT_MENU(XRCID("menu_save_camera"),  pca_vehicle_frame::save_camera)
  EVT_MENU(XRCID("menu_save_3d_parts"),  pca_vehicle_frame::save_3d_parts)
  EVT_MENU(XRCID("menu_save_svg"),  pca_vehicle_frame::save_svg)
  EVT_MENU(XRCID("menu_save_edge_image"),  pca_vehicle_frame::save_edge_image)
  EVT_MENU(XRCID("menu_save_video_frame"),  pca_vehicle_frame::save_video_frame)
  EVT_MENU(XRCID("menu_save_as_ps"),  pca_vehicle_frame::save_as_ps)
  EVT_MENU(XRCID("menu_set_camera_rel"),  pca_vehicle_frame::set_camera_relative)
  EVT_MENU(XRCID("menu_scale_ft2m"),  pca_vehicle_frame::scale_camera)
  EVT_MENU(XRCID("menu_set_mode_video"),  pca_vehicle_frame::set_mode_video)
  EVT_MENU(XRCID("menu_show_edges"),  pca_vehicle_frame::show)
  EVT_MENU(XRCID("menu_show_axes"),  pca_vehicle_frame::show)
  EVT_MENU(XRCID("menu_draw_reprojected"),  pca_vehicle_frame::show)
  EVT_MENU(XRCID("menu_enable_tracking"),  pca_vehicle_frame::show)
  EVT_MENU(XRCID("menu_draw_matches"),  pca_vehicle_frame::draw)
  EVT_MENU(XRCID("menu_draw_sil_matches"),  pca_vehicle_frame::draw)
  EVT_MENU(XRCID("menu_draw_jacobians"),  pca_vehicle_frame::draw)
  EVT_MENU(XRCID("menu_use_dodec"),  pca_vehicle_frame::set_vehicle_model)
  EVT_MENU(XRCID("menu_use_ferryman"),  pca_vehicle_frame::set_vehicle_model)
  EVT_MENU(XRCID("menu_use_detailed1"),  pca_vehicle_frame::set_vehicle_model)
  EVT_MENU(XRCID("menu_use_detailed2"),  pca_vehicle_frame::set_vehicle_model)
  EVT_MENU(XRCID("menu_use_detailed3"),  pca_vehicle_frame::set_vehicle_model)
  EVT_MENU(XRCID("menu_compute_error"),  pca_vehicle_frame::compute_error)
  EVT_MENU(XRCID("menu_capture_3d_view"),  pca_vehicle_frame::capture_view)
  EVT_MENU(XRCID("menu_capture_proj_view"),  pca_vehicle_frame::capture_view)
  EVT_MENU(XRCID("menu_run_animation"),  pca_vehicle_frame::run_animation)
  EVT_MENU(XRCID("menu_run_track_results"),  pca_vehicle_frame::run_track_results)

  EVT_BUTTON(XRCID("clear_pca_button"),  pca_vehicle_frame::clear_pca)
  EVT_RADIOBUTTON(XRCID("radio_interaction_none"), pca_vehicle_frame::change_imode)
  EVT_RADIOBUTTON(XRCID("radio_interaction_move"), pca_vehicle_frame::change_imode)
  EVT_BUTTON(XRCID("fit_model_button"),  pca_vehicle_frame::fit_model)
 
  EVT_CHECKBOX(XRCID("video_frame_active"), pca_vehicle_frame::active_frame)

  EVT_CHECKBOX(XRCID("tx_checkbox"), pca_vehicle_frame::toggle_extrinsics)
  EVT_CHECKBOX(XRCID("ty_checkbox"), pca_vehicle_frame::toggle_extrinsics)
  EVT_CHECKBOX(XRCID("tz_checkbox"), pca_vehicle_frame::toggle_extrinsics)
  EVT_CHECKBOX(XRCID("rx_checkbox"), pca_vehicle_frame::toggle_extrinsics)
  EVT_CHECKBOX(XRCID("ry_checkbox"), pca_vehicle_frame::toggle_extrinsics)
  EVT_CHECKBOX(XRCID("rz_checkbox"), pca_vehicle_frame::toggle_extrinsics)
  EVT_SPINCTRL(XRCID("num_to_save_spin"), pca_vehicle_frame::change_spin_options)
  EVT_TEXT(XRCID("lambda_text"), pca_vehicle_frame::change_options)
  EVT_TEXT(XRCID("edge_scale_text"), pca_vehicle_frame::change_options)

  EVT_TEXT(XRCID("tx_text"), pca_vehicle_frame::change_translation)
  EVT_TEXT(XRCID("ty_text"), pca_vehicle_frame::change_translation)
  EVT_TEXT(XRCID("tz_text"), pca_vehicle_frame::change_translation)
  EVT_TEXT(XRCID("rx_text"), pca_vehicle_frame::change_rotation)
  EVT_TEXT(XRCID("ry_text"), pca_vehicle_frame::change_rotation)
  EVT_TEXT(XRCID("rz_text"), pca_vehicle_frame::change_rotation)
  EVT_TEXT_ENTER(XRCID("tx_text"), pca_vehicle_frame::update_projection)
  EVT_TEXT_ENTER(XRCID("ty_text"), pca_vehicle_frame::update_projection)
  EVT_TEXT_ENTER(XRCID("tz_text"), pca_vehicle_frame::update_projection)
  EVT_TEXT_ENTER(XRCID("rx_text"), pca_vehicle_frame::update_projection)
  EVT_TEXT_ENTER(XRCID("ry_text"), pca_vehicle_frame::update_projection)
  EVT_TEXT_ENTER(XRCID("rz_text"), pca_vehicle_frame::update_projection)
END_EVENT_TABLE()


void pca_vehicle_frame::on_about(wxCommandEvent& event)
{
  wxString msg;
  msg.Printf(wxT("Built with %s"), wxVERSION_STRING);
  wxMessageBox(msg, wxT("About PCA Vehicle GUI"),
               wxOK | wxICON_INFORMATION, this);
}

void pca_vehicle_frame::on_quit(wxCommandEvent& event)
{
  Close();
}



//=========================================================================
//: load an image 
//=========================================================================
void pca_vehicle_frame::load_image(wxCommandEvent& event)
{
  wxString wildcard = wxT("All Images (*.png;*.jpg;*.jpeg;*.tif;*.tiff;*.bmp)|"
                          "*.png;*.jpg;*.jpeg;*.tif;*.tiff;*.bmp|"
                          "PNG (*.png)|*.png|"
                          "JPEG (*.jpg;*.jpeg)|*.jpg;*.jpeg|"
                          "TIFF (*.tif;*.tiff)|*.tif;*.tiff|"
                          "BMP (*.bmp)|*.bmp");
  wxFileDialog file_dialog(this, wxT("Select an Image File"),
                           wxEmptyString, wxEmptyString,
                           wildcard, wxOPEN);

  if(file_dialog.ShowModal() != wxID_OK)
    return;

  vcl_string filename = static_cast<const char*>(file_dialog.GetPath().mb_str(wxConvUTF8));

  if(!manager_.load_image(filename))
  {
    wxMessageDialog err_dialog( this,
    wxT("Failed to load image:\n")+file_dialog.GetPath(),
    wxT("Open File Error"),
    wxOK|wxICON_ERROR);

    err_dialog.ShowModal();
  }
  
  //: make sure fit options are updated at least once if not manually changed
  update_fit_options();
}


//=========================================================================
//: load a video
//=========================================================================
void pca_vehicle_frame::load_video(wxCommandEvent& event)
{
  vidl_istream_sptr is = vidl_gui_open_istream_dialog();
  // need to get num_frames before the manager "opens" the stream
  // because num_frames() is only valid when called before advance.
  int num_frames = -1;
  if(is && is->is_seekable()){
    double duration = is->duration();
    double frame_rate = is->frame_rate();
    int est_num_frames = static_cast<int>(duration*frame_rate +0.5);
    if (est_num_frames > 0)
      num_frames = est_num_frames;
    else // count the frames
      num_frames = is->num_frames();
  }
  if(manager_.open_istream(is)){
    video_control_->set_num_frames(num_frames);
    video_panel_->Show(true);
    video_panel_->GetParent()->Layout();
  }
  manager_.enable_replay_mode(false);
  
  //: make sure fit options are updated at least once if not manually changed
  update_fit_options();
}


//=========================================================================
//: load a camera matrix
//=========================================================================
void pca_vehicle_frame::load_camera(wxCommandEvent& event)
{
  wxString wildcard = wxT("Camera File (*.cam)|*.cam|"
                          "All Files |*");
  wxFileDialog file_dialog(this, wxT("Select a Camera File"),
                           wxEmptyString, wxEmptyString,
                           wildcard, wxOPEN);

  if(file_dialog.ShowModal() != wxID_OK)
    return;

  vcl_string filename = static_cast<const char*>(file_dialog.GetPath().mb_str(wxConvUTF8));

  if(!manager_.load_camera(filename))
  {
    wxMessageDialog err_dialog( this,
    wxT("Perspective Decomposition Failed"),
    wxT("Open File Error"),
    wxOK|wxICON_ERROR);

    err_dialog.ShowModal();
  }

}


//=========================================================================
//: save a camera matrix
//=========================================================================
void pca_vehicle_frame::save_camera(wxCommandEvent& event)
{
  wxString wildcard = wxT("Camera File (*.cam)|*.cam|"
                          "All Files |*");
  wxFileDialog file_dialog(this, wxT("Save to Camera File"),
                           wxEmptyString, wxEmptyString,
                           wildcard, wxSAVE);

  if(file_dialog.ShowModal() != wxID_OK)
    return;

  vcl_string filename = static_cast<const char*>(file_dialog.GetPath().mb_str(wxConvUTF8));
  
  wxMessageDialog norm_dialog( this,
                             wxT("Write normalized camera file?"),
                             wxT("Save Camera"),
                             wxYES|wxNO|wxICON_QUESTION);
  
  bool normalize = norm_dialog.ShowModal() == wxID_YES;

  manager_.save_camera(filename,normalize);
}


//=========================================================================
//: save a 3d parts file
//=========================================================================
void pca_vehicle_frame::save_3d_parts(wxCommandEvent& event)
{
  wxString wildcard = wxT("Parts 3D File (*.parts3d)|*.parts3d|"
                          "All Files |*");
  wxFileDialog file_dialog(this, wxT("Save to Parts 3D File"),
                           wxEmptyString, wxEmptyString,
                           wildcard, wxSAVE);
  
  if(file_dialog.ShowModal() != wxID_OK)
    return;
  
  vcl_string filename = static_cast<const char*>(file_dialog.GetPath().mb_str(wxConvUTF8));  
  manager_.save_3d_parts(filename);
}


//=========================================================================
//: save the projected contours as SVG
//=========================================================================
void pca_vehicle_frame::save_svg(wxCommandEvent& event)
{
  wxString wildcard = wxT("SVG (*.svg)|*.svg|"
                          "All Files |*");
  wxFileDialog file_dialog(this, wxT("Save Contours to SVG"),
                           wxEmptyString, wxEmptyString,
                           wildcard, wxSAVE);
  
  if(file_dialog.ShowModal() != wxID_OK)
    return;
  
  vcl_string filename = static_cast<const char*>(file_dialog.GetPath().mb_str(wxConvUTF8));  
  manager_.save_svg(filename);
}


//=========================================================================
//: save an image of the edges
//=========================================================================
void pca_vehicle_frame::save_edge_image(wxCommandEvent& event)
{
  wxString wildcard = wxT("All Images (*.png;*.jpg;*.jpeg;*.tif;*.tiff;*.bmp)|"
                          "*.png;*.jpg;*.jpeg;*.tif;*.tiff;*.bmp|"
                          "PNG (*.png)|*.png|"
                          "JPEG (*.jpg;*.jpeg)|*.jpg;*.jpeg|"
                          "TIFF (*.tif;*.tiff)|*.tif;*.tiff|"
                          "BMP (*.bmp)|*.bmp");
  wxFileDialog file_dialog(this, wxT("Save to edge map image"),
                           wxEmptyString, wxEmptyString,
                           wildcard, wxSAVE);
  
  if(file_dialog.ShowModal() != wxID_OK)
    return;
  
  vcl_string filename = static_cast<const char*>(file_dialog.GetPath().mb_str(wxConvUTF8));  
  manager_.save_edge_image(filename);
}


//=========================================================================
//: save an image of the video
//=========================================================================
void pca_vehicle_frame::save_video_frame(wxCommandEvent& event)
{
  wxString wildcard = wxT("All Images (*.png;*.jpg;*.jpeg;*.tif;*.tiff;*.bmp)|"
                          "*.png;*.jpg;*.jpeg;*.tif;*.tiff;*.bmp|"
                          "PNG (*.png)|*.png|"
                          "JPEG (*.jpg;*.jpeg)|*.jpg;*.jpeg|"
                          "TIFF (*.tif;*.tiff)|*.tif;*.tiff|"
                          "BMP (*.bmp)|*.bmp");
  wxFileDialog file_dialog(this, wxT("Save Video Frame"),
                           wxEmptyString, wxEmptyString,
                           wildcard, wxSAVE);
  
  if(file_dialog.ShowModal() != wxID_OK)
    return;
  
  vcl_string filename = static_cast<const char*>(file_dialog.GetPath().mb_str(wxConvUTF8));  
  manager_.save_video_frame(filename);
}


//=========================================================================
//: save rendered geometry as postscript
//=========================================================================
void pca_vehicle_frame::save_as_ps(wxCommandEvent& event)
{
  gui_utils::render_to_ps(canvas_proj_->get_tableau());
}


//=========================================================================
//: load a mesh
//=========================================================================
void pca_vehicle_frame::load_mesh(wxCommandEvent& event)
{
  wxString wildcard = wxT("Wavefront (*.obj)|*.obj|PLY2 (*.ply2)|*.ply2");
  wxFileDialog file_dialog(this, wxT("Select a Mesh File"),
                           wxEmptyString, wxEmptyString,
                           wildcard, wxOPEN);

  if(file_dialog.ShowModal() != wxID_OK)
    return;

  vcl_string meshfile = static_cast<const char*>(file_dialog.GetPath().mb_str(wxConvUTF8));
  
  // if the current model includes parts we must open a parts file at the same time
  vcl_string partsfile = "";
  if(!manager_.mesh().parts().empty()){
    wxString wildcard2 = wxT("Parts File (*.parts)|*.parts");
    wxFileDialog file_dialog2(this, wxT("Select Parts for the Mesh"),
                              wxEmptyString, wxEmptyString,
                              wildcard2, wxOPEN);
    
    if(file_dialog2.ShowModal() != wxID_OK)
      return;
    
    partsfile = static_cast<const char*>(file_dialog2.GetPath().mb_str(wxConvUTF8));
  }


  if(!manager_.load_mesh(meshfile,partsfile)){
    wxMessageDialog err_dialog( this,
        wxT("Could not load mesh file:\n")+file_dialog.GetPath(),
        wxT("Open File Error"),
        wxOK|wxICON_ERROR);

    err_dialog.ShowModal();
    return;
  }

  if(!manager_.mesh().params().empty())
  {
    const vnl_vector<double>& params = manager_.mesh().params();
    vcl_vector<double> vals(params.size());
    for(unsigned int i=0; i<vals.size(); ++i)
    {
      vals[i] = params[i];
    }
    sliders_->update_data(vals);
  }
}


//=========================================================================
//: load a ground truth mesh
//=========================================================================
void pca_vehicle_frame::load_truth_mesh(wxCommandEvent& event)
{
  wxString wildcard = wxT("Wavefront (*.obj)|*.obj|PLY2 (*.ply2)|*.ply2");
  wxFileDialog file_dialog(this, wxT("Select a Mesh File for Ground Truth"),
                           wxEmptyString, wxEmptyString,
                           wildcard, wxOPEN);
  
  if(file_dialog.ShowModal() != wxID_OK)
    return;
  
  vcl_string filename = static_cast<const char*>(file_dialog.GetPath().mb_str(wxConvUTF8));
  
  
  if(!manager_.load_truth_mesh(filename)){
    wxMessageDialog err_dialog( this,
                               wxT("Could not load mesh file:\n")+file_dialog.GetPath(),
                               wxT("Open File Error"),
                               wxOK|wxICON_ERROR);
    
    err_dialog.ShowModal();
    return;
  }
}


//=========================================================================
//: Vehicle surface parts
//=========================================================================
void pca_vehicle_frame::load_parts(wxCommandEvent& event)
{
  wxString wildcard = wxT("Parts File (*.parts)|*.parts");
  wxFileDialog file_dialog(this, wxT("Select a Parts File"),
                           wxEmptyString, wxEmptyString,
                           wildcard, wxOPEN);

  if(file_dialog.ShowModal() != wxID_OK)
    return;

  vcl_string filename = static_cast<const char*>(file_dialog.GetPath().mb_str(wxConvUTF8));

  if(!manager_.load_parts(filename)){
    wxMessageDialog err_dialog( this,
        wxT("Could not load parts file:\n")+file_dialog.GetPath(),
        wxT("Open File Error"),
        wxOK|wxICON_ERROR);

    err_dialog.ShowModal();
    return;
  }

}


//=========================================================================
//: Vehicle PCA File
//=========================================================================
void pca_vehicle_frame::load_pca(wxCommandEvent& event)
{
  // verify that a mesh has been loaded first
  // probably not needed now that a default mesh is created
  if(!manager_.mesh().is_init())
  {
    wxMessageDialog err_dialog( this,
        wxT("A mesh must be loaded before loading a PCA file\n"
            "Click OK to load a mesh first"),
        wxT("Open File Error"),
        wxOK|wxCANCEL|wxICON_ERROR);

    if(err_dialog.ShowModal() == wxID_OK)
      load_mesh(event);
    else
      return;

    if(!manager_.mesh().is_init())
      return;
  }


  wxString wildcard = wxT("PCA File (*.pca)|*.pca");
  wxFileDialog file_dialog(this, wxT("Select a PCA File"),
                           wxEmptyString, wxEmptyString,
                           wildcard, wxOPEN);

  if(file_dialog.ShowModal() != wxID_OK)
    return;

  vcl_string filename = static_cast<const char*>(file_dialog.GetPath().mb_str(wxConvUTF8));

  if(!manager_.load_pca(filename)){
    wxMessageDialog err_dialog( this,
        wxT("Could not load PCA file:\n")+file_dialog.GetPath(),
        wxT("Open File Error"),
        wxOK|wxICON_ERROR);

    err_dialog.ShowModal();
    return;
  }
  
  init_sliders();
}


//=========================================================================
//: initialize the sliders with the PCA data
//=========================================================================
void pca_vehicle_frame::init_sliders()
{
  const vnl_vector<double>& params = manager_.mesh().params();
  const vnl_vector<double>& std_devs = manager_.mesh().std_devs();
  unsigned int num = params.size();
  vcl_vector<double> min_vals(num,-1), max_vals(num,1), vals(num);
  for(unsigned int i=0; i<num; ++i)
  {
    vals[i] = params[i];
  }
  sliders_->CreateSliders(vals,min_vals,max_vals);
  
  wxSpinCtrl* sc = dynamic_cast<wxSpinCtrl*>(this->FindWindow(wxT("num_to_save_spin")));
  if(sc)
    sc->SetRange(0,num);
}


//=========================================================================
//: scale the camera units
//=========================================================================
void pca_vehicle_frame::scale_camera(wxCommandEvent& event)
{
  // assume "feet to meters" for now
  const double scale = 0.3048;

  manager_.scale_camera(scale);
}


//=========================================================================
//: set the camera to include the transformation of the vehicle
//=========================================================================
void pca_vehicle_frame::set_camera_relative(wxCommandEvent& event)
{
  manager_.set_camera_relative();
}


//=========================================================================
//: set the fitting mode to video or multiview
//=========================================================================
void pca_vehicle_frame::set_mode_video(wxCommandEvent& event)
{
  manager_.set_fit_mode(event.IsChecked());
  wxCheckBox* fa = dynamic_cast<wxCheckBox*>(this->FindWindow(wxT("video_frame_active")));
  fa->Show(!manager_.is_fit_mode_video());
  fa->GetParent()->GetSizer()->Layout();
}


//=========================================================================
//: change the visibility of something
//=========================================================================
void pca_vehicle_frame::show(wxCommandEvent& event)
{
  if(event.GetId() == XRCID("menu_show_edges"))
  {
    manager_.show_edges(event.IsChecked());
  }
  if(event.GetId() == XRCID("menu_show_axes"))
  {
    manager_.show_axes(event.IsChecked());
  }
  if(event.GetId() == XRCID("menu_draw_reprojected"))
  {
    manager_.draw_reprojected(event.IsChecked());
  }
  if(event.GetId() == XRCID("menu_enable_tracking"))
  {
    manager_.enable_tracking(event.IsChecked());
  }
}


//=========================================================================
//: change the visibility of something
//=========================================================================
void pca_vehicle_frame::set_vehicle_model(wxCommandEvent& event)
{
  wxMenuItem *mi1 = this->GetMenuBar()->FindItem(XRCID("menu_use_dodec"));
  wxMenuItem *mi2 = this->GetMenuBar()->FindItem(XRCID("menu_use_ferryman"));
  wxMenuItem *mi3 = this->GetMenuBar()->FindItem(XRCID("menu_use_detailed1"));
  wxMenuItem *mi4 = this->GetMenuBar()->FindItem(XRCID("menu_use_detailed2"));
  wxMenuItem *mi5 = this->GetMenuBar()->FindItem(XRCID("menu_use_detailed3"));
  assert(mi1 && mi2 && mi3);
  
  // uncheck all
  mi1->Check(false);
  mi2->Check(false);
  mi3->Check(false);
  mi4->Check(false);
  mi5->Check(false);
  
  if(event.GetId() == XRCID("menu_use_dodec"))
  {
    manager_.set_vehicle_model(pca_vehicle_manager::DODECAHEDRAL);
    mi1->Check(true);
    init_sliders();
  }
  if(event.GetId() == XRCID("menu_use_ferryman"))
  {
    manager_.set_vehicle_model(pca_vehicle_manager::FERRYMAN);
    mi2->Check(true);
    init_sliders();
  }
  if(event.GetId() == XRCID("menu_use_detailed1"))
  {
    manager_.set_vehicle_model(pca_vehicle_manager::DETAILED1);
    mi3->Check(true);
    init_sliders();
  }
  if(event.GetId() == XRCID("menu_use_detailed2"))
  {
    manager_.set_vehicle_model(pca_vehicle_manager::DETAILED2);
    mi4->Check(true);
    init_sliders();
  }
  if(event.GetId() == XRCID("menu_use_detailed3"))
  {
    manager_.set_vehicle_model(pca_vehicle_manager::DETAILED3);
    mi5->Check(true);
    init_sliders();
  }
}


//=========================================================================
//: draw something
//=========================================================================
void pca_vehicle_frame::draw(wxCommandEvent& event)
{
  if(event.GetId() == XRCID("menu_draw_matches"))
  {
    manager_.draw_matches();
  }
  if(event.GetId() == XRCID("menu_draw_jacobians"))
  {
    manager_.draw_jacobians();
  }
  if(event.GetId() == XRCID("menu_draw_sil_matches"))
  {
    manager_.draw_silhouette_matches();
  }
}


//=========================================================================
//: compute the RMS error between the pca mesh and ground truth 
//=========================================================================
void pca_vehicle_frame::compute_error(wxCommandEvent& event)
{
  manager_.compute_error();
}


//=========================================================================
//: Start/Stop capture of a video from one of the views
//=========================================================================
void pca_vehicle_frame::capture_view(wxCommandEvent& event)
{
  if(event.GetId() == XRCID("menu_capture_3d_view"))
  {
    manager_.capture_view(1,event.IsChecked());
  }
  else if(event.GetId() == XRCID("menu_capture_proj_view"))
  {
    manager_.capture_view(2,event.IsChecked());
  }
}


//=========================================================================
//: Run an animation script
//=========================================================================
void pca_vehicle_frame::run_animation(wxCommandEvent& event)
{
  wxString wildcard = wxT("Animation Script (*.txt)|*.txt|"
                          "All Files |*");
  wxFileDialog file_dialog(this, wxT("Select an Animation Script File"),
                           wxEmptyString, wxEmptyString,
                           wildcard, wxOPEN);
  
  if(file_dialog.ShowModal() != wxID_OK)
    return;
  
  vcl_string filename = static_cast<const char*>(file_dialog.GetPath().mb_str(wxConvUTF8));
  
  manager_.run_animation(filename);
  const vnl_vector<double>& params = manager_.mesh().params();
  vcl_vector<double> vals(params.size());
  for(unsigned int i=0; i<vals.size(); ++i)
    vals[i] = params[i];
  
  // refresh the translation and rotation parts
  refresh_values();
  
  Freeze();
  sliders_->update_data(vals,false);
  Thaw();
}


//=========================================================================
//: Run an stored tracking result
//=========================================================================
void pca_vehicle_frame::run_track_results(wxCommandEvent& event)
{
  wxString wildcard = wxT("Tracking Result (*.txt)|*.txt|"
                          "All Files |*");
  wxFileDialog file_dialog(this, wxT("Select an Tracking Result File"),
                           wxEmptyString, wxEmptyString,
                           wildcard, wxOPEN);
  
  if(file_dialog.ShowModal() != wxID_OK)
    return;
  
  vcl_string filename = static_cast<const char*>(file_dialog.GetPath().mb_str(wxConvUTF8));
  
  vcl_string vid_file, cam_file, model_type;
  vcl_map<unsigned int,vcl_vector<modrec_vehicle_state> > state_map;
  unsigned int start_frame = modrec_read_track_file(filename,vid_file,cam_file,model_type,state_map);
  
  // load video file
  vidl_istream_sptr is = new vidl_ffmpeg_istream(vid_file);
  if(!is){
    vcl_cerr<< "could not open video file: "<<vid_file<<vcl_endl;
    return;
  }
  manager_.set_fit_mode(true);
  
  // need to get num_frames before the manager "opens" the stream
  // because num_frames() is only valid when called before advance.
  int num_frames = -1;
  if(is && is->is_seekable()){
    double duration = is->duration();
    double frame_rate = is->frame_rate();
    int est_num_frames = static_cast<int>(duration*frame_rate +0.5);
    if (est_num_frames > 0)
      num_frames = est_num_frames;
    else // count the frames
      num_frames = is->num_frames();
  }
  if(manager_.open_istream(is)){
    manager_.video_seek(start_frame);
    video_control_->set_num_frames(num_frames);
    video_control_->set_frame(start_frame,false);
    video_panel_->Show(true);
    video_panel_->GetParent()->Layout();
  }

  
  // load camera file
  if(!manager_.load_camera(cam_file))
  {
    wxMessageDialog err_dialog( this,
                               wxT("Perspective Decomposition Failed"),
                               wxT("Open File Error"),
                               wxOK|wxICON_ERROR);
    
    err_dialog.ShowModal();
  }
  
  if(model_type == "Dodecahedral")
    manager_.set_vehicle_model(pca_vehicle_manager::DODECAHEDRAL);
  else if(model_type == "Ferryman")
    manager_.set_vehicle_model(pca_vehicle_manager::FERRYMAN);
  else if(model_type == "Detailed1")
    manager_.set_vehicle_model(pca_vehicle_manager::DETAILED1);
  else if(model_type == "Detailed2")
    manager_.set_vehicle_model(pca_vehicle_manager::DETAILED2);
  else if(model_type == "Detailed3")
    manager_.set_vehicle_model(pca_vehicle_manager::DETAILED3);
  else{
    vcl_cerr << "unknown model type: "<<model_type<<vcl_endl;
    return;
  }
  
  manager_.enable_replay_mode(true);
  manager_.set_state_map(state_map);
  //manager_.run_track_results(filename);
}


//=========================================================================
//: Reset the PCA parameters to zero
//=========================================================================
void pca_vehicle_frame::clear_pca(wxCommandEvent& event)
{
  const vnl_vector<double>& params = manager_.mesh().params();
  unsigned int num = params.size();
  if(num == 0)
    return;

  vcl_vector<double> vals(num,0.0);

  wxSpinCtrl* sc = dynamic_cast<wxSpinCtrl*>(this->FindWindow(wxT("num_to_save_spin")));
  if(sc)
  {
    int num_save = sc->GetValue();
    for(int i=0; i<num_save; ++i){
      vals[i] = params[i];
    }
  }

  manager_.change_mesh_params(vals);
  Freeze();
  sliders_->update_data(vals,false);
  Thaw();
}


void pca_vehicle_frame::fit_model(wxCommandEvent& event)
{

  wxSpinCtrl* sc = dynamic_cast<wxSpinCtrl*>(this->FindWindow(wxT("num_itr_spin")));
  assert(sc);
  unsigned int num_itr = sc->GetValue();
  
  vul_timer t;
  manager_.fit_model(num_itr);
  vcl_cout << "fit time "<< t.user() << vcl_endl;
  t.mark();
  
  // refresh all the values
  refresh_values();

  
  manager_.update_all_displays();
  
  vcl_cout << "redraw time "<< t.user() << vcl_endl;
}


//=========================================================================
//: The interaction mode has changed
//=========================================================================
void pca_vehicle_frame::change_imode(wxCommandEvent& event)
{
  if(event.GetId() == XRCID("radio_interaction_none"))
  {
    manager_.set_interaction_mode(pca_vehicle_manager::NONE);
  }
  else if(event.GetId() == XRCID("radio_interaction_move"))
  {
    manager_.set_interaction_mode(pca_vehicle_manager::MOVE);
  }
}

//: Handle vgui messages
void pca_vehicle_frame::handle_message(const vgui_message& m)
{
  if(manager_.handle_message(m))
    return;

  const wxSliderPanel* p = static_cast<const wxSliderPanel*>(m.from);
  int i=-1;
  if(m.data)
    i = *static_cast<const int*>(m.data);

  if(m.user == wxSliderPanel::enter){
    manager_.change_mesh_params(p->data());
  }
  else if(m.user == wxSliderPanel::update){
    manager_.update_mesh_param(i,p->data()[i]);
  }
  else if(m.user == wxVideoControl::m_seek){
    vcl_cout << "video control seek " << i << vcl_endl;
    manager_.video_seek(i);
    if(wxCheckBox* ckbox = dynamic_cast<wxCheckBox*>(this->FindWindow(wxT("video_frame_active"))))
    {
      ckbox->SetValue(manager_.is_current_frame_active());
    }
  }
  else if(m.user == wxVideoControl::m_preview){
    vcl_cout << "video control preview " << i << vcl_endl;
  }
  else if(m.user == wxVideoControl::m_next){
    vcl_cout << "video control next " << i << vcl_endl;
    manager_.video_seek(i);
    if(wxCheckBox* ckbox = dynamic_cast<wxCheckBox*>(this->FindWindow(wxT("video_frame_active"))))
    {
      ckbox->SetValue(manager_.is_current_frame_active());
    }
  }
  else if(m.user == wxVideoControl::m_prev){
    vcl_cout << "video control prev " << i << vcl_endl;
    manager_.video_seek(i);
    if(wxCheckBox* ckbox = dynamic_cast<wxCheckBox*>(this->FindWindow(wxT("video_frame_active"))))
    {
      ckbox->SetValue(manager_.is_current_frame_active());
    }
  }
  else if(m.user == wxVideoControl::m_play){
    vcl_cout << "video control play " << i << vcl_endl;
    bool video_valid = true;
    while(video_control_->is_playing() && (video_valid = manager_.advance_video())){
      video_control_->set_frame(manager_.current_frame(),false);
      vgui::run_till_idle();
    }
    if(!video_valid)
      video_control_->pause();
  }
  else if(m.user == wxVideoControl::m_pause){
    vcl_cout << "video control pause " << i << vcl_endl;
  }
}


//: Callback for changed interaction mode
void pca_vehicle_frame::interaction_mode_changed()
{
  pca_vehicle_manager::interact_mode mode = manager_.interaction_mode();
  wxRadioButton* rb = NULL;
  switch(mode)
  {
    case pca_vehicle_manager::NONE:
      rb = dynamic_cast<wxRadioButton*>(this->FindWindow(wxT("radio_interaction_none")));
      break;
    case pca_vehicle_manager::MOVE:
      rb = dynamic_cast<wxRadioButton*>(this->FindWindow(wxT("radio_interaction_move")));
      break;
  }
  if(rb)
    rb->SetValue(true);
}


//: Refresh the slider and text box values
void pca_vehicle_frame::refresh_values()
{
  const vnl_vector<double>& params = manager_.mesh().params();
  vcl_vector<double> vals(params.size());
  for(unsigned int i=0; i<vals.size(); ++i)
    vals[i] = params[i];
  
  Freeze();
  sliders_->update_data(vals,false);
  Thaw();
  
  vgl_vector_3d<double> t = manager_.translation();
  wxTextCtrl* text = dynamic_cast<wxTextCtrl*>(this->FindWindow(wxT("tx_text")));
  text->SetValue(wxString::Format(wxT("%g"),t.x()));
  text = dynamic_cast<wxTextCtrl*>(this->FindWindow(wxT("ty_text")));
  text->SetValue(wxString::Format(wxT("%g"),t.y()));
  text = dynamic_cast<wxTextCtrl*>(this->FindWindow(wxT("tz_text")));
  text->SetValue(wxString::Format(wxT("%g"),t.z()));
  
  vnl_vector_fixed<double,3> r = manager_.rotation().as_rodrigues();
  text = dynamic_cast<wxTextCtrl*>(this->FindWindow(wxT("rx_text")));
  text->SetValue(wxString::Format(wxT("%g"),r[0]));
  text = dynamic_cast<wxTextCtrl*>(this->FindWindow(wxT("ry_text")));
  text->SetValue(wxString::Format(wxT("%g"),r[1]));
  text = dynamic_cast<wxTextCtrl*>(this->FindWindow(wxT("rz_text")));
  text->SetValue(wxString::Format(wxT("%g"),r[2]));
}


//: Update the manager with the current fit options
void pca_vehicle_frame::update_fit_options()
{
  // determine which parameters are enabled for optimization
  wxCheckBox* cb;
  vcl_vector<bool> options(7,false);
  cb = dynamic_cast<wxCheckBox*>(this->FindWindow(wxT("pca_checkbox")));
  assert(cb);
  options[0] = cb->IsChecked();
  cb = dynamic_cast<wxCheckBox*>(this->FindWindow(wxT("tx_checkbox")));
  assert(cb);
  options[1] = cb->IsChecked();
  cb = dynamic_cast<wxCheckBox*>(this->FindWindow(wxT("ty_checkbox")));
  assert(cb);
  options[2] = cb->IsChecked();
  cb = dynamic_cast<wxCheckBox*>(this->FindWindow(wxT("tz_checkbox")));
  assert(cb);
  options[3] = cb->IsChecked();
  cb = dynamic_cast<wxCheckBox*>(this->FindWindow(wxT("rx_checkbox")));
  assert(cb);
  options[4] = cb->IsChecked();
  cb = dynamic_cast<wxCheckBox*>(this->FindWindow(wxT("ry_checkbox")));
  assert(cb);
  options[5] = cb->IsChecked();
  cb = dynamic_cast<wxCheckBox*>(this->FindWindow(wxT("rz_checkbox")));
  assert(cb);
  options[6] = cb->IsChecked();
  wxSpinCtrl* sc = dynamic_cast<wxSpinCtrl*>(this->FindWindow(wxT("num_to_save_spin")));
  assert(sc);
  unsigned int num_pc = sc->GetValue();
  
  wxTextCtrl* ltext = dynamic_cast<wxTextCtrl*>(this->FindWindow(wxT("lambda_text")));
  assert(ltext);
  double lambda;
  ltext->GetValue().ToDouble(&lambda);
  
  wxTextCtrl* stext = dynamic_cast<wxTextCtrl*>(this->FindWindow(wxT("edge_scale_text")));
  assert(stext);
  double edge_scale;
  stext->GetValue().ToDouble(&edge_scale);
  
  manager_.set_fit_options(options, num_pc, lambda, edge_scale);
}



//=========================================================================
//: Activate or deactivate the current frame for optimization 
//=========================================================================
void pca_vehicle_frame::active_frame(wxCommandEvent& event)
{
  manager_.set_frame_active(event.IsChecked());
}


//=========================================================================
//: handle global keyboard shortcuts
//=========================================================================
void pca_vehicle_frame::keyboard_commands(wxKeyEvent& event)
{
  vcl_cout << "Found key pressed"<<vcl_endl;
  switch(event.GetKeyCode())
  {
    case WXK_LEFT:
      vcl_cout << "Left key pressed"<<vcl_endl;
      break;
    case WXK_RIGHT:
      vcl_cout << "Right key pressed"<<vcl_endl;
      break;
    defualt:
      event.Skip();
  }
}


//=========================================================================
//: Toggle the use of each extrinsic parameter
//=========================================================================
void pca_vehicle_frame::toggle_extrinsics(wxCommandEvent& event)
{
  wxCheckBox* box = dynamic_cast<wxCheckBox*>(event.GetEventObject());
  wxTextCtrl* text = NULL;

  if(event.GetId() == XRCID("tx_checkbox"))
    text = dynamic_cast<wxTextCtrl*>(this->FindWindow(wxT("tx_text")));
  else if(event.GetId() == XRCID("ty_checkbox"))
    text = dynamic_cast<wxTextCtrl*>(this->FindWindow(wxT("ty_text")));
  else if(event.GetId() == XRCID("tz_checkbox"))
    text = dynamic_cast<wxTextCtrl*>(this->FindWindow(wxT("tz_text")));
  else if(event.GetId() == XRCID("rx_checkbox"))
    text = dynamic_cast<wxTextCtrl*>(this->FindWindow(wxT("rx_text")));
  else if(event.GetId() == XRCID("ry_checkbox"))
    text = dynamic_cast<wxTextCtrl*>(this->FindWindow(wxT("ry_text")));
  else if(event.GetId() == XRCID("rz_checkbox"))
    text = dynamic_cast<wxTextCtrl*>(this->FindWindow(wxT("rz_text")));

  if(text)
    text->Enable(event.IsChecked());
  
  update_fit_options();
}


//=========================================================================
//: Callback on the update of fitting options
//=========================================================================
void pca_vehicle_frame::change_spin_options(wxSpinEvent& event)
{
  update_fit_options();
}

void pca_vehicle_frame::change_options(wxCommandEvent& event)
{
  update_fit_options();
}


//=========================================================================
//: Callback on the update of translation parameters
//=========================================================================
void pca_vehicle_frame::change_translation(wxCommandEvent& event)
{
  const vgl_vector_3d<double>& t = manager_.translation();
  double x=t.x(),y=t.y(),z=t.z();
  if(wxTextCtrl* text = dynamic_cast<wxTextCtrl*>(this->FindWindow(wxT("tx_text"))))
    text->GetValue().ToDouble(&x);
  if(wxTextCtrl* text = dynamic_cast<wxTextCtrl*>(this->FindWindow(wxT("ty_text"))))
    text->GetValue().ToDouble(&y);
  if(wxTextCtrl* text = dynamic_cast<wxTextCtrl*>(this->FindWindow(wxT("tz_text"))))
    text->GetValue().ToDouble(&z);

  manager_.set_translation(vgl_vector_3d<double>(x,y,z));

  event.Skip();
}


//=========================================================================
//: Callback on the update of rotation parameters
//=========================================================================
void pca_vehicle_frame::change_rotation(wxCommandEvent& event)
{
  vnl_vector_fixed<double,3> r = manager_.rotation().as_rodrigues();
  if(wxTextCtrl* text = dynamic_cast<wxTextCtrl*>(this->FindWindow(wxT("rx_text"))))
    text->GetValue().ToDouble(&r[0]);
  if(wxTextCtrl* text = dynamic_cast<wxTextCtrl*>(this->FindWindow(wxT("ry_text"))))
    text->GetValue().ToDouble(&r[1]);
  if(wxTextCtrl* text = dynamic_cast<wxTextCtrl*>(this->FindWindow(wxT("rz_text"))))
    text->GetValue().ToDouble(&r[2]);

  manager_.set_rotation(vgl_rotation_3d<double>(r));

  event.Skip();
}


//=========================================================================
//: Callback on the update of translation parameters
//=========================================================================
void pca_vehicle_frame::update_projection(wxCommandEvent& event)
{
  manager_.update_projection();
  event.Skip();
}


