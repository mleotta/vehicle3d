// This is basic/dbgui/dbgui_gnd_cal_tableau.cxx
#include "gnd_cal_tableau.h"
//:
// \file

#include <vgui/vgui_gl.h>
#include <vgui/vgui_menu.h>
#include <vgui/vgui_command.h>
#include <vgui/vgui_projection_inspector.h>
#include <vgui/vgui_dialog.h>
#include <vgui/vgui_message.h>

#include <vpgl/algo/vpgl_camera_compute.h>


//: Constructor
dbgui_gnd_cal_tableau::dbgui_gnd_cal_tableau(const vpgl_perspective_camera<double>& cam)
  : mode_(NONE), draw_calib_(false), cnr_idx_(0),
    gesture0(vgui_LEFT, vgui_MODIFIER_NULL, true ), // press
    gesture1(vgui_LEFT, vgui_MODIFIER_NULL, false), // release
    camera_(cam)
{
}


//----------------------------------------------------------------------------
//: Return the name of this tableau ('dbgui_gnd_cal_tableau').
vcl_string dbgui_gnd_cal_tableau::type_name() const
{
  return "dbgui_gnd_cal_tableau";
}


//----------------------------------------------------------------------------
//: Add option to the popup menu
void dbgui_gnd_cal_tableau::add_popup(vgui_menu& menu)
{
  vgui_menu popup;

  popup.add("World Box",
            new vgui_command_simple<dbgui_gnd_cal_tableau>(this,
                   &dbgui_gnd_cal_tableau::start_calib));

  vcl_string name = "Draw Calibration";
  name = (draw_calib_?"[X] ":"[ ] ") + name;
  popup.add(name.c_str(),
            new vgui_command_simple<dbgui_gnd_cal_tableau>(this,
                   &dbgui_gnd_cal_tableau::toggle_draw));

  menu.add( "Ground Calibrate", popup );
}


//----------------------------------------------------------------------------
//: Start interaction for world plane calibration
void dbgui_gnd_cal_tableau::start_calib()
{
  mode_ = SELECT;
  cnr_idx_ = 0;
  draw_calib_ = true;
}


//----------------------------------------------------------------------------
//: Finish the calibration
void dbgui_gnd_cal_tableau::finish_calib()
{
  vpgl_calibration_matrix<double> K = camera_.get_calibration();

  static double width = 1.0;
  static double height = 1.0;
  double focal = K.focal_length();
  double xc = K.principal_point().x();
  double yc = K.principal_point().y();
  double xs = K.x_scale();
  double ys = K.y_scale();
  double skew = K.skew();
  vgui_dialog dim_dlg("Plane Dimensions");
  dim_dlg.field("Width (world units)", width);
  dim_dlg.field("Height (world units)", height);
  dim_dlg.message("Intrinsic Camera Parameters");
  dim_dlg.field("focal length (pixels)", focal);
  dim_dlg.field("Principal Point X (pixels)", xc);
  dim_dlg.field("Principal Point Y (pixels)", yc);
  dim_dlg.field("X scale", xs);
  dim_dlg.field("Y scale", ys);
  dim_dlg.field("Skew", skew);
  if(!dim_dlg.ask()){
    mode_ = NONE;
    cnr_idx_ = 0;
    return;
  }

  K.set_focal_length(focal);
  K.set_principal_point(vgl_point_2d<double>(xc,yc));
  K.set_x_scale(xs);
  K.set_y_scale(ys);
  K.set_skew(skew);
  camera_.set_calibration(K);
  double world_pts[4][2] = {{0,0}, {width,0}, {width,height}, {0,height} };


  vcl_vector< vgl_point_2d<double> > image_pts, ground_pts;
  for(unsigned int i=0; i<4; ++i){
    image_pts.push_back(vgl_point_2d<double>(corners_[i][0],corners_[i][1]));
    ground_pts.push_back(vgl_point_2d<double>(world_pts[i][0],world_pts[i][1]));
  }
  if(!vpgl_perspective_camera_compute::compute(image_pts, ground_pts, camera_))
    vcl_cout << "camera computation failed" << vcl_endl;

  vcl_cout << "Camera = "<< camera_ << vcl_endl;

  if(mode_==SELECT){
    world_pts_.clear();
    for(unsigned i=0; i<4; ++i){
      vgl_point_3d<double> w(world_pts[i][0],world_pts[i][1],0.0);
      world_pts_.push_back(w);
    }
  }

  // notify observers
  vgui_message m;
  m.from = this;
  notify(m);

  mode_ = NONE;
  cnr_idx_ = 0;
  post_redraw();
}


//----------------------------------------------------------------------------
//: Handle all events sent to this tableau.
bool dbgui_gnd_cal_tableau::handle(const vgui_event& e)
{
  if(e.type == vgui_DRAW && draw_calib_){
    glColor3f(1,0,0);
    glBegin(GL_LINE_LOOP);
      for(unsigned int i=0; i<world_pts_.size(); ++i){
        vgl_point_2d<double> pt = camera_.project(world_pts_[i]);
        glVertex2d(pt.x(),pt.y());
      }
    glEnd();
  }

  if(mode_ != NONE){

    float ix, iy;
    vgui_projection_inspector().window_to_image_coordinates(e.wx, e.wy, ix, iy);

    if(e.type == vgui_DRAW_OVERLAY){
    glColor3f(0,1,0);
      glBegin(GL_LINE_STRIP);
        for(unsigned int i=0; i<cnr_idx_; ++i)
          glVertex2d(corners_[i][0],corners_[i][1]);
        glVertex2f(last_x,last_y);
      glEnd();

      glBegin(GL_POINTS);
      for(unsigned int i=0; i<cnr_idx_; ++i)
        glVertex2f(float(corners_[i][0]),float(corners_[i][1]));
      glEnd(); 
    }

    if(e.type == vgui_MOUSE_MOTION){
      last_x = ix;
      last_y = iy;
      post_overlay_redraw();
    }

    if(gesture1(e)){
      corners_[cnr_idx_][0] = ix;
      corners_[cnr_idx_][1] = iy;
      ++cnr_idx_;
      if(cnr_idx_ > 3)
        finish_calib();
      post_overlay_redraw();
    }
  }
  return true;
}

