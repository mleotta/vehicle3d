// This is gui/gnd_cal_tableau.h
#ifndef dbgui_gnd_cal_tableau_h_
#define dbgui_gnd_cal_tableau_h_

//:
// \file
// \brief tableau for calibrating camera extrinsics with a world ground plane
// \author Matt Leotta (mleotta@lems.brown.edu)
// \date 2/27/07
//
//          Copyright Matthew Leotta 2006 - 2010.
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file ../LICENSE_1_0.txt or copy at
//          http://www.boost.org/LICENSE_1_0.txt)
//
// \verbatim
//  Modifications
// \endverbatim


#include <vgui/vgui_tableau.h>
#include <vgui/vgui_observable.h>
#include <vgui/vgui_event_condition.h>
#include <vgl/vgl_point_3d.h>
#include <vpgl/vpgl_perspective_camera.h>

#include "gnd_cal_tableau_sptr.h"



//: tableau for calibrating camera extrinsics with a world ground plane
class dbgui_gnd_cal_tableau : public vgui_tableau, public vgui_observable
{
  public:
    //: Constructor
    dbgui_gnd_cal_tableau(const vpgl_perspective_camera<double>& cam);

    //: Return the name of this tableau ('dbgui_gnd_cal_tableau').
    vcl_string type_name() const;

    //: Add option to the popup menu.
    void add_popup(vgui_menu&);

    //: Start interaction for world plane calibration
    void start_calib();

    //: Toggle drawing
    void toggle_draw() { draw_calib_ = !draw_calib_; }

    //: Handle all events sent to this tableau.
    bool handle(const vgui_event&);

    //: Return the current camera estimate
    const vpgl_perspective_camera<double>& camera() const { return camera_; }

    //: Set a camera (intrinsics are take from this)
    void set_camera(const vpgl_perspective_camera<double>& cam) { camera_ = cam; }

  protected:
    //: Finish the calibration
    void finish_calib();

  private:
    enum calib_mode {NONE, SELECT,};
    calib_mode mode_;

    bool draw_calib_;

    double corners_[4][2];
    unsigned int cnr_idx_;

    vgui_event_condition gesture0, gesture1;
    float last_x, last_y;

    vcl_vector<vgl_point_3d<double> > world_pts_;

    unsigned int min_x_;
    unsigned int max_x_;
    unsigned int min_y_;
    unsigned int max_y_;

    vpgl_perspective_camera<double> camera_;
};


//: Create a smart-pointer to a dbgui_gnd_cal_tableau tableau.
struct dbgui_gnd_cal_tableau_new : public dbgui_gnd_cal_tableau_sptr
{
  typedef dbgui_gnd_cal_tableau_sptr base;

  //: Constructor - make a tableau 
  dbgui_gnd_cal_tableau_new(const vpgl_perspective_camera<double>& cam)
    : base(new dbgui_gnd_cal_tableau(cam)) { }
};



#endif // dbgui_gnd_cal_tableau_h_
