// This is vpro/vis/vpro_run_tableau.cxx
#include <vpro/vis/vpro_run_tableau.h>
//:
// \file
// \author Matthew Leotta
// \date   July 14, 2006
// \brief  See vpro_run_tableau.h for a description of this file.
//
//          Copyright Matthew Leotta 2006 - 2010.
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file ../../LICENSE_1_0.txt or copy at
//          http://www.boost.org/LICENSE_1_0.txt)
//


#include <vcl_iostream.h>
#include <vcl_sstream.h>
#include <vgui/vgui.h>
#include <vgui/vgui_gl.h>
#include <vgui/vgui_command.h>
#include <vgui/vgui_menu.h>
#include <vgui/vgui_text_put.h>

//----------------------------------------------------------------------------
//: Constructor - don't use this, use vpro_run_tableau_new.
vpro_run_tableau::vpro_run_tableau(const vpro_executive& g)
  : graph_(g), fps_(0.0), display_(false), running_(false)
{
  graph_.init();
}


//----------------------------------------------------------------------------
//: Handle events
bool vpro_run_tableau::handle(const vgui_event& e)
{
  if (display_ && e.type == vgui_DRAW)
  {
    glDisable(GL_CULL_FACE);
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_TEXTURE_2D);
    glDisable(GL_LIGHTING);
    glShadeModel(GL_FLAT);

    GLint vp[4];
    glGetIntegerv(GL_VIEWPORT, vp);
    int width  = vp[2];
    int height = vp[3];

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(0, width,  // left, right
            height, 0, // bottom, top
            -1,+1);    // near, far
  
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    glColor3f(1.0f,0.0f,0.0f);
    glRasterPos2f(10.0f,20.0f);

    vcl_stringstream out;
    out << "FPS: "<< fps_;
    vgui_text_put(out.str().c_str());

    return true;
  }

  return vgui_tableau::handle(e);
}


//----------------------------------------------------------------------------
//: Handle keypress
bool vpro_run_tableau::key_press (int x, int y, vgui_key key, vgui_modifier)
{
  switch(key){
    case ' ':
      enable_idle_once();
      return true;
    case 'p':
      enable_idle();
      return true;
    case 's':
      running_ = false;
      return true;
    default:
      break;
  }
  return false;
}


//----------------------------------------------------------------------------
//: Enable idle events (start the processing)
void vpro_run_tableau::enable_idle()
{
  this->post_idle_request();
  running_ = true;
}


//----------------------------------------------------------------------------
//: Enable idle events for one iteration
void vpro_run_tableau::enable_idle_once()
{
  run_step();
  running_ = false;
}


//----------------------------------------------------------------------------
//: Toggle the text display
void vpro_run_tableau::toggle_display()
{
  this->display_ = !this->display_;
}


//----------------------------------------------------------------------------
//: Builds a popup menu
void vpro_run_tableau::get_popup(const vgui_popup_params& params,
                                        vgui_menu &menu)
{
  vgui_menu submenu;
  submenu.add("start processing",new vgui_command_simple<vpro_run_tableau>(this,&vpro_run_tableau::enable_idle));
  submenu.add("process once",new vgui_command_simple<vpro_run_tableau>(this,&vpro_run_tableau::enable_idle_once));
  submenu.add("toggle display",new vgui_command_simple<vpro_run_tableau>(this,&vpro_run_tableau::toggle_display));
  menu.add("Process Control",submenu);

}


//----------------------------------------------------------------------------
//: Handle idle events
bool vpro_run_tableau::idle()
{
  if((++count_)%10 == 0){
    fps_ = 10000.0/time_.real();
    time_.mark();
  }

  if(running_){
    running_ = run_step() && running_;
    return running_;
  }

  return false;
}

//----------------------------------------------------------------------------
//: Run one step of the processing
bool vpro_run_tableau::run_step()
{
  static bool semaphore = false;
  if(semaphore)
    return true;
  semaphore = true;
  vpro_signal s = graph_.run_step();
  vgui::run_till_idle();
  semaphore = false;
  assert(s != VPRO_INVALID);
  return (s == VPRO_VALID || s == VPRO_WAIT);
}


