// This is spl/vis/spl_basic_gui_observers.h
#ifndef spl_basic_gui_observers_h_
#define spl_basic_gui_observers_h_
//:
// \file
// \brief  Commonly used spl_observers that update vgui_tableau
// \author Matthew Leotta (mleotta@brown.lems.edu)
// \date   April 22, 2009
//
//          Copyright Matthew Leotta 2006 - 2010.
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file ../../LICENSE_1_0.txt or copy at
//          http://www.boost.org/LICENSE_1_0.txt)
//
// \verbatim
//  Modifications
// \endverbatim

#include <spl/spl_observer.h>
#include <vgui/vgui_image_tableau.h>
#include <vil/vil_image_resource.h>

//: update a vgui_image_tableau with a vil_image_resource_sptr
class spl_image_observer: public spl_observer
{
public:
  spl_image_observer(const vgui_image_tableau_sptr& itab)
  : image_tab(itab) {}
  
  //: Called by the process when the data is ready
  virtual bool notify(const spl_storage_sptr& data, unsigned long timestamp)
  {
    assert(image_tab);
    assert(data);
    if(data->info() != SPL_VALID)
      image_tab->set_image_resource(NULL);
    else{
      assert(data->type_id() == typeid(vil_image_resource_sptr));
      image_tab->set_image_resource(data->data<vil_image_resource_sptr>());
    }
    image_tab->post_redraw();
    return true;
  }
  vgui_image_tableau_sptr image_tab;
};


#endif // spl_basic_gui_observers_h_
