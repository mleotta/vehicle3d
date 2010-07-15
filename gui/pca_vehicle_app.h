// This is mleotta/gui/pca_vehicle/pca_vehicle_app.h
#ifndef pca_vehicle_app_h_
#define pca_vehicle_app_h_
//=========================================================================
//:
// \file
// \brief  wxWidgets main application class.
// \author Matt Leotta (mleotta)
//
//=========================================================================

#include <wx/app.h>

class pca_vehicle_frame;

//-------------------------------------------------------------------------
//: The wxWidgets main application class.
//-------------------------------------------------------------------------
class pca_vehicle_app : public wxApp
{
 public:
  //: Constructor - default.
  pca_vehicle_app();

  //: Destructor.
  virtual ~pca_vehicle_app() {}
  
  //: Called on app initialization.
  virtual bool OnInit();

  //: Provides access to this Apps main frame.
  pca_vehicle_frame* get_main_frame() const { return frame_; }

 private:
  pca_vehicle_frame*  frame_;
};

//: Implements pca_vehicle_app& wxGetApp().
DECLARE_APP(pca_vehicle_app)

#endif // pca_vehicle_app_h_
