// This is mleotta/gui/pca_vehicle/pca_vehicle_app.cxx
//=========================================================================
//:
// \file
// \brief  wxWidgets main application class.
//
// See pca_vehicle_app.h for details.
//=========================================================================

#include "pca_vehicle_app.h"
#include "pca_vehicle_frame.h"

#include <vgui/wx/vgui_wx.h>
#include <wx/xrc/xmlres.h>
#include <vgui/vgui.h>
#include <bgui3d/bgui3d.h>
#include <vul/vul_file.h>

#if defined(__APPLE__)
#include <CFBundle.h>
#endif


extern void InitXmlResource(); // defined in generated file


//: Give wxWidgets the means to create a pca_vehicle_app object.
IMPLEMENT_APP(pca_vehicle_app)

pca_vehicle_app::pca_vehicle_app(void)
  : frame_(0)
{
}


#if defined(__APPLE__)
//: convert a CFRUL to a vcl_string and release the reference 
vcl_string CFURLRef_to_string(const CFURLRef& url)
{
  CFStringRef str = CFURLCopyFileSystemPath( url, kCFURLPOSIXPathStyle );
  CFIndex size = CFStringGetMaximumSizeForEncoding(CFStringGetLength(str), 
                                                   kCFStringEncodingUTF8);
  char* s = new char[size+1];
  s[size] = 0;
  CFStringGetCString(str, s, size, kCFStringEncodingASCII );
  vcl_string url_str(s);
  CFRelease(str);
  CFRelease(url);
  return url_str;
}
#endif


//: Initialize the application.
bool pca_vehicle_app::OnInit(void)
{
  // select the vgui_wx toolkit
  vgui::select("wx");

  // initialize bgui_3d
  bgui3d_init();

  vgui_wx::InitVguiHandlers();
  wxXmlResource::Get()->InitAllHandlers();
  InitXmlResource();

  frame_ = new pca_vehicle_frame(0,
                            wxID_ANY,
                            wxT("PCA Vehicle GUI"),
                            wxPoint(0, 0),
                            wxSize(1000, 800),
                            wxDEFAULT_FRAME_STYLE);
  
  // try to load the default data files
  vcl_string path = "";
#if defined(__APPLE__) 
  // on OS X, look for files in the bundle resources directory
  CFBundleRef mainBundle = CFBundleGetMainBundle();
  path = CFURLRef_to_string(CFBundleCopyBundleURL(mainBundle)) + "/";
  path += CFURLRef_to_string(CFBundleCopyResourcesDirectoryURL(mainBundle)) + "/";
#endif

  
  if(vul_file::exists(path+"default_dodec.pca")){
    frame_->manager_.set_vehicle_model(pca_vehicle_manager::DODECAHEDRAL);
    if(frame_->manager_.load_pca(path+"default_dodec.pca")){
      unsigned int num = frame_->manager_.mesh().params().size();
      vcl_vector<double> vals(num,0.0);
      frame_->manager_.change_mesh_params(vals);
    }
  }
  
  if(vul_file::exists(path+"default_ferryman.pca")){
    frame_->manager_.set_vehicle_model(pca_vehicle_manager::FERRYMAN);
    if(frame_->manager_.load_pca(path+"default_ferryman.pca")){
      unsigned int num = frame_->manager_.mesh().params().size();
      vcl_vector<double> vals(num,0.0);
      frame_->manager_.change_mesh_params(vals);
    }
  }
  
  frame_->manager_.set_vehicle_model(pca_vehicle_manager::DETAILED1);
  if(vul_file::exists(path+"default.parts") &&
     frame_->manager_.load_parts(path+"default.parts")){
    if(vul_file::exists(path+"default1.pca") &&
       frame_->manager_.load_pca(path+"default1.pca")){
      unsigned int num = frame_->manager_.mesh().params().size();
      vcl_vector<double> vals(num,0.0);
      frame_->manager_.change_mesh_params(vals);
    }
  }
  
  frame_->manager_.set_vehicle_model(pca_vehicle_manager::DETAILED2);
  if(vul_file::exists(path+"default.parts") &&
     frame_->manager_.load_parts(path+"default.parts")){
    if(vul_file::exists(path+"default2.pca") &&
       frame_->manager_.load_pca(path+"default2.pca")){
      unsigned int num = frame_->manager_.mesh().params().size();
      vcl_vector<double> vals(num,0.0);
      frame_->manager_.change_mesh_params(vals);
    }
  }
  
  frame_->manager_.set_vehicle_model(pca_vehicle_manager::DETAILED3);
  if(vul_file::exists(path+"default.parts") &&
     frame_->manager_.load_parts(path+"default.parts")){
    if(vul_file::exists(path+"default3.pca") &&
       frame_->manager_.load_pca(path+"default3.pca")){
      unsigned int num = frame_->manager_.mesh().params().size();
      vcl_vector<double> vals(num,0.0);
      frame_->manager_.change_mesh_params(vals);
      frame_->init_sliders();
    }
  }
  
  
  frame_->Show(true);

  // start the event loop
  return true;
}




