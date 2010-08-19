// This is gui/gui_utils.cxx

//          Copyright Matthew Leotta 2006 - 2010.
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file ../LICENSE_1_0.txt or copy at
//          http://www.boost.org/LICENSE_1_0.txt)

//:
// \file

#include "gui_utils.h"
#include <gl2ps/gl2ps.h>
#include <vgui/vgui.h>
#include <vgui/vgui_tableau.h>
#include <vgui/vgui_dialog.h>

//: Render the tableaux heirarchy to PostScript
void
gui_utils::render_to_ps(const vgui_tableau_sptr& tableau)
{
  GLint file_type = 1;
  
  vgui_dialog save_ps_dlg("Render to PostScript");
  static vcl_string file_name = "";
  static vcl_string ext = "*.ps";
  save_ps_dlg.file("File:", ext, file_name);
  vcl_vector<vcl_string> types;
  types.push_back("(PS)  PostScript");
  types.push_back("(EPS) Encapsulated PostScript");
  types.push_back("(PDF) Portable Document Format");
  types.push_back("(TEX) Text Only");
  int type = 0;
  save_ps_dlg.choice("Output Type", types, type);
  if( !save_ps_dlg.ask())
    return;

  switch(type){
    case 0: file_type = GL2PS_PS;  break;
    case 1: file_type = GL2PS_EPS; break;
    case 2: file_type = GL2PS_PDF; break;
    case 3: file_type = GL2PS_TEX; break;
  }

  FILE *fp = fopen(file_name.c_str(), "wb");
  GLint buffsize = 0, state = GL2PS_OVERFLOW;
  GLint viewport[4];
  glGetIntegerv(GL_VIEWPORT, viewport);

  while( state == GL2PS_OVERFLOW ){
    buffsize += 1024*1024;
    gl2psBeginPage ( "MyTitle", "MySoftware", viewport,
                     file_type, GL2PS_BSP_SORT,
                     GL2PS_SILENT | GL2PS_SIMPLE_LINE_OFFSET | GL2PS_NO_BLENDING |
                     GL2PS_OCCLUSION_CULL | GL2PS_BEST_ROOT,
                     GL_RGBA, 0, NULL, 0, 0, 0,
                     buffsize, fp, file_name.c_str() );
    tableau->handle(vgui_DRAW);
    state = gl2psEndPage();
  }
  fclose(fp);
}

