// This is gui/gui_utils.h
#ifndef gui_utils_h_
#define gui_utils_h_
//:
// \file
// \brief  More useful static functions to augment vgui_utils
// \author Matt Leotta, (mleotta@lems.brown.edu)
// \date 9/9/04
//
// \verbatim
//  Modifications
// \endverbatim

#include <vgui/vgui_tableau_sptr.h>

//: More useful static functions to augment vgui_utils
class gui_utils
{
 public:
  //: Render the tableaux heirarchy to PostScript
  static void render_to_ps(const vgui_tableau_sptr& tableau);

};

#endif // gui_utils_h_
