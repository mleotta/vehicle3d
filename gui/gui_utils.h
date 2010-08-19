// This is gui/gui_utils.h
#ifndef gui_utils_h_
#define gui_utils_h_
//:
// \file
// \brief  More useful static functions to augment vgui_utils
// \author Matt Leotta, (mleotta@lems.brown.edu)
// \date 9/9/04
//
//          Copyright Matthew Leotta 2006 - 2010.
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file ../LICENSE_1_0.txt or copy at
//          http://www.boost.org/LICENSE_1_0.txt)
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
