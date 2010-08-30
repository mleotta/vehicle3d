// This is dbpro/filters/bbgm_filters.h
#ifndef bbgm_filters_h_
#define bbgm_filters_h_

//:
// \file
// \brief dbpro filters for background modeling
// \author Matt Leotta (mleotta@lems.brown.edu)
// \date 6/12/06
//
//          Copyright Matthew Leotta 2006 - 2010.
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file ../../LICENSE_1_0.txt or copy at
//          http://www.boost.org/LICENSE_1_0.txt)
//
// \verbatim
//  Modifications
// \endverbatim


#include <vil/vil_image_resource.h>
#include <vil/vil_image_view.h>
#include <vil/vil_new.h>

#include <dbpro/dbpro_process.h>
#include <bbgm/bbgm_update.h>
#include <bbgm/bbgm_apply.h>
#include <bbgm/bbgm_image_of.h>
#include <bbgm/bbgm_image_sptr.h>

#include <bbgm/bbgm_detect.h>



template <class updater_t, class T>
class bbgm_update_filter : public dbpro_filter
{
public:
  typedef typename updater_t::distribution_type dist_t;
  bbgm_update_filter(const updater_t& u) : updater_(u) {}
  
  dbpro_signal execute()
  {
    assert(input_type_id(0) == typeid(bbgm_image_sptr));
    bbgm_image_sptr model_sptr = input<bbgm_image_sptr >(0);
    bbgm_image_of<dist_t>* model = static_cast<bbgm_image_of<dist_t>*>(model_sptr.ptr());
    
    assert(input_type_id(1) == typeid(vil_image_resource_sptr));
    vil_image_resource_sptr image = input<vil_image_resource_sptr>(1);
    vil_image_view<T> img = image->get_view();
    
    if(model->ni() != img.ni() || model->nj() != img.nj())
      model->set_size(img.ni(),img.nj());
    
    update(*model,img,updater_);
    
    output(0,model_sptr);
    return DBPRO_VALID;
  }
  
private:
  updater_t updater_;
};


template <class updater_t, class T>
class bbgm_update_masked_filter : public dbpro_filter
{
public:
  typedef typename updater_t::distribution_type dist_t;
  bbgm_update_masked_filter(const updater_t& u) : updater_(u) {}
  
  dbpro_signal execute()
  {
    assert(input_type_id(0) == typeid(bbgm_image_sptr));
    bbgm_image_sptr model_sptr = input<bbgm_image_sptr >(0);
    bbgm_image_of<dist_t>* model = static_cast<bbgm_image_of<dist_t>*>(model_sptr.ptr());
    
    assert(input_type_id(1) == typeid(vil_image_resource_sptr));
    vil_image_resource_sptr image = input<vil_image_resource_sptr>(1);
    vil_image_view<T> img = image->get_view();
    
    assert(input_type_id(2) == typeid(vil_image_resource_sptr));
    vil_image_resource_sptr mask_image = input<vil_image_resource_sptr>(2);
    vil_image_view<bool> mask = mask_image->get_view();
    
    if(model->ni() != img.ni() || model->nj() != img.nj())
      model->set_size(img.ni(),img.nj());
    
    update_masked(*model,img,updater_,mask);
    
    output(0,model_sptr);
    return DBPRO_VALID;
  }
  
private:
  updater_t updater_;
};



template <class detector_t, class T>
class bbgm_detect_filter : public dbpro_filter
{
public:
  typedef typename detector_t::distribution_type dist_t;
  bbgm_detect_filter(const detector_t& d) : detector_(d) {}
  
  dbpro_signal execute()
  {
    assert(input_type_id(0) == typeid(bbgm_image_sptr));
    bbgm_image_sptr model_sptr = input<bbgm_image_sptr >(0);
    bbgm_image_of<dist_t>* model = static_cast<bbgm_image_of<dist_t>*>(model_sptr.ptr());
    
    assert(input_type_id(1) == typeid(vil_image_resource_sptr));
    vil_image_resource_sptr image = input<vil_image_resource_sptr>(1);
    vil_image_view<T> img = image->get_view();
    
    vil_image_view<bool> mask(img.ni(),img.nj(),1);
    mask.fill(false);
    
    vil_structuring_element se;
    se.set_to_disk(2.0);
    detect(*model,img,mask,detector_,se);
    //bbgm_apply(*model,detector_,img,mask);
    
    output(0, vil_new_image_resource_of_view(mask));
    return DBPRO_VALID;
  }
  
private:
  detector_t detector_;
};



template <class detector_t, class T>
class bbgm_detect_masked_filter : public dbpro_filter
{
public:
  typedef typename detector_t::distribution_type dist_t;
  bbgm_detect_masked_filter(const detector_t& d) : detector_(d) {}
  
  dbpro_signal execute()
  {
    assert(input_type_id(0) == typeid(bbgm_image_sptr));
    bbgm_image_sptr model_sptr = input<bbgm_image_sptr >(0);
    bbgm_image_of<dist_t>* model = static_cast<bbgm_image_of<dist_t>*>(model_sptr.ptr());
    
    assert(input_type_id(1) == typeid(vil_image_resource_sptr));
    vil_image_resource_sptr image = input<vil_image_resource_sptr>(1);
    vil_image_view<T> img = image->get_view();
    
    assert(input_type_id(2) == typeid(vil_image_resource_sptr));
    vil_image_resource_sptr in_mask_image = input<vil_image_resource_sptr>(2);
    vil_image_view<bool> in_mask = in_mask_image->get_view();
    
    vil_image_view<bool> mask(img.ni(),img.nj(),1);
    mask.fill(false);
    
    vil_structuring_element se;
    se.set_to_disk(2.0);
    detect_masked(*model,img,mask,detector_,se,in_mask);
    //bbgm_apply_masked(*model,detector_,img,mask);
    
    output(0, vil_new_image_resource_of_view(mask));
    return DBPRO_VALID;
  }
  
private:
  detector_t detector_;
};


#endif //bbgm_filters_h_
