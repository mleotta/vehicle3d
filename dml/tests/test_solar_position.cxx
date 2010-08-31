//          Copyright Matthew Leotta 2006 - 2010.
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file ../../LICENSE_1_0.txt or copy at
//          http://www.boost.org/LICENSE_1_0.txt)

#include <testlib/testlib_test.h>
#include <dml/dml_solar_position.h>
#include <vcl_iostream.h>




MAIN( test_solar_position )
{
   START ("Solar Position");

   double alt,az;
   dml_solar_position(79, 17, // March 20th, 12:00 PM EST
                      41.82608, -71.39991, // Latitude,Longitude of Brown University
                      alt, az);
   // These results are not expected to be extremely precise                    
   TEST_NEAR("altitude", alt, 0.82885686, 1e-4);  
   TEST_NEAR("azimuth",  az,  3.1862731, 1e-4);    
   
   dml_solar_position(79, 14, // March 20th, 9:00 AM EST
                      41.82608, -71.39991, // Latitude,Longitude of Brown University
                      alt, az);
   // These results are not expected to be extremely precise                    
   TEST_NEAR("altitude", alt, 0.56443948, 1e-4);  
   TEST_NEAR("azimuth",  az,  2.1952751, 1e-4);                   
   
   dml_solar_position(79, 20, // March 20th, 3:00 PM EST
                      41.82608, -71.39991, // Latitude,Longitude of Brown University
                      alt, az);
   // These results are not expected to be extremely precise                    
   TEST_NEAR("altitude", alt, 0.52726397, 1e-4);  
   TEST_NEAR("azimuth",  az,  4.1434116, 1e-4);  
   
   SUMMARY();
}


