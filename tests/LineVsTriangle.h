#ifndef TEST_LINEVSTRIANGLE_H
#define TEST_LINEVSTRIANGLE_H

#include "Soy.h"

CASE_BEGIN(LineVsTriangle)

LineVsTriangle()
{
 CASE_DESCRIPTION("LineVsTriangle");
 
 TEST_DESCRIPTION("Line (0,0,0)(0,-1,0)(10), Triangle (-1,0,-1,1,0,1,1,0,-1)")
 {
  // 1,-1    1,1
  // 
  // -1,-1,  1,-1
  beastie::Line     line  = beastie::Line(Ogre::Vector3(0.5,10,0.5), Ogre::Vector3::NEGATIVE_UNIT_Y, 10);
  beastie::Triangle tri   = beastie::Triangle(-1,0,-1,   1,0,1,   1,0,-1);
  
  std::cout << tri.getNormal() << std::endl;
  
  beastie::Intersection intersection = line.intersection(&tri);
  
  TEST_FAIL_IF(intersection.hit == false, "Line should hit triangle")
  TEST_FAIL_IF(Ogre::Vector3::ZERO.distance(Ogre::Vector3::ZERO) > beastie::eps, "Intersection point is incorrect")
 }
 
} CASE_END

#endif
