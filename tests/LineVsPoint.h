#ifndef TEST_LINEVSPOINT_H
#define TEST_LINEVSPOINT_H

#include "Soy.h"

CASE_BEGIN(LineVsPoint)

LineVsPoint()
{
 CASE_DESCRIPTION("LineVsPoint");
 
 TEST_DESCRIPTION("Point at(0,0,0), Line at(0,10,0), Dir (0,-1,0), Length 10")
 {
  beastie::Line  line  = beastie::Line(Ogre::Vector3(0,10,0), Ogre::Vector3::NEGATIVE_UNIT_Y, 10);
  beastie::Point point = beastie::Point(Ogre::Vector3::ZERO);
  
  beastie::Intersection intersection = line.intersection(&point);
  
  TEST_FAIL_IF(intersection.hit == false, "Line should hit point")
  TEST_FAIL_IF(Ogre::Vector3::ZERO.distance(Ogre::Vector3::ZERO) > beastie::eps, "Intersection point is incorrect")
 }
 
} CASE_END

#endif
