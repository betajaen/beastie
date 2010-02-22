#ifndef TEST_LINEVSPLANE_H
#define TEST_LINEVSPLANE_H

#include "Soy.h"

CASE_BEGIN(LineVsPlane)

LineVsPlane()
{
 CASE_DESCRIPTION("NxOgre::Path");
 
 TEST_DESCRIPTION("Empty Path")
 {
  beastie::Plane plane = beastie::Plane(Ogre::Vector3::UNIT_Y, 0);
  beastie::Line  line = beastie::Line(Ogre::Vector3(0, 10, 0), Ogre::Vector3::NEGATIVE_UNIT_Y, 20);
  
  beastie::Intersection intersection = line.intersection(&plane);
  
  TEST_FAIL_IF(intersection.hit == false, "Line should hit plane")
  TEST_FAIL_IF(Ogre::Vector3::ZERO.distance(intersection.position) > beastie::eps, "Intersection point is incorrect")
 }

} CASE_END

#endif