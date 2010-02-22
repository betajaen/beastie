#ifndef TEST_LINEVSPLANE_H
#define TEST_LINEVSPLANE_H

#include "Soy.h"

CASE_BEGIN(LineVsPlane)

LineVsPlane()
{
 CASE_DESCRIPTION("LineVsPlane");
 
 TEST_DESCRIPTION("Plane +Y, Line at(0,10,0), Dir (0,-1,0), Length 20")
 {
  beastie::Plane plane = beastie::Plane(Ogre::Vector3::UNIT_Y, 0);
  beastie::Line  line = beastie::Line(Ogre::Vector3(0, 10, 0), Ogre::Vector3::NEGATIVE_UNIT_Y, 20);
  
  beastie::Intersection intersection = line.intersection(&plane);
  
  TEST_FAIL_IF(intersection.hit == false, "Line should hit plane")
  TEST_FAIL_IF(Ogre::Vector3::ZERO.distance(intersection.position) > beastie::eps, "Intersection point is incorrect")
 }

 TEST_DESCRIPTION("Plane +Y, Line at(0,10,0), Dir (0,-1,0), Length 10")
 {
  beastie::Plane plane = beastie::Plane(Ogre::Vector3::UNIT_Y, 0);
  beastie::Line  line = beastie::Line(Ogre::Vector3(0, 10, 0), Ogre::Vector3::NEGATIVE_UNIT_Y, 10);
  
  beastie::Intersection intersection = line.intersection(&plane);
  
  TEST_FAIL_IF(intersection.hit == false, "Line should hit plane")
  TEST_FAIL_IF(Ogre::Vector3::ZERO.distance(intersection.position) > beastie::eps, "Intersection point is incorrect")
 }

 TEST_DESCRIPTION("Plane +Y, Line at(0,10,0), Dir (0,-1,0), Length 5")
 {
  beastie::Plane plane = beastie::Plane(Ogre::Vector3::UNIT_Y, 0);
  beastie::Line  line = beastie::Line(Ogre::Vector3(0, 10, 0), Ogre::Vector3::NEGATIVE_UNIT_Y, 5);
  
  beastie::Intersection intersection = line.intersection(&plane);
  
  TEST_FAIL_IF(intersection.hit == true, "Line shouldn't hit plane")
 }

} CASE_END

#endif