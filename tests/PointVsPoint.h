#ifndef TEST_POINTVSPOINT_H
#define TEST_POINTVSPOINT_H

#include "Soy.h"

CASE_BEGIN(PointVsPoint)

PointVsPoint()
{ 
  
 CASE_DESCRIPTION("PointVsPoint");
  
 TEST_DESCRIPTION("Point at(0,0,0), Point at (0,0,0)")
 {
  beastie::Point pointA = beastie::Point();
  beastie::Point pointB = beastie::Point();
  
  beastie::Intersection intersection = pointA.intersection(&pointB);
  
  TEST_FAIL_IF(intersection.hit == false, "Points should intersect")
 }
  
 TEST_DESCRIPTION("Point at(eps,0,0), Point at (0,0,0)")
 {
  beastie::Point pointA = beastie::Point(beastie::eps,0,0);
  beastie::Point pointB = beastie::Point(0,0,0);
  
  beastie::Intersection intersection = pointA.intersection(&pointB);
  
  TEST_FAIL_IF(intersection.hit == false, "Points should intersect")
 }
  
 TEST_DESCRIPTION("Point at(eps/3,eps/3,eps/3), Point at (0,0,0)")
 {
  beastie::Point pointA = beastie::Point(beastie::eps/3,beastie::eps/3,beastie::eps/3);
  beastie::Point pointB = beastie::Point(0,0,0);
  
  beastie::Intersection intersection = pointA.intersection(&pointB);
  
  TEST_FAIL_IF(intersection.hit == false, "Points should intersect")
 }
   
 TEST_DESCRIPTION("Point at(-100,-100,-100), Point at (100,100,100)")
 {
  beastie::Point pointA = beastie::Point(-100,-100,-100);
  beastie::Point pointB = beastie::Point(100,100,100);
  
  beastie::Intersection intersection = pointA.intersection(&pointB);
  
  TEST_FAIL_IF(intersection.hit == true, "Points shouldn't intersect")
 }
 
} CASE_END

#endif
