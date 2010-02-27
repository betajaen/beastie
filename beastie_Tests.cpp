#include "beastie.h"
#include <iostream>

bool failed = false;
void main() {if (!failed){std::cout <<"* OK\n";}}

#define TEST(NAME) void NAME()
#define RUN(FUNC) FUNC();
#define FAIL(COND, REASON) if(COND){failed=true;std::cout << "Failed at " << __FUNCTION__ << "\nReason: " << REASON << "\n";return;}

struct _Triangle
{
 _Triangle()
 {
  beastie::Triangle tri;
  tri.normalise();
 }
} _triangle;

struct PointVsPoint
{
 PointVsPoint()
 {
  RUN(ZeroZero)
  RUN(ZeroOne)
 }
 
 TEST(ZeroZero)
 {
  beastie::ContactPair pair;
  beastie::Point a, b;
  a.setPosition(Ogre::Vector3::ZERO);
  b.setPosition(Ogre::Vector3::ZERO);
  a.collide(&b, pair);
  
  FAIL(pair.hit == false, "Points should collide");
 }

 TEST(ZeroOne)
 {
  beastie::ContactPair pair;
  beastie::Point a, b;
  a.setPosition(Ogre::Vector3::ZERO);
  b.setPosition(Ogre::Vector3(0,1,0));
  a.collide(&b, pair);
  
  FAIL(pair.hit == true, "Points shouldn't collide");
 }
 
} point_vs_point;

struct PointVsLine
{
 
 PointVsLine()
 {
  RUN(DownZero)
 }
 
 TEST(DownZero)
 {
  
 }
} point_vs_line;