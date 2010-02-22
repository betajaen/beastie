/*  
    
    beastie, Collision Detection Library for Ogre.
    
    Copyright (c) 2010 Robin Southern
    
    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:
    
    The above copyright notice and this permission notice shall be included in
    all copies or substantial portions of the Software.
    
    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
    THE SOFTWARE.
*/

#include "beastie.h"

#include <iostream>

#define RETURN_IF(COND) if (COND) return;
#define SQUARED(X) X*X

void beastie::Tests::intersection(class beastie::Point* pointA,class beastie::Point* pointB, struct beastie::Intersection& intersection)
{
 if (pointA->mPosition.squaredDistance(pointB->mPosition) <= beastie::epsSquared)
 {
  intersection.hit = true;
  intersection.position = pointA->mPosition;
 }
}

// Triangle vs Point
void beastie::Tests::intersection(beastie::Triangle* triangle, beastie::Point* point, beastie::Intersection& intersection)
{
 Ogre::Vector3 normal = Ogre::Math::calculateBasicFaceNormal(triangle->mA, triangle->mB, triangle->mC);
 if (Ogre::Math::pointInTri3D(point->mPosition, triangle->mA, triangle->mB, triangle->mC, normal))
 {
  intersection.hit = true;
  intersection.position = point->mPosition;
 }
}

// Point vs Line
void beastie::Tests::intersection(beastie::Point* point, beastie::Line* line, beastie::Intersection& intersection)
{
 beastie::Tests::intersection(line, point, intersection);
}

// Line vs Point
void beastie::Tests::intersection(beastie::Line* line, beastie::Point* point, beastie::Intersection& intersection)
{
 RETURN_IF(line->mLength == 0)
 RETURN_IF(line->mDirection == Ogre::Vector3::ZERO);
 
 Ogre::Vector3 v = point->mPosition - line->mPosition;
 const Ogre::Vector3 s = line->getOtherPosition() - line->getPosition();
 Ogre::Real dot = v.dotProduct(s) / (SQUARED(line->getLength()));
 v = v - (s * dot);
 
 RETURN_IF(v.squaredLength() >= epsSquared);
 
 intersection.hit = true;
 intersection.position = point->mPosition;
}

void beastie::Tests::intersection(beastie::Point* point, beastie::Plane* plane, beastie::Intersection& intersection)
{
 Tests::intersection(plane, point, intersection);
}

// Plane vs Point
void beastie::Tests::intersection(beastie::Plane* plane, beastie::Point* point, beastie::Intersection& intersection)
{

 float d = point->mPosition.dotProduct(plane->mNormal) + plane->mDistance;
 RETURN_IF(d > beastie::eps);
 
 intersection.hit = true;
 intersection.position = point->mPosition;
}

// Triangle vs Line
void beastie::Tests::intersection(beastie::Triangle* triangle, beastie::Line* line, beastie::Intersection& intersection)
{
}

// Line vs Line
void beastie::Tests::intersection(beastie::Line* line1, beastie::Line* line2, beastie::Intersection& intersection)
{
}

// Plane vs Line
void beastie::Tests::intersection(beastie::Plane* plane, beastie::Line* line, beastie::Intersection& intersection)
{
 beastie::Tests::intersection(line, plane, intersection);
}

// Line vs Triangle
void beastie::Tests::intersection(beastie::Line* line, beastie::Triangle* triangle, beastie::Intersection& intersection)
{
}

// Triangle vs Triangle
void beastie::Tests::intersection(class beastie::Triangle* triangle1, class beastie::Triangle* triangle2, beastie::Intersection& intersection)
{
}

// Plane vs Triangle
void beastie::Tests::intersection(class beastie::Plane* plane,class beastie::Triangle* triangle, beastie::Intersection& intersection)
{
}

// Triangle vs Plane
void beastie::Tests::intersection(beastie::Triangle* triangle, beastie::Plane* plane, beastie::Intersection& intersection)
{
}

// Line vs Plane
void beastie::Tests::intersection(beastie::Line* line, beastie::Plane* plane, beastie::Intersection& intersection)
{
 Ogre::Real denom = plane->mNormal.dotProduct(line->mDirection);
 RETURN_IF(Ogre::Math::Abs(denom) < eps)
 
 Ogre::Real nom = plane->mNormal.dotProduct(line->mPosition) + plane->mDistance;
 Ogre::Real t   = -(nom/denom);
 if (t <= line->mLength)
 {
  intersection.hit = true;
  intersection.position = line->mPosition + (line->mDirection * t);
 }
}

// Plane vs Plane
void beastie::Tests::intersection(beastie::Plane* plane1, beastie::Plane* plane2, beastie::Intersection& intersection)
{
}

#undef RETURN_IF
#undef SQUARED
