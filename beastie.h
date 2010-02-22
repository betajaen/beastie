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

#ifndef such_a_wee_beastie_h
#define such_a_wee_beastie_h

#include "OGRE/OgrePrerequisites.h"
#include "OGRE/OgreVector3.h"
#include "OGRE/OgreManualObject.h"

// Max. Number of Triangles per DynamicTriangleMesh.
#define BEASTIE_DYNAMICTRIANGLEMESH_MAX_TRI_COUNT 32

namespace beastie
{
 
 static const Ogre::Real eps        =  std::numeric_limits<Ogre::Real>::epsilon();
 static const Ogre::Real epsSquared =  eps * eps;
 
 static const Ogre::ColourValue     VisualDebuggerPointColour           = Ogre::ColourValue(0,0,1,1);
 static const Ogre::ColourValue     VisualDebuggerLineColour            = Ogre::ColourValue(0,1,0,1);
 static const Ogre::ColourValue     VisualDebuggerPlaneColour           = Ogre::ColourValue(1,0,1,1);
 static const Ogre::ColourValue     VisualDebuggerTriangleColour        = Ogre::ColourValue(1,1,1,1);
 static const Ogre::ColourValue     VisualDebuggerDynamicTriangleColour = Ogre::ColourValue(0,1,1,1);
 
 // Abstract Shape
 class Shape;
 
 // Single point in space.
 class Point;

 // Line based from position, direction and length.
 class Line;

 // Single Triangle
 class Triangle;

 // Infinite plane based from a normal and distance from 0,0,0
 class Plane;

 // Optimised Triangle Mesh that can be moved, but limited by Triangle Count. (BEASTIE_DYNAMICTRIANGLEMESH_MAX_TRI_COUNT)
 class DynamicTriangleMesh;

 // Optimised Triangle Mesh that can't be moved, etc.
 class StaticTriangleMesh;

 // Result of an intersection test
 struct Intersection
 {
  Intersection() : hit(false) {}
  // Did hit?
  bool           hit;
  // Global position of the hit.
  Ogre::Vector3  position;
 };
 
 // Collision tests
 class Tests
 {
   
  public:
   
   static void intersection(Point*,    Point*, Intersection&);
   static void intersection(Line*,     Point*, Intersection&);
   static void intersection(Triangle*, Point*, Intersection&);
   static void intersection(Plane*,    Point*, Intersection&);

   static void intersection(Point*,    Line*, Intersection&);
   static void intersection(Line*,     Line*, Intersection&);
   static void intersection(Triangle*, Line*, Intersection&);
   static void intersection(Plane*,    Line*, Intersection&);

   static void intersection(Point*,    Triangle*, Intersection&);
   static void intersection(Line*,     Triangle*, Intersection&);
   static void intersection(Triangle*, Triangle*, Intersection&);
   static void intersection(Plane*,    Triangle*, Intersection&);

   static void intersection(Point*,    Plane*, Intersection&);
   static void intersection(Line*,     Plane*, Intersection&);
   static void intersection(Triangle*, Plane*, Intersection&);
   static void intersection(Plane*,    Plane*, Intersection&);
   
 };
 
 class Shape
 {
   
  public:
   
   enum ShapeType
   {
    ShapeType_Point,
    ShapeType_Line,
    ShapeType_Triangle,
    ShapeType_Plane
   };
   
   inline virtual ShapeType     getShapeType() const = 0;
   
   inline virtual Ogre::Vector3 getPosition() const = 0;
   
   inline virtual void          setPosition(const Ogre::Vector3&) = 0 ;
   
   virtual Intersection         intersection(Shape*) = 0;
   
   inline virtual Point*        asPoint() { return 0; }
   
   inline virtual bool          isPoint() const { return false; } 
   
   inline virtual Line*         asLine() { return 0; }
   
   inline virtual bool          isLine() const { return false; } 

   inline virtual Triangle*     asTriangle() { return 0; }
   
   inline virtual bool          isTriangle() const { return false; } 

   inline virtual Plane*        asPlane() { return 0; }
   
   inline virtual bool          isPlane() const { return false; } 

 };
 
 
 class Point : public Shape
 {
   
   friend class Tests;
   friend class VisualRenderer;
   
   public:
    
    inline Point(Ogre::Real X = 0, Ogre::Real Y = 0, Ogre::Real Z = 0)
     : mPosition(X, Y, Z)                                           {   }
    
    inline Point(const Ogre::Vector3& position)
     : mPosition(position)                                          {   }
    
    inline ShapeType     getShapeType() const                       {   return Shape::ShapeType_Point;}
    
    inline Ogre::Vector3 getPosition() const                        {   return mPosition;}
    
    inline void          setPosition(const Ogre::Vector3& position) {   mPosition = position;}
    
    inline Point*        asPoint()                                  {   return static_cast<Point*>(this); }
    
    inline bool          isPoint() const                            {   return true; } 
    
    inline Intersection  intersection(Shape* shape)
    {
      Intersection hit;
            if (this == shape)       {hit.hit=true;hit.position=getPosition();}
       else if (shape->isPoint())    Tests::intersection(shape->asPoint(), this, hit);
       else if (shape->isLine())     Tests::intersection(shape->asLine(), this, hit);
       else if (shape->isTriangle()) Tests::intersection(shape->asTriangle(), this, hit);
       else if (shape->isPlane())    Tests::intersection(shape->asPlane(), this, hit);
      return hit;
    }
    
  protected:
    
    Ogre::Vector3 mPosition;
    
 };
 
 class Line : public Shape
 {
   
   friend class Tests;
   friend class VisualRenderer;
   
   public:
    
    inline Line(Ogre::Real X = 0,  Ogre::Real  Y = 0, Ogre::Real  Z = 0,
                Ogre::Real dX = 0, Ogre::Real dY = 0, Ogre::Real dZ = 0,
                Ogre::Real length = 0)
      : mPosition(X, Y, Z), mDirection(dX, dY, dZ), mLength(length)             {   }
    
    inline Line(const Ogre::Vector3& position, const Ogre::Vector3& direction,
                Ogre::Real length)
      : mPosition(position), mDirection(direction), mLength(length)             {   }
    
    inline ShapeType     getShapeType() const                                   {   return Shape::ShapeType_Line;}
    
    inline Ogre::Vector3 getPosition() const                                    {   return mPosition;}
    
    inline Ogre::Vector3 getDirection() const                                   {   return mDirection;}
    
    inline Ogre::Real    getLength() const                                      {   return mLength;}
    
    inline Ogre::Vector3 getOtherPosition() const                               {   return mPosition + (mDirection * mLength);}

    inline void          setPosition(const Ogre::Vector3& position)             {   mPosition = position;  }
    
    inline void          setPosition(Ogre::Real X, Ogre::Real Y, Ogre::Real Z)  {   mPosition = Ogre::Vector3(X,Y,Z);  }
    
    inline void          setDirection(const Ogre::Vector3& direction)           {   mDirection = direction; mDirection.normalise(); }
    
    inline void          setDirection(Ogre::Real X, Ogre::Real Y, Ogre::Real Z) {   mDirection = Ogre::Vector3(X,Y,Z);}
    
    inline void          setLength(Ogre::Real length)                           {   mLength = length;}
    
    inline Line*         asLine()                                               {   return static_cast<Line*>(this); }
    
    inline bool          isLine() const                                         {   return true; } 
   
    inline Intersection  intersection(Shape* shape)
    {
      Intersection hit;
            if (this == shape)       {hit.hit=true;hit.position=getPosition();}
       else if (shape->isPoint())    Tests::intersection(shape->asPoint(), this, hit);
       else if (shape->isLine())     Tests::intersection(shape->asLine(), this, hit);
       else if (shape->isTriangle()) Tests::intersection(shape->asTriangle(), this, hit);
       else if (shape->isPlane())    Tests::intersection(shape->asPlane(), this, hit);
      return hit;
    }
    
  protected:
    
    Ogre::Vector3 mPosition, mDirection;
    Ogre::Real    mLength;
    
 };
 
 class Triangle : public Shape
 {
   
   friend class Tests;
   friend class VisualRenderer;
   
   public:
    
    inline Triangle()
            : mA(), mB(), mC()                                  {   }
    
    inline Triangle(const Ogre::Vector3& a, const Ogre::Vector3& b,
                    const Ogre::Vector3& c)
            : mA(a), mB(b), mC(c)                               {   }
    
    inline Triangle(Ogre::Real aX, Ogre::Real aY, Ogre::Real aZ,
             Ogre::Real bX, Ogre::Real bY, Ogre::Real bZ,
             Ogre::Real cX, Ogre::Real cY, Ogre::Real cZ)
            : mA(aX, aY, aZ), mB(bX, bY, bZ), mC(cX, cY, cZ)    {   }
    
    inline ShapeType     getShapeType() const                   {  return Shape::ShapeType_Triangle;}
    
    inline Ogre::Vector3 getPosition() const                    {  return Ogre::Vector3::ZERO; /* TODO */ }
    
    inline void          setPosition(const Ogre::Vector3&)      {   }
    
    inline void          setPositionA(const Ogre::Vector3&)     {   }
    
    inline void          setPositionB(const Ogre::Vector3&)     {   }
    
    inline void          setPositionC(const Ogre::Vector3&)     {   }
    
    inline Ogre::Vector3 getPositionA() const                   {   return mA;  }
    
    inline Ogre::Vector3 getPositionB() const                   {   return mB;  }
    
    inline Ogre::Vector3 getPositionC() const                   {   return mC;  }
    
    inline Triangle*     asTriangle()                           {   return static_cast<Triangle*>(this); }
    
    inline bool          isTriangle() const                     {   return true; }
    
    inline Intersection  intersection(Shape* shape)
    {
      Intersection hit;
            if (this == shape)       {hit.hit=true;hit.position=getPosition();}
       else if (shape->isPoint())    Tests::intersection(shape->asPoint(), this, hit);
       else if (shape->isLine())     Tests::intersection(shape->asLine(), this, hit);
       else if (shape->isTriangle()) Tests::intersection(shape->asTriangle(), this, hit);
       else if (shape->isPlane())    Tests::intersection(shape->asPlane(), this, hit);
      return hit;
    }
    
   protected:
    
    Ogre::Vector3 mA, mB, mC;
    
 };
 
 
 class Plane : public Shape
 {
   
   friend class Tests;
   friend class VisualRenderer;
   
   public:
    
    Plane(Ogre::Real nX = 0, Ogre::Real nY = 0, Ogre::Real nZ = 0, Ogre::Real distance = 0)
     : mNormal(nX,nY,nZ), mDistance(distance)                   {   }
    
    Plane(const Ogre::Vector3& normal, Ogre::Real distance)
     : mNormal(normal), mDistance(distance)                     {   }
    
    inline ShapeType     getShapeType() const                   {   return Shape::ShapeType_Plane;}
    
    inline Ogre::Vector3 getPosition() const                    {   return mNormal * mDistance;}
    
    inline void          setPosition(const Ogre::Vector3& pos)  {   mNormal = pos; mNormal.normalise(); mDistance = mNormal.length();}
    
    inline void          setNormal(const Ogre::Vector3& normal) {   mNormal = normal;}
    
    inline Ogre::Vector3 getNormal() const                      {   return mNormal;}
    
    inline void          setDistance(Ogre::Real distance)       {   mDistance = distance; }
    
    inline Ogre::Real    getDistance() const                    {   return mDistance; }
    
    inline Plane*        asPlane()                              {   return static_cast<Plane*>(this); }
    
    inline bool          isPlane() const                        {   return true; }
    
    inline Intersection  intersection(Shape* shape)
    {
      Intersection hit;
            if (this == shape)       {hit.hit=true;hit.position=getPosition();}
       else if (shape->isPoint())    Tests::intersection(shape->asPoint(), this, hit);
       else if (shape->isLine())     Tests::intersection(shape->asLine(), this, hit);
       else if (shape->isTriangle()) Tests::intersection(shape->asTriangle(), this, hit);
       else if (shape->isPlane())    Tests::intersection(shape->asPlane(), this, hit);
      return hit;
    }
    
  protected:
    
    Ogre::Vector3 mNormal;
    
    Ogre::Real    mDistance;
    
 };
 
 class VisualRenderer
 {
   
  public:
   
   VisualRenderer(Ogre::ManualObject* manualObject)
    : mManualObject(manualObject)                            {   }
   
   void beginRender()
   {
    if (mManualObject->getNumSections() == 0)
     mManualObject->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_LINE_LIST);
    else
     mManualObject->beginUpdate(0);
   }
   
   void renderShape(Shape* shape)
   {
    if (shape == 0)
     return;
    else if (shape->isPoint())
    {
     Point* point = shape->asPoint();
#ifndef BEASTIE_VISUALDEBUGGER_SIMPLE
     mManualObject->position(point->mPosition.x-0.1f, point->mPosition.y, point->mPosition.z);
     mManualObject->colour(VisualDebuggerPointColour);
     mManualObject->position(point->mPosition.x+0.1f, point->mPosition.y, point->mPosition.z);
     mManualObject->colour(VisualDebuggerPointColour);
     mManualObject->position(point->mPosition.x, point->mPosition.y, point->mPosition.z-0.1f);
     mManualObject->colour(VisualDebuggerPointColour);
     mManualObject->position(point->mPosition.x, point->mPosition.y, point->mPosition.z+0.1f);
     mManualObject->colour(VisualDebuggerPointColour);
     mManualObject->position(point->mPosition.x, point->mPosition.y-0.1f, point->mPosition.z);
     mManualObject->colour(VisualDebuggerPointColour);
     mManualObject->position(point->mPosition.x, point->mPosition.y+0.1f, point->mPosition.z);
     mManualObject->colour(VisualDebuggerPointColour);
#else
     mManualObject->position(point->mPosition.x, point->mPosition.y, point->mPosition.z);
     mManualObject->colour(VisualDebuggerPointColour);
     mManualObject->position(point->mPosition.x, point->mPosition.y+Ogre::Real(0.001), point->mPosition.z);
     mManualObject->colour(VisualDebuggerPointColour);
#endif
    }
    else if (shape->isLine())
    {
     Line* line = shape->asLine();
     mManualObject->position(line->mPosition);
     mManualObject->colour(VisualDebuggerLineColour);
     mManualObject->position(line->mPosition + (line->mDirection * line->mLength));
     mManualObject->colour(VisualDebuggerLineColour);
    }
    else if (shape->isPlane())
    {
     Plane* plane = shape->asPlane();
     mManualObject->position(plane->mNormal * plane->mDistance );
     mManualObject->colour(VisualDebuggerPlaneColour);
     mManualObject->position(plane->mNormal * (plane->mDistance + 1) );
     mManualObject->colour(VisualDebuggerPlaneColour);
    }
    else if (shape->isTriangle())
    {
     Triangle* triangle = shape->asTriangle();
     mManualObject->position(triangle->mA);
     mManualObject->colour(VisualDebuggerTriangleColour);
     mManualObject->position(triangle->mB);
     mManualObject->colour(VisualDebuggerTriangleColour);
     mManualObject->position(triangle->mB);
     mManualObject->colour(VisualDebuggerTriangleColour);
     mManualObject->position(triangle->mC);
     mManualObject->colour(VisualDebuggerTriangleColour);
     mManualObject->position(triangle->mC);
     mManualObject->colour(VisualDebuggerTriangleColour);
     mManualObject->position(triangle->mA);
     mManualObject->colour(VisualDebuggerTriangleColour);
    }
   }
   
   void endRender()
   {
    mManualObject->end();
   }
   
  protected:
   
   Ogre::ManualObject*  mManualObject;
   
 };
 
}

#endif
