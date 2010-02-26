/*! md_page. Beastie

**Note** -- The most recent copy of beastie can be found at the [Beastie Git Repository](http://github.com/betajaen/beastie).

Beastie
=======

Software Licence
----------------

> Copyright (c) 2010 Robin Southern
> 
> Permission is hereby granted, free of charge, to any person obtaining a copy
> of this software and associated documentation files (the "Software"), to deal
> in the Software without restriction, including without limitation the rights
> to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
> copies of the Software, and to permit persons to whom the Software is
> furnished to do so, subject to the following conditions:
> 
> The above copyright notice and this permission notice shall be included in
> all copies or substantial portions of the Software.
> 
> THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
> IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
> FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
> AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
> LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
> OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
> THE SOFTWARE.

What is Beastie?
----------------

Beastie is a C++ based simplified collision detection library for the [Ogre3D graphics engine](http://www.ogre3d.org), and it only comes in two files, `beastie.h` and `beastie.cpp`. "Beastie" should be always pronounced in a Glaswegian accent. 

It only detects collisions and does not respond to them as you would typically encounter in a more fully featured physics engine or more advanced collision library.
However the detections are accurate and are optimised, building some response code on top of beastie shouldn't be to difficult, an example of some of this is in the "wee.cpp" test application that can be found at the beastie Git repository.

Beastie only uses Ogre3D as a dependency, and is developed with Ogre 1.7 in mind; but should work with any 1.x version.

Using Beastie
-------------

All of Beastie functions are contained in the @beastie@ namespace, any functions are put in Utils or Tests based on their use. All classes and functions use the camelCase notation, with the exception that class names have an uppercased first character.

To include it in your project, simply copy the `beastie.cpp` and `beastie.h` files into your project directory, then include them in your project, making sure you correctly include and link to Ogre.


Shapes
------

All beastie shapes inherit from the `Shape` class, so they can be upcasted into a Shape if you wish to store them as pointers, or intersect them via the `Shape::intersect` function.
  
    // Straight to the point.
    Point point1 = Point(1,2,3);
    // And as very abstract.
    Shape* point2 = new Point(3,2,1);
    point2->intersect(&point);

Shapes have a collection of functions to tell you what type of Shape it is, if handled as a pointer.

* `Shape::getShapeType`
* `Shape::isPoint`
* `Shape::isLine` _and so on_

A Shape can be downcasted into a specific shape; `Shape::asPoint`, `Shape::asLine`... These functions should be checked with `Shape::isPoint`, `Shape::isLine`... before downcasting, in the event of a wrong is function used, a null pointer is returned.

    Line* line = shape->asLine();

Points
------

Points represent a single point in 3D space and can be used for particles or other wee things.

Points have a single position property (`Point::setPosition`, `Point::getPosition`), its range is from [-`FLT_MAX`, -`FLT_MAX`, -`FLT_MAX`] to [`FLT_MAX`, `FLT_MAX`, `FLT_MAX`].

    // Intersection test
    Point a = Point(Ogre::Vector3::ZERO);
    Point b = Point(Ogre::Vector3::ZERO);
    a.intersect(&b); // .hit = true
    b->setPosition(1,0,0);
    a.intersect(&b); // .hit = false

To help against floating point precision, the maximum radius of the point is beastie::eps is included in most intersection functions.

    a.setPosition(Ogre::Vector3::ZERO);
    b.setPosition(eps,0,0);
    a.intersection(&b); // .hit = true

Lines
-----

Lines are single line in 3D space with a specified length. They can be used in numerous ways but one of the most common is raycasting.

Lines are made from three properties:

* a origin (`Ogre::Vector3`)
* a direction (`Ogre::Vector3`)
* a length (`Ogre::Real`)

### Origin

Origins indicate where the line starts in 3D-Space, it has the same range as the Point's position, and can be set/get via `Line::setPosition` and `Line::getPosition`.

### Direction

Directions indicate where the line should go, they have the range of [-1,-1,-1]...[1,1,1] and must be normalised before use (`Ogre::Vector3::normalise`), and can be set/get via `Line::setDirection` and `Line::getDirection`.

### Length

The Length is how long the line is; it has the range of (0, FLT_MAX], and is set/get from `Line::setLength` and `Line::getLength`.


### Intersections and Raycasting

When intersecting with another shape, the intersection returned with the Line's origin to indicate the length of the intersection, in otherwords Lines can be used for RayCasting against any of Beastie's shapes.

    Line  line = Line(Ogre::Vector3(0, 10, 0), Ogre::Vector3::NEGATIVE_UNIT_Y, 20);
    Plane plane = Plane(Ogre::Vector3::UNIT_Y, 0);
    line.intersect(&plane); // .hit = true, .position = Ogre::Vector3::ZERO

The distance of the raycast is the distance between the line's origin and the point of intersection;

    Ogre::Real rayLength = line.getPosition().distance(intersection.position);

Triangle
--------

Triangles are a single triangle in 3D space. Triangles can be used in any event where a single triangle may be needed, but many in the case of many Triangles being used at once, you should consider using the StaticMesh or DynamicMesh for performance reasons.

A Triangle is made from four properties, using the NormalisedTriangle class as a base:

* A vertex (`Ogre::Vector3`)
* B vertex (`Ogre::Vector3`)
* C vertex (`Ogre::Vector3`)
* Normal (`Ogre::Vector3`)

### Vertices

The vertices can be changed at runtime using the set/get functions `Triangle::setVertexA`, `Triangle::getVertexA`,`Triangle::setVertexB`, `Triangle::getVertexB`,`Triangle::setVertexC` and `Triangle::getVertexC`.

All vertices have the same range as Point's position.

### Normal

The normal is optional, and can be auto-calculated from the A,B,C vertices. This is achieved via two of the constructors or `Triangle::calcualteNormal`.

The normal should be in range of [-1,-1,-1]...[1,1,1] and must be normalised before use (`Ogre::Vector3::normalise`).

*/
#ifndef crivens_a_wee_beastie_in_tha_cludgie_h
#define crivens_a_wee_beastie_in_tha_cludgie_h

#include "OGRE/OgrePrerequisites.h"
#include "OGRE/OgreVector3.h"
#include "OGRE/OgreManualObject.h"

namespace beastie
{

 class Shape;
 class Point;
 class Line;
 class Triangle;
 class Plane;
 struct Triangles;
 class Mesh;
 class DynamicMesh;
 class StaticMesh;

 /*! enum. ShapeType
     desc.
         Types of shapes represented by an enum, given by Shape::getShapeType
     see.
         Shape::getShapeType
 */
 enum ShapeType
 {
  //! Single Point
  ShapeType_Point       = (1<<0),
  //! Line
  ShapeType_Line        = (1<<1),
  //! Single triangle and normal.
  ShapeType_Triangle    = (1<<2),
  //! Infinite plane.
  ShapeType_Plane       = (1<<3),
  //! Collection of triangles that move around constantly.
  ShapeType_DynamicMesh = (1<<4),
  //! Optimised collection of triangles that never moved.
  ShapeType_StaticMesh  = (1<<5),
  //! All shapes filter
  AllShapes             = ShapeType_Point       | 
                          ShapeType_Line        | 
                          ShapeType_Triangle    | 
                          ShapeType_Plane       | 
                          ShapeType_DynamicMesh | 
                          ShapeType_StaticMesh
 };
 
 static const unsigned int          BeastieVersion                      = 10;
 static const Ogre::Real            eps                                 =  std::numeric_limits<Ogre::Real>::epsilon();
 static const Ogre::Real            epsSquared                          =  eps * eps;
 static const Ogre::Real            negativeEps                         = -eps;
 static const Ogre::Real            negativeEpsSquared                  = -epsSquared;
 static const Ogre::Vector3         Vector3_ONE                         = Ogre::Vector3(1,1,1);
 
 static const Ogre::ColourValue     VisualDebuggerPointColour           = Ogre::ColourValue(0,0,1,1);
 static const Ogre::ColourValue     VisualDebuggerLineColour            = Ogre::ColourValue(0,1,0,1);
 static const Ogre::ColourValue     VisualDebuggerPlaneColour           = Ogre::ColourValue(1,0,1,1);
 static const Ogre::ColourValue     VisualDebuggerTriangleColour        = Ogre::ColourValue(1,1,1,1);
 static const Ogre::ColourValue     VisualDebuggerDynamicTriangleColour = Ogre::ColourValue(0,1,1,1);
 
 /*! struct. Intersection
     desc.
          Results of an intersection.
 */
 struct Intersection
 {

  Intersection()
  : hit(false)
    { // constructor
    }

  /*! variable. hit
      desc.
          Did hit?
  */
  bool hit;

  /*! variable. position
      desc.
          Calculated global position of the intersection
  */
  Ogre::Vector3 position;

 };
 
 /*! struct. NormalisedTriangle
     desc.
         Three vertices representing a triangle and its normal.
 */
 struct NormalisedTriangle
 {

  /*! variable. a
      desc.
           Vertex A
  */
  Ogre::Vector3 a;

  /*! variable. b
      desc.
           Vertex B
  */
  Ogre::Vector3 b;

  /*! variable. c
      desc.
           Vertex C
  */
  Ogre::Vector3 c;

  /*! variable. n
      desc.
           Normal of the triangle.
      note.
           This is automatically created via NormalisedTriangle::normalise()
  */
  Ogre::Vector3 n;

  /*! function. transform
      desc.
          Transform this triangle by a position, orientation and/or scale, and calculates the new normal.
      args.
         const Ogre::Vector3& position -- Position to translate.
         const Ogre::Quaternion& orientation -- Changed orientation.
         const Ogre::Vector3& scale -- Relative Scale.
      return.
         The transformed NormalisedTriangle.
  */
  inline NormalisedTriangle transform(
        const Ogre::Vector3& position,
        const Ogre::Quaternion& orientation,
        const Ogre::Vector3& scale) const
    {
     NormalisedTriangle out;
     out.a = (orientation * (a * scale)) + position;
     out.b = (orientation * (b * scale)) + position;
     out.c = (orientation * (c * scale)) + position;
     out.normalise();
     return out;
    }

  /*! function. normalise
      desc.
          Calculate the normal (NormalisedTriangle::n) from the three vertices.
  */
  inline void normalise()
    {
     n = Ogre::Math::calculateBasicFaceNormal(a,b,c);
    }
  
  /*! function. zero
      desc.
          Sets all components to Ogre::Vector3::ZERO
  */
  inline void zero()
    {
     a = b = c = n = Ogre::Vector3::ZERO;
    }
 };

 /*! class. Utils
     desc.
         Misc. utilities for Ogre and Beastie
 */
 struct Utils
 {

  /*! function. getPickLine
      desc.
          Equivalent to Ogre::Camera::getCameraToViewportRay but with an beastie::Line instead of Ogre::Ray
      args.
          Ogre::Camera* -- Camera to raycast from
          float x       -- Relative X (0..1) coordinate.
          float y       -- Relative Y (0..1) coordinate.
      return.
          Line -- The line
  */
  static Line getCameraToViewportRay(Ogre::Camera*, float x, float y);

  /*! function. rayToLine
      desc.
          Converts an Ogre::Ray with an optional lineLength into a beastie::Line
      args.
          const Ogre::Ray& ray -- Ray to convert
          Ogre::Real lineLength -- Line length (Default: 100)
      return.
          Line -- Converted line.
  */
  static Line rayToLine(const Ogre::Ray& ray, Ogre::Real lineLength = 100);

  /*! function. toString
      desc.
           Turns a beastie::ShapeType enum into an Ogre::String
      args.
          ShapeType -- ShapeType
      return.
          Ogre::String -- The String.
  */
  static inline Ogre::String toString(ShapeType type)
    {
     if (type == beastie::ShapeType_Point)
      return std::string("Point");
     else if (type == beastie::ShapeType_Line)
      return std::string("Line");
     else if (type == beastie::ShapeType_Triangle)
      return std::string("Triangle");
     else if (type == beastie::ShapeType_Plane)
      return std::string("Plane");
     else if (type == beastie::ShapeType_DynamicMesh)
      return std::string("DynamicMesh");
     return std::string("Unknown");
    }

  /*! function. meshToVector
      desc.
          Converts a Ogre::Mesh into a std::vector of NormalisedTriangles
      arg.
          const Ogre::MeshPtr& mesh -- Mesh to convert.
          std::vector<NormalisedTriangle>& -- Reference to a vector to write to.
  */
  static void  meshToVector(const Ogre::MeshPtr& mesh, std::vector<NormalisedTriangle>& vec);

  /*! function. uniqueID
      desc.
          Returns an unique incremental integer.
  */
  static unsigned int uniqueID();
  
  private:
   
   static unsigned int sUniqueID;
   
 };

 /*! class. Tests
     desc.
         Collection of optimised functions for collision tests.
 */
 struct Tests
 {

   /*! function. intersection
       desc.
           Point Vs Point intersection.
       args.
           Point* - Point a.
           Point* - Point b.
           Interesection& - Reference to intersection to write results to.
   */
   static void intersection(Point*,          Point*, Intersection&);

   /*! function. intersection
       desc.
           Line Vs Point intersection.
       args.
           Line* - Line.
           Point* - Point.
           Interesection& - Reference to intersection to write results to.
   */
   static void intersection(Line*,           Point*, Intersection&);

   /*! function. intersection
       desc.
           Triangle Vs Point intersection.
       args.
           Triangle* - Triangle.
           Point* - Point.
           Interesection& - Reference to intersection to write results to.
   */
   static void intersection(Triangle*,       Point*, Intersection&);

   /*! function. intersection
       desc.
           Plane Vs Point intersection.
       args.
           Plane* - Triangle.
           Point* - Point.
           Interesection& - Reference to intersection to write results to.
   */
   static void intersection(Plane*,          Point*, Intersection&);

   /*! function. intersection
       desc.
           DynamicMesh Vs Point intersection.
       args.
           DynamicMesh* - Dynamic Mesh.
           Point* - Point.
           Interesection& - Reference to intersection to write results to.
   */
   static void intersection(DynamicMesh*,    Point*, Intersection&);

   /*! function. intersection
       desc.
           Point Vs Line intersection.
       args.
           Point* - Point.
           Line* - Line.
           Interesection& - Reference to intersection to write results to.
   */
   static void intersection(Point*,          Line*, Intersection&);

   /*! function. intersection
       desc.
           Point Vs Line intersection.
       args.
           Line* - Line a.
           Line* - Line b.
           Interesection& - Reference to intersection to write results to.
   */
   static void intersection(Line*,           Line*, Intersection&);

   /*! function. intersection
       desc.
           Triangle Vs Line intersection.
       args.
           Triangle* - Triangle.
           Line* - Line.
           Interesection& - Reference to intersection to write results to.
   */
   static void intersection(Triangle*,       Line*, Intersection&);

   /*! function. intersection
       desc.
           Plane Vs Line intersection.
       args.
           Plane* - Plane.
           Line* - Line.
           Interesection& - Reference to intersection to write results to.
   */
   static void intersection(Plane*,          Line*, Intersection&);

   /*! function. intersection
       desc.
           Dynamic Mesh Vs Line intersection.
       args.
           DynamicMesh* - Dynamic Mesh.
           Line* - Line.
           Interesection& - Reference to intersection to write results to.
   */
   static void intersection(DynamicMesh*,    Line*, Intersection&);

   /*! function. intersection
       desc.
           Point Vs Triangle intersection.
       args.
           Point* - Point.
           Triangle* - Triangle.
           Interesection& - Reference to intersection to write results to.
   */
   static void intersection(Point*,          Triangle*, Intersection&);

   /*! function. intersection
       desc.
           Line Vs Triangle intersection.
       args.
           Line* - Line.
           Triangle* - Triangle.
           Interesection& - Reference to intersection to write results to.
   */
   static void intersection(Line*,           Triangle*, Intersection&);

   /*! function. intersection
       desc.
           Triangle Vs Triangle intersection.
       args.
           Triangle* - Triangle a.
           Triangle* - Triangle b.
           Interesection& - Reference to intersection to write results to.
   */
   static void intersection(Triangle*,       Triangle*, Intersection&);

   /*! function. intersection
       desc.
           Plane Vs Triangle intersection.
       args.
           Plane* - Plane.
           Triangle* - Triangle.
           Interesection& - Reference to intersection to write results to.
   */
   static void intersection(Plane*,          Triangle*, Intersection&);

   /*! function. intersection
       desc.
           DynamicMesh Vs Triangle intersection.
       args.
           DynamicMesh* - Dynamic Mesh.
           Triangle* - Triangle.
           Interesection& - Reference to intersection to write results to.
   */
   static void intersection(DynamicMesh*,    Triangle*, Intersection&);

   /*! function. intersection
       desc.
           Point Vs Plane intersection.
       args.
           Point* - Point.
           Plane* - Plane.
           Interesection& - Reference to intersection to write results to.
   */
   static void intersection(Point*,          Plane*, Intersection&);

   /*! function. intersection
       desc.
           Line Vs Plane intersection.
       args.
           Line* - Line.
           Plane* - Plane.
           Interesection& - Reference to intersection to write results to.
   */
   static void intersection(Line*,           Plane*, Intersection&);

   /*! function. intersection
       desc.
           Triangle Vs Plane intersection.
       args.
           Triangle* - Triangle.
           Plane* - Plane.
           Interesection& - Reference to intersection to write results to.
   */
   static void intersection(Triangle*,       Plane*, Intersection&);

   /*! function. intersection
       desc.
           Plane Vs Plane intersection.
       args.
           Plane* - Plane a.
           Plane* - Plane b.
           Interesection& - Reference to intersection to write results to.
   */
   static void intersection(Plane*,          Plane*, Intersection&);

   /*! function. intersection
       desc.
           DynamicMesh Vs Plane intersection.
       args.
           DynamicMesh* - Dynamic Mesh.
           Plane* - Plane.
           Interesection& - Reference to intersection to write results to.
   */
   static void intersection(DynamicMesh*,    Plane*, Intersection&);

   /*! function. intersection
       desc.
           Point Vs Dynamic Mesh intersection.
       args.
           Point* - Point.
           DynamicMesh* - Dynamic Mesh.
           Interesection& - Reference to intersection to write results to.
   */
   static void intersection(Point*,          DynamicMesh*, Intersection&);

   /*! function. intersection
       desc.
           Line Vs Dynamic Mesh intersection.
       args.
           Line* - Line.
           DynamicMesh* - Dynamic Mesh.
           Interesection& - Reference to intersection to write results to.
   */
   static void intersection(Line*,           DynamicMesh*, Intersection&);

   /*! function. intersection
       desc.
           Triangle Vs Dynamic Mesh intersection.
       args.
           Triangle* - Triangle.
           DynamicMesh* - Dynamic Mesh.
           Interesection& - Reference to intersection to write results to.
   */
   static void intersection(Triangle*,       DynamicMesh*, Intersection&);

   /*! function. intersection
       desc.
           Plane Vs Dynamic Mesh intersection.
       args.
           Plane* - Plane.
           DynamicMesh* - Dynamic Mesh.
           Interesection& - Reference to intersection to write results to.
   */
   static void intersection(Plane*,          DynamicMesh*, Intersection&);

   /*! function. intersection
       desc.
           Point Vs Dynamic Mesh intersection.
       args.
           DynamicMesh* - Dynamic Mesh a.
           DynamicMesh* - Dynamic Mesh b.
           Interesection& - Reference to intersection to write results to.
   */
   static void intersection(DynamicMesh*,    DynamicMesh*, Intersection&);
   
 };

 /*! class. Shape
     desc.
          Abstract Shape class that all beastie shapes inherit from.
 */
 class Shape
 {
   
  public:

   /*! function. getShapeType
       desc.
           Get the type of shape as a ShapeType enum.
       return.
           ShapeType - The type of shape.
   */
   inline virtual ShapeType                getShapeType() const = 0;

   /*! function. intersection
       desc.
           Performs an intersection test with this shape with another.
       args.
           Shape* -- Shape to test against.
       return.
           Intersection -- Result of intersection.
   */
   virtual Intersection                    intersection(Shape*) = 0;

   /*! function. asPoint
       desc.
            Casts shape into a Point
       note.
            If the shape isn't a Point, a NULL pointer is returned.
       return.
            Point* -- The Point or NULL.
   */
   inline virtual Point*                   asPoint() { return 0; }

   /*! function. isPoint
       desc.
           Is this shape a Point?
       return.
           bool - If the shape is a Point or not.
   */
   inline virtual bool                     isPoint() const { return false; } 

   /*! function. asLine
       desc.
            Casts shape into a Line
       note.
            If the shape isn't a Line, a NULL pointer is returned.
       return.
            Line* -- The Line or NULL.
   */
   inline virtual Line*                    asLine() { return 0; }

   /*! function. isLine
       desc.
           Is this shape a Line?
       return.
           bool - If the shape is a Line or not.
   */
   inline virtual bool                     isLine() const { return false; } 

   /*! function. asTriangle
       desc.
            Casts shape into a Triangle
       note.
            If the shape isn't a Triangle, a NULL pointer is returned.
       return.
            Triangle* -- The Triangle or NULL.
   */
   inline virtual Triangle*                asTriangle() { return 0; }

   /*! function. isTriangle
       desc.
           Is this shape a Triangle?
       return.
           bool - If the shape is a Triangle or not.
   */
   inline virtual bool                     isTriangle() const { return false; } 

   /*! function. asPlane
       desc.
            Casts shape into a Plane
       note.
            If the shape isn't a Plane, a NULL pointer is returned.
       return.
            Plane* -- The Plane or NULL.
   */
   inline virtual Plane*                   asPlane() { return 0; }

   /*! function. isPlane
       desc.
           Is this shape a Plane?
       return.
           bool - If the shape is a Plane or not.
   */
   inline virtual bool                     isPlane() const { return false; } 

   /*! function. asDynamicMesh
       desc.
            Casts shape into a DynamicMesh
       note.
            If the shape isn't a DynamicMesh, a NULL pointer is returned.
       return.
            DynamicMesh* -- The DynamicMesh or NULL.
   */
   inline virtual DynamicMesh*             asDynamicMesh() { return 0; }

   /*! function. isDynamicMesh
       desc.
           Is this shape a DynamicMesh?
       return.
           bool - If the shape is a DynamicMesh or not.
   */
   inline virtual bool                     isDynamicMesh() const { return false; } 
   
   /*! function. getShapeID
       desc.
           Get the unique identifier for this shape.
       return.
           unsigned int - The Shape ID
   */
   inline unsigned int                     getShapeID() const    { return mShapeID; }
   
  protected:
   
   unsigned int                            mShapeID;
 };

 /*! class. Point
     desc.
         Represents a single point in 3D space, using an Ogre::Vector3 as an attribute.
 */
 class Point : public Shape
 {
   
   friend struct Utils;
   friend struct Tests;
   friend class VisualRenderer;
   
   public:

    /*! constructor. Point
        desc.
            Constructor to give seperate components for the position.
        note.
            This is the default constructor.
        args.
            Ogre::Real X -- X coordinate (Default: 0)
            Ogre::Real Y -- Y coordinate (Default: 0)
            Ogre::Real Z -- Z coordinate (Default: 0)
    */
    inline Point(Ogre::Real X = 0, Ogre::Real Y = 0, Ogre::Real Z = 0)
    : mPosition(X, Y, Z)
      {
       mShapeID = Utils::uniqueID();
      }

    /*! constructor. Point
        desc.
            Constructor to give the position of the Point as an Ogre::Vector3
        args.
            const Ogre::Vector3& position -- Position of the point.
    */
    inline Point(const Ogre::Vector3& position)
    : mPosition(position)
      {
       mShapeID = Utils::uniqueID();
      }

    /*! function. getShapeType
        desc.
            Get the type of shape as a ShapeType enum.
        return.
            ShapeType - The type of shape.
    */
    inline ShapeType getShapeType() const
      {
       return ShapeType_Point;
      }

    /*! function. getPosition
        desc.
            Get the position of the Point as an Ogre::Vector3
        return.
            Ogre::Vector3 - The position of the Point.
    */
    inline Ogre::Vector3 getPosition() const
      {
       return mPosition;
      }

    /*! function. setPosition
        desc.
            Set the position of the Point from an Ogre::Vector3
        args.
            const Ogre::Vector3& position - The new position of the point.
    */
    inline void setPosition(const Ogre::Vector3& position)
       {
        mPosition = position;
       }

    /*! function. asPoint
        desc.
             Casts shape into a Point
        return.
             Point* -- The Point.
    */
    inline Point* asPoint()
      {
       return static_cast<Point*>(this);
      }

    /*! function. isPoint
        desc.
            Is this shape a Point?
        return.
            bool - If the shape is a Point or not.
    */
    inline bool isPoint() const
      {
       return true;
      }

    /*! function. intersection
        desc.
            Performs an intersection test with this shape with another.
        args.
            Shape* -- Shape to test against.
        return.
            Intersection -- Result of intersection.
    */
    inline Intersection  intersection(Shape* shape)
      {
        Intersection hit;
        if (shape->getShapeID() == mShapeID)
          {
           hit.hit = true;
           hit.position = mPosition;
          }
        else if (shape->isPoint())
          Tests::intersection(shape->asPoint(), this, hit);
        else if (shape->isLine())
          Tests::intersection(shape->asLine(), this, hit);
        else if (shape->isTriangle())
          Tests::intersection(shape->asTriangle(), this, hit);
        else if (shape->isPlane())
          Tests::intersection(shape->asPlane(), this, hit);
        else if (shape->isDynamicMesh())
          Tests::intersection(shape->asDynamicMesh(), this, hit);
        return hit;
      }

  protected:

    Ogre::Vector3 mPosition;

 };
 
 /*! class. Line
     desc.
         Single line in 3D space, represented by a origin, directional and length components.
 */
 class Line : public Shape
 {
   
   friend struct Utils;
   friend struct Tests;
   friend class VisualRenderer;
   
   public:
    
    /*! constructor. Line
        desc.
            Constructor to give seperate components for the origin, direction and length.
        note.
            This is the default constructor.
        args.
            Ogre::Real oX -- Origin X coordinate (Default: 0)
            Ogre::Real oY -- Origin Y coordinate (Default: 0)
            Ogre::Real oZ -- Origin Z coordinate (Default: 0)
            Ogre::Real dX -- Direction X coordinate (Default: 0)
            Ogre::Real dY -- Direction Y coordinate (Default: 0)
            Ogre::Real dZ -- Direction Z coordinate (Default: 0)
            Ogre::Real length -- Length of the line (Default: 0)
        note.
            Direction should be normalised before passing on as an argument.
    */
    inline Line(
      Ogre::Real oX = 0, Ogre::Real oY = 0, Ogre::Real oZ = 0,
      Ogre::Real dX = 0, Ogre::Real dY = 0, Ogre::Real dZ = 0,
      Ogre::Real length = 0)
      : mOrigin(oX, oY, oZ),
        mDirection(dX, dY, dZ),
        mLength(length)
      {
       mShapeID = Utils::uniqueID();
      }
    
    /*! constructor. Line
        desc.
            Constructor to give seperate components for the position, direction and length.
        note.
            This is the default constructor.
        args.
            const Ogre::Vector3& position -- Position of the line
            const Ogre::Vector3& direction -- Direction of the line
            Ogre::Real length -- Length of the line
        note.
            Direction should be normalised before passing on as an argument.
    */
    inline Line(
      const Ogre::Vector3& origin,
      const Ogre::Vector3& direction,
      Ogre::Real length)
      :  mOrigin(origin),
         mDirection(direction),
         mLength(length)
      {
       mShapeID = Utils::uniqueID();
      }

    /*! function. getShapeType
        desc.
            Get the type of shape as a ShapeType enum.
        return.
            ShapeType - The type of shape.
    */
    inline ShapeType getShapeType() const
      {
       return ShapeType_Line;
      }

    /*! function. getOrigin
        desc.
            Get the origin of the Line as an Ogre::Vector3
        return.
            Ogre::Vector3 - The origin of Line.
    */
    inline Ogre::Vector3 getOrigin() const
      {
       return mOrigin;
      }

    /*! function. getDirection
        desc.
            Get the direction of the Line as an Ogre::Vector3
        return.
            Ogre::Vector3 - The direction of Line.
    */
    inline Ogre::Vector3 getDirection() const
      {
       return mDirection;
      }

    /*! function. getLength
        desc.
            Get the length of the Line as an Ogre::Real
        return.
            Ogre::Real - The length of Line.
    */
    inline Ogre::Real getLength() const
      {
       return mLength;
      }

    /*! function. getOtherPosition
        desc.
            Calculate the other position of the line using the direction and length.
        return.
            Ogre::Vector3 - The calculated position.
    */
    inline Ogre::Vector3 getOtherPosition() const
      {
       return mOrigin + (mDirection * mLength);
      }

    /*! function. setOrigin
        desc.
            Set the origin of the Line from an Ogre::Vector3
        args.
            const Ogre::Vector3& origin -- New origin of Line.
    */
    inline void setOrigin(const Ogre::Vector3& origin)
      {
       mOrigin = origin;
      }

    /*! function. setOrigin
        desc.
            Set the origin of the Line from three seperate Ogre::Real components.
        args.
            Ogre::Real X -- New X origin of Line.
            Ogre::Real Y -- New Y origin of Line.
            Ogre::Real Z -- New Z origin of Line.
    */
    inline void setOrigin(Ogre::Real X, Ogre::Real Y, Ogre::Real Z)
      {
       mOrigin.x = X;
       mOrigin.y = Y;
       mOrigin.z = Z;
      }

    /*! function. setDirection
        desc.
            Set the direction of the Line from an Ogre::Vector3
        args.
            const Ogre::Vector3& direction -- New direction of Line.
        note.
            Direction should be normalised before passing on as an argument.
    */
    inline void setDirection(const Ogre::Vector3& direction)
      {
       mDirection = direction;
      }

    /*! function. setDirection
        desc.
            Set the direction of the Line from three seperate Ogre::Real components.
        args.
            Ogre::Real X -- New X direction of Line.
            Ogre::Real Y -- New Y direction of Line.
            Ogre::Real Z -- New Z direction of Line.
        note.
            Direction should be normalised before passing on as an argument.
    */
    inline void setDirection(Ogre::Real X, Ogre::Real Y, Ogre::Real Z)
      {
       mDirection = Ogre::Vector3(X,Y,Z);
      }

    /*! function. setLength
        desc.
            Set the length of the Line from an Ogre::Real components.
        args.
            Ogre::Real length -- New length of Line.
        note.
            Length of the line should be at least zero in length.
    */
    inline void setLength(Ogre::Real length)
      {
       mLength = length;
      }

    /*! function. asLine
        desc.
             Casts shape into a Line
        return.
             Line* -- The Line.
    */
    inline Line* asLine() 
      {
       return static_cast<Line*>(this);
      }

    /*! function. isLine
        desc.
            Is this shape a Line?
        return.
            bool - If the shape is a Line or not.
    */
    inline bool isLine() const
      {
       return true;
      }

    /*! function. intersection
        desc.
            Performs an intersection test with this shape with another.
        args.
            Shape* -- Shape to test against.
        return.
            Intersection -- Result of intersection.
    */
    inline Intersection  intersection(Shape* shape)
      {
        Intersection hit;
        if (shape->getShapeID() == mShapeID)
          {
           hit.hit = true;
           hit.position = mOrigin;
          }
        else if (shape->isPoint())
          Tests::intersection(shape->asPoint(), this, hit);
        else if (shape->isLine())
          Tests::intersection(shape->asLine(), this, hit);
        else if (shape->isTriangle())
          Tests::intersection(shape->asTriangle(), this, hit);
        else if (shape->isPlane())
          Tests::intersection(shape->asPlane(), this, hit);
        else if (shape->isDynamicMesh())
          Tests::intersection(shape->asDynamicMesh(), this, hit);
        return hit;
      }

  protected:

    Ogre::Vector3 mOrigin, mDirection;
    Ogre::Real    mLength;

 };

  /*! class. Triangle
      desc.
          Single triangle in 3D space, represented by three vertices and a normal.
 */
 class Triangle : public Shape
 {
   
   friend struct Utils;
   friend struct Tests;
   friend class VisualRenderer;
   
   public:
        
    /*! constructor. Triangle
        desc.
            Constructor that sets the triangle size to zero.
        note.
            This is the default constructor.
    */
    inline Triangle()
      {
        mTriangle.zero();
        mShapeID = Utils::uniqueID();
      }

    /*! constructor. Triangle
        desc.
            Constructor to give three vertices and auto-calculate the normal.
        args.
            const Ogre::Vector3& a -- Vertex A
            const Ogre::Vector3& b -- Vertex B
            const Ogre::Vector3& c -- Vertex C
    */
    inline Triangle(
      const Ogre::Vector3& a, const Ogre::Vector3& b,
      const Ogre::Vector3& c)
      {
       mTriangle.a = a;
       mTriangle.b = b;
       mTriangle.c = c;
       mTriangle.normalise();
       mShapeID = Utils::uniqueID();
      }
    
    /*! constructor. Triangle
        desc.
            Constructor to give three vertices and the normal.
        args.
            const Ogre::Vector3& a -- Vertex A
            const Ogre::Vector3& b -- Vertex B
            const Ogre::Vector3& c -- Vertex C
            const Ogre::Vector3& normal -- Face normal
    */
    inline Triangle(
      const Ogre::Vector3& a,
      const Ogre::Vector3& b,
      const Ogre::Vector3& c,
      const Ogre::Vector3& normal)
      {
       mTriangle.a = a;
       mTriangle.b = b;
       mTriangle.c = c;
       mTriangle.n = normal;
       mShapeID = Utils::uniqueID();
      }

    /*! constructor. Triangle
        desc.
            Constructor to give three vertices and auto-calculate the normal.
        args.
            const Ogre::Real aX -- Vertex A, X component
            const Ogre::Real aY -- Vertex A, Y component
            const Ogre::Real aZ -- Vertex A, Z component
            const Ogre::Real bX -- Vertex B, X component
            const Ogre::Real bY -- Vertex B, Y component
            const Ogre::Real bZ -- Vertex B, Z component
            const Ogre::Real cX -- Vertex C, X component
            const Ogre::Real cY -- Vertex C, Y component
            const Ogre::Real cZ -- Vertex C, Z component
    */
    inline Triangle(
      Ogre::Real aX, Ogre::Real aY, Ogre::Real aZ,
      Ogre::Real bX, Ogre::Real bY, Ogre::Real bZ,
      Ogre::Real cX, Ogre::Real cY, Ogre::Real cZ)
      {
       mTriangle.a.x = aX;
       mTriangle.a.y = aY;
       mTriangle.a.z = aZ;
       mTriangle.b.x = bX;
       mTriangle.b.y = bY;
       mTriangle.b.z = bZ;
       mTriangle.c.x = cX;
       mTriangle.c.y = cY;
       mTriangle.c.z = cZ;
       mTriangle.normalise();
       mShapeID = Utils::uniqueID();
      }

    /*! constructor. Triangle
        desc.
            Constructor to give three vertices and the normal.
        args.
            const Ogre::Real aX -- Vertex A, X component
            const Ogre::Real aY -- Vertex A, Y component
            const Ogre::Real aZ -- Vertex A, Z component
            const Ogre::Real bX -- Vertex B, X component
            const Ogre::Real bY -- Vertex B, Y component
            const Ogre::Real bZ -- Vertex B, Z component
            const Ogre::Real cX -- Vertex C, X component
            const Ogre::Real cY -- Vertex C, Y component
            const Ogre::Real cZ -- Vertex C, Z component
            const Ogre::Real nX -- Normal, X component
            const Ogre::Real nY -- Normal, Y component
            const Ogre::Real nZ -- Normal, Z component
    */
    inline Triangle(
      Ogre::Real aX, Ogre::Real aY, Ogre::Real aZ,
      Ogre::Real bX, Ogre::Real bY, Ogre::Real bZ,
      Ogre::Real cX, Ogre::Real cY, Ogre::Real cZ,
      Ogre::Real nX, Ogre::Real nY, Ogre::Real nZ)
      {
       mTriangle.a.x = aX;
       mTriangle.a.y = aY;
       mTriangle.a.z = aZ;
       mTriangle.b.x = bX;
       mTriangle.b.y = bY;
       mTriangle.b.z = bZ;
       mTriangle.c.x = cX;
       mTriangle.c.y = cY;
       mTriangle.c.z = cZ;
       mTriangle.n.x = nX;
       mTriangle.n.y = nY;
       mTriangle.n.z = nZ;
       mShapeID = Utils::uniqueID();
      }

    /*! function. getShapeType
        desc.
            Get the type of shape as a ShapeType enum.
        return.
            ShapeType - The type of shape.
    */
    inline ShapeType     getShapeType() const
      {
       return ShapeType_Triangle;
      }

    /*! function. transform
        desc.
            Transform the triangle from a position, orientation and scale, and calculates the new normal.
        args.
          const Ogre::Vector3& position -- Relative position
          const Ogre::Quaternion& orientation -- Relative orientation (Default: Ogre::Quaternion::IDENTITY)
          const Ogre::Vector3& scale -- Relative scale (Default: beastie::Vector3_ONE)
    */
    inline void transform(
          const Ogre::Vector3& position,
          const Ogre::Quaternion& orientation = Ogre::Quaternion::IDENTITY,
          const Ogre::Vector3& scale = beastie::Vector3_ONE)
      {
       mTriangle = mTriangle.transform(position, orientation, scale);
      }

    /*! function. setVertexA
        desc.
            Set the position of A vertex.
        args.
            const Ogre::Vector3& a -- New Position of vertex A.
    */
    inline void setVertexA(const Ogre::Vector3& a)
      {
       mTriangle.a = a;
      }

    /*! function. setVertexB
        desc.
            Set the position of B vertex.
        args.
            const Ogre::Vector3& b -- New Position of vertex B.
    */
    inline void setVertexB(const Ogre::Vector3& b)
      {
       mTriangle.b = b;
      }

    /*! function. setVertexC
        desc.
            Set the position of C vertex.
        args.
            const Ogre::Vector3& c -- New Position of vertex C.
    */
    inline void          setVertexC(const Ogre::Vector3& c)
      {
       mTriangle.c = c;
      }

    /*! function. setNormal
        desc.
            Set the normal of the triangle face.
        note.
            Must be normalised (Ogre::Vector3::normalise) before use.
        args.
            const Ogre::Vector3& normal -- New face normal.
    */
    inline void          setNormal(const Ogre::Vector3& normal)
      {
       mTriangle.n = normal;
      }

    /*! function. getVertexA
        desc.
            Get the position of A vertex.
        return.
            Ogre::Vector3 -- Position of vertex A.
    */
    inline Ogre::Vector3 getVertexA() const
      {
       return mTriangle.a;
      }

    /*! function. getVertexB
        desc.
            Get the position of B vertex.
        return.
            Ogre::Vector3 -- Position of vertex B.
    */
    inline Ogre::Vector3 getVertexB() const
      {
       return mTriangle.b;
      }

    /*! function. getVertexC
        desc.
            Get the position of C vertex.
        return.
            Ogre::Vector3 -- Position of vertex C.
    */
    inline Ogre::Vector3 getVertexC() const
      {
       return mTriangle.c;
      }

    /*! function. getNormal
        desc.
            Get the face normal.
        return.
            Ogre::Vector3 -- Face Normal
    */
    inline Ogre::Vector3 getNormal() const
      {
       return mTriangle.n;
      }

    /*! function. calculateNormal
        desc.
            Calculates the normal of the face from the three vertices.
    */
    inline void calculateNormal()
      {
       mTriangle.normalise();
      }

    /*! function. asTriangle
        desc.
             Casts shape into a Triangle
        return.
             Triangle* -- The Triangle.
    */
    inline Triangle* asTriangle()
      {
       return static_cast<Triangle*>(this);
      }

    /*! function. isTriangle
        desc.
            Is this shape a Triangle?
        return.
            bool - If the shape is a Triangle or not.
    */
    inline bool isTriangle() const
      {
       return true;
      }

    /*! function. intersection
        desc.
            Performs an intersection test with this shape with another.
        args.
            Shape* -- Shape to test against.
        return.
            Intersection -- Result of intersection.
    */
    inline Intersection  intersection(Shape* shape)
      {
        Intersection hit;
        if (shape->getShapeID() == mShapeID)
          {
           hit.hit = true;
           hit.position = mTriangle.a;
          }
        else if (shape->isPoint())
          Tests::intersection(shape->asPoint(), this, hit);
        else if (shape->isLine())
          Tests::intersection(shape->asLine(), this, hit);
        else if (shape->isTriangle())
          Tests::intersection(shape->asTriangle(), this, hit);
        else if (shape->isPlane())
          Tests::intersection(shape->asPlane(), this, hit);
        else if (shape->isDynamicMesh())
          Tests::intersection(shape->asDynamicMesh(), this, hit);
        return hit;
      }
    
   protected:
    
    NormalisedTriangle mTriangle;
    
 };

  /*! class. Plane
      desc.
          An infinite plane in 3D-space.
 */
 class Plane : public Shape
 {
   
   friend struct Utils;
   friend struct Tests;
   friend class VisualRenderer;
   
   public:

    /*! constructor. Plane
        desc.
            Constructor to give Plane normal and direction (from 0,0,0 along the normal).
        note.
            This is the default constructor.
        args.
            Ogre::Real nX -- Plane Normal, X (Default: 0)
            Ogre::Real nY -- Plane Normal, Y (Default: 1)
            Ogre::Real nZ -- Plane Normal, Z (Default: 0)
            Ogre::Real -- Distance from 0,0,0 along normal. (Default: 0)
    */
    inline Plane(
      Ogre::Real nX = 0,
      Ogre::Real nY = 1,
      Ogre::Real nZ = 0,
      Ogre::Real distance = 0)
      : mNormal(nX,nY,nZ),
        mDistance(distance)
      {
       mShapeID = Utils::uniqueID();
      }

    /*! constructor. Plane
        desc.
            Constructor to give Plane normal and direction (from 0,0,0 along the normal).
        note.
            This is the default constructor.
        args.
            const Ogre::Vector3& normal -- Plane normal (Must be normalised before use)
            Ogre::Real -- Distance from 0,0,0 along normal.
    */
    inline Plane(
      const Ogre::Vector3& normal,
      Ogre::Real distance)
      : mNormal(normal),
        mDistance(distance)
      {
       mShapeID = Utils::uniqueID();
      }

    /*! function. getShapeType
        desc.
            Get the type of shape as a ShapeType enum.
        return.
            ShapeType - The type of shape.
    */
    inline ShapeType getShapeType() const
      {
       return ShapeType_Plane;
      }

    /*! function. setNormal
        desc.
            Set the normal of the plane.
        note.
            Must be normalised (Ogre::Vector3::normalise) before use.
        args.
            const Ogre::Vector3& normal -- New face normal.
    */
    inline void setNormal(const Ogre::Vector3& normal)
      {
       mNormal = normal;
      }

    /*! function. getNormal
        desc.
            Get the plane normal.
        return.
            Ogre::Vector3 -- Plane normal.
    */
    inline Ogre::Vector3 getNormal() const
      {
       return mNormal;
      }

    /*! function. setDistance
        desc.
            Set the distance of the plane
        note.
            Distance must be zero or positive.
        args.
            Ogre::Real distance -- Distance from the center, along the normal.
    */
    inline void setDistance(Ogre::Real distance)
      {
       mDistance = distance;
      }

    /*! function. getDistance
        desc.
            Get the plane distance.
        return.
            Ogre::Real -- Plane distance.
    */
    inline Ogre::Real getDistance() const
      {
       return mDistance;
      }

    /*! function. asPlane
        desc.
             Casts shape into a plane
        return.
             Plane* -- The Plane.
    */
    inline Plane* asPlane()
      {
       return static_cast<Plane*>(this);
      }

    /*! function. isPlane
        desc.
            Is this shape a Plane?
        return.
            bool - If the shape is a Plane or not.
    */
    inline bool isPlane() const
      {
       return true;
      }

    /*! function. intersection
        desc.
            Performs an intersection test with this shape with another.
        args.
            Shape* -- Shape to test against.
        return.
            Intersection -- Result of intersection.
    */
    inline Intersection  intersection(Shape* shape)
      {
        Intersection hit;
        if (shape->getShapeID() == mShapeID)
          {
           hit.hit = true;
           hit.position = mNormal * mDistance;
          }
        else if (shape->isPoint())
          Tests::intersection(shape->asPoint(), this, hit);
        else if (shape->isLine())
          Tests::intersection(shape->asLine(), this, hit);
        else if (shape->isTriangle())
          Tests::intersection(shape->asTriangle(), this, hit);
        else if (shape->isPlane())
          Tests::intersection(shape->asPlane(), this, hit);
        else if (shape->isDynamicMesh())
          Tests::intersection(shape->asDynamicMesh(), this, hit);
        return hit;
      }
    
    
  protected:
    
    Ogre::Vector3 mNormal;
    
    Ogre::Real    mDistance;
    
 };

 /* struct. Triangles
    desc.
        Array of NormalisedTriangles with Garbage Collection.
 */
 struct Triangles
 {

  friend class Mesh;

  /*! constructor. Triangles
      desc.
          Default Constructor, with the size being 0.
  */
  inline Triangles()
    : mTriangles(0),
      mSize(0)
      { // empty constructor.
      }

  /*! destructor. Triangles
  */
  inline ~Triangles()
    {
     if (mTriangles && mSize)
      delete mTriangles;
    }

  /*! function. reserve
      desc.
          Resize the array, without preserving the data.
      args.
          size_t count -- New array size (in number of triangles).
  */
  inline void reserve(size_t count)
  {
   if (count > mSize)
   {
    if (mTriangles != 0)
     delete mTriangles;
    mTriangles = new NormalisedTriangle[count];
    mSize = count;
   }
  }

  /*! function. size
      desc.
          Get the size of the array.
      return.
          size_t -- Size of the array.
  */
  inline size_t size() const
  {
   return mSize;
  }

  /*! variable. operator[]
      desc.
          Element access operator.
      args.
          size_t n -- Element to access, must be at least Triangles::size
      return.
          NormalisedTriangle& -- Triangle at n
  */
  inline NormalisedTriangle& operator[](size_t n) const
  {
   return mTriangles[n];
  }

  private:
   
   NormalisedTriangle* mTriangles;
   
   unsigned int        mSize;
   
 };

 /*! class. Mesh
     desc.
         Optimised mesh for the DynamicMesh and StaticMesh classes. Meshes can be shared between DynamicMesh and StaticMeshes.
 */
 class Mesh
 {

  public:

   /*! constructor. Mesh
       desc.
           Constructor to convert an Ogre Mesh from.
       args.
           const Ogre::MeshPtr& mesh -- Mesh to convert from.
   */
   inline Mesh(const Ogre::MeshPtr& mesh)
     {
      beastie::Utils::meshToVector(mesh, mTriangles);
     }

   /*! constructor. Mesh
       desc.
           Constructor to copy a collection of NormalisedTriangles from.
       args.
           const std::vector<NormalisedTriangle>& triangle -- Triangles to copy from.
   */
   inline Mesh(const std::vector<NormalisedTriangle>& triangles)
     {
      mTriangles.reserve(triangles.size());
      std::copy(triangles.begin(), triangles.end(), mTriangles.begin());
     }

   /*! function. copyTriangles
       desc.
           Copy from this mesh into a Triangles class.
       args.
           Triangles& triangles -- Reference of Triangles to copy to.
   */
   inline void copyTriangles(Triangles& triangles)
     {
      triangles.reserve(mTriangles.size());
      std::copy(mTriangles.begin(), mTriangles.end(), triangles.mTriangles);
     }

   /*! function. transformTriangles
       desc.
           Transform the triangles from this mesh into a Triangles class.
       args.
           Triangles& triangles -- Reference of Triangles to copy to.
           const Ogre::Vector3& position -- Relative positions.
           const Ogre::Quaternion& orientation -- Relative orientation.
           const Ogre::Vector3& scale -- Relative scale.
   */
   inline void transformTriangles(
     Triangles& triangles,
     const Ogre::Vector3& position,
     const Ogre::Quaternion& orientation,
     const Ogre::Vector3& scale)
     {
      triangles.reserve(mTriangles.size());
      for (unsigned int i=0;i < mTriangles.size();i++)
       triangles.mTriangles[i] = mTriangles[i].transform(position, orientation, scale);
     }

  protected:

   std::vector<NormalisedTriangle> mTriangles;

 };

 /*! class. DynamicMesh
     desc.
         Collection of triangles that can be moved around in the Scene. 
     note.
         For performance reasons you should limit your triangle count; the low tens is a good start.
 */
 class DynamicMesh : public Shape
 {
   
   friend struct Utils;
   friend struct Tests;
   friend class VisualRenderer;
   
   public:

    /*! constructor. DynamicMesh
    */
    inline DynamicMesh(
      beastie::Mesh* mesh,
      const Ogre::Vector3& position = Ogre::Vector3::ZERO,
      const Ogre::Quaternion& orientation = Ogre::Quaternion::IDENTITY,
      const Ogre::Vector3& scale = beastie::Vector3_ONE)
      : mPosition(position),
        mOrientation(orientation),
        mScale(scale)
      {
       mShapeID = Utils::uniqueID();setMesh(mesh);
      }

    /*! function. getShapeType
        desc.
            Get the type of shape as a ShapeType enum.
        return.
            ShapeType - The type of shape.
    */
    inline ShapeType getShapeType() const
      {
       return ShapeType_DynamicMesh;
      }

    /*! function. getPosition
        desc.
            Get the DynamicMesh position.
        return.
            Ogre::Vector3 -- The position.
    */
    inline Ogre::Vector3 getPosition() const
      {
       return mPosition;
      }

    /*! function. getOrientation
        desc.
            Get the DynamicMesh orientation.
        return.
            Ogre::Quaternion -- The orientation.
    */
    inline Ogre::Quaternion getOrientation() const
      {
       return mOrientation;
      }

    /*! function. getScale
        desc.
            Get the DynamicMesh scale.
        return.
            Ogre::Vector3 -- The scale.
    */
    inline Ogre::Vector3 getScale() const
      {
       return mScale;
      }

    /*! function. setPosition
        desc.
            Set the position of the DynamicMesh
        args.
            const Ogre::Vector3& position -- Position of the DynamicMesh.
    */
    inline void setPosition(const Ogre::Vector3& position)
      {
       mPosition = position;
       mNeedsUpdate = true;
      }

    /*! function. setOrientation
        desc.
            Set the orientation of the DynamicMesh
        args.
            const Ogre::Quaternion& orientation -- Orientation of the DynamicMesh.
    */
    inline void setOrientation(const Ogre::Quaternion& orientation)
      {
       mOrientation = orientation;
       mNeedsUpdate = true;
      }

    /*! function. setScale
        desc.
            Set the scale of the DynamicMesh
        args.
            const Ogre::Vector3& scale -- Scale of the DynamicMesh.
    */
    inline void setScale(const Ogre::Vector3& scale)
      {
       mScale = scale;
       mNeedsUpdate = true;
      }

    /*! function. setMesh
        desc.
            Set the mesh to use from the DynamicMesh
        args.
            beastie::Mesh* -- Mesh to use.
    */
    inline void setMesh(beastie::Mesh* mesh)
      {
       mMesh = mesh;
       mesh->transformTriangles(mTriangles, mPosition, mOrientation, mScale);
       mNeedsUpdate = false;
      }

    /*! function. getMesh
        desc.
            Get the mesh that this mesh is using.
        return.
            beastie::Mesh* -- The mesh is using.
    */
    inline beastie::Mesh* getMesh()
      {
       return mMesh;
      }

    /*! function. asDynamicMesh
        desc.
             Casts shape into a DynamicMesh
        return.
             DynamicMesh* -- The DynamicMesh.
    */
    inline DynamicMesh* asDynamicMesh()
      {
       return static_cast<DynamicMesh*>(this);
      }

    /*! function. isDynamicMesh
        desc.
            Is this shape a DynamicMesh?
        return.
            bool - If the shape is a DynamicMesh or not.
    */
    inline bool isDynamicMesh() const
      {
       return true;
      }

    /*! function. intersection
        desc.
            Performs an intersection test with this shape with another.
        args.
            Shape* -- Shape to test against.
        return.
            Intersection -- Result of intersection.
    */
    inline Intersection  intersection(Shape* shape)
      {
        Intersection hit;
        if (shape->getShapeID() == mShapeID)
          {
           hit.hit = true;
           hit.position = mPosition;
          }
        else if (shape->isPoint())
          Tests::intersection(shape->asPoint(), this, hit);
        else if (shape->isLine())
          Tests::intersection(shape->asLine(), this, hit);
        else if (shape->isTriangle())
          Tests::intersection(shape->asTriangle(), this, hit);
        else if (shape->isPlane())
          Tests::intersection(shape->asPlane(), this, hit);
        else if (shape->isDynamicMesh())
          Tests::intersection(shape->asDynamicMesh(), this, hit);
        return hit;
      }

   protected:

    Mesh*                 mMesh;
    Triangles             mTriangles;
    Ogre::Vector3         mPosition,
                          mScale;
    Ogre::Quaternion      mOrientation;
    Ogre::AxisAlignedBox  mAABB;
    bool                  mNeedsUpdate;

 };

 /*! class. VisualRenderer
     desc.
         Renders shapes as a Ogre::Manual Object for debugging.
 */
 class VisualRenderer
 {
   
  public:

   /*! constructor. VisualRenderer
       desc. 
            Constructor with pre-made ManualObject to use. 
       note.
            By default the first sub-mesh will use the "BaseWhiteNoLighting" material, if you want to 
            use a custom material, then start a new sub-mesh with your material as a OT_LINE_LIST, thereafter
            the VisualDebugger will use that, instead of starting a new one.
   */
   VisualRenderer(
    Ogre::ManualObject* manualObject)
    : mManualObject(manualObject)
   { // empty constructor
   }

   /*! function. beginRender
       desc.
           Start rendering of the debugger, call before renderShape.
   */
   void beginRender()
   {
    if (mManualObject->getNumSections() == 0)
    {
     mManualObject->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_LINE_LIST);
     mManualObject->setDynamic(true);
    }
    else
     mManualObject->beginUpdate(0);
   }

   /*! function. renderShape
       desc.
           Render a single shape, call per as many shapes you want to render.
       note.
           Remember to beginRender before rendering, and endRender when you have finished!
       args.
           Shape* shape -- Shape to render.
   */
   void renderShape(Shape* shape)
   {
    if (shape == 0)
     return;
    else if (shape->isPoint())
    {
     Point* point = shape->asPoint();
#ifndef BEASTIE_VISUALDEBUGGER_SIMPLE
     pos_col(point->mPosition.x-0.1f, point->mPosition.y, point->mPosition.z,   VisualDebuggerPointColour);
     pos_col(point->mPosition.x+0.1f, point->mPosition.y, point->mPosition.z,   VisualDebuggerPointColour);
     pos_col(point->mPosition.x, point->mPosition.y, point->mPosition.z-0.1f,   VisualDebuggerPointColour);
     pos_col(point->mPosition.x, point->mPosition.y, point->mPosition.z+0.1f,   VisualDebuggerPointColour);
     pos_col(point->mPosition.x, point->mPosition.y-0.1f, point->mPosition.z,   VisualDebuggerPointColour);
     pos_col(point->mPosition.x, point->mPosition.y+0.1f, point->mPosition.z,   VisualDebuggerPointColour);
#else
     pos_col(point->mPosition, VisualDebuggerPointColour);
     pos_col(point->mPosition.x, point->mPosition.y+Ogre::Real(0.001), point->mPosition.z, VisualDebuggerPointColour);
#endif
    }
    else if (shape->isLine())
    {
     Line* line = shape->asLine();
     pos_col(line->mOrigin, VisualDebuggerLineColour);
     pos_col(line->mOrigin + (line->mDirection * line->mLength), VisualDebuggerLineColour);
    }
    else if (shape->isPlane())
    {
     Plane* plane = shape->asPlane();
     pos_col(plane->mNormal * plane->mDistance, VisualDebuggerPlaneColour);
     pos_col(plane->mNormal * (plane->mDistance + 1), VisualDebuggerPlaneColour);
    }
    else if (shape->isTriangle())
    {
     Triangle* triangle = shape->asTriangle();
     pos_col(triangle->mTriangle.a, VisualDebuggerTriangleColour);
     pos_col(triangle->mTriangle.b, VisualDebuggerTriangleColour);
     pos_col(triangle->mTriangle.b, VisualDebuggerTriangleColour);
     pos_col(triangle->mTriangle.c, VisualDebuggerTriangleColour);
     pos_col(triangle->mTriangle.c, VisualDebuggerTriangleColour);
     pos_col(triangle->mTriangle.a, VisualDebuggerTriangleColour);
    }
   }

   /*! function. endRender
       desc.
           End rendering of shapes.
   */
   void endRender()
   {
    mManualObject->end();
   }
   
  protected:
   
   inline void pos_col(const Ogre::Vector3& vec, const Ogre::ColourValue& col)
   {
    mManualObject->position(vec);
    mManualObject->colour(col);
   }
   
   inline void pos_col(float x, float y, float z, const Ogre::ColourValue& col)
   {
    mManualObject->position(x,y,z);
    mManualObject->colour(col);
   }

   Ogre::ManualObject*  mManualObject;
   
 };
 
}

#endif
