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

Beastie is a C++ based simplified collision detection library for the [Ogre3D graphics engine](http://www.ogre3d.org), and it is neatly packaged in a single header, `beastie.h`. "Beastie" should be always pronounced in a Glaswegian accent. 

It only detects collisions and does not respond to them as you would typically encounter in a more fully featured physics engine or more advanced collision library.
However the detections are accurate and are optimised, building some response code on top of beastie shouldn't be to difficult, an example of some of this is in the "wee.cpp" test application that can be found at the beastie Git repository.

Beastie only uses Ogre3D as a dependency, and is developed with Ogre 1.7 in mind; but should work with any 1.x version.

Using Beastie
-------------

All of Beastie functions are contained in the `beastie` namespace, any functions are put in Utils or Tests based on their use. All classes and functions use the camelCase notation, with the exception that class names have an uppercased first character.

To include it in your project, simply copy the `beastie.h` files into your project directory, then include them in your project, making sure you correctly include and link to Ogre. For compliation speed reasons you may want to `#include` beastie in your Static headers file.

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

To help against floating point precision, the maximum radius of the point is beastie::epsilon is included in most intersection functions.

    a.setPosition(Ogre::Vector3::ZERO);
    b.setPosition(epsilon,0,0);
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

Plane
-----

Planes are infinite planes in 3D Space. Planes are useful for ground, or as safety nets to prevent anything falling into infinity.

A Plane is made from two properties;

* Normal (`Ogre::Vector3`)
* Distance (`Ogre::Real`)

### Normal

Normal is the face of the place.

*/
#ifndef crivens_a_wee_beastie_in_tha_cludgie_h
#define crivens_a_wee_beastie_in_tha_cludgie_h


#include "OGRE/OgreVector3.h"
#include "OGRE/OgreQuaternion.h"
#include <utility>
#include "math.h"

namespace beastie_impl
{
 typedef Ogre::Vector3     Vector;
 typedef Ogre::Quaternion  Quaternion;
 typedef Ogre::Real        Real;
 class Shape;
 template<int=0> class PointT;
 template<int=0> class LineT;
 template<int=0> class PlaneT;
 template<int=0> class SphereT;
 template<int=0> class BoxT;
 template<int=0> struct TriangleT;
 template<int=0> class DynamicMeshT;
 template<int=0> class StaticMeshT;
 template<int=0> struct ContactPairT;
 template<typename> struct ContainerT;
 typedef beastie_impl::PointT<>             Point;
 typedef beastie_impl::LineT<>              Line;
 typedef beastie_impl::PlaneT<>             Plane;
 typedef beastie_impl::SphereT<>            Sphere;
 typedef beastie_impl::BoxT<>               Box;
 typedef beastie_impl::TriangleT<>          Triangle;
 typedef beastie_impl::StaticMeshT<>        StaticMesh;
 typedef beastie_impl::DynamicMeshT<>       DynamicMesh;
 typedef beastie_impl::ContactPairT<>       ContactPair;
 typedef beastie_impl::Container<Triangle>  TriangleList;
}

namespace beastie
{
 typedef beastie_impl::Vector      Vector;
 typedef beastie_impl::Quaternion  Quaternion;
 typedef beastie_impl::Real        Real;
 namespace constants
 {
  static const Real epsilon            =  std::numeric_limits<Real>::epsilon();
  static const Real epsilonSquared     =  epsilon * epsilon;
  static const Real negEpsilon         = -epsilon;
  static const Real negEpsilonSquared  = -epsilonSquared;
  static const Real pi                 =  3.141592653589793f;
  static const Real piSquared          = pi * pi;
  static const Real e                  =  2.718281828459045f;
 }
 namespace preferences
 {
  static const Real particleRadius     = beastie::constants::epsilonSquared;
 }
 namespace functions
 {
  inline Real squared(Real x)
  {
   return x * x;
  }
  inline Real squareRoot(Real x)
  {
   return sqrtf(x);
  }
  inline Real absolute(Real x)
  {
   return x < 0 ? -x : x;
  }
  inline void normalise(const Vector& a, const Vector& b, const Vector& c, Vector& n)
  {
   n = ((a - b).crossProduct(a - c)).normalise();
  }
 }
 typedef beastie_impl::PointT<>        Point;
 typedef beastie_impl::LineT<>         Line;
 typedef beastie_impl::PlaneT<>        Plane;
 typedef beastie_impl::SphereT<>       Sphere;
 typedef beastie_impl::BoxT<>          Box;
 typedef beastie_impl::TriangleT<>     Triangle;
 typedef beastie_impl::StaticMeshT<>   StaticMesh;
 typedef beastie_impl::DynamicMeshT<>  DynamicMesh;
 typedef beastie_impl::ContactPairT<>  ContactPair;
}

namespace beastie_impl
{

 template<int> struct ContactPairT
 {
  inline ContactPairT()
    : hit(false)
    { // empty constructor.
    }
  
  bool     hit;
  Vector   normal;
  Vector   position;
  Real     depth;
  Shape*   first;
  Shape*   second;
 };
 
 
 template<int> struct TriangleT
 {
  inline TriangleT()
    : a(Vector::ZERO),
      b(Vector::ZERO),
      c(Vector::ZERO),
      n(Vector::ZERO)
    { // empty constructor
    }
  
  inline void normalise()
    {
     beastie::functions::normalise(a, b, c, n);
    }
    
  Vector a, b, c, n;
  
 };

 template<typename T> struct Container
 {
  
 };

 class Shape
 {
   
  public:
   
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
    //! Infinite plane.
    ShapeType_Plane       = (1<<2),
    //! Sphere.
    ShapeType_Sphere      = (1<<3),
    //! Box.
    ShapeType_Box         = (1<<4),
    //! Collection of triangles that move around constantly.
    ShapeType_DynamicMesh = (1<<5),
    //! Optimised collection of triangles that never moved.
    ShapeType_StaticMesh  = (1<<6),
    //! All shapes filter
    AllShapes             = ShapeType_Point       | 
                            ShapeType_Line        | 
                            ShapeType_Plane       | 
                            ShapeType_Sphere      | 
                            ShapeType_Box         | 
                            ShapeType_DynamicMesh | 
                            ShapeType_StaticMesh
   };
 
   /*! function. getShapeType
       desc.
           Get the type of shape as a ShapeType enum.
       return.
           ShapeType - The type of shape.
   */
   inline virtual ShapeType                getShapeType() const = 0;

   /*! function. collide
       desc.
           Performs an collision test with this shape with another.
       args.
           Shape* -- Shape to test against.
           CollisionPair& -- CollisionPair
       return.
           bool -- If it collided or not.
   */
   virtual bool                           collide(Shape*, ContactPair&) = 0;

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

   /*! function. asSphere
       desc.
            Casts shape into a Sphere
       note.
            If the shape isn't a Sphere, a NULL pointer is returned.
       return.
            Plane* -- The Sphere or NULL.
   */
   inline virtual Sphere*                  asSphere() { return 0; }

   /*! function. isSphere
       desc.
           Is this shape a Sphere?
       return.
           bool - If the shape is a Sphere or not.
   */
   inline virtual bool                     isSphere() const { return false; } 

 }; // PointT<>
 
 /*! class. PointT
     desc.
         Single point in 3D-Space.
 */
 template<int> class PointT : public Shape
 {
  
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
    inline PointT(Real X = 0, Real Y = 0, Real Z = 0)
      : mPosition(X, Y, Z)
      { // empty constructor
      }

    /*! constructor. Point
        desc.
            Constructor to give the position of the Point as a Vector
        args.
            const Vector& position -- Position of the point.
    */
    inline PointT(const Vector& position)
      : mPosition(position)
      { // empty constructor
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
            Get the position of the Point as a Vector
        return.
            Vector - The position of the Point.
    */
    inline Vector getPosition() const
      {
       return mPosition;
      }

    /*! function. setPosition
        desc.
            Set the position of the Point from an Vector
        args.
            const Vector& position - The new position of the point.
    */
    inline void setPosition(const Vector& position)
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

    /*! function. collide
        desc.
            Performs an collision test with this shape with another.
        args.
            Shape* -- Shape to test against.
            CollisionPair& -- Full results of the collision.
        return.
            bool -- If it collided or not.
    */
    inline bool  collide(Shape* shape, ContactPair& pair)
      {
        if (shape->isPoint())
         pointCollide(shape->asPoint(), pair);
        else if (shape->isLine())
         lineCollide(shape->asLine(), pair);
        return pair.hit;
      }

    /*! function. pointCollide
        desc.
            Performs an collision test with this shape with another point.
        args.
            PointT<>* -- Point to test against.
            CollisionPair& -- Full results of the collision.
        return.
            bool -- If it collided or not.
    */
    inline bool pointCollide(PointT<>* other, ContactPair& pair)
      {
       Real d = other->mPosition.squaredDistance(mPosition);
       if (d < beastie::preferences::particleRadius)
       {
        pair.hit = true;
        pair.depth = beastie::functions::squareRoot(d);
        pair.normal = (other->mPosition - mPosition).normalise();
        pair.position = mPosition + (pair.normal * pair.depth);
        pair.first = this;
        pair.second = other;
       }
       return pair.hit;
      }

    /*! function. lineCollide
        desc.
            Performs an collision test with this shape with a line.
        args.
            LineT<>* -- Point to test against.
            CollisionPair& -- Full results of the collision.
        return.
            bool -- If it collided or not.
    */
    inline bool lineCollide(LineT<>* line, ContactPair& pair)
      {
       if (line->isValid() == false)
        return false;
       
       Ogre::Vector3 v = mPosition - line->getOrigin();
       const Ogre::Vector3 s = line->getOtherPosition() - line->getOrigin();
       Ogre::Real dot = v.dotProduct(s) / (beastie::functions::squared(line->getLength()));
       v = v - (s * dot);
       
       if (v.squaredLength() >= beastie::constants::epsilonSquared)
        return false;
       
       pair.hit = true;
       pair.depth = v.length();
       pair.normal = -line->getDirection();
       pair.position = mPosition;
       pair.first = this;
       pair.second = line;
       
       return true;
      }

  protected:
    Vector mPosition;

 };
 

 
 /*! class. LineT
     desc.
         Single line in 3D-Space. Represented by an origin, direction and distance.
 */
 template<int> class LineT : public Shape
 {
  
  public:

    /*! constructor. Line
        desc.
            Constructor to give seperate components for the origin, direction and length.
        note.
            This is the default constructor.
        args.
            Real oX -- Origin X coordinate (Default: 0)
            Real oY -- Origin Y coordinate (Default: 0)
            Real oZ -- Origin Z coordinate (Default: 0)
            Real dX -- Direction X coordinate (Default: 0)
            Real dY -- Direction Y coordinate (Default: 0)
            Real dZ -- Direction Z coordinate (Default: 0)
            Real length -- Length of the line (Default: 0)
        note.
            Direction should be normalised before passing on as an argument.
    */
    inline LineT(
      Real oX = 0, Real oY = 0, Real oZ = 0,
      Real dX = 0, Real dY = 0, Real dZ = 0,
      Real length = 0)
      : mOrigin(oX, oY, oZ),
        mDirection(dX, dY, dZ),
        mLength(length)
      { // empty constructor
      }
    
    /*! constructor. Line
        desc.
            Constructor to give seperate components for the origin, direction and length.
        note.
            This is the default constructor.
        args.
            const Vector& position -- Position of the line
            const Vector& direction -- Direction of the line
            Real length -- Length of the line
        note.
            Direction should be normalised before passing on as an argument.
    */
    inline LineT(
      const Vector& origin,
      const Vector& direction,
      Real length)
      :  mOrigin(origin),
         mDirection(direction),
         mLength(length)
      { // empty constructor
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
            Get the origin of the Line as an Vector
        return.
            Vector - The origin of Line.
    */
    inline Vector getOrigin() const
      {
       return mOrigin;
      }

    /*! function. getDirection
        desc.
            Get the direction of the Line as an Vector
        return.
            Vector - The direction of Line.
    */
    inline Vector getDirection() const
      {
       return mDirection;
      }

    /*! function. getLength
        desc.
            Get the length of the Line as an Real
        return.
            Real - The length of Line.
    */
    inline Real getLength() const
      {
       return mLength;
      }

    /*! function. getOtherPosition
        desc.
            Calculate the other position of the line using the direction and length.
        return.
            Vector - The calculated position.
    */
    inline Vector getOtherPosition() const
      {
       return mOrigin + (mDirection * mLength);
      }

    /*! function. setOrigin
        desc.
            Set the origin of the Line from an Vector
        args.
            const Vector& origin -- New origin of Line.
    */
    inline void setOrigin(const Vector& origin)
      {
       mOrigin = origin;
      }

    /*! function. setOrigin
        desc.
            Set the origin of the Line from three seperate Real components.
        args.
            Real X -- New X origin of Line.
            Real Y -- New Y origin of Line.
            Real Z -- New Z origin of Line.
    */
    inline void setOrigin(Real X, Real Y, Real Z)
      {
       mOrigin.x = X;
       mOrigin.y = Y;
       mOrigin.z = Z;
      }

    /*! function. setDirection
        desc.
            Set the direction of the Line from an Vector
        args.
            const Vector& direction -- New direction of Line.
        note.
            Direction should be normalised before passing on as an argument.
    */
    inline void setDirection(const Vector& direction)
      {
       mDirection = direction;
      }

    /*! function. setDirection
        desc.
            Set the direction of the Line from three seperate Real components.
        args.
            Real X -- New X direction of Line.
            Real Y -- New Y direction of Line.
            Real Z -- New Z direction of Line.
        note.
            Direction should be normalised before passing on as an argument.
    */
    inline void setDirection(Real X, Real Y, Real Z)
      {
       mDirection = Vector(X,Y,Z);
      }

    /*! function. setLength
        desc.
            Set the length of the Line from an Real components.
        args.
            Real length -- New length of Line.
        note.
            Length of the line should be at least zero in length.
    */
    inline void setLength(Real length)
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

    /*! function. isValid
        desc.
            Does the line have a length and direction?
        return.
            bool - If the line is valid or not.
    */
    inline bool isValid() const
    {
     return mLength > 0 && mDirection != Vector::ZERO;
    }

    /*! function. collide
        desc.
            Performs an collision test with this shape with another.
        args.
            Shape* -- Shape to test against.
            CollisionPair& -- Full results of the collision.
        return.
            bool -- If it collided or not.
    */
    inline bool  collide(Shape* shape, ContactPair& pair)
      {
        if (shape->isPoint())
         shape->asPoint()->lineCollide(this, pair);
        else if (shape->isLine())
         lineCollide(shape->asLine(), pair);
        return pair.hit;
      }

    /*! function. lineCollide
        desc.
            Performs an collision test with this shape with a line.
        args.
            LineT<>* -- Point to test against.
            CollisionPair& -- Full results of the collision.
        return.
            bool -- If it collided or not.
    */
    inline bool lineCollide(LineT<>* other, ContactPair& pair)
      {
      }

  protected:

    Vector mOrigin, mDirection;
    Real    mLength;

 }; // LineT<>


 /*! class. PlaneT
     desc.
          Infinite plane in 3D-space described by a normal and distance.
 */
 template<int> class PlaneT : public Shape
 {
  
  public:
 
    /*! constructor. Plane
        desc.
            Constructor to give Plane normal and direction (from 0,0,0 along the normal).
        note.
            This is the default constructor.
        args.
            Real nX -- Plane Normal, X (Default: 0)
            Real nY -- Plane Normal, Y (Default: 1)
            Real nZ -- Plane Normal, Z (Default: 0)
            Real distance -- Distance from 0,0,0 along normal. (Default: 0)
    */
    inline PlaneT(
      Real nX = 0,
      Real nY = 1,
      Real nZ = 0,
      Real distance = 0)
      : mNormal(nX,nY,nZ),
        mDistance(distance)
      { // empty constructor
      }

    /*! constructor. Plane
        desc.
            Constructor to give Plane normal and direction (from 0,0,0 along the normal).
        note.
            This is the default constructor.
        args.
            const Vector& normal -- Plane normal (Must be normalised before use)
            Real -- Distance from 0,0,0 along normal.
    */
    inline PlaneT(
      const Vector& normal,
      Real distance)
      : mNormal(normal),
        mDistance(distance)
      { // empty constructor
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
            Must be normalised (Vector::normalise) before use.
        args.
            const Vector& normal -- New face normal.
    */
    inline void setNormal(const Vector& normal)
      {
       mNormal = normal;
      }

    /*! function. getNormal
        desc.
            Get the plane normal.
        return.
            Vector -- Plane normal.
    */
    inline Vector getNormal() const
      {
       return mNormal;
      }

    /*! function. setDistance
        desc.
            Set the distance of the plane
        note.
            Distance must be zero or positive.
        args.
            Real distance -- Distance from the center, along the normal.
    */
    inline void setDistance(Real distance)
      {
       mDistance = distance;
      }

    /*! function. getDistance
        desc.
            Get the plane distance.
        return.
            Real -- Plane distance.
    */
    inline Real getDistance() const
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

  protected:

    Vector mNormal;
    Real    mDistance;

 }; // PlaneT<>


}

#endif
// beastie.h
