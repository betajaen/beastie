/*! page. Beastie
    
    p. **Note** -- The most recent copy of beastie can be found at http://github.com/betajaen/beastie
    
    h1. Beastie
    
    h2. Software Licence
       
    bc.. Copyright (c) 2010 Robin Southern
         
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
       
    h2. What is Beastie?
      
    p. Beastie is a C++ based simplified collision detection library for the 
       "Ogre3D graphics engine":(http://www.ogre3d.org), and it only comes in two
       files, "beastie.h" and "beastie.cpp". "Beastie" should be always pronounced
       in a Glaswegian accent. 
       
    p. It only detects collisions and does not respond to them as you would typically
       encounter in a more fully featured physics engine or more advanced collision library.
       However the detections are accurate and are optimised, building some response code on
       top of beastie shouldn't be to difficult, an example of some of this is in the "wee.cpp"
       test application that can be found at the beastie GIT repository.
       
    p. Beastie only uses Ogre3D as a dependency, and beastie is developed with OGRE 1.7
       in mind, so you should use at least that.
      
    h2. Using Beastie
      
    p. All of Beastie functions are contained in the @beastie@ namespace, any functions are
       put in Utils or Tests based on their use. All classes and functions use the camelCase
       notation, with the exception that classes uppercase the first character.
    
    p. To include it in your project, simply copy the "beastie.cpp" and "beastie.h" files into
       your project directory, then include them in your project. Beastie is developed with
       Ogre 1.7 in mind; but should work with any 1.x version.
    
    h3. Shapes
    
    p. All beastie shapes inherit from the @Shape@ class, so they can be upcasted into
       a Shape if you wish to store them as pointers, or intersect them via the Shape::intersect
       function.
      
    bc.. // Straight to the point.
         Point point1 = Point(1,2,3);
         // And as very abstract.
         Shape* point2 = new Point(3,2,1);
         point2->intersect(&point);
     
    p. Shapes have a collection of functions to tell you what type of Shape it is, if handled as a
       pointer.
     
    * Shape::getShapeType
    * Shape::isPoint
    * Shape::isLine __and so on__
    
    p. A Shape can be downcasted into a specific shape; Shape::asPoint, Shape::asLine... These
       functions should be checked with Shape::isPoint, Shape::isLine... before downcasting,
       in the event of a wrong is function used, a null pointer is returned.
    
    bc. Line* line = shape->asLine();
    
    h3. Points
    
    p. Points represent a single point in 3D space and can be used for particles or other
       wee things.
       
    p. Points have a single position property (Point::setPosition, Point::getPosition), its
       range is from [-FLT_MAX, -FLT_MAX, -FLT_MAX] to [FLT_MAX, FLT_MAX, FLT_MAX]
    
    bc.. // Intersection test
         Point a = Point(Ogre::Vector3::ZERO);
         Point b = Point(Ogre::Vector3::ZERO);
         a.intersect(&b); // .hit = true
         b->setPosition(1,0,0);
         a.intersect(&b); // .hit = false
    
    p. To help against floating point precision the maximum radius of the point is beastie::eps,
       is included in most intersection functions.
    
    bc.. a.setPosition(Ogre::Vector3::ZERO);
         b.setPosition(eps,0,0);
         a.intersection(&b); // .hit = true
    
    h3. Lines
    
    p. Lines are single line in 3D space with a specified length. There uses are plentyfull, but can
       be used as a substitute for Raycasting.
    
    p. Lines are made from three properties:
    
    * a position (Ogre::Vector3)
    * a direction (Ogre::Vector3)
    * a length (Ogre::Real)
    
    p. Positions (Line::setPosition, Line::getPosition) have the same range as a Point's position.
    
    p. Directions (Line::setDirection, Line::getDirection) have the range [-1,-1,-1]...[1,1,1], and
       must be normalised (Ogre::Vector3::normalise)
    
    p. Length (Line::setLength, Line::getDirection) has the range of (0, FLT_MAX].

    p. When intersecting with another shape, lines return the point of intersection against the line,
       the side effect is that lines can be used for ray casting.

    bc. Line  line = Line(Ogre::Vector3(0, 10, 0), Ogre::Vector3::NEGATIVE_UNIT_Y, 20);
        Plane plane = Plane(Ogre::Vector3::UNIT_Y, 0);
        
        line.intersect(&plane); // .hit = true, .position = Ogre::Vector3::ZERO
        
    p. The distance of the raycast can be calculated by the length between the two vectors.
       
    bc. Intersection intersection = line.intersect(&plane);
        
        Ogre::Real rayLength = line.getPosition().distance(intersection.position);
   
*/
#ifndef crivens_a_wee_beastie_in_der_cludgie_h
#define crivens_a_wee_beastie_in_der_cludgie_h

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
  ShapeType_Point,
  //! Line
  ShapeType_Line,
  //! Single triangle and normal.
  ShapeType_Triangle,
  //! Infinite plane.
  ShapeType_Plane,
  //! Collection of triangles that move around constantly.
  ShapeType_DynamicMesh,
  //! Optimised collection of triangles that never moved.
  ShapeType_StaticMesh,
 };
 
 static const unsigned int          BeastieVersion                      = 9;
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
          Transform this triangle by a position, orientation and/or scale.
      args.
         const Ogre::Vector3& position -- Position to translate.
         const Ogre::Quaternion& orientation -- Changed orientation.
         const Ogre::Vector3& scale -- Relative Scale.
      return.
         The transformed NormalisedTriangle.
  */
  inline NormalisedTriangle transform(const Ogre::Vector3& position, const Ogre::Quaternion& orientation, const Ogre::Vector3& scale) const
    {
     NormalisedTriangle out;
     out.a = (orientation * (a * scale)) + position;
     out.b = (orientation * (b * scale)) + position;
     out.c = (orientation * (c * scale)) + position;
     out.n = n;
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
         Represents a single point in 3D space.
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
            ShapeType - The type of shape.
    */
    inline Ogre::Vector3 getPosition() const
      {
       return mPosition;
      }
    
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
       else if (shape->isDynamicMesh())           Tests::intersection(shape->asDynamicMesh(), this, hit);
      return hit;
    }
    
  protected:
    
    Ogre::Vector3 mPosition;
    
 };
 
 class Line : public Shape
 {
   
   friend struct Utils;
   friend struct Tests;
   friend class VisualRenderer;
   
   public:
    
    inline Line(Ogre::Real X = 0,  Ogre::Real  Y = 0, Ogre::Real  Z = 0,
                Ogre::Real dX = 0, Ogre::Real dY = 0, Ogre::Real dZ = 0,
                Ogre::Real length = 0)
      : mPosition(X, Y, Z), mDirection(dX, dY, dZ), mLength(length)             {   mShapeID = Utils::uniqueID(); }
    
    inline Line(const Ogre::Vector3& position, const Ogre::Vector3& direction,
                Ogre::Real length)
      :  mPosition(position), mDirection(direction), mLength(length)            {   mShapeID = Utils::uniqueID(); }
    
    inline ShapeType     getShapeType() const                                   {   return ShapeType_Line;}
    
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
       else if (shape->isDynamicMesh())           Tests::intersection(shape->asDynamicMesh(), this, hit);
      return hit;
    }
    
  protected:
    
    Ogre::Vector3 mPosition, mDirection;
    Ogre::Real    mLength;
    
 };
 
 class Triangle : public Shape
 {
   
   friend struct Utils;
   friend struct Tests;
   friend class VisualRenderer;
   
   public:
    
    inline Triangle()
            :  mA(), mB(), mC(), mNormal()                      {   mShapeID = Utils::uniqueID(); }
    
    inline Triangle(const Ogre::Vector3& a, const Ogre::Vector3& b,
                    const Ogre::Vector3& c)
            :  mA(a), mB(b), mC(c)                              {   mShapeID = Utils::uniqueID(); calculateNormal();  }
    
    inline Triangle(const Ogre::Vector3& a, const Ogre::Vector3& b,
                    const Ogre::Vector3& c, const Ogre::Vector3& normal)
            : mA(a), mB(b), mC(c), mNormal(normal)              {   mShapeID = Utils::uniqueID();  }
    
    inline Triangle(Ogre::Real aX, Ogre::Real aY, Ogre::Real aZ,
             Ogre::Real bX, Ogre::Real bY, Ogre::Real bZ,
             Ogre::Real cX, Ogre::Real cY, Ogre::Real cZ)
            : mA(aX, aY, aZ), mB(bX, bY, bZ), mC(cX, cY, cZ)    {   mShapeID = Utils::uniqueID();calculateNormal();  }
    
    inline Triangle(Ogre::Real aX, Ogre::Real aY, Ogre::Real aZ,
             Ogre::Real bX, Ogre::Real bY, Ogre::Real bZ,
             Ogre::Real cX, Ogre::Real cY, Ogre::Real cZ,
             Ogre::Real nX, Ogre::Real nY, Ogre::Real nZ)
            :  mA(aX, aY, aZ), mB(bX, bY, bZ), mC(cX, cY, cZ),
              mNormal(nX, nY, nZ)                               {  mShapeID = Utils::uniqueID();  }
    
    inline ShapeType     getShapeType() const                   {   return ShapeType_Triangle;}
    
    inline void          setVertexA(const Ogre::Vector3& a)     {   mA = a;}
    
    inline void          setVertexB(const Ogre::Vector3& b)     {   mB = b;}
    
    inline void          setVertexC(const Ogre::Vector3& c)     {   mC = c;}
    
    inline void          setNormal(const Ogre::Vector3& normal) {   mNormal = normal;  }
    
    inline Ogre::Vector3 getVertexA() const                     {   return mA;  }
    
    inline Ogre::Vector3 getVertexB() const                     {   return mB;  }
    
    inline Ogre::Vector3 getVertexC() const                     {   return mC;  }
    
    inline Ogre::Vector3 getNormal() const                      {   return mNormal;  }
    
    inline void          calculateNormal()                      {   mNormal = Ogre::Math::calculateBasicFaceNormal(mA,mB,mC);  }
    
    inline Triangle*     asTriangle()                           {   return static_cast<Triangle*>(this); }
    
    inline bool          isTriangle() const                     {   return true; }
    
    inline Intersection  intersection(Shape* shape)
    {
      Intersection hit;
            if (this == shape)       {hit.hit=true;}
       else if (shape->isPoint())    Tests::intersection(shape->asPoint(), this, hit);
       else if (shape->isLine())     Tests::intersection(shape->asLine(), this, hit);
       else if (shape->isTriangle()) Tests::intersection(shape->asTriangle(), this, hit);
       else if (shape->isPlane())    Tests::intersection(shape->asPlane(), this, hit);
       else if (shape->isDynamicMesh())           Tests::intersection(shape->asDynamicMesh(), this, hit);
      return hit;
    }
    
   protected:
    
    Ogre::Vector3 mA, mB, mC, mNormal;
    
 };
 
 
 class Plane : public Shape
 {
   
   friend struct Utils;
   friend struct Tests;
   friend class VisualRenderer;
   
   public:
    
    Plane(Ogre::Real nX = 0, Ogre::Real nY = 0, Ogre::Real nZ = 0, Ogre::Real distance = 0)
     : mNormal(nX,nY,nZ), mDistance(distance)                   {   mShapeID = Utils::uniqueID();}
    
    Plane(const Ogre::Vector3& normal, Ogre::Real distance)
     : mNormal(normal), mDistance(distance)                     {   mShapeID = Utils::uniqueID();}
    
    inline ShapeType     getShapeType() const                   {   return ShapeType_Plane;}
    
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
       else if (shape->isDynamicMesh())           Tests::intersection(shape->asDynamicMesh(), this, hit);
      return hit;
    }
    
  protected:
    
    Ogre::Vector3 mNormal;
    
    Ogre::Real    mDistance;
    
 };
 
 class Mesh
 {
   
  public:
   
   struct Triangles
   {
    Triangles() : mTriangles(0), mSize(0) {}
   ~Triangles() { if (mTriangles && mSize) delete mTriangles;}
    void reserve(size_t count)
    {
     if (count > mSize)
     {
      if (mTriangles != 0)
       delete mTriangles;
      mTriangles = new NormalisedTriangle[count];
      mSize = count;
     }
    }
    NormalisedTriangle* mTriangles;
    unsigned int        mSize;
   };

   Mesh(const Ogre::MeshPtr& mesh)                               {
                                                                  beastie::Utils::meshToVector(mesh, mTriangles);
                                                                 }
   
   Mesh(const std::vector<NormalisedTriangle>& triangles)        {
                                                                  mTriangles.reserve(triangles.size());
                                                                  std::copy(triangles.begin(), triangles.end(), mTriangles.begin());
                                                                 }
   
   inline void copyTriangles(Triangles& triangles)
   {
    triangles.reserve(mTriangles.size());
    std::copy(mTriangles.begin(), mTriangles.end(), triangles.mTriangles);
   }
   
   inline void transformTriangles(
           Triangles& triangles, const Ogre::Vector3& position,
           const Ogre::Quaternion& orientation, const Ogre::Vector3& scale)
   {
    triangles.reserve(mTriangles.size());
    for (unsigned int i=0;i < mTriangles.size();i++)
     triangles.mTriangles[i] = mTriangles[i].transform(position, orientation, scale);
   }
   
  protected:
   
   std::vector<NormalisedTriangle> mTriangles;
   
 };
 
 class DynamicMesh : public Shape
 {
   
   friend struct Utils;
   friend struct Tests;
   friend class VisualRenderer;
   
   public:
    
    inline DynamicMesh(beastie::Mesh* mesh,
            const Ogre::Vector3& position = Ogre::Vector3::ZERO,
            const Ogre::Quaternion& orientation = Ogre::Quaternion::IDENTITY,
            const Ogre::Vector3& scale = beastie::Vector3_ONE)
            : mPosition(position), mOrientation(orientation), mScale(scale)            {   mShapeID = Utils::uniqueID();setMesh(mesh); }
    
    inline ShapeType            getShapeType() const                                   {   return ShapeType_DynamicMesh;}
        
    inline Ogre::Vector3        getPosition() const                                    {   return mPosition;}
    
    inline Ogre::Quaternion     getOrientation() const                                 {   return mOrientation;}
    
    inline Ogre::Vector3        getScale() const                                       {   return mScale;}
    
    inline void                 setPosition(const Ogre::Vector3& position)             {   mPosition = position;  mNeedsUpdate = true;}
    
    inline void                 setOrientation(const Ogre::Quaternion& orientation)    {   mOrientation = orientation;  mNeedsUpdate = true;}
    
    inline void                 setScale(const Ogre::Vector3& scale)                   {   mScale = scale;  mNeedsUpdate = true;}
    
    inline DynamicMesh*         asDynamicMesh()                                        {   return static_cast<DynamicMesh*>(this); }
    
    inline bool                 isDynamicMesh() const                                  {   return true; }
    
    inline void                 setMesh(Mesh* mesh)
         {   mMesh = mesh;  mesh->transformTriangles(mTriangles, mPosition, mOrientation, mScale); mNeedsUpdate = false; }

    inline Intersection         intersection(Shape* shape)
    {
      Intersection hit;
            if (this == shape)                    {hit.hit=true;}
       else if (shape->isPoint())                 Tests::intersection(shape->asPoint(), this, hit);
       else if (shape->isLine())                  Tests::intersection(shape->asLine(), this, hit);
       else if (shape->isTriangle())              Tests::intersection(shape->asTriangle(), this, hit);
       else if (shape->isPlane())                 Tests::intersection(shape->asPlane(), this, hit);
       else if (shape->isDynamicMesh())           Tests::intersection(shape->asDynamicMesh(), this, hit);
      return hit;
    }
    
   protected:
    
    Mesh*                            mMesh;
    Ogre::Vector3                    mPosition, mScale;
    Ogre::Quaternion                 mOrientation;
    Mesh::Triangles                  mTriangles;
    Ogre::AxisAlignedBox             mAABB;
    bool                             mNeedsUpdate;
 };
 
 class VisualRenderer
 {
   
  public:
   
   VisualRenderer(Ogre::ManualObject* manualObject)
    : mManualObject(manualObject)                            {   }
   
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
     pos_col(line->mPosition, VisualDebuggerLineColour);
     pos_col(line->mPosition + (line->mDirection * line->mLength), VisualDebuggerLineColour);
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
     pos_col(triangle->mA, VisualDebuggerTriangleColour);
     pos_col(triangle->mB, VisualDebuggerTriangleColour);
     pos_col(triangle->mB, VisualDebuggerTriangleColour);
     pos_col(triangle->mC, VisualDebuggerTriangleColour);
     pos_col(triangle->mC, VisualDebuggerTriangleColour);
     pos_col(triangle->mA, VisualDebuggerTriangleColour);
    }
   }
   
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
