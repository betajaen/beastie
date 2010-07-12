// beastie.h, version: 20, date: 12-July-2010

/*



                                                                                  
                                                                                  
           crivvens!                                                              
                      such                                                        
                                                                                  
                            a                                                     
                                                                                  
                               wee                                                
                                                                                  
     88                                                       88              88  
     88                                                ,d     ""              88  
     88                                                88                     88  
     88,dPPYba,    ,adPPYba,  ,adPPYYba,  ,adPPYba,  MM88MMM  88   ,adPPYba,  88  
     88P'    "8a  a8P_____88  ""     `Y8  I8[    ""    88     88  a8P_____88  88  
     88       d8  8PP"""""""  ,adPPPPP88   `"Y8ba,     88     88  8PP"""""""  ""  
     88b,   ,a8"  "8b,   ,aa  88,    ,88  aa    ]8I    88,    88  "8b,   ,aa  aa  
     8Y"Ybbd8"'    `"Ybbd8"'  `"8bbdP"Y8  `"YbbdP"'    "Y888  88   `"Ybbd8"'  88  
                                                                                  
                                                                                  
                                                                                  
                                                                                  
                                                                                  
                                                                                  
                                                                                  
                                                                                  




Software Licence
----------------
                                                                                  
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

What is Beastie?
----------------

Beastie is a C++ based simplified collision detection library for the [Ogre3D graphics engine](http://www.ogre3d.org), and it is neatly packaged in a single header, `beastie.h`. "Beastie" should be always pronounced in a Glaswegian accent. 

It only detects collisions and does not respond to them as you would typically encounter in a more fully featured physics engine or more advanced collision library.
However the detections are accurate and are optimised, building some response code on top of beastie shouldn't be to difficult, an example of some of this is in the "beastie.example.cpp" test application that can be found at the beastie Git repository.

Beastie only uses Ogre3D as a dependency, and is developed with Ogre 1.7 in mind; but should work with any 1.x version.

Using Beastie
-------------

All of beastie classes are contained in the `beastie` namespace. All member functions use the `camelCase` notation where as the class names use the `lower_case_underscore_notation`. There is a private namespace called `beastie::its_a_secret` this is to hold the internal workings of beastie and shouldn't be needed be to used by an application, or it's programmer.

To include beastie in your project, simply copy the `beastie.h` file into your project directory, then include it in your project, making sure you correctly include and link to Ogre. To increase compliation speed you may want to `#include` beastie in your static headers file.

Collisions
----------

Testing for collisions may be done via the `intercept` member function either in; `line`, `plane`, `box` or `collision_tree` classes. Additionally duplicate static functions can be found in the `intersections` class.

    intersection_result result;
    line.intersect(my_plane, result);               // These two lines are interchangable
    intersections::line(my_line, my_plane, result); // based on preference.

The available collisions may be summed up in this table.

    +-------+------+-----+------+------+
    |       | Line | Box | Plane| Tree |
    +-------+------+-----+------+------+
    | Line  | ?    | ?   | ?    | Yes  |
    | Box   | Yes  | ?   | ?    | ?    |
    | Plane | ?    | ?   | ?    | ?    |
    | Tree  | ?    | ?   | ?    |      |
    +-------+------+-----+------+------+

*/

#ifndef crivvens_such_a_wee_beastie_h
#define crivvens_such_a_wee_beastie_h

#include "OGRE/OgreVector3.h"
#include "OGRE/OgreQuaternion.h"
#include "OGRE/OgreRay.h"
#include "OGRE/OgreMath.h"
#include "OGRE/OgreMesh.h"
#include "OGRE/OgreSubMesh.h"
#include "OGRE/OgreMeshManager.h"

// forwards
namespace beastie
{
 namespace its_a_secret
 {
  template<typename> class line_t;
  template<typename> class box_t;
  template<typename> class plane_t;
  template<typename> class mesh_t;
  template<typename> class octree_t;
  template<typename> class node_t;
  template<typename> class collision_tree_t;
  template<typename> struct intersections_t;
  
  typedef octree_t<void> octree;
  typedef node_t<void> node;
  
 } // namespace its_a_secret
 typedef its_a_secret::line_t<void> line;
 typedef its_a_secret::box_t<void> box;
 typedef its_a_secret::plane_t<void> plane;
 typedef its_a_secret::mesh_t<void> mesh;
 typedef its_a_secret::collision_tree_t<void> collision_tree;
 typedef its_a_secret::node_t<void> node;
 typedef its_a_secret::intersections_t<void> intersections;
} // namespace beastie

// actual
namespace beastie
{

struct triangle
{
 Ogre::Vector3 a,b,c,n;
};

struct intersection_result
{
 Ogre::Real     distance;
 Ogre::Vector3  globalPosition;
 Ogre::Vector3  globalNormal;
 mesh*          hitMesh;
 triangle       hitTriangle;
};

static const Ogre::Real eps = std::numeric_limits<Ogre::Real>::epsilon();
static const Ogre::Real epsSquared = eps * eps;
static const Ogre::Real negativeEps = -eps;
static const Ogre::Real negativeEpsSquared = -epsSquared;

namespace its_a_secret
{
 
 Ogre::Real inline absolute(const Ogre::Real& val)
 {
  return fabsf(val);
 }
 
 template<typename> struct intersections_t
 {
  
  static inline bool line(const beastie::line& ln, const beastie::plane& pl, Ogre::Vector3& globalPos)
  {
   Ogre::Real denom = pl.normal().dotProduct(ln.direction());
   
   if (Ogre::Math::Abs(denom) < eps)
    return false;
   
   Ogre::Real nom = pl.normal().dotProduct(ln->origin()) + pl.distance;
   Ogre::Real t   = -(nom/denom);
   
   if (t >= 0 && t <= ln.length())
   {
    globalPos = ln.at(t);
    return true;
   }
   
   return false;
  }
  
  static inline bool line(const beastie::line& ln, const beastie::box& bx, Ogre::Vector3& globalPos)
  {
   return line(ln, bx.aabb(), globalPos);
  }
  
  static inline bool line(const beastie::line& ln, const Ogre::AxisAlignedBox& bx, Ogre::Vector3& globalPos)
  {
   Ogre::Ray ry = ln.ray();
   std::pair<bool, Ogre::Real> result = Ogre::Math::intersects(ry, bx);
   
   if (result.first == false)
    return false;
   
   globalPos = ry.getOrigin() + (ry.getDirection() * result.second);
   
   return true;
  }
  
  static inline bool line(const beastie::line& ln, triangle* tri, Ogre::Real& distance)
  {
   std::pair<bool, float> res = Ogre::Math::intersects(ln.ray(), tri->a, tri->b, tri->c, tri->n, true, false);
   distance = res.second;
   return res.first;
  }
  
  static inline bool line(const beastie::line& ln, mesh* msh, const Ogre::Matrix4& meshGlobalTransformation, Ogre::Real& distance, int& triangleId, Ogre::Vector3& hitPosition)
  {
   // Convert matrix into local space matrix.
   static Ogre::Matrix4 localSpace;
   localSpace = meshGlobalTransformation.inverse();
   
   // Convert line to mesh local space.
   beastie::line local_ln(localSpace * ln.origin(), localSpace * ln.end());
   
   float bestDistance = local_ln.length();
   triangle *bestTriangle = 0;
   
   triangle *tri = msh->begin(), *end = msh->end();
   
   Ogre::Real t, denom, n0, n1, n2, u1, v1, u2, v2, u0, v0, alpha, beta, area, tolerance;
   size_t i0, i1;
   
   while(tri != end)
   {
    denom = (*tri).n.dotProduct(local_ln.direction());
    
    if (denom > beastie::eps)
    {
     tri++;
     continue;
    }
    
    t = (*tri).n.dotProduct((*tri).a - local_ln.origin()) / denom;

    if (t < 0 || t > bestDistance)
    {
     tri++;
     continue;
    }
    
    n0 = absolute((*tri).n[0]);
    n1 = absolute((*tri).n[1]);
    n2 = absolute((*tri).n[2]);
    
    i0 = 1;
    i1 = 2;
    
    if (n1 > n2)
    {
     if (n1 > n0)
      i0 = 0;
    }
    else
    {
     if (n2 > n0)
      i1 = 0;
    }
    
    u0 = t * local_ln.direction()[i0] + local_ln.origin()[i0] - (*tri).a[i0];
    v0 = t * local_ln.direction()[i1] + local_ln.origin()[i1] - (*tri).a[i1];

    u1 = (*tri).b[i0] - (*tri).a[i0];
    v1 = (*tri).b[i1] - (*tri).a[i1];

    u2 = (*tri).c[i0] - (*tri).a[i0];
    v2 = (*tri).c[i1] - (*tri).a[i1];

    alpha = u0 * v2 - u2 * v0;
    beta  = u1 * v0 - u0 * v1;
    area  = u1 * v2 - u2 * v1;
    
    tolerance = beastie::negativeEps * area;
    
    if (area > 0)
    {
     if (alpha < tolerance || beta < tolerance || alpha+beta > area-tolerance)
     {
      tri++;
      continue;
     }
    }
    else
    {
     if (alpha > tolerance || beta > tolerance || alpha+beta < area-tolerance)
     {
      tri++;
      continue;
     }
    }
    
     if (bestTriangle == 0 || t < bestDistance)
     {
      bestTriangle = tri;
      bestDistance = t;
     }

    tri++;
   }
   
   if (bestTriangle == 0)
    return false;
   
   hitPosition = meshGlobalTransformation * local_ln.at(bestDistance);
   distance = ln.origin().distance(hitPosition);
   triangleId = bestTriangle - msh->begin();
   
   return true;
  }

 };
 
 /*! class. line
     desc.
         A single line in 3D space. It is represented by an origin (a point), a direction (normalised vector)
         and a distance (real number).
         
         Lines are typically used with detection of intersections with planes or meshes, as well as raycasting.
     note.
         Line is equivalent to the Ogre::Ray class.
 */
 template<typename> class line_t
 {
  public:
   
   /*! constructor. line
       desc.
           Empty constructor. Line has no direction or length.
   */
   inline line_t() : lineLength(0) {}
   
   /*! constructor. line
       desc.
           Constructor with line origin, direction and length arguments.
       note.
           direction must be normalised (Ogre::Vector3::normaliseCopy)
   */
   inline line_t(const Ogre::Vector3& origin, const Ogre::Vector3& direction, const Ogre::Real& length)
  : lineOrigin(origin), lineDirection(direction), lineLength(length) {}
   
  /*! constructor. line
      desc.
          Constructor that creates a line from an Ogre Ray and length.
  */
  inline line_t(const Ogre::Ray& ray, const Ogre::Real& rayLength)
  : lineOrigin(ray.getOrigin()), lineDirection(ray.getDirection()), lineLength(rayLength) {}
  
  /*! constructor. line
      desc.
          Constructor that creates a line from two points
  */
  inline line_t(const Ogre::Vector3& line_origin, const Ogre::Vector3& line_end)
  : lineOrigin(line_origin)
  {
   lineDirection = (line_end - lineOrigin);
   lineLength = lineDirection.length();
   lineDirection.normalise();
  }

  /*! constructor. origin
      desc.
          Get the line's origin
  */
  inline Ogre::Vector3  origin() const
  {
   return lineOrigin;
  }
  
  /*! constructor. end
      desc.
          Get the line's end position.
  */
  inline Ogre::Vector3 end() const
  {
   return at(lineLength);
  }
  
  /*! constructor. direction
      desc.
          Get the line's direction
  */
  inline Ogre::Vector3  direction() const
  {
   return lineDirection;
  }

  /*! constructor. length
      desc.
          Get the line's length
  */
  inline Ogre::Real  length() const
  {
   return lineLength;
  }

  /*! constructor. at
      desc.
          Get a coordinate from the origin based upon the direction and given distance.
  */
  inline Ogre::Vector3  at(const Ogre::Real& distance) const 
  {
   return lineOrigin + (lineDirection * distance);
  }
  
  inline Ogre::Ray ray() const
  {
   return Ogre::Ray(lineOrigin, lineDirection);
  }

  protected:
   
   Ogre::Vector3 lineOrigin, lineDirection;
   Ogre::Real    lineLength;
   
 };
 
 
 /*! class. box
     desc.
         A box in 3D space. It is axis aligned so it has no specific direction. It is represented by two
         vectors; minimum (the most minimal point of the box) and maximum (the most maxmium point of the box).
         
     note.
         Box is equivalent to the Ogre::AxisAlignedBox
 */
 template<typename> class box_t
 {
   
  public:
   
   box_t()
   : boxMin(0,0,0), boxMax(0,0,0) {}
   
   box_t(const Ogre::Vector3& box_min, const Ogre::Vector3& box_max)
   : boxMin(box_min), boxMax(box_max) {}
   
   const Ogre::Vector3& min() const
   {
    return boxMin;
   }
   
   Ogre::Vector3& min()
   {
    return boxMin;
   }
   
   const Ogre::Vector3& max() const
   {
    return boxMax;
   }
   
   Ogre::Vector3& max()
   {
    return boxMax;
   }

   void min(float x, float y, float z)
   {
    boxMin.x = x;
    boxMin.y = y;
    boxMin.z = z;
   }
   
   void max(float x, float y, float z)
   {
    boxMax.x = x;
    boxMax.y = y;
    boxMax.z = z;
   }
   
   void min(const Ogre::Vector3& new_min)
   {
    boxMin = new_min;
   }
   
   void max(const Ogre::Vector3& new_max)
   {
    boxMax = new_max;
   }
   
   Ogre::AxisAlignedBox aabb() const
   {
    return Ogre::AxisAlignedBox(boxMin, boxMax);
   }
   
  protected:
   
   Ogre::Vector3 boxMin, boxMax;
   
 };
 
 
 /*! class. plane
     desc.
         An infinite plane in 3d space. It is represented by a normal (normalised vector); of the where
         the direction of the face of the plane is pointing, and a distance (real number) of how far
         it is away from the world center (0,0,0)
         
     note.
         Plane is equivalent to the Ogre::Plane
 */
 template<typename> class plane_t
 {
   
  public:
   
   plane_t()
   : planeNormal(0,1,0), planeDistance(0) {}
   
   plane_t(const Ogre::Plane& pl)
   : planeNormal(pl.normal), planeDistance(pl.d) {}
   
   plane_t(const Ogre::Vector3& plane_normal, const Ogre::Real& plane_distance = 0)
   : planeNormal(plane_normal), planeDistance(plane_distance)
   {}
   
   void normal(Ogre::Real x, Ogre::Real y, Ogre::Real z)
   {
    planeNormal.x = x;
    planeNormal.y = y;
    planeNormal.z = z;
   }

   void normal(const Ogre::Vector3& new_normal)
   {
    planeNormal = new_normal;
   }
    
   Ogre::Vector3  normal() const
   {
    return planeNormal;
   }
   
   void distance(const Ogre::Real& new_distance)
   {
    planeDistance = new_distance;
   }
   
   Ogre::Real distance() const
   {
    return planeDistance;
   }
   
  protected:
   
   Ogre::Vector3  planeNormal;
   Ogre::Real     planeDistance;
   
 };

 /*! class. mesh
     desc.
         A optimised mesh in represented by seperate triangles. Many nodes may 
         share a single mesh, because of this all triangles are unmodified, any raycasting or
         other operations done on the mesh must be converted into mesh space before use.
         
     note.
         mesh is equivalent to the Ogre::Mesh
     see.
         collision_tree::getOrLoadMesh
 */
 template<typename> class mesh_t : public Ogre::GeneralAllocatedObject
 {
   
  public:
   
   typedef std::vector<triangle> triangles;
   
   mesh_t(const Ogre::MeshPtr& meshPtr)
   : meshOgreMesh(meshPtr), meshReferences(0), meshTriangles(0), meshNbTriangles(0)
   {
    _inspect();
   }
   
  ~mesh_t()
   {
    if (meshTriangles)
     free(meshTriangles);
   }
   
   Ogre::AxisAlignedBox& getAABB()
   {
    return meshAABB;
   }
   
   inline triangle* begin()
   {
    return meshTriangles;
   }

   inline triangle* end()
   {
    return (meshTriangles + meshNbTriangles);
   }
   
   inline triangle& at(int id)
   {
    return *(meshTriangles + id);
   }
   
   inline size_t   nbTriangles() const
   {
    return meshNbTriangles;
   }

   void _inspect()
   {
    
    Ogre::Vector3* verts = 0;
    unsigned long* indices = 0;
    size_t nbVerts = 0, nbIndices = 0, nbSubMeshIndices = 0;
    bool addedShared = false;
    for (Ogre::ushort i=0;i < meshOgreMesh->getNumSubMeshes(); ++i)
    {
     Ogre::SubMesh* submesh = meshOgreMesh->getSubMesh(i);
     if (submesh->useSharedVertices)
     {
      if (!addedShared)
      {
       nbVerts += meshOgreMesh->sharedVertexData->vertexCount;
       addedShared = true;
      }
     }
     else
     {
      nbVerts += submesh->vertexData->vertexCount;
     }
     nbIndices += submesh->indexData->indexCount;
    }
    
    verts = new Ogre::Vector3[nbVerts];
    indices = new unsigned long[nbIndices];
    
    addedShared = false;
    Ogre::VertexData* vData = 0;
    Ogre::IndexData* iData = 0;
    size_t currentOffset = 0, sharedOffset = 0, nextOffset = 0, indexOffset = 0, offset = 0;

    for (Ogre::ushort i = 0; i < meshOgreMesh->getNumSubMeshes();++i)
    {
      Ogre::SubMesh* submesh = meshOgreMesh->getSubMesh(i);
      vData = submesh->useSharedVertices ? meshOgreMesh->sharedVertexData : submesh->vertexData;
      
      if ((!submesh->useSharedVertices) || (submesh->useSharedVertices && !addedShared))
      {
       if (submesh->useSharedVertices)
       {
        addedShared = true;
        sharedOffset = currentOffset;
       }
       
       const Ogre::VertexElement* posElem = vData->vertexDeclaration->findElementBySemantic(Ogre::VES_POSITION);
       Ogre::HardwareVertexBufferSharedPtr vBuf = vData->vertexBufferBinding->getBuffer(posElem->getSource());
       
       Ogre::uchar* vertex = static_cast<Ogre::uchar*>(vBuf->lock(Ogre::HardwareBuffer::HBL_READ_ONLY));
       float* pos;
       
       for (size_t j=0;j < vData->vertexCount; ++j, vertex += vBuf->getVertexSize())
       {
        posElem->baseVertexPointerToElement(vertex, &pos);
        verts[currentOffset + j].x = pos[0];
        verts[currentOffset + j].y = pos[1];
        verts[currentOffset + j].z = pos[2];
       }
       vBuf->unlock();
       nextOffset += vData->vertexCount;
      }
      
      iData = submesh->indexData;
      nbSubMeshIndices = iData->indexCount;
      
      Ogre::HardwareIndexBufferSharedPtr iBuf = iData->indexBuffer;
      
      unsigned long* indexL = static_cast<unsigned long*>(iBuf->lock(Ogre::HardwareBuffer::HBL_READ_ONLY));
      unsigned short* indexS = reinterpret_cast<unsigned short*>(indexL);
      
      if (iBuf->getType() == Ogre::HardwareIndexBuffer::IT_32BIT)
      {
       for (size_t k = 0; k < nbSubMeshIndices;++k)
        indices[indexOffset++] = indexL[k] + static_cast<unsigned long>(offset);
      }
      else
      {
       for (size_t k = 0; k < nbSubMeshIndices;++k)
        indices[indexOffset++] = static_cast<unsigned long>(indexS[k]) + static_cast<unsigned long>(offset);
      }

     iBuf->unlock();
     currentOffset = nextOffset;
    } // for
    
    if (meshTriangles)
     free(meshTriangles);
    
    meshNbTriangles = nbIndices / 3;
    meshTriangles = (triangle*) malloc(sizeof(triangle) * meshNbTriangles);
    
    meshAABB.setNull();
    meshNextTriangleId = 0;
    
    triangle* tri = meshTriangles;
    for (size_t i=0;i < nbIndices;i+=3)
    {
     tri->a = verts[indices[i]];
     tri->b = verts[indices[i+1]];
     tri->c = verts[indices[i+2]];
     tri->n = Ogre::Math::calculateBasicFaceNormal(tri->a, tri->b, tri->c);
     meshAABB.merge(tri->a);
     meshAABB.merge(tri->b);
     meshAABB.merge(tri->c);
     tri++;
    }
    
    delete indices;
    delete verts;
    
   }
   
    bool raycast(const line& global_ln, const Ogre::Matrix4& transform, intersection_result& int_result)
    {
     
     // Convert to local space.
     Ogre::Matrix4 inverse = transform.inverse();
     line local_ln = line(inverse * global_ln.origin(), inverse * global_ln.end());
     
     
     Ogre::Ray ray = local_ln.ray();
     std::pair<bool, float> pair;
     
     float foundDistance = 0;
     float bestDistance = local_ln.length();
     int bestTriangle = -1;
     
     for (size_t i=0;i < meshTriangles.size();i++)
     {
      
      triangle& tri = meshTriangles[i];
 
      pair = Ogre::Math::intersects(ray, tri.a, tri.b, tri.c, tri.n, true, false);
      
      if (pair.first && (pair.second <= bestDistance))
      {
       bestTriangle = i;
       bestDistance = pair.second;
      }
     
      
     } // for
     
     if (bestTriangle == -1)
      return false;
     
     int_result.distance = bestDistance; /// TODO: This is in local space, has to be converted to world space.
     int_result.globalPosition = local_ln.at(bestDistance);

     int_result.hitMesh = this;
     int_result.hitTriangle = meshTriangles[bestTriangle];
     int_result.hitTriangle.a = transform * int_result.hitTriangle.a;
     int_result.hitTriangle.b = transform * int_result.hitTriangle.b;
     int_result.hitTriangle.c = transform * int_result.hitTriangle.c;
     //int_result.hitTriangle.a = (globalOrientation * int_result.hitTriangle.a) + globalPos;
     //int_result.hitTriangle.b = (globalOrientation * int_result.hitTriangle.b) + globalPos;
     //int_result.hitTriangle.c = (globalOrientation * int_result.hitTriangle.c) + globalPos;
     int_result.globalNormal = int_result.hitTriangle.n; // temp.
     return true;
      
    } //  bool raycast(...)
    

   void _draw(Ogre::ManualObject* obj, const Ogre::Matrix4& transform)
   {
    
    Ogre::Vector3 a, b, c;
    
    for (triangle* it = begin(); it != end(); it++)
    {
     a = transform * (*it).a;
     b = transform * (*it).b;
     c = transform * (*it).c;

     obj->position(a);
     obj->colour(Ogre::ColourValue::Green);
     obj->position(b);
     obj->colour(Ogre::ColourValue::Green);

     obj->position(b);
     obj->colour(Ogre::ColourValue::Green);
     obj->position(c);
     obj->colour(Ogre::ColourValue::Green);

     obj->position(c);
     obj->colour(Ogre::ColourValue::Green);
     obj->position(a);
     obj->colour(Ogre::ColourValue::Green);

    }
    
   }
   
   void _ref()
   {
    meshReferences++;
   }
   
   void _unref()
   {
    meshReferences--;
   }
   
  protected:
   
   Ogre::MeshPtr         meshOgreMesh;
   triangle*             meshTriangles;
   unsigned int          meshNbTriangles;
   
   Ogre::AxisAlignedBox  meshAABB;
   unsigned int          meshNextTriangleId;
   unsigned int          meshReferences;
 };
 
 /*! class. node
     desc.
         A node is a point in 3D space with an attached mesh. It is used for complex collision detection
         for when boxes, planes or lines do not accurately represent the 3D world.
     note.
         mesh is equivalent to the Ogre::SceneNode with a single Ogre::Entity attached.
 */
 template<typename> class node_t : public Ogre::GeneralAllocatedObject
 {
   
  public:
   
   /*! destructor. node
       desc.
           Constructor for node
   */
   node_t(const Ogre::String& name, mesh* meshPtr, collision_tree* creator, const Ogre::Vector3& globalPosition = Ogre::Vector3::ZERO, const Ogre::Quaternion& globalOrientation = Ogre::Quaternion::IDENTITY)
    : octreeNodeOctant(0),
      octreeNodeName(name),
      octreeNodeCreator(creator),
      octreeNodeMesh(meshPtr),
      octreeNodePosition(globalPosition),
      octreeNodeOrientation(globalOrientation),
      octreeNodeTransform(Ogre::Matrix4::IDENTITY)
    {
    octreeNodeMesh->_ref();
     _update();
    }
   
   /*! destructor. node
       desc.
           Destructor for node
   */
  ~node_t()
   {
    // dereference mesh, etc.
    // update?
    octreeNodeMesh->_unref();
   }

   mesh* getMesh()
   {
    return octreeNodeMesh;
   }
   
   Ogre::String getName() const
   {
    return octreeNodeName;
   }

   void   setPosition(const Ogre::Vector3& globalPosition)
   {
    octreeNodePosition = globalPosition;
    _update();
   }
   
   Ogre::Vector3 getPosition() const
   {
    return octreeNodePosition;
   }

   Ogre::Quaternion getOrientation() const
   {
    return octreeNodeOrientation;
   }

   void setOrientation(const Ogre::Quaternion& globalOrientation)
   {
    octreeNodeOrientation = globalOrientation;
    _update();
   }

   Ogre::Matrix4 getTransform() const
   {
    return octreeNodeTransform;
   }

   octree* getOctant()
   {
    return octreeNodeOctant;
   }
   
   void setOctant(octree* o)
   {
    octreeNodeOctant = o;
   }
   
   bool isIn(Ogre::AxisAlignedBox& box)
   {
    if (box.isNull())
     return false;
    
    Ogre::Vector3 center = octreeNodeWorldAABB.getMaximum().midPoint( octreeNodeWorldAABB.getMinimum() );
    Ogre::Vector3 bmin = box.getMinimum();
    Ogre::Vector3 bmax = box.getMaximum();
    
    bool centre = (bmax > center && bmin < center);
    
    if (!centre)
     return false;
    
    Ogre::Vector3 octreeSize = bmax - bmin;
    Ogre::Vector3 nodeSize = octreeNodeWorldAABB.getMaximum() - octreeNodeWorldAABB.getMinimum();
    
    return nodeSize < octreeSize;
   }
   
   Ogre::AxisAlignedBox& getLocalAABB()
   {
    return octreeNodeLocalAABB;
   }
   
   Ogre::AxisAlignedBox& getWorldAABB()
   {
    return octreeNodeWorldAABB;
   }

   inline bool raycast(const line& ln, intersection_result& int_result)
   {
    
    int tri_id = 0;
    
    if (intersections::line(ln, octreeNodeMesh, octreeNodeTransform, int_result.distance, tri_id, int_result.globalPosition))
    {
     int_result.hitTriangle = octreeNodeMesh->at(tri_id);
     int_result.hitTriangle.a = octreeNodeTransform * int_result.hitTriangle.a;
     int_result.hitTriangle.b = octreeNodeTransform * int_result.hitTriangle.b;
     int_result.hitTriangle.c = octreeNodeTransform * int_result.hitTriangle.c;
     return true;
    }
    
    return false;
   }
   
   void _update()
   {
    
    octreeNodeTransform.makeTransform(octreeNodePosition, Ogre::Vector3(1,1,1), octreeNodeOrientation);
    
    octreeNodeWorldAABB.setNull();
    octreeNodeLocalAABB.setNull();
    
    octreeNodeLocalAABB.merge(octreeNodeMesh->getAABB());
    octreeNodeWorldAABB.merge(octreeNodeLocalAABB);
    octreeNodeWorldAABB.transformAffine(octreeNodeTransform);
     
    if (octreeNodeWorldAABB.isNull() == false)
     octreeNodeCreator->_updateNode(this);
    
   }
   
  protected:
   
   Ogre::AxisAlignedBox    octreeNodeLocalAABB;
   
   Ogre::AxisAlignedBox    octreeNodeWorldAABB;
   
   octree*                 octreeNodeOctant;
   
   mesh*                   octreeNodeMesh;
   
   Ogre::Vector3           octreeNodePosition;
   
   Ogre::Quaternion        octreeNodeOrientation;
   
   Ogre::Matrix4           octreeNodeTransform;
   
   collision_tree*         octreeNodeCreator;
   
   Ogre::String            octreeNodeName;
   
 }; // class node_t<typename>

 template<typename> class octree_t : public Ogre::GeneralAllocatedObject
 {
  public:
   
   typedef std::list<node*> nodes_t;
   
   octree_t(octree_t* parent, collision_tree* creator)
   : octreeParent(parent), octreeNbNodes(0), octreeCreator(creator)
   {
    for (int i=0;i<2;i++)
     for (int j=0;j<2;j++)
      for (int k=0;k<2;k++)
       octreeChildren[i][j][k] = 0;
    
    creator->_addOctree(this);
   }
   
  ~octree_t()
   {
    for (int i=0;i<2;i++)
     for (int j=0;j<2;j++)
      for (int k=0;k<2;k++)
       if (octreeChildren[i][j][k] != 0)
        OGRE_DELETE octreeChildren[i][j][k];
    
    octreeCreator->_removeOctree(this);
    octreeParent = 0;
   }
   
   void add(node* node)
   {
    octreeNodes.push_back(node);
    node->setOctant(this);
    _ref();
   }
   
   void remove(node* node)
   {
    octreeNodes.erase(std::find(octreeNodes.begin(), octreeNodes.end(), node));
    node->setOctant(0);
    _unref();
   }
    
   unsigned int nbNodes() const
   {
    return octreeNbNodes;
   }
   
   void getChildIndexes(const Ogre::AxisAlignedBox& box, int* x, int* y, int* z) const
   {
    Ogre::Vector3 centre = octreeBox.getMaximum().midPoint(octreeBox.getMinimum());
    
    Ogre::Vector3 ncenter = box.getMaximum().midPoint(box.getMinimum());
    
    if (ncenter.x > centre.x)
     *x = 1;
    else
     *x = 0;
    
    if (ncenter.y > centre.y)
     *y = 1;
    else
     *y = 0;
    
    if (ncenter.z > centre.z)
     *z = 1;
    else
     *z = 0;
   }
   
   bool isTwiceSize(const Ogre::AxisAlignedBox& box) const
   {
    if (box.isInfinite())
     return false;
    
    Ogre::Vector3 halfOctreeBoxSize = octreeBox.getHalfSize();
    Ogre::Vector3 boxSize = box.getSize();
    
    return (  (boxSize.x <= halfOctreeBoxSize.x) &&
              (boxSize.y <= halfOctreeBoxSize.y) &&
              (boxSize.z <= halfOctreeBoxSize.z) );
    
   }
   
   Ogre::AxisAlignedBox getCulledBounds() const
   {
    return Ogre::AxisAlignedBox(octreeBox.getMinimum() - octreeHalfSize, octreeBox.getMaximum() + octreeHalfSize);
   }
   
  public:
    
   nodes_t              octreeNodes;
   octree_t<void>*      octreeChildren[2][2][2];
   Ogre::Vector3        octreeHalfSize;
   Ogre::AxisAlignedBox octreeBox;
   
  protected:
   
   void _ref()
   {
    octreeNbNodes++;
    if (octreeParent)
     octreeParent->_ref();
   }
   
   void _unref()
   {
    octreeNbNodes--;
    if (octreeParent)
     octreeParent->_unref();
   }
   
   octree*           octreeParent;
   int               octreeNbNodes;
   collision_tree*   octreeCreator;
   
 }; // class octree_t<typename>

 template<typename> class collision_tree_t : public Ogre::GeneralAllocatedObject
 {
  
  public:
   
   typedef std::map<Ogre::String, node*> nodeList;
   typedef std::map<Ogre::String, mesh*> meshList;
   typedef std::vector<octree*>          octreeList;
   
   struct sorted_node
   {
    Ogre::Vector3 p; // intersection position
    node*         n; // node itself.
   };
   
   typedef std::vector<sorted_node>       sortedNodeList;
   
   struct ClosestDistanceSortNodeFunction
   {
    
    ClosestDistanceSortNodeFunction(const Ogre::Vector3& o) : origin(o) {}
    bool operator() (sorted_node& i, sorted_node& j)
    {
     return (i.p.squaredDistance(origin) < j.p.squaredDistance(origin));
    }
    
    Ogre::Vector3 origin;
   };
   
   enum BoxIntersection
   {
    BoxIntersection_Outside    = 0,
    BoxIntersection_Inside     = 1,
    BoxIntersection_Intersect  = 2
   };
   
   collision_tree_t()
   : collisionTreeOctree(0),
     collisionTreeNextNodeId(0),
     collisionDebugNode(0)
   {
    
    Ogre::AxisAlignedBox b( -10000, -10000, -10000, 10000, 10000, 10000 );
    init(b, 8);
   }
   
   void init(Ogre::AxisAlignedBox& box, int depth)
   {
    
    if (collisionTreeOctree != 0)
     OGRE_DELETE collisionTreeOctree;
    
    collisionTreeOctree = OGRE_NEW octree(0, this);
    collisionTreeMaxDepth = depth;
    collisionTreeBox = box;
    collisionTreeOctree->octreeBox = box;
    collisionTreeOctree->octreeHalfSize = (box.getMaximum() - box.getMinimum()) / 2;
    collisionTreeNbObjects = 0;
   }
   
  ~collision_tree_t()
   {
    if (collisionTreeOctree)
     OGRE_DELETE collisionTreeOctree;
    
    // TODO: For each mesh...delete them too.
   }
   
   void _updateNode(node* n)
   {
    const Ogre::AxisAlignedBox& box = n->getWorldAABB();
    
    if (box.isNull())
     return;
    
    if (collisionTreeOctree == 0)
     return;
    
    if (n->getOctant() == 0)
    {
     if (n->isIn(collisionTreeOctree->octreeBox) == false)
      collisionTreeOctree->add(n);
     else
      _addNode(n, collisionTreeOctree);
     return;
    }
    
    if (n->isIn(n->getOctant()->octreeBox) == false)
    {
     _removeNode(n);
     if (n->isIn(collisionTreeOctree->octreeBox) == false)
      collisionTreeOctree->add(n);
     else
      _addNode(n, collisionTreeOctree);
    }
    
   } // void updateNode(node* node)
   
   void _removeNode(node* n)
   {
    if (collisionTreeOctree == 0)
     return;
    
    if (n->getOctant())
     n->getOctant()->remove(n);
    
    n->setOctant(0);
   }

   void _addOctree(octree* o)
   {
    if (collisionTreeOctree == 0)
     return;
    
    collisionTreeOctrees.push_back(o);
   }

   void _removeOctree(octree* o)
   {
    
    if (collisionTreeOctree == 0)
     return;
    
    for (octreeList::iterator it = collisionTreeOctrees.begin(); it != collisionTreeOctrees.end(); it++)
    {
     if ((*it) == o)
     {
      collisionTreeOctrees.erase(it);
      return;
     }
    }
    
   }

   void _addNode(node* n, octree* octant, int depth = 0)
   {
    
    if (collisionTreeOctree == 0)
     return;
    
    const Ogre::AxisAlignedBox& box = n->getWorldAABB();
    
    
    if (  (depth < collisionTreeMaxDepth) && octant->isTwiceSize(box)  )
    {
     
     int x, y, z;
     
     octant->getChildIndexes(box, &x, &y, &z);
     
     if (octant->octreeChildren[x][y][z] == 0)
     {
      octant->octreeChildren[x][y][z] = OGRE_NEW octree(octant, this);
      const Vector3& octantMin = octant->octreeBox.getMinimum();
      const Vector3& octantMax = octant->octreeBox.getMaximum();
      
      Ogre::Vector3 min, max;
      
      if (x==0)
      {
       min.x = octantMin.x;
       max.x = (octantMin.x + octantMax.x) / 2;
      }
      else
      {
       min.x = (octantMin.x + octantMax.x) / 2;
       max.x = octantMax.x;
      }
      
      if (y==0)
      {
       min.y = octantMin.y;
       max.y = ( octantMin.y + octantMax.y) / 2;
      }
      else
      {
       min.y = ( octantMin.y + octantMax.y) / 2;
       max.y = octantMax.y;
      }
      
      if (z==0)
      {
       min.z = octantMin.z;
       max.z = (octantMin.z + octantMax.z) / 2;
      }
      else
      {
       min.z = (octantMin.z + octantMax.z) / 2;
       max.z = octantMax.z;
      }
      
      octant->octreeChildren[x][y][z]->octreeBox.setExtents(min, max);
      octant->octreeChildren[x][y][z]->octreeHalfSize = (max - min) / 2;
      
     } // if (octant->octreeChildren[x][y][z] == 0)
     
     _addNode(n, octant->octreeChildren[x][y][z], ++depth);
     
    } // if ((depth < collisionTreeMaxDepth) && octant->isTwiceSize(box);
    else
    {
     octant->add(n);
    }
   } // void addNode(node* node, octree* octant, int depth) 
   
   node*  createNode(const Ogre::String& mesh_name, const Ogre::Vector3& globalPos = Ogre::Vector3::ZERO, const Ogre::Quaternion& globalOrientation = Ogre::Quaternion::IDENTITY)
   {
    std::stringstream s;
    s << "node_" << collisionTreeNextNodeId++;
    return createNode(s.str(), mesh_name, globalPos, globalOrientation);
   }

   node*  createNode(const Ogre::MeshPtr& mesh_ptr, const Ogre::Vector3& globalPos = Ogre::Vector3::ZERO, const Ogre::Quaternion& globalOrientation = Ogre::Quaternion::IDENTITY)
   {
    std::stringstream s;
    s << "node_" << collisionTreeNextNodeId++;
    return createNode(s.str(), mesh_ptr, globalPos, globalOrientation);
   }
   
   
   node*  createNode(const Ogre::String& nodeName, const Ogre::String& mesh_name, const Ogre::Vector3& globalPos = Ogre::Vector3::ZERO, const Ogre::Quaternion& globalOrientation = Ogre::Quaternion::IDENTITY)
   {
    Ogre::MeshPtr meshPtr = Ogre::MeshManager::getSingleton().getByName(mesh_name);
    return createNode(nodeName, meshPtr, globalPos, globalOrientation);
   }

   node*  createNode(const Ogre::String& nodeName, const Ogre::MeshPtr& mesh_ptr, const Ogre::Vector3& globalPos = Ogre::Vector3::ZERO, const Ogre::Quaternion& globalOrientation = Ogre::Quaternion::IDENTITY)
   {
    mesh* meshPtr = getOrLoadMesh(mesh_ptr);
    node* n = OGRE_NEW node(nodeName, meshPtr, this, globalPos, globalOrientation);
    collisionTreeNodes[nodeName] = n;
    return n;
   }

   void  destroyNode(const Ogre::String& name)
   {
    nodeList::iterator i = collisionTreeNodes.find(name);

    if (collisionTreeNodes.find(name) == collisionTreeNodes.end())
    {
      OGRE_EXCEPT(Exception::ERR_ITEM_NOT_FOUND, "Node '" + name + "' not found.",
        "collision_tree::destroyNode");
    }
    
    node* n = (*i).second;
    collisionTreeNodes.erase(i);
    _removeNode(n);
    OGRE_DELETE n;
   }

   void  destroyNode(node* n)
   {
    collisionTreeNodes.erase(std::find(collisionTreeNodes.begin(), collisionTreeNodes.end(), n->getName()));
    _removeNode(n);
    OGRE_DELETE n;
   }
   
   mesh*          getOrLoadMesh(const Ogre::MeshPtr& meshPtr)
   {
    meshList::iterator i = collisionTreeMeshes.find(meshPtr->getName());
    
    if (i != collisionTreeMeshes.end())
     return (*i).second;
    
    mesh* m = OGRE_NEW mesh(meshPtr);
    collisionTreeMeshes[meshPtr->getName()] = m;
    
    return m;
    
   }
   
   void           unloadMesh(const Ogre::MeshPtr& mesh);
   
   void           unloadMesh(const Ogre::String& name);
   
   bool           isMeshLoaded(const Ogre::MeshPtr& mesh);
   
   bool           isMeshLoaded(const Ogre::String& name);
   
   bool raycast(const Ogre::Ray& ray, const Ogre::Real& length, intersection_result& result)
   {
    return raycast(line(ray, length), result);
   }

   bool raycastBounds(const Ogre::Ray& ray, const Ogre::Real& length, intersection_result& result)
   {
    return raycast(line(ray, length), result);
   }

   bool  raycast(const Ogre::Vector3& origin, const Ogre::Vector3& normalisedDirection, const Ogre::Real& length, intersection_result& int_result)
   {
    return raycast(line(origin, normalisedDirection, length), int_result);
   }

   bool  raycastBounds(const Ogre::Vector3& origin, const Ogre::Vector3& normalisedDirection, const Ogre::Real& length, intersection_result&)
   {
    return raycastBounds(line(origin, normalisedDirection, length), int_result);
   }

   bool  raycastBounds(const line& line, intersection_result& int_result)
   {
    sortedNodeList list;
    findNodesIn(line, list, false, collisionTreeOctree);
    
    if (list.size() == 0)
     return false;
    
    // get intersect positions from line to node AABB.
    for (sortedNodeList::iterator it = list.begin(); it != list.end(); it++)
     intersections::line(line, (*it).n->getWorldAABB(), (*it).p);
    
    // sort the nodeList by closest first, using intersect positions just found.
    std::sort(list.begin(), list.end(), ClosestDistanceSortNodeFunction(line.origin()));
    
    int_result.distance = list.begin().p;
    int_result.globalNormal = -line.direction(); // Probably not.
    int_result.globalPosition = line.origin() + (line.direction() * int_result.distance);
    int_result.hitMesh = 0;
    
    return true;
   }

   bool  raycast(const line& line, intersection_result& int_result)
   {
    
    sortedNodeList list;
    findNodesIn(line, list, false, collisionTreeOctree);
    
    if (list.size() == 0)
     return false;

    // get intersect positions from line to node AABB.
    for (sortedNodeList::iterator it = list.begin(); it != list.end(); it++)
     intersections::line(line, (*it).n->getWorldAABB(), (*it).p);
    
    // sort the nodeList by closest first, using intersect positions just found.
    std::sort(list.begin(), list.end(), ClosestDistanceSortNodeFunction(line.origin()));
    
    // Ray cast closest first, then the further ones away. If a distance is closer then
    // that is more likely to be the triangle.
     
    // Although the first node is most likely to be hit, there are cases even though the
    // AABB is closer, that a triangle of a node further away is closer to the one with
    // the closest AABB.
    
    intersection_result temp;
    intersection_result working;
    working.distance = 0;
    
    for (sortedNodeList::iterator it = list.begin(); it != list.end(); it++)
    {
     temp.distance = 0;
     (*it).n->raycast(line, temp);
     if (working.distance == 0 || temp.distance < working.distance)
      working = temp;
    }
    
    int_result.distance = working.distance;
    int_result.globalPosition = working.globalPosition;
    int_result.globalNormal = working.globalNormal;
    int_result.hitMesh = working.hitMesh;
    int_result.hitTriangle = working.hitTriangle;
    
    return int_result.distance != 0.f;
   }
   
   /*! function. findNodesIn
       desc.
   */
   template<typename beastie_class>
   void  findNodesIn(const beastie_class& ref, sortedNodeList& list, bool full, octree* octant)
   {
    
    if (full == false)
    {
     Ogre::AxisAlignedBox obox = octant->getCulledBounds();
     
     BoxIntersection intersection = intersect(ref, obox);
     
     if (intersection == BoxIntersection_Outside)
      return;
     
     full = (intersection == BoxIntersection_Inside);
     
    }

    octree::nodes_t::iterator it = octant->octreeNodes.begin();
    
    while (it != octant->octreeNodes.end())
    {
     node* n = (*it);
     
     if (full)
     {
      sorted_node sn;
      sn.n = n;
      list.push_back(sn);
     }
     else
     {
      if ( intersect(ref, n->getWorldAABB()) != BoxIntersection_Outside )
      {
       sorted_node sn;
       sn.n = n;
       list.push_back(sn);
      }
     }
     
     ++it;
    } // while 
    
    
    octree* child;
    
    if ( (child=octant->octreeChildren[0][0][0]) != 0)
     findNodesIn(ref, list, full, child);

    if ( (child=octant->octreeChildren[1][0][0]) != 0)
     findNodesIn(ref, list, full, child);

    if ( (child=octant->octreeChildren[0][1][0]) != 0)
     findNodesIn(ref, list, full, child);

    if ( (child=octant->octreeChildren[1][1][0]) != 0)
     findNodesIn(ref, list, full, child);

    if ( (child=octant->octreeChildren[0][0][1]) != 0)
     findNodesIn(ref, list, full, child);

    if ( (child=octant->octreeChildren[1][0][1]) != 0)
     findNodesIn(ref, list, full, child);

    if ( (child=octant->octreeChildren[0][1][1]) != 0)
     findNodesIn(ref, list, full, child);

    if ( (child=octant->octreeChildren[1][1][1]) != 0)
     findNodesIn(ref, list, full, child);

   } // void findNodesIn(...)
   

   BoxIntersection  intersect(const line& line, const Ogre::AxisAlignedBox& box)
   {
    
    if (box.isNull())
     return BoxIntersection_Outside;
    
    if (box.isInfinite())
     return BoxIntersection_Intersect;
    
    //std::cout << "intersect.a\n";

    bool inside = true;
    const Vector3& boxMin = box.getMinimum();
    const Vector3& boxMax = box.getMaximum();
    Vector3 origin = line.origin();
    Vector3 dir = line.direction();
    Ogre::Vector3 maxT(-1,-1,-1);
    
    for (int i=0;i < 3;i++)
    {
     if (origin[i] < boxMin[i])
     {
      inside = false;
      if (dir[i] > 0)
       maxT[i] = (boxMin[i] - origin[i]) / dir[i];
     }
     else if (origin[i] > boxMax[i])
     {
      inside = false;
      if (dir[i] < 0)
       maxT[i] = (boxMax[i] - origin[i]) / dir[i];
     }
    }
    
    if (inside)
     return BoxIntersection_Intersect;
    
    int whichPlane = 0;
    if (maxT[1] > maxT[whichPlane])
     whichPlane = 1;
    if (maxT[2] > maxT[whichPlane])
     whichPlane = 2;
    
    if (  ((int)maxT[whichPlane]) & 0x80000000)
     return BoxIntersection_Outside;
    
    for (int i=0; i < 3; i++)
    {
     if ( i != whichPlane)
     {
      float f = origin[i] + maxT[whichPlane] * dir[i];
      if (f < (boxMin[i] - 0.00001f) || f > (boxMax[i] + 0.00001f))
      {
       //std::cout << "intersect.d1\n";
       return BoxIntersection_Outside;
      }
     }
    }
    
    return BoxIntersection_Intersect;
   
    
   } // BoxIntersection  intersect(...)
   
   
   
   void  renderVisualDebugger(Ogre::SceneNode* debugNodePtr = 0)
   {
    
    if (collisionDebugNode == 0)
    {
     collisionDebugNode = debugNodePtr;
     collisionDebugObject = debugNodePtr->getCreator()->createManualObject();
     collisionDebugNode->attachObject(collisionDebugObject);
     collisionDebugObject->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_LINE_LIST);
    }
    else
     collisionDebugObject->beginUpdate(0);
    
    for (nodeList::iterator it = collisionTreeNodes.begin(); it != collisionTreeNodes.end(); it++)
    {
     node* n = (*it).second;
     _drawBox(n->getWorldAABB(), Ogre::ColourValue::Red);
     n->getMesh()->_draw(collisionDebugObject, n->getTransform());
    } // for

    for (octreeList::iterator it = collisionTreeOctrees.begin(); it != collisionTreeOctrees.end(); it++)
    {
     octree* o = (*it);
     _drawBox(o->octreeBox, Ogre::ColourValue::Blue);
    }
    
    collisionDebugObject->end();
    
   } // void  renderVisualDebugger(...)
   
  protected:
   
   void           _drawBox(const Ogre::AxisAlignedBox& box, const Ogre::ColourValue& col)
   {
    // 0--3
    collisionDebugObject->position(box.getCorner(Ogre::AxisAlignedBox::FAR_LEFT_BOTTOM));
    collisionDebugObject->colour(col);
    collisionDebugObject->position(box.getCorner(Ogre::AxisAlignedBox::FAR_RIGHT_BOTTOM));
    collisionDebugObject->colour(col);

    // 3--7
    collisionDebugObject->position(box.getCorner(Ogre::AxisAlignedBox::FAR_RIGHT_BOTTOM));
    collisionDebugObject->colour(col);
    collisionDebugObject->position(box.getCorner(Ogre::AxisAlignedBox::NEAR_RIGHT_BOTTOM));
    collisionDebugObject->colour(col);

    // 7--6
    collisionDebugObject->position(box.getCorner(Ogre::AxisAlignedBox::NEAR_RIGHT_BOTTOM));
    collisionDebugObject->colour(col);
    collisionDebugObject->position(box.getCorner(Ogre::AxisAlignedBox::NEAR_LEFT_BOTTOM));
    collisionDebugObject->colour(col);

    // 6--0
    collisionDebugObject->position(box.getCorner(Ogre::AxisAlignedBox::NEAR_LEFT_BOTTOM));
    collisionDebugObject->colour(col);
    collisionDebugObject->position(box.getCorner(Ogre::AxisAlignedBox::FAR_LEFT_BOTTOM));
    collisionDebugObject->colour(col);
    
    ///
    collisionDebugObject->position(box.getCorner(Ogre::AxisAlignedBox::FAR_LEFT_TOP));
    collisionDebugObject->colour(col);
    collisionDebugObject->position(box.getCorner(Ogre::AxisAlignedBox::FAR_RIGHT_TOP));
    collisionDebugObject->colour(col);
    collisionDebugObject->position(box.getCorner(Ogre::AxisAlignedBox::FAR_RIGHT_TOP));
    collisionDebugObject->colour(col);
    collisionDebugObject->position(box.getCorner(Ogre::AxisAlignedBox::NEAR_RIGHT_TOP));
    collisionDebugObject->colour(col);
    collisionDebugObject->position(box.getCorner(Ogre::AxisAlignedBox::NEAR_RIGHT_TOP));
    collisionDebugObject->colour(col);
    collisionDebugObject->position(box.getCorner(Ogre::AxisAlignedBox::NEAR_LEFT_TOP));
    collisionDebugObject->colour(col);
    collisionDebugObject->position(box.getCorner(Ogre::AxisAlignedBox::NEAR_LEFT_TOP));
    collisionDebugObject->colour(col);
    collisionDebugObject->position(box.getCorner(Ogre::AxisAlignedBox::FAR_LEFT_TOP));
    collisionDebugObject->colour(col);

    ///
    collisionDebugObject->position(box.getCorner(Ogre::AxisAlignedBox::FAR_LEFT_BOTTOM));
    collisionDebugObject->colour(col);
    collisionDebugObject->position(box.getCorner(Ogre::AxisAlignedBox::FAR_LEFT_TOP));
    collisionDebugObject->colour(col);
    ///
    collisionDebugObject->position(box.getCorner(Ogre::AxisAlignedBox::FAR_RIGHT_BOTTOM));
    collisionDebugObject->colour(col);
    collisionDebugObject->position(box.getCorner(Ogre::AxisAlignedBox::FAR_RIGHT_TOP));
    collisionDebugObject->colour(col);
    ///
    collisionDebugObject->position(box.getCorner(Ogre::AxisAlignedBox::NEAR_RIGHT_BOTTOM));
    collisionDebugObject->colour(col);
    collisionDebugObject->position(box.getCorner(Ogre::AxisAlignedBox::NEAR_RIGHT_TOP));
    collisionDebugObject->colour(col);
    ///
    collisionDebugObject->position(box.getCorner(Ogre::AxisAlignedBox::NEAR_LEFT_BOTTOM));
    collisionDebugObject->colour(col);
    collisionDebugObject->position(box.getCorner(Ogre::AxisAlignedBox::NEAR_LEFT_TOP));
    collisionDebugObject->colour(col);

   }
   
   octree*                        collisionTreeOctree;
   line                           collisionTreeRaycastLine;
   nodeList                       collisionTreeNodes;
   meshList                       collisionTreeMeshes;
   octreeList                     collisionTreeOctrees;
   int                            collisionTreeMaxDepth;
   Ogre::AxisAlignedBox           collisionTreeBox;
   int                            collisionTreeNbObjects;
   unsigned int                   collisionTreeNextNodeId;
   Ogre::ManualObject*            collisionDebugObject;
   Ogre::SceneNode*               collisionDebugNode;

 }; // class colision_tree_t<typename>
  
 } // namespace its_a_secret





} // namespace beastie


#endif
