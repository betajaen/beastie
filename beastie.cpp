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
#include "OGRE/OgreRay.h"
#include "OGRE/OgreCamera.h"
#include "OGRE/OgreMesh.h"
#include "OGRE/OgreSubMesh.h"

#include <vector>
#include <iostream>

#ifdef _DEBUG
#  define RETURN_IF(COND, REASON) if (COND) {std::cout << REASON << "\n";return;}
#else
#  define RETURN_IF(COND, REASON) if (COND) {return;}
#endif

#define CONTINUE_IF(COND) if (COND) continue;

#define SQUARED(X) X*X
#define ABS(X) Ogre::Math::Abs(X)

unsigned int beastie::Utils::sUniqueID = 0;

unsigned int beastie::Utils::uniqueID()
{
 return beastie::Utils::sUniqueID++;
}

void beastie::Utils::meshToVector(const Ogre::MeshPtr& mesh, std::vector<NormalisedTriangle>& vec)
{
 vec.clear();
 
 Ogre::Vector3* verts = 0;
 unsigned long* indices = 0;
 size_t nbVerts = 0, nbIndices = 0, nbSubMeshIndices = 0;
 bool addedShared = false;
 for (Ogre::ushort i=0;i < mesh->getNumSubMeshes(); ++i)
 {
  Ogre::SubMesh* submesh = mesh->getSubMesh(i);
  if (submesh->useSharedVertices)
  {
   if (!addedShared)
   {
    nbVerts += mesh->sharedVertexData->vertexCount;
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
 Ogre::IndexData*  iData = 0;
 size_t currentOffset = 0, sharedOffset = 0, nextOffset = 0, indexOffset = 0, offset = 0;

 for (Ogre::ushort i = 0; i < mesh->getNumSubMeshes();++i)
 {
   Ogre::SubMesh* submesh = mesh->getSubMesh(i);
   vData = submesh->useSharedVertices ? mesh->sharedVertexData : submesh->vertexData;
   
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
 
 vec.clear();
 vec.reserve(nbIndices / 3);
 for (size_t i=0;i < nbIndices;i+=3)
 {
  NormalisedTriangle tri;
  tri.a = verts[indices[i]];
  tri.b = verts[indices[i+1]];
  tri.c = verts[indices[i+2]];
  tri.normalise();
  vec.push_back(tri);
 }
 
 delete indices;
 delete verts;
}


beastie::Line  beastie::Utils::rayToLine(const Ogre::Ray& in, Ogre::Real lineLength)
{
 beastie::Line out;
 out.setDirection(in.getDirection());
 out.setOrigin(in.getOrigin());
 out.setLength(lineLength);
 return out;
}

beastie::Line  beastie::Utils::getCameraToViewportRay(Ogre::Camera* cam, float x, float y)
{
 // Stolen from OgreCamera.cpp
 Ogre::Matrix4 inverseVP = (cam->getProjectionMatrix() * cam->getViewMatrix(true)).inverse();
 
 Ogre::Real nx = (2.0f * x) - 1.0f,  ny = 1.0f - (2.0f * y);
 
 Ogre::Vector3 nearPoint(nx,ny,-1.0f), midPoint(nx,ny,0.0f);
 
 Line line;
 line.mOrigin  = inverseVP * nearPoint;
 line.mDirection = Ogre::Vector3(inverseVP * midPoint) - line.mOrigin;
 line.mDirection . normalise();
 
 return line;
}

// Point vs Point
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
 Ogre::Vector3 normal = Ogre::Math::calculateBasicFaceNormal(triangle->mTriangle.a, triangle->mTriangle.b, triangle->mTriangle.c);
 if (Ogre::Math::pointInTri3D(point->mPosition, triangle->mTriangle.a, triangle->mTriangle.b, triangle->mTriangle.c, normal))
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

// Point vs Triangle
void beastie::Tests::intersection(class beastie::Point *,class beastie::Triangle *,struct beastie::Intersection &)
{
}

// Line vs Point
void beastie::Tests::intersection(beastie::Line* line, beastie::Point* point, beastie::Intersection& intersection)
{
 RETURN_IF(line->mLength == 0, "Length is 0")
 RETURN_IF(line->mDirection == Ogre::Vector3::ZERO, "Direction is 0");
 
 Ogre::Vector3 v = point->mPosition - line->mOrigin;
 const Ogre::Vector3 s = line->getOtherPosition() - line->mOrigin;
 Ogre::Real dot = v.dotProduct(s) / (SQUARED(line->getLength()));
 v = v - (s * dot);
 
 RETURN_IF(v.squaredLength() >= epsSquared, "Point not within range");
 
 intersection.hit = true;
 intersection.position = point->mPosition;
}

// Point vs Plane
void beastie::Tests::intersection(beastie::Point* point, beastie::Plane* plane, beastie::Intersection& intersection)
{
 Tests::intersection(plane, point, intersection);
}

// Plane vs Point
void beastie::Tests::intersection(beastie::Plane* plane, beastie::Point* point, beastie::Intersection& intersection)
{
 float d = point->mPosition.dotProduct(plane->mNormal) + plane->mDistance;
 RETURN_IF(d > beastie::eps, "Not within distance");
 
 intersection.hit = true;
 intersection.position = point->mPosition;
}

// Triangle vs Line
void beastie::Tests::intersection(beastie::Triangle* triangle, beastie::Line* line, beastie::Intersection& intersection)
{
 beastie::Tests::intersection(line, triangle, intersection);
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
 // Stolen (and slightly re-written) from OgreMath.cpp
 
 Ogre::Real denom = triangle->mTriangle.n.dotProduct(line->getDirection());
 RETURN_IF(denom > beastie::eps, "Ray on other side"); // Intersection on otherside of triangle face.
 Ogre::Real t = triangle->mTriangle.n.dotProduct(triangle->mTriangle.a - line->getOrigin()) / denom;
 RETURN_IF(t < 0, "t<0");
 
 size_t i0, i1;
 {
  const Ogre::Real n0 = Ogre::Math::Abs(triangle->mTriangle.n[0]);
  const Ogre::Real n1 = Ogre::Math::Abs(triangle->mTriangle.n[1]);
  const Ogre::Real n2 = Ogre::Math::Abs(triangle->mTriangle.n[2]);
  
  i0 = 1; i1 = 2;
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
 }
 
 {
  Ogre::Real u1 = triangle->mTriangle.b[i0] - triangle->mTriangle.a[i0];
  Ogre::Real v1 = triangle->mTriangle.b[i1] - triangle->mTriangle.a[i1];
  Ogre::Real u2 = triangle->mTriangle.c[i0] - triangle->mTriangle.a[i0];
  Ogre::Real v2 = triangle->mTriangle.c[i1] - triangle->mTriangle.a[i1];
  
  Ogre::Real u0 = t * line->mDirection[i0] + line->mOrigin[i0] -  triangle->mTriangle.a[i0];
  Ogre::Real v0 = t * line->mDirection[i1] + line->mOrigin[i1] -  triangle->mTriangle.a[i1];
  
  Ogre::Real alpha = u0 * v2 - u2 * v0;
  Ogre::Real beta  = u1 * v0 - u0 * v1;
  Ogre::Real area  = u1 * v2 - u2 * v1;
  
  Ogre::Real tolerence = beastie::negativeEps * area;
  
  if (area > 0)
  {
   RETURN_IF(alpha < tolerence || beta < tolerence || alpha+beta > area-tolerence, "area>0")
  }
  else
  {
   RETURN_IF(alpha > tolerence || beta > tolerence || alpha+beta < area-tolerence, "else")
  }
 }
 
 intersection.hit = true;
 intersection.position = line->mOrigin + (line->mDirection * t);
 
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
 RETURN_IF(Ogre::Math::Abs(denom) < eps, "denom < eps")
 
 Ogre::Real nom = plane->mNormal.dotProduct(line->mOrigin) + plane->mDistance;
 Ogre::Real t   = -(nom/denom);
 if (t >= 0 && t <= line->mLength)
 {
  intersection.hit = true;
  intersection.position = line->mOrigin + (line->mDirection * t);
 }
}

// Plane vs Plane
void beastie::Tests::intersection(beastie::Plane* plane1, beastie::Plane* plane2, beastie::Intersection& intersection)
{
}

// DynamicMesh vs DynamicMesh
void beastie::Tests::intersection(beastie::DynamicMesh *,beastie::DynamicMesh *, beastie::Intersection &)
{
}

// Plane vs DynamicMesh
void beastie::Tests::intersection(beastie::Plane *,beastie::DynamicMesh *, beastie::Intersection &)
{
}

// Triangle vs DynamicMesh
void beastie::Tests::intersection(beastie::Triangle *,beastie::DynamicMesh *, beastie::Intersection &)
{
}

// Line vs DynamicMesh
void beastie::Tests::intersection(beastie::Line* line,beastie::DynamicMesh* dynMesh, beastie::Intersection& intersection)
{
 
 // TODO: AABox Intersection here.
 
 if (dynMesh->mNeedsUpdate)
 {
  dynMesh->mMesh->transformTriangles(dynMesh->mTriangles, dynMesh->mPosition, dynMesh->mOrientation, dynMesh->mScale);
  dynMesh->mNeedsUpdate = false;
 }
 
  // Stolen (and slightly re-written) from OgreMath.cpp
  
 size_t i0, i1; Ogre::Real t, u0, u1, v0, v1, u2, v2, alpha, beta, area;
 
 for (unsigned int i=0;i < dynMesh->mTriangles.size();i++)
 {
  
  NormalisedTriangle& triangle = dynMesh->mTriangles[i];
  
  u1 = triangle.n.dotProduct(line->getDirection());
  CONTINUE_IF(u1 > beastie::eps); // Intersection on otherside of triangle face.
  
  t = triangle.n.dotProduct(triangle.a - line->mOrigin) / u1;
  CONTINUE_IF(t < 0);
  
  u1 = ABS(triangle.n[0]); // n0
  v1 = ABS(triangle.n[1]); // n1
  u2 = ABS(triangle.n[2]); // n2
  
  i0 = 1; i1 = 2;
  if (u1 > u2)
     {
      if (u1 > u1)
       i0 = 0;
     }
  else
     {
      if (u2 > u1)
       i1 = 0;
     }
  
  
  u1 = triangle.b[i0] - triangle.a[i0];
  v1 = triangle.b[i1] - triangle.a[i1];
  u2 = triangle.c[i0] - triangle.a[i0];
  v2 = triangle.c[i1] - triangle.a[i1];
  
  u0 = t * line->mDirection[i0] + line->mOrigin[i0] -  triangle.a[i0];
  v0 = t * line->mDirection[i1] + line->mOrigin[i1] -  triangle.a[i1];
  
  alpha = u0 * v2 - u2 * v0;
  beta  = u1 * v0 - u0 * v1;
  area  = u1 * v2 - u2 * v1;
  
  u1 = beastie::negativeEps * area;
  
  if (area > 0)
  {
   CONTINUE_IF(alpha < u1 || beta < u1 || alpha+beta > area-u1)
  }
  else
  {
   CONTINUE_IF(alpha > u1 || beta > u1 || alpha+beta < area-u1)
  }
 
  intersection.hit = true;
  intersection.position = line->mOrigin + (line->mDirection * t);
 
  return;
 } // for
 
}

// Point vs DynamicMesh
void beastie::Tests::intersection(beastie::Point *, beastie::DynamicMesh *, beastie::Intersection &)
{
}

void beastie::Tests::intersection(beastie::DynamicMesh *, beastie::Point*, beastie::Intersection &)
{
}

void beastie::Tests::intersection(beastie::DynamicMesh* dynMesh, beastie::Line* line, beastie::Intersection& intersection)
{
 beastie::Tests::intersection(line, dynMesh, intersection);
}

void beastie::Tests::intersection(beastie::DynamicMesh *, beastie::Plane*, beastie::Intersection &)
{
}

void beastie::Tests::intersection(beastie::DynamicMesh *, beastie::Triangle*, beastie::Intersection &)
{
}



#undef RETURN_IF
#undef CONTINUE_IF
#undef SQUARED
#undef ABS
