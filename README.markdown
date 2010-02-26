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
