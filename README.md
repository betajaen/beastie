


                                                                                  
                                                                                  
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
================

Beastie is a C++ based simplified collision detection library for the [Ogre3D]  
graphics engine, and it is neatly packaged in a single header, `beastie.h`.  

"Beastie" should be always pronounced in a Glaswegian accent. 

[Ogre3D]: http://www.ogre3d.org

It only detects collisions and does not respond to them as you would typically  
encounter in a more fully featured physics engine or more advanced collision  
library.

However the detections are accurate and are optimised, building some response  
code on top of beastie shouldn't be to difficult, an example of some of this  
is in the "beastie.example.cpp" test application that can be found at the beastie  
Git repository.

Beastie only uses Ogre as a dependency, and is developed with Ogre 1.7 in mind;  
but should work with any 1.x version.

Using Beastie
=============

All of beastie classes are contained in the `beastie` namespace. All member  
functions use the `camelCase` notation where as the class names use the  
`lower_case_underscore_notation`. There is a private namespace called  
`beastie::its_a_secret` this is to hold the internal workings of beastie and  
shouldn't be needed be to used by an application, or it's programmer.

To include beastie in your project, simply copy the `beastie.h` file into your  
project directory, then include it in your project, making sure you correctly  
include and link to Ogre. To increase compliation speed you may want to  
`#include` beastie in your static headers file.

Collisions
==========

Testing for collisions may be done via the `intercept` member function either  
in; `line`, `plane`, `box` or `collision_tree` classes. Additionally duplicate  
static functions can be found in the `intersections` class.


    ray_query result;
    
    // These two lines are interchangable based on preference.
    line.intersect(my_plane, result); 
    intersections::line(my_line, my_plane, result);

The available collisions may be summed up in this table.

                                                                              
    +----------+------+-----+------+----------+                               
    |          | Line | Box | Plane| Triangle |  <- function name             
    +----------+------+-----+------+----------+                               
    | Line     | No   | Yes | Yes  | Yes      |                               
    | Box      | Yes  | No  | No   | No       |                               
    | Plane    | Yes  | No  | No   | No       |                               
    | Triangle | Yes  | No  | No   | No       |                               
    +----------+------+-----+------+----------+                               
        .                                                                     
       /|\                                                                    
        |___  Testing against.                                                
                                                                              
                                                                              

---

Lines  `beastie::line`
======================

Lines are single line in 3D space with a specified length. They can be used  
in numerous ways but one of the most common use is raycasting.

Lines are made from three properties:

* a origin (`Ogre::Vector3`)
* a direction (`Ogre::Vector3`)
* a length (`Ogre::Real`)

Typical usage:

    // Creation
    beastie::line ln(Vector3(10,10,10), Vector3(0,-1,0), 15);

    // Creation using an Ogre Ray.
    beastie::line ln(Ogre::Ray(10,10,10), Vector3(0,-1,0), 15);

    // Intersecting a plane.
    beastie::line ln(Vector3(11,10,11), Vector3(0,-1,0), 15);
    beasite::plane pl;
    Ogre::Vector3 hitPos;
    line.intersects(pl, &hitPos);
    std::cout << hitPos << std::endl;   // Will be 1,0,1


Box  `beastie::box`
===================

Boxes are axis aligned boxes in 3D space with a specified volume. They can  
be used in different ways but most commonly use is for Axis-Aligned Boxes for  
octrees and meshes.

Boxes are made from two properties:

* minimum point (`Ogre::Vector3`)
* maximum point (`Ogre::Vector3`)

Plane  `beastie::plane`
=======================

Planes are infinite flat dimension surfaces.

Planes are made from two properties:

* directional normal (`Ogre::Vector3`)
* distance from world center (`Ogre::Vector3`)

Typical usage is:

    // Creation
    beastie::plane pl(Ogre::Vector3(0,1,0), 0);
    
    // Intersecting a line.
    
    beasite::plane pl(Ogre::Vector3(0,1,0), 0);
    beastie::line ln(Vector3(11,10,11), Vector3(0,-1,0), 15);
    Ogre::Vector3 hitPos;
    plane.intersects(ln, &hitPos);
    std::cout << hitPos << std::endl;   // Will be 1,0,1


Mesh  `beastie::mesh`
=====================

Meshes are a collection many triangles for collisions.  Meshes are used when  
a box, or plane doesn't accurately represent your visual mesh.

A mesh may be be split up into different sections for optimisation reasons,  
these are called sub-meshes.

A single mesh is shared between nodes

Node  `beastie::node`
=====================

Nodes are used to create an instant of a shape `box`, `plane` or `mesh`.  

__Currently meshes are only supported.__

Using Ogre as an reference; nodes are like to Ogre Scenesnodes. Meshes, planes  
or boxes attached to the nodes are like to Ogre Entities.

Unlike Ogre, beastie's nodes cannot be placed inside of each other, as each  
node is regarded to be in the global space.

SceneNodes are created and maintained by the `beastie::collision_tree` class.

Tree  `beastie::collision_tree`
===============================

Trees contains nodes, and acts like the Ogre SceneManager class, or a large  
portion of it, with each portion completely isolated from each one another.

You should treat collision_tree's as pointers and their lifetime should be  
the same as your SceneManager.

Typical usage is:

    // Creation
    beastie::collision_tree tree = OGRE_NEW collision_tree();

    // Destruction (After or near when your SceneManager is deleted)
    OGRE_DELETE tree;

    // Raycasting
    tree->createNode("tudorhouse.mesh", Ogre::Vector3(0,550,0);
    ray_query result;
    line ln(Ogre::Vector3(0,25000,0), Ogre::Vector3(0,-1,0), 25000);
    tree->intersect(ln, result);

