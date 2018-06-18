/*
* Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
*
* This software is provided 'as-is', without any express or implied
* warranty.  In no event will the authors be held liable for any damages
* arising from the use of this software.
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 3. This notice may not be removed or altered from any source distribution.
*/

/* 
 * Base code for CS 251 Software Systems Lab 
 * Department of Computer Science and Engineering, IIT Bombay
 * 
 */


#include "cs251_base.hpp"
#include "render.hpp"

#ifdef __APPLE__
	#include <GLUT/glut.h>
#else
	#include "GL/freeglut.h"
#endif

#include <cstring>
using namespace std;

#include "dominos.hpp"

/*!
  \namespace cs251 cs251 is a namespace
  */
  /*! It contains a constructor of dominos_t class and a sim pointer variable of type sim_t
*/
namespace cs251
{
      /**  This is the constructor
   
   */
  
  dominos_t::dominos_t()
  {
    //Ground
    /*! \section Upper Upper Hill 
      * At there is upper hill code  from which bikes starts riding and comes down 
    */
//ground from which bike will start
      b2Body* b6; 
	double xi,yi;
	xi = -45.0f;
	yi = 40.0f;
    {
      for (int i = 0; i < 5; ++i){ 
      //b2EdgeShape shape; 
      //shape.Set(b2Vec2(xi , yi), b2Vec2(-44.0f+0.1f*i*i , 39.0f-0.1f*i*i*i ));
      xi = -44.0f+0.1f*i*i;
      yi = 39.0f-0.1f*i*i*i;
     // b2BodyDef bd; 
      //b6 = m_world->CreateBody(&bd);
      //b6->CreateFixture(&shape, 0.0f);
    }
	}
 {
      for (int i = 0; i < 4; ++i){ 
      //b2EdgeShape shape; 
      //shape.Set(b2Vec2(xi , yi), b2Vec2(xi+0.07f*i*i , yi-0.07*i ));
      xi = xi+0.07f*i*i;
      yi = yi-0.07f*i;
      b2BodyDef bd; 
      //b6 = m_world->CreateBody(&bd);
      //b6->CreateFixture(&shape, 0.0f);
    }
	}
 {
      for (int i = 0; i < 5; ++i){ 
      b2EdgeShape shape; 
      shape.Set(b2Vec2(xi , yi), b2Vec2(xi+0.02f*i*i*i*i , yi-0.02*i*i ));
      xi = xi+0.02f*i*i*i*i;
      yi = yi-0.02f*i;
      b2BodyDef bd; 
      b6 = m_world->CreateBody(&bd);
      b6->CreateFixture(&shape, 0.0f);
    }
	 {
      for (int i = 0; i < 3; ++i){ 
      b2EdgeShape shape; 
      shape.Set(b2Vec2(xi , yi), b2Vec2(xi+0.1f*i*i*i*i , yi-0.1f*i*i*i ));
      xi = xi+0.1f*i*i*i*i;
      yi = yi-0.1f*i*i*i;
      b2BodyDef bd; 
      b6 = m_world->CreateBody(&bd);
      b6->CreateFixture(&shape, 0.0f);
    }
	}
	}
{
      for (int i = 0; i < 4; ++i){ 
      b2EdgeShape shape; 
      shape.Set(b2Vec2(xi , yi), b2Vec2(xi+0.1f*i*i , yi-0.1f*i*i*i ));
      xi = xi+0.1f*i*i;
      yi = yi-0.1f*i*i*i;
      b2BodyDef bd; 
      b6 = m_world->CreateBody(&bd);
      b6->CreateFixture(&shape, 0.0f);
    }
	}
//1st ground(see saw)
       /*! \section Groung1 Plane Ground
      * It is for plane ground on which there is see saw kept .
      */
    b2Body* b1;  
    {
      
      b2EdgeShape shape; 
      shape.Set(b2Vec2(-90.0f, 15.0f), b2Vec2(-10.0f, 15.0f));
      b2BodyDef bd; 
      b1 = m_world->CreateBody(&bd); 
      b1->CreateFixture(&shape, 0.0f);
    }
//1st incline (after pendulum)
    /*! \section Ground2 Inclined Ground after pendulum
      * On this inclined ground, bike which comes down the upper hill gets speed and obstruction due to \b Square \b Boxes
      * but eventually gets manages to surpaas those. 
      */
 b2Body* b2;  
    {
      
      b2EdgeShape shape; 
      shape.Set(b2Vec2(-12.0f, 15.0f), b2Vec2(2.0f, 5.0f));
      b2BodyDef bd; 
      b2 = m_world->CreateBody(&bd);
      b2->CreateFixture(&shape, 0.0f);
    }
//barries
    /*! \section Barrier 
      * Its just the small inclined obstruction to hold \b Squares \b Boxes at start. 
      */

 b2Body* b5;  
    {
      
      b2EdgeShape shape; 
      shape.Set(b2Vec2(2.0f, 5.0f), b2Vec2(3.50f, 6.5f));
      b2BodyDef bd; 
      b5 = m_world->CreateBody(&bd);
      b5->CreateFixture(&shape, 0.0f);
  
  }
//2nd incline (after pendulum)
  /*! \section Ground3 Inclined Ground after Barrier
      * It is for plane inclned before it there is the \b barrier  W
      * <br> When gets here it goes towards the three big boxes on front .
      */
     b2Body* b3;  
    {
      
      b2EdgeShape shape; 
      shape.Set(b2Vec2(2.0f, 5.0f), b2Vec2(15.0f, 0.0f));
      b2BodyDef bd; 
      b3 = m_world->CreateBody(&bd);
      b3->CreateFixture(&shape, 0.0f);
    }
//last ground
    /*! \section Ground4 Last Plane
      * It is for last plane ground .
      * <br> After striking the three \b Big \b Boxes on it the gets stopped.
      */
      b2Body* b4;  
    {
      
      b2EdgeShape shape; 
      shape.Set(b2Vec2(15.0f, 0.0f), b2Vec2(500.0f, 0.0f));
      b2BodyDef bd; 
      b4 = m_world->CreateBody(&bd);
      b4->CreateFixture(&shape, 0.0f);
    }

//see saw
            /*! \section SeeSaw See-saw 
      * Here the see-saw on the first plane ground .
      * When bike falls on right side of it , the box kept left of it gets lifted and bike goes front.
      */
    {
      //The triangle wedge
      b2Body* sbody;
      b2PolygonShape poly;
      b2Vec2 vertices[3];
      vertices[0].Set(-1,0);
      vertices[1].Set(1,0);
      vertices[2].Set(0,1.5);
      poly.Set(vertices, 3);
      b2FixtureDef wedgefd;
      wedgefd.shape = &poly;
      wedgefd.density = 10.0f;
      wedgefd.friction = 0.0f;
      wedgefd.restitution = 0.0f;
      b2BodyDef wedgebd;
      wedgebd.position.Set(-30.0f, 15.0f);
      sbody = m_world->CreateBody(&wedgebd);
      sbody->CreateFixture(&wedgefd);

      //The plank on top of the wedge
      b2PolygonShape shape;
      shape.SetAsBox(8.0f, 0.2f);
      b2BodyDef bd2;
      bd2.position.Set(-30.0f, 16.5f);
      bd2.type = b2_dynamicBody;
      b2Body* body = m_world->CreateBody(&bd2);
      b2FixtureDef *fd2 = new b2FixtureDef;
      fd2->density = 1.f;
      fd2->shape = new b2PolygonShape;
      fd2->shape = &shape;
      body->CreateFixture(fd2);

      b2RevoluteJointDef jd;
      b2Vec2 anchor;
      anchor.Set(-30.0f, 16.5f);
      jd.Initialize(sbody, body, anchor);
      m_world->CreateJoint(&jd);

      //The light box on the right side of the see-saw
     b2PolygonShape shape2;
      shape2.SetAsBox(1.0f, 1.0f);
      b2BodyDef bd3;
      bd3.position.Set(-35.0f, 17.0f);
      bd3.type = b2_dynamicBody;
      b2Body* body3 = m_world->CreateBody(&bd3);
      b2FixtureDef *fd3 = new b2FixtureDef;
      fd3->density = 0.01f;
      fd3->shape = new b2PolygonShape;
      fd3->shape = &shape2;
      body3->CreateFixture(fd3);
    
}


  //The pendulum that knocks the dominos off
/*! \section SeeSaw See-saw 
      * Here the see-saw on the first plane ground .
      * When bike falls on right side of it , the box kept left of it gets lifted and bike goes front.
      */
    {
      b2Body* b2;
      {
	//small box for support
	b2PolygonShape shape;
	shape.SetAsBox(0.025f, 0.015f);
	  
	//other end of string
	b2BodyDef bd;
	bd.position.Set(-13.5f, 30.0f);
	b2 = m_world->CreateBody(&bd);
	b2->CreateFixture(&shape, 10.0f);
      }
	
      b2Body* b4;
      {
	// bob
	b2PolygonShape shape;
	shape.SetAsBox(0.25f, 0.25f);
	  
	b2BodyDef bd;
	bd.type = b2_dynamicBody;
	bd.position.Set(-14.5f, 20.0f);
	b4 = m_world->CreateBody(&bd);
	b4->CreateFixture(&shape, 2.0f);
      }
	
      b2RevoluteJointDef jd;
      b2Vec2 anchor;
      anchor.Set(-13.5f, 30.0f);
      jd.Initialize(b2, b4, anchor);
      m_world->CreateJoint(&jd);
    }



    //garbage boxes
//10 lower garbage boxes
          /*! \section SquareBoxes Square Boxes 
      * These square boxes used to obstruct the path of bike  and gets scattered after hit by bike.
      */
{
      b2PolygonShape shape2;
      shape2.SetAsBox(0.5f, 0.5f);
      
      for (int i = 0; i < 10; ++i){
      b2BodyDef bd3;
      bd3.position.Set(-9.8f+ 1.4f * i, 15.0f - 1.0f * i);
      bd3.type = b2_dynamicBody;
      b2Body* body3 = m_world->CreateBody(&bd3);
      b2FixtureDef *fd3 = new b2FixtureDef;
      fd3->density = 0.1f;
      fd3->friction = 0.2f;
      fd3->shape = new b2PolygonShape;
      fd3->shape = &shape2;
      body3->CreateFixture(fd3);
	}
}
//10 upper garbage boxes
{
      b2PolygonShape shape2;
      shape2.SetAsBox(0.5f, 0.5f);
      
      for (int i = 0; i < 10; ++i){
      b2BodyDef bd3;
      bd3.position.Set(-9.8f+ 1.4f * i, 16.0f - 1.0f * i);
      bd3.type = b2_dynamicBody;
      b2Body* body3 = m_world->CreateBody(&bd3);
      b2FixtureDef *fd3 = new b2FixtureDef;
      fd3->density = 0.1f;
      fd3->friction = 0.2f;
      fd3->shape = new b2PolygonShape;
      fd3->shape = &shape2;
      body3->CreateFixture(fd3);
	}
}


//3 big boxes
/*! \section BigBoxes  Big Boxes
      * These big boxes ,on the last plane ground , 
      */
{
      b2PolygonShape shape2;
      shape2.SetAsBox(2.0f, 2.0f);
      
      for (int i = 0; i < 3; ++i){
      b2BodyDef bd3;
      bd3.position.Set(40.0f, 2.0f + 2.0f * i);
      bd3.type = b2_dynamicBody;
      b2Body* body3 = m_world->CreateBody(&bd3);
      b2FixtureDef *fd3 = new b2FixtureDef;
      fd3->density = 1.0f;
      fd3->friction = 1.0f;
      fd3->shape = new b2PolygonShape;
      fd3->shape = &shape2;
      body3->CreateFixture(fd3);
	}
}
double Y_increase;
Y_increase=35.0f;
double X_increase;
X_increase=-54.5f;
double scale;
scale=0.50f;
//central part of the bike
/*! \section Bike  Big Boxes
      * This is the main part in it ,the  \b Bike.
      * <br> It contains two tyres , one main body part , one handle .
      * <br> It also contains the driver represented as its head (circle) and wedges connecting those parts.
      * <br>    
      */

extern b2Body* sbody;
{
      
      b2PolygonShape poly;
      b2Vec2 vertices[6];
      vertices[0].Set(0*scale,0*scale);
      vertices[1].Set(2.5*scale,0*scale);
      vertices[2].Set(3.6*scale,3.0*scale);
      vertices[3].Set(3.35*scale,3.5*scale);
      vertices[4].Set(0*scale,2.75*scale);
      vertices[5].Set(-1.0*scale,1.50*scale);
      poly.Set(vertices, 6);
      b2FixtureDef wedgefd;
      wedgefd.shape = &poly;
      wedgefd.density = 5.0f;
      wedgefd.friction = 0.0f;
      wedgefd.restitution = 0.0f;
      b2BodyDef wedgebd;
      wedgebd.position.Set(30.0f*scale+X_increase, 0.0f*scale+Y_increase);
      wedgebd.type = b2_dynamicBody;
      sbody = m_world->CreateBody(&wedgebd);
      sbody->CreateFixture(&wedgefd);
/*
//The pendulum that acts as head of driver
   {
      b2Body* b2;
      {
	//small box for support
	b2PolygonShape shape;
	shape.SetAsBox(0.025f, 0.015f);
	  
	//other end of string
	b2BodyDef bd;
	bd.position.Set(-10.5f, 30.0f);
	b2 = m_world->CreateBody(&bd);
	b2->CreateFixture(&shape, 10.0f);
      }
	
      b2Body* sbody;
      {
	// bob
      b2CircleShape circle;
      circle.m_radius = 0.6;
	
      b2FixtureDef ballfd;
      ballfd.shape = &circle;
      ballfd.density = 10.0f;
      ballfd.friction = 0.1f;
      ballfd.restitution = 0.0f;
      b2BodyDef ballbd;
      ballbd.type = b2_dynamicBody;
      ballbd.position.Set(-11.1f, 30.0f);
      sbody = m_world->CreateBody(&ballbd);
      sbody->CreateFixture(&ballfd);
      }
	
      b2RevoluteJointDef jd;
      b2Vec2 anchor;
      anchor.Set(-10.5f, 30.0f);
      jd.Initialize(b2,sbody, anchor);
      m_world->CreateJoint(&jd);

*/
      //joint Head and main
      // ROD
b2Body* obodyRO;
      b2PolygonShape polyRO;
      b2Vec2 verticesRO[4];
      verticesRO[0].Set(0.0f*scale,0.0f*scale);
      verticesRO[1].Set(1.0f*scale,0.0*scale);
      verticesRO[2].Set(1.0f*scale,3.0f*scale);
      verticesRO[3].Set(0.0f*scale,3.0f*scale);
      polyRO.Set(verticesRO, 4);
      b2FixtureDef wedgefdRO;
      wedgefdRO.shape = &polyRO;
      wedgefdRO.density = 0.10f;
      wedgefdRO.friction = 0.0f;
      wedgefdRO.restitution = 0.0f;
      b2BodyDef wedgebdRO;
      wedgebdRO.position.Set(30.0f*scale+X_increase, 2.75f*scale+Y_increase);
      wedgebdRO.type = b2_dynamicBody;
      obodyRO = m_world->CreateBody(&wedgebdRO);
      obodyRO->CreateFixture(&wedgefdRO);


// Head and main
b2Body* Head;
      b2CircleShape circle;
      circle.m_radius = 0.8;
	
      b2FixtureDef ballHd;
      ballHd.shape = &circle;
      ballHd.density = 0.5f;
      ballHd.friction = 0.1f;
      ballHd.restitution = 0.0f;
      b2BodyDef ballHbd;
      ballHbd.type = b2_dynamicBody;
      ballHbd.position.Set(30.5f*scale+X_increase, 5.75f*scale+Y_increase);
      Head = m_world->CreateBody(&ballHbd);
      Head->CreateFixture(&ballHd);

//joint Rod and main
b2RevoluteJointDef jdRO1,jdRO2;
      b2Vec2 anchorRO1,anchorRO2;
      anchorRO1.Set(30.0f*scale+X_increase, 2.75f*scale+Y_increase);
      anchorRO2.Set(31.0f*scale+X_increase, 2.75f*scale+Y_increase);
      jdRO1.Initialize(sbody, obodyRO, anchorRO1);
      m_world->CreateJoint(&jdRO1);
      jdRO2.Initialize(sbody, obodyRO, anchorRO2);
      m_world->CreateJoint(&jdRO2);
//joint Head and ROD
b2RevoluteJointDef jdHD1,jdHD2;
      b2Vec2 anchorHD1,anchorHD2;
      anchorHD1.Set(30.0f*scale+X_increase, 5.75f*scale+Y_increase);
      anchorHD2.Set(31.0f*scale+X_increase, 5.75f*scale+Y_increase);
      jdHD1.Initialize(obodyRO, Head, anchorHD1);
      m_world->CreateJoint(&jdHD1);
      jdHD2.Initialize(obodyRO, Head, anchorHD2);
      m_world->CreateJoint(&jdHD2);
// central L

 b2Body* obodyL;
      b2PolygonShape polyL;
      b2Vec2 verticesL[4];
      verticesL[0].Set(0*scale,0*scale);
      verticesL[1].Set(1.0f*scale,1.25f*scale);
      verticesL[2].Set(-1.5f*scale,1.75f*scale);
      verticesL[3].Set(-2.0f*scale,1.0f*scale);
      polyL.Set(verticesL, 4);
      b2FixtureDef wedgefdL;
      wedgefdL.shape = &polyL;
      wedgefdL.density = 1.0f;
      wedgefdL.friction = 0.0f;
      wedgefdL.restitution = 0.0f;
      b2BodyDef wedgebdL;
      wedgebdL.position.Set(29.0f*scale+X_increase, 1.50f*scale+Y_increase);
      wedgebdL.type = b2_dynamicBody;
      obodyL = m_world->CreateBody(&wedgebdL);
      obodyL->CreateFixture(&wedgefdL);

//joint L and main
b2RevoluteJointDef jdL1,jdL2;
      b2Vec2 anchorL1,anchorL2;
      anchorL1.Set(29.0f*scale+X_increase, 1.50f*scale+Y_increase);
      anchorL2.Set(30.0f*scale+X_increase, 2.75f*scale+Y_increase);
      jdL1.Initialize(sbody, obodyL, anchorL1);
      m_world->CreateJoint(&jdL1);
      jdL2.Initialize(sbody, obodyL, anchorL2);
      m_world->CreateJoint(&jdL2);

//central LL
b2Body* obodyLL;
      b2PolygonShape polyLL;
      b2Vec2 verticesLL[4];
      verticesLL[0].Set(0.0f*scale,0.0f*scale);
      verticesLL[1].Set(0.5f*scale,0.75f*scale);
      verticesLL[2].Set(-3.0f*scale,0.65f*scale);
      verticesLL[3].Set(-1.50f*scale,0.0f*scale);
      polyLL.Set(verticesLL, 4);
      b2FixtureDef wedgefdLL;
      wedgefdLL.shape = &polyLL;
      wedgefdLL.density = 1.0f;
      wedgefdLL.friction = 0.0f;
      wedgefdLL.restitution = 0.0f;
      b2BodyDef wedgebdLL;
      wedgebdLL.position.Set(27.0f*scale+X_increase, 2.50f*scale+Y_increase);
      wedgebdLL.type = b2_dynamicBody;
      obodyLL = m_world->CreateBody(&wedgebdLL);
      obodyLL->CreateFixture(&wedgefdLL);

//joint LL and L
b2RevoluteJointDef jdLL1,jdLL2;
      b2Vec2 anchorLL1,anchorLL2;
      anchorLL1.Set(27.0f*scale+X_increase, 2.50f*scale+Y_increase);
      anchorLL2.Set(27.4f*scale+X_increase, 3.10f*scale+Y_increase);
      jdLL1.Initialize(sbody, obodyLL, anchorLL1);
      m_world->CreateJoint(&jdLL1);
      jdLL2.Initialize(sbody, obodyLL, anchorLL2);
      m_world->CreateJoint(&jdLL2);

// central R
b2Body* obodyR;
      b2PolygonShape polyR;
      b2Vec2 verticesR[4];
      verticesR[0].Set(0.0f*scale,0.0f*scale);
      verticesR[1].Set(2.0f*scale,0.65f*scale);
      verticesR[2].Set(2.30f*scale,1.65f*scale);
      verticesR[3].Set(0.6f*scale,1.65f*scale);
      polyR.Set(verticesR, 4);
      b2FixtureDef wedgefdR;
      wedgefdR.shape = &polyR;
      wedgefdR.density = 1.0f;
      wedgefdR.friction = 0.0f;
      wedgefdR.restitution = 0.0f;
      b2BodyDef wedgebdR;
      wedgebdR.position.Set(33.0f*scale+X_increase, 1.35f*scale+Y_increase);
      wedgebdR.type = b2_dynamicBody;
      obodyR = m_world->CreateBody(&wedgebdR);
      obodyR->CreateFixture(&wedgefdR);

//joint R and main
b2RevoluteJointDef jdR1,jdR2;
      b2Vec2 anchorR1,anchorR2;
      anchorR1.Set(33.0f*scale+X_increase, 1.350f*scale+Y_increase);
      anchorR2.Set(33.6f*scale+X_increase, 3.0f*scale+Y_increase);
      jdR1.Initialize(sbody, obodyR, anchorR1);
      m_world->CreateJoint(&jdR1);
      jdR2.Initialize(sbody, obodyR, anchorR2);
      m_world->CreateJoint(&jdR2);

// central RR
b2Body* obodyRR;
      b2PolygonShape polyRR;
      b2Vec2 verticesRR[5];
      verticesRR[0].Set(0.0f*scale,0.0f*scale);
      verticesRR[1].Set(1.5f*scale,-0.2f*scale);
      verticesRR[2].Set(2.5f*scale,-1.0f*scale);
      verticesRR[3].Set(2.5f*scale,-0.30f*scale);
      verticesRR[4].Set(0.3f*scale,1.0f*scale);
      polyRR.Set(verticesRR, 5);
      b2FixtureDef wedgefdRR;
      wedgefdRR.shape = &polyRR;
      wedgefdRR.density = 1.0f;
      wedgefdRR.friction = 0.0f;
      wedgefdRR.restitution = 0.0f;
      b2BodyDef wedgebdRR;
      wedgebdRR.position.Set(35.0f*scale+X_increase, 2.0f*scale+Y_increase);
      wedgebdRR.type = b2_dynamicBody;
      obodyRR = m_world->CreateBody(&wedgebdRR);
      obodyRR->CreateFixture(&wedgefdRR);

//joint RR and R
b2RevoluteJointDef jdRR1,jdRR2;
      b2Vec2 anchorRR1,anchorRR2;
      anchorRR1.Set(35.0f*scale+X_increase, 2.0f*scale+Y_increase);
      anchorRR2.Set(35.3f*scale+X_increase, 3.0f*scale+Y_increase);
      jdRR1.Initialize(sbody, obodyRR, anchorRR1);
      m_world->CreateJoint(&jdRR1);
      jdRR2.Initialize(sbody, obodyRR, anchorRR2);
      m_world->CreateJoint(&jdRR2);
// central F
b2Body* obodyF;
      b2PolygonShape polyF;
      b2Vec2 verticesF[4];
      verticesF[0].Set(-0.375f*scale,0.75f*scale);
      verticesF[1].Set(-0.775f*scale,0.75*scale);
      verticesF[2].Set(2.75f*scale,-4.8f*scale);
      verticesF[3].Set(3.15f*scale,-4.8f*scale);
      polyF.Set(verticesF, 4);
      b2FixtureDef wedgefdF;
      wedgefdF.shape = &polyF;
      wedgefdF.density = 1.0f;
      wedgefdF.friction = 0.0f;
      wedgefdF.restitution = 0.0f;
      b2BodyDef wedgebdF;
      wedgebdF.position.Set(33.35f*scale+X_increase, 3.5f*scale+Y_increase);
      wedgebdF.type = b2_dynamicBody;
      obodyF = m_world->CreateBody(&wedgebdF);
      obodyF->CreateFixture(&wedgefdF);

//joint F and main
b2RevoluteJointDef jdF1,jdF2,jdF3;
      b2Vec2 anchorF1,anchorF2,anchorF3;
      anchorF1.Set(33.5f*scale+X_increase, 3.5f*scale+Y_increase);
      anchorF2.Set(27.0f*scale+X_increase, 3.4f*scale+Y_increase);
      //anchorF3.Set(33.6f, 3.0f);
      jdF1.Initialize(sbody, obodyF, anchorF1);
      m_world->CreateJoint(&jdF1);
      jdF2.Initialize(sbody, obodyF, anchorF2);
      m_world->CreateJoint(&jdF2);
      //jdF3.Initialize(sbody, obodyF, anchorF3);
      //m_world->CreateJoint(&jdF3);
// central B
b2Body* obodyB;
      b2PolygonShape polyB;
      b2Vec2 verticesB[4];
      verticesB[0].Set(0.0f*scale,0.0f*scale);
      verticesB[1].Set(-0.8f*scale,0.0f*scale);
      verticesB[2].Set(-4.4f*scale,-1.8f*scale);
      verticesB[3].Set(-3.6f*scale,-1.8f*scale);
      polyB.Set(verticesB, 4);
      b2FixtureDef wedgefdB;
      wedgefdB.shape = &polyB;
      wedgefdB.density = 1.0f;
      wedgefdB.friction = 0.0f;
      wedgefdB.restitution = 0.0f;
      b2BodyDef wedgebdB;
      wedgebdB.position.Set(30.4f*scale+X_increase, 1.5f*scale+Y_increase);
      wedgebdB.type = b2_dynamicBody;
      obodyB = m_world->CreateBody(&wedgebdB);
      obodyB->CreateFixture(&wedgefdB);

//joint B and main
b2RevoluteJointDef jdB1,jdB2;
      b2Vec2 anchorB1,anchorB2;
      anchorB1.Set(30.4f*scale+X_increase, 1.5f*scale+Y_increase);
      anchorB2.Set(29.6f*scale+X_increase, 1.5f*scale+Y_increase);
      //anchorF3.Set(33.6f, 3.0f);
      jdB1.Initialize(sbody, obodyB, anchorB1);
      m_world->CreateJoint(&jdB1);
      jdB2.Initialize(sbody, obodyB, anchorB2);
      m_world->CreateJoint(&jdB2);
      //jdF3.Initialize(sbody, obodyF, anchorF3);
      //m_world->CreateJoint(&jdF3);


// central B and T
b2Body* obodyBT;
      b2CircleShape circleBT;
      circleBT.m_radius = 2.0*scale;
	
      b2FixtureDef ballfdBT;
      ballfdBT.shape = &circleBT;
      ballfdBT.density = 1.0f;
      ballfdBT.friction = 0.1f;
      ballfdBT.restitution = 0.0f;
      b2BodyDef ballbdBT;
      ballbdBT.type = b2_dynamicBody;
      ballbdBT.position.Set(26.0f*scale+X_increase, -0.9f*scale+Y_increase);
      obodyBT = m_world->CreateBody(&ballbdBT);
      obodyBT->CreateFixture(&ballfdBT);

//joint B and T
b2RevoluteJointDef jdBT1;
      b2Vec2 anchorBT1;
      anchorBT1.Set(26.0f*scale+X_increase, -0.9f*scale+Y_increase);
      jdBT1.Initialize(sbody, obodyBT, anchorBT1);
      m_world->CreateJoint(&jdBT1);

// central F and T circle
b2Body* obodyFT;
      b2CircleShape circleFT;
      circleFT.m_radius = 2.0*scale;
	
      b2FixtureDef ballfdFT;
      ballfdFT.shape = &circleFT;
      ballfdFT.density = 1.0f;
      ballfdFT.friction = 0.1f;
      ballfdFT.restitution = 0.0f;
      b2BodyDef ballbdFT;
      ballbdFT.type = b2_dynamicBody;
      ballbdFT.position.Set(36.1f*scale+X_increase, -1.2f*scale+Y_increase);
      obodyFT = m_world->CreateBody(&ballbdFT);
      obodyFT->CreateFixture(&ballfdFT);

//joint F and T
b2RevoluteJointDef jdFT1;
      b2Vec2 anchorFT1;
      anchorFT1.Set(36.1f*scale+X_increase, -1.2f*scale+Y_increase);
      jdFT1.Initialize(sbody, obodyFT, anchorFT1);
      m_world->CreateJoint(&jdFT1);
}
//ball at top

{
      b2Body* sbody;
      b2CircleShape circle;
      circle.m_radius = 1.3;
	
      b2FixtureDef ballfd;
      ballfd.shape = &circle;
      ballfd.density = 10.0f;
      ballfd.friction = 0.1f;
      ballfd.restitution = 0.0f;
      b2BodyDef ballbd;
      ballbd.type = b2_dynamicBody;
      ballbd.position.Set(-42.0f, 41.0f);
      sbody = m_world->CreateBody(&ballbd);
      sbody->CreateFixture(&ballfd);
    }
 //apna command
   



/* {
    	b2PolygonShape shape;
      shape.SetAsBox(10.0f, 10.0f);
	
      b2BodyDef bd;
      bd.position.Set(0.0f, 10.0f);
      b2Body* ground = m_world->CreateBody(&bd);
      ground->CreateFixture(&shape, 0.0f);
    }

*/


    //apna command ends

  /*  //Top horizontal shelf
    {
      b2PolygonShape shape;
      shape.SetAsBox(6.0f, 0.25f);
	
      b2BodyDef bd;
      bd.position.Set(-31.0f, 30.0f);
      b2Body* ground = m_world->CreateBody(&bd);
      ground->CreateFixture(&shape, 0.0f);
    }

    //Dominos
    {
      b2PolygonShape shape;
      shape.SetAsBox(0.1f, 1.0f);
	
      b2FixtureDef fd;
      fd.shape = &shape;
      fd.density = 20.0f;
      fd.friction = 0.1f;
		
      for (int i = 0; i < 10; ++i)
	{
	  b2BodyDef bd;
	  bd.type = b2_dynamicBody;
	  bd.position.Set(-35.5f + 1.0f * i, 31.25f);
	  b2Body* body = m_world->CreateBody(&bd);
	  body->CreateFixture(&fd);
	}
    }
      
    //Another horizontal shelf
    {
      b2PolygonShape shape;
      shape.SetAsBox(7.0f, 0.25f, b2Vec2(-20.f,20.f), 0.0f);
	
      b2BodyDef bd;
      bd.position.Set(1.0f, 6.0f);
      b2Body* ground = m_world->CreateBody(&bd);
      ground->CreateFixture(&shape, 0.0f);
    }


    //The pendulum that knocks the dominos off
    {
      b2Body* b2;
      {
	b2PolygonShape shape;
	shape.SetAsBox(0.25f, 1.5f);
	  
	b2BodyDef bd;
	bd.position.Set(-36.5f, 28.0f);
	b2 = m_world->CreateBody(&bd);
	b2->CreateFixture(&shape, 10.0f);
      }
	
      b2Body* b4;
      {
	b2PolygonShape shape;
	shape.SetAsBox(0.25f, 0.25f);
	  
	b2BodyDef bd;
	bd.type = b2_dynamicBody;
	bd.position.Set(-40.0f, 33.0f);
	b4 = m_world->CreateBody(&bd);
	b4->CreateFixture(&shape, 2.0f);
      }
	
      b2RevoluteJointDef jd;
      b2Vec2 anchor;
      anchor.Set(-37.0f, 40.0f);
      jd.Initialize(b2, b4, anchor);
      m_world->CreateJoint(&jd);
    }
      
    //The train of small spheres
    {
      b2Body* spherebody;
	
      b2CircleShape circle;
      circle.m_radius = 0.5;
	
      b2FixtureDef ballfd;
      ballfd.shape = &circle;
      ballfd.density = 1.0f;
      ballfd.friction = 0.0f;
      ballfd.restitution = 0.0f;
	
      for (int i = 0; i < 10; ++i)
	{
	  b2BodyDef ballbd;
	  ballbd.type = b2_dynamicBody;
	  ballbd.position.Set(-22.2f + i*1.0, 26.6f);
	  spherebody = m_world->CreateBody(&ballbd);
	  spherebody->CreateFixture(&ballfd);
	}
    }

    //The pulley system
    {
      b2BodyDef *bd = new b2BodyDef;
      bd->type = b2_dynamicBody;
      bd->position.Set(-10,15);
      bd->fixedRotation = true;
      
      //The open box
      b2FixtureDef *fd1 = new b2FixtureDef;
      fd1->density = 10.0;
      fd1->friction = 0.5;
      fd1->restitution = 0.f;
      fd1->shape = new b2PolygonShape;
      b2PolygonShape bs1;
      bs1.SetAsBox(2,0.2, b2Vec2(0.f,-1.9f), 0);
      fd1->shape = &bs1;
      b2FixtureDef *fd2 = new b2FixtureDef;
      fd2->density = 10.0;
      fd2->friction = 0.5;
      fd2->restitution = 0.f;
      fd2->shape = new b2PolygonShape;
      b2PolygonShape bs2;
      bs2.SetAsBox(0.2,2, b2Vec2(2.0f,0.f), 0);
      fd2->shape = &bs2;
      b2FixtureDef *fd3 = new b2FixtureDef;
      fd3->density = 10.0;
      fd3->friction = 0.5;
      fd3->restitution = 0.f;
      fd3->shape = new b2PolygonShape;
      b2PolygonShape bs3;
      bs3.SetAsBox(0.2,2, b2Vec2(-2.0f,0.f), 0);
      fd3->shape = &bs3;
       
      b2Body* box1 = m_world->CreateBody(bd);
      box1->CreateFixture(fd1);
      box1->CreateFixture(fd2);
      box1->CreateFixture(fd3);

      //The bar
      bd->position.Set(10,15);	
      fd1->density = 34.0;	  
      b2Body* box2 = m_world->CreateBody(bd);
      box2->CreateFixture(fd1);

      // The pulley joint
      b2PulleyJointDef* myjoint = new b2PulleyJointDef();
      b2Vec2 worldAnchorOnBody1(-10, 15); // Anchor point on body 1 in world axis
      b2Vec2 worldAnchorOnBody2(10, 15); // Anchor point on body 2 in world axis
      b2Vec2 worldAnchorGround1(-10, 20); // Anchor point for ground 1 in world axis
      b2Vec2 worldAnchorGround2(10, 20); // Anchor point for ground 2 in world axis
      float32 ratio = 1.0f; // Define ratio
      myjoint->Initialize(box1, box2, worldAnchorGround1, worldAnchorGround2, box1->GetWorldCenter(), box2->GetWorldCenter(), ratio);
      m_world->CreateJoint(myjoint);
    }

    //The revolving horizontal platform
    {
      b2PolygonShape shape;
      shape.SetAsBox(2.2f, 0.2f);
	
      b2BodyDef bd;
      bd.position.Set(14.0f, 14.0f);
      bd.type = b2_dynamicBody;
      b2Body* body = m_world->CreateBody(&bd);
      b2FixtureDef *fd = new b2FixtureDef;
      fd->density = 1.f;
      fd->shape = new b2PolygonShape;
      fd->shape = &shape;
      body->CreateFixture(fd);

      b2PolygonShape shape2;
      shape2.SetAsBox(0.2f, 2.0f);
      b2BodyDef bd2;
      bd2.position.Set(14.0f, 16.0f);
      b2Body* body2 = m_world->CreateBody(&bd2);

      b2RevoluteJointDef jointDef;
      jointDef.bodyA = body;
      jointDef.bodyB = body2;
      jointDef.localAnchorA.Set(0,0);
      jointDef.localAnchorB.Set(0,0);
      jointDef.collideConnected = false;
      m_world->CreateJoint(&jointDef);
    }

    //The heavy sphere on the platform
    {
      b2Body* sbody;
      b2CircleShape circle;
      circle.m_radius = 1.0;
	
      b2FixtureDef ballfd;
      ballfd.shape = &circle;
      ballfd.density = 50.0f;
      ballfd.friction = 0.0f;
      ballfd.restitution = 0.0f;
      b2BodyDef ballbd;
      ballbd.type = b2_dynamicBody;
      ballbd.position.Set(14.0f, 18.0f);
      sbody = m_world->CreateBody(&ballbd);
      sbody->CreateFixture(&ballfd);
    }


    //The see-saw system at the bottom
    {
      //The triangle wedge
      b2Body* sbody;
      b2PolygonShape poly;
      b2Vec2 vertices[3];
      vertices[0].Set(-1,0);
      vertices[1].Set(1,0);
      vertices[2].Set(0,1.5);
      poly.Set(vertices, 3);
      b2FixtureDef wedgefd;
      wedgefd.shape = &poly;
      wedgefd.density = 10.0f;
      wedgefd.friction = 0.0f;
      wedgefd.restitution = 0.0f;
      b2BodyDef wedgebd;
      wedgebd.position.Set(30.0f, 0.0f);
      sbody = m_world->CreateBody(&wedgebd);
      sbody->CreateFixture(&wedgefd);

      //The plank on top of the wedge
      b2PolygonShape shape;
      shape.SetAsBox(15.0f, 0.2f);
      b2BodyDef bd2;
      bd2.position.Set(30.0f, 1.5f);
      bd2.type = b2_dynamicBody;
      b2Body* body = m_world->CreateBody(&bd2);
      b2FixtureDef *fd2 = new b2FixtureDef;
      fd2->density = 1.f;
      fd2->shape = new b2PolygonShape;
      fd2->shape = &shape;
      body->CreateFixture(fd2);

      b2RevoluteJointDef jd;
      b2Vec2 anchor;
      anchor.Set(30.0f, 1.5f);
      jd.Initialize(sbody, body, anchor);
      m_world->CreateJoint(&jd);

      //The light box on the right side of the see-saw
      b2PolygonShape shape2;
      shape2.SetAsBox(2.0f, 2.0f);
      b2BodyDef bd3;
      bd3.position.Set(40.0f, 2.0f);
      bd3.type = b2_dynamicBody;
      b2Body* body3 = m_world->CreateBody(&bd3);
      b2FixtureDef *fd3 = new b2FixtureDef;
      fd3->density = 0.01f;
      fd3->shape = new b2PolygonShape;
      fd3->shape = &shape2;
      body3->CreateFixture(fd3);
    }*/
  }

  sim_t *sim = new sim_t("Dominos", dominos_t::create);
}
