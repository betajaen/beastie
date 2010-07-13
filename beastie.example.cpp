#include "OGRE/OGRE.h"
#include "OGRE/SdkSample.h"
#include "OGRE/SamplePlugin.h"

#include "beastie.h"

using namespace Ogre;
using namespace OgreBites;

class _OgreSampleClassExport BeastieExample : public SdkSample
{
 
public:
 
 beastie::collision_tree* mTree;
 beastie::plane           mPlane;

 Ogre::SceneNode*         mVisualDebuggerNode;
 
 void makeScene()
 {
  
  // All our mesh collision detection goes through a collision tree, this is 
  // equivalent to an Ogre SceneManager, or a large portion of it.
  mTree = OGRE_NEW beastie::collision_tree();
  
  // Other low-res collision detection is achieved via the ground plane, such where the mouse clicks.
  mPlane = beastie::plane(Ogre::Vector3::UNIT_Y);
  

  // Create 10 tudor houses, randomly.
  for (unsigned int i=0;i < 10;i++)
  {
   
   Ogre::Vector3 v;
   v.x = Ogre::Math::RangeRandom(-7500,7500);
   v.y = 550;
   v.z = Ogre::Math::RangeRandom(-7500,7500);
   
   // Create the SceneNode/Entity
   createNodeEntityPair("tudorhouse.mesh", v);

   // Create the appropriate collision object at the same position, using the same mesh.
   mTree->createNode("tudorhouse.mesh", v);
   
  }
  
  
  // Create the "VisualDebugger"
  // Need's a node to display it too.
  mVisualDebuggerNode = mSceneMgr->getRootSceneNode()->createChildSceneNode();
  
  // Tell the tree to create/render it once.
  mTree->renderVisualDebugger(mVisualDebuggerNode);
  
  // Then hide it.
  mVisualDebuggerNode->setVisible(false);
  
  
 }

 void destroyScene()
 {
  OGRE_DELETE mTree;
 }

 SceneNode* createNodeEntityPair(const Ogre::String& meshName, const Ogre::Vector3& position, const Ogre::Quaternion& orientation = Ogre::Quaternion::IDENTITY)
 {
  SceneNode* node = mSceneMgr->getRootSceneNode()->createChildSceneNode(position, orientation);
  node->attachObject(  mSceneMgr->createEntity(meshName)  );
  return node;
 }
 
 bool mouseReleased(const OIS::MouseEvent& evt, OIS::MouseButtonID id)
 {
  if (evt.state.buttonDown(OIS::MB_Left) == false)
   SdkSample::mouseReleased(evt, id);
  
  Ogre::Ray ray = mTrayMgr->getCursorRay(mCamera);

  beastie::ray_query result;
  
  Ogre::Timer timer;
  unsigned long rayTime = 0;
  
  timer.reset();
  bool ret = mTree->raycast(ray, 10000, result);
  rayTime = timer.getMicroseconds();
  
  mResultObject->beginUpdate(0);
  
  if (ret)
  {
   // Hit something.

   mResultObject->position(result.hitTriangle.a + (result.hitTriangle.n * 0.3f));
   mResultObject->position(result.hitTriangle.b + (result.hitTriangle.n * 0.3f));
   mResultObject->position(result.hitTriangle.c + (result.hitTriangle.n * 0.3f));
   
   mHitPosLabelX->setCaption(Ogre::StringConverter::toString(result.globalPosition.x));
   mHitPosLabelY->setCaption(Ogre::StringConverter::toString(result.globalPosition.y));
   mHitPosLabelZ->setCaption(Ogre::StringConverter::toString(result.globalPosition.z));

  }
  else
  {
   // Didn't hit anything.
   
   // Try raycasting the ground plane.
   Ogre::Vector3 globalPosition;
  
   if (beastie::intersections::line( beastie::line(ray, 100000) , mPlane, globalPosition ))
   {
    mHitPosLabelX->setCaption(Ogre::StringConverter::toString(globalPosition.x));
    mHitPosLabelY->setCaption(Ogre::StringConverter::toString(globalPosition.y));
    mHitPosLabelZ->setCaption(Ogre::StringConverter::toString(globalPosition.z));

    float const radius = 50;
 
    // accuracy is the count of points (and lines).
    // Higher values make the circle smoother, but may slowdown the performance.
    // The performance also is related to the count of circles.
    float const accuracy = 15;
 
    for(float theta = 0; theta <= 2 * Math::PI; theta += Ogre::Math::PI / accuracy) {
      mResultObject->position(globalPosition.x + (radius * cos(theta)), 0, globalPosition.z + (radius * sin(theta)));
      mResultObject->position(globalPosition);
    }


   }
   else
   {
    
    // Really didn't click on anything at all.
    
    mHitPosLabelX->setCaption("-");
    mHitPosLabelY->setCaption("-");
    mHitPosLabelZ->setCaption("-");
    mResultObject->position(0,0,0);
    mResultObject->position(0,0,0);
    mResultObject->position(0,0,0);
    
   }
   
   
  }
  
  mResultObject->end();

  return true;
 }
 
 bool keyReleased(const OIS::KeyEvent& evt)
 {
  return SdkSample::keyReleased(evt);
 }

 
 bool frameRenderingQueued(const FrameEvent& evt)
 {
  return SdkSample::frameRenderingQueued(evt);
 }

 
 /// Boring setup content here.
 
 void setupContent()
 {
  
  ColourValue background = ColourValue(BackgroundColour);
  mViewport->setBackgroundColour(background);
  mSceneMgr->setFog(Ogre::FOG_LINEAR, BackgroundColour, 0.7f, 15000.0f, 18000.0f); 
  
  // use a small amount of ambient lighting
  mSceneMgr->setAmbientLight( ColourValue(0.5f, 0.5f, 0.5f) );

  // add a bright light above the scene
  Light* light = mSceneMgr->createLight();
  light->setType(Light::LT_POINT);
  light->setPosition(-10, 40, 20);
  light->setSpecularColour(ColourValue::White);
  
  mCamera->setPosition(1500,1200,1500);
  mCamera->lookAt(0,100,0);
  mCamera->setNearClipDistance(2.0f);
  mCamera->setFarClipDistance(100000.0f);
  mCameraMan->setTopSpeed(600);
  setDragLook(true);
  
  mTrayMgr->createLabel(OgreBites::TL_TOP, "GoOn", "Go'on click yer hoose yoh numpty", 280);
  
  mToggleDebugBox = mTrayMgr->createCheckBox(OgreBites::TL_TOPLEFT, "ToggleDebug", "Visual Debugger", 180);
  
  mHitPosLabelX = mTrayMgr->createLabel(OgreBites::TL_TOPRIGHT, "HitPositionX", "-", 140);
  mHitPosLabelY = mTrayMgr->createLabel(OgreBites::TL_TOPRIGHT, "HitPositionY", "-", 140);
  mHitPosLabelZ = mTrayMgr->createLabel(OgreBites::TL_TOPRIGHT, "HitPositionZ", "-", 140);
  
  mReferenceObject = new Ogre::ManualObject("ReferenceGrid");
  mReferenceObject->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_LINE_LIST);

  mReferenceObject->position(0,0,0); mReferenceObject->colour(0.3f,0.6f,0.3f,1.0f);
  mReferenceObject->position(0,100,0); mReferenceObject->colour(0.3f,0.3f,0.3f,1.0f);
  
  Ogre::Vector3 s = Ogre::Vector3(-50.0f,0.0f,-50.0f);
  Ogre::Vector3 a;a.y=0.0f;
  
  for (unsigned int i=0; i < 100;++i)
  {
   a.x = 0.0f; a.z = Ogre::Real(i);
   mReferenceObject->position((a+s)*100); mReferenceObject->colour(GroundColour);
   a.x = 50.0f; a.z = Ogre::Real(i);
   if (i!=50)
   {
    mReferenceObject->position((a+s)*100); mReferenceObject->colour(GridColour);
    mReferenceObject->position((a+s)*100); mReferenceObject->colour(GridColour);
   }
   else
   {
    mReferenceObject->position((a+s)*100); mReferenceObject->colour(Ogre::ColourValue(0.6f,0.3f,0.3f));
    mReferenceObject->position((a+s)*100); mReferenceObject->colour(Ogre::ColourValue(0.6f,0.3f,0.3f));
   }
   a.x = 100.0f; a.z = Ogre::Real(i);
   mReferenceObject->position((a+s)*100); mReferenceObject->colour(GroundColour);
   a.x = Ogre::Real(i); a.z = 0.0f;
   mReferenceObject->position((a+s)*100); mReferenceObject->colour(GroundColour);
   a.x = Ogre::Real(i); a.z = 50.0f;
   if (i!=50)
   {
    mReferenceObject->position((a+s)*100); mReferenceObject->colour(GridColour);
    mReferenceObject->position((a+s)*100); mReferenceObject->colour(GridColour);
   }
   else
   {
    mReferenceObject->position((a+s)*100); mReferenceObject->colour(Ogre::ColourValue(0.3f,0.3f,0.6f));
    mReferenceObject->position((a+s)*100); mReferenceObject->colour(Ogre::ColourValue(0.3f,0.3f,0.6f));
   }
   a.x = Ogre::Real(i); a.z = 100.0f;
   mReferenceObject->position((a+s)*100); mReferenceObject->colour(GroundColour);
  }
  
  mReferenceObject->end();
  
  mReferenceObject->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_LINE_LIST);
  mReferenceObject->position( -1E5,-1E5,-1E5); mReferenceObject->colour(GridColour);
  mReferenceObject->position( -1E5,-1E5,-1E5); mReferenceObject->colour(GridColour);
  mReferenceObject->end();

  mReferenceObject->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_TRIANGLE_LIST);
  mReferenceObject->position( 10000, -10.f, 10000); mReferenceObject->colour(GroundColour);
  mReferenceObject->position( 10000, -10.f,-10000); mReferenceObject->colour(GroundColour);
  mReferenceObject->position(-10000, -10.f,-10000); mReferenceObject->colour(GroundColour);
  mReferenceObject->position( 10000, -10.f, 10000); mReferenceObject->colour(GroundColour);
  mReferenceObject->position(-10000, -10.f,-10000); mReferenceObject->colour(GroundColour);
  mReferenceObject->position(-10000, -10.f, 10000); mReferenceObject->colour(GroundColour);
  mReferenceObject->end();
  
  mSceneMgr->getRootSceneNode()->attachObject(mReferenceObject);

  mResultObject = mSceneMgr->createManualObject();
  mSceneMgr->getRootSceneNode()->createChildSceneNode()->attachObject(mResultObject);

  mResultObject->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_TRIANGLE_LIST);
  mResultObject->position(0,0,0);
  mResultObject->position(0,0,0);
  mResultObject->position(0,0,0);
  mResultObject->end();

  makeScene();
  
 }
 
 void cleanupContent()
 {
  destroyScene();
 }
 
 void checkBoxToggled(CheckBox* box)
 {
  if (box == mToggleDebugBox)
  {
   mVisualDebuggerNode->setVisible(mToggleDebugBox->isChecked());
  }
 }
 
 BeastieExample()
 {
  mInfo["Title"] = "Beastie Example";
  mInfo["Description"] = "Sample example using beastie's collision detection with Ogre";
  mInfo["Thumbnail"] = "thumb_skybox.png";
  mInfo["Category"] = "Collision Detection";
  mInfo["Help"] = "Click on something!";
  
  BackgroundColour = Ogre::ColourValue(0.1337f, 0.1337f, 0.1337f, 1.0f);
  GridColour = Ogre::ColourValue(0.2000f, 0.2000f, 0.2000f, 1.0f);
  GroundColour = Ogre::ColourValue(0.2337f, 0.2337f, 0.2337f, 1.0f);
  
 }

 Ogre::ManualObject*          mReferenceObject;
 Ogre::ColourValue            BackgroundColour;
 Ogre::ColourValue            GridColour;
 Ogre::ColourValue            GroundColour;

 Ogre::ManualObject*          mResultObject;
 
 OgreBites::CheckBox*         mToggleDebugBox;
 OgreBites::Label*            mHitPosLabelX;
 OgreBites::Label*            mHitPosLabelY;
 OgreBites::Label*            mHitPosLabelZ;

};

SamplePlugin* sp;
Sample* s;

extern "C" _OgreSampleExport void dllStartPlugin()
{
s = new BeastieExample;
sp = OGRE_NEW OgreBites::SamplePlugin(s->getInfo()["Title"] + " Sample");
sp->addSample(s);
Ogre::Root::getSingleton().installPlugin(sp);
}

extern "C" _OgreSampleExport void dllStopPlugin()
{
Ogre::Root::getSingleton().uninstallPlugin(sp);
OGRE_DELETE sp;
delete s;
}

