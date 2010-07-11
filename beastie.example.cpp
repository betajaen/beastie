
#include "OGRE/SdkSample.h"
#include "OGRE/SamplePlugin.h"
#include "beastie.h"

using namespace Ogre;
using namespace OgreBites;

class _OgreSampleClassExport BeastieExample : public SdkSample
{
 
public:
 
 beastie::collision_tree* mTree;
 Ogre::ManualObject*      testObj;
 
 void makeScene()
 {
  
  beastie::plane pl;
  pl.distance(300);
  pl.normal(0,1,0);
  
  beastie::box bx;
  bx.max();

  
  mTree = OGRE_NEW beastie::collision_tree();
  
  SceneNode* house = createNodeEntityPair("tudorhouse.mesh", Vector3(-800,550,-800));
  
  mTree->createNode("tudorhouse.mesh", Vector3(-800,550,-800));
  
  SceneNode* house2 = createNodeEntityPair("tudorhouse.mesh", Vector3(-850,550,-850));
  
  mTree->createNode("tudorhouse.mesh", Vector3(-850,550,-850));
  
  mTree->renderVisualDebugger(mSceneMgr->getRootSceneNode());

  testObj = mSceneMgr->createManualObject();
  mSceneMgr->getRootSceneNode()->createChildSceneNode()->attachObject(testObj);
  testObj->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_LINE_LIST);
  testObj->position(0,0,0);
  testObj->position(0,0,0);
  testObj->end();
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

  beastie::intersection_result result;
  
  bool ret = mTree->raycast(ray, 10000, result);
  
  testObj->beginUpdate(0);
  if (ret)
  {
   std::cout << "Distance => " << result.distance << "\n";
   testObj->position(result.hitTriangle.a);
   testObj->position(result.hitTriangle.b);
   testObj->position(result.hitTriangle.b);
   testObj->position(result.hitTriangle.c);
   testObj->position(result.hitTriangle.c);
   testObj->position(result.hitTriangle.a);
   
   Ogre::Vector3 mid = result.hitTriangle.a.midPoint(result.hitTriangle.b);
   mid = mid.midPoint(result.hitTriangle.c);

   testObj->position(mid);
   testObj->position(mid + (result.hitTriangle.n * 50));
  }
  else
  {
   std::cout << "Did not hit anything\n";
   testObj->position(0,0,0);
   testObj->position(0,0,0);
  }
  testObj->end();

  return true;
 }
 
 bool keyPressed(const OIS::KeyEvent& evt)
 {
  return SdkSample::keyPressed(evt);
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

  makeScene();
  
 }
 
 void cleanupContent()
 {
  destroyScene();
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

