// "Wee" 
// A Beastie Test Application
//
// No Ogre Resources are required.
// 1. Just add to a Visual Studio Project with Beastie
// 2. Link to Ogre and OIS
// 3. And Compile!

#include "OGRE/Ogre.h"
#include "OGRE/OgreFontManager.h"
#include "OGRE/OgreTextAreaOverlayElement.h"

#include "OIS/OIS.h"
#include "beastie.h"

typedef Ogre::StringConverter StrConv;

class App* app;

inline Ogre::ColourValue getRGB(float, float, float);

static const unsigned int      WindowResWidth        = 1280;
static const unsigned int      WindowResHeight       = 720;
static const bool              WindowAA              = true;
static const Ogre::ColourValue BackgroundColour      = Ogre::ColourValue(0.1337f, 0.1337f, 0.1337f, 1.0f);
static const Ogre::ColourValue GridColour            = Ogre::ColourValue(0.2000f, 0.2000f, 0.2000f, 1.0f);
static const Ogre::ColourValue GroundColour          = Ogre::ColourValue(0.2337f, 0.2337f, 0.2337f, 1.0f);
static const std::string       GuiFont               = "c:\\windows\\fonts\\arial.ttf";

class App : public Ogre::FrameListener, public OIS::KeyListener, public OIS::MouseListener, public Ogre::WindowEventListener
{
  
 public:
 
 void  makeBeastieScene()
 {
  mGroundPlane = beastie::Plane(Ogre::Vector3::UNIT_Y,  0);
 }
 
 // Point based "particle"
 class Particle
 {
  public:
   
   Particle(const Ogre::Vector3& pos)
    : point(pos) {   }
   
   // Translate the particle only if it doesn't collide with anything static.
   void           move(const Ogre::Vector3& trans) 
   {
    Ogre::Vector3 original = point.getPosition();
    point.setPosition(original + trans);
    for (unsigned int i=0;i < app->mStatics.size();i++)
    {
     beastie::Intersection in = point.intersection(app->mStatics[i]);
     if (in.hit)
     {
      point.setPosition(in.position);
      return;
     }
    }
   }
   
   beastie::Point*       getPoint()
        { return &point; }
   
   beastie::Intersection intersect(beastie::Shape* other)
        { return point.intersection(other); }
   
   Ogre::Vector3         position() const
        { return point.getPosition(); }
   
  protected:
   beastie::Point point;
 };

 beastie::Line                mCameraLine;  // Raycasting line
 beastie::Plane               mGroundPlane; // Ground Plane
 std::vector<beastie::Shape*> mStatics;     // Copy of all "statics"
 std::vector<Particle*>       mParticles;   // Master Copy of all Particles

 ///////////////////////////////////////////////////////////////////////////////////////////
 //
 //  Boring Ogre Init and FrameListener stuff.
 //
 ///////////////////////////////////////////////////////////////////////////////////////////
 
 App()
  : mExit(false),
    mCameraDistance(15),
    mCameraAngle(45),
    mCameraHeight(5),
    mCameraAcceleration(Ogre::Vector3::ZERO),
    mOverlay(0),
    mNameGenerator("Wee"),
    mRayOverlayTimer(0.0f),
    mMouse(0),
    mKeyboard(0)
 {
  
  mRoot = new Ogre::Root();
  
#ifdef _DEBUG
  mRoot->loadPlugin("RenderSystem_Direct3D9_d.dll");
#else
  mRoot->loadPlugin("RenderSystem_Direct3D9.dll");
#endif
  
  Ogre::RenderSystem* renderSys = mRoot->getAvailableRenderers().at(0);
  mRoot->setRenderSystem(renderSys);
  renderSys->setConfigOption("Full Screen", "No");
  renderSys->setConfigOption("Video Mode",
    Ogre::StringConverter::toString(WindowResWidth) + " x " + 
    Ogre::StringConverter::toString(WindowResHeight) + " @ 32-bit colour");
  renderSys->setConfigOption("FSAA", "2");
  mWindow = mRoot->initialise(true, "Crivens! A wee Beastie!");
  
  Ogre::WindowEventUtilities::addWindowEventListener(mWindow, this);
  
  std::string fontDir, fontName;
  {
   size_t fontDelim = GuiFont.find_last_of('\\');
   fontDir = GuiFont.substr(0, fontDelim);
   fontName = GuiFont.substr(fontDelim+1);
  }
  
  Ogre::ResourceGroupManager::getSingletonPtr()->addResourceLocation(fontDir, "FileSystem");
  Ogre::ResourcePtr font = Ogre::FontManager::getSingletonPtr()->create("WeeFont", "General");
  font->setParameter("type", "truetype");
  font->setParameter("source", fontName);
  font->setParameter("size", "18");
  font->setParameter("resolution", "96");
  font->load();
  
  mSceneManager = mRoot->createSceneManager(Ogre::ST_GENERIC);
  mCamera = mSceneManager->createCamera("Camera");
  mCamera->setPosition(5,5,5);
  mCamera->lookAt(0,1,0);
  
  mCamera->setNearClipDistance(0.01f);
  mCamera->setFarClipDistance(1000.0f);
  
  Ogre::Viewport* vp = mWindow->addViewport(mCamera);
  vp->setBackgroundColour(BackgroundColour);
  
  mCameraNode = mSceneManager->getRootSceneNode()->createChildSceneNode();
  mCameraNode->attachObject(mCamera);
  mSceneManager->setFog(Ogre::FOG_LINEAR, BackgroundColour, 0.9f, 100.0f, 500.0f); 
  mRoot->addFrameListener(this);
  
  makeBeastieScene();
  
  OIS::ParamList pl;
  size_t windowHnd = 0;
  std::ostringstream windowHndStr;
  
  mWindow->getCustomAttribute("WINDOW", &windowHnd);
  windowHndStr << windowHnd;
  pl.insert(std::make_pair(std::string("WINDOW"), windowHndStr.str()));
  pl.insert(std::make_pair(std::string("w32_mouse"), std::string("DISCL_FOREGROUND" )));
  pl.insert(std::make_pair(std::string("w32_mouse"), std::string("DISCL_NONEXCLUSIVE")));
  
  mInputManager = OIS::InputManager::createInputSystem(pl);
  mKeyboard = static_cast<OIS::Keyboard*>(mInputManager->createInputObject(OIS::OISKeyboard, true));
  mKeyboard->setEventCallback(this);
  mMouse = static_cast<OIS::Mouse*>(mInputManager->createInputObject(OIS::OISMouse, true));
  mMouse->setEventCallback(this);
  
  windowResized(mWindow);
  
  mReferenceObject = new Ogre::ManualObject("ReferenceGrid");
  mReferenceObject->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_LINE_LIST);

  mReferenceObject->position(0,0,0);  mReferenceObject->colour(0.3f,0.6f,0.3f,1.0f);
  mReferenceObject->position(0,1,0);  mReferenceObject->colour(0.3f,0.3f,0.3f,1.0f);
  
  Ogre::Vector3 s = Ogre::Vector3(-50.0f,0.0f,-50.0f);
  Ogre::Vector3 a;a.y=-0.01f;
  for (unsigned int i=0; i < 100;++i)
  {
   a.x = 0.0f;  a.z = Ogre::Real(i);
   mReferenceObject->position(a+s); mReferenceObject->colour(GroundColour);
   a.x = 50.0f; a.z = Ogre::Real(i);
   if (i!=50)
   {
    mReferenceObject->position(a+s); mReferenceObject->colour(GridColour);
    mReferenceObject->position(a+s); mReferenceObject->colour(GridColour);
   }
   else
   {
    mReferenceObject->position(a+s); mReferenceObject->colour(Ogre::ColourValue(0.6f,0.3f,0.3f));
    mReferenceObject->position(a+s); mReferenceObject->colour(Ogre::ColourValue(0.6f,0.3f,0.3f));
   }
   a.x = 100.0f; a.z = Ogre::Real(i);
   mReferenceObject->position(a+s); mReferenceObject->colour(GroundColour);
   a.x = Ogre::Real(i);  a.z = 0.0f;
   mReferenceObject->position(a+s); mReferenceObject->colour(GroundColour);
   a.x = Ogre::Real(i); a.z = 50.0f;
   if (i!=50)
   {
    mReferenceObject->position(a+s); mReferenceObject->colour(GridColour);
    mReferenceObject->position(a+s); mReferenceObject->colour(GridColour);
   }
   else
   {
    mReferenceObject->position(a+s); mReferenceObject->colour(Ogre::ColourValue(0.3f,0.3f,0.6f));
    mReferenceObject->position(a+s); mReferenceObject->colour(Ogre::ColourValue(0.3f,0.3f,0.6f));
   }
   a.x = Ogre::Real(i); a.z = 100.0f;
   mReferenceObject->position(a+s); mReferenceObject->colour(GroundColour);
  }
  mReferenceObject->end();
  
  mReferenceObject->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_LINE_LIST);
  mReferenceObject->position( -1E3,-1E3,-1E3); mReferenceObject->colour(GridColour);
  mReferenceObject->position( -1E3,-1E3,-1E3); mReferenceObject->colour(GridColour);
  mReferenceObject->end();

  mReferenceObject->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_TRIANGLE_LIST);
  mReferenceObject->position( 1000, -1.f, 1000); mReferenceObject->colour(GroundColour);
  mReferenceObject->position( 1000, -1.f,-1000); mReferenceObject->colour(GroundColour);
  mReferenceObject->position(-1000, -1.f,-1000); mReferenceObject->colour(GroundColour);
  mReferenceObject->position( 1000, -1.f, 1000); mReferenceObject->colour(GroundColour);
  mReferenceObject->position(-1000, -1.f,-1000); mReferenceObject->colour(GroundColour);
  mReferenceObject->position(-1000, -1.f, 1000); mReferenceObject->colour(GroundColour);
  mReferenceObject->end();
  
  
  mSceneManager->getRootSceneNode()->attachObject(mReferenceObject);
  
  mFPSOverlay = createText("0", 5, 5);
  createText("FPS", 30, 5, true);
  
  mRayOverlay = createText("", 5, mWindow->getHeight() - 25.f);
  mStatics.push_back(&mGroundPlane);
  
 }
 
 ~App()
 {
  delete mRoot;
 }
 
 void go()
 {
  mRoot->startRendering();
 }
 
 bool frameStarted(const Ogre::FrameEvent& evt)
 {
  
  mKeyboard->capture();
  mMouse->capture();
  
  if (mExit || mKeyboard->isKeyDown(OIS::KC_ESCAPE) || mWindow->isClosed())
   return false;
  
  const OIS::MouseState& ms = mMouse->getMouseState();
  
  if (ms.buttonDown(OIS::MB_Right))
  {
   mCamera->yaw(Ogre::Radian(ms.X.rel * -0.013f));
   mCamera->pitch(Ogre::Radian(ms.Y.rel * -0.013f));
  }
  
  if (ms.buttonDown(OIS::MB_Left))
   cameraRaycast(ms);
  
  if (mKeyboard->isKeyDown(OIS::KC_A))
   moveCamera(Ogre::Vector3::NEGATIVE_UNIT_X, evt.timeSinceLastFrame);
  else if (mKeyboard->isKeyDown(OIS::KC_D))
   moveCamera(Ogre::Vector3::UNIT_X, evt.timeSinceLastFrame);
  else if (mKeyboard->isKeyDown(OIS::KC_Q))
   moveCamera(Ogre::Vector3::UNIT_Y, evt.timeSinceLastFrame);
  else if (mKeyboard->isKeyDown(OIS::KC_Z))
   moveCamera(Ogre::Vector3::NEGATIVE_UNIT_Y, evt.timeSinceLastFrame);
  else if (mKeyboard->isKeyDown(OIS::KC_W))
   moveCamera(Ogre::Vector3::NEGATIVE_UNIT_Z, evt.timeSinceLastFrame);
  else if (mKeyboard->isKeyDown(OIS::KC_S))
   moveCamera(Ogre::Vector3::UNIT_Z, evt.timeSinceLastFrame);
  
  updateCamera();
  updateGUI();
  
  if (mRayOverlayTimer > 0)
   mRayOverlayTimer -= evt.timeSinceLastFrame;
  
  if (mRayOverlayTimer < 0)
  {
   mRayOverlay->setCaption("");
   mRayOverlayTimer = 0.0f;
  }
  
  return true;
 }

 void windowResized(Ogre::RenderWindow* rw)
 {
  if (mMouse == 0)
   return;
  
  const OIS::MouseState& ms = mMouse->getMouseState();
  ms.width = rw->getWidth();
  ms.height = rw->getHeight();
 }

 bool keyPressed(const OIS::KeyEvent& e)
 {
  return true;
 }

 bool keyReleased(const OIS::KeyEvent& e)
 {
  if (e.key == OIS::KC_ESCAPE)
   mExit = true;
  return true;
 }
 
 bool mouseMoved( const OIS::MouseEvent &arg )
 {
  if (arg.state.buttonDown(OIS::MB_Left))
   cameraRaycast(arg.state);
  return true;
 }
 
 bool mousePressed( const OIS::MouseEvent &arg, OIS::MouseButtonID id )
 {
  return true;
 }
 
 bool mouseReleased( const OIS::MouseEvent &arg, OIS::MouseButtonID id )
 {
  if (arg.state.buttonDown(OIS::MB_Left))
   cameraRaycast(arg.state);
  return true;
 }
 
 void cameraRaycast(const OIS::MouseState& ms)
 {
  
  Ogre::Real X = Ogre::Real(ms.X.abs) / Ogre::Real(mWindow->getWidth());
  Ogre::Real Y = Ogre::Real(ms.Y.abs) / Ogre::Real(mWindow->getHeight());
  
  mCameraLine = beastie::Utils::getPickLine(mCamera, X, Y);
  mCameraLine.setLength(100);
  beastie::Intersection in;
  for (unsigned int i=0; i < mStatics.size();i++)
  {
   in = mCameraLine.intersection(mStatics[i]);
   if (in.hit)
   {
    std::stringstream s;
    s << "Hit a " << beastie::Utils::toString(mStatics[i]->getShapeType()) << " at " << StrConv::toString(in.position);
    mRayOverlay->setCaption(s.str());
    drawReferenceLineObject(mCameraLine.getPosition(), in.position);
    mRayOverlayTimer = 3.0f;
    return;
   }
  }
  
  drawReferenceLineObject();
  mRayOverlay->setCaption("");
  mRayOverlayTimer = 0.0f;
 }
 
 void moveCamera(const Ogre::Vector3& direction, float modifier)
 {
  mCameraAcceleration += (direction * 2.5 * modifier);
  mCameraAcceleration.x = Ogre::Math::Clamp<Ogre::Real>(mCameraAcceleration.x, -1.0f, 1.0f);
  mCameraAcceleration.y = Ogre::Math::Clamp<Ogre::Real>(mCameraAcceleration.y, -1.0f, 1.0f);
  mCameraAcceleration.z = Ogre::Math::Clamp<Ogre::Real>(mCameraAcceleration.z, -1.0f, 1.0f);
 }
 
 void updateGUI()
 {
  mFPSOverlay->setCaption(StrConv::toString(Ogre::Math::Ceil(mWindow->getLastFPS())));
 }
 
 void updateCamera()
 {
  if (mCameraAcceleration.squaredLength() > 0.0f)
  {
   Ogre::Vector3 position = mCamera->getPosition() + (mCamera->getOrientation() * mCameraAcceleration);
   
   position.x = Ogre::Math::Clamp<Ogre::Real>(position.x, -500.0f, 500.0f);
   position.y = Ogre::Math::Clamp<Ogre::Real>(position.y, 0.5f, 1000.0f);
   position.z = Ogre::Math::Clamp<Ogre::Real>(position.z, -500.0f, 500.0f);

   mCamera->setPosition(position);
   mCameraAcceleration *= 0.75f;
  }
 }
 
 void drawReferenceLineObject(const Ogre::Vector3& a = Ogre::Vector3(-1E3,-1E3,-1E3), const Ogre::Vector3& b = Ogre::Vector3(-1E3,-1E3,-1E3))
 {
  mReferenceObject->beginUpdate(1);
  mReferenceObject->position(a);
  mReferenceObject->colour(GridColour);
  mReferenceObject->position(b);
  mReferenceObject->colour(Ogre::ColourValue::White);
  mReferenceObject->end();
 }
 
 Ogre::OverlayElement*  createText(const std::string& text, float x, float y, bool half = false)
 {
  
  if (mOverlay == 0)
  {
   mOverlay = Ogre::OverlayManager::getSingletonPtr()->create("BeastieOverlay");
   mOverlayPanel = static_cast<Ogre::OverlayContainer*>(Ogre::OverlayManager::getSingletonPtr()->createOverlayElement("Panel", "BeastieOverlayContainer"));
   mOverlay->add2D(mOverlayPanel);
   mOverlayPanel->show();
   mOverlay->show();
  }
  
   Ogre::TextAreaOverlayElement* elem = static_cast<Ogre::TextAreaOverlayElement*>(
     Ogre::OverlayManager::getSingletonPtr()->createOverlayElement("TextArea", mNameGenerator.generate()));
   
   elem->setMetricsMode(Ogre::GMM_PIXELS);
   elem->setPosition(x, y);
   elem->setDimensions(100.0f, 100.0f);
   if (half)
    elem->setCharHeight(14);
   else
    elem->setCharHeight(26);
   elem->setFontName("WeeFont");
   elem->setCaption(text);
   elem->show();
   
   mOverlayPanel->addChild(elem);
   return elem;
 }
 
 Ogre::Root*                  mRoot;
 Ogre::SceneManager*          mSceneManager;
 Ogre::RenderWindow*          mWindow;
 Ogre::Camera*                mCamera;
 Ogre::SceneNode*             mCameraNode;
 Ogre::Vector3                mCameraAcceleration;
 OIS::InputManager*           mInputManager;
 OIS::Keyboard*               mKeyboard;
 OIS::Mouse*                  mMouse;
 Ogre::Degree                 mCameraAngle;
 Ogre::Real                   mCameraDistance;
 Ogre::Real                   mCameraHeight;
 Ogre::ManualObject*          mReferenceObject;
 Ogre::Overlay*               mOverlay;
 Ogre::OverlayContainer*      mOverlayPanel;
 bool                         mExit;
 Ogre::NameGenerator          mNameGenerator;
 Ogre::OverlayElement*        mFPSOverlay;
 Ogre::OverlayElement*        mRayOverlay;
 float                        mRayOverlayTimer;
 
};

inline Ogre::ColourValue getRGB(float r, float g, float b)
   {  return Ogre::ColourValue(r*0.0039f,g*0.0039f,b*0.0039f,1.0f);  }

void Soy();

int main(int argc, char **argv)
{
#ifndef BEASTIE_DOES_TESTING
 Soy();
#else
 app = new App();
 app->go();
 delete app;
#endif
}
