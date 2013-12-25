#ifndef __Character_H__
#define __Character_H__
#define NUM_CLOTHES 7	// in SinBadCharacterController.h & CharacterSample.h
#define SELECT_POS_X -460
#define SELECT_POS_Y -290
#define M_PI    3.14159265358979323846
#define CLICK_CLICK_MAX 70
#define CLICK_LR_MAX 15

#include "SdkSample.h"
#include "SinbadCharacterController.h"
#include "OGRE/SamplePlugin.h"
#include "NxOgre.h"
#include "critter.h"

using namespace Ogre;
using namespace OgreBites;
using namespace NxOgre;
using namespace Critter;

int mySelectItem;


class _OgreSampleClassExport Sample_Character : public SdkSample
{
public:

	NxOgre::World*          mWorld;
	NxOgre::Scene*          mScene;

	float                   mLastTimeStep;
	NxOgre::MeshManager*    mMeshManager;
	NxOgre::Material*       mDefaultMaterial;
	Critter::RenderSystem*  mRenderSystem;
	Critter::Body*          mBody;
	Fluid*                  mFluid;
	FluidEmitter*           mEmitter;
	NxOgre::Mesh*           mFlagMesh;

	Cloth*					mCloth;

	float                   mMaxWind;
	OgreBites::Slider*      mWindSlider;
	OgreBites::Slider*      mClothSlider;

	NxOgre::Mesh*           mOrbMesh;
	KinematicBody*          mySkeletonSphere[128];

	int						mSelectedMesh;
	int						mdata;
	int						wearing_clothes;   // wearing_clothes = -1 => wearing nothing
	int						clickclickcounter;
	bool					clickclickflag;

	bool					clothsleep;
	bool                    debuger;

	NxOgre::VisualDebuggerDescription desc;

	std::vector<Ogre::OverlayContainer*> mylooklook;

	Sample_Character() : mFlagMesh(0)
	{
		mInfo["Title"] = "Character";
		mInfo["Description"] = "A demo showing 3rd-person character control and use of TagPoints.";
		mInfo["Thumbnail"] = "thumb_char.png";
		mInfo["Category"] = "Animation";
		/*mInfo["Help"] = "Use the WASD keys to move Sinbad, and the space bar to jump. "
			"Use mouse to look around and mouse wheel to zoom. Press Q to take out or put back "
"Sinbad's swords. With the swords equipped, you can left click to slice vertically or "
			"right click to slice horizontally. When the swords are not equipped, press E to "
			"start/stop a silly dance routine.";*/
		mInfo["Help"] = "";

		mQuitSlider = NULL;
	}

	Ogre::OverlayElement* mDepthPanel;
	//--------------------------------------modified -------------------------------------
	//Ogre::OverlayElement* mImagePanel;
	//--------------------------------------modified -------------------------------------
	OgreBites::YesNoSlider* mQuitSlider;

	void setupPhysics()
	{
	  clothsleep = false;
	  debuger = true;
	  mdata = 0;
	  mWorld = NxOgre::World::createWorld();

	  NxOgre::ResourceSystem::getSingleton()->openProtocol(new Critter::OgreResourceProtocol());

	  mWorld->getRemoteDebugger()->connect();
  
	  NxOgre::SceneDescription scene_description;
	  scene_description.mGravity = NxOgre::Constants::MEAN_EARTH_GRAVITY;
	  scene_description.mUseHardware = true;

	  mScene = mWorld->createScene(scene_description);
  
	  mScene->getMaterial(0)->setAll(0.1, 0.9, 0.5);
  
	  mMeshManager = NxOgre::MeshManager::getSingleton();

	  mScene->createSceneGeometry(NxOgre::PlaneGeometryDescription());

	  mRenderSystem = new Critter::RenderSystem(mScene, mSceneMgr);

	  desc.showDebug();
	  mRenderSystem->createVisualDebugger(desc);
	  setupSkeleton();
	
	  mMaxWind = 0.0f;
	  //mWindSlider = mTrayMgr->createLongSlider(OgreBites::TL_RIGHT, "Wind", "Wind", 300, 100, 100, 0, 100, 100);
	  //mWindSlider->setValue(mMaxWind, false);
	}

	void makeWall(Vec3 globalPose)
	{
		for(int i=0;i<10;i++)
		{
			for(int j=0;j<10;j++)
			{
				makeBox(Vec3(globalPose.x,i+globalPose.y+0.5,j+globalPose.z));
			}
		}
	}
	/*
	void itemSelected(SelectMenu* menu)
	{
		if (menu->getName() == "ClothesType")
		{
			mSelectedMesh = menu->getSelectionIndex();
			switch(mSelectedMesh)
			{
			case 0:
				break;
			case 1:
				destoryCloth();
				createCloth();
				break;
			case 2:
				destoryCloth();
				break;
			}
		}
	}
	*/
	
	void createCloth()
	{
		std::string mesh;

		 switch(wearing_clothes)
		 {
			 case 0:
				 mesh.assign("ogre://test_cloth5.xcl");
				 mCloth = makeFlag(Vec3(0,6.5,0),"nxogre.metal",mesh);
				 break;
			 case 1:
				 mesh.assign("ogre://test_cloth4.xcl");
				 mCloth = makeFlag(Vec3(0,6.5,0),"nxogre.metal",mesh);
				 break;
			 case 2:
				 mesh.assign("ogre://test_cloth5.xcl");
				 break;
			 default:
				 mesh.assign("ogre://test_cloth5.xcl");
		 }

		 //mCloth = makeFlag(Vec3(0,6.5,0),"nxogre.metal",mesh);
		 clothsleep = true;
	}

	void destoryCloth()
	{
		mRenderSystem->destroyCloth(mCloth);
	}

	void stopPhysics()
	{
		NxOgre::World::destroyWorld();
	}

	Body* makeSphere(const Matrix44& globalPose, const float radius)
	{
		Critter::BodyDescription bodyDescription;
		bodyDescription.mMass = 40.0f;

		NxOgre::SphereDescription mSphereDescription;
		mSphereDescription.mRadius = radius;

		Body* sphere = mRenderSystem->createBody(mSphereDescription, globalPose,"sphere.mesh", bodyDescription);
		sphere->getNode()->setScale(0.0095*radius);

		return sphere;
	}

	void setupSkeleton()
	{
		for(int i=0;i<56;i++)
		{
			if(i==0)
				mySkeletonSphere[i] =  makeSkeletonCapsule(Vec3(0,0,0),Vec3(0,0,0),0.0f,0.3f,8.0f);
			else if(i<12)
				mySkeletonSphere[i] =  makeSkeletonSphere(Vec3(0,0,0),3.0f);
			else if(i<15)
				mySkeletonSphere[i] =  makeSkeletonSphere(Vec3(0,0,0),3.2f);
			else if (i<20)
				mySkeletonSphere[i] =  makeSkeletonSphere(Vec3(0,0,0),1.5f);
				//mySkeletonSphere[i] =  makeSkeletonCapsule(Vec3(0,0,0),Vec3(0,0,1),90.0f,0.2f,6.0f);
			else
				mySkeletonSphere[i] =  makeSkeletonSphere(Vec3(0,0,0),1.5f);
		}
	}

	void updateSkeleton()
	{
		unsigned* index;
		*index = 0;
	
		//mySkeletonSphere[(*index)++]->moveGlobalPosition(mChara->rootPos);
	
		updateBasicSkeleton(index);
		
		update_humerusLPos_humerusRPos(index);
		/*
		//update_ulnaLPos_humerusLPos(index);
		update_ulnaRPos_humerusRPos(index);
		//update_handLPos_ulnaLPos(index);
		update_handRPos_ulnaRPos(index);
		/*
		update_humerusLPos_stomachPos(index);
		update_humerusRPos_stomachPos(index);
		update_thighLPos_stomachPos(index);
		update_thighRPos_stomachPos(index);
		*/
		
		update_calfLPos_thighLPos(index);
		update_calfRPos_thighRPos(index);
		update_footLPos_calfLPos(index);
		update_footRPos_calfRPos(index);
		
	}

	void updateBasicSkeleton(unsigned* index)
	{
		//mySkeletonSphere[(*index)++]->moveGlobalPosition(mChara->rootPos);
		mySkeletonSphere[(*index)++]->moveGlobalPosition(mChara->stomachPos);
		//mySkeletonSphere[(*index)++]->moveGlobalPosition(mChara->waistPos);
		//mySkeletonSphere[(*index)++]->moveGlobalPosition(mChara->chestPos);
		mySkeletonSphere[(*index)++]->moveGlobalPosition(mChara->humerusLPos);
		mySkeletonSphere[(*index)++]->moveGlobalPosition(mChara->humerusRPos);
		mySkeletonSphere[(*index)++]->moveGlobalPosition(mChara->ulnaLPos);
		mySkeletonSphere[(*index)++]->moveGlobalPosition(mChara->ulnaRPos);
		mySkeletonSphere[(*index)++]->moveGlobalPosition(mChara->thighLPos);
		mySkeletonSphere[(*index)++]->moveGlobalPosition(mChara->thighRPos);
		mySkeletonSphere[(*index)++]->moveGlobalPosition(mChara->calfLPos);
		mySkeletonSphere[(*index)++]->moveGlobalPosition(mChara->calfRPos);
		mySkeletonSphere[(*index)++]->moveGlobalPosition(mChara->handLPos);
		mySkeletonSphere[(*index)++]->moveGlobalPosition(mChara->handRPos);
		mySkeletonSphere[(*index)++]->moveGlobalPosition(mChara->footLPos);
		mySkeletonSphere[(*index)++]->moveGlobalPosition(mChara->footRPos);
	}

	void update_chest(unsigned* index, Vector3 humerusL_midpos ,Vector3 humerusR_midpos)
	{
		Vector3 nomal = Nomal_f_3Vetex(mChara->humerusLPos,mChara->humerusRPos,mChara->stomachPos)*0.6;

		Vector3 chest1pos = Vector3(humerusL_midpos.x + nomal.x, humerusL_midpos.y-1, humerusL_midpos.z + nomal.z);
		Vector3 chest2pos = Vector3(humerusR_midpos.x + nomal.x, humerusR_midpos.y-1, humerusR_midpos.z + nomal.z);

		mySkeletonSphere[(*index)++]->moveGlobalPosition(chest1pos);
		mySkeletonSphere[(*index)++]->moveGlobalPosition(chest2pos);
}

	void update_humerusLPos_humerusRPos(unsigned* index)
	{
		Vector3 humerusmid = mChara->humerusLPos.midPoint(mChara->humerusRPos);
		/*
		if(angle >= 180.0f)
			angle = 0.0f;
	
		Vec4 mQuat = Quaternion(Vec3(1,1,0), angle++);
		NxOgre::Matrix44 pose(humerusmid, NxOgre::Quat(mQuat.x,mQuat.y,mQuat.z,mQuat.w));

		mySkeletonSphere[(*index)++]->moveGlobalPose(pose);
		*/
		Vector3 humerusL_midpos = mChara->humerusLPos.midPoint(humerusmid);
		Vector3 humerusR_midpos = mChara->humerusRPos.midPoint(humerusmid);

		update_chest(index,humerusL_midpos.midPoint(humerusmid),humerusR_midpos.midPoint(humerusmid));

		mySkeletonSphere[(*index)++]->moveGlobalPosition(humerusmid);

		mySkeletonSphere[(*index)++]->moveGlobalPosition(humerusL_midpos);
		mySkeletonSphere[(*index)++]->moveGlobalPosition(humerusR_midpos);

		mySkeletonSphere[(*index)++]->moveGlobalPosition(humerusL_midpos.midPoint(humerusmid));
		mySkeletonSphere[(*index)++]->moveGlobalPosition(humerusL_midpos.midPoint(mChara->humerusLPos));

		mySkeletonSphere[(*index)++]->moveGlobalPosition(humerusR_midpos.midPoint(humerusmid));
		mySkeletonSphere[(*index)++]->moveGlobalPosition(humerusR_midpos.midPoint(mChara->humerusRPos));

		humerusmid.y += 1;
		mySkeletonSphere[(*index)++]->moveGlobalPosition(humerusmid);
	}

	void update_ulnaLPos_humerusLPos(unsigned* index)
	{
		Vector3 midPos = mChara->ulnaLPos.midPoint(mChara->humerusLPos);
		mySkeletonSphere[(*index)++]->moveGlobalPosition(midPos);
		mySkeletonSphere[(*index)++]->moveGlobalPosition(mChara->ulnaLPos.midPoint(midPos));
		mySkeletonSphere[(*index)++]->moveGlobalPosition(mChara->humerusLPos.midPoint(midPos));
	}

	void update_ulnaRPos_humerusRPos(unsigned* index)
	{
		Vector3 midPos = mChara->ulnaRPos.midPoint(mChara->humerusRPos);
		mySkeletonSphere[(*index)++]->moveGlobalPosition(midPos);
		mySkeletonSphere[(*index)++]->moveGlobalPosition(mChara->ulnaRPos.midPoint(midPos));
		mySkeletonSphere[(*index)++]->moveGlobalPosition(mChara->humerusRPos.midPoint(midPos));
	}
	void update_handLPos_ulnaLPos(unsigned* index)
	{
		Vector3 midPos = mChara->handLPos.midPoint(mChara->ulnaLPos);
		mySkeletonSphere[(*index)++]->moveGlobalPosition(midPos);
		mySkeletonSphere[(*index)++]->moveGlobalPosition(mChara->handLPos.midPoint(midPos));
		mySkeletonSphere[(*index)++]->moveGlobalPosition(mChara->ulnaLPos.midPoint(midPos));
	}

	void update_handRPos_ulnaRPos(unsigned* index)
	{
		Vector3 midPos = mChara->handRPos.midPoint(mChara->ulnaRPos);
		mySkeletonSphere[(*index)++]->moveGlobalPosition(midPos);
		mySkeletonSphere[(*index)++]->moveGlobalPosition(mChara->handRPos.midPoint(midPos));
		mySkeletonSphere[(*index)++]->moveGlobalPosition(mChara->ulnaRPos.midPoint(midPos));
	}
	void update_humerusLPos_stomachPos(unsigned* index)
	{
		Vector3 stomachLmid = mChara->humerusLPos.midPoint(mChara->stomachPos);
		mySkeletonSphere[(*index)++]->moveGlobalPosition(stomachLmid);
		mySkeletonSphere[(*index)++]->moveGlobalPosition(mChara->humerusLPos.midPoint(stomachLmid));
		mySkeletonSphere[(*index)++]->moveGlobalPosition(mChara->stomachPos.midPoint(stomachLmid));
	}
	void update_humerusRPos_stomachPos(unsigned* index)
	{
		Vector3 stomachRmid = mChara->humerusRPos.midPoint(mChara->stomachPos);
		mySkeletonSphere[(*index)++]->moveGlobalPosition(stomachRmid);
		mySkeletonSphere[(*index)++]->moveGlobalPosition(mChara->humerusRPos.midPoint(stomachRmid));
		mySkeletonSphere[(*index)++]->moveGlobalPosition(mChara->stomachPos.midPoint(stomachRmid));
	}
	void update_thighLPos_stomachPos(unsigned* index)
	{
		Vector3 midPos = mChara->thighLPos.midPoint(mChara->stomachPos);
		mySkeletonSphere[(*index)++]->moveGlobalPosition(midPos);
		mySkeletonSphere[(*index)++]->moveGlobalPosition(mChara->thighLPos.midPoint(midPos));
		mySkeletonSphere[(*index)++]->moveGlobalPosition(mChara->stomachPos.midPoint(midPos));
	}
	void update_thighRPos_stomachPos(unsigned* index)
	{
		Vector3 midPos = mChara->thighRPos.midPoint(mChara->stomachPos);
		mySkeletonSphere[(*index)++]->moveGlobalPosition(midPos);
		mySkeletonSphere[(*index)++]->moveGlobalPosition(mChara->thighRPos.midPoint(midPos));
		mySkeletonSphere[(*index)++]->moveGlobalPosition(mChara->stomachPos.midPoint(midPos));
	}
	void update_calfLPos_thighLPos(unsigned* index)
	{
		Vector3 midPos = mChara->calfLPos.midPoint(mChara->thighLPos);
		mySkeletonSphere[(*index)++]->moveGlobalPosition(midPos);
		mySkeletonSphere[(*index)++]->moveGlobalPosition(mChara->calfLPos.midPoint(midPos));
		mySkeletonSphere[(*index)++]->moveGlobalPosition(mChara->thighLPos.midPoint(midPos));
	}
	void update_calfRPos_thighRPos(unsigned* index)
	{
		Vector3 midPos = mChara->calfRPos.midPoint(mChara->thighRPos);
		mySkeletonSphere[(*index)++]->moveGlobalPosition(midPos);
		mySkeletonSphere[(*index)++]->moveGlobalPosition(mChara->calfRPos.midPoint(midPos));
		mySkeletonSphere[(*index)++]->moveGlobalPosition(mChara->thighRPos.midPoint(midPos));
	}
	void update_footLPos_calfLPos(unsigned* index)
	{
		Vector3 midPos = mChara->footLPos.midPoint(mChara->calfLPos);
		mySkeletonSphere[(*index)++]->moveGlobalPosition(midPos);
		mySkeletonSphere[(*index)++]->moveGlobalPosition(mChara->footLPos.midPoint(midPos));
		mySkeletonSphere[(*index)++]->moveGlobalPosition(mChara->calfLPos.midPoint(midPos));
	}
	void update_footRPos_calfRPos(unsigned* index)
	{
		Vector3 midPos = mChara->footRPos.midPoint(mChara->calfRPos);
		mySkeletonSphere[(*index)++]->moveGlobalPosition(midPos);
		mySkeletonSphere[(*index)++]->moveGlobalPosition(mChara->footRPos.midPoint(midPos));
		mySkeletonSphere[(*index)++]->moveGlobalPosition(mChara->calfRPos.midPoint(midPos));
	}

	Vec4 Quaternion(const Vec3 n, float a)
	{
		float x,y,z,w;

		a = a /360 * (float)M_PI * 2;

		w = cos(a/2);
		x = n.x*sin(a/2);
		y = n.y*sin(a/2);
		z = n.z*sin(a/2);

		return Vec4(x,y,z,w);
	}

	Vector3 Nomal_f_3Vetex(Vector3 a, Vector3 b, Vector3 c)
	{
		Vector3 v = c - a;
		Vector3 u = a - b;
		Vector3 n = v.crossProduct(u).normalisedCopy();

		return n;
	}

	void updateGUI()
	{
		
		for (int i=0; i<NUM_CLOTHES; i++)
		{
			mylooklook[i]->setPosition(SELECT_POS_X + (154*(i-mySelectItem)), SELECT_POS_Y);
			
			Ogre::BorderPanelOverlayElement* frame =
							(Ogre::BorderPanelOverlayElement*)mylooklook[i]->getChildIterator().getNext();
			frame->setDimensions(mylooklook[i]->getWidth() + 16.0, mylooklook[i]->getHeight() + 16.0);
			if (mySelectItem == i)
				frame->setBorderMaterialName("SdkTrays/Frame/Over");
			else 
				frame->setBorderMaterialName("SdkTrays/Frame");

			if (i < mySelectItem -2 || i > mySelectItem +2)
				mylooklook[i]->hide();
			else
				mylooklook[i]->show();
		}		
	}

	KinematicBody* makeSkeletonSphere(const Matrix44& globalPose, const float scale)
	{
		BodyDescription bodyDescription;
		bodyDescription.mMass = 40.0f;

		SphereDescription mSphereDescription;
		mSphereDescription.mRadius = 0.105f*scale;

		KinematicBody* body = mRenderSystem->createKinematicBody(mSphereDescription, globalPose,"sphere.mesh",bodyDescription);
		body->getNode()->getSceneNode()->setScale(scale,scale,scale);

		return body;
	}

	KinematicBody* makeSkeletonCapsule(const Vec3 globalPose, const Vec3 axis,const float angle, const float radius, const float height)
	{
		BodyDescription bodyDescription;
		bodyDescription.mMass = 5.0f;

		CapsuleDescription mCapsuleDescription;
		mCapsuleDescription.mRadius = radius;
		mCapsuleDescription.mHeight = height;

		Vec4 mQuat = Quaternion(Vec3(axis.x,axis.y,axis.z), angle);
		NxOgre::Matrix44 pose(globalPose, NxOgre::Quat(mQuat.x,mQuat.y,mQuat.z,mQuat.w));

		KinematicBody* body = mRenderSystem->createKinematicBody(mCapsuleDescription, pose,"sphere.mesh", bodyDescription);

		return body;
	}


	bool frameRenderingQueued(const FrameEvent& evt)
	{
		// let character update animations and camera
		mChara->addTime(evt.timeSinceLastFrame);
		mWorld->advance(evt.timeSinceLastFrame);

		mLastTimeStep = mScene->getTimeStep().getModified();

		updateSkeleton();
		updateGUI();
		detectPress();
		/*
		if(mdata>=100)
		{
			//std::cout<<mChara->ulnaLPos.x<<" "<<mChara->ulnaLPos.y<<mChara->ulnaLPos.z<<" "<<std::endl
			std::cout<<"hello"<<std::endl;
			mdata=0;
		}
		mdata++;
		*/
		if(clothsleep)
			mCloth->putToSleep();
		
		return SdkSample::frameRenderingQueued(evt);
	}

	Body* makeBox(const Matrix44& globalPose, const Vec3& initialVelocity = Vec3::ZERO)
	{
	  Critter::BodyDescription bodyDescription;
	  bodyDescription.mMass = 40.0f;
	  bodyDescription.mLinearVelocity = initialVelocity;
  
	  Body* box = mRenderSystem->createBody(BoxDescription(1,1,1), globalPose, "cube.1m.mesh", bodyDescription);

	  return box;
	}
	/*
	Cloth* makeFlag(const Vec3& pos, const std::string& flag_image)
	{
	  mFlagMesh = NxOgre::MeshGenerator::makePlane(Vec2(4,2), 0.1, NxOgre::Enums::MeshType_Cloth, "file://flag.xcl");
  
	  NxOgre::ClothDescription desc;
	  desc.mMesh = mFlagMesh;
	  desc.mThickness = 0.2;
	  desc.mFriction = 0.5;
	  desc.mFlags |= NxOgre::Enums::ClothFlags_BendingResistance;
	  desc.mFlags |= NxOgre::Enums::ClothFlags_TwoWayCollisions;
	  desc.mGlobalPose.set(pos);
	  desc.mGlobalPose.set(-NxOgre::Math::HalfPi, NxOgre::Enums::X);
	  Cloth* cloth = mRenderSystem->createCloth(desc);

	  return cloth;
	}
	*/
	
	 Cloth* makeFlag(const Vec3& pos, const std::string& flag_image,const std::string& mesh )
	{
	  //if (mFlagMesh == 0)
	   mFlagMesh = NxOgre::MeshManager::getSingleton()->load(mesh);
	  //mFlagMesh = NxOgre::MeshManager::getSingleton()->load("test_cloth3.xcl");
  
	  // Make the cloth for the flag.
	  NxOgre::ClothDescription desc;
	  desc.mMesh = mFlagMesh;
	  desc.mThickness = 0.8f;
	  desc.mFriction = 1.0f;
	  desc.mDensity = 0.1f;
	  desc.mBendingStiffness = 1.0f;
	  desc.mDampingCoefficient = 0.0f;
	  desc.mStretchingStiffness = 1.0f;
	  desc.mCollisionResponseCoefficient = 0.001f;
	  desc.mFlags |= NxOgre::Enums::ClothFlags_Damping;
      desc.mFlags |= NxOgre::Enums::ClothFlags_COMDamping;
	  //desc.mPressure = 0.0f;
	  //desc.mFlags |= NxOgre::Enums::ClothFlags_DisableSelfCollision;

	  //desc.mFlags |= NxOgre::Enums::ClothFlags_BendingResistance;
	  //desc.mFlags |= NxOgre::Enums::ClothFlags_TwoWayCollisions;
	  desc.mGlobalPose.set(pos);

	  Cloth* cloth = mRenderSystem->createCloth(desc, flag_image);
	  //Cloth* cloth = mRenderSystem->createCloth(desc);

	  //cloth->attachToShape((*geom->getShapes().begin()), 0);
	  //mCloths.push_back(cloth);

	  return cloth;
	}

	void buttonHit(Button* b)
	{
		if (b->getName() == "wear")   // start or stop sample
		{
			printf("wear clothes%d\n", mySelectItem);
			wearing_clothes = mySelectItem;
			createCloth();
		}
		else if (b->getName() == "takeoff")
		{
			printf("put off clothes%d\n", wearing_clothes);
			wearing_clothes = -1;
			destoryCloth();
		}
		else if (b->getName() == "left")
		{
			if (mySelectItem>0)
				mySelectItem -= 1;
		}
		else if (b->getName() == "right")
		{
			if (mySelectItem<NUM_CLOTHES-1)
				mySelectItem+=1;
		}
	}

	void setupGUI()
	{
		clickclickcounter=0;
		clickclickflag = false;
		wearing_clothes = -1;
		mTrayMgr->createButton(TL_RIGHT, "left", "left");
		mTrayMgr->createButton(TL_RIGHT, "right", "right");
		mTrayMgr->createButton(TL_RIGHT, "wear", "wear");
		mTrayMgr->createButton(TL_RIGHT, "takeoff", "put off");

		int pngX;
		pngX = SELECT_POS_X;
		mySelectItem=0;
		Ogre::OverlayManager& om = Ogre::OverlayManager::getSingleton();
		
		for (int i=0; i<NUM_CLOTHES; i++)
		{
			Ogre::String oName = "clothes" + Ogre::StringConverter::toString(i);
			Ogre::String pngName = oName + ".png";
			
			MaterialPtr mat = MaterialManager::getSingleton().create(oName, "Essential");
			TextureUnitState* t =mat->getTechnique(0)->getPass(0)->createTextureUnitState(pngName);
			
			Ogre::BorderPanelOverlayElement* bp = (Ogre::BorderPanelOverlayElement*)
				om.createOverlayElementFromTemplate("SdkTrays/Picture", "BorderPanel", oName);
			bp->setHorizontalAlignment(Ogre::GHA_RIGHT);
			bp->setVerticalAlignment(Ogre::GVA_CENTER);
			bp->setMaterialName(oName);
			bp->setDimensions(128,96);

			mTrayMgr->getTraysLayer()->add2D(bp);
			mylooklook.push_back(bp);

			mylooklook[i]->setMaterialName(oName);

			Ogre::BorderPanelOverlayElement* frame =
							(Ogre::BorderPanelOverlayElement*)mylooklook[i]->getChildIterator().getNext();
			frame->setDimensions(mylooklook[i]->getWidth() + 16.0, mylooklook[i]->getHeight() + 16.0);
			if (mySelectItem==i)
				frame->setBorderMaterialName("SdkTrays/Frame/Over");
			else
				frame->setBorderMaterialName("SdkTrays/Frame");

			mylooklook[i]->setPosition(pngX, SELECT_POS_Y);
			mylooklook[i]->setDimensions(128,96);
			pngX += mylooklook[i]->getWidth() + 16 + 10;
			//*/
		}
	}
	
	void detectPress()
	{
		if (mSceneNode0->getPosition().x > 5 && mSceneNode0->getPosition().y > 11 && clickclickcounter > CLICK_CLICK_MAX)
		{
			clickclickcounter=0;
			printf("put off clothes%d\n", wearing_clothes);
			wearing_clothes = -1;
			destoryCloth();
		}
		else if (mSceneNode0->getPosition().x > 5 && mSceneNode0->getPosition().y > 11)
			clickclickcounter++;
		else if (mSceneNode0->getPosition().y > 12 && clickclickcounter > CLICK_CLICK_MAX)
		{
			clickclickcounter=0;
			printf("wear clothes%d\n", mySelectItem);
			wearing_clothes = mySelectItem;
			createCloth();
		}
		else if (mSceneNode0->getPosition().y > 12)
			clickclickcounter++;
		else if (mSceneNode0->getPosition().x > 6 && clickclickcounter > CLICK_LR_MAX)
		{
			clickclickcounter=0;
			if (mySelectItem<NUM_CLOTHES-1) mySelectItem+=1;
		}
		else if (mSceneNode0->getPosition().x > 6)
			clickclickcounter++;
		else if (mSceneNode1->getPosition().x < -7 && clickclickcounter > CLICK_LR_MAX)
		{
			clickclickcounter=0;
			if (mySelectItem>0) mySelectItem -= 1;
		}
		else if (mSceneNode1->getPosition().x < -6)
			clickclickcounter++;
		else 
			clickclickcounter=0;
	}

	bool keyPressed(const OIS::KeyEvent& evt)
	{
		// relay input events to character controller
		if (!mTrayMgr->isDialogVisible()) mChara->injectKeyDown(evt);

		if (evt.key == OIS::KC_H)
		{
			clothsleep = false;
		}

		if (evt.key == OIS::KC_J)
		{
			debuger = !debuger;
			if(debuger)
				mRenderSystem->createVisualDebugger(desc);
			else
				mRenderSystem->destroyVisualDebugger();
		}

		return SdkSample::keyPressed(evt);
	}
	
	bool keyReleased(const OIS::KeyEvent& evt)
	{
		// relay input events to character controller
		if (!mTrayMgr->isDialogVisible()) mChara->injectKeyUp(evt);
		return SdkSample::keyReleased(evt);
	}

#if OGRE_PLATFORM == OGRE_PLATFORM_IPHONE
	bool touchPressed(const OIS::MultiTouchEvent& evt)
	{
		// relay input events to character controller
		if (!mTrayMgr->isDialogVisible()) mChara->injectMouseDown(evt);
		return SdkSample::touchPressed(evt);
	}

	bool touchMoved(const OIS::MultiTouchEvent& evt)
	{
		// relay input events to character controller
		if (!mTrayMgr->isDialogVisible()) mChara->injectMouseMove(evt);
		return SdkSample::touchMoved(evt);
	}
#else
	bool mouseMoved(const OIS::MouseEvent& evt)
	{
		// relay input events to character controller
		if (!mTrayMgr->isDialogVisible()) mChara->injectMouseMove(evt);
		return SdkSample::mouseMoved(evt);
	}

	bool mousePressed(const OIS::MouseEvent& evt, OIS::MouseButtonID id)
	{
		// relay input events to character controller
		if (!mTrayMgr->isDialogVisible()) mChara->injectMouseDown(evt, id);
		return SdkSample::mousePressed(evt, id);
	}
#endif
	friend class SinbadCharacterController;
protected:

	void SetupDepthMaterial()
	{
		// Create the texture
		TexturePtr texture = TextureManager::getSingleton().createManual(
				"MyDepthTexture", // name
				ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
				TEX_TYPE_2D,      // type
				m_Width, m_Height,         // width & height
				0,                // number of mipmaps
				PF_BYTE_BGRA,     // pixel format
				TU_DYNAMIC_WRITE_ONLY_DISCARDABLE);

		
		// Create a material using the texture
		MaterialPtr material = MaterialManager::getSingleton().create(
				"DepthTextureMaterial", // name
				ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);

		material->getTechnique(0)->getPass(0)->createTextureUnitState("MyDepthTexture");
		material->getTechnique(0)->getPass(0)->setSceneBlending(SBT_TRANSPARENT_ALPHA);
	}

	//----------------------------------------- modified -------------------------------------
	/*
	void SetupImageMaterial()
	{
		// Create the texture
		TexturePtr texture1 = TextureManager::getSingleton().createManual(
				"MyImageTexture", // name
				ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
				TEX_TYPE_2D,      // type
				m_Width, m_Height,         // width & height
				0,                // number of mipmaps
				PF_BYTE_BGRA,     // pixel format
				TU_DYNAMIC_WRITE_ONLY_DISCARDABLE);

		
		// Create a material using the texture
		MaterialPtr material1 = MaterialManager::getSingleton().create(
				"ImageTextureMaterial", // name
				ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);

		material1->getTechnique(0)->getPass(0)->createTextureUnitState("MyImageTexture");
		material1->getTechnique(0)->getPass(0)->setDepthCheckEnabled(false);
        material1->getTechnique(0)->getPass(0)->setDepthWriteEnabled(false);
        material1->getTechnique(0)->getPass(0)->setLightingEnabled(false);
	}

	//----------------------------------------- modified -------------------------------------
	*/

	void setupContent()
	{   
	
	  mSceneNode0 = mSceneMgr->getRootSceneNode()->createChildSceneNode();
	  Entity* mSphere0 = mSceneMgr->createEntity("Sphere1", "sphere.mesh");
	  mSceneNode0->attachObject(mSphere0);
	  mSceneNode0->setPosition(newHandPos);
	  mSceneNode0->setScale(Vector3(0.002,0.002,0.002));
	  mSceneNode0->showBoundingBox(true);

	  mSceneNode1 = mSceneMgr->getRootSceneNode()->createChildSceneNode();
	  Entity* mSphere1 = mSceneMgr->createEntity("Sphere2", "sphere.mesh");
	  mSceneNode1->attachObject(mSphere1);
	  mSceneNode1->setPosition(newlHandPos);
	  mSceneNode1->setScale(Vector3(0.002,0.002,0.002));
	  mSceneNode1->showBoundingBox(true);
	/*
		// set background and some fog
	  mSceneNode0 = mSceneMgr->getRootSceneNode()->createChildSceneNode();
	  Entity* mSphere0 = mSceneMgr->createEntity("Sphere1", "sphere.mesh");
	  mSceneNode0->attachObject(mSphere0);
	  mSceneNode0->setPosition(rootPos);
	  mSceneNode0->setScale(Vector3(0.005,0.005,0.005));
	  mSceneNode0->showBoundingBox(true);

	  mSceneNode1 = mSceneMgr->getRootSceneNode()->createChildSceneNode();
	  Entity* mSphere1 = mSceneMgr->createEntity("Sphere2", "sphere.mesh");
	  mSceneNode1->attachObject(mSphere1);
	  mSceneNode1->setPosition(stomachPos);
	  mSceneNode1->setScale(Vector3(0.005,0.005,0.005));
	  mSceneNode1->showBoundingBox(true);

	  mSceneNode2 = mSceneMgr->getRootSceneNode()->createChildSceneNode();
	  Entity* mSphere2 = mSceneMgr->createEntity("Sphere3", "sphere.mesh");
	  mSceneNode2->attachObject(mSphere2);
	  mSceneNode2->setPosition(waistPos);
	  mSceneNode2->setScale(Vector3(0.005,0.005,0.005));
	  mSceneNode2->showBoundingBox(true);

	  mSceneNode3 = mSceneMgr->getRootSceneNode()->createChildSceneNode();
	  Entity* mSphere3 = mSceneMgr->createEntity("Sphere4", "cube.mesh");
	  mSceneNode3->attachObject(mSphere3);
	  mSceneNode3->setPosition(chestPos);
	  mSceneNode3->setScale(Vector3(0.01,0.02,0.01));
	  mSceneNode3->showBoundingBox(true);

	  mSceneNode4 = mSceneMgr->getRootSceneNode()->createChildSceneNode();
	  Entity* mSphere4 = mSceneMgr->createEntity("Sphere5", "sphere.mesh");
	  mSceneNode4->attachObject(mSphere4);
	  mSceneNode4->setPosition(humerusLPos);
	  mSceneNode4->setScale(Vector3(0.005,0.005,0.005));
	  mSceneNode4->showBoundingBox(true);

	  mSceneNode5 = mSceneMgr->getRootSceneNode()->createChildSceneNode();
	  Entity* mSphere5 = mSceneMgr->createEntity("Sphere6", "sphere.mesh");
	  mSceneNode5->attachObject(mSphere5);
	  mSceneNode5->setPosition(humerusRPos);
	  mSceneNode5->setScale(Vector3(0.005,0.005,0.005));
	  mSceneNode5->showBoundingBox(true);

	  mSceneNode6 = mSceneMgr->getRootSceneNode()->createChildSceneNode();
	  Entity* mSphere6 = mSceneMgr->createEntity("Sphere7", "sphere.mesh");
	  mSceneNode6->attachObject(mSphere6);
	  mSceneNode6->setPosition(ulnaLPos);
	  mSceneNode6->setScale(Vector3(0.005,0.005,0.005));
	  mSceneNode6->showBoundingBox(true);

	  mSceneNode7 = mSceneMgr->getRootSceneNode()->createChildSceneNode();
	  Entity* mSphere7 = mSceneMgr->createEntity("Sphere8", "sphere.mesh");
	  mSceneNode7->attachObject(mSphere7);
	  mSceneNode7->setPosition(ulnaRPos);
	  mSceneNode7->setScale(Vector3(0.005,0.005,0.005));
	  mSceneNode7->showBoundingBox(true);

	   mSceneNode8 = mSceneMgr->getRootSceneNode()->createChildSceneNode();
	  Entity* mSphere8 = mSceneMgr->createEntity("Sphere9", "sphere.mesh");
	  mSceneNode8->attachObject(mSphere8);
	  mSceneNode8->setPosition(thighLPos);
	  mSceneNode8->setScale(Vector3(0.005,0.005,0.005));
	  mSceneNode8->showBoundingBox(true);

	  mSceneNode9 = mSceneMgr->getRootSceneNode()->createChildSceneNode();
	  Entity* mSphere9 = mSceneMgr->createEntity("Sphere10", "sphere.mesh");
	  mSceneNode9->attachObject(mSphere9);
	  mSceneNode9->setPosition(thighRPos);
	  mSceneNode9->setScale(Vector3(0.005,0.005,0.005));
	  mSceneNode9->showBoundingBox(true);

	  mSceneNode10 = mSceneMgr->getRootSceneNode()->createChildSceneNode();
	  Entity* mSphere10 = mSceneMgr->createEntity("Sphere11", "sphere.mesh");
	  mSceneNode10->attachObject(mSphere10);
	  mSceneNode10->setPosition(calfLPos);
	  mSceneNode10->setScale(Vector3(0.005,0.005,0.005));
	  mSceneNode10->showBoundingBox(true);

	  mSceneNode11 = mSceneMgr->getRootSceneNode()->createChildSceneNode();
	  Entity* mSphere11 = mSceneMgr->createEntity("Sphere12", "sphere.mesh");
	  mSceneNode11->attachObject(mSphere11);
	  mSceneNode11->setPosition(calfRPos);
	  mSceneNode11->setScale(Vector3(0.005,0.005,0.005));
	  mSceneNode11->showBoundingBox(true);
  
  
	  mSceneNode12 = mSceneMgr->getRootSceneNode()->createChildSceneNode();
	  Entity* mSphere12 = mSceneMgr->createEntity("Sphere13", "sphere.mesh");
	  mSceneNode12->attachObject(mSphere12);
	  mSceneNode12->setPosition(handLPos);
	  mSceneNode12->setScale(Vector3(0.005,0.005,0.005));
	  mSceneNode12->showBoundingBox(true);

	  mSceneNode13 = mSceneMgr->getRootSceneNode()->createChildSceneNode();
	  Entity* mSphere13 = mSceneMgr->createEntity("Sphere14", "sphere.mesh");
	  mSceneNode13->attachObject(mSphere13);
	  mSceneNode13->setPosition(handRPos);
	  mSceneNode13->setScale(Vector3(0.005,0.005,0.005));
	  mSceneNode13->showBoundingBox(true);

	  mSceneNode14 = mSceneMgr->getRootSceneNode()->createChildSceneNode();
	  Entity* mSphere14 = mSceneMgr->createEntity("Sphere15", "sphere.mesh");
	  mSceneNode14->attachObject(mSphere14);
	  mSceneNode14->setPosition(footLPos);
	  mSceneNode14->setScale(Vector3(0.005,0.005,0.005));
	  mSceneNode14->showBoundingBox(true);

	  mSceneNode15 = mSceneMgr->getRootSceneNode()->createChildSceneNode();
	  Entity* mSphere15 = mSceneMgr->createEntity("Sphere16", "sphere.mesh");
	  mSceneNode15->attachObject(mSphere15);
	  mSceneNode15->setPosition(footRPos);
	  mSceneNode15->setScale(Vector3(0.005,0.005,0.005));
	  mSceneNode15->showBoundingBox(true);
	  */
		setupGUI();		

		mViewport->setBackgroundColour(ColourValue(1.0f, 1.0f, 0.8f));
		mSceneMgr->setFog(Ogre::FOG_LINEAR, ColourValue(1.0f, 1.0f, 0.8f), 0,15, 100);

		// set shadow properties
		mSceneMgr->setShadowTechnique(SHADOWTYPE_TEXTURE_MODULATIVE);
		mSceneMgr->setShadowColour(ColourValue(0.5, 0.5, 0.5));
		mSceneMgr->setShadowTextureSize(1024);
		mSceneMgr->setShadowTextureCount(1);

		// disable default camera control so the character can do its own
		mCameraMan->setStyle(CS_MANUAL);

		// use a small amount of ambient lighting
		mSceneMgr->setAmbientLight(ColourValue(0.3, 0.3, 0.3));

		// add a bright light above the scene
		Light* light = mSceneMgr->createLight();
		light->setType(Light::LT_POINT);
		light->setPosition(-10, 40, 20);
		light->setSpecularColour(ColourValue::White);

		// create a floor mesh resource
		Ogre::MeshManager::getSingleton().createPlane("floor", ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
			Plane(Vector3::UNIT_Y, 0),100, 100, 10, 10, true, 1, 10, 10, Vector3::UNIT_Z);

		// create a floor entity, give it a material, and place it at the origin
        Entity* floor = mSceneMgr->createEntity("Floor", "floor");
        floor->setMaterialName("Examples/Rockwall");
		floor->setCastShadows(false);
        mSceneMgr->getRootSceneNode()->attachObject(floor);

		// create our character controller
		mChara = new SinbadCharacterController(mCamera);

		setupPhysics();
		//setupControls();
		
		
		mTrayMgr->toggleAdvancedFrameStats();
		
		StringVector items;
		//items.push_back("Mirror");
		//items.push_back("Smoothing");
		//ParamsPanel* help = mTrayMgr->createParamsPanel(TL_TOPLEFT, "HelpMessage", 200, items);

		//mChara->m_help = help;
		mChara->m_pTrayMgr = mTrayMgr;

		SetupDepthMaterial();
		
		mDepthPanel = Ogre::OverlayManager::getSingleton().createOverlayElement("Panel","DepthPanel");
		mDepthPanel->setMaterialName("DepthTextureMaterial");
		mDepthPanel->setMetricsMode(Ogre::GMM_RELATIVE);
		mDepthPanel->setWidth(0.25);
		mDepthPanel->setHeight(0.25*m_Height/m_Width);
		mDepthPanel->setHorizontalAlignment(GHA_RIGHT);
		mDepthPanel->setVerticalAlignment(GVA_BOTTOM);
		mDepthPanel->setLeft(-mDepthPanel->getWidth());
		mDepthPanel->setTop(-mDepthPanel->getHeight());
		
		
		//SetupImageMaterial();

		mTrayMgr->getTraysLayer()->add2D((Ogre::OverlayContainer*)mDepthPanel);
		
		mDepthPanel->show();
		
		/////////////////////////////////// background  //////////////////////////////////////////
	    
		// Create background rectangle covering the whole screen
		//Rectangle2D* rect = new Rectangle2D(true);
		//rect->setCorners(-1.0, 1.0, 1.0, -1.0);
		//rect->setMaterial("ImageTextureMaterial");

		// Render the background before everything else
		//rect->setRenderQueueGroup(RENDER_QUEUE_BACKGROUND);

		// Use infinite AAB to always stay visible
	    //AxisAlignedBox aabInf;
		//aabInf.setInfinite();
		//rect->setBoundingBox(aabInf);

		// Attach background to the scene
		//SceneNode* node = mSceneMgr->getRootSceneNode()->createChildSceneNode("Background");
		//node->attachObject(rect);
		
		//node->setVisible(false);

		///////////////////////////////////////////////////////////////////////////////////////////
		
		mTrayMgr->hideLogo();
		//help->setParamValue("Mirror", "M");
		//help->setParamValue("Smoothing", "H/N 0.6");
		
		
		if(mQuitSlider == NULL)
		{
			mQuitSlider = mTrayMgr->createYesNoSlider(TL_NONE,"QuitSlider","Quit?",300,200,0,1,10);
			mQuitSlider->hide();
		}
		
		mChara->m_quitSlider = mQuitSlider;
	}

	void cleanupContent()
	{
		// clean up character controller and the floor mesh
		if (mChara) delete mChara;
		Ogre::MeshManager::getSingleton().remove("floor");
		stopPhysics();
	}

	SinbadCharacterController* mChara;
};

#endif
