#include "TheSkeleton.h"

#define ballnum 56

TheSkeleton::TheSkeleton(Critter::RenderSystem* ComeRenderSystem)
{
	mRenderSystem = ComeRenderSystem;

	mySkeletonSphere = new KinematicBody*[ballnum];

	setupSkeleton();
}

TheSkeleton::~TheSkeleton()
{
	delete mChara;
	delete mRenderSystem;
	delete mySkeletonSphere;
}

void TheSkeleton::giveChara(SinbadCharacterController* ComeChara)
{
	mChara = ComeChara;
}

void TheSkeleton::setupSkeleton()
{
	for(int i=0;i<ballnum;i++)
			mySkeletonSphere[i] =  makeSkeletonSphere(Vec3(0,0,0),7.0f);
}

void TheSkeleton::updateSkeleton()
{
	unsigned* index;
	*index = 0;
	updateBasicSkeleton(index);
	update_humerusLPos_humerusRPos(index);
	update_ulnaLPos_humerusLPos(index);
	update_ulnaRPos_humerusRPos(index);
	update_handLPos_ulnaLPos(index);
	update_handRPos_ulnaRPos(index);
	update_humerusLPos_stomachPos(index);
	update_humerusRPos_stomachPos(index);
	update_thighLPos_stomachPos(index);
	update_thighRPos_stomachPos(index);
	update_calfLPos_thighLPos(index);
	update_calfRPos_thighRPos(index);
	update_footLPos_calfLPos(index);
	update_footRPos_calfRPos(index);

}

void TheSkeleton::updateBasicSkeleton(unsigned* index)
{
	mySkeletonSphere[(*index)++]->moveGlobalPosition(mChara->rootPos);
	mySkeletonSphere[(*index)++]->moveGlobalPosition(mChara->stomachPos);
	mySkeletonSphere[(*index)++]->moveGlobalPosition(mChara->waistPos);
	mySkeletonSphere[(*index)++]->moveGlobalPosition(mChara->chestPos);
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

void TheSkeleton::update_humerusLPos_humerusRPos(unsigned* index)
{
	Vector3 humerusmid = mChara->humerusLPos.midPoint(mChara->humerusRPos);
	mySkeletonSphere[(*index)++]->moveGlobalPosition(humerusmid);
	mySkeletonSphere[(*index)++]->moveGlobalPosition(mChara->humerusLPos.midPoint(humerusmid));
	mySkeletonSphere[(*index)++]->moveGlobalPosition(mChara->humerusRPos.midPoint(humerusmid));

	humerusmid.y += 1;
	mySkeletonSphere[(*index)++]->moveGlobalPosition(humerusmid);
}

void TheSkeleton::update_ulnaLPos_humerusLPos(unsigned* index)
{
	Vector3 midPos = mChara->ulnaLPos.midPoint(mChara->humerusLPos);
	mySkeletonSphere[(*index)++]->moveGlobalPosition(midPos);
	mySkeletonSphere[(*index)++]->moveGlobalPosition(mChara->ulnaLPos.midPoint(midPos));
	mySkeletonSphere[(*index)++]->moveGlobalPosition(mChara->humerusLPos.midPoint(midPos));
}
void TheSkeleton::update_ulnaRPos_humerusRPos(unsigned* index)
{
	Vector3 midPos = mChara->ulnaRPos.midPoint(mChara->humerusRPos);
	mySkeletonSphere[(*index)++]->moveGlobalPosition(midPos);
	mySkeletonSphere[(*index)++]->moveGlobalPosition(mChara->ulnaRPos.midPoint(midPos));
	mySkeletonSphere[(*index)++]->moveGlobalPosition(mChara->humerusRPos.midPoint(midPos));
}
void TheSkeleton::update_handLPos_ulnaLPos(unsigned* index)
{
	Vector3 midPos = mChara->handLPos.midPoint(mChara->ulnaLPos);
	mySkeletonSphere[(*index)++]->moveGlobalPosition(midPos);
	mySkeletonSphere[(*index)++]->moveGlobalPosition(mChara->handLPos.midPoint(midPos));
	mySkeletonSphere[(*index)++]->moveGlobalPosition(mChara->ulnaLPos.midPoint(midPos));
}
void TheSkeleton::update_handRPos_ulnaRPos(unsigned* index)
{
	Vector3 midPos = mChara->handRPos.midPoint(mChara->ulnaRPos);
	mySkeletonSphere[(*index)++]->moveGlobalPosition(midPos);
	mySkeletonSphere[(*index)++]->moveGlobalPosition(mChara->handRPos.midPoint(midPos));
	mySkeletonSphere[(*index)++]->moveGlobalPosition(mChara->ulnaRPos.midPoint(midPos));
}
void TheSkeleton::update_humerusLPos_stomachPos(unsigned* index)
{
	Vector3 stomachLmid = mChara->humerusLPos.midPoint(mChara->stomachPos);
	mySkeletonSphere[(*index)++]->moveGlobalPosition(stomachLmid);
	mySkeletonSphere[(*index)++]->moveGlobalPosition(mChara->humerusLPos.midPoint(stomachLmid));
	mySkeletonSphere[(*index)++]->moveGlobalPosition(mChara->stomachPos.midPoint(stomachLmid));
}
void TheSkeleton::update_humerusRPos_stomachPos(unsigned* index)
{
	Vector3 stomachRmid = mChara->humerusRPos.midPoint(mChara->stomachPos);
	mySkeletonSphere[(*index)++]->moveGlobalPosition(stomachRmid);
	mySkeletonSphere[(*index)++]->moveGlobalPosition(mChara->humerusRPos.midPoint(stomachRmid));
	mySkeletonSphere[(*index)++]->moveGlobalPosition(mChara->stomachPos.midPoint(stomachRmid));
}
void TheSkeleton::update_thighLPos_stomachPos(unsigned* index)
{
	Vector3 midPos = mChara->thighLPos.midPoint(mChara->stomachPos);
	mySkeletonSphere[(*index)++]->moveGlobalPosition(midPos);
	mySkeletonSphere[(*index)++]->moveGlobalPosition(mChara->thighLPos.midPoint(midPos));
	mySkeletonSphere[(*index)++]->moveGlobalPosition(mChara->stomachPos.midPoint(midPos));
}
void TheSkeleton::update_thighRPos_stomachPos(unsigned* index)
{
	Vector3 midPos = mChara->thighRPos.midPoint(mChara->stomachPos);
	mySkeletonSphere[(*index)++]->moveGlobalPosition(midPos);
	mySkeletonSphere[(*index)++]->moveGlobalPosition(mChara->thighRPos.midPoint(midPos));
	mySkeletonSphere[(*index)++]->moveGlobalPosition(mChara->stomachPos.midPoint(midPos));
}
void TheSkeleton::update_calfLPos_thighLPos(unsigned* index)
{
	Vector3 midPos = mChara->calfLPos.midPoint(mChara->thighLPos);
	mySkeletonSphere[(*index)++]->moveGlobalPosition(midPos);
	mySkeletonSphere[(*index)++]->moveGlobalPosition(mChara->calfLPos.midPoint(midPos));
	mySkeletonSphere[(*index)++]->moveGlobalPosition(mChara->thighLPos.midPoint(midPos));
}
void TheSkeleton::update_calfRPos_thighRPos(unsigned* index)
{
	Vector3 midPos = mChara->calfRPos.midPoint(mChara->thighRPos);
	mySkeletonSphere[(*index)++]->moveGlobalPosition(midPos);
	mySkeletonSphere[(*index)++]->moveGlobalPosition(mChara->calfRPos.midPoint(midPos));
	mySkeletonSphere[(*index)++]->moveGlobalPosition(mChara->thighRPos.midPoint(midPos));
}
void TheSkeleton::update_footLPos_calfLPos(unsigned* index)
{
	Vector3 midPos = mChara->footLPos.midPoint(mChara->calfLPos);
	mySkeletonSphere[(*index)++]->moveGlobalPosition(midPos);
	mySkeletonSphere[(*index)++]->moveGlobalPosition(mChara->footLPos.midPoint(midPos));
	mySkeletonSphere[(*index)++]->moveGlobalPosition(mChara->calfLPos.midPoint(midPos));
}
void TheSkeleton::update_footRPos_calfRPos(unsigned* index)
{
	Vector3 midPos = mChara->footRPos.midPoint(mChara->calfRPos);
	mySkeletonSphere[(*index)++]->moveGlobalPosition(midPos);
	mySkeletonSphere[(*index)++]->moveGlobalPosition(mChara->footRPos.midPoint(midPos));
	mySkeletonSphere[(*index)++]->moveGlobalPosition(mChara->calfRPos.midPoint(midPos));
}

KinematicBody* TheSkeleton::makeSkeletonSphere(const Matrix44& globalPose, const float scale)
{
	BodyDescription bodyDescription;
	bodyDescription.mMass = 40.0f;

	SphereDescription mSphereDescription;
	mSphereDescription.mRadius = 0.105f*scale;

	KinematicBody* body = mRenderSystem->createKinematicBody(mSphereDescription, globalPose, "nxogre.orb.mesh",bodyDescription);
	body->getNode()->getSceneNode()->setScale(scale,scale,scale);

	return body;
}

Critter::Body* TheSkeleton::makeSphere(const Matrix44& globalPose, const float radius)
{
	BodyDescription bodyDescription;
	bodyDescription.mMass = 40.0f;

	SphereDescription mSphereDescription;
	mSphereDescription.mRadius = radius;

	Body* sphere = mRenderSystem->createBody(mSphereDescription, globalPose,"sphere.mesh", bodyDescription);
	sphere->getNode()->setScale(0.0095*radius);

	return sphere;
}