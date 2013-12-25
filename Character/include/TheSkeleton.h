///////////////////////////////////////////////////
//This is for setup the Skeleton
//								by Nokex 31/10/2013
///////////////////////////////////////////////////

#include "NxOgre.h"
#include "critter.h"
#include "SinbadCharacterController.h"

using namespace NxOgre;
using namespace Critter; 

class TheSkeleton
{
private:
	Critter::RenderSystem*  mRenderSystem;
	Critter::KinematicBody** mySkeletonSphere;
	SinbadCharacterController* mChara;
	KinematicBody* makeSkeletonSphere(const Matrix44&, const float);

	void updateBasicSkeleton(unsigned*);
	void update_humerusLPos_humerusRPos(unsigned*);
	void update_ulnaLPos_humerusLPos(unsigned*);
	void update_ulnaRPos_humerusRPos(unsigned*);
	void update_handLPos_ulnaLPos(unsigned*);
	void update_handRPos_ulnaRPos(unsigned*);
	void update_humerusLPos_stomachPos(unsigned*);
	void update_humerusRPos_stomachPos(unsigned*);
	void update_thighLPos_stomachPos(unsigned*);
	void update_thighRPos_stomachPos(unsigned*);
	void update_calfLPos_thighLPos(unsigned*);
	void update_calfRPos_thighRPos(unsigned*);
	void update_footLPos_calfLPos(unsigned*);
	void update_footRPos_calfRPos(unsigned*);

public:
	TheSkeleton(Critter::RenderSystem*);
	~TheSkeleton();

	void giveChara(SinbadCharacterController*);

	Body* makeSphere(const Matrix44& ,const float);
	void updateSkeleton();
	void setupSkeleton();


};