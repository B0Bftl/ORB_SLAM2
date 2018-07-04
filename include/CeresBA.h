#ifndef CERESBA_H
#define CERESBA_H

#include "Map.h"
#include "MapPoint.h"
#include "KeyFrame.h"
#include "LoopClosing.h"
#include "Frame.h"


namespace ORB_SLAM2 {


	class CeresBA {

		public:

		void static localBundleAdjustmentCeres(list<MapPoint*>& lLocalMapPoints,
		                                       list<KeyFrame*>& lLocalKeyFrames,
		                                       list<KeyFrame*>& lFixedKeyFrames);

	};

}

#endif // CERESBA_H