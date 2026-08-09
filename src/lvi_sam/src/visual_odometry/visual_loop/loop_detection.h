#pragma once

#include "parameters.h"
#include "keyframe.h"

using namespace DVision;
using namespace DBoW2;

class LoopDetector
{
public:

	BriefDatabase db;
	std::unique_ptr<BriefVocabulary> voc;

	map<int, cv::Mat> image_pool;

	list<std::unique_ptr<KeyFrame>> keyframelist;

	LoopDetector();
	void loadVocabulary(std::string voc_path);
	
	void addKeyFrame(std::unique_ptr<KeyFrame> cur_kf, bool flag_detect_loop);
	void addKeyFrameIntoVoc(KeyFrame* keyframe);
	KeyFrame* getKeyFrame(int index);

	void visualizeKeyPoses(double time_cur);

	int detectLoop(KeyFrame* keyframe, int frame_index);
};
