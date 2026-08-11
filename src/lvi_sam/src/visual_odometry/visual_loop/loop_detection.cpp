#include "loop_detection.h"
#include <opencv2/imgproc/imgproc.hpp>

LoopDetector::LoopDetector(){}

void LoopDetector::reset()
{
    db.clear();
    image_pool.clear();
    keyframelist.clear();
}


void LoopDetector::loadVocabulary(std::string voc_path)
{
    voc = std::make_unique<BriefVocabulary>(voc_path);
    db.setVocabulary(*voc, false, 0);
}

void LoopDetector::addKeyFrame(std::unique_ptr<KeyFrame> cur_kf, bool flag_detect_loop)
{
	KeyFrame* current = cur_kf.get();
	int loop_index = -1;
    if (flag_detect_loop)
    {
        loop_index = detectLoop(current, current->index);
    }
    else
    {
        addKeyFrameIntoVoc(current);
    }

    // check loop if valid using ransan and pnp
	if (loop_index != -1)
	{
        KeyFrame* old_kf = getKeyFrame(loop_index);

        if (old_kf != nullptr && current->findConnection(old_kf))
        {
            std_msgs::msg::Float64MultiArray match_msg;
            match_msg.data.push_back(current->time_stamp);
            match_msg.data.push_back(old_kf->time_stamp);
            pub_match_msg->publish(match_msg);
        }
	}

    // add keyframe
	keyframelist.push_back(std::move(cur_kf));
}

KeyFrame* LoopDetector::getKeyFrame(int index)
{
    auto it = keyframelist.begin();
    for (; it != keyframelist.end(); it++)   
    {
        if((*it)->index == index)
            break;
    }
    if (it != keyframelist.end())
        return it->get();
    else
        return nullptr;
}

int LoopDetector::detectLoop(KeyFrame* keyframe, int frame_index)
{
    // put image into image_pool; for visualization
    cv::Mat compressed_image;
    if (DEBUG_IMAGE)
    {
        int feature_num = keyframe->keypoints.size();
        cv::resize(keyframe->image, compressed_image, cv::Size(376, 240));
        putText(compressed_image, "feature_num:" + to_string(feature_num), cv::Point2f(10, 10), cv::HersheyFonts::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255));
        image_pool[frame_index] = compressed_image;
    }
    //first query; then add this frame into database!
    QueryResults ret;
    // DBoW2 interprets a negative max_id as "query all entries". Wait until a
    // historical candidate exists before querying. This vendored L1 scorer
    // deliberately also returns the newest database entry as ret[0], which is
    // used below as the neighbour-score baseline; loop candidates come from
    // the remaining, older results.
    if (frame_index >= LOOP_MIN_INDEX_GAP)
        db.query(
            keyframe->brief_descriptors, ret, 4,
            frame_index - LOOP_MIN_INDEX_GAP);
    //printf("query time: %f", t_query.toc());
    //cout << "Searching for Image " << frame_index << ". " << ret << endl;

    db.add(keyframe->brief_descriptors);
    //printf("add feature time: %f", t_add.toc());
    // ret[0] is the nearest neighbour's score. threshold change with neighour score
    
    cv::Mat loop_result;
    if (DEBUG_IMAGE)
    {
        loop_result = compressed_image.clone();
        if (ret.size() > 0)
            putText(loop_result, "neighbour score:" + to_string(ret[0].Score), cv::Point2f(10, 50), cv::HersheyFonts::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255));
    }
    // visual loop result 
    if (DEBUG_IMAGE)
    {
        for (unsigned int i = 0; i < ret.size(); i++)
        {
            int tmp_index = ret[i].Id;
            auto it = image_pool.find(tmp_index);
            cv::Mat tmp_image = (it->second).clone();
            putText(tmp_image, "index:  " + to_string(tmp_index) + "loop score:" + to_string(ret[i].Score), cv::Point2f(10, 50), cv::HersheyFonts::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255));
            cv::hconcat(loop_result, tmp_image, loop_result);
        }
    }
    // a good match with its nerghbour
    bool find_loop = false;
    if (!ret.empty() && ret[0].Score > LOOP_PRIMARY_SCORE_THRESHOLD)
    {
        for (unsigned int i = 1; i < ret.size(); i++)
        {
            //if (ret[i].Score > ret[0].Score * 0.3)
            if (ret[i].Score > LOOP_SECONDARY_SCORE_THRESHOLD)
            {          
                find_loop = true;
                
                if (DEBUG_IMAGE && 0)
                {
                    int tmp_index = ret[i].Id;
                    auto it = image_pool.find(tmp_index);
                    cv::Mat tmp_image = (it->second).clone();
                    putText(tmp_image, "loop score:" + to_string(ret[i].Score), cv::Point2f(10, 50), cv::HersheyFonts::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255));
                    cv::hconcat(loop_result, tmp_image, loop_result);
                }
            }

        }
    }
    
    if (DEBUG_IMAGE)
    {
        cv::imshow("loop_result", loop_result);
        cv::waitKey(20);
    }
    
    if (find_loop)
    {
        int min_index = -1;
        for (unsigned int i = 0; i < ret.size(); i++)
        {
            if (min_index == -1 ||
                (static_cast<int>(ret[i].Id) < min_index &&
                 ret[i].Score > LOOP_SECONDARY_SCORE_THRESHOLD))
                min_index = ret[i].Id;
        }
        return min_index;
    }
    else
        return -1;

}

void LoopDetector::addKeyFrameIntoVoc(KeyFrame* keyframe)
{
    // put image into image_pool; for visualization
    cv::Mat compressed_image;
    if (DEBUG_IMAGE)
    {
        int feature_num = keyframe->keypoints.size();
        cv::resize(keyframe->image, compressed_image, cv::Size(376, 240));
        putText(compressed_image, "feature_num:" + to_string(feature_num), cv::Point2f(10, 10), cv::HersheyFonts::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255));
        image_pool[keyframe->index] = compressed_image;
    }

    db.add(keyframe->brief_descriptors);
}

void LoopDetector::visualizeKeyPoses(double time_cur)
{
    if (keyframelist.empty() || pub_key_pose->get_subscription_count() == 0)
        return;

    visualization_msgs::msg::MarkerArray markerArray;

    int count = 0;
    int count_lim = 10;

    visualization_msgs::msg::Marker markerNode;
    markerNode.header.frame_id = "vins_world";
    markerNode.header.stamp = rclcpp::Time(time_cur * 1e9);
    markerNode.action = visualization_msgs::msg::Marker::ADD;
    markerNode.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    markerNode.ns = "keyframe_nodes";
    markerNode.id = 0;
    markerNode.pose.orientation.w = 1;
    markerNode.scale.x = 0.3; markerNode.scale.y = 0.3; markerNode.scale.z = 0.3; 
    markerNode.color.r = 0; markerNode.color.g = 0.8; markerNode.color.b = 1;
    markerNode.color.a = 1;

    for (auto rit = keyframelist.rbegin(); rit != keyframelist.rend(); ++rit)
    {
        if (count++ > count_lim)
            break;

        geometry_msgs::msg::Point p;
        p.x = (*rit)->origin_vio_T.x();
        p.y = (*rit)->origin_vio_T.y();
        p.z = (*rit)->origin_vio_T.z();
        markerNode.points.push_back(p);
    }

    markerArray.markers.push_back(markerNode);
    pub_key_pose->publish(markerArray);
}
