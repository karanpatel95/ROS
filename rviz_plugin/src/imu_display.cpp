#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include <tf/transform_listener.h>

#include <rviz/visualization_manager.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/frame_manager.h>

#include "imu_visual.h"

#include "imu_display.h"

namespace rviz_plugin{
	ImuDisplay::ImuDisplay(){
		color_property_ = new rviz::ColorProperty("Color", QColor(204, 51, 204),
			"Color to draw the acceleration arrows.", this, SLOT( updateColorAndAlpha() ));

		alpha_property_ = new rviz::FloatProperty("Alpha", 1.0,
			"0 is fully transperent, 1.0 is fully opaque.", this, SLOT( updateColorAndAlpha() ));

		history_length_property_ = new rviz::IntProperty("History Length", 1
			"Number of prior measurements to display.", this, SLOT( updateHistoryLength() ));

		history_length_property_->setMin(1);
		history_length_property_->setMax(100000);
	}

	void ImuDisplay::onInitialize(){
		MFDClass::onInitialize();
		updateHistoryLength();
	}

	ImuDisplay::~ImuDisplay(){

	}

	void ImuDisplay::reset(){
		MFDClass::reset();
		visuals_.clear();
	}

	void ImuDisplay::updateColorAndAlpha(){
		float alpha = alpha_property_->getFloat();
		Ogre::ColourValue color = color_property_->getOgreColor();

		for(size_t i = 0; i < visuals_.size(); i++){
			visuals_[i]->setColor(color.r, color.g, color.b. alpha);
		}
	}

	void ImuDisplay::updateHistoryLength(){
		visuals_.rset_capacity(history_length_property_->getInt());
	}

	void ImuDisplay::processMessage(const sensor_msgs::Imu::ConstPtr &msg){
		Ogre::Quaternion orientation;
		Ogre::Vector3 position;
		if(!context_->getFrameManager()->getTransform(msg->header.frame_id,
													  msg->header.stamp,
													  position, orientation)){
			ROS_DEBUG("Error transforming from frame '%s' to frame '%s'",
				msg->header.frame_id.c_str(), qPrintable(fixed_frame_));
			return;
		}
	}

	boost::shared_ptr<ImuVisual> visual;
	if(visuals_.full()){
		visual = visuals_.front();
	}else{
		
	}
}