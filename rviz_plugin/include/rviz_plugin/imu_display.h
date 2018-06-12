#ifndef IMU_DISPLAY_H
#define IMU_DISPLAY_H

#include <boost/circular_buffer.hpp>

#include <sensor_msgs/Imu.h>
#include <rviz/message_filter_display.h>

namespace Ogre{
	class ScenceNode;
}

namespace rviz{
class ColorProperty;
class FloatProperty;
class IntProperty;
}

namespace rviz_plugin{
class ImuVisual;

class ImuDisplay : public rviz::MessageFilterDisplay<sensor_msgs::Imu>{
Q_OBJECT
public:
    ImuDisplay();
    virtual ~ImuDisplay();
    
protected:
    virtual void onInitialize();
    virtual void reset();
    
private Q_SLOTS:
    void updateColorAndAlpha();
    void updateHistoryLength();

private:
	void processMessage(const sensor_msgs::Imu::ConstPtr &msg);

	boost::circular_buffer<boost::shared_ptr<ImuVisual>> visuals_;

	rviz::ColorProperty* color_property_;
	rviz::FloatProperty* alpha_property_;
	rviz::IntProperty* history_length_property_;
    
};
}

#endif