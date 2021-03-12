/**
 * @file line_sensor_plugin.h
 *
 * @brief Gazebo plugin to line sensor
 *
 * @author Lucas Haug <lucas.haug@thunderatz.org>
 * @author Felipe Bagni <felipe.bagni@thunderatz.org>
 *
 * @date 03/2021
 *
 * @copyright MIT License - Copyright (c) 2021 ThundeRatz
 */

#ifndef __LINE_SENSOR_PLUGIN_H__
#define __LINE_SENSOR_PLUGIN_H__

#include <gazebo/gazebo.hh>
#include <gazebo/plugins/CameraPlugin.hh>
#include <gazebo/common/Image.hh>
#include <ros/ros.h>

/*****************************************
 * Class Definition
 *****************************************/

namespace gazebo {
    class LineSensorPlugin : public CameraPlugin {
        public:
            LineSensorPlugin();

            void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf);

            void OnNewFrame(const unsigned char* _image,
                            unsigned int _width, unsigned int _height,
                            unsigned int _depth, const std::string& _format);

        private:
            common::Image render;

            std::unique_ptr<ros::NodeHandle> ros_node;
            ros::Publisher brightness_pub;

            std::string brightness_topic;

            int adc_resolution;
    };
}

#endif // __LINE_SENSOR_PLUGIN_H__
