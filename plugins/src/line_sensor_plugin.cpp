/**
 * @file line_sensor_plugin.c
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

#include <gazebo/gazebo.hh>
#include <gazebo/plugins/CameraPlugin.hh>
#include <gazebo/common/Image.hh>
#include <ignition/math/Color.hh>
#include <ignition/math/Vector3.hh>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt32.h>
#include <math.h>
#include "line_sensor_plugin.h"

/*****************************************
 * Private Constants
 *****************************************/

#define GAMMA_VALUE 2.2F

#define IMAGE_WIDTH 480U
#define IMAGE_HEIGHT 480U
#define IMAGE_FORMAT "R8G8B8"

#define HORIZONTAL_FIELD_OF_VIEW 1.2F

#define CLIP_NEAR 0.001F
#define CLIP_FAR 0.1F

#define QUEUE_SIZE 1U

/*****************************************
 * Private Macros
 *****************************************/

#define map(former_value, former_min, former_max, new_min, new_max) \
    new_min + ((former_value - former_min) * (new_max - new_min)) / (former_max - former_min);

#define gamma_compression(value) \
    std::pow(value, 1.0 / GAMMA_VALUE)

#define gamma_expansion(value) \
    std::pow(value, GAMMA_VALUE)

/*****************************************
 * Class Definition
 *****************************************/

namespace gazebo {
    // Register this plugin with the simulator
    GZ_REGISTER_SENSOR_PLUGIN(LineSensorPlugin)

    /*****************************************
    * Public Methods Bodies Definitions
    *****************************************/

    LineSensorPlugin::LineSensorPlugin() : CameraPlugin() {
        gzmsg << "Creating Line Sensor Plugin" << std::endl;
    }

    void LineSensorPlugin::Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf) {
        std::string robot_namespace = "";

        if (_sdf->HasElement("robotNamespace")) {
            robot_namespace += _sdf->Get<std::string>("robotNamespace");
        }

        this->ros_node.reset(new ros::NodeHandle(robot_namespace));

        if (!ros::isInitialized()) {
            ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
                << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
            return;
        }

        if (_sdf->HasElement("brightnessTopic")) {
            this->brightness_topic = _sdf->Get<std::string>("brightnessTopic");

            this->brightness_pub = this->ros_node->advertise<std_msgs::UInt32>(this->brightness_topic, QUEUE_SIZE);
        } else {
            gzerr << "Parameter <brightness_topic> is missing." << std::endl;
        }

        if (_sdf->HasElement("ADCResolution")) {
            this->adc_resolution = _sdf->Get<int>("ADCResolution");
        } else {
            gzerr << "Parameter <adc_resolution> is missing." << std::endl;
        }

        CameraPlugin::Load(_parent, _sdf);

        ignition::math::Angle angle;
        angle.Radian(HORIZONTAL_FIELD_OF_VIEW);

        this->camera->SetHFOV(angle);
        this->camera->SetImageSize(IMAGE_WIDTH, IMAGE_HEIGHT);
        this->camera->SetClipDist(CLIP_NEAR, CLIP_FAR);
        this->format = IMAGE_FORMAT;
    }

    void LineSensorPlugin::OnNewFrame(const unsigned char* _image, unsigned int _width, unsigned int _height, unsigned int _depth,
                    const std::string& _format) {
        this->render.SetFromData(_image, _width, _height, this->render.ConvertPixelFormat(_format));

        ignition::math::Color avg_color = this->render.AvgColor();

        // Gamma compression
        avg_color.R(gamma_compression(avg_color.R()));
        avg_color.G(gamma_compression(avg_color.G()));
        avg_color.B(gamma_compression(avg_color.B()));

        // Calculate brightness
        ignition::math::Vector3f yuv_avg_color = avg_color.YUV();
        float rgb_brightness = yuv_avg_color.X();

        int adc_brightness = map(rgb_brightness, 0, 1, 0, std::pow(2, this->adc_resolution) - 1);

        // Public brightness value
        std_msgs::UInt32 msg;
        msg.data = adc_brightness;

        this->brightness_pub.publish(msg);
    }
}
