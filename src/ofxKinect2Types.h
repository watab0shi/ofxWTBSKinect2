#pragma once

#include "ofxKinect2Enums.h"
#include "Kinect.h"

namespace ofxKinect2
{
  union StreamHandle
  {
    IColorFrameReader*                p_color_frame_reader;
    IDepthFrameReader*                p_depth_frame_reader;
    IBodyFrameReader*                 p_body_frame_reader;
    IBodyIndexFrameReader*            p_body_index_frame_reader;
    IAudioBeamFrameReader*            p_audio_beam_frame_reader;
    IInfraredFrameReader*             p_infrared_frame_reader;
    ILongExposureInfraredFrameReader* p_long_exposure_infrared_frame_reader;
  };

  union CameraSettingsHandle
  {
    IColorCameraSettings* p_color_camera_settings;
  };

  union DeviceHandle
  {
    IKinectSensor* kinect2;
  };

  struct Mode
  {
    PixelFormat pixel_format;
    int         resolution_x;
    int         resolution_y;
  };

  struct Frame
  {
    int        data_size;
    void*      data;

    SensorType sensor_type;
    UINT64     timestamp;
    int        frame_index;

    int        width;
    int        height;

    float      horizontal_field_of_view;
    float      vertical_field_of_view;
    float      diagonal_field_of_view;

    Mode       mode;
    int        stride;
  };
}
