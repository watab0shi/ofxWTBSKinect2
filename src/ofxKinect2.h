#pragma once

#include "ofMain.h"
#include "ofxKinect2Types.h"
#include "utils/DoubleBuffer.h"


// ofxKinect2
//--------------------------------------------------------------------------------
namespace ofxKinect2
{
  void  init();

  class Device;
  class Stream;
  class Mapper;

  class ColorStream;
  class DepthStream;
  class IrStream;
  class BodyIndexStream;

  class Body;
  class BodyStream;

  template< class Interface >
  inline void safe_release( Interface *& _p_release )
  {
    if( _p_release )
    {
      _p_release->Release();
      _p_release = nullptr;
    }
  }
}


// Device
//--------------------------------------------------------------------------------
class ofxKinect2::Device
{
  friend class ofxKinect2::Stream;

public:
  Device();
  ~Device();

  bool setup();
  void update();
  void exit();

  bool isOpen() const
  {
    if( device.kinect2 == nullptr ) return false;

    bool b = false;
    device.kinect2->get_IsOpen( ( BOOLEAN* )&b );
    return b;
  }

  DeviceHandle&       get() { return device; }
  const DeviceHandle& get() const { return device; }

protected:
  DeviceHandle                  device;
  vector< ofxKinect2::Stream* > streams;
};


// Stream
//--------------------------------------------------------------------------------
class ofxKinect2::Stream : public ofThread
{
  friend class ofxKinect2::Device;

public:
  virtual ~Stream()
  {
  }

  virtual bool open();
  virtual void close();

  virtual void update();

  void         draw( float _x = 0, float _y = 0 );
  virtual void draw( float _x, float _y, float _w, float _h );

  bool isOpen() const
  {
    return ( stream.p_audio_beam_frame_reader             != nullptr ) || 
           ( stream.p_body_frame_reader                   != nullptr ) ||
           ( stream.p_body_index_frame_reader             != nullptr ) ||
           ( stream.p_color_frame_reader                  != nullptr ) ||
           ( stream.p_depth_frame_reader                  != nullptr ) ||
           ( stream.p_infrared_frame_reader               != nullptr ) ||
           ( stream.p_long_exposure_infrared_frame_reader != nullptr );
  }

  // getter
  int                         getWidth() const;
  int                         getHeight() const;

  inline bool                 isFrameNew() const { return is_frame_new; }
  inline uint64_t             getFrameTimestamp() const { return kinect2_timestamp; }

  ofTexture&                  getTexture() { return tex; }
  const ofTexture&            getTexture() const { return tex; }

  StreamHandle&               get() { return stream; }
  const StreamHandle&         get() const { return stream; }

  CameraSettingsHandle&       getCameraSettings() { return camera_settings; }
  const CameraSettingsHandle& getCameraSettings() const { return camera_settings; }

  // operator
  operator StreamHandle&() { return stream; }
  operator const StreamHandle&() const { return stream; }

protected:
  Stream()
  {
  }

  void         threadedFunction();
  bool         setup( Device& _device, SensorType _sensor_type );
  virtual bool readFrame();
  void         updateTimestamp( Frame _frame );

  Frame                frame;
  StreamHandle         stream;
  CameraSettingsHandle camera_settings;
  uint64_t             kinect2_timestamp, opengl_timestamp;

  bool                 is_frame_new, texture_needs_update;
  bool                 is_mirror;

  ofTexture            tex;
  Device*              device;
};


// ColorStream
//--------------------------------------------------------------------------------
class ofxKinect2::ColorStream : public ofxKinect2::Stream
{
public:
  ColorStream() : Stream()
  {
  }

  ~ColorStream()
  {
  }

  bool setup( ofxKinect2::Device& _device )
  {
    buffer = nullptr;
    return Stream::setup( _device, SENSOR_COLOR );
  }

  bool open();
  void close();

  void update();

  // getter
  ofColor         getColorAt( int _x, int _y );
  ofColor         getColorAt( ofVec2f color_point );
  ofFloatColor    getFloatColorAt( int _x, int _y );
  ofFloatColor    getFloatColorAt( ofVec2f color_point );

  ofPixels&       getPixels() { return pix.getFrontBuffer(); }
  const ofPixels& getPixels() const { return pix.getFrontBuffer(); }

  int             getExposureTime();
  int             getFrameInterval();
  float           getGain();
  float           getGamma();

protected:
  bool readFrame();
  void setPixels( Frame _frame );

  DoubleBuffer< ofPixels > pix;
  unsigned char*           buffer;
};


// DepthStream
//--------------------------------------------------------------------------------
class ofxKinect2::DepthStream : public ofxKinect2::Stream
{
public:
  DepthStream() : Stream()
  {
  }

  ~DepthStream()
  {
  }

  bool setup( ofxKinect2::Device& _device )
  {
    near_value = 50;
    far_value  = 10000;
    return Stream::setup( _device, SENSOR_DEPTH );
  }

  bool open();
  void close();

  void update();
  
  // setter
  inline void setNear( float _near ){ near_value = _near; }
  inline void setFar( float _far ){ far_value = _far; }
  inline void setInvert( float invert ){ is_invert = invert; }

  // getter
  unsigned short       getDepthAt( int _x, int _y );
  unsigned short       getDepthAt( ofVec2f depth_point );

  ofShortPixels&       getPixels() { return pix.getFrontBuffer(); }
  const ofShortPixels& getPixels() const { return pix.getFrontBuffer(); }

  ofShortPixels&       getPixels( int _near, int _far, bool invert = false );
  const ofShortPixels& getPixels( int _near, int _far, bool invert = false ) const;

  inline float         getFar() const { return far_value; }
  inline float         getNear() const { return near_value; }
  inline bool          getInvert() const { return is_invert; }

protected:
  bool readFrame();
  void setPixels( Frame _frame );

  DoubleBuffer< ofShortPixels > pix;
  float                         near_value;
  float                         far_value;
  bool                          is_invert;
};


// IrStream
//--------------------------------------------------------------------------------
class ofxKinect2::IrStream : public ofxKinect2::Stream
{
public:
  IrStream() : Stream()
  {
  }

  ~IrStream()
  {
  }

  bool setup( ofxKinect2::Device& _device )
  {
    return Stream::setup( _device, SENSOR_IR );
  }

  bool open();
  void close();

  void update();

  ofShortPixels&       getPixels() { return pix.getFrontBuffer(); }
  const ofShortPixels& getPixels() const { return pix.getFrontBuffer(); }

protected:
  bool readFrame();
  void setPixels( Frame _frame );

  DoubleBuffer< ofShortPixels > pix;
};


// BodyIndexStream
//--------------------------------------------------------------------------------
class ofxKinect2::BodyIndexStream : public ofxKinect2::Stream
{
public:
  BodyIndexStream() : Stream()
  {
    colors[ 0 ] = ofColor::red;
    colors[ 1 ] = ofColor::green;
    colors[ 2 ] = ofColor::blue;
    colors[ 3 ] = ofColor::cyan;
    colors[ 4 ] = ofColor::magenta;
    colors[ 5 ] = ofColor::yellow;
  }

  ~BodyIndexStream()
  {
  }

  bool setup( ofxKinect2::Device& _device )
  {
    return Stream::setup( _device, SENSOR_BODY_INDEX );
  }

  bool open();
  void close();

  void update();

  ofPixels&       getPixels() { return pix.getFrontBuffer(); }
  const ofPixels& getPixels() const { return pix.getFrontBuffer(); }

protected:
  bool readFrame();
  void setPixels( Frame _frame );

  DoubleBuffer< ofPixels > pix;
  unsigned char*           buffer;
  ofColor                  colors[ BODY_COUNT ];
};


// Body
//--------------------------------------------------------------------------------
class ofxKinect2::Body
{
  friend class BodyStream;

public:
  typedef ofPtr< Body > Ref;

  Body()
    : left_hand_state( HandState_Unknown )
    , right_hand_state( HandState_Unknown )
    , is_tracked( false )
    , is_update_scale( false )
  {
    joints.resize( JointType_Count );
    joint_points.resize( JointType_Count );
  }

  void setup( ofxKinect2::Device& _device )
  {
    device = &_device;
  }

  void update( IBody* _body );
  void drawBody();
  void drawBone( JointType _joint0, JointType _joint1);
  void drawHands();
  void drawHandLeft();
  void drawHandRight();
  void drawLean();

  void setId( UINT64 _id ){ id = _id; }
  void setTracked( bool _is_tracked){ is_tracked = _is_tracked; }

  inline int              getId() const { return id;}
  inline bool             isTracked() const { return is_tracked;}

  inline HandState        getLeftHandState() const { return left_hand_state; }
  inline HandState        getRightHandState() const { return left_hand_state; }

  inline size_t           getNumJoints(){ return JointType_Count; }

  const Joint&            getJoint( size_t _idx ){ return joints[ _idx ]; }

  const ofPoint&          getJointPoint( size_t _idx ){ return joint_points[ _idx ]; }
  const vector< ofPoint > getJointPoints(){ return joint_points; }
  
  const ofVec2f&          getLean() { return body_lean; }

private:
  ofPoint jointToScreen( const JointType _jointType );
  ofPoint bodyPointToScreen( const CameraSpacePoint& _bodyPoint );

  Device*         device;
  vector<Joint>   joints;
  vector<ofPoint> joint_points;
  bool            is_tracked;
  UINT64          id;
  TrackingState   lean_state;
  ofVec2f         body_lean;
  bool            is_update_scale;

  HandState       left_hand_state;
  HandState       right_hand_state;
};


// BodyStream
//--------------------------------------------------------------------------------
class ofxKinect2::BodyStream : public Stream
{
public:
  BodyStream() : Stream()
  {
  }
  
  ~BodyStream()
  {
  }

  bool setup( ofxKinect2::Device& _device )
  {
    for( int i = 0; i < BODY_COUNT; ++i )
    {
      Body body;
      body.setup( _device );
      bodies.push_back( body );
    }
    return Stream::setup( _device, SENSOR_BODY );
  }
  bool open();
  void close();

  void update();

  void draw();
  void drawBody();
  void drawBone( JointType _joint0, JointType _joint1 );
  void drawHands();
  void drawHandLeft();
  void drawHandRight();
  void drawLean();

  inline size_t        getNumBodies() { return bodies.size(); }
  const vector< Body > getBodies() { return bodies; }

  const Body getBody( size_t _idx )
  {
    for( int i = 0; i < bodies.size(); ++i )
    {
      if( bodies[ i ].getId() == _idx )
      {
        return bodies[ _idx ];
      }
    }
    return bodies[ 0 ];
  }

  ofShortPixels&       getPixels(){ return pix.getFrontBuffer(); }
  const ofShortPixels& getPixels() const { return pix.getFrontBuffer(); }
  ofShortPixels&       getPixels( int _near, int _far, bool invert = false );
  const ofShortPixels& getPixels( int _near, int _far, bool invert = false ) const;

protected:
  bool readFrame();
  void setPixels( Frame _frame );

  DoubleBuffer< ofShortPixels > pix;
  vector< Body >                bodies;
};


// Mapper
//--------------------------------------------------------------------------------
class ofxKinect2::Mapper
{
public:
  Mapper()
    : p_mapper( nullptr )
    , depth_space_points( nullptr )
    , camera_space_points( nullptr )
    , depth_values( nullptr )
    , depth_pixels( nullptr )
    , color_pixels( nullptr )
  {
  }

  ~Mapper()
  {
    exit();
  }

  bool setup( ofxKinect2::Device& _device );

  void exit()
  {
    safe_release( p_mapper );

    if( depth_space_points )
    {
      delete depth_space_points;
      depth_space_points = nullptr;
    }
    if( camera_space_points )
    {
      delete camera_space_points;
      camera_space_points = nullptr;
    }
    if( depth_values )
    {
      delete depth_values;
      depth_values = nullptr;
    }
  }

  // map
  ofVec3f                mapDepthToCameraSpace( int _x, int _y );
  ofVec3f                mapDepthToCameraSpace( ofVec2f _depth_point );
  vector< ofVec3f >      mapDepthToCameraSpace();
  vector< ofVec3f >      mapDepthToCameraSpace( vector< ofVec2f > _depth_points );
  vector< ofVec3f >      mapDepthToCameraSpace( ofRectangle _depth_area );

  ofVec2f                mapDepthToColorSpace( int _x, int _y );
  ofVec2f                mapDepthToColorSpace( ofVec2f depth_point );
  vector< ofVec2f >      mapDepthToColorSpace();
  vector< ofVec2f >      mapDepthToColorSpace( vector< ofVec2f > _depth_points );
  vector< ofVec2f >      mapDepthToColorSpace( ofRectangle _depth_area );

  vector< ofVec3f >      mapColorToCameraSpace();
  vector< ofVec2f >      mapColorToDepthSpace();

  ofVec2f                mapCameraToDepthSpace( float x, float _y, float _z );
  ofVec2f                mapCameraToDepthSpace( ofVec3f _camera_point );
  vector< ofVec2f >      mapCameraToDepthSpace( vector< ofVec3f > _camera_points );

  ofVec2f                mapCameraToColorSpace( float x, float _y, float _z );
  ofVec2f                mapCameraToColorSpace( ofVec3f _camera_point );
  ofVec2f                mapCameraToColorSpace( CameraSpacePoint _camera_point );
  vector< ofVec2f >      mapCameraToColorSpace( vector< ofVec3f > _camera_points );

  // setter
  void setDepthFromShortPixels( const ofShortPixels* _depth_pixels ){ depth_pixels = _depth_pixels; }
  void setDepth( ofxKinect2::DepthStream& _depth_stream ){ depth_pixels = &_depth_stream.getPixels(); }
  void setColorFromPixels( const ofPixels* _color_pixels ){ color_pixels = _color_pixels; }
  void setColor( ofxKinect2::ColorStream& _color_stream ){ color_pixels = &_color_stream.getPixels(); }

  // getter
  vector< ofFloatColor >   getFloatColorsCoordinatesToDepthFrame();
  vector< ofColor >        getColorsCoordinatesToDepthFrame();
  ofPixels                 getColorFrameCoordinatesToDepthFrame();

  ICoordinateMapper*       get() { return p_mapper; }
  const ICoordinateMapper* get() const { return p_mapper; }

  bool                     isReady( bool _depth = true, bool _color = true );

private:
  Device*                device;
  ICoordinateMapper*     p_mapper;
  const ofShortPixels*   depth_pixels;
  const ofPixels*        color_pixels;
  ofPixels               coordinate_color_pixels;

  DepthSpacePoint*       depth_space_points;
  CameraSpacePoint*      camera_space_points;
  UINT16*                depth_values;
  vector< ofVec2f >      camera_to_depth_points;
  vector< ofVec2f >      depth_to_color_points;
  vector< ofVec3f >      depth_to_camera_points;
  vector< ofVec2f >      color_to_depth_points;
  vector< ofVec3f >      color_to_camera_points;

  vector< ofFloatColor > depth_to_float_colors;
  vector< ofColor >      depth_to_colors;
};