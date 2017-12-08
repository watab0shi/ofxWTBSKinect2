#include "ofxKinect2.h"
#include "utils\DepthRemapToRange.h"

namespace ofxKinect2
{
  void init()
  {
    static bool inited = false;
    if( inited ) return;
    inited = true;
  }
}

using namespace ofxKinect2;


// Device::Device
//----------------------------------------------------------
Device::Device()
{
  device.kinect2 = nullptr;
}

// Device::~Device
//----------------------------------------------------------
Device::~Device()
{
  exit();
}

// Device::setup
//----------------------------------------------------------
bool Device::setup()
{
  ofxKinect2::init();

  HRESULT hr = GetDefaultKinectSensor( &device.kinect2 );

  if( SUCCEEDED( hr ) )
  {
    if( device.kinect2 )
    {
      device.kinect2->Open();
      return true;
    }
  }

  ofLogWarning( "ofxKinect2::Device" ) << " Kinect v2 not found.";
  return false;
}

// Device::exit
//----------------------------------------------------------
void Device::exit()
{
  if( device.kinect2 ) device.kinect2->Close();

  safe_release( device.kinect2 );

  ofRemove( streams, [ this ]( Stream* _s ){
    _s->close();
    return true;
  } );

  streams.clear();
}

// Device::update
//----------------------------------------------------------
void Device::update()
{
  if( !isOpen() ) return;

  for( auto s : streams )
  {
    s->is_frame_new     = s->kinect2_timestamp != s->opengl_timestamp;
    s->opengl_timestamp = s->kinect2_timestamp;
  }
}


// Stream::setup
//----------------------------------------------------------
bool Stream::setup( Device& _device, SensorType _sensor_type )
{
  if( !_device.isOpen() ) return false;

  _device.streams.push_back( this );
  device               = &_device;
  kinect2_timestamp    = 0;
  opengl_timestamp     = 0;
  frame.sensor_type    = _sensor_type;
  frame.frame_index    = 0;
  frame.stride         = 0;
  frame.data           = nullptr;
  frame.data_size      = 0;
  is_frame_new         = false;
  texture_needs_update = false;

  return true;
}

// Stream::open
//----------------------------------------------------------
bool Stream::open()
{
  startThread();
  return true;
}

// Stream::close
//----------------------------------------------------------
void Stream::close()
{
  while( !lock() )
  {
  }
  {
    frame.frame_index = 0;
    frame.stride = 0;
    frame.data = nullptr;
    frame.data_size = 0;
    stopThread();
    unlock();
  }
}

// Stream::threadedFunction
//----------------------------------------------------------
void Stream::threadedFunction()
{
  while( isThreadRunning() )
  {
    if( lock() )
    {
      if( readFrame() )
      {
        kinect2_timestamp    = frame.timestamp;
        texture_needs_update = true;
      }
      unlock();
    }
  }
}

// Stream::readFrame
//----------------------------------------------------------
bool Stream::readFrame()
{
  return false;
}

// Stream::updateTimestamp
//----------------------------------------------------------
void Stream::updateTimestamp( Frame _frame )
{
  kinect2_timestamp = _frame.timestamp;
}

// Stream::update
//----------------------------------------------------------
void Stream::update()
{
  texture_needs_update = false;
}

// Stream::getWidth
//----------------------------------------------------------
int Stream::getWidth() const
{
  return frame.width;
}

// Stream::getHeight
//----------------------------------------------------------
int Stream::getHeight() const
{
  return frame.height;
}

// Stream::draw
//----------------------------------------------------------
void Stream::draw( float _x, float _y )
{
  draw( _x, _y, getWidth(), getHeight() );
}

void Stream::draw( float _x, float _y, float _w, float _h )
{
  if( texture_needs_update ) update();

  if( tex.isAllocated() )
  {
    ofSetColor( ofColor::white );
    tex.draw( _x, _y, _w, _h );
  }
}




// ColorStream::readFrame
//----------------------------------------------------------
bool ColorStream::readFrame()
{
  bool readed = false;
  if( !stream.p_color_frame_reader )
  {
    ofLogWarning( "ofxKinect2::ColorStream" ) << "Stream is not open.";
    return readed;
  }
  Stream::readFrame();

  IColorFrame* p_frame = nullptr;
  HRESULT      hr      = stream.p_color_frame_reader->AcquireLatestFrame( &p_frame );

  if( SUCCEEDED( hr ) )
  {
    IFrameDescription* p_frame_description = nullptr;
    ColorImageFormat   image_format        = ColorImageFormat_None;

    hr = p_frame->get_RelativeTime( ( INT64* )&frame.timestamp );

    if( SUCCEEDED( hr ) )
    {
      hr = p_frame->get_FrameDescription( &p_frame_description );
    }

    if( SUCCEEDED( hr ) )
    {
      hr = p_frame_description->get_Width( &frame.width );
    }

    if( SUCCEEDED( hr ) )
    {
      hr = p_frame_description->get_Height( &frame.height );
    }

    if( SUCCEEDED( hr ) )
    {
      hr = p_frame_description->get_HorizontalFieldOfView( &frame.horizontal_field_of_view );
    }
    if( SUCCEEDED( hr ) )
    {
      hr = p_frame_description->get_VerticalFieldOfView( &frame.vertical_field_of_view );
    }

    if( SUCCEEDED( hr ) )
    {
      hr = p_frame_description->get_DiagonalFieldOfView( &frame.diagonal_field_of_view );
    }

    if( SUCCEEDED( hr ) )
    {
      hr = p_frame->get_RawColorImageFormat( &image_format );
    }

    if( SUCCEEDED( hr ) )
    {
      if( buffer == nullptr )
      {
        buffer = new unsigned char[ frame.width * frame.height * 4 ];
      }
      if( image_format == ColorImageFormat_Rgba )
      {
        hr = p_frame->AccessRawUnderlyingBuffer( ( UINT* )&frame.data_size, reinterpret_cast< BYTE** >( &frame.data ) );
      }
      else
      {
        frame.data      = buffer;
        frame.data_size = frame.width * frame.height * 4 * sizeof( unsigned char );
        hr = p_frame->CopyConvertedFrameDataToArray( ( UINT )frame.data_size, reinterpret_cast< BYTE* >( frame.data ), ColorImageFormat_Rgba );
      }
    }

    if( SUCCEEDED( hr ) )
    {
      readed = true;
      setPixels( frame );
    }

    safe_release( p_frame_description );
  }

  safe_release( p_frame );

  return readed;
}

// ColorStream::setPixels
//----------------------------------------------------------
void ColorStream::setPixels( Frame _frame )
{
  Stream::updateTimestamp( _frame );

  const unsigned char * src = ( const unsigned char* )_frame.data;
  if( !src ) return;

  pix.getBackBuffer().setFromPixels( src, _frame.width, _frame.height, OF_IMAGE_COLOR_ALPHA );
  pix.swap();
}

// ColorStream::update
//----------------------------------------------------------
void ColorStream::update()
{
  if( !tex.isAllocated() )
  {
    tex.allocate( getWidth(), getHeight(), GL_RGB );
  }

  if( lock() )
  {
    tex.loadData( pix.getFrontBuffer() );
    Stream::update();
    unlock();
  }
}

// ColorStream::open
//----------------------------------------------------------
bool ColorStream::open()
{
  if( !device->isOpen() )
  {
    ofLogWarning( "ofxKinect2::ColorStream" ) << "No ready Kinect2 found.";
    return false;
  }

  IColorFrameSource* p_source = nullptr;
  HRESULT            hr       = device->get().kinect2->get_ColorFrameSource( &p_source );

  if( SUCCEEDED( hr ) )
  {
    hr = p_source->OpenReader( &stream.p_color_frame_reader );
  }

  IFrameDescription* p_frame_description = nullptr;
  p_source->get_FrameDescription( &p_frame_description );
  
  if( SUCCEEDED( hr ) )
  {
    int res_x               = 0;
    int res_y               = 0;
    hr                      = p_frame_description->get_Width( &res_x );
    hr                      = p_frame_description->get_Width( &res_y );
    frame.mode.resolution_x = res_x;
    frame.mode.resolution_y = res_y;
    frame.width             = res_x;
    frame.height            = res_y;

    pix.allocate( res_x, res_y, 4 );
  }
  safe_release( p_frame_description );
  safe_release( p_source );

  if( FAILED( hr ) )
  {
    ofLogWarning( "ofxKinect2::ColorStream" ) << "Can't open stream.";
    return false;
  }

  return Stream::open();
}

// ColorStream::close
//----------------------------------------------------------
void ColorStream::close()
{
  Stream::close();
  safe_release( stream.p_color_frame_reader );
}

// ColorStream::getColorAt
//----------------------------------------------------------
ofColor ColorStream::getColorAt( int _x, int _y )
{
  if( !pix.getFrontBuffer().isAllocated() || _x < 0 || _y < 0 || _x >= pix.getFrontBuffer().getWidth() || _y >= pix.getFrontBuffer().getHeight() )
  {
    return ofColor( 0, 0, 0, 0 );
  }

  int     index = ( _x + _y * pix.getFrontBuffer().getWidth() ) * 4;
  ofColor color = ofColor( pix.getFrontBuffer()[ index ], pix.getFrontBuffer()[ index + 1 ], pix.getFrontBuffer()[ index + 2 ], pix.getFrontBuffer()[ index + 3 ] );

  return color;
}

// ColorStream::getColorAt
//----------------------------------------------------------
ofColor ColorStream::getColorAt( ofVec2f _color_point )
{
  return getColorAt( _color_point.x, _color_point.y );
}

ofFloatColor ColorStream::getFloatColorAt( int _x, int _y )
{
  if( !pix.getFrontBuffer().isAllocated() || _x < 0 || _y < 0 || _x >= pix.getFrontBuffer().getWidth() || _y >= pix.getFrontBuffer().getHeight() )
  {
    return ofFloatColor( 0, 0, 0, 0 );
  }

  int          index = ( _x + _y * pix.getFrontBuffer().getWidth() ) * 4;
  ofFloatColor color = ofColor( pix.getFrontBuffer()[ index ] / 255.f, pix.getFrontBuffer()[ index + 1 ] / 255.f, pix.getFrontBuffer()[ index + 2 ] / 255.f, pix.getFrontBuffer()[ index + 3 ] / 255.f );

  return color;
}

// ColorStream::getFloatColorAt
//----------------------------------------------------------
ofFloatColor ColorStream::getFloatColorAt( ofVec2f _color_point )
{
  return getFloatColorAt( _color_point.x, _color_point.y );
}

// ColorStream::getExposureTime
//----------------------------------------------------------
int ColorStream::getExposureTime()
{
  TIMESPAN exposure_time;
  camera_settings.p_color_camera_settings->get_ExposureTime( &exposure_time );
  return ( int )exposure_time;
}

// ColorStream::getFrameInterval
//----------------------------------------------------------
int ColorStream::getFrameInterval()
{
  TIMESPAN frame_interval;
  camera_settings.p_color_camera_settings->get_FrameInterval( &frame_interval );
  return ( int )frame_interval;
}

// ColorStream::getGain
//----------------------------------------------------------
float ColorStream::getGain()
{
  float gain;
  camera_settings.p_color_camera_settings->get_Gain( &gain );
  return gain;
}

// ColorStream::getGamma
//----------------------------------------------------------
float ColorStream::getGamma()
{
  float gamma;
  camera_settings.p_color_camera_settings->get_Gamma( &gamma );
  return gamma;
}





// DepthStream
//----------------------------------------------------------
bool DepthStream::readFrame()
{
  bool readed = false;
  if( !stream.p_depth_frame_reader )
  {
    ofLogWarning( "ofxKinect2::DepthStream" ) << "Stream is not open.";
    return readed;
  }
  Stream::readFrame();

  IDepthFrame* p_frame = nullptr;
  HRESULT      hr      = stream.p_depth_frame_reader->AcquireLatestFrame( &p_frame );

  if( SUCCEEDED( hr ) )
  {
    IFrameDescription* p_frame_description = nullptr;

    hr = p_frame->get_RelativeTime( ( INT64* )&frame.timestamp );

    if( SUCCEEDED( hr ) )
    {
      hr = p_frame->get_FrameDescription( &p_frame_description );
    }

    if( SUCCEEDED( hr ) )
    {
      hr = p_frame_description->get_Width( &frame.width );
    }

    if( SUCCEEDED( hr ) )
    {
      hr = p_frame_description->get_Height( &frame.height );
    }

    if( SUCCEEDED( hr ) )
    {
      hr = p_frame_description->get_HorizontalFieldOfView( &frame.horizontal_field_of_view );
    }

    if( SUCCEEDED( hr ) )
    {
      hr = p_frame_description->get_VerticalFieldOfView( &frame.vertical_field_of_view );
    }

    if( SUCCEEDED( hr ) )
    {
      hr = p_frame_description->get_DiagonalFieldOfView( &frame.diagonal_field_of_view );
    }

    if( SUCCEEDED( hr ) )
    {
      hr = p_frame->get_DepthMinReliableDistance( ( USHORT* )&near_value );
    }

    if( SUCCEEDED( hr ) )
    {
      hr = p_frame->get_DepthMaxReliableDistance( ( USHORT* )&far_value );
    }

    if( SUCCEEDED( hr ) )
    {
      hr = p_frame->get_DepthMinReliableDistance( ( USHORT* )&near_value );
    }

    if( SUCCEEDED( hr ) )
    {
      hr = p_frame->AccessUnderlyingBuffer( ( UINT* )&frame.data_size, reinterpret_cast< UINT16** >( &frame.data ) );
    }

    if( SUCCEEDED( hr ) )
    {
      readed = true;
      setPixels( frame );
    }

    safe_release( p_frame_description );
  }

  safe_release( p_frame );

  return readed;
}

// DepthStream::setPixels
//----------------------------------------------------------
void DepthStream::setPixels( Frame _frame )
{
  Stream::updateTimestamp( _frame );
  
  const unsigned short *pixels = ( const unsigned short*)_frame.data;
  if( !pixels ) return;

  int w = _frame.width;
  int h = _frame.height;
  
  pix.allocate( w, h, 1 );
  pix.getBackBuffer().setFromPixels( pixels, w, h, OF_IMAGE_GRAYSCALE );
  pix.swap();
}

// DepthStream::update
//----------------------------------------------------------
void DepthStream::update()
{
  if( !tex.isAllocated() )
  {
    tex.allocate( getWidth(), getHeight(), GL_RGBA, true, GL_LUMINANCE, GL_UNSIGNED_SHORT );
  }

  if( lock() )
  {
    ofShortPixels _pix;
    depthRemapToRange( pix.getFrontBuffer(), _pix, near_value, far_value, is_invert );
    tex.loadData( _pix );
    Stream::update();

    unlock();
  }
}

// DepthStream::getPixels
//----------------------------------------------------------
ofShortPixels& DepthStream::getPixels( int _near, int _far, bool _invert )
{
  ofShortPixels _pix;
  depthRemapToRange( getPixels(), _pix, _near, _far, _invert );
  return _pix;
}

// DepthStream::getPixels
//----------------------------------------------------------
const ofShortPixels& DepthStream::getPixels( int _near, int _far, bool _invert ) const
{
  ofShortPixels _pix;
  depthRemapToRange( getPixels(), _pix, _near, _far, _invert );
  return _pix;
}

// DepthStream::getDepthAt
//----------------------------------------------------------
unsigned short DepthStream::getDepthAt( int _x, int _y )
{
  int index = ( _y * pix.getFrontBuffer().getWidth() ) + _x;

  if( pix.getFrontBuffer().isAllocated() )
  {
    return pix.getFrontBuffer()[ index];
  }
  else
  {
    ofLogNotice( "ofKinect2::DepthStream" ) << "Cannot get depth.";
    return 0;
  }
}

// DepthStream::getDepthAt
//----------------------------------------------------------
unsigned short DepthStream::getDepthAt( ofVec2f _depth_point )
{
  return getDepthAt( _depth_point.x, _depth_point.y );
}

// DepthStream::open
//----------------------------------------------------------
bool DepthStream::open()
{
  if( !device->isOpen() )
  {
    ofLogWarning( "ofxKinect2::DepthStream" ) << "No ready Kinect2 found.";
    return false;
  }

  is_invert  = true;
  near_value = 0;
  far_value  = 10000;

  IDepthFrameSource* p_source = nullptr;
  HRESULT            hr       = device->get().kinect2->get_DepthFrameSource( &p_source );

  if( SUCCEEDED( hr ) )
  {
    hr = p_source->OpenReader( &stream.p_depth_frame_reader );
  }

  safe_release( p_source );
  if( FAILED( hr ) )
  {
    ofLogWarning( "ofxKinect2::DepthStream" ) << "Can't open stream.";
    return false;
  }

  return Stream::open();
}

// DepthStream::close
//----------------------------------------------------------
void DepthStream::close()
{
  Stream::close();
  safe_release( stream.p_depth_frame_reader );
}






// IrStream::readFrame
//----------------------------------------------------------
bool IrStream::readFrame()
{
  bool readed = false;
  if( !stream.p_infrared_frame_reader )
  {
    ofLogWarning( "ofxKinect2::IrStream" ) << "Stream is not open.";
    return readed;
  }
  Stream::readFrame();

  IInfraredFrame* p_frame = nullptr;
  HRESULT         hr      = stream.p_infrared_frame_reader->AcquireLatestFrame( &p_frame );

  if( SUCCEEDED( hr ) )
  {
    IFrameDescription* p_frame_description = nullptr;

    hr = p_frame->get_RelativeTime( ( INT64* )&frame.timestamp );

    if( SUCCEEDED( hr ) )
    {
      hr = p_frame->get_FrameDescription( &p_frame_description );
    }

    if( SUCCEEDED( hr ) )
    {
      hr = p_frame_description->get_Width( &frame.width );
    }

    if( SUCCEEDED( hr ) )
    {
      hr = p_frame_description->get_Height( &frame.height );
    }

    if( SUCCEEDED( hr ) )
    {
      hr = p_frame_description->get_HorizontalFieldOfView( &frame.horizontal_field_of_view );
    }

    if( SUCCEEDED( hr ) )
    {
      hr = p_frame_description->get_VerticalFieldOfView( &frame.vertical_field_of_view );
    }

    if( SUCCEEDED( hr ) )
    {
      hr = p_frame_description->get_DiagonalFieldOfView( &frame.diagonal_field_of_view );
    }

    if( SUCCEEDED( hr ) )
    {
      hr = p_frame->AccessUnderlyingBuffer( ( UINT* )&frame.data_size, reinterpret_cast< UINT16** >( &frame.data ) );
    }

    if( SUCCEEDED( hr ) )
    {
      readed = true;
      setPixels( frame );
    }

    safe_release( p_frame_description );
  }

  safe_release( p_frame );

  return readed;
}

// IrStream::setPixels
//----------------------------------------------------------
void IrStream::setPixels( Frame _frame )
{
  Stream::updateTimestamp( _frame );

  const unsigned short *pixels = ( const unsigned short* )_frame.data;
  if( !pixels ) return;

  int w = _frame.width;
  int h = _frame.height;
  
  pix.allocate( w, h, 1 );
  pix.getBackBuffer().setFromPixels( pixels, w, h, OF_IMAGE_GRAYSCALE );
  pix.swap();
}

// IrStream::update
//----------------------------------------------------------
void IrStream::update()
{
  if( !tex.isAllocated() )
  {
    tex.allocate( getWidth(), getHeight(), GL_LUMINANCE );
  }

  if( lock() )
  {
    tex.loadData( pix.getFrontBuffer() );
    Stream::update();
    unlock();
  }
}

// IrStream::open
//----------------------------------------------------------
bool IrStream::open()
{
  if( !device->isOpen() )
  {
    ofLogWarning( "ofxKinect2::IrStream" ) << "No ready Kinect2 found.";
    return false;
  }

  IInfraredFrameSource* p_source = nullptr;
  HRESULT               hr       = device->get().kinect2->get_InfraredFrameSource( &p_source );

  if( SUCCEEDED( hr ) )
  {
    hr = p_source->OpenReader( &stream.p_infrared_frame_reader );
  }

  safe_release( p_source );
  if( FAILED( hr ) )
  {
    ofLogWarning( "ofxKinect2::IrStream" ) << "Can't open stream.";
    return false;
  }

  return Stream::open();
}

// IrStream::close
//----------------------------------------------------------
void IrStream::close()
{
  Stream::close();
  safe_release( stream.p_infrared_frame_reader );
}







// BodyIndexStream::readFrame
//----------------------------------------------------------
bool BodyIndexStream::readFrame()
{
  bool readed = false;
  if( !stream.p_body_index_frame_reader )
  {
    ofLogWarning( "ofxKinect2::BodyIndexStream" ) << "Stream is not open.";
    return readed;
  }
  Stream::readFrame();

  IBodyIndexFrame* p_frame = nullptr;
  HRESULT          hr      = stream.p_body_index_frame_reader->AcquireLatestFrame( &p_frame );

  if( SUCCEEDED( hr ) )
  {
    IFrameDescription* p_frame_description = nullptr;

    hr = p_frame->get_RelativeTime( ( INT64* )&frame.timestamp );

    if( SUCCEEDED( hr ) )
    {
      hr = p_frame->get_FrameDescription( &p_frame_description );
    }

    if( SUCCEEDED( hr ) )
    {
      hr = p_frame_description->get_Width( &frame.width );
    }

    if( SUCCEEDED( hr ) )
    {
      hr = p_frame_description->get_Height( &frame.height );
    }

    if( SUCCEEDED( hr ) )
    {
      hr = p_frame_description->get_HorizontalFieldOfView( &frame.horizontal_field_of_view );
    }

    if( SUCCEEDED( hr ) )
    {
      hr = p_frame_description->get_VerticalFieldOfView( &frame.vertical_field_of_view );
    }

    if( SUCCEEDED( hr ) )
    {
      hr = p_frame_description->get_DiagonalFieldOfView( &frame.diagonal_field_of_view );
    }

    unsigned int bpp = 0;
    if( SUCCEEDED( hr ) )
    {
      hr = p_frame_description->get_BytesPerPixel( &bpp );
    }

    if( SUCCEEDED( hr ) )
    {
      hr = p_frame->AccessUnderlyingBuffer( ( UINT* )&frame.data_size, reinterpret_cast< BYTE** >( &frame.data ) );
    }

    if( SUCCEEDED( hr ) )
    {
      readed = true;
      setPixels( frame );
    }
    safe_release( p_frame_description );
  }

  safe_release( p_frame );

  return readed;
}

// BodyIndexStream::setPixels
//----------------------------------------------------------
void BodyIndexStream::setPixels( Frame _frame )
{
  Stream::updateTimestamp( _frame );

  int            w      = _frame.width;
  int            h      = _frame.height;
  unsigned char* pixels = new unsigned char[ w * h * 4 ];

  //for( int i  = 0; i < w * h; ++i )
  //{
  //  int           index = i * 4;
  //  unsigned char p     = ( ( unsigned char* )_frame.data )[ i ];
  //  ofColor       color = colors[ p ];
  //  
  //  if( p != 255 )
  //  {
  //    pixels[ index + 0 ] = color.r;
  //    pixels[ index + 1 ] = color.g;
  //    pixels[ index + 2 ] = color.b;
  //    pixels[ index + 3 ] = 255;
  //  }
  //  else
  //  {
  //    pixels[ index + 0 ] = 0;
  //    pixels[ index + 1 ] = 0;
  //    pixels[ index + 2 ] = 0;
  //    pixels[ index + 3 ] = 0;
  //  }
  //}

  ofPixels indexPix;
  indexPix.setFromPixels( ( unsigned char* )_frame.data, w, h, OF_IMAGE_GRAYSCALE );

  int i = 0;
  for( auto& p : indexPix )
  {
    int     index = i * 4;
    ofColor color = colors[ p ];

    if( p != 255 )
    {
      pixels[ index + 0 ] = color.r;
      pixels[ index + 1 ] = color.g;
      pixels[ index + 2 ] = color.b;
      pixels[ index + 3 ] = 255;
    }
    else
    {
      pixels[ index + 0 ] = 0;
      pixels[ index + 1 ] = 0;
      pixels[ index + 2 ] = 0;
      pixels[ index + 3 ] = 0;
    }
    ++i;
  }
  
  pix.allocate( w, h, 4 );
  pix.getBackBuffer().setFromPixels( pixels, w, h, OF_IMAGE_COLOR_ALPHA );
  pix.swap();

  delete[] pixels;
}

// BodyIndexStream::update
//----------------------------------------------------------
void BodyIndexStream::update()
{
  if( !tex.isAllocated() )
  {
    tex.allocate( getWidth(), getHeight(), GL_RGB );
  }

  if( lock() )
  {
    tex.loadData( pix.getFrontBuffer() );
    Stream::update();
    unlock();
  }
}

// BodyIndexStream::open
//----------------------------------------------------------
bool BodyIndexStream::open()
{
  if( !device->isOpen() )
  {
    ofLogWarning( "ofxKinect2::BodyIndexStream" ) << "No ready Kinect2 found.";
    return false;
  }

  IBodyIndexFrameSource* p_source = nullptr;
  HRESULT                hr       = device->get().kinect2->get_BodyIndexFrameSource( &p_source );

  if( SUCCEEDED( hr ) )
  {
    hr = p_source->OpenReader( &stream.p_body_index_frame_reader );
  }

  safe_release( p_source );
  if( FAILED( hr ) )
  {
    ofLogWarning( "ofxKinect2::BodyIndexStream" ) << "Can't open stream.";
    return false;
  }

  return Stream::open();
}

// BodyIndexStream::close
//----------------------------------------------------------
void BodyIndexStream::close()
{
  Stream::close();
  safe_release( stream.p_infrared_frame_reader );
}






// Body::update
//----------------------------------------------------------
void Body::update( IBody* _body )
{
  joints.resize( JointType_Count );
  joint_points.resize( JointType_Count );

  HRESULT hr = _body->get_HandLeftState( &left_hand_state );
  hr         = _body->get_HandRightState( &right_hand_state );

  _body->get_LeanTrackingState( &lean_state );

  PointF pnt;
  _body->get_Lean( &pnt );
  
  CameraSpacePoint cpnt;
  cpnt.X = pnt.X;
  cpnt.Y = pnt.Y;
  cpnt.Z = 0;
  body_lean = bodyPointToScreen( cpnt );

  hr = _body->GetJoints( JointType_Count, &joints.front() );

  if( SUCCEEDED( hr ) )
  {
    is_update_scale = false;
  }
}

// Body::jointToScreen
//----------------------------------------------------------
ofPoint Body::jointToScreen( const JointType _jointType )
{
  CameraSpacePoint bodyPoint = joints[ _jointType ].Position;
  return bodyPointToScreen( bodyPoint );
}

// Body::bodyPointToScreen
//----------------------------------------------------------
ofPoint Body::bodyPointToScreen( const CameraSpacePoint& _bodyPoint )
{
  // Calculate the body's position on the screen
  ColorSpacePoint    colorPoint = { 0 };
  ICoordinateMapper* mapper     = nullptr;
  HRESULT            hr         = device->get().kinect2->get_CoordinateMapper( &mapper );
  if( SUCCEEDED( hr ) )
  {
    mapper->MapCameraPointToColorSpace( _bodyPoint, &colorPoint );
  }
  else
  {
    ofLogError( "ofxKinect2::Body" ) << "can't get Coordinate Mapper.";
    return ofPoint( 0, 0 );
  }

  // TODO: width/ height
  float screenPointX = static_cast< float >( colorPoint.X );
  float screenPointY = static_cast< float >( colorPoint.Y );

  return ofPoint( screenPointX, screenPointY );
}

// Body::drawBody
//----------------------------------------------------------
void Body::drawBody()
{
  if( !is_tracked ) return;

  drawBone( JointType_Head,          JointType_Neck );
  drawBone( JointType_Neck,          JointType_SpineShoulder );
  drawBone( JointType_SpineShoulder, JointType_SpineMid );
  drawBone( JointType_SpineMid,      JointType_SpineBase );
  drawBone( JointType_SpineShoulder, JointType_ShoulderLeft );
  drawBone( JointType_SpineShoulder, JointType_ShoulderRight );
  drawBone( JointType_SpineBase,     JointType_HipLeft );
  drawBone( JointType_SpineBase,     JointType_HipRight );

  drawBone( JointType_ShoulderLeft,  JointType_ElbowLeft );
  drawBone( JointType_ElbowLeft,     JointType_WristLeft );
  drawBone( JointType_WristLeft,     JointType_HandLeft );
  drawBone( JointType_HandLeft,      JointType_HandTipLeft );
  drawBone( JointType_WristLeft,     JointType_ThumbLeft );

  drawBone( JointType_ShoulderRight, JointType_ElbowRight );
  drawBone( JointType_ElbowRight,    JointType_WristRight );
  drawBone( JointType_WristRight,    JointType_HandRight );
  drawBone( JointType_HandRight,     JointType_HandTipRight );
  drawBone( JointType_WristRight,    JointType_ThumbRight );

  drawBone( JointType_HipLeft,       JointType_KneeLeft );
  drawBone( JointType_KneeLeft,      JointType_AnkleLeft );
  drawBone( JointType_AnkleLeft,     JointType_FootLeft );

  drawBone( JointType_HipRight,      JointType_KneeRight );
  drawBone( JointType_KneeRight,     JointType_AnkleRight );
  drawBone( JointType_AnkleRight,    JointType_FootRight );

  ofPushStyle();
  {
    int i = 0;
    for( auto& j : joints )
    {
      if( j.TrackingState == TrackingState_Tracked )
      {
        ofSetColor( 50, 200, 50 );
        ofEllipse( jointToScreen( ( JointType )i ), 5, 5 );
      }
      ++i;
    }
    ofSetColor( ofColor::red );
    ofDrawBitmapString( ofToString( id ), jointToScreen( JointType_Head ) );
  }
  ofPopStyle();
}

// Body::drawBone
//----------------------------------------------------------
void Body::drawBone( JointType _joint0, JointType _joint1 )
{
  if( !is_tracked ) return;

  TrackingState state0 = joints[ _joint0 ].TrackingState;
  TrackingState state1 = joints[ _joint1 ].TrackingState;

  if( ( state0 == TrackingState_NotTracked ) || ( state1 == TrackingState_NotTracked ) ) return;
  if( ( state0 == TrackingState_Inferred )   && ( state1 == TrackingState_Inferred ) )   return;

  ofPushStyle();
  {
    if( ( state0 == TrackingState_Tracked ) && ( state1 == TrackingState_Tracked ) )
    {
      ofSetColor( ofColor::green );
    }
    else
    {
      ofSetColor( ofColor::gray );
    }
    ofLine( jointToScreen( _joint0 ), jointToScreen( _joint1 ) );
  }
  ofPopStyle();
}

// Body::drawHands
//----------------------------------------------------------
void Body::drawHands()
{
  drawHandLeft();
  drawHandRight();
}

// Body::drawHandLeft
//----------------------------------------------------------
void Body::drawHandLeft()
{
  if( !is_tracked ) return;

  ofPushStyle();
  {
    switch( left_hand_state )
    {
    case HandState_Closed:
      ofSetColor( ofColor::red );
      break;

    case HandState_Open:
      ofSetColor( ofColor::green );
      break;

    case HandState_Lasso:
      ofSetColor( ofColor::blue );
      break;
    }
    ofEllipse( jointToScreen( JointType_HandLeft ), 30, 30 );
  }
  ofPopStyle();
}

// Body::drawHandRight
//----------------------------------------------------------
void Body::drawHandRight()
{
  if( !is_tracked ) return;

  ofPushStyle();
  {
    switch( right_hand_state )
    {
    case HandState_Closed:
      ofSetColor( ofColor::red );
      break;

    case HandState_Open:
      ofSetColor( ofColor::green );
      break;

    case HandState_Lasso:
      ofSetColor( ofColor::blue );
      break;
    }
    ofEllipse( jointToScreen( JointType_HandRight ), 30, 30 );
  }
  ofPopStyle();
}

// Body::drawLean
//----------------------------------------------------------
void Body::drawLean()
{
  if( !is_tracked ) return;

  if( lean_state == TrackingState_Tracked )
  {
    ofPushStyle();
    {
      ofSetColor( ofColor::magenta );
      ofLine( jointToScreen( JointType_SpineBase ), jointToScreen( JointType_SpineBase ) + body_lean );
    }
    ofPopStyle();
  }
}






// BodyStream::readFrame
//----------------------------------------------------------
bool BodyStream::readFrame()
{
  bool readed = false;
  if( !stream.p_body_frame_reader )
  {
    ofLogWarning( "ofxKinect2::BodyStream" ) << "Stream is not open.";
    return readed;
  }

  IBodyFrame* p_frame = nullptr;
  HRESULT     hr      = stream.p_body_frame_reader->AcquireLatestFrame( &p_frame );

  if( SUCCEEDED( hr ) )
  {
    hr = p_frame->get_RelativeTime( ( INT64* )&frame.timestamp );

    IBody* ppBodies[ BODY_COUNT ] = { 0 };

    if( SUCCEEDED( hr ) )
    {
      hr = p_frame->GetAndRefreshBodyData( _countof( ppBodies ), ppBodies );
    }

    if( SUCCEEDED( hr ) )
    {
      int i = 0;
      for( auto b : ppBodies )
      {
        BOOLEAN tracked = false;

        if( b )
        {
          b->get_IsTracked( &tracked );

          UINT64 id = -1;
          b->get_TrackingId( &id );
          
          bodies[ i ].setTracked( ( bool )tracked );
          bodies[ i ].setId( id );
          
          if( tracked ) bodies[ i ].update( b );
        }

        ++i;
      }

      for( auto b : ppBodies ) safe_release( b );

      readed = true;
      setPixels( frame );
    }
  }

  safe_release( p_frame );

  return readed;
}

// BodyStream::draw
//----------------------------------------------------------
void BodyStream::draw()
{
  for( auto& b : bodies )
  {
    b.drawBody();
    b.drawHands();
    b.drawLean();
  }
}

// BodyStream::drawBody
//----------------------------------------------------------
void BodyStream::drawBody()
{
  for( auto& b : bodies ) b.drawBody();
}

// BodyStream::drawBone
//----------------------------------------------------------
void BodyStream::drawBone( JointType _joint0, JointType _joint1 )
{
  for( auto& b : bodies ) b.drawBone( _joint0, _joint1 );
}

// BodyStream::drawHands
//----------------------------------------------------------
void BodyStream::drawHands()
{
  for( auto& b : bodies ) b.drawHands();
}

// BodyStream::drawHandLeft
//----------------------------------------------------------
void BodyStream::drawHandLeft()
{
  for( auto& b : bodies ) b.drawHandLeft();
}

// BodyStream::drawHandRight
//----------------------------------------------------------
void BodyStream::drawHandRight()
{
  for( auto& b : bodies ) b.drawHandRight();
}

// BodyStream::drawLean
//----------------------------------------------------------
void BodyStream::drawLean()
{
  for( auto& b : bodies ) b.drawLean();
}

// BodyStream::setPixels
//----------------------------------------------------------
void BodyStream::setPixels( Frame _frame )
{
  Stream::updateTimestamp( _frame );
}

// BodyStream::update
//----------------------------------------------------------
void BodyStream::update()
{
  if( lock() )
  {
    Stream::update();
    unlock();
  }
}

// BodyStream::open
//----------------------------------------------------------
bool BodyStream::open()
{
  if( !device->isOpen() )
  {
    ofLogWarning( "ofxKinect2::BodyStream" ) << "No ready Kinect2 found.";
    return false;
  }

  IBodyFrameSource* p_source = nullptr;
  HRESULT           hr       = device->get().kinect2->get_BodyFrameSource( &p_source );

  if( SUCCEEDED( hr ) )
  {
    hr = p_source->OpenReader( &stream.p_body_frame_reader );
  }

  safe_release( p_source );
  if( FAILED( hr ) )
  {
    ofLogWarning( "ofxKinect2::BodyStream" ) << "Can't open stream.";
    return false;
  }

  return Stream::open();
}

// BodyStream::close
//----------------------------------------------------------
void BodyStream::close()
{
  Stream::close();
  safe_release( stream.p_body_frame_reader );
}

// BodyStream::
//----------------------------------------------------------
ofShortPixels& BodyStream::getPixels( int _near, int _far, bool _invert )
{
  ofShortPixels _pix;
  depthRemapToRange( getPixels(), _pix, _near, _far, _invert );
  return _pix;
}

//----------------------------------------------------------
const ofShortPixels& BodyStream::getPixels( int _near, int _far, bool _invert ) const
{
  ofShortPixels _pix;
  depthRemapToRange( getPixels(), _pix, _near, _far, _invert );
  return _pix;
}






// Mapper::setup
//----------------------------------------------------------
bool Mapper::setup( Device& _device )
{
  device     = &_device;
  HRESULT hr = _device.get().kinect2->get_CoordinateMapper( &p_mapper );

  if( SUCCEEDED( hr ) )
  {
    return true;
  }
  else
  {
    ofLogWarning( "ofxKinect2::Mapper" ) << "Cannot get Coordinate Mapper.";
    return false;
  }
}

// Mapper::isReady
//----------------------------------------------------------
bool Mapper::isReady( bool _depth, bool _color )
{
  if( _depth )
  {
    if( !depth_pixels || !depth_pixels->isAllocated() ) return false;
  }

  if( _color )
  {
    if( !color_pixels || !color_pixels->isAllocated() ) return false;
  }

  return true;
}

// Mapper::mapDepthToCameraSpace
//----------------------------------------------------------
ofVec3f Mapper::mapDepthToCameraSpace( int _x, int _y )
{
  if( !isReady( true, false ) ) return ofVec3f();

  DepthSpacePoint depthPoint = { _x, _y };
  int             index      = ( _y * depth_pixels->getWidth() ) + _x;
  UINT16          depth      = depth_pixels->getData()[ index ];

  if( depth_to_camera_points.size() == 0 ) depth_to_camera_points.resize( depth_pixels->size() );

  p_mapper->MapDepthPointToCameraSpace( depthPoint, depth, reinterpret_cast< CameraSpacePoint* >( depth_to_camera_points.data() ) );
  return depth_to_camera_points[ 0 ];
}

// Mapper::mapDepthToCameraSpace
//----------------------------------------------------------
ofVec3f Mapper::mapDepthToCameraSpace( ofVec2f _depth_point )
{
  return mapDepthToCameraSpace( _depth_point.x, _depth_point.y );
}

// Mapper::mapDepthToCameraSpace
//----------------------------------------------------------
vector< ofVec3f > Mapper::mapDepthToCameraSpace()
{
  if( !isReady( true, false ) ) return vector< ofVec3f >();

  UINT depth_size = depth_pixels->size();

  if( depth_to_camera_points.size() != depth_size ) depth_to_camera_points.resize( depth_size );

  p_mapper->MapDepthFrameToCameraSpace( depth_size, depth_pixels->getData(), depth_size, reinterpret_cast< CameraSpacePoint* >( depth_to_camera_points.data() ) );
  return depth_to_camera_points;
}

// Mapper::mapDepthToCameraSpace
//----------------------------------------------------------
vector< ofVec3f > Mapper::mapDepthToCameraSpace( vector< ofVec2f > _depth_points )
{
  if( !isReady( true, false ) ) return vector< ofVec3f >();

  UINT depth_size = _depth_points.size();
  if( !depth_space_points ) depth_space_points = new DepthSpacePoint[ depth_pixels->size() ];
  if( !depth_values )       depth_values       = new UINT16[ depth_pixels->size() ];

  for( int i = 0; i < depth_size; ++i )
  {
    depth_space_points[ i ] = { _depth_points[ i ].x, _depth_points[ i ].y };
    int index               = ( ( int )_depth_points[ i ].y * depth_pixels->getWidth() ) + ( int )_depth_points[ i ].x;
    depth_values[ i ]       = depth_pixels->getData()[ index ];
  }

  if( depth_to_camera_points.size() != depth_size ) depth_to_camera_points.resize( depth_size );

  p_mapper->MapDepthPointsToCameraSpace( depth_size, depth_space_points, depth_size, depth_values, depth_size, reinterpret_cast< CameraSpacePoint* >( depth_to_camera_points.data() ) );
  return depth_to_camera_points;
}

//----------------------------------------------------------
vector< ofVec3f > Mapper::mapDepthToCameraSpace( ofRectangle _depth_area )
{
  if( !isReady( true, false ) ) return vector< ofVec3f >();

  UINT depth_size = _depth_area.getWidth() * _depth_area.getHeight();

  if( !depth_space_points ) depth_space_points = new DepthSpacePoint[ depth_pixels->size() ];  
  if( !depth_values )       depth_values       = new UINT16[ depth_pixels->size() ];

  const unsigned short* data    = depth_pixels->getData();
  int                   width   = _depth_area.getWidth();
  int                   d_width = depth_pixels->getWidth();

  for( int i = 0; i < depth_size; ++i )
  {
    depth_space_points[ i ] = { float( i % width ), ( float )floor( i / width ) };
    int index               = ( ( int )depth_space_points[ i ].Y * d_width ) + ( int )depth_space_points[ i ].X;
    depth_values[ i ]       = data[ index ];
  }

  if( depth_to_camera_points.size() != depth_size ) depth_to_camera_points.resize( depth_size );

  p_mapper->MapDepthPointsToCameraSpace( depth_size, depth_space_points, depth_size, depth_values, depth_size, reinterpret_cast< CameraSpacePoint* >( depth_to_camera_points.data() ) );
  return depth_to_camera_points;
}

// mapDepthToColorSpace
//----------------------------------------------------------
ofVec2f Mapper::mapDepthToColorSpace( int _x, int _y )
{
  if( !isReady( true, false ) ) return ofVec2f();

  DepthSpacePoint depthPoint;
  depthPoint.X = _x;
  depthPoint.Y = _y;
  int    index = _x + _y * depth_pixels->getWidth();
  UINT16 depth = depth_pixels->getData()[ index ];

  if( depth_to_color_points.size() == 0 ) depth_to_color_points.resize( depth_pixels->size() );

  p_mapper->MapDepthPointToColorSpace( depthPoint, depth, reinterpret_cast< ColorSpacePoint* >( depth_to_color_points.data() ) );
  return depth_to_color_points[ 0 ];
}

//----------------------------------------------------------
ofVec2f Mapper::mapDepthToColorSpace( ofVec2f _depth_point )
{
  return mapDepthToColorSpace( _depth_point.x, _depth_point.y );
}

//----------------------------------------------------------
vector< ofVec2f > Mapper::mapDepthToColorSpace()
{
  if( !isReady( true, false ) ) return vector< ofVec2f >();

  UINT depth_size = depth_pixels->size();
  if( depth_to_color_points.size() != depth_size ) depth_to_color_points.resize( depth_size );

  p_mapper->MapDepthFrameToColorSpace( depth_size, depth_pixels->getData(), depth_size, reinterpret_cast< ColorSpacePoint* >( depth_to_color_points.data() ) );
  return depth_to_color_points;
}

//----------------------------------------------------------
vector< ofVec2f > Mapper::mapDepthToColorSpace( vector< ofVec2f > _depth_points )
{
  if( !isReady( true, false ) ) return vector< ofVec2f >();

  UINT depth_size = _depth_points.size();
  if( !depth_space_points ) depth_space_points = new DepthSpacePoint[ depth_pixels->size() ];
  if( !depth_values )       depth_values       = new UINT16[ depth_pixels->size() ];
 
  int                   d_width = depth_pixels->getWidth();
  const unsigned short* data    = depth_pixels->getData();
  for( int i = 0; i < depth_size; ++i )
  {
    depth_space_points[ i ] = { _depth_points[ i ].x, _depth_points[ i ].y };
    int index               = ( ( int )depth_space_points[ i ].Y * d_width ) + ( int )depth_space_points[ i ].X;
    depth_values[ i ]       = data[ index ];
  }

  if( depth_to_color_points.size() != depth_size ) depth_to_color_points.resize( depth_size );

  p_mapper->MapDepthPointsToColorSpace( depth_size, depth_space_points, depth_size, depth_values, depth_size, reinterpret_cast< ColorSpacePoint* >( depth_to_color_points.data() ) );
  return depth_to_color_points;
}
//----------------------------------------------------------
vector< ofVec2f > Mapper::mapDepthToColorSpace( ofRectangle depth_area )
{
  if( !isReady( true, false ) ) return vector< ofVec2f >();
  
  UINT depth_size = depth_area.getWidth() * depth_area.getHeight();
  if( !depth_space_points ) depth_space_points = new DepthSpacePoint[ depth_pixels->size() ];
  if( !depth_values )       depth_values       = new UINT16[ depth_pixels->size() ];

  int                   width   = depth_area.getWidth();
  int                   d_width = depth_pixels->getWidth();
  const unsigned short* data    = depth_pixels->getData();

  for( int i = 0; i < depth_size; ++i )
  {
    depth_space_points[ i ].X = i % width;
    depth_space_points[ i ].Y = ( int )floor( i / width );
    int index                 = ( ( int )depth_space_points[ i ].Y * d_width ) + ( int )depth_space_points[ i ].X;
    depth_values[ i ]         = data[ index ];
  }

  if( depth_to_color_points.size() != depth_size ) depth_to_color_points.resize( depth_size );

  p_mapper->MapDepthPointsToColorSpace( depth_size, depth_space_points, depth_size, depth_values, depth_size, reinterpret_cast< ColorSpacePoint* >( depth_to_color_points.data() ) );
  return depth_to_color_points;
}

//  Mapper::mapColorToCameraSpace
//----------------------------------------------------------
vector< ofVec3f > Mapper::mapColorToCameraSpace()
{
  if( !isReady( true, true ) ) return vector< ofVec3f >();

  int depth_size = depth_pixels->size();
  int color_size = color_pixels->size();
  if( color_to_camera_points.size() != color_size ) color_to_camera_points.resize( color_size );
  
  p_mapper->MapColorFrameToCameraSpace( depth_size, depth_pixels->getData(), color_size, reinterpret_cast< CameraSpacePoint* >( color_to_camera_points.data() ) );
  return color_to_camera_points;
}

// Mapper::mapColorToDepthSpace
//----------------------------------------------------------
vector< ofVec2f > Mapper::mapColorToDepthSpace()
{
  if( !isReady( true, true ) ) return vector< ofVec2f >();

  UINT depth_size = depth_pixels->size();
  UINT color_size = color_pixels->size();
  if( color_to_depth_points.size() != color_size ) color_to_depth_points.resize( color_size );

  p_mapper->MapColorFrameToDepthSpace( depth_size, depth_pixels->getData(), color_size, reinterpret_cast< DepthSpacePoint* >( color_to_depth_points.data() ) );
  return color_to_depth_points;
}

// Mapper::mapCameraToDepthSpace
//----------------------------------------------------------
ofVec2f Mapper::mapCameraToDepthSpace( float _x, float _y, float _z )
{
  if( !isReady( true, false ) ) return ofVec2f();

  CameraSpacePoint camera_space_point = { _x, _y, _z };

  if( camera_to_depth_points.size() == 0 ) camera_to_depth_points.resize( depth_pixels->size() );

  p_mapper->MapCameraPointToDepthSpace( camera_space_point, reinterpret_cast< DepthSpacePoint* >( camera_to_depth_points.data() ) );
  return camera_to_depth_points[ 0 ];
}

//----------------------------------------------------------
ofVec2f Mapper::mapCameraToDepthSpace( ofVec3f _camera_point )
{
  return mapCameraToDepthSpace(  _camera_point.x, _camera_point.y, _camera_point.z );
}

//----------------------------------------------------------
vector< ofVec2f > Mapper::mapCameraToDepthSpace( vector< ofVec3f > _camera_points )
{
  if( !isReady( true, false ) ) return vector< ofVec2f >();

  UINT camera_size = _camera_points.size();

  if( !camera_space_points ) camera_space_points = new CameraSpacePoint[ depth_pixels->size() ];

  for( int i = 0; i < camera_size; ++i )
  {
    camera_space_points[ i ] = { _camera_points[ i ].x, _camera_points[ i ].y, _camera_points[ i ].z };
  }

  if( camera_to_depth_points.size() != camera_size ) camera_to_depth_points.resize( camera_size );

  p_mapper->MapCameraPointsToDepthSpace( camera_size, camera_space_points, camera_size, reinterpret_cast< DepthSpacePoint* >( camera_to_depth_points.data() ) );
  return camera_to_depth_points;
}

// Mapper::mapCameraToColorSpace
//----------------------------------------------------------
ofVec2f Mapper::mapCameraToColorSpace( float _x, float _y, float _z )
{
  //if( !isReady( true, false ) ) return ofVec2f();

  CameraSpacePoint camera_space_point = { _x, _y, _z };

  //if( depth_to_color_points.size() == 0 ) depth_to_color_points.resize( depth_pixels->size() );

  //p_mapper->MapCameraPointToColorSpace( camera_space_point, reinterpret_cast< ColorSpacePoint* >( depth_to_color_points.data() ) );
  //return depth_to_color_points[ 0 ];

  ColorSpacePoint color_space_point = { 0, 0 };
  p_mapper->MapCameraPointToColorSpace( camera_space_point, &color_space_point );

  return ofVec2f( color_space_point.X, color_space_point.Y );
}

//----------------------------------------------------------
ofVec2f Mapper::mapCameraToColorSpace( CameraSpacePoint _camera_point )
{
  return mapCameraToColorSpace( _camera_point.X, _camera_point.Y, _camera_point.Z );
}

//----------------------------------------------------------
ofVec2f Mapper::mapCameraToColorSpace( ofVec3f _camera_point )
{
  return mapCameraToColorSpace( _camera_point.x, _camera_point.y, _camera_point.z );
}

//----------------------------------------------------------
vector< ofVec2f > Mapper::mapCameraToColorSpace( vector< ofVec3f > _camera_points )
{
  if( !isReady( true, false ) ) return vector< ofVec2f >();

  UINT camera_size = _camera_points.size();
  if( !camera_space_points ) camera_space_points = new CameraSpacePoint[ depth_pixels->size() ];

  for( int i = 0; i < camera_size; ++i )
  {
    camera_space_points[ i ] = { _camera_points[ i ].x, _camera_points[ i ].y, _camera_points[ i ].z };
  }

  if( depth_to_color_points.size() != camera_size ) depth_to_color_points.resize( camera_size );

  p_mapper->MapCameraPointsToColorSpace( camera_size, camera_space_points, camera_size, reinterpret_cast< ColorSpacePoint* >( depth_to_color_points.data() ) );
  return depth_to_color_points;
}


// Mapper::getFloatColorsCoordinatesToDepthFrame
//----------------------------------------------------------
vector< ofFloatColor > Mapper::getFloatColorsCoordinatesToDepthFrame()
{
  if( !isReady( true, true ) ) return vector< ofFloatColor >();

  UINT          depth_size = depth_pixels->size();
  const UINT16* depth_pix  = depth_pixels->getData();

  if( depth_to_color_points.size() == 0 )          depth_to_color_points.resize( depth_pixels->size() );
  if( depth_to_float_colors.size() != depth_size ) depth_to_float_colors.resize( depth_size );

  p_mapper->MapDepthFrameToColorSpace( depth_size, depth_pix, depth_size, reinterpret_cast< ColorSpacePoint* >( depth_to_color_points.data() ) );

  int                  col_width  = color_pixels->getWidth();
  int                  col_height = color_pixels->getHeight();
  const unsigned char* data       = color_pixels->getData();
  for( int i = 0; i < depth_size; ++i )
  {
    ofFloatColor &col = depth_to_float_colors[ i ];
    int index = ( ( int )depth_to_color_points[ i ].y * col_width ) + ( int )depth_to_color_points[ i ].x;

    if( depth_to_color_points[ i ].x >= 0 && depth_to_color_points[ i ].x < col_width &&
        depth_to_color_points[ i ].y >= 0 && depth_to_color_points[ i ].y < col_height )
    {
      col.r = data[ index * 4     ] / 255.f;
      col.g = data[ index * 4 + 1 ] / 255.f;
      col.b = data[ index * 4 + 2 ] / 255.f;
      col.a = data[ index * 4 + 3 ] / 255.f;
    }
    else
    {
      col = ofFloatColor( 0, 0, 0, 0 );
    }
  }
  return depth_to_float_colors;
}

// Mapper::getColorsCoordinatesToDepthFrame
//----------------------------------------------------------
vector< ofColor > Mapper::getColorsCoordinatesToDepthFrame()
{
  if( !isReady( true, true ) ) return vector< ofColor >();

  UINT          depth_size = depth_pixels->size();
  const UINT16* depth_pix  = depth_pixels->getData();

  if( depth_to_color_points.size() == 0 )    depth_to_color_points.resize( depth_size );
  if( depth_to_colors.size() != depth_size ) depth_to_colors.resize( depth_size );

  p_mapper->MapDepthFrameToColorSpace( depth_size, depth_pix, depth_size, reinterpret_cast< ColorSpacePoint* >( depth_to_color_points.data() ) );

  int                  col_width  = color_pixels->getWidth();
  int                  col_height = color_pixels->getHeight();
  const unsigned char* data       = color_pixels->getData();
  for( int i = 0; i < depth_size; ++i )
  {
    ofColor& col   = depth_to_colors[ i ];
    int      index = ( ( int )depth_to_color_points[ i ].y * col_width) + ( int )depth_to_color_points[ i ].x;
    if( depth_to_color_points[ i ].x >= 0 && depth_to_color_points[ i ].x < col_width &&
        depth_to_color_points[ i ].y >= 0 && depth_to_color_points[ i ].y < col_height )
    {
      col.r = data[ index * 4     ];
      col.g = data[ index * 4 + 0 ];
      col.b = data[ index * 4 + 2 ];
      col.a = data[ index * 4 + 3 ];
    }
    else
    {
      col = ofColor( 0, 0, 0, 0 );
    }
  }

  return depth_to_colors;
}

// Mapper::getColorFrameCoordinatesToDepthFrame
//----------------------------------------------------------
ofPixels Mapper::getColorFrameCoordinatesToDepthFrame()
{
  if( !isReady( true, true ) ) return ofPixels();

  UINT          depth_size = depth_pixels->size();
  const UINT16* depth_pix  = depth_pixels->getData();

  if( depth_to_color_points.size() == 0 )    depth_to_color_points.resize( depth_size );
  if( depth_to_colors.size() != depth_size ) depth_to_colors.resize( depth_size );

  p_mapper->MapDepthFrameToColorSpace( depth_size, depth_pix, depth_size, reinterpret_cast< ColorSpacePoint* >( depth_to_color_points.data() ) );

  int col_width  = color_pixels->getWidth();
  int col_height = color_pixels->getHeight();

  if( !coordinate_color_pixels.isAllocated() ) coordinate_color_pixels.allocate( depth_pixels->getWidth(), depth_pixels->getHeight(), OF_PIXELS_RGBA );

  const unsigned char* data = color_pixels->getData();
  for( int i = 0; i < depth_size; ++i )
  {
    int index = ( ( int )depth_to_color_points[ i ].y * col_width ) + ( int )depth_to_color_points[ i ].x;
    if( depth_to_color_points[ i ].x >= 0 && depth_to_color_points[ i ].x < col_width &&
        depth_to_color_points[ i ].y >= 0 && depth_to_color_points[ i ].y < col_height )
    {
      coordinate_color_pixels[ i * 4     ] = data[ index * 4 ];
      coordinate_color_pixels[ i * 4 + 0 ] = data[ index * 4 + 0 ];
      coordinate_color_pixels[ i * 4 + 2 ] = data[ index * 4 + 2 ];
      coordinate_color_pixels[ i * 4 + 3 ] = data[ index * 4 + 3 ];
    }
    else
    {
      coordinate_color_pixels[ i * 4     ] = 0;
      coordinate_color_pixels[ i * 4 + 0 ] = 0;
      coordinate_color_pixels[ i * 4 + 2 ] = 0;
      coordinate_color_pixels[ i * 4 + 3 ] = 0;
    }
  }

  return coordinate_color_pixels;
}
