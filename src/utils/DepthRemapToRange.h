#pragma once

#include "ofMain.h"

namespace ofxKinect2
{
  inline void depthRemapToRange( const ofShortPixels &_src, ofShortPixels &_dst, int _near, int _far, int _invert )
  {
    _dst.allocate( _src.getWidth(), _src.getHeight(), 1 );
    
    unsigned short* dst_ptr = _dst.getPixels();
    
    if( _invert ) std::swap( _near, _far );
    
    for( auto& p : _src )
    {
      *dst_ptr = ofMap( p, _near, _far, 0, 65535, true );
      ++dst_ptr;
    }
  }
}