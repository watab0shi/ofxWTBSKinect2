//#pragma comment(linker, "/subsystem:\"windows\" /entry:\"mainCRTStartup\"")

#include "ofMain.h"
#include "ofApp.h"


int main()
{
  //ofLogToFile( "ofLog.txt" );

  ofSetupOpenGL( 1920, 1080, OF_WINDOW );
  ofRunApp( new ofApp() );
}
