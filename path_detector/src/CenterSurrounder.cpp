
#include "path_detector/CenterSurrounder.h"


CenterSurrounder::CenterSurrounder() {};



CenterSurrounder::~CenterSurrounder() {};



void CenterSurrounder::centerSurround(ImgPyr& _src, ImgPyr& _dst, bool _onOff) {


    if ( _dst.countLevels() >= 5 ){

        _centerSurroundCore(_src.getLevel(2), _src.getLevel(5), _dst.getLevel(0), _onOff);
        _centerSurroundCore(_src.getLevel(2), _src.getLevel(6), _dst.getLevel(1), _onOff);
        _centerSurroundCore(_src.getLevel(3), _src.getLevel(6), _dst.getLevel(2), _onOff);
        _centerSurroundCore(_src.getLevel(3), _src.getLevel(7), _dst.getLevel(3), _onOff);
        _centerSurroundCore(_src.getLevel(4), _src.getLevel(7), _dst.getLevel(4), _onOff);
        _centerSurroundCore(_src.getLevel(4), _src.getLevel(8), _dst.getLevel(5), _onOff);

    } else {

        _centerSurroundCore(_src.getLevel(2), _src.getLevel(4), _dst.getLevel(0), _onOff);
        _centerSurroundCore(_src.getLevel(2), _src.getLevel(5), _dst.getLevel(1), _onOff);
        _centerSurroundCore(_src.getLevel(3), _src.getLevel(4), _dst.getLevel(2), _onOff);
        _centerSurroundCore(_src.getLevel(3), _src.getLevel(5), _dst.getLevel(3), _onOff);
    }
}



void CenterSurrounder::centerSurround(ImgPyr& _src1, ImgPyr& _src2, ImgPyr& _dst, bool _onOff)
{
  if (_dst.countLevels() >= 5){

    _centerSurroundCore( _src1.getLevel(2), _src2.getLevel(5), _dst.getLevel(0), _onOff );
    _centerSurroundCore( _src1.getLevel(2), _src2.getLevel(6), _dst.getLevel(1), _onOff );
    _centerSurroundCore( _src1.getLevel(3), _src2.getLevel(6), _dst.getLevel(2), _onOff );
    _centerSurroundCore( _src1.getLevel(3), _src2.getLevel(7), _dst.getLevel(3), _onOff );
    _centerSurroundCore( _src1.getLevel(4), _src2.getLevel(7), _dst.getLevel(4), _onOff );
    _centerSurroundCore( _src1.getLevel(4), _src2.getLevel(8), _dst.getLevel(5), _onOff );
  } else {
    _centerSurroundCore( _src1.getLevel(2), _src2.getLevel(4), _dst.getLevel(0), _onOff );
    _centerSurroundCore( _src1.getLevel(2), _src2.getLevel(5), _dst.getLevel(1), _onOff );
    _centerSurroundCore( _src1.getLevel(3), _src2.getLevel(4), _dst.getLevel(2), _onOff );
    _centerSurroundCore( _src1.getLevel(3), _src2.getLevel(5), _dst.getLevel(3), _onOff );
  }
}



void CenterSurrounder::_centerSurroundCore(const cv::Mat& c, const cv::Mat& s, cv::Mat& _output, bool _onoff) {

    cv::Size size( c.cols, c.rows );

    // image containing surrounding with same scale as center
    cv::Mat surround( size, c.type() );
    cv::Mat outTmp( size, c.type() );

    // up-scaling level _s to match _c
    cv::resize( s, surround, surround.size(), cv::INTER_CUBIC );

    //bitwise subtraction between center and surround
    if ( _onoff )
        cv::subtract( c, surround, outTmp );
    else
        cv::subtract( surround, c, outTmp );

    // resizing output temporary to desired output
    cv::resize( outTmp, _output, _output.size(), cv::INTER_CUBIC );
}
