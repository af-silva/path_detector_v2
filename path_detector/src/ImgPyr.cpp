/*
 * Author: Ricardo Mendonça
 */

#include "path_detector/ImgPyr.h"


ImgPyr::ImgPyr() : isHomogeneous(false) {}



ImgPyr::ImgPyr(const char* _name, const bool _homogeneous) {

    isHomogeneous = _homogeneous;
    name = _name;
}



void ImgPyr::setName( const char* _name ) {

    name = _name; // setting name of the pyramid
}



ImgPyr::ImgPyr(const ImgPyr& orig) {

    name = orig.name;
    nrLevels = orig.nrLevels;
    levels = orig.levels;
}



ImgPyr::~ImgPyr() {

    levels.clear();
}



ImgPyr& ImgPyr::operator =(const ImgPyr& pyr) {

    // Only do assignment if pyr is a different object from "this".
    if (this != &pyr) {
        levels.clear();
        nrLevels = pyr.nrLevels;
        name = pyr.name;
        levels = pyr.levels;
        isHomogeneous = pyr.isHomogeneous;
    }
    return *this;
}



ImgPyr& ImgPyr::operator -=(const ImgPyr& pyr) {

    //if (nrLevels != pyr.nrLevels)
        //ERROR :: Do Something... ver tb homogeneous
    for (int i=0; i < nrLevels; i++)
        cv::subtract(levels[i], pyr.levels[i], levels[i]);

    return *this;
}



const ImgPyr ImgPyr::operator -(const ImgPyr& pyr) {

    return ImgPyr(*this) -= pyr;
}



cv::Mat& ImgPyr::getLevel(const int idx) {

    return (levels.at(idx));
}



int ImgPyr::countLevels() {

    return nrLevels;
}



void ImgPyr::view(){

  for (int i=0; i < nrLevels; i++){
    //char windowName[255];
    std::stringstream ss;
    ss << name << " " << i;
    //sprintf(windowName, "%s %d", name, i);
    cv::namedWindow( ss.str().c_str(), cv::WINDOW_AUTOSIZE );
    cv::imshow( ss.str().c_str(), levels[i] );
  }
}



void ImgPyr::sumLevels(cv::Mat& _output){

    //if (!isHomogeneous)
        //throw exception...
  _output.setTo(cv::Scalar(0));

  for (int i=0; i < nrLevels; i++)
    cv::add(levels[i], _output, _output);
}



void ImgPyr::build(const int _nrLevels, const cv::Mat& _lowest_lvl){

    levels.clear();
    nrLevels = _nrLevels; // setting the number of levels
    cv::buildPyramid(_lowest_lvl.clone(), levels, _nrLevels-1);

    if (isHomogeneous) {
        convertToHomogeneous(false, cv::Size());
    }
}



void ImgPyr::applyFilter(const cv::Mat& kernel){

    CvPoint anchor = cvPoint(1,1);

    for (int i = 0; i < nrLevels; i++)
        cv::filter2D(levels[i], levels[i], levels[0].depth(), kernel, anchor);
}



void ImgPyr::convertToHomogeneous(bool new_size, cv::Size _size){

    if (new_size) {

        cv::Mat aux(_size.height, _size.width, levels[0].depth());
        for (int i = 0; i < nrLevels; i++){
            cv::resize(levels[i], aux, aux.size());
            levels[i].release();
            levels[i] = aux.clone();
        }
    }
    else {
        cv::Mat aux(levels[0].size(), levels[0].depth());
        for (int i = 1; i < nrLevels; i++){
            cv::resize(levels[i], aux, aux.size());
            levels[i].release();
            levels[i] = aux.clone();
        }
    }


    isHomogeneous = true;
}
