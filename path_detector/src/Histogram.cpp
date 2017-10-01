/*
 * File:   Histogram.cpp
 */

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "path_detector/Histogram.h"
#include <iostream>


HistogramND::HistogramND(const char* _name, int dim, int* nBins, float** ranges, int* imgChannels) {

    //if (dim > CV_MAX_DIMS)
        // throw exception

    // Prepare arguments for histogram
    name = _name;
    dims = dim;
    histSize = new int[dim];
    memcpy(histSize, nBins, sizeof(int)*dim);

    histRanges = new float*[dim];
    for (int i=0; i<dim; i++) {
        histRanges[i] = new float[2];
        histRanges[i][0] = ranges[i][0];
        histRanges[i][1] = ranges[i][1];
    }

    channels = new int[dim];
    memcpy(channels, imgChannels, sizeof(int)*dim);

}



HistogramND::HistogramND(const char* _name, int dim, int* nBins, float** ranges, int* imgChannels,
        const cv::Mat& histData) {

    //if (dim > CV_MAX_DIMS)
        // throw exception

    // Prepare arguments for histogram
    name = _name;
    dims = dim;
    histSize = new int[dim];
    memcpy(histSize, nBins, sizeof(int)*dim);

    histRanges = new float*[dim];
    for (int i=0; i<dim; i++) {
        histRanges[i] = new float[2];
        histRanges[i][0] = ranges[i][0];
        histRanges[i][1] = ranges[i][1];
    }

    channels = new int[dim];
    memcpy(channels, imgChannels, sizeof(int)*dim);

    data = histData.clone();

}



HistogramND::HistogramND(const HistogramND& orig) {

    name = orig.name;
    dims = orig.dims;
    histSize = new int[dims];
    memcpy(histSize, orig.histSize, sizeof(int)*dims);

    histRanges = new float*[dims];
    for (int i=0; i<dims; i++) {
        histRanges[i] = new float[2];
        histRanges[i][0] = orig.histRanges[i][0];
        histRanges[i][1] = orig.histRanges[i][1];
    }

    channels = new int[dims];
    memcpy(channels, orig.channels, sizeof(int)*dims);

    data = orig.data.clone();
}



HistogramND::~HistogramND() {

    delete histSize;
    delete channels;
    delete[] histRanges;
    data.release();
}



HistogramND& HistogramND::operator =(const HistogramND& hist) {

    // Only do assignment if pyr is a different object from "this".
    if (this != &hist) {

        name = hist.name;
        dims = hist.dims;
        histSize = new int[dims];
        memcpy(histSize, hist.histSize, sizeof(int)*dims);

        histRanges = new float*[dims];
        for (int i=0; i<dims; i++) {
            histRanges[i] = new float[2];
            histRanges[i][0] = hist.histRanges[i][0];
            histRanges[i][1] = hist.histRanges[i][1];
        }

        channels = new int[dims];
        memcpy(channels, hist.channels, sizeof(int)*dims);

        data = hist.data.clone();
    }
    return *this;
}


//
const cv::Mat& HistogramND::compute(const cv::Mat& image, const cv::Mat& mask/*, int nImgs*/) {

    const float** ranges = (const float**) histRanges;
    //cv::MatND hist;
    cv::calcHist(&image, 1, channels, mask, data, dims, histSize, ranges);
    //hist.copyTo(data);
    return data;
}


//
cv::Mat& HistogramND::getHist(){
    return data;
}


//
double HistogramND::compare(const HistogramND& other_hist, int method) {

    cv::Mat eqHist1(data.size(), CV_32FC1);
    cv::Mat eqHist2(data.size(), CV_32FC1);

    cv::normalize(data, eqHist1, 0, 1, cv::NORM_MINMAX, eqHist1.depth());

    cv::normalize(other_hist.data, eqHist2, 0, 1, cv::NORM_MINMAX, eqHist2.depth());

    return cv::compareHist(eqHist1, eqHist2, method);
}


//
void HistogramND::normalize(const int alpha) {
    cv::normalize(data, data, alpha);
}


//
void HistogramND::backproject(const cv::Mat& input, const double scale, cv::Mat &output) {

    output.create(input.size(), CV_8UC1);

    cv::calcBackProject(&input, 1, channels, data, output, (const float**) histRanges, scale, true);
}


// Computes the ND histogram and returns an image of it.
cv::Mat* HistogramND::getHistImages(const cv::Mat& image) {

    // Get min and max bin values
    double maxVal = 0;
    double minVal = 0;

    int histSize[1]; // number of bins
    float hranges[2]; // min and max pixel value
    const float* ranges[1];
    int channels[1]; // only 1 channel used here
    // Prepare arguments for 1D histogram
    histSize[0]= 256;
    hranges[0] = 0.0;
    hranges[1] = 255.0;
    ranges[0] = hranges;
    channels[0] = 0; // by default, we look at channel 0

    cv::Mat* planes = new cv::Mat[image.channels()];
    cv::split(image, planes);
    cv::Mat hist;

    // Images on which to display histogram planes
    cv::Mat* imgs = new cv::Mat[image.channels()];
    for (int i=0; i < image.channels(); i++)
    {
        cv::calcHist(&planes[i], 1, channels, cv::Mat(), hist, 1, histSize, ranges);

        imgs[i] = cv::Mat(histSize[0], histSize[0], CV_8U, cv::Scalar(255));
        // set highest point at 90% of nbins
        int hpt = static_cast<int>(0.9*histSize[0]);
        cv::minMaxLoc(planes[i], &minVal, &maxVal, 0, 0);
        // Draw a vertical line for each bin
        for( int h = 0; h < histSize[0]; h++ )
        {
            float binVal = hist.at<float>(h);
            int intensity = static_cast<int>(binVal*hpt  / maxVal);
            // This function draws a line between 2 points
            cv::line( imgs[i], cv::Point(h, histSize[0]),
                     cv::Point(h, histSize[0]-intensity ), cv::Scalar::all(0) );
        }
    }


    return imgs;
}



void HistogramND::show(float scale) {

    // Get min and max bin values
    double maxVal = 0;
    double minVal = 0;

    if ( (dims != 1) || (data.channels() != 1) )
        return;

    cv::Mat img(histSize[0]*scale, histSize[0]*scale, CV_8U, cv::Scalar(255));

    cv::minMaxLoc(data, &minVal, &maxVal, 0, 0);

    // set highest point at 90% of nbins
    int hpt = static_cast<int>(0.9*histSize[0]*scale);
    // Draw a vertical line for each bin
    for( int h = 0; h < histSize[0]; h++ )
    {
        float binVal = data.at<float>(h);
        int intensity = static_cast<int>(binVal*hpt  / maxVal);
        // This function draws a line between 2 points
        cv::line( img, cv::Point(h*scale, histSize[0]*scale),
                  cv::Point(h*scale, histSize[0]*scale-intensity ), cv::Scalar::all(0) );
    }

    cv::imshow(name, img);
    cv::waitKey(5);

}






/****************************************************************************/




Histogram1D::Histogram1D(const int bins, const double minVal, const double maxVal) {

    // Prepare arguments for 1D histogram
    histSize[0]= bins;
    hranges[0]= minVal;
    hranges[1]= maxVal;
    ranges[0]= hranges;
    channels[0]= 0; // by default, we look at channel 0
}



Histogram1D::~Histogram1D() {

    data.release();
}



Histogram1D& Histogram1D::operator =(const Histogram1D& hist) {

    // Only do assignment if pyr is a different object from "this".
    if (this != &hist) {

        histSize[0] = hist.histSize[0];
        hranges[0]= hist.hranges[0];
        hranges[1]= hist.hranges[1];
        ranges[0]= hranges;
        channels[0]= 0;
        data = hist.data.clone();
    }
    return *this;
}


//
const cv::Mat& Histogram1D::compute(const cv::Mat& image, const cv::Mat& mask) {

    cv::calcHist(&image, 1, channels, mask, data, 1, histSize, ranges);

    return data;
}


//
void Histogram1D::normalize(const int alpha) {
    cv::normalize(data, data, alpha);
}


//
void Histogram1D::backproject(const cv::Mat& input, const double scale, cv::Mat &output) {

    output.create(input.size(), CV_8UC1);

    cv::calcBackProject(&input, 1, channels, data, output, ranges, scale, true);
}


//
cv::Mat& Histogram1D::getHist(){

    return data;
}


//
double Histogram1D::compare(const Histogram1D& other_hist, int method) {

    return cv::compareHist(data, other_hist.data, method);
}



cv::Mat Histogram1D::getHistImage() {

    // Get min and max bin values
    double maxVal=0;
    double minVal=0;
    cv::minMaxLoc(data, &minVal, &maxVal, 0, 0);
    // Image on which to display histogram
    cv::Mat histImg(histSize[0], histSize[0], CV_8U, cv::Scalar(255));
    // set highest point at 90% of nbins
    int hpt = static_cast<int>(0.9*histSize[0]);
    // Draw a vertical line for each bin
    for( int h = 0; h < histSize[0]; h++ ) {
        float binVal = data.at<float>(h);
        int intensity = static_cast<int>(binVal*hpt/maxVal);
        // This function draws a line between 2 points
        cv::line(histImg,cv::Point(h,histSize[0]),
        cv::Point(h,histSize[0]-intensity),
        cv::Scalar::all(0));
    }
    return histImg;
}
