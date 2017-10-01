#include "path_detector/TrailAnalyzer.h"


TrailAnalyzer::TrailAnalyzer( cv::Size size, double weight )
{

    sampleRegion = cv::Mat( size, CV_8UC1, cv::Scalar(0) );
    sampleMask = cv::Mat( size, CV_8UC1, cv::Scalar(0) );
    seedMap = cv::Mat( size, CV_8UC3, cv::Scalar(0) );
    hsv = cv::Mat( size, CV_8UC1, cv::Scalar(0) );
    tempMask = cv::Mat( size.height+2, size.width+2, CV_8UC1 );

    seed.x = size.width/2;
    seed.y = size.height/1.5;

    infoWeight = weight;
    firstFrame = true;
    trailProbMap = cv::Mat( size, CV_8UC1, cv::Scalar(0));
    sensitivity = 20;

    int hist_bins[]  = { 16,16,16 };
    float h_range[] = { 0.0, 255.0 };
    float s_range[] = { 0.0, 255.0 };
    float v_range[] = { 0.0, 255.0 };
    float* hist_ranges[]  = { h_range, s_range, v_range };
    int channels[] = {0, 1, 2};

    nTrailInfo = HistogramND("N_HIST", 3, hist_bins, hist_ranges, channels);
    pTrailInfo = HistogramND("P_HIST", 3, hist_bins, hist_ranges, channels);

};


TrailAnalyzer::~TrailAnalyzer() {};


void TrailAnalyzer::reset() {

    sampleRegion = cv::Scalar(0);
    seedMap = cv::Scalar(0);
    sampleMask = cv::Scalar(0);
    tempMask = cv::Scalar(0);
    hsv = cv::Scalar(0);
    firstFrame = true;
    trailProbMap = cv::Scalar(0);
}


void TrailAnalyzer::run( const cv::Mat& inputFrame, const cv::Mat& region,
                         const cv::Mat& saliencyMap) {

    cv::Rect maskRect;
    cv::Mat tempMat = saliencyMap.clone();
    double minVal, maxVal;
    cv::Point minIdx, maxIdx;

    cv::threshold( region, sampleRegion, 240, 255, cv::THRESH_BINARY );

    convertToC1C2C3(inputFrame, hsv);

    cv::split( hsv, planes );

    sampleMask = cv::Scalar(0);
    tempMask = cv::Scalar(0);

    // find the best seed location
    cv::minMaxLoc(saliencyMap, &minVal, &maxVal, &minIdx, &maxIdx, sampleRegion );

    seed.x =  maxIdx.x ;
    seed.y =  maxIdx.y ;

    // compute sample mask starting from seed
    cv::floodFill( tempMat, tempMask, seed, cv::Scalar(255), &maskRect, cv::Scalar(sensitivity),
                cv::Scalar(sensitivity), cv::FLOODFILL_FIXED_RANGE | cv::FLOODFILL_MASK_ONLY | 8 );

    tempMat = tempMask( cv::Rect(1,1, saliencyMap.cols, saliencyMap.rows) );
    cv::threshold( tempMat, sampleMask, 0, 255, cv::THRESH_BINARY );

    // get new information about the trail
    nTrailInfo.compute( hsv, sampleMask );
    nTrailInfo.normalize(1);

    cv::Mat bins = nTrailInfo.getHist();
    cv::Mat prevBins = pTrailInfo.getHist();

    // update trail descriptor with new information
    if ( !firstFrame ) {
        bins = bins * infoWeight + prevBins * (1.0 - infoWeight);
    }

    pTrailInfo = nTrailInfo;


    cv::cvtColor( saliencyMap, seedMap, CV_GRAY2BGR );
    cv::circle( seedMap, cv::Point(maxIdx.x, maxIdx.y), 2, CV_RGB(255,0,0), CV_FILLED );
    cv::circle( seedMap, cv::Point(seed.x, seed.y), 2, CV_RGB(0,255,0), CV_FILLED );

    nTrailInfo.backproject( hsv, 255.0, trailProbMap);

    cv::medianBlur(trailProbMap, trailProbMap, 3);

    if (firstFrame)
        firstFrame = false;
}



const cv::Mat& TrailAnalyzer::getSampleMask() {

    return sampleMask;
}


const cv::Mat& TrailAnalyzer::getTrailProbMap() {

    return trailProbMap;
}


const cv::Mat& TrailAnalyzer::getSeedMap() {

    return seedMap;
}


const double& TrailAnalyzer::getInfoWeight() {

    return infoWeight;
}


bool TrailAnalyzer::setInfoWeight(double new_weight) {

    if ( (new_weight < 0) || (new_weight > 1.0) )
        return false;
    else
        infoWeight = new_weight;

    return true;
}


void TrailAnalyzer::setAnalyzerSensitivity(unsigned int _sensitivity) {

    sensitivity = _sensitivity;
}


void TrailAnalyzer::convertToC1C2C3(const cv::Mat& source, cv::Mat& output){

    // allocate if necessary
    cv::Mat temp(source.size(), CV_32FC3);
    output.create(source.size(), CV_32FC3);

    int nl= source.rows;  // number of lines
    int nc= source.cols ; // number of columns
    // is it a continous image?
    if (source.isContinuous()) { // then no padded pixels
        nc= nc*nl;
        nl= 1; // it is now a 1D array
    }

    // for all pixels
    for (int j=0; j< nl; j++) {
        // pointer to first column of line j
        uchar* data = source.data + j*source.step;
        float* out  = (float*)(temp.data + j*temp.step);
        for (int i=0; i< nc; i++) {

            *out++= cv::saturate_cast<float>( atan( float(data[2]) / float(std::max(data[1],data[0])) )); //c1
            *out++= cv::saturate_cast<float>( atan( float(data[1]) / float(std::max(data[2],data[0])) )); //c2
            *out++= cv::saturate_cast<float>( atan( float(data[0]) / float(std::max(data[2],data[1])) )); //c3
            data += 3;
        } // end of line
    }
    temp *= 255;
    temp.convertTo(output, CV_8UC3);

}
