
#include "path_detector/Normalizer.h"
#include <iostream>


void Normalizer::normalize( const cv::Mat& _src, cv::Mat& _dst, const unsigned int _normType) {


    _dst.create(_src.size(), _src.type());

    int M = 255;
    cv::normalize( _src, _dst, 0, double(M), CV_MINMAX );
    double average, globalAverage;
    int count;

    double scalingFactor;

    if (_normType == NORMALIZATION_ROAD) {
        count = localMaximaNr(_src, 20.0, average, globalAverage, true, M);
        scalingFactor = (double(M) - globalAverage) / double(M);
        scalingFactor = (scalingFactor*scalingFactor);
        //std::cout << scalingFactor << std::endl;
    }

    else if (_normType == NORMALIZATION_ITTI) {
        // for better results in people images increase the threshold
        // between 100 and 128
        count = localMaximaNr(_src, 0.0, average, globalAverage, true, M);
        scalingFactor = (double(M) - average)/double(M);
        scalingFactor = (scalingFactor*scalingFactor);
    }

    else if (_normType == NORMALIZATION_FRINTROP) {
        count = localMaximaNr(_src, 128.0, average, globalAverage, true, M);
        scalingFactor = 1/(sqrt(count));
    }

    if ( isinf(scalingFactor) )
        scalingFactor = 0.;
    if ( scalingFactor != scalingFactor ) //is nan
        scalingFactor = 0;

    _dst = _src * scalingFactor;

}


void Normalizer::normalize(ImgPyr& _pyramid, const unsigned int _type ) {

    int levels = _pyramid.countLevels();
    //std::cout << "Pyr LEVEL ";
    for (int i=0; i < levels; i++) {
        //std::cout << i << "... " ;
        normalize(_pyramid.getLevel(i), _pyramid.getLevel(i), _type);
        //std::cout << "Done ";
    }
   // std::cout << std::endl << std::endl;
}



unsigned int Normalizer::localMaximaNr(const cv::Mat& _src, const float _threshold, double& _average,
                                       double& _globalAverage, bool _isNormalised, int _max) {

    double average = 0;             // average of local maxima's intensities
    unsigned int counter = 0;       // counter of local maxima
    double globalAverage = 0;       // global average
    unsigned int globalCounter = 0; // counter of analysed pixels
    float weight = 0;               // accumulated weights for the contributions for the average
    bool found_max = false;         // flag stating whether the global maximum has been found also as a
                                    // local one
    double minVal, maxVal;          //
    float temp = 0.0f;              //

    // determining global maximum if the image is not already normalised
    unsigned int max = _max;
    if (!_isNormalised) {
        cv::minMaxLoc( _src, &minVal, &maxVal );  //globalMax(_src);
        max = (unsigned int) maxVal;
    }
    #ifdef VIEW_LOCAL_MAXIMA
    //for drawing purposes
    //IplImage* imgTmp = cvCreateImage(cvSize(_src->width,_src->height),IPL_DEPTH_8U,1);
    //cvSetZero(imgTmp);
    cv::Mat imgTmp(_src.cols, _src.rows, CV_8UC1);
    imgTmp = cv::Scalar(0);
    #endif

    int nl = _src.rows;    // number of lines
    int nc = _src.cols;    // number of columns

    for (int i = 1; i < nc-1; i++) {

        const cv::Mat tempM = _src.col(i);
        int s = _src.step;
        const uchar* prev = (const uchar*) _src.col(i-1).data; //ptr<const uchar>(j-1);     // previous column
        const uchar* cur = (const uchar*) tempM.data; //ptr<const uchar>(j);        // current column
        const uchar* next = (const uchar*) _src.col(i+1).data; //ptr<const uchar>(j+1);     // next column

        for (int j = 1; j < nl-1; j++) {

            // performing global metrics (new stuff)
            temp = float(cur[j]);
            weight += ( temp = sqrt( ( float(j) / float(nl) ) ) );
            globalAverage += ( float(cur[j*s]) * temp );
            //std::cout << globalAverage << std::endl;
            globalCounter++;

            // checking for the local maxima (itti & frintrop)
            if ( (cur[j*s] > prev[j*s-1*s]) && (cur[j*s] > prev[j*s]) && (cur[j*s] > prev[j*s+1*s]) &&
                 (cur[j*s] > next[j*s-1*s]) && (cur[j*s] > next[j*s]) && (cur[j*s] > next[j*s+1*s]) &&
                 (cur[j*s] > cur[j*s-1*s])   && (cur[j*s] > cur[j*s+i*s]) && (cur[j*s] >= _threshold) )
            {
                counter++;
                average += cur[j*s];

                if ( cur[j*s] == max )
                    found_max = true;

                #ifdef VIEW_LOCAL_MAXIMA
                    cv::circle(imgTmp, cv::Point(i,j), 1, CV_RGB(255,255,255),2, 7, 0 );
                #endif
            }
        }

        //performing global metrics (first and last row)
        weight += ( temp = sqrt( ( float(nl-1) / float(nl) ) ) ); // last row (first row -> always zero)
        globalAverage += (float(cur[0]) * temp ); // first row
        globalAverage += (float(cur[nl*s-1*s]) * temp ); // last row
        globalCounter += 2; // (counts first and last row)
    }

    // performing the global average (new stuff)
    _globalAverage = globalAverage / weight;

    // computing the average intensity of all local maxima (itti & frintrop)
    if (counter > 1) {
        if (found_max) {
            average = average - max;
            _average = average / double(counter-1);
        } else {
            _average = average / double(counter);
        }
    } else if (counter == 1) {
                _average = 0;
            } else {
                _average = max;
            }

    #ifdef VIEW_LOCAL_MAXIMA
    cv::namedWindow("LOCAL MAXIMA", CV_WINDOW_AUTOSIZE );
    cv::addWeighted(_src,0.7,imgTmp,0.3,0.0, imgTmp);
    cv::imshow("LOCAL MAXIMA", imgTmp);
    cout << "Local Maxima. Counter: " << counter << " Average: " << average << endl;
    cv::waitKey(0);
    //cvReleaseImage(&imgTmp);
    #endif

    return counter;
}
