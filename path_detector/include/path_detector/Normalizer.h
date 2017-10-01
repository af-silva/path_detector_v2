/*
 *  Ricardo Mendonça
 *
 *
 */

#ifndef NORMALIZER_H
#define NORMALIZER_H


#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "ImgPyr.h"


#define NORMALIZATION_ITTI 0        // normalization by Itti's definition
#define NORMALIZATION_FRINTROP 1    // normalization by Frinprop's definition
#define NORMALIZATION_SANTANA 2     // normalization by Santana's definition
#define NORMALIZATION_ROAD 3
#define NORMALIZATION_FIXED 4
#define NORMALIZATION_NONE 10


class Normalizer
{
    public:

        //Normalizer();

        //virtual ~Normalizer();

        static void normalize( const cv::Mat& _src, cv::Mat& _dst, const unsigned int _normalisationType );

        static void normalize(ImgPyr& _pyramid, const unsigned int _type );

    private:

        static unsigned int localMaximaNr(const cv::Mat& _src, const float _threshold, double& _average, double& _globalAverage,
                                   bool _isNormalised, int _max);

        //unsigned int globalMax(const cv::Mat& _src);

};

#endif // NORMALIZER_H
