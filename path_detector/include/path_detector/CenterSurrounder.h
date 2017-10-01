/*
 * CenterSurrounder
 *
 * Created by Pedro Santana (Sat Feb 28 17:20:25 2009)
 * Modified by Ricardo Mendonça <r.mendonca@fct.unl.pt>
 *
 *
 * Copyright 2011
 */

#ifndef CENTER_SURROUNDER_H
#define CENTER_SURROUNDER_H


#include "ImgPyr.h"


class CenterSurrounder {


    public:

        CenterSurrounder();

        virtual ~CenterSurrounder();

        void centerSurround(ImgPyr& _src, ImgPyr& _dst, bool _onOff);

        void centerSurround(ImgPyr& _src1, ImgPyr& _src2, ImgPyr& _dst, bool _onOff);

    private:

        void _centerSurroundCore(const cv::Mat& c, const cv::Mat& s, cv::Mat& _output, bool _onoff);

        //void centerSurround(const int _c, const int _s, Pyramid _pyramid_c,
        //                    Pyramid _pyramid_s, IplImage* _output, bool _onoff, bool _normaliseHere, int _normalisationMethod);

};

#endif // CENTER_SURROUNDER_H
