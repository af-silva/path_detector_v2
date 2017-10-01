/*
 * Saliency.h
 *
 * Created by Pedro Santana (Sat Feb 28 17:20:25 2009)
 * Modified by Ricardo Mendonça <r.mendonca@fct.unl.pt>
 *
 *
 * Copyright 2011
 */

#ifndef SALIENCY_H_
#define SALIENCY_H_

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "ImgPyr.h"
#include "CenterSurrounder.h"
#include "Normalizer.h"


class Saliency {

    public:

        Saliency(cv::Size size);

        ~Saliency();

        void run( const cv::Mat& input, const cv::Mat& mask, const float weight);
        void run( const cv::Mat& input);

        void setNormalizeAll(bool _normalize);


        const cv::Mat& getBUSalMap();
        const cv::Mat& getTDSalMap();
        const cv::Mat& getTDConspC();
        const cv::Mat& getTDConspI();
        const cv::Mat& getBUConspC();
        const cv::Mat& getBUConspI();


private:

        cv::Mat buSaliencyMap;  // Bottom-Up Saliency Map
        cv::Mat tdSaliencyMap;  // Top-Down Saliency Map

        cv::Mat conspIMap;      // Bottom-Up Intensity Conspicuity Map
        cv::Mat conspCMap;      // Bottom-Up Colour Conspicuity Map
        cv::Mat tdConspIMap;    // Top-Down Intensity Conspicuity Map
        cv::Mat tdConspCMap;    // Top-Down Colour Conspicuity Map

        cv::Mat excitationMap;      //
        cv::Mat inhibitionMap;      //

        cv::Mat imgI;       // Intensity (Luminance = (r + g + b) / 3)
        cv::Mat imgColour;  // Colour (RGB color space)
        cv::Mat imgR;       // R Channel Image
        cv::Mat imgG;       // G Channel Image
        cv::Mat imgB;       // B Channel Image
        cv::Mat imgY;       // ??

        cv::Mat intensityOnOffMap;  // I On Off Feature Map
        cv::Mat intensityOffOnMap;  // I Off On Feature Map

        cv::Mat colourRGOnOffMap;   // RG On Off Feature Map
        cv::Mat colourRGOffOnMap;   // RG Off On Feature Map
        cv::Mat colourBYOnOffMap;   // BY On Off Feature Map
        cv::Mat colourBYOffOnMap;   // BY Off On Feature Map

        ImgPyr pyrI;    // Intensity pyramid
        ImgPyr pyrR;    // R Channel pyramid
        ImgPyr pyrG;    // G Channel pyramid
        ImgPyr pyrB;    // B Channel pyramid
        ImgPyr pyrY;    // ? pyramid
        ImgPyr pyrRG;   // Double-Opponency RG pyramid
        ImgPyr pyrGR;   // Double-Opponency GR pyramid
        ImgPyr pyrBY;   // Double-Opponency BY pyramid
        ImgPyr pyrYB;   // Double-Opponency YB pyramid

        ImgPyr pyrIOnOff;   // Intensity On-Off pyramid
        ImgPyr pyrIOffOn;   // Intensity Off-On pyramid
        ImgPyr pyrRGOnOff;  // RG On-Off pyramid
        ImgPyr pyrRGOffOn;  // RG Off-On pyramid
        ImgPyr pyrBYOnOff;  // BY On-Off pyramid
        ImgPyr pyrBYOffOn;  // BY On-Off pyramid

        CenterSurrounder csOp;


  /*unsigned int image_width;
  unsigned int image_height;
  CvSize image_size;
  CvSize maps_size;*/

        int center_surround_pyr_nr;
        int center_surround_maps_nr;

        bool saliency_multiplicative, normalize_conspicuity_maps, normalize_orientations_maps;
        bool normalize_orientation_scales, normalize_colour_maps, normalize_colour_scales;
        bool normalize_intensity_maps, normalize_intensity_scales;
        int normalization_method;

        bool use_itti;

        cv::Mat scoreVector;

        // computes luminance image from coloured input image and return max luminance
        double computeLuminanceImg(const cv::Mat& input, cv::Mat& output);

        // computes luminance and colour features from coloured input image
        void computeImgFeatures(const cv::Mat& input, cv::Mat& imgI, cv::Mat& imgR,
                              cv::Mat& imgG, cv::Mat& imgB, cv::Mat& imgY);
        // computes luminance conspicuity map
        void computeLuminanceConspicuity(const cv::Mat& input, cv::Mat& output);
        // computes colour conspicuity map
        void computeColourConspicuity(const cv::Mat& imgR, const cv::Mat& imgG,
                                    const cv::Mat& imgB, const cv::Mat& imgY,
                                    cv::Mat& output);
        // computes bottom up saliency map
        void computeBUSaliency(const cv::Mat& conspI, const cv::Mat& conspC, cv::Mat& output);
        // computes top down saliency map
        void computeTDSaliency(const cv::Mat& mask, const double featW, cv::Mat& scoreVect, cv::Mat& output);
        // calculate feature score
        double calcFeatScore(const cv::Mat& input, const cv::Mat& mask);

};

#endif 	    /* !SALIENCY_H_ */
