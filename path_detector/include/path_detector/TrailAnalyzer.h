#ifndef TRAILANALYZER_H
#define TRAILANALYZER_H

#include <opencv2/opencv.hpp>
#include "Histogram.h"

class TrailAnalyzer {

    private:

        cv::Mat sampleRegion;       // valid region for sampling
        cv::Point seed;             // sample seed
        cv::Mat seedMap;            // seed location on saliency map
        cv::Mat sampleMask;         // sample mask obtained with the seed
        cv::Mat tempMask;           // temporary sample mask
        cv::Mat hsv;                // input frame converted to HSV color space
        cv::Mat planes[3];          // HSV color space channels
        HistogramND pTrailInfo;     // previous trail descriptor
        HistogramND nTrailInfo;     // new trail descriptor
        double infoWeight;          // new trail information weight
        bool firstFrame;            // first frame flag
        unsigned int sensitivity;   // sensitivity for obtaining sample mask
        cv::Mat trailProbMap;       // trail probability map

    public:

        TrailAnalyzer( cv::Size input_size, double weight );
        ~TrailAnalyzer();

        void reset();

        //
        void run( const cv::Mat& inputFrame, const cv::Mat& sampleRegion,
                  const cv::Mat& saliencyMap);

        //
        const cv::Mat& getSampleMask();

        //
        const cv::Mat& getTrailProbMap();

        //
        const cv::Mat& getSeedMap();

        //
        const double& getInfoWeight();

        //
        bool setInfoWeight(double new_weight);

        //
        void setAnalyzerSensitivity(unsigned int sensitivity);

        //
        void convertToC1C2C3(const cv::Mat& source, cv::Mat& output);

};

#endif // TRAILANALYZER_H
