/* 
 * File:   Histogram.h
 * Author: ricardo
 *
 * Created on 6 de Julho de 2011, 15:26
 */

#ifndef HISTOGRAM_H
#define	HISTOGRAM_H

#include <opencv2/core/core.hpp>

class HistogramND {


public:
        int dims;              // dimensions
        int* histSize;         // number of bins for each dimensions
        float** histRanges;    // min & max pixel value (range) for each bin
        cv::Mat data;          // histogram data
        int* channels;         // channels to use on computation
        const char* name;



        HistogramND() {}
        HistogramND(const char* name, int dim, int* nBins, float** ranges, int* imgChannels);
        HistogramND(const char* name, int dim, int* nBins, float** ranges, int* imgChannels, const cv::Mat& hist);
        HistogramND(const HistogramND& orig);
        HistogramND& operator =(const HistogramND& );
        virtual ~HistogramND();

        const cv::Mat& compute(const cv::Mat& image, const cv::Mat& mask/*, int nImgs*/);

        void normalize(const int alpha);

        cv::Mat& getHist();

        double compare(const HistogramND& hist, int method);

        void backproject(const cv::Mat& input, const double scale, cv::Mat& output);

        static cv::Mat* getHistImages(const cv::Mat& image);

        void show(float scale);

};



class Histogram1D {

    public:

        int histSize[1];        // number of bins
        float hranges[2];       // min and max pixel value
        const float* ranges[1];
        int channels[1];        // only 1 channel used here
        cv::Mat data;           // histogram data


        Histogram1D(const int bins, const double min, const double max);

        ~Histogram1D();

        Histogram1D& operator =(const Histogram1D& );

        const cv::Mat& compute(const cv::Mat& image, const cv::Mat& mask);

        void normalize(const int alpha);

        double compare(const Histogram1D& other_hist, int method);

        void backproject(const cv::Mat& input, const double scale, cv::Mat& output);

        cv::Mat& getHist();

        cv::Mat getHistImage();
};


#endif	/* HISTOGRAM_H */

