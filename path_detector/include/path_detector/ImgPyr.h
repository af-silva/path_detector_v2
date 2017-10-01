/* 
 * File:   ImgPyr.h
 * Author: Ricardo Mendonça
 *
 * Created on 4 de Julho de 2011, 21:36
 */

#ifndef IMGPYR_H
#define	IMGPYR_H

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

class ImgPyr {

public:

    // Construtor
    /**
     * Creates a new pyramid, _pyramid, given the number of levels,
     * _nrLevels, and the input image (i.e. level 0) is _pyramid[0].
     */
    ImgPyr();


    ImgPyr(const char* _name, const bool _homogeneous=false);


    // Copy Construtor
    ImgPyr(const ImgPyr& orig);

    //Destrutor
    virtual ~ImgPyr();
    
    
    ImgPyr& operator=(const ImgPyr& pyr);


    ImgPyr& operator-=(const ImgPyr& pyr);

    /* Creates a pyramid by performing a bitwise subtraction at each level
     * of the provided sources ( this - pyr )
     */
    const ImgPyr operator-(const ImgPyr& pyr);


    cv::Mat& getLevel(const int level_idx);

    
    void setName( const char* _name );


    int countLevels();


    /**
     * Creates windows and display on them all levels of the pyramid
     */
    void view();


    /**
     * Adds all images in a pyramid and output it to _output. Only
     * homogenous pyramids considered for now.
     */
    void sumLevels(cv::Mat& _output);


    void build(const int _nrLevels, const cv::Mat& _lowest_lvl);


    void applyFilter(const cv::Mat& kernel);


    void convertToHomogeneous(bool new_size, cv::Size size);


private:

    int nrLevels;

    bool isHomogeneous;

    const char* name;

    std::vector<cv::Mat> levels;

};

#endif	/* IMGPYR_H */

