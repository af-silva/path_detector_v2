/* 
 * File:   AntSystem.h
 *
 * Author: Ricardo Mendonça
 *         Pedro Santana
 *         Nelson Alves
 *
 * Created on 17 de Novembro de 2010, 0:16
 */

#ifndef ANTSYSTEM_H
#define	ANTSYSTEM_H

#include <opencv2/opencv.hpp>
#include "Ant.h"
#include "Saliency.h"
#include "TrailAnalyzer.h"

#define NEW_DISPLAY
//#define OLD_DISPLAY
//#define COST_MAP_DISPLAY

class AntSystem {

    private:

        // object that embodies an ant (swarm)
        Ant ant;
        // object to perform saliency computation
        Saliency saliency;
        // object to analyze trail samples
        TrailAnalyzer analyzer;

        bool learningMode;
        bool useLSDMask;
        bool useLSDMaskOnOutput;

        float* displayLUT;  // pheromone display lookup table

        //cv::Mat nf_phMap;   //
        cv::Mat phCI_aux;   //
        cv::Mat nfATTEN;    //
        cv::Mat nfDOG;      //
        cv::Mat nfG1;       //
        cv::Mat nfG2;       //
        cv::Mat nfGAUSS;    //
        cv::Mat nf_aux;     //
        cv::Mat ph_aux;     //

        cv::Mat trailModelMapI; //
        cv::Mat trailModelMapC; //

        cv::Mat attenPrevNeuralField;   //
        cv::Mat neuralField;            //
        cv::Mat neuralField8U;          //

        cv::Mat pheromoneMapCI; //
        cv::Mat pheromoneMapI;  //
        cv::Mat pheromoneMapC;  //
        cv::Mat nf_phMapI;      //
        cv::Mat nf_phMapC;      //


        cv::Mat aux160x120x8Ux3C;   //
        cv::Mat aux160x120x8Ux1C;   //
        cv::Mat aux80x60x32Fx1C;    //
        cv::Mat aux80x60x8Ux1C;     //
        cv::Mat aux80x60x8Ux3C;     //

        cv::Mat inputFrame;                 //
        cv::Mat smallInputFrame;            //
        cv::Mat previousSmallInputFrame;    //
        cv::Mat opticalFlowImg;             //
        
        std::map<unsigned int, unsigned int> shape_map;
        cv::Point2d point;
        cv::Mat neuralFieldBin;          //
        

        bool roadMode;
        int n_p_ants;

        float* meanConspVectC, *meanConspVectI, *meanPheroVectC, *meanPheroVectI;

        void initialise();

        void createConspVect(const cv::Mat& conspMap, float* meanConspVect);

        void createPheroVect(const cv::Mat& pheroMap, float* meanPheroVect);

        void performMotionCompensation(cv::Mat& neural_field);

        int OpticalFlowLK(cv::Mat& imgA, cv::Mat& imgB, cv::Mat& imgC,
                          CvPoint2D32f* cornersA, CvPoint2D32f* cornersB, int maxCorners);

        void executePAnts(const cv::Mat& conspicMap);

        void executeAntsFreeMode(const cv::Mat& conspicMapI, const cv::Mat& conspicMapC, const cv::Mat& lsdMask);

        void updateNeuralField(const cv::Mat &mask);

        void displayPheromone(const cv::Mat& pheromone, cv::Mat& output, cv::Mat& pathTrace);
        
        void applyMask(const cv::Mat& in, const cv::Mat& mask, cv::Mat& out);

        //void createWayPoints();


public:

        AntSystem( int _n_p_ants );

        ~AntSystem();

        void reset();

        void run(const cv::Mat& inputFrame, cv::Mat& output, cv::Mat& pathTrace, const cv::Mat&lsdMask, double vel);

        void setLearningMode(bool onOff);

        void changeAntBehaviour(Ant::Behaviour behaviour, double value);

        void changeAntMaxHeight(int height);

        void changeAnalyzerSensitivity(unsigned int value);

        bool isLearning();

        bool usingLSDSlamMask();
        bool usingLSDSlamMaskOutput();

        void useLSDSlamMask(bool use);
        void useLSDSlamMaskOutput(bool use);

        const cv::Mat& getNeuralField();

        const cv::Mat& getSampleMask();
        const cv::Mat& getSeedMap();
        const cv::Mat& getTrailProbMap();

        const cv::Mat& getPheromoneMapCI();
        const cv::Mat& getPheromoneMapI();
        const cv::Mat& getPheromoneMapC();

        const cv::Mat& getTrailMapI();
        const cv::Mat& getTrailMapC();

        const cv::Mat& getTDConspIMap();
        const cv::Mat& getTDConspCMap();   
        const cv::Mat& getTDSalMap();

        const cv::Mat& getBUConspIMap();
        const cv::Mat& getBUConspCMap();
        const cv::Mat& getBUSalMap();
        
        const cv::Point2d& getWayPoint();
};


#endif	/* ANTSYSTEM_H */

