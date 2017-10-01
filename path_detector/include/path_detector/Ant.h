#ifndef ANT_H
#define ANT_H

#include <opencv2/opencv.hpp>

//#define NO_TOPDOWN

class Ant {

public:

    // Ant's Behaviours
    enum Behaviour {

        Greedy = 0, /* Pursues the regions of highest intensity */
        Track  = 1, /* Pursues regions whose average intensity is more similar to the ones already visited */
        Centre = 2, /* Pursues the central regions to keep equidistant from the trail’s boundaries */
        Ahead  = 3, /* Pursues only upwards regions and doesn't care about intensity */
        Commit = 4,  /* Chooses the previously selected region */
        Random = 5
    };


    Ant();
    ~Ant();

    // initialises ant
    void init( const cv::Point start_point );

    //reset
    void reset();

    // execute ant
    void run( const cv::Mat& conspMap, const cv::Mat& pheroMap, const cv::Mat &lsdMask);

    void dropPheromone(cv::Mat& pheromoneMap, cv::Mat &ph);

    void dropPheromone(cv::Mat& pheromoneMap, cv::Mat &ph, cv::Mat &phinf);

    //
    void setBehaviourWeight(Behaviour behaviour, double value);

    //
    void setMaxHeight(int height);

    //
    float getReward();

    //
    void setUseLSDMaks(bool useMask);


private:

    // Ant's Neighbour Structure
    struct Neighbour {

        cv::Point coord;    // neighbour's coordinates
        int consp;          // conspicuity level
        int phero;          // pheromone level
    };


    cv::Point launch;               //
    float pheromone;                //
    int maxheight;                  //
    int maxhops;                    //
    int hops;                       //
    std::vector<CvPoint> path;      //
    float reward;                   //
    bool useLSDMask;                //

    Neighbour neigs[12];            //
    cv::RNG randGen;                //

    double bWeight[6];              // Weight for each behaviour


    // fills in the structure neigs with the position (in image coordinates) of each region
    // around the ant (defined with coordinates x and y). pheromone and conspicuity info is also
    // associated to that position
    void findNeighbours(cv::Point coord, Neighbour neigs[], const cv::Mat& img, const cv::Mat& phero);

    // select the best action (in resmean) taking into account ant's sensory input (in neighs)
    void selectBehaviour(Neighbour neighs[], int prev, int meanC, float center, float rand_factor, int resmean[]);



};



#endif // ANT_H
