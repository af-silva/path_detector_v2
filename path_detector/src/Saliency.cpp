//
// SaliencyMap.cc
//

#include <iostream>
#include "path_detector/Saliency.h"



Saliency::Saliency(cv::Size size) {

    center_surround_pyr_nr = 9;
    center_surround_maps_nr = 6;



    cv::Size maps_size( size.width/4, size.height/4 );

    use_itti = false;

    // in case we want to have orientations modulating colour and intensity
    saliency_multiplicative = 0;

    //normalization enabling flags defaults
    normalize_conspicuity_maps = 1;
    normalize_orientations_maps = 1;
    normalize_orientation_scales = 1;
    normalize_colour_maps = 1;
    normalize_colour_scales = 1;
    normalize_intensity_maps = 1;
    normalize_intensity_scales = 1;

    // normalisation method by default
    normalization_method = NORMALIZATION_ROAD;

    // allocating intensity image
    imgI = cv::Mat( size, CV_8UC1 );

    // allocating colour-opponent channels
    imgR = cv::Mat( size, CV_8UC1 );
    imgG = cv::Mat( size, CV_8UC1 );
    imgB = cv::Mat( size, CV_8UC1 );
    imgY = cv::Mat( size, CV_8UC1 );

    // allocating intensity feature maps
    intensityOnOffMap = cv::Mat( maps_size, CV_8UC1 );
    intensityOffOnMap = cv::Mat( maps_size, CV_8UC1 );

    // allocating double colour-opponent features maps
    colourRGOnOffMap = cv::Mat( maps_size, CV_8UC1 );
    colourRGOffOnMap = cv::Mat( maps_size, CV_8UC1 );
    colourBYOnOffMap = cv::Mat( maps_size, CV_8UC1 );
    colourBYOffOnMap = cv::Mat( maps_size, CV_8UC1 );

    // allocating conspicuity maps images
    conspIMap = cv::Mat( maps_size, CV_8UC1 );
    conspCMap = cv::Mat( maps_size, CV_8UC1 );
    tdConspIMap = cv::Mat( maps_size, CV_8UC1 );
    tdConspCMap = cv::Mat( maps_size, CV_8UC1 );

    // allocating saliency map image
    buSaliencyMap = cv::Mat( maps_size, CV_8UC1 );
    tdSaliencyMap = cv::Mat( maps_size, CV_8UC1 );
    excitationMap = cv::Mat( maps_size, CV_8UC1 );
    inhibitionMap = cv::Mat( maps_size, CV_8UC1 );

    pyrI = ImgPyr("INTENSITY");
    pyrR = ImgPyr("COLOUR R");
    pyrG = ImgPyr("COLOUR G");
    pyrB = ImgPyr("COLOUR B");
    pyrY = ImgPyr("COLOUR Y");

    pyrRG = ImgPyr("COLOUR RG");
    pyrGR = ImgPyr("COLOUR GR");
    pyrBY = ImgPyr("COLOUR BY");
    pyrYB = ImgPyr("COLOUR YB");

    pyrIOnOff = ImgPyr("INTENSITY ON-OFF", true);
    pyrIOffOn = ImgPyr("INTENSITY OFF-ON", true);
    pyrRGOnOff = ImgPyr("COLOUR RG ON-OFF", true);
    pyrRGOffOn = ImgPyr("COLOUR RG OFF-ON", true);
    pyrBYOnOff = ImgPyr("COLOUR BY ON-OFF", true);
    pyrBYOffOn = ImgPyr("COLOUR BY OFF-ON", true);

    pyrIOnOff.build(center_surround_maps_nr, cv::Mat(maps_size, CV_8UC1, cv::Scalar(255)));
    pyrIOffOn.build(center_surround_maps_nr, cv::Mat(maps_size, CV_8UC1, cv::Scalar(255)));
    pyrRGOnOff.build(center_surround_maps_nr, cv::Mat(maps_size, CV_8UC1, cv::Scalar(255)));
    pyrRGOffOn.build(center_surround_maps_nr, cv::Mat(maps_size, CV_8UC1, cv::Scalar(255)));
    pyrBYOnOff.build(center_surround_maps_nr, cv::Mat(maps_size, CV_8UC1, cv::Scalar(255)));
    pyrBYOffOn.build(center_surround_maps_nr, cv::Mat(maps_size, CV_8UC1, cv::Scalar(255)));


    scoreVector = cv::Mat( 1, 8, CV_32FC1 );
    scoreVector = cv::Scalar(0.0);
}



Saliency::~Saliency(){
  //releasing data structures
}



// TODO - André - calculo dos conspicuity maps de cor e intensidade
void Saliency::run(const cv::Mat& input, const cv::Mat& td_mask, const float weight) {

    // compute input image features
    computeImgFeatures( input, imgI, imgR, imgG, imgB, imgY );

    // compute conspicuity maps (colour and luminance)
    computeLuminanceConspicuity( imgI, conspIMap );
    computeColourConspicuity( imgR, imgG, imgB, imgY, conspCMap );

    computeBUSaliency( conspIMap, conspCMap, buSaliencyMap );

    cv::medianBlur(conspIMap, conspIMap, 3);
    cv::medianBlur(conspCMap, conspCMap, 3);

    if (td_mask.data) {
        computeTDSaliency(td_mask, weight, scoreVector, tdSaliencyMap );
    }

//    cv::imshow("TD Sal", tdSaliencyMap);
//    cv::imshow("TD Consp I", tdConspIMap);
//    cv::imshow("TD Consp C", tdConspCMap);
}


//
void Saliency::run(const cv::Mat& input) {

    // compute input image features
    computeImgFeatures( input, imgI, imgR, imgG, imgB, imgY );

    // compute conspicuity maps (colour and luminance)
    computeLuminanceConspicuity( imgI, conspIMap );
    computeColourConspicuity( imgR, imgG, imgB, imgY, conspCMap );

    computeBUSaliency( conspIMap, conspCMap, buSaliencyMap );

    cv::medianBlur(conspIMap, conspIMap, 3);
    cv::medianBlur(conspCMap, conspCMap, 3);
}



void Saliency::setNormalizeAll(bool _normalize) {

  normalize_conspicuity_maps = _normalize;
  normalize_orientations_maps = _normalize;
  normalize_orientation_scales = _normalize;
  normalize_colour_maps = _normalize;
  normalize_colour_scales = _normalize;
  normalize_intensity_maps = _normalize;
  normalize_intensity_scales = _normalize;
}


//
void Saliency::computeImgFeatures(const cv::Mat& input, cv::Mat& imgI, cv::Mat& imgR, cv::Mat& imgG, cv::Mat& imgB, cv::Mat& imgY)
{

    // computes luminance image and maximum luminance
    double maxLuminance = 0.0;
    double lum_threshold = 0.0;
    cv::cvtColor(input, imgI, CV_BGR2GRAY);
    cv::minMaxLoc(imgI, &lum_threshold, &maxLuminance); //computeLuminanceImg(input, imgI);
    lum_threshold = 0.1 * maxLuminance;

    // computes colour images
    int nl = input.rows;    // number of lines
    int nc = input.cols;    // number of columns

    // allocate if necessary
    imgR.create( input.size(), CV_8UC1 );
    imgG.create( input.size(), CV_8UC1 );
    imgB.create( input.size(), CV_8UC1 );
    imgY.create( input.size(), CV_8UC1 );

    if ( input.isContinuous() ) {   // continous image?
        nc = nc * nl;               // then no padded pixels
        nl = 1;                     // it is now a 1D array
    }

    for (int i = 0; i < nl; i++) {

        const uchar* ptrIn = input.ptr<uchar>(i);
        uchar* ptrI = imgI.ptr<uchar>(i);
        uchar* ptrR = imgR.ptr<uchar>(i);
        uchar* ptrG = imgG.ptr<uchar>(i);
        uchar* ptrB = imgB.ptr<uchar>(i);
        uchar* ptrY = imgY.ptr<uchar>(i);

        for (int j = 0; j < nc; j++) {

            // creating the four colour channels (R,G,B,Y) for this pixel
            if (ptrI[j] > lum_threshold) {
                ptrR[j] = cv::saturate_cast<uchar>(85*fmax(0, ptrIn[2] - (ptrIn[1] + ptrIn[0])/2) / ptrI[j]);
                ptrG[j] = cv::saturate_cast<uchar>(85*fmax(0, ptrIn[1] - (ptrIn[2] + ptrIn[0])/2) / ptrI[j]);
                ptrB[j] = cv::saturate_cast<uchar>(85*fmax(0, ptrIn[0] - (ptrIn[2] + ptrIn[1])/2) / ptrI[j]);
                ptrY[j] = cv::saturate_cast<uchar>(85*fmax(0, (ptrIn[2] + ptrIn[1])/2 - ptrIn[0] - fabs(ptrIn[2]-ptrIn[1]))/ptrI[j]);

            } else  // colour of low luminance pixels is discarded
                ptrR[j] =  ptrG[j] =  ptrB[j] =  ptrY[j] = 0;

            ptrIn += 3;
        }
    }

}


// computes luminance image from coloured input image
/*double Saliency::computeLuminanceImg(const cv::Mat& input, cv::Mat& output) {

    double maxLum = -1;     // maximum luminance (default: -1)
    double lum = 0.0;       // luminance level
    int nl = input.rows;    // number of lines
    int nc = input.cols;    // number of columns

    // allocate if necessary
    output.create( input.size(), CV_8UC1 );

    if ( input.isContinuous() ) {   // continous image?
        nc = nc * nl;               // then no padded pixels
        nl = 1;                     // it is now a 1D array
    }

    for (int row = 0; row < nl; row++) {

        const uchar* ptrIn = input.ptr<uchar>(row);
        uchar* ptrOut = output.ptr<uchar>(row);

        for (int col = 0; col < nc; col++) {

            lum = cv::saturate_cast<uchar>((ptrIn[0] + ptrIn[1] + ptrIn[2] )/ 3);
            *ptrOut++ = lum;
            ptrIn += 3;
            if (lum > maxLum)
                maxLum = lum;
        }
    }
    return maxLum;
}*/


// computes luminance conspicuity map
void Saliency::computeLuminanceConspicuity(const cv::Mat& input, cv::Mat& output) {

    // allocate if necessary
    output.create( input.size(), input.type() );

    // intensity image pyramid
    pyrI.build(center_surround_pyr_nr, input);

    // center-surround operations
    csOp.centerSurround( pyrI, pyrIOnOff, true );
    csOp.centerSurround( pyrI, pyrIOffOn, false );

    // scales normalisation operations
    if (normalize_intensity_scales && !use_itti) {
      Normalizer::normalize(pyrIOnOff, normalization_method);
      Normalizer::normalize(pyrIOffOn, normalization_method);
    }

    // creating intensity across-scale maps
    pyrIOnOff.sumLevels( intensityOnOffMap );
    pyrIOffOn.sumLevels( intensityOffOnMap );

    // maps normalisation operations
    if (normalize_intensity_maps) {
        Normalizer::normalize(intensityOnOffMap, intensityOnOffMap, normalization_method);
        Normalizer::normalize(intensityOffOnMap, intensityOffOnMap, normalization_method);
    }

    // adding feature maps (re-scaled) to build up conspicuity map
    output = (intensityOnOffMap * 0.5 + intensityOffOnMap * 0.5);

    // normalizing luminance conspicuity map
    if (normalize_conspicuity_maps)
        Normalizer::normalize(output, output, normalization_method);

    cv::normalize(output, output, 0, 255, CV_MINMAX);
}


// computes colour conspicuity map
void Saliency::computeColourConspicuity(const cv::Mat& inputR, const cv::Mat& inputG,
                                        const cv::Mat& inputB, const cv::Mat& inputY,
                                        cv::Mat& output) {

    // allocate if necessary
    output.create( inputR.size(), inputR.type() );

    // independent colour channels pyramids
    pyrR.build(center_surround_pyr_nr, inputR);
    pyrG.build(center_surround_pyr_nr, inputG);
    pyrB.build(center_surround_pyr_nr, inputB);
    pyrY.build(center_surround_pyr_nr, inputY);

    pyrRG = pyrR - pyrG;
    pyrGR = pyrG - pyrR;
    pyrBY = pyrB - pyrY;
    pyrYB = pyrY - pyrB;

    // center-surround operations
    csOp.centerSurround( pyrRG, pyrRG, pyrRGOnOff, true );
    csOp.centerSurround( pyrGR, pyrGR, pyrRGOffOn, true );
    csOp.centerSurround( pyrBY, pyrBY, pyrBYOnOff, true );
    csOp.centerSurround( pyrYB, pyrYB, pyrBYOffOn, true );

    // normalization scale operations
    // double t = (double) cv::getTickCount();
    if (normalize_colour_scales && !use_itti) {
        Normalizer::normalize( pyrRGOnOff, normalization_method );
        Normalizer::normalize( pyrBYOnOff, normalization_method );
        Normalizer::normalize( pyrRGOffOn, normalization_method );
        Normalizer::normalize( pyrBYOffOn, normalization_method );
    }
    //t = (cv::getTickCount() - t) / cv::getTickFrequency(); // the elapsed time in ms
    //std::cout << "Normalization ColourOnOffPyrs -- Duration (ms): " << (t*1000) << std::endl;

    // creating colour across-scale maps
    pyrRGOnOff.sumLevels( colourRGOnOffMap );
    pyrRGOffOn.sumLevels( colourRGOffOnMap );
    pyrBYOnOff.sumLevels( colourBYOnOffMap );
    pyrBYOffOn.sumLevels( colourBYOffOnMap );

    // maps normalisation operations
    //t = (double) cv::getTickCount();
    if (normalize_colour_maps) {
        Normalizer::normalize(colourRGOnOffMap, colourRGOnOffMap, normalization_method);
        Normalizer::normalize(colourRGOffOnMap, colourRGOffOnMap, normalization_method);
        Normalizer::normalize(colourBYOnOffMap, colourBYOnOffMap, normalization_method);
        Normalizer::normalize(colourBYOffOnMap, colourBYOffOnMap, normalization_method);
    }
    //t = (cv::getTickCount() - t) / cv::getTickFrequency(); // the elapsed time in ms
    //std::cout << "Normalization ColourOnOffMaps -- Duration (ms): " << (t*1000) << std::endl;

//    cv::imshow("NRGOF", colourRGOnOffMap);
//    cv::imshow("NRGFO", colourRGOffOnMap);
//    cv::imshow("NBYOF", colourBYOnOffMap);
//    cv::imshow("NBYFO", colourBYOffOnMap);

    // adding feature maps (re-scaled) to build up conspicuity map
    output = colourRGOnOffMap * 0.25 + colourRGOffOnMap * 0.25 + colourBYOnOffMap * 0.25 + colourBYOffOnMap * 0.25;

    // normalizing conspicuity map
    //t = (double) cv::getTickCount();
    if (normalize_conspicuity_maps)
        Normalizer::normalize(output, output, normalization_method);
    //t = (cv::getTickCount() - t) / cv::getTickFrequency(); // the elapsed time in ms
    //std::cout << "Normalization ConspCMap -- Duration (ms): " << (t*1000) << std::endl;

    cv::normalize(output, output, 0, 255, CV_MINMAX );
}


// computes
void Saliency::computeBUSaliency(const cv::Mat& conspI, const cv::Mat& conspC, cv::Mat& output) {

    // allocate if necessary
    output.create( conspI.size(), conspI.type() );

    output = conspI * 0.5 + conspC * 0.5;

    cv::medianBlur(output, output, 3);

    cv::normalize(output, output, 0, 255, CV_MINMAX );
}


// computes
void Saliency::computeTDSaliency( const cv::Mat& mask, const double featW, cv::Mat& scoreVect, cv::Mat& output ) {

    excitationMap = cv::Scalar(0);
    inhibitionMap = cv::Scalar(0);

    cv::Mat maps[8];

    cv::Mat tdConspCIMap( cv::Size(80,60), CV_8UC1 );

    maps[0] = intensityOnOffMap;
    maps[1] = intensityOffOnMap;
    maps[2] = colourRGOnOffMap;
    maps[3] = colourRGOffOnMap;
    maps[4] = colourBYOnOffMap;
    maps[5] = colourBYOffOnMap;
    maps[6] = conspIMap;
    maps[7] = conspCMap;

    //double t = (double) cv::getTickCount();

    float* ptr = (float*) scoreVect.data;
    for (int i=0; i<6; i++) {

        ptr[i] = ptr[i] * featW + calcFeatScore(maps[i], mask) * (1-featW);

    }

    // adding feature maps to build up conspicuity maps
    excitationMap = cv::Scalar(0);
    inhibitionMap = cv::Scalar(0);

    for (int i=0; i<2; i++)
    {
        //std::cout << ptr[i] << std::endl;
        if (scoreVector.at<float>(i) > 1.0)
            excitationMap += maps[i] * scoreVector.at<float>(i);
        else if (scoreVector.at<float>(i) < 1.0){
            inhibitionMap += maps[i] * 1.0/scoreVector.at<float>(i);
        }
    }

    tdConspIMap = excitationMap - inhibitionMap;

    excitationMap = cv::Scalar(0);
    inhibitionMap = cv::Scalar(0);

    for (int i=2; i<6; i++)
    {
        //std::cout << ptr[i] << std::endl;
        if (scoreVector.at<float>(i) > 1.0)
            excitationMap += maps[i] * scoreVector.at<float>(i);
        else if (scoreVector.at<float>(i) < 1.0){
            inhibitionMap += maps[i] * 1.0/scoreVector.at<float>(i);
        }
    }

    tdConspCMap = excitationMap - inhibitionMap;

    // normalizing conspicuity maps operations
    if (normalize_conspicuity_maps){
      Normalizer::normalize(tdConspIMap, tdConspIMap, normalization_method);
      Normalizer::normalize(tdConspCMap, tdConspCMap, normalization_method);

      //Normalizer::normalize(tdConspCIMap, tdConspCIMap, normalization_method);
    }

    cv::normalize(tdConspIMap, tdConspIMap, 0, 255, CV_MINMAX);
    cv::normalize(tdConspCMap, tdConspCMap, 0, 255, CV_MINMAX);

    cv::medianBlur(tdConspIMap, tdConspIMap, 3);
    cv::medianBlur(tdConspCMap, tdConspCMap, 3);

    output = tdConspCMap * 0.5 + tdConspIMap * 0.5;
    cv::normalize(output, output, 0, 255, CV_MINMAX);
}


//
double Saliency::calcFeatScore(const cv::Mat& input, const cv::Mat& mask) {

  double result;            //
  double innerPixelCount;   //
  double outerPixelCount;   //
  double innerSalSum;       //
  double outerSalSum;       //

  int nl = input.rows;    // number of lines
  int nc = input.cols;    // number of columns

  if ( input.isContinuous() && mask.isContinuous() ) {   // continous image?
      nc = nc * nl;               // then no padded pixels
      nl = 1;                     // it is now a 1D array
  }

  innerSalSum = outerSalSum = outerPixelCount = innerPixelCount = 0;

  for (int j = 0; j < nl; j++) {

      const uchar* ptrI = input.ptr<uchar>(j);
      const uchar* ptrM = mask.ptr<uchar>(j);

      for (int i = 0; i < nc; i++) {

          if ( ptrM[i] != 0 ) {

              innerSalSum += ptrI[i];
              innerPixelCount++;
          }
          else
          {
              outerSalSum += ptrI[i];
              outerPixelCount++;
          }
      }
  }

  result = (innerSalSum / innerPixelCount)  /  ( outerSalSum / outerPixelCount);


  if ( isinf(result) )
      return 0.0;
  else if ( isnan(result) )
    return 0.0;
  else
    return result;
}


//
const cv::Mat& Saliency::getBUSalMap() {
    return buSaliencyMap;
}

//
const cv::Mat& Saliency::getTDSalMap(){
    return tdSaliencyMap;
}

//
const cv::Mat& Saliency::getTDConspC() {
    return tdConspCMap;
}

//
const cv::Mat& Saliency::getTDConspI() {
    return tdConspIMap;
}

const cv::Mat& Saliency::getBUConspC() {
    return conspCMap;
}

const cv::Mat& Saliency::getBUConspI() {
    return conspIMap;
}
