#include "path_detector/AntSystem.h"

int frame = 0;

// receives info regarding the size of the GUI and whether to use trail or road mode
// as well as the number of p-ants per conspicuity map
AntSystem::AntSystem(int _n_p_ants)
:
    saliency(cv::Size(320, 240)), analyzer(cv::Size(80, 60), 0.1)
{
  roadMode = false;
  useLSDMask = false;

  n_p_ants = _n_p_ants;  // af-silva TODO - depende da velocidade

  // most of the structures are made global in the class context to avoid
  // spending time creating/destroying them in the main cycle

  // input images in different formats
  inputFrame = cv::Mat(cv::Size(320, 240), CV_8UC3);
  smallInputFrame = cv::Mat(cv::Size(160, 120), CV_8UC1);
  previousSmallInputFrame = cv::Mat(cv::Size(160, 120), CV_8UC1);

  // image where optical flow is overlaid (visualisation purposes)
  opticalFlowImg = cv::Mat(cv::Size(160, 120), CV_8UC1);

  aux80x60x8Ux3C = cv::Mat(cv::Size(80, 60), CV_8UC3);
  aux80x60x32Fx1C = cv::Mat(cv::Size(80, 60), CV_32FC1);

  // pheromone maps that have the influence of the neural field. used to affect
  // p-ants motion. these are in fact auxiliary structures that are used to avoid
  // propagating neural field activity to the final saliency map.
  // hence, neural field activity is used to guide p-ants but should not directly
  // propagate to saliency map, which will in turn feed the neural field.
  nf_phMapI = cv::Mat(cv::Size(80, 60), CV_8UC1);
  nf_phMapC = cv::Mat(cv::Size(80, 60), CV_8UC1);

  meanPheroVectC = new float[80];
  meanPheroVectI = new float[80];
  meanConspVectC = new float[80];
  meanConspVectI = new float[80];

  //
  displayLUT = new float[156];
  for (int i = 0; i < 156; i++)
    displayLUT[i] = 0.0064516 * float(i);

  // pheromone map without the influence of neural field
  pheromoneMapCI = cv::Mat(cv::Size(80, 60), CV_8UC1);
  pheromoneMapI = cv::Mat(cv::Size(80, 60), CV_8UC1);
  pheromoneMapC = cv::Mat(cv::Size(80, 60), CV_8UC1);

  trailModelMapI = cv::Mat(cv::Size(80, 60), CV_8UC1);
  trailModelMapC = cv::Mat(cv::Size(80, 60), CV_8UC1);

  // neural field structure
  neuralField = cv::Mat(cv::Size(80, 60), CV_32FC1);
  neuralField8U = cv::Mat(cv::Size(80, 60), CV_8UC1);
  neuralFieldBin = cv::Mat(cv::Size(80, 60), CV_8UC1);
  // attenuated version of the neural field as it was in the previous frame
  attenPrevNeuralField = cv::Mat(cv::Size(80, 60), CV_8UC1);

  // auxiliary images for neural field computation
  nfATTEN = cv::Mat(cv::Size(80, 60), CV_32FC1);
  nfDOG = cv::Mat(cv::Size(80, 60), CV_32FC1);
  nfG1 = cv::Mat(cv::Size(80, 60), CV_32FC1);
  nfG2 = cv::Mat(cv::Size(80, 60), CV_32FC1);
  nfGAUSS = cv::Mat(cv::Size(80, 60), CV_32FC1);

  // auxiliary images for intermediate steps
  aux160x120x8Ux3C = cv::Mat(cv::Size(160, 120), CV_8UC3);
  aux160x120x8Ux1C = cv::Mat(cv::Size(160, 120), CV_8UC1);
  phCI_aux = cv::Mat(cv::Size(80, 60), CV_8UC1);

  // resetting key structures
  reset();
}

//
AntSystem::~AntSystem()
{
  delete meanConspVectC;
  delete meanConspVectI;
  delete meanPheroVectC;
  delete meanPheroVectI;
  delete displayLUT;
}

//
void AntSystem::initialise()
{

  pheromoneMapCI = cv::Scalar(0);
  pheromoneMapI = cv::Scalar(0);
  pheromoneMapC = cv::Scalar(0);
  nf_phMapI = cv::Scalar(0);
  nf_phMapC = cv::Scalar(0);
  nfATTEN = cv::Scalar(0);
  nfDOG = cv::Scalar(0);
  nfG1 = cv::Scalar(0);
  nfG2 = cv::Scalar(0);
  nfGAUSS = cv::Scalar(0);
}

// resetting key structures
void AntSystem::reset()
{

  learningMode = true;
  useLSDMask = false;

  attenPrevNeuralField = cv::Scalar(0);
  neuralField = cv::Scalar(0);
  trailModelMapI = cv::Scalar(0);
  trailModelMapC = cv::Scalar(0);
  attenPrevNeuralField = cv::Scalar(0);
  neuralField = cv::Scalar(0);
  neuralField8U = cv::Scalar(0);
  neuralFieldBin = cv::Scalar(0);
  aux160x120x8Ux3C = cv::Scalar(0);
  aux160x120x8Ux1C = cv::Scalar(0);
  aux80x60x32Fx1C = cv::Scalar(0);
  aux80x60x8Ux1C = cv::Scalar(0);
  aux80x60x8Ux3C = cv::Scalar(0);
  inputFrame = cv::Scalar(0);
  smallInputFrame = cv::Scalar(0);
  previousSmallInputFrame = cv::Scalar(0);
  opticalFlowImg = cv::Scalar(0);

  initialise();

  ant.reset();
  analyzer.reset();

}

//
void AntSystem::run(const cv::Mat& input, cv::Mat& output, cv::Mat& pathTrace, const cv::Mat&lsdMask, double vel)
{

  // homography computation
  smallInputFrame.copyTo(previousSmallInputFrame);
  cv::resize(input, aux160x120x8Ux3C, aux160x120x8Ux3C.size());
  cv::cvtColor(aux160x120x8Ux3C, smallInputFrame, CV_RGB2GRAY);
  performMotionCompensation(neuralField);

  cv::resize(input, aux80x60x8Ux3C, aux80x60x8Ux3C.size());

  if (learningMode)
    saliency.run(input);  // importante, ver ficheiro Saliency.cpp
  else
  {
    if (frame == 0)
    {
      //nao devia ter aki um saliency run primeiro para gerar BU ?
      analyzer.run(aux80x60x8Ux3C, phCI_aux, saliency.getBUSalMap());
      saliency.run(input, analyzer.getSampleMask(), 0.0);
    }
    else
    {
      analyzer.run(aux80x60x8Ux3C, phCI_aux, saliency.getTDSalMap());
      saliency.run(input, analyzer.getSampleMask(), 0.98);
    }


    trailModelMapI = saliency.getTDConspI() * 0.5 + analyzer.getTrailProbMap() * 0.5;
    trailModelMapC = saliency.getTDConspC() * 0.5 + analyzer.getTrailProbMap() * 0.5;

    cv::normalize(trailModelMapI, trailModelMapI, 0.0, 255.0, CV_MINMAX, 0);
    cv::normalize(trailModelMapC, trailModelMapC, 0.0, 255.0, CV_MINMAX, 0);
  }

  // initialisation of system images
  initialise();


  attenPrevNeuralField.copyTo(nf_phMapI);
  attenPrevNeuralField.copyTo(nf_phMapC);

  // iterate p-ants to create the pheromone map
  if (learningMode)
  {
    executeAntsFreeMode(saliency.getBUConspI(), saliency.getBUConspC(), lsdMask);
  }
  else
  {
    executeAntsFreeMode(trailModelMapI, trailModelMapC, lsdMask);
  }

  // update neural field with computed pheromone map
  updateNeuralField(lsdMask);

  // output pheromone results
  input.copyTo(output);

  displayPheromone(phCI_aux, output, pathTrace);
  cv::cvtColor(pathTrace, pathTrace, CV_BGR2GRAY);

  frame++;
}

//
void AntSystem::createConspVect(const cv::Mat& conspMap, float* meanConspVect)
{
  int win_height = 5;
  int win_width = 9;
  int init_row = conspMap.rows - 5;  //not at the bottom to avoid noisy data

  // collapsing slideWindowHeight rows into a single one (hist_array)
  float hist_array[conspMap.cols];

  for (int i = 0; i < conspMap.cols; i++)
    hist_array[i] = 0.0;

  for (int j = 0; j < win_height; j++)
  {
    uchar* imgptr = (uchar*) (conspMap.data + (init_row - j) * conspMap.step);

    for (int i = 0; i < conspMap.cols; i++)
      hist_array[i] += imgptr[i];
  }

  // smoothing hist_array with a sliding window
  for (int i = 0; i < conspMap.cols; i++)
  {
    meanConspVect[i] = 0;
    int cols = 0;

    for (int j = (i - (win_width - 1) / 2); j <= (i + (win_width - 1) / 2); j++)
    {
      // if sliding window out of bounds don't process
      if (j < 0 || j >= conspMap.cols)
        continue;
      else
      {
        meanConspVect[i] += hist_array[j];
        cols++;
      }
    }
    meanConspVect[i] /= float(cols * win_height);
  }
}

//
void AntSystem::createPheroVect(const cv::Mat& pheroMap, float* maxPheroVect)
{
  int win_height = 5;
  int win_width = 9;
  int init_row = pheroMap.rows - 5;  //not at the bottom to avoid noisy data

  for (int i = 0; i < pheroMap.cols; i++)
    maxPheroVect[i] = 0.0;

  for (int i = 0; i < win_height; i++)
  {
    uchar* phptr = (uchar*) (pheroMap.data + (init_row - i) * pheroMap.step);

    for (int j = 0; j < pheroMap.cols; j++)
    {
      for (int x = (j - (win_width - 1) / 2); x <= (j + (win_width - 1) / 2); x++)
      {
        // if sliding window out of bounds don't process
        if (x < 0 || x >= pheroMap.cols)
          continue;
        else
        if (phptr[x] > maxPheroVect[j])
          maxPheroVect[j] = phptr[x];
      }
    }
  }

}

//
void AntSystem::executeAntsFreeMode(const cv::Mat &conspicMapI, const cv::Mat &conspicMapC, const cv::Mat &lsdMask)
{

  // random factor for p-ant deployment
  float ph_factor = 0.3;

  int iteration = 0;

  // reduced boundaries to avoid noise in the image's limits
  int high_x = conspicMapI.cols - /*15*/10, high_y = 55, low_x = /*15*/10, low_y = 50;

  //
  createConspVect(conspicMapI, meanConspVectI);
  createConspVect(conspicMapC, meanConspVectC);
  createPheroVect(nf_phMapI, meanPheroVectI);
  createPheroVect(nf_phMapC, meanPheroVectC);

  // deploying n_p_ants per conspicuity map
  int p_antCounter = 0;

  while (p_antCounter < 2 * n_p_ants)
  {

    // handling intensity channel
    if (p_antCounter % 2 == 0)
    {

      // picking a random column to deploy p-ant
      int rand_x = rand() % (high_x - low_x + 1) + low_x;
      int rand_y = rand() % (high_y - low_y + 1) + low_y;

      float norm_value = float(rand() % 256);
      float value_score = meanPheroVectI[rand_x] * ph_factor +
          meanConspVectI[rand_x] * (1.0 - ph_factor);

      iteration++;

      // deploy the p-ant in the chosen location with higher chances if
      // if conspicuity and pheromone levels are high
      if ((norm_value < value_score) || (iteration > 1000))
      {

        iteration = 0;

        ant.init(cv::Point(rand_x, rand_y));
        // run the p-ant in the conspicuity map
        ant.setUseLSDMaks(useLSDMask);
        ant.run(conspicMapI, nf_phMapI, lsdMask);

        // deploy pheromone on both pheromone maps on the pixels visited by the p-ant TODO andre
        if(ant.getReward() > 1){
        ant.dropPheromone(pheromoneMapI, nf_phMapI, nf_phMapC);
        p_antCounter++;
        }

      }
    }
    else
    {  // handling color channel (repetition of previous case but for color)

      int rand_x = rand() % (high_x - low_x + 1) + low_x;
      int rand_y = rand() % (high_y - low_y + 1) + low_y;

      float norm_value = float(rand() % 256);
      float value_score = meanPheroVectC[rand_x] * ph_factor +
          meanConspVectC[rand_x] * (1.0 - ph_factor);

      iteration++;

      // deploy the p-ant in the chosen location with higher chances if
      // if conspicuity and pheromone levels are high
      if ((norm_value < value_score) || (iteration > 1000))
      {

        iteration = 0;

        ant.init(cv::Point(rand_x, rand_y));
        // run the p-ant in the conspicuity map
        ant.setUseLSDMaks(useLSDMask);
        ant.run(conspicMapC, nf_phMapC, lsdMask); // TODO andre

        // deploy pheromone on both pheromone maps on the pixels visited by the p-ant TODO andre
        if(ant.getReward() > 1){
        ant.dropPheromone(pheromoneMapC, nf_phMapC, nf_phMapI);
        p_antCounter++;
        }

        // making random factor to be smaller for latter p-ants,
        // as in exploitation vs exploration typical trade-off or
        // as considered in simulation annealing
        ph_factor += 0.05;  //0.02
        if (ph_factor > 1) ph_factor = 1;
      }
    }
  }


  // blend pheromone maps (not poluted with neural field info) to obtain final pheromone map
  pheromoneMapCI = pheromoneMapI * 0.5 + pheromoneMapC * 0.5;

  cv::normalize(pheromoneMapCI, pheromoneMapCI, 0, 255, CV_MINMAX);
  cv::GaussianBlur(pheromoneMapCI, pheromoneMapCI, cv::Size(7, 7), 0);
}

//
void AntSystem::updateNeuralField(const cv::Mat &lsdMask)
{
  // obtain an attenuated version of the neural field
  double tao = 0.03;
  nfATTEN = neuralField * (tao * 2.0);  // TODO - af-silva Constante temporal neural field

  // Neural Field Difference of Gaussians
  neuralField.copyTo(nfG1);
  neuralField.copyTo(nfG2);
  cv::GaussianBlur(nfG1, nfG1, cv::Size(25, 25), 0);
  cv::GaussianBlur(nfG2, nfG2, cv::Size(91, 91), 0);
  nfDOG = nfG1 - nfG2;
  nfDOG *= (tao * 2.5);

  // Gaussian Pheromone
  /* saliencyMap */
  pheromoneMapCI.convertTo(nfGAUSS, nfGAUSS.type(), 1.0 / 255.0);
  cv::GaussianBlur(nfGAUSS, nfGAUSS, cv::Size(11, 11), 0);
  nfGAUSS *= (tao * 2.0);




  if (useLSDMask) // TODO andre
  {
    cv::Mat aux80x60x8Ux1C_mask = cv::Mat(60, 80, CV_8UC1, cv::Scalar(0));
    aux80x60x8Ux1C = cv::Scalar(0);  //cvZero(aux80x60x8Ux1C);
    neuralField.convertTo(aux80x60x8Ux1C, aux80x60x8Ux1C.type(), 255.0);  //cvConvertScale(neural_field, aux80x60x8Ux1C, 255, 0);
    lsdMask.copyTo(aux80x60x8Ux1C_mask);
    applyMask(aux80x60x8Ux1C, aux80x60x8Ux1C_mask, aux80x60x8Ux1C);
    cv::imshow("Neural Field", aux80x60x8Ux1C_mask);
    aux80x60x8Ux1C.convertTo(neuralField, neuralField.type(), 1.0 / 255.0);  //cvConvertScale( aux80x60x8Ux1C, neural_field, 1./255., 0 );
  }

  // Neural Field Update
  neuralField = (((neuralField - nfATTEN) + nfDOG) + nfGAUSS);


  // Neural field feedback (to affect pheromone maps in the next frame)
  neuralField.convertTo(phCI_aux, phCI_aux.type(), 255.0);
  attenPrevNeuralField = phCI_aux * 0.1;

  cv::normalize(phCI_aux, phCI_aux, 0.0, 255.0, CV_MINMAX);
  cv::normalize(pheromoneMapC, pheromoneMapC, 0, 255, CV_MINMAX);
  cv::GaussianBlur(pheromoneMapC, pheromoneMapC, cv::Size(7, 7), 0);
  cv::normalize(pheromoneMapI, pheromoneMapI, 0, 255, CV_MINMAX);
  cv::GaussianBlur(pheromoneMapI, pheromoneMapI, cv::Size(7, 7), 0);

}

//
void AntSystem::displayPheromone(const cv::Mat &pheromone, cv::Mat &output, cv::Mat &traceimg)
{

  cv::Mat cloud(cv::Size(output.cols, output.rows), CV_8UC1);
  cv::resize(pheromone, cloud, cloud.size());

  int nc = output.cols;
  int nl = output.rows;

  float w;
  for (int i = 0; i < nl; i++)
  {

    uchar* phptr = (uchar*) (cloud.data + i * cloud.step);
    uchar* imgptr = (uchar*) (output.data + i * output.step);
    uchar* trace = (uchar*) (traceimg.data + i * traceimg.step);

    for (int j = 0; j < nc; j++)
    {
#ifdef NEW_DISPLAY
      if (phptr[j] >= 100)
      {
        w = displayLUT[phptr[j] - 100];
        imgptr[j * 3] = imgptr[j * 3] * (1.0 - w);
        imgptr[j * 3 + 1] = imgptr[j * 3 + 1] * (1.0 - w);
        imgptr[j * 3 + 2] = imgptr[j * 3 + 2] * (1.0 - w) + phptr[j] * w;

        // af-silva changes this
        trace[j * 3] = trace[j * 3] * (1.0 - w);
        trace[j * 3 + 1] = trace[j * 3 + 1] * (1.0 - w);
        trace[j * 3 + 2] = trace[j * 3 + 2] * (1.0 - w) + phptr[j] * w;
      }
#endif

#ifdef COST_MAP_DISPLAY
      if (phptr[j] >= 100)
      {
        imgptr[j*3] = imgptr[j*3] * 0.5;
        imgptr[j*3+1] = 190;
        imgptr[j*3+2] = imgptr[j*3+2] * 0.5;
      }
      else
      {
        imgptr[j*3] = imgptr[j*3] * 0.6;
        imgptr[j*3+1] = imgptr[j*3+1] * 0.6;
        imgptr[j*3+2] = 150;
      }
#endif

#ifdef OLD_DISPLAY
      int intensity = 0;
      if (phptr[j] >= 217 )
      {
        if (phptr[j] == 255)
        cv::circle(output, cvPoint(j,i), 3, CV_RGB(255,0,0), CV_FILLED);
        else
        {
          if (phptr[j] >= 245)
          intensity = phptr[j];
          else if (phptr[j] >= 235)
          intensity = phptr[j]-50;
          else intensity = phptr[j]-100;
          cv::circle(output, cvPoint(j,i), 2, CV_RGB(intensity,0,0), 1);
        }
      }
#endif
    }
  }
}

//
void AntSystem::setLearningMode(bool mode)
{

#ifndef NO_TOPDOWN
  learningMode = mode;

  if (mode)
    analyzer.setInfoWeight(1.0);
  else
  {
    frame = 0;
    analyzer.setInfoWeight(0.1);
  }
#endif
}

//
bool AntSystem::isLearning()
{
  return learningMode;
}

//
bool AntSystem::usingLSDSlamMask()
{
  return useLSDMask;
}


//
void AntSystem::useLSDSlamMask(bool use)
{
  useLSDMask = use;
}


//
void AntSystem::changeAntBehaviour(Ant::Behaviour behaviour, double value)
{
  ant.setBehaviourWeight(behaviour, value);
}

void AntSystem::changeAntMaxHeight(int height)
{
  ant.setMaxHeight(height);
}

//
void AntSystem::changeAnalyzerSensitivity(unsigned int value)
{
  analyzer.setAnalyzerSensitivity(value);
}

//
void AntSystem::performMotionCompensation(cv::Mat& neural_field)
{

  //max number of interest points to be searched in the images
  int maxCorners = 50;

  // finding correspondences between interest points in both current and previous frames
  CvPoint2D32f *crnrsA = new CvPoint2D32f[maxCorners];
  CvPoint2D32f *crnrsB = new CvPoint2D32f[maxCorners];

  int n_corners = OpticalFlowLK(previousSmallInputFrame, smallInputFrame, opticalFlowImg, crnrsA, crnrsB, maxCorners);
  CvPoint2D32f* cornersA = new CvPoint2D32f[n_corners];
  CvPoint2D32f* cornersB = new CvPoint2D32f[n_corners];

  // handling strange correspondences
  for (int k = 0; k < n_corners; k++)
  {
    cornersA[k] = crnrsA[k];
    cornersB[k] = crnrsB[k];
    if ((cornersA[k].x < 0) || (cornersA[k].x > 500)) cornersA[k].x = 0;
    if ((cornersA[k].y < 0) || (cornersA[k].y > 500)) cornersA[k].y = 0;
    if ((cornersB[k].x < 0) || (cornersB[k].x > 500)) cornersB[k].x = 0;
    if ((cornersB[k].y < 0) || (cornersB[k].y > 500)) cornersB[k].y = 0;
    if (fabs(cornersA[k].x - cornersB[k].x) > 50)
    {
      cornersA[k].x = 0;
      cornersB[k].x = 0;
    }
    if (fabs(cornersA[k].y - cornersB[k].y) > 50)
    {
      cornersA[k].y = 0;
      cornersB[k].y = 0;
    }
  }

  // computing homography from correspondences
  float cAf[n_corners][2];
  float cBf[n_corners][2];

  // a significant number of correspondences are required; otherwise
  // compensation is not activated
  if (n_corners >= 20)
  {
    for (int i = 0; i < n_corners; i++)
    {

      cAf[i][0] = cornersA[i].x;
      cAf[i][1] = cornersA[i].y;
      cBf[i][0] = cornersB[i].x;
      cBf[i][1] = cornersB[i].y;
    }

    CvMat *H = cvCreateMat(3, 3, CV_32F);
    CvMat *cA, *cB, cA_aux, cB_aux;
    cA = cvCreateMat(n_corners, 2, CV_32F);
    cB = cvCreateMat(n_corners, 2, CV_32F);
    cA_aux = cvMat(n_corners, 2, CV_32F, cAf);
    cvConvert(&cA_aux, cA);
    cB_aux = cvMat(n_corners, 2, CV_32F, cBf);
    cvConvert(&cB_aux, cB);

    cvFindHomography(cA, cB, H);

    cv::Mat Homo(H, true);

    // applying the homography matrix to the neural field in order to perform the motion compensation
    // (most of the operations can be removed with a better handling of the types)
    aux80x60x8Ux1C = cv::Scalar(0);    //cvZero(aux80x60x8Ux1C);
    aux80x60x32Fx1C = cv::Scalar(0);    //cvZero(aux80x60x32Fx1C);
    aux160x120x8Ux1C = cv::Scalar(0);    //cvZero(aux160x120x8Ux1C);
    neural_field.convertTo(aux80x60x8Ux1C, aux80x60x8Ux1C.type(), 255.0);  //cvConvertScale(neural_field, aux80x60x8Ux1C, 255, 0);
    cv::resize(aux80x60x8Ux1C, aux160x120x8Ux1C, aux160x120x8Ux1C.size());  //cvResize(aux80x60x8Ux1C, aux160x120x8Ux1C);
    aux80x60x32Fx1C = aux160x120x8Ux1C.clone();    //aux80x60x32Fx1C = cvCloneImage(aux160x120x8Ux1C);
    //cvWarpPerspective(aux160x120x8Ux1C, aux80x60x32Fx1C, H, CV_INTER_LINEAR | CV_WARP_FILL_OUTLIERS );
    cv::warpPerspective(aux160x120x8Ux1C, aux80x60x32Fx1C, Homo, aux80x60x32Fx1C.size(),
        CV_INTER_LINEAR | CV_WARP_FILL_OUTLIERS);
    cv::resize(aux80x60x32Fx1C, aux160x120x8Ux1C, aux160x120x8Ux1C.size());
    cv::resize(aux160x120x8Ux1C, aux80x60x8Ux1C, aux80x60x8Ux1C.size());
    aux80x60x8Ux1C.convertTo(neural_field, neural_field.type(), 1.0 / 255.0);  //cvConvertScale( aux80x60x8Ux1C, neural_field, 1./255., 0 );
    cvReleaseMat(&H);
    cvReleaseMat(&cA);
    cvReleaseMat(&cB);
  }

  // - visualization stuff ---------
  neural_field.convertTo(neuralField8U, neuralField8U.type(), 255, 0);

  delete[] cornersA;
  delete[] cornersB;
  delete[] crnrsA;
  delete[] crnrsB;

}

//
int AntSystem::OpticalFlowLK(cv::Mat& imgA, cv::Mat& imgB, cv::Mat& imgC,
    CvPoint2D32f* cornersA, CvPoint2D32f* cornersB, int maxCorners)
{

  CvSize img_sz;
  img_sz.width = imgA.cols;  //cvGetSize( imgA );
  img_sz.height = imgA.rows;
  int win_size = 10;
  int n_corners = 0;
  CvPoint2D32f* crnrsA = new CvPoint2D32f[maxCorners];
  //std::vector<cv::Point2f> crnrsA;
  CvPoint2D32f* crnrsB = new CvPoint2D32f[maxCorners];
  //std::vector<cv::Point2f> crnrsB;

  // features to track
  IplImage* eig_image = cvCreateImage(img_sz, IPL_DEPTH_32F, 1);
  IplImage* tmp_image = cvCreateImage(img_sz, IPL_DEPTH_32F, 1);
  int corner_count = maxCorners;

  IplImage ipl_imgA = imgA;
  IplImage ipl_imgB = imgB;

  cvGoodFeaturesToTrack(&ipl_imgA, eig_image, tmp_image, crnrsA, &corner_count, 0.01, 5.0, 0, 3, 0, 0.04);

  cvFindCornerSubPix(&ipl_imgA, crnrsA, corner_count, cvSize(win_size, win_size),
      cvSize(-1, -1), cvTermCriteria(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, 0.03));

  // Call the Lucas Kanade algorithm
  char features_found[maxCorners];
  float feature_errors[maxCorners];

  CvSize pyr_sz;
  pyr_sz.width = imgA.cols + 8;
  pyr_sz.height = imgB.rows / 3;
  IplImage* pyrA = cvCreateImage(pyr_sz, IPL_DEPTH_32F, 1);
  IplImage* pyrB = cvCreateImage(pyr_sz, IPL_DEPTH_32F, 1);

  cvCalcOpticalFlowPyrLK(&ipl_imgA, &ipl_imgB, pyrA, pyrB, crnrsA, crnrsB, corner_count,
      cvSize(win_size, win_size), 5, features_found, feature_errors,
      cvTermCriteria( CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, 0.1), 0);

  for (int i = 0; i < corner_count; i++)
  {

    if (features_found[i] != 0 && feature_errors[i] < 550)
    {
      cv::Point p0 = cv::Point(cvRound(crnrsA[i].x), cvRound(crnrsA[i].y));
      cv::Point p1 = cv::Point(cvRound(crnrsB[i].x), cvRound(crnrsB[i].y));
      cv::line(imgC, p0, p1, CV_RGB(255, 255, 255));
      cornersA[n_corners] = crnrsA[i];
      cornersB[n_corners] = crnrsB[i];
      n_corners++;
    }
  }

  cvReleaseImage(&eig_image);
  cvReleaseImage(&tmp_image);
  cvReleaseImage(&pyrA);
  cvReleaseImage(&pyrB);
  delete[] crnrsA;
  delete[] crnrsB;

  return n_corners;
}

/**
 * TODO andre
 */
void AntSystem::applyMask(const cv::Mat& in, const cv::Mat& mask, cv::Mat& out)
{

  in.convertTo(in, CV_8UC1);
  //cv::medianBlur(mask, mask, 5); // TODO
  cv::Mat temp = cv::Mat(in.size(), in.type());
  if (in.type() == CV_8UC1 && mask.type() == CV_8UC1 && out.type() == CV_8UC1)
  {
    for (int i = 0; i < mask.size().height; i++)
    {
      for (int j = 0; j < mask.size().width; j++)
      {
        // TODO ANDRE AQUI test
        //if (mask.at<uchar>(i, j) > 0)
        if (mask.at<uchar>(i, j) > 0) // old 25
        {
          temp.at<uchar>(i, j) = in.at<uchar>(i, j) * 0.5; // old 0.7
        }
        else
        {
          temp.at<uchar>(i, j) = in.at<uchar>(i, j);
        }
      }
    }
  }
  temp.copyTo(out);
}

const cv::Mat& AntSystem::getNeuralField()
{
  return neuralField8U;
}

const cv::Mat& AntSystem::getTDSalMap()
{
  return saliency.getTDSalMap();
}

const cv::Mat& AntSystem::getBUSalMap()
{
  return saliency.getBUSalMap();
}

const cv::Mat& AntSystem::getTrailProbMap()
{
  return analyzer.getTrailProbMap();
}

const cv::Mat& AntSystem::getPheromoneMapI()
{
  return pheromoneMapI;
}

const cv::Mat& AntSystem::getPheromoneMapC()
{
  return pheromoneMapC;
}

const cv::Mat& AntSystem::getPheromoneMapCI()
{
  return pheromoneMapCI;
}

const cv::Mat& AntSystem::getTrailMapI()
{
  return trailModelMapI;
}

const cv::Mat& AntSystem::getTrailMapC()
{
  return trailModelMapC;
}

const cv::Mat& AntSystem::getTDConspCMap()
{
  return saliency.getTDConspC();
}

const cv::Mat& AntSystem::getTDConspIMap()
{
  return saliency.getTDConspI();
}

const cv::Mat& AntSystem::getBUConspCMap()
{
  return saliency.getBUConspC();
}

const cv::Mat& AntSystem::getBUConspIMap()
{
  return saliency.getBUConspI();
}

const cv::Mat& AntSystem::getSampleMask()
{
  return analyzer.getSampleMask();
}

const cv::Mat& AntSystem::getSeedMap()
{
  return analyzer.getSeedMap();
}
