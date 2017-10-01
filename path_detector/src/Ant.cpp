#include "path_detector/Ant.h"

// 50 in the paper - 30 better for less tilted cameras
Ant::Ant() :
    pheromone(/*3.0*/2.0), maxheight(25/*40*/), maxhops(50),
        hops(0)
{
  //default beahviour's weight
  bWeight[Greedy] = 0.45;
  bWeight[Track] = 0.35;
  bWeight[Ahead] = 0.05;
  bWeight[Commit] = 0.05;
  bWeight[Centre] = 0.1;
  bWeight[Random] = /*0.1*/0.0;  //doesn't affect ant's behaviour anymore...
  std::srand((unsigned) time(0));  //new seed
  useLSDMask = false;
  reward = 0.0;
}
;

Ant::~Ant()
{

  path.clear();
}

void Ant::reset()
{

  pheromone = 2.0;
  maxheight = 30;  // TODO - af-silva inclinação
  maxhops = 50;   // TODo - af-silva inclinação
  hops = 0;
  bWeight[Greedy] = 0.45;
  bWeight[Track] = 0.35;
  bWeight[Ahead] = 0.05;
  bWeight[Commit] = 0.05;
  bWeight[Centre] = 0.1;
  bWeight[Random] = 0.0;
  std::srand((unsigned) time(0));  //new seed
}

void Ant::init(const cv::Point startPoint)
{
  useLSDMask = false;
  launch.x = startPoint.x;
  launch.y = startPoint.y;
  path.clear();
  hops = 0;
  reward = 0.0;
}

void Ant::run(const cv::Mat& conspMap, const cv::Mat& pheroMap, const cv::Mat &lsdMask)
{

  reward = 0.0;  // TODO - andre - reward = -1
  // variable to store the previous selected p-ant's action (for commit behavior)
  int prev = 10;
  // variables to represent the average conspicuity, pheromone during p-ants motion
  int meanC = 0, meanPh = 0;
  // other aux variables
  int resmean[3] = { 0 };
  cv::Point centroid;
  float center = 0, centerwidth = 0, centerdist = 0;

  // random factor in p-ant's behavior
  float rand_factor = 0.4;

  // setting the initial p-ant's position
  int x = launch.x;
  int y = launch.y;

  uchar* imgPtr = (uchar*) (conspMap.data + y * conspMap.step);
  uchar* phPtr = (uchar*) (pheroMap.data + y * pheroMap.step);
  //uchar* rewPtr = (uchar*) (rewardMap.data + y * rewardMap.step);

  int obs = 0;

  // run the p-ant for a maximum number of hops
  for (int j = 0; j < maxhops; j++)
  {

    int conspLevel = imgPtr[x];
    int pheroLevel = phPtr[x];
    //int backlevel = rewPtr[x];

    reward += conspLevel * (60 - y) / maxheight;

    // centroid calculation, for the centre behaviour
    // search for an edge on one direction
    int dev = 50/*15*/;
    for (int k = x; k > 0; k--)
    {
      if ((imgPtr[k] >= conspLevel + dev) || (imgPtr[k] <= conspLevel - dev) || (k == 2))
      {
        centroid.x = k;
        break;
      }
    }
    // search for an edge on the other direction
    for (int k = x; k < conspMap.cols - 1; k++)
    {
      if ((imgPtr[k] >= conspLevel + dev) || (imgPtr[k] <= conspLevel - dev) || (k == conspMap.cols - 2))
      {
        centroid.y = k;
        break;
      }
    }
    // compute centroid
    centerwidth = float(centroid.y) - float(centroid.x);
    centerdist = float(centroid.y) - (float(centerwidth) / 2.) - float(x);
    //center = centerdist / centerwidth;
    center = centerdist / /*centerwidth;*/(float(centerwidth) / 2.);
    //trailWidth.push_back(centerwidth);

    // neighbour table construction
    findNeighbours(cv::Point(x, y), neigs, conspMap, pheroMap);

    // storing current ant's location for subsequent processing
    path.push_back(cv::Point(x, y));

    // new stop condition -- abrupt changes on conspicuity
    //if ( abs(resmean[1] - meanC) > 80 )
    //  break;

    if (hops == 0)
    {
      meanC = conspLevel;
      meanPh = pheroLevel;
    }
    else
    {
      meanC = (int) ((meanC * (1.0 - 1.0 / float(hops))) + (resmean[1] * (1.0 / float(hops))));
      meanPh = (int) ((meanPh * (1.0 - 1.0 / float(hops))) + (resmean[2] * (1.0 / float(hops))));
      //meanC = (int) ((meanC*0.8/*(1.0-1.0/float(hops))*/)+(resmean[1]*0.2/*(1.0/float(hops))*/));
      //meanPh = (int) ((meanPh*0.8/*(1.0-1.0/float(hops))*/)+(resmean[2]*0.2/*(1.0/float(hops))*/));
    }

    // selecting the best action, using p-ant's behaviors
    selectBehaviour(neigs, prev, meanC, /*meanPh,*/center, rand_factor, resmean);
    int best = resmean[0];
    prev = best;

    // stop conditions: stop if out of the operation range or already moved enough
    int distance = (int) sqrt(
        pow(double(launch.x - neigs[best].coord.x), 2.) + pow(double(launch.y - neigs[best].coord.y), 2.));
    if ((neigs[best].coord.x > conspMap.cols) || (neigs[best].coord.x < 0) || (distance > maxheight))
      break;
    if (y < 1)
      break;


    // update p-ant's position based on selected action
    x = neigs[best].coord.x;
    y = neigs[best].coord.y;

    // verificar se a coord x/y esta na mascara e poem o reward = 0
    // last value => 25 |
    if(useLSDMask && lsdMask.at<uchar>(cv::Point(x,y)) > 0)
    {
      obs++; // number of times that the p-ant past an obstacle
    }

    imgPtr = (uchar*) (conspMap.data + y * conspMap.step);
    phPtr = (uchar*) (pheroMap.data + y * pheroMap.step);

    // random factor decreases as p-ant reaches the max number of iterations
    rand_factor -= rand_factor * 0.002;

    hops++;
  }
  // store execution state to enable the subsequent pheromone dropping process;
  //bproj_mean = bproj_sum / hops;
  reward = reward / hops;


  // y=1-exp(-7*x)
  // sigmoide
  if(useLSDMask)
  {
    cv::imshow("Ants Pheromona Mask",lsdMask);
    // no pior dos casos o numero de hops e 50
    reward = sqrt(( 1.0 - (double)obs / (double)hops)) * reward; // TODO ANDRE
  }
}

float Ant::getReward()
{
  return reward;
}

void Ant::setUseLSDMaks(bool useMask)
{
  useLSDMask = useMask;
}

void Ant::findNeighbours(cv::Point coord, Neighbour neighs[], const cv::Mat& conspMap, const cv::Mat& pheroMap)
{

  int n = -6;
  uchar* imgptr = (uchar*) (conspMap.data + coord.y * conspMap.step);
  uchar* phptr = (uchar*) (pheroMap.data + coord.y * pheroMap.step);

  for (int i = 0; i < 6; i++)
  {

    neighs[i].coord.y = coord.y;
    neighs[i].coord.x = coord.x + n;
    neighs[i].consp = imgptr[coord.x + n];
    neighs[i].phero = phptr[coord.x + n];
    n += 2;

    if (i == 2)
      n += 2;
  }

  n = -4;
  coord.y -= 2;
  imgptr = (uchar*) (conspMap.data + coord.y * conspMap.step);
  phptr = (uchar*) (pheroMap.data + coord.y * pheroMap.step);

  for (int i = 6; i < 10; i++)
  {

    neighs[i].coord.y = coord.y;
    neighs[i].coord.x = coord.x + n;
    neighs[i].consp = imgptr[coord.x + n];
    neighs[i].phero = phptr[coord.x + n];
    n += 2;

    if (i == 7)
      n += 2;
  }

  neighs[10].coord.y = coord.y;
  neighs[10].coord.x = coord.x;
  neighs[10].consp = imgptr[coord.x];
  neighs[10].phero = phptr[coord.x];

  coord.y -= 2;
  imgptr = (uchar*) (conspMap.data + coord.y * conspMap.step);
  phptr = (uchar*) (pheroMap.data + coord.y * pheroMap.step);

  neighs[11].coord.y = coord.y;
  neighs[11].coord.x = coord.x;
  neighs[11].consp = imgptr[coord.x];
  neighs[11].phero = phptr[coord.x];

}

// TODO - andre - afectar os comportamentos das formigas com base na mascara
void Ant::selectBehaviour(Neighbour neighs[], int prev, int meanC, float center, float rand_factor, int resmean[])
{

  // mean intensity for the regions surrounding the pixel
  float leftC = float((neighs[0].consp + neighs[1].consp + neighs[2].consp) / 3);
  float rightC = float((neighs[3].consp + neighs[4].consp + neighs[5].consp) / 3);
  float upleftC = float((neighs[6].consp + neighs[7].consp) / 2);
  float uprightC = float((neighs[8].consp + neighs[9].consp) / 2);
  float upC = float((neighs[10].consp + neighs[11].consp) / 2);

  // mean pheromone for the regions surrounding the pixel
  float leftPh = float((neighs[0].phero + neighs[1].phero + neighs[2].phero) / 3);
  float rightPh = float((neighs[3].phero + neighs[4].phero + neighs[5].phero) / 3);
  float upleftPh = float((neighs[6].phero + neighs[7].phero) / 2);
  float uprightPh = float((neighs[8].phero + neighs[9].phero) / 2);
  float upPh = float((neighs[10].phero + neighs[11].phero) / 2);

  // structures with mean region pheromone and conspicuity levels
  // (these are the percepts of the ant's behaviors - see below)
  float regionsC[5] = { leftC, upleftC, upC, uprightC, rightC };
  float regionsPh[5] = { leftPh, upleftPh, upPh, uprightPh, rightPh };

  // variables with the average of all regions
  float meanRegC = 0;
  float meanRegPh = 0;
  for (int i = 0; i < 5; i++)
  {
    meanRegC += regionsC[i];
    meanRegPh += regionsPh[i];
  }
  meanRegC = meanRegC / 5.;
  meanRegPh = meanRegPh / 5.;

  // TODO - perguntar ao Pedro
  // fcentre action preferences
  // - preference is higher in the direction of the centroid (attraction towards it)
  /*float c_dist[5], c = 1;
   if (center < 0){
   for (int i=0; i<5; i++) {
   c_dist[i] = c * fabs(center)*10.;
   c-= 0.2;
   }
   } else {
   for (int i=4; i>=0; i--) {
   c_dist[i] = c * fabs(center)*10.;
   c-= 0.2;
   }
   }*/

  float c_dist[5], c = 1;
  if (center < /*0*/-0.05)
  {
    for (int i = 0; i < 5; i++)
    {
      c_dist[i] = c * fabs(center)/**10.*/;
      c -= 0.2;
    }
  }
  else if ((center >= -0.05) && (center <= 0.05))
  {
    c_dist[0] = 0.2;
    c_dist[1] = 0.8;
    c_dist[2] = 1.0;
    c_dist[3] = 0.8;
    c_dist[4] = 0.2;
  }
  else
  {
    for (int i = 4; i >= 0; i--)
    {
      c_dist[i] = c * fabs(center)/**10.*/;
      c -= 0.2;
    }
  }

  // for the action taken in the previous iteration, transform from action index (prev)
  // to pixel index (prev_dir): remember that an action is associated to a region,
  // which is in turn associated to a set of pixels.
  float prev_dir = 0;
  switch (prev)
  {
    case 2:
      prev_dir = 1;
      break;
    case 3:
      prev_dir = 5;
      break;
    case 7:
      prev_dir = 2;
      break;
    case 8:
      prev_dir = 4;
      break;
    case 10:
      prev_dir = 3;
      break;
  }

  // going over all actions to select the one with highest score
  float bestScore = 0;
  int bestAction = 0;

  for (int i = 0; i < 5; i++)
  {
    // track behavior vote for action i
    float dif = (255. - fabs(float(meanC) - regionsC[i])) / 255.;
    // greedy behavior vote for action i
    float color = regionsC[i] / 255.;
    // commit behavior vote for action i
    float dir = 1. - (fabs(float(i + 1) - prev_dir) / 4.);
    // ahead behavior vote for action i
    float dir2 = 1. - (fabs(float(i + 1) - 3.) / 2.);
    // random behavior vote for action i
    float rand_dir = float((std::rand() % 101) / 100.0);  //*/ + 0.6*dir);
    //std::cout << rand_dir << std::endl;

    // fusing all votes with associated weights
    float scoreC = bWeight[Track] * dif + bWeight[Greedy] * color + bWeight[Commit] * dir
        + bWeight[Ahead] * dir2 + bWeight[Centre] * c_dist[i] + rand_factor/*bWeight[Random]*/* rand_dir;

    // getting pheromone contribution
    float scorePh = regionsPh[i] / 255.;

    // blending votes with pheromone
    float score = 0.5 * scoreC + 0.5 * scorePh;

    // check if action i is better than the previously one analysed
    if (score >= bestScore)
    {
      bestAction = i;
      bestScore = score;
    }

  }

  //if a deadlock is detected move randomly
  if ((bestAction == 0 && prev_dir == 5) || (bestAction == 4 && prev_dir == 1))
    bestAction = rand() % 4;

  // for the best action, transform from action index (prev)
  // to pixel index (prev_dir): remember that an action is associated to a region,
  // which is in turn associated to a set of pixels.
  float result;
  switch (bestAction)
  {
    case 0:
      result = 2;
      break;
    case 1:
      result = 7;
      break;
    case 2:
      result = 10;
      break;
    case 3:
      result = 8;
      break;
    case 4:
      result = 3;
      break;
  }

  resmean[0] = result;
  resmean[1] = (int) meanRegC;
  resmean[2] = (int) meanRegPh;

  return;

}

void Ant::dropPheromone(cv::Mat& pheromoneMap, cv::Mat &ph, cv::Mat &phinf)
{

#ifdef NO_TOPDOWN
  reward = 0;  //BYPASS
#endif

  float deploy_pheromone = reward * 2.0 / 255.0 + pheromone;

  // across visual features influence
  float inf = 0.3;

  // let us deploy pheromone along the p-ant's executed path
  for (int j = 0; j < hops; j++)
  {

    int x = path[j].x;
    int y = path[j].y;

    uchar* phptr, *pheroptr, *infptr;
    phptr = (uchar*) (ph.data + y * ph.step);
    infptr = (uchar*) (phinf.data + y * phinf.step);
    pheroptr = (uchar*) (pheromoneMap.data + y * pheromoneMap.step);

    // pheromone level decays as we move away from the p-ant
    for (int k = -6; k <= 6; k++)
    {

      float decay = fmax(0, (1.0 - 0.15 * fabs(double(k))));

      if (pheroptr[x + k] + deploy_pheromone * decay < 255)
        pheroptr[x + k] += deploy_pheromone * decay;
      else
        pheroptr[x + k] = 255;

      if (phptr[x + k] + deploy_pheromone * decay < 255)
        phptr[x + k] += deploy_pheromone * decay;
      else
        phptr[x + k] = 255;

      if (infptr[x + k] + inf * deploy_pheromone * decay < 255)
        infptr[x + k] += inf * deploy_pheromone * decay;
      else
        infptr[x + k] = 255;
    }
  }
}

void Ant::setBehaviourWeight(Behaviour behaviour, double value)
{

  bWeight[behaviour] = value;
}

void Ant::setMaxHeight(int height)
{
  maxheight = height;
}
