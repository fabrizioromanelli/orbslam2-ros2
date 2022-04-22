#ifndef __FUSER__
#define __FUSER__

#include <iostream>
#include "pose.hpp"
#include "medianFilter.hpp"
#include <unsupported/Eigen/Splines>

#define FILTER_WINDOW   6
#define RECOVERY_BUFFER 6

class Fuser
{
  // Variables
  public:
    enum trackQoS {
      LOST = 0,
      LOW = 1,
      MED = 2,
      OK = 3
    };

    enum fuserState {
      UNINITIALIZED = 0,
      RUNNING = 1
    };

  protected:
    Pose pose;
    Pose posePrev;
    Pose poseFiltered;
    Pose poseFilteredPrev;
    Pose orbPose;
    Pose deltaVO;
    Pose deltaORB;

  private:
    unsigned int recoverSteps;
    double REDUCTION_FACTOR;
    bool recovered;
    bool firstRecover;
    bool medianFilterReady;
    unsigned int fuserStatus;
    unsigned int orbQoS;
    unsigned int camQoS;
    std::vector<double> deltaCamVO;
    std::vector<double> deltaOrbVO;
    Pose camVOPrev;
    Pose orbVOPrev;
    Pose firstCamVO;
    Pose camRecover;
    double alphaBlending; // Fuser blending coefficient
    double alphaWeight;   // Fuser weight coefficient
    std::vector<Pose> orbPoseBuffer;
    std::vector<Pose> poseBuffer;
    std::vector<unsigned int> orbQoSPrev, orbQoSFilterReset;
    unsigned int counter;

  // Methods
  public:
    Fuser();
    ~Fuser();
    void synchronizer(double, double, double, Pose, Pose, Pose&);
    bool fuse(Pose, Pose);
    Pose getFusedPose();

    Pose getOrbPose();
    Pose getdeltaVOPose();
    Pose getdeltaORBPose();

  protected:

  private:
    void sensorFusion(std::vector<double> &, std::vector<double> &);

};

#endif // __FUSER__