#include "fuser.hpp"

class SplineInterpolator {
  public:
    SplineInterpolator(Eigen::Vector2d const &x_vec, Eigen::Vector2d const &y_vec): x_min(x_vec.minCoeff()), x_max(x_vec.maxCoeff()),
      spline_(Eigen::SplineFitting<Eigen::Spline<double, 1>>::Interpolate(y_vec.transpose(), std::min<int>(x_vec.rows() - 1, 3), scaled_values(x_vec)))
    {}

    double operator()(double x) const {
      return spline_(scaled_value(x))(0);
    }

  private:
    double scaled_value(double x) const {
      return (x - x_min) / (x_max - x_min);
    }

    Eigen::RowVectorXd scaled_values(Eigen::VectorXd const &x_vec) const {
      return x_vec.unaryExpr([this](double x) { return scaled_value(x); }).transpose();
    }

    double x_min;
    double x_max;

    Eigen::Spline<double, 1> spline_;
};

Fuser::Fuser(): REDUCTION_FACTOR(0.01), recovered(false), firstRecover(true), medianFilterReady(false), fuserStatus(UNINITIALIZED), orbQoS(LOST), camQoS(LOST), alphaBlending(0.75), alphaWeight(0.7), counter(0)
{
  recoverSteps = FILTER_WINDOW + 1;
  deltaCamVO.reserve(pose.getPoseElements());
  deltaOrbVO.reserve(pose.getPoseElements());
  pose.setTranslation(0.0,0.0,0.0);
  pose.setRotation(1.0,0.0,0.0,0.0);
  posePrev.setTranslation(0.0,0.0,0.0);
  posePrev.setRotation(1.0,0.0,0.0,0.0);
  orbQoSPrev.reserve(RECOVERY_BUFFER);
  orbQoSFilterReset.reserve(FILTER_WINDOW);
}

Fuser::~Fuser()
{}

// This function synchronizes a sample s3 on the basis of two
// previous samples (s1, s2) at timestep timestep3.
void Fuser::synchronizer(double timestep1, double timestep2, double timestep3, Pose s1, Pose s2, Pose & s3)
{
  // This is used to initialize
  if (timestep1 == -1)
    s3 = s2;
  else if (timestep3 < timestep1)
    s3 = s1;
  else if ((timestep3 < timestep2) || (timestep1 == timestep2))
    s3 = s2;
  else {
    Eigen::Vector2d tvals = {timestep1, timestep2};
    Eigen::Vector2d Xvals = {s1.getTranslation()(0), s2.getTranslation()(0)};
    Eigen::Vector2d Yvals = {s1.getTranslation()(1), s2.getTranslation()(1)};
    Eigen::Vector2d Zvals = {s1.getTranslation()(2), s2.getTranslation()(2)};
    SplineInterpolator sX(tvals, Xvals), sY(tvals, Yvals), sZ(tvals, Zvals);

    s3.setTranslation(sX(timestep3), sY(timestep3), sZ(timestep3));

    if (s1.getRotation().w() != 0.0 && s2.getRotation().w() != 0.0 && s1.getRotation().x() != 0.0 && s2.getRotation().x() != 0.0 && s1.getRotation().y() != 0.0 && s2.getRotation().y() != 0.0 && s1.getRotation().z() != 0.0 && s2.getRotation().z() != 0.0)
    {
      Eigen::Vector2d qwvals = {s1.getRotation().w(), s2.getRotation().w()};
      Eigen::Vector2d qxvals = {s1.getRotation().x(), s2.getRotation().x()};
      Eigen::Vector2d qyvals = {s1.getRotation().y(), s2.getRotation().y()};
      Eigen::Vector2d qzvals = {s1.getRotation().z(), s2.getRotation().z()};
      SplineInterpolator sQW(tvals, qwvals), sQX(tvals, qxvals), sQY(tvals, qyvals), sQZ(tvals, qzvals);
      Eigen::Quaterniond tmpRot = {sQW(timestep3), sQX(timestep3), sQY(timestep3), sQZ(timestep3)};
      s3.setRotation(tmpRot);
    }
  }

  return;
}

Pose Fuser::getFusedPose()
{
  return(poseFiltered);
}

// Debugging purpose only
Pose Fuser::getOrbPose()
{
  return(orbPose);
}

Pose Fuser::getdeltaVOPose()
{
  return(deltaVO);
}

Pose Fuser::getdeltaORBPose()
{
  return(deltaORB);
}
// ...Debugging purpose only

bool Fuser::fuse(Pose camVO, Pose orbVO)
{
  Pose _orbPose          = orbVO;
  camQoS                 = camVO.getAccuracy();
  orbQoS                 = orbVO.getAccuracy();
  unsigned int orbQoSNow = orbVO.getAccuracy();

  if (fuserStatus == UNINITIALIZED) {
    firstCamVO  = camVO;
  }

  // Recover after an ORB fault, must re-initialize the ORB trajectory with
  // the corrected roto-translation.
  if (counter > RECOVERY_BUFFER) {
    if ((orbQoS == OK) && (orbQoSPrev[0] == LOST)) {
      unsigned int okCounter = 0;
      for (unsigned int i = 1; i < RECOVERY_BUFFER; i++) {
        if (orbQoSPrev[i] == OK) {
          okCounter++;
        }
      }

      if (okCounter == RECOVERY_BUFFER - 1) {
        recovered = true;
        firstRecover = true;
        camRecover = poseFilteredPrev;
        std::cout << "ORBSLAM2 recovered @ " << camRecover.getTranslation()[Pose::X] << " " << camRecover.getTranslation()[Pose::Y] << " " << camRecover.getTranslation()[Pose::Z] << " " << camRecover.getRotation().w() << " " << camRecover.getRotation().x() << " " << camRecover.getRotation().y() << " " << camRecover.getRotation().z() <<std::endl;
      }
    }

    // This prevents including wrong ORB measurements before computing the
    // corrected ORB trajectory.
    for (unsigned int i = 1; i < RECOVERY_BUFFER; i++) {
      if (orbQoSPrev[i] == LOST) {
        orbQoS = LOST;
        recovered = false;
        break;
      }
    }
  }

  // Recovering orb Pose
  if (recovered) {
    _orbPose.rotoTranslation(camRecover.getTranslation(), camRecover.getRotation());

    if (firstRecover) {
      orbVOPrev.rotoTranslation(camRecover.getTranslation(), camRecover.getRotation());
      firstRecover = false;
      recoverSteps = 0;
    }

    recoverSteps++;
  } else if (orbQoS != LOST) {
    _orbPose.rotoTranslation(firstCamVO.getTranslation(), firstCamVO.getRotation());
  }

  // Filtering orb Pose
  if (medianFilterReady) {
    // Filtering orb spikes with median filter
    std::vector<MedianFilter<double, FILTER_WINDOW>> orbFilters(pose.getPoseElements());

    for (unsigned int j = 0; j < FILTER_WINDOW - 1; j++) {
      orbPoseBuffer[j] = orbPoseBuffer[j+1];

      // Fill the median filters buffers
      orbFilters[Pose::X].addSample(orbPoseBuffer[j].getTranslation()[Pose::X]);
      orbFilters[Pose::Y].addSample(orbPoseBuffer[j].getTranslation()[Pose::Y]);
      orbFilters[Pose::Z].addSample(orbPoseBuffer[j].getTranslation()[Pose::Z]);
      orbFilters[Pose::WQ].addSample(orbPoseBuffer[j].getRotation().w());
      orbFilters[Pose::XQ].addSample(orbPoseBuffer[j].getRotation().x());
      orbFilters[Pose::YQ].addSample(orbPoseBuffer[j].getRotation().y());
      orbFilters[Pose::ZQ].addSample(orbPoseBuffer[j].getRotation().z());
    }
    orbPoseBuffer[FILTER_WINDOW-1] = _orbPose;
    orbFilters[Pose::X].addSample(orbPoseBuffer[FILTER_WINDOW-1].getTranslation()[Pose::X]);
    orbFilters[Pose::Y].addSample(orbPoseBuffer[FILTER_WINDOW-1].getTranslation()[Pose::Y]);
    orbFilters[Pose::Z].addSample(orbPoseBuffer[FILTER_WINDOW-1].getTranslation()[Pose::Z]);
    orbFilters[Pose::WQ].addSample(orbPoseBuffer[FILTER_WINDOW-1].getRotation().w());
    orbFilters[Pose::XQ].addSample(orbPoseBuffer[FILTER_WINDOW-1].getRotation().x());
    orbFilters[Pose::YQ].addSample(orbPoseBuffer[FILTER_WINDOW-1].getRotation().y());
    orbFilters[Pose::ZQ].addSample(orbPoseBuffer[FILTER_WINDOW-1].getRotation().z());

    _orbPose.setTranslation(orbFilters[Pose::X].getMedian(), orbFilters[Pose::Y].getMedian(), orbFilters[Pose::Z].getMedian());
    Eigen::Quaterniond tmpQuat = {orbFilters[Pose::WQ].getMedian(), orbFilters[Pose::XQ].getMedian(), orbFilters[Pose::YQ].getMedian(), orbFilters[Pose::ZQ].getMedian()};
    _orbPose.setRotation(tmpQuat);
  } else {
    orbPoseBuffer.push_back(_orbPose);
    if (orbPoseBuffer.size() == FILTER_WINDOW)
      medianFilterReady = true;
  }

  orbVO = _orbPose;

  if (fuserStatus == UNINITIALIZED) {
    deltaCamVO[Pose::X]  = camVO.getTranslation()[Pose::X];
    deltaCamVO[Pose::Y]  = camVO.getTranslation()[Pose::Y];
    deltaCamVO[Pose::Z]  = camVO.getTranslation()[Pose::Z];
    deltaCamVO[Pose::WQ] = camVO.getRotation().w() - 1.0;
    deltaCamVO[Pose::XQ] = camVO.getRotation().x();
    deltaCamVO[Pose::YQ] = camVO.getRotation().y();
    deltaCamVO[Pose::ZQ] = camVO.getRotation().z();

    deltaOrbVO[Pose::X]  = orbVO.getTranslation()[Pose::X];
    deltaOrbVO[Pose::Y]  = orbVO.getTranslation()[Pose::Y];
    deltaOrbVO[Pose::Z]  = orbVO.getTranslation()[Pose::Z];
    deltaOrbVO[Pose::WQ] = orbVO.getRotation().w() - 1.0;
    deltaOrbVO[Pose::XQ] = orbVO.getRotation().x();
    deltaOrbVO[Pose::YQ] = orbVO.getRotation().y();
    deltaOrbVO[Pose::ZQ] = orbVO.getRotation().z();
  } else {
    deltaCamVO[Pose::X]  = camVO.getTranslation()[Pose::X] - camVOPrev.getTranslation()[Pose::X];
    deltaCamVO[Pose::Y]  = camVO.getTranslation()[Pose::Y] - camVOPrev.getTranslation()[Pose::Y];
    deltaCamVO[Pose::Z]  = camVO.getTranslation()[Pose::Z] - camVOPrev.getTranslation()[Pose::Z];
    deltaCamVO[Pose::WQ] = camVO.getRotation().w() - camVOPrev.getRotation().w();
    deltaCamVO[Pose::XQ] = camVO.getRotation().x() - camVOPrev.getRotation().x();
    deltaCamVO[Pose::YQ] = camVO.getRotation().y() - camVOPrev.getRotation().y();
    deltaCamVO[Pose::ZQ] = camVO.getRotation().z() - camVOPrev.getRotation().z();

    deltaOrbVO[Pose::X]  = orbVO.getTranslation()[Pose::X] - orbVOPrev.getTranslation()[Pose::X];
    deltaOrbVO[Pose::Y]  = orbVO.getTranslation()[Pose::Y] - orbVOPrev.getTranslation()[Pose::Y];
    deltaOrbVO[Pose::Z]  = orbVO.getTranslation()[Pose::Z] - orbVOPrev.getTranslation()[Pose::Z];
    deltaOrbVO[Pose::WQ] = orbVO.getRotation().w() - orbVOPrev.getRotation().w();
    deltaOrbVO[Pose::XQ] = orbVO.getRotation().x() - orbVOPrev.getRotation().x();
    deltaOrbVO[Pose::YQ] = orbVO.getRotation().y() - orbVOPrev.getRotation().y();
    deltaOrbVO[Pose::ZQ] = orbVO.getRotation().z() - orbVOPrev.getRotation().z();
  }

  sensorFusion(deltaCamVO, deltaOrbVO);

  // Filtering fused Pose
  if (medianFilterReady && recoverSteps > FILTER_WINDOW) {
    // Filtering fused pose with median filter
    std::vector<MedianFilter<double, FILTER_WINDOW>> poseFilters(pose.getPoseElements());

    for (unsigned int j = 0; j < FILTER_WINDOW - 1; j++) {
      poseBuffer[j] = poseBuffer[j+1];

      // Fill the median filters buffers
      poseFilters[Pose::X].addSample(poseBuffer[j].getTranslation()[Pose::X]);
      poseFilters[Pose::Y].addSample(poseBuffer[j].getTranslation()[Pose::Y]);
      poseFilters[Pose::Z].addSample(poseBuffer[j].getTranslation()[Pose::Z]);
      poseFilters[Pose::WQ].addSample(poseBuffer[j].getRotation().w());
      poseFilters[Pose::XQ].addSample(poseBuffer[j].getRotation().x());
      poseFilters[Pose::YQ].addSample(poseBuffer[j].getRotation().y());
      poseFilters[Pose::ZQ].addSample(poseBuffer[j].getRotation().z());
    }
    poseBuffer[FILTER_WINDOW-1] = pose;
    poseFilters[Pose::X].addSample(poseBuffer[FILTER_WINDOW-1].getTranslation()[Pose::X]);
    poseFilters[Pose::Y].addSample(poseBuffer[FILTER_WINDOW-1].getTranslation()[Pose::Y]);
    poseFilters[Pose::Z].addSample(poseBuffer[FILTER_WINDOW-1].getTranslation()[Pose::Z]);
    poseFilters[Pose::WQ].addSample(poseBuffer[FILTER_WINDOW-1].getRotation().w());
    poseFilters[Pose::XQ].addSample(poseBuffer[FILTER_WINDOW-1].getRotation().x());
    poseFilters[Pose::YQ].addSample(poseBuffer[FILTER_WINDOW-1].getRotation().y());
    poseFilters[Pose::ZQ].addSample(poseBuffer[FILTER_WINDOW-1].getRotation().z());

    poseFiltered.setTranslation(poseFilters[Pose::X].getMedian(), poseFilters[Pose::Y].getMedian(), poseFilters[Pose::Z].getMedian());
    Eigen::Quaterniond tmpQuat = {poseFilters[Pose::WQ].getMedian(), poseFilters[Pose::XQ].getMedian(), poseFilters[Pose::YQ].getMedian(), poseFilters[Pose::ZQ].getMedian()};
    poseFiltered.setRotation(tmpQuat);
  } else {
    if (poseBuffer.size() != FILTER_WINDOW)
      poseBuffer.push_back(pose);
    else {
      for (unsigned int i = 0; i < FILTER_WINDOW - 1; i++)
        poseBuffer[i] = poseBuffer[i+1];

      poseBuffer[FILTER_WINDOW - 1] = pose;
    }
    poseFiltered = pose;
    if (poseBuffer.size() == FILTER_WINDOW)
      medianFilterReady = true;

    // Smoothing trajectory during filter reset phase (for FILTER_WINDOW steps).
    if (counter > FILTER_WINDOW) {
      if (orbQoSFilterReset[0] != LOST) {
        double _x = poseFilteredPrev.getTranslation()[Pose::X] + (poseBuffer[poseBuffer.size()-2].getTranslation()[Pose::X] - pose.getTranslation()[Pose::X])*REDUCTION_FACTOR;
        double _y = poseFilteredPrev.getTranslation()[Pose::Y] + (poseBuffer[poseBuffer.size()-2].getTranslation()[Pose::Y] - pose.getTranslation()[Pose::Y])*REDUCTION_FACTOR;
        double _z = poseFilteredPrev.getTranslation()[Pose::Z] + (poseBuffer[poseBuffer.size()-2].getTranslation()[Pose::Z] - pose.getTranslation()[Pose::Z])*REDUCTION_FACTOR;

        poseFiltered.setTranslation(_x, _y, _z);

        double _wq = poseFilteredPrev.getRotation().w() + (poseBuffer[poseBuffer.size()-2].getRotation().w() - pose.getRotation().w())*REDUCTION_FACTOR;
        double _xq = poseFilteredPrev.getRotation().x() + (poseBuffer[poseBuffer.size()-2].getRotation().x() - pose.getRotation().x())*REDUCTION_FACTOR;
        double _yq = poseFilteredPrev.getRotation().y() + (poseBuffer[poseBuffer.size()-2].getRotation().y() - pose.getRotation().y())*REDUCTION_FACTOR;
        double _zq = poseFilteredPrev.getRotation().z() + (poseBuffer[poseBuffer.size()-2].getRotation().z() - pose.getRotation().z())*REDUCTION_FACTOR;

        poseFiltered.setRotation(_wq, _xq, _yq, _zq);
      } else {
        poseFiltered = poseFilteredPrev;
      }
    }
  }

  // Saving previous cam and orb VO poses
  camVOPrev        = camVO;
  orbVOPrev        = orbVO;
  posePrev         = pose;
  poseFilteredPrev = poseFiltered;

  // Check for NaNs in the backend
  if (std::isnan(camVOPrev.getRotation().w()) || std::isnan(camVOPrev.getRotation().x()) || std::isnan(camVOPrev.getRotation().y()) || std::isnan(camVOPrev.getRotation().z())) {
    std::cerr << "NaN in camVOPrev: " << camVOPrev.getRotation().w() << "," << camVOPrev.getRotation().x() << "," << camVOPrev.getRotation().y() << "," << camVOPrev.getRotation().z() << std::endl;
  }

  if (std::isnan(orbVOPrev.getRotation().w()) || std::isnan(orbVOPrev.getRotation().x()) || std::isnan(orbVOPrev.getRotation().y()) || std::isnan(orbVOPrev.getRotation().z())) {
    std::cerr << "NaN in orbVOPrev: " << orbVOPrev.getRotation().w() << "," << orbVOPrev.getRotation().x() << "," << orbVOPrev.getRotation().y() << "," << orbVOPrev.getRotation().z() << std::endl;
  }

  if (std::isnan(posePrev.getRotation().w()) || std::isnan(posePrev.getRotation().x()) || std::isnan(posePrev.getRotation().y()) || std::isnan(posePrev.getRotation().z())) {
    std::cerr << "NaN in posePrev: " << posePrev.getRotation().w() << "," << posePrev.getRotation().x() << "," << posePrev.getRotation().y() << "," << posePrev.getRotation().z() << std::endl;
  }

  if (std::isnan(poseFilteredPrev.getRotation().w()) || std::isnan(poseFilteredPrev.getRotation().x()) || std::isnan(poseFilteredPrev.getRotation().y()) || std::isnan(poseFilteredPrev.getRotation().z())) {
    std::cerr << "NaN in poseFilteredPrev: " << poseFilteredPrev.getRotation().w() << "," << poseFilteredPrev.getRotation().x() << "," << poseFilteredPrev.getRotation().y() << "," << poseFilteredPrev.getRotation().z() << std::endl;
  }

  // Buffering orb QoS for recovery.
  for (unsigned int i = 0; i < RECOVERY_BUFFER - 1; i++)
    orbQoSPrev[i] = orbQoSPrev[i+1];

  orbQoSPrev[RECOVERY_BUFFER - 1] = orbQoSNow;

  // Buffering orb QoS for filter reset and smoothing.
  for (unsigned int i = 0; i < FILTER_WINDOW - 1; i++)
    orbQoSFilterReset[i] = orbQoSFilterReset[i+1];

  orbQoSFilterReset[FILTER_WINDOW - 1] = orbQoSNow;

  if (fuserStatus == UNINITIALIZED) {
    fuserStatus = RUNNING;
  }

  counter++;

  return(true);
}

// This function fuses ORBSLAM2 with T265 VO.
// This function uses a blending algorithm to fuse ORBSLAM2 with T265 Visual Odometry.
void Fuser::sensorFusion(std::vector<double> & deltaCamVO, std::vector<double> & deltaOrbVO)
{
  double alpha;
  std::vector<double> delta(pose.getPoseElements());

  switch (camQoS)
  {
    case LOST:
      alpha = 0.0;
      break;
    case LOW:
      alpha = 0.0;
      break;
    case MED:
      alpha = alphaBlending * alphaWeight;
      break;
    case OK:
      alpha = alphaBlending;
      break;
    default:
      alpha = 0.0;
      break;
  }

  if (orbQoS == LOST) {
    alpha = 1.0;
  }

  delta[Pose::X]  = deltaCamVO[Pose::X] * alpha + deltaOrbVO[Pose::X]*(1.0 - alpha);
  delta[Pose::Y]  = deltaCamVO[Pose::Y] * alpha + deltaOrbVO[Pose::Y]*(1.0 - alpha);
  delta[Pose::Z]  = deltaCamVO[Pose::Z] * alpha + deltaOrbVO[Pose::Z]*(1.0 - alpha);
  delta[Pose::WQ] = deltaCamVO[Pose::WQ] * alpha + deltaOrbVO[Pose::WQ]*(1.0 - alpha);
  delta[Pose::XQ] = deltaCamVO[Pose::XQ] * alpha + deltaOrbVO[Pose::XQ]*(1.0 - alpha);
  delta[Pose::YQ] = deltaCamVO[Pose::YQ] * alpha + deltaOrbVO[Pose::YQ]*(1.0 - alpha);
  delta[Pose::ZQ] = deltaCamVO[Pose::ZQ] * alpha + deltaOrbVO[Pose::ZQ]*(1.0 - alpha);

  pose.setTranslation(posePrev.getTranslation()[Pose::X] + delta[Pose::X], posePrev.getTranslation()[Pose::Y] + delta[Pose::Y], posePrev.getTranslation()[Pose::Z] + delta[Pose::Z]);
  pose.setRotation(posePrev.getRotation().w() + delta[Pose::WQ], posePrev.getRotation().x() + delta[Pose::XQ], posePrev.getRotation().y() + delta[Pose::YQ], posePrev.getRotation().z() + delta[Pose::ZQ]);

  return;
}