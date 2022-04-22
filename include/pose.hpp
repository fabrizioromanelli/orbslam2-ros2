#ifndef __POSE__
#define __POSE__

#include <Eigen/Dense>

class Pose {
  // Variables
  public:
    enum poseCoords {
      X = 0,
      Y = 1,
      Z = 2,
      WQ = 3,
      XQ = 4,
      YQ = 5,
      ZQ = 6
    };

    enum trackQoS {
      LOST = 0,
      LOW = 1,
      MED = 2,
      OK = 3
    };

  private:
    Eigen::Vector3d translation;
    Eigen::Quaterniond rotation;
    unsigned int poseAccuracy;

  // Methods
  public:
    Pose();
    Pose(Eigen::Vector3d, Eigen::Quaterniond);
    Pose(double, double, double, double, double, double, double);
    ~Pose();
    void setTranslation(double, double, double);
    void setTranslation(Eigen::Vector3d);
    void setRotation(double w, double x, double y, double z);
    void setRotation(Eigen::Quaterniond);
    void getTranslation(Eigen::Vector3d&);
    void getRotation(Eigen::Quaterniond&);
    void setAccuracy(unsigned int);
    unsigned int getAccuracy();
    Eigen::Vector3d getTranslation();
    Eigen::Quaterniond getRotation();
    unsigned int getPoseElements();
    void rotoTranslation(Eigen::Vector3d, Eigen::Quaterniond);

  private:
    Eigen::Quaterniond hamiltonProduct(Eigen::Quaterniond, Eigen::Quaterniond);
};

#endif /* __POSE__ */