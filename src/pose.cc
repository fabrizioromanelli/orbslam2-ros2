#include "pose.hpp"

Pose::Pose() : poseAccuracy(LOST)
{}

Pose::Pose(Eigen::Vector3d _trans, Eigen::Quaterniond _rot) : poseAccuracy(LOST)
{
  translation = _trans;
  rotation = _rot;
}

Pose::Pose(double _x, double _y, double _z, double _qw, double _qx, double _qy, double _qz) : poseAccuracy(LOST)
{
  translation = {_x, _y, _z};
  rotation = {_qw, _qx, _qy, _qz};
}

Pose::~Pose()
{
  
}

void Pose::setAccuracy(unsigned int _accuracy)
{
  poseAccuracy = _accuracy;
}

void Pose::setTranslation(double _x, double _y, double _z)
{
  translation = {_x, _y, _z};
}

void Pose::setTranslation(Eigen::Vector3d _trans)
{
  translation = _trans;
}

void Pose::setRotation(double _qw, double _qx, double _qy, double _qz)
{
  Eigen::Quaterniond tmp;
  tmp = {_qw, _qx, _qy, _qz};
  rotation = tmp.normalized();
}

void Pose::setRotation(Eigen::Quaterniond _rot)
{
  rotation = _rot.normalized();
}

unsigned int Pose::getAccuracy()
{
  return(poseAccuracy);
}

void Pose::getTranslation(Eigen::Vector3d& _trans)
{
  _trans = translation;
}

void Pose::getRotation(Eigen::Quaterniond& _rot)
{
  _rot = rotation;
}

Eigen::Vector3d Pose::getTranslation()
{
  return(translation);
}

Eigen::Quaterniond Pose::getRotation()
{
  return(rotation);
}

unsigned int Pose::getPoseElements()
{
  return(3 + 4); // 3 is the (x,y,z) components, 4 is the number of elements in a quaternion
}

// Execute a roto-translation of the current pose of _trans and _rot.
// This function performs the rotation of R to R' as: R' = H(H(Q, R), Q').
void Pose::rotoTranslation(Eigen::Vector3d _trans, Eigen::Quaterniond _rot)
{
  Eigen::Quaterniond r = {0.0, translation[X], translation[Y], translation[Z]};
  Eigen::Quaterniond rRotated = hamiltonProduct(hamiltonProduct(_rot.conjugate(), r), _rot);
  translation[X] = _trans[X] + rRotated.x();
  translation[Y] = _trans[Y] + rRotated.y();
  translation[Z] = _trans[Z] + rRotated.z();
  rotation       = rotation*_rot;
  return;
}

// Execute the hamilton product between quaterions.
// The result is returned.
Eigen::Quaterniond Pose::hamiltonProduct(Eigen::Quaterniond q1, Eigen::Quaterniond q2)
{
  Eigen::Quaterniond qr;
  qr.w() = q1.w()*q2.w() - q1.x()*q2.x() - q1.y()*q2.y() - q1.z()*q2.z();
  qr.x() = q1.w()*q2.x() + q1.x()*q2.w() - q1.y()*q2.z() + q1.z()*q2.y();
  qr.y() = q1.w()*q2.y() + q1.x()*q2.z() + q1.y()*q2.w() - q1.z()*q2.x();
  qr.z() = q1.w()*q2.z() - q1.x()*q2.y() + q1.y()*q2.x() + q1.z()*q2.w();
  return(qr);
}
