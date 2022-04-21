#include "frame_conversions.h"
#include <math.h>
#include <vector>

using namespace std;

vector<float> util::euler_frame_conversion(vector<float> point,
                                           vector<float> euler_angles,
                                           vector<float> translations) {

  const float x_old = point.at(0) + translations.at(0);
  const float y_old = point.at(1) + translations.at(1);
  const float z_old = point.at(2) + translations.at(2);

  // alpha : roll
  // beta : pitch
  // gamma : yaw

  // https://en.wikipedia.org/wiki/Rotation_matrix#Basic_rotations
  // See this page for the written down rotation matrix
  const float roll = euler_angles.at(0);
  const float pitch = euler_angles.at(1);
  const float yaw = euler_angles.at(2);

  vector<float> res;

  const float x_new =
      cos(pitch) * cos(yaw) * x_old +
      (sin(roll) * sin(pitch) * cos(yaw) - cos(roll) * sin(yaw)) * y_old +
      (cos(roll) * sin(pitch) * cos(yaw) + sin(roll) * sin(yaw)) * z_old;
  res.push_back(x_new);

  const float y_new =
      cos(pitch) * sin(yaw) * x_old +
      (sin(roll) * sin(pitch) * sin(yaw) + cos(roll) * cos(yaw)) * y_old +
      (cos(roll) * sin(pitch) * sin(yaw) - sin(roll) * cos(yaw)) * z_old;
  res.push_back(y_new);

  const float z_new = -sin(pitch) * x_old + sin(roll) * cos(pitch) * y_old +
                      cos(roll) * cos(pitch) * z_old;
  res.push_back(z_new);
  return res;
}

vector<float> util::apply_euler_frame_rotation(vector<float> point,
                                               vector<float> euler_angles) {
  vector<float> res;

  float x_old = point.at(0);
  float y_old = point.at(1);
  float z_old = point.at(2);

  const float roll = euler_angles.at(0);
  const float pitch = euler_angles.at(1);
  const float yaw = euler_angles.at(2);

  const float x_new =
      cos(pitch) * cos(yaw) * x_old +
      (sin(roll) * sin(pitch) * cos(yaw) - cos(roll) * sin(yaw)) * y_old +
      (cos(roll) * sin(pitch) * cos(yaw) + sin(roll) * sin(yaw)) * z_old;
  res.push_back(x_new);

  const float y_new =
      cos(pitch) * sin(yaw) * x_old +
      (sin(roll) * sin(pitch) * sin(yaw) + cos(roll) * cos(yaw)) * y_old +
      (cos(roll) * sin(pitch) * sin(yaw) - sin(roll) * cos(yaw)) * z_old;
  res.push_back(y_new);

  const float z_new = -sin(pitch) * x_old + sin(roll) * cos(pitch) * y_old +
                      cos(roll) * cos(pitch) * z_old;
  res.push_back(z_new);
  return res;
}

vector<float> util::apply_frame_translation(vector<float> point,
                                            vector<float> translations) {
  vector<float> res;

  const float x_new = point.at(0) + translations.at(0);
  const float y_new = point.at(1) + translations.at(1);
  const float z_new = point.at(2) + translations.at(2);

  res.push_back(x_new);
  res.push_back(y_new);
  res.push_back(z_new);
  return res;
}
