#include <vector>

namespace util {

std::vector<float> euler_frame_conversion(std::vector<float> point,
                                          std::vector<float> euler_angles,
                                          std::vector<float> translations);
std::vector<float> apply_euler_frame_rotation(std::vector<float> point, std::vector<float> euler_angles);
std::vector<float> apply_frame_translation(std::vector<float> point, std::vector<float> translations);
}; // namespace util
