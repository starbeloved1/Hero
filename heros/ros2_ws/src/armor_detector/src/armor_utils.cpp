#include "armor_detector/armor_utils.hpp"

namespace armor_detector
{

bool shouldKeepColor(int detected_color, int target_color)
{
  if (target_color == -2) {
    return true;
  }
  if (detected_color == 2 || detected_color == 3) {
    return false;
  }
  if (target_color == -3) {
    return detected_color == 0 || detected_color == 1;
  }
  return detected_color == target_color;
}

int mapModelClass2HeroId(int model_class)
{
  // 0526 模型：0=基地、1=英雄、2=工程、3/4/5=步兵、6=前哨、7=哨兵、8=未知。
  // Hero 旧工程的公开编号：前哨为 7，哨兵为 0，基地为 6。
  switch (model_class) {
    case 0:
      return 6;
    case 1:
      return 1;
    case 2:
      return 2;
    case 3:
      return 3;
    case 4:
      return 4;
    case 5:
      return 5;
    case 6:
      return 7;
    case 7:
      return 0;
    default:
      return -1;
  }
}

}  // armor_detector
