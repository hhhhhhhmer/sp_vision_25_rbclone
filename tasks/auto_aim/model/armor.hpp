#ifndef AUTO_AIM__ARMOR_HPP
#define AUTO_AIM__ARMOR_HPP

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

namespace auto_aim
{
enum Color
{
  red,
  blue,
  extinguish,
  purple
};
const std::vector<std::string> COLORS = {"red", "blue", "extinguish", "purple"};

enum ArmorType
{
  big,
  small
};
const std::vector<std::string> ARMOR_TYPES = {"big", "small"};

enum ArmorName
{
  one,
  two,
  three,
  four,
  five,
  sentry,
  outpost,
  base,
  not_armor
};
const std::vector<std::string> ARMOR_NAMES = {"one",    "two",     "three", "four",     "five",
                                              "sentry", "outpost", "base",  "not_armor"};

enum ArmorPriority
{
  first = 1,
  second,
  third,
  forth,
  fifth
};

// clang-format off
const std::vector<std::tuple<Color, ArmorName, ArmorType>> armor_properties = {
  {blue, sentry, small},     {red, sentry, small},     {extinguish, sentry, small},
  {blue, one, small},        {red, one, small},        {extinguish, one, small},
  {blue, two, small},        {red, two, small},        {extinguish, two, small},
  {blue, three, small},      {red, three, small},      {extinguish, three, small},
  {blue, four, small},       {red, four, small},       {extinguish, four, small},
  {blue, five, small},       {red, five, small},       {extinguish, five, small},
  {blue, outpost, small},    {red, outpost, small},    {extinguish, outpost, small},
  {blue, base, big},         {red, base, big},         {extinguish, base, big},      {purple, base, big},       
  {blue, base, small},       {red, base, small},       {extinguish, base, small},    {purple, base, small},    
  {blue, three, big},        {red, three, big},        {extinguish, three, big}, 
  {blue, four, big},         {red, four, big},         {extinguish, four, big},  
  {blue, five, big},         {red, five, big},         {extinguish, five, big}};
// clang-format on

struct Lightbar
{
  std::size_t id = 0;
  Color color = Color::extinguish;
  cv::Point2f center{0.0F, 0.0F};
  cv::Point2f top{0.0F, 0.0F};
  cv::Point2f bottom{0.0F, 0.0F};
  cv::Point2f top2bottom{0.0F, 0.0F};
  std::vector<cv::Point2f> points;
  double angle = 0.0;
  double angle_error = 0.0;
  double length = 0.0;
  double width = 0.0;
  double ratio = 0.0;
  cv::RotatedRect rotated_rect;

  /** @brief 由最小外接旋转矩形构造灯条 @param rotated_rect 灯条轮廓的旋转矩形 @param id 灯条编号 */
  Lightbar(const cv::RotatedRect & rotated_rect, std::size_t id);
  /** @brief 构造空灯条 */
  Lightbar() = default;
};

struct Armor
{
  Color color = Color::extinguish;
  Lightbar left, right;     //used to be const
  cv::Point2f center;       // 不是对角线交点，不能作为实际中心！
  cv::Point2f center_norm{-1.0F, -1.0F};  // 归一化坐标；负值表示检测器尚未提供
  std::vector<cv::Point2f> points;

  double ratio = 0.0;              // 两灯条的中点连线与长灯条的长度之比
  double side_ratio = 0.0;         // 长灯条与短灯条的长度之比
  double rectangular_error = 0.0;  // 灯条和中点连线所成夹角与π/2的差值

  ArmorType type = ArmorType::small;
  ArmorName name = ArmorName::not_armor;
  ArmorPriority priority = ArmorPriority::fifth;
  int class_id = -1;
  cv::Rect box;
  cv::Mat pattern;
  double confidence = 0.0;
  bool duplicated = false;

  Eigen::Vector3d xyz_in_gimbal = Eigen::Vector3d::Zero();  // 单位：m
  Eigen::Vector3d xyz_in_world = Eigen::Vector3d::Zero();   // 单位：m
  Eigen::Vector3d ypr_in_gimbal = Eigen::Vector3d::Zero();  // 单位：rad
  Eigen::Vector3d ypr_in_world = Eigen::Vector3d::Zero();   // 单位：rad
  Eigen::Vector3d ypd_in_world = Eigen::Vector3d::Zero();   // 球坐标系

  double yaw_raw = 0.0;  // rad

  /**
   * @brief 是否已由窄窗 PnP 写入可用的装甲朝向（ypr_in_world[0]）
   *
   * UV 观测逐帧不跑完整 PnP，装甲朝向 θ 只有"两条灯条间距"这一路弱信息（正对时导数还是 0），
   * 闭环里相位会滞后。Solver::try_solve_yaw() 会用窄窗搜索补一个直接观测，置位本标志后
   * RVfromFYT 才会把它当成 1 维测量用；传统 PnP 路径不读这个标志。
   */
  bool pnp_yaw_valid = false;

  /** @brief 构造空装甲板 */
  Armor() = default;
  /** @brief 由左右灯条构造传统视觉装甲板 @param left 左灯条 @param right 右灯条 */
  Armor(const Lightbar & left, const Lightbar & right);
  /** @brief 由单一类别检测结果构造装甲板 @param class_id 模型类别编号 @param confidence 置信度 @param box 检测框 @param armor_keypoints 装甲板关键点 */
  Armor(
    int class_id, float confidence, const cv::Rect & box, std::vector<cv::Point2f> armor_keypoints);
  /** @brief 由单一类别检测结果和 ROI 偏移构造装甲板 @param class_id 模型类别编号 @param confidence 置信度 @param box 检测框 @param armor_keypoints 装甲板关键点 @param offset ROI 在原图中的偏移 */
  Armor(
    int class_id, float confidence, const cv::Rect & box, std::vector<cv::Point2f> armor_keypoints,
    cv::Point2f offset);
  /** @brief 由颜色和数字类别构造装甲板 @param color_id 颜色类别编号 @param num_id 数字类别编号 @param confidence 置信度 @param box 检测框 @param armor_keypoints 装甲板关键点 */
  Armor(
    int color_id, int num_id, float confidence, const cv::Rect & box,
    std::vector<cv::Point2f> armor_keypoints);
  /** @brief 由颜色、数字类别和 ROI 偏移构造装甲板 @param color_id 颜色类别编号 @param num_id 数字类别编号 @param confidence 置信度 @param box 检测框 @param armor_keypoints 装甲板关键点 @param offset ROI 在原图中的偏移 */
  Armor(
    int color_id, int num_id, float confidence, const cv::Rect & box,
    std::vector<cv::Point2f> armor_keypoints, cv::Point2f offset);
};

}  // namespace auto_aim

#endif  // AUTO_AIM__ARMOR_HPP
