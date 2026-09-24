#ifndef TOOLS__UV_OVERLAY_HPP
#define TOOLS__UV_OVERLAY_HPP

#include <opencv2/opencv.hpp>

#include <list>
#include <optional>

#include "tasks/auto_aim/tracking/target.hpp"
#include "tasks/auto_aim/tracking/uv_model.hpp"

namespace tools
{
/** @brief UV 观测可视化配置（实时程序会在缩放后的画面上绘制，因此坐标与字号都可调） */
struct UvOverlayConfig
{
  /** @brief 角点/十字尺寸缩放（缩放后的画面应给 < 1 的值） */
  double scale = 1.0;
  /** @brief 字号缩放 */
  double font_scale = 1.0;
  /** @brief 顶部状态行文本位置 (px) */
  cv::Point status_pos{10, 30};
  /** @brief 残差文本首行的 y 坐标 (px) */
  int text_base_y = 150;
  /** @brief 相邻两条残差文本的行距 (px) */
  int text_line_step = 60;
  /** @brief 最多显示的匹配装甲板条数（避免文字刷屏） */
  int max_shown = 2;
  /** @brief 是否绘制"整车 4 块装甲板的模型预测轮廓" */
  bool draw_all_armors = true;
  /** @brief 是否绘制 UVL 残差数值文本 */
  bool draw_residual_text = true;
};

/**
 * @brief 在图像上叠加 UV 观测可视化，返回需要放大的装甲板区域（可选，交给 draw_zoom_inset）
 *
 * 画三样东西：
 *  1) 模型预测的全部装甲板轮廓（灰色=非当前装甲板，绿色=当前 last_id），可以直观看到
 *     滤波器对整车各块装甲板几何的估计；
 *  2) 每块检测到的装甲板：观测到的 4 个角点（青色实心点）与模型预测的 4 个角点（红色斜十字），
 *     两者之间用橙色连线表示残差——连线越短说明 UV 观测拟合越好；
 *  3) 每条灯条的 UVL 残差数值（角度/中心/长度）。
 *
 * 只有在 UV 模式下（target.ekf().uv_model().ready()）才绘制；否则只画一行模式提示，
 * 返回空矩形（此时调用方不应再画放大贴图）。
 *
 * @param img 待绘制的图像（原地修改）
 * @param target 当前跟踪目标（提供 EKF 状态与 UV 观测模型）
 * @param armors 本帧检测到的装甲板列表
 * @param uv_mode UV 观测是否开启（对应 Tracker::uv_enabled()）
 * @param fused 本帧融合次数（对应 Tracker::last_update_count()）
 * @param nis 最近一次更新的 NIS
 * @param config 绘制参数
 * @return 第一块匹配装甲板的外接矩形；非 UV 模式或无匹配时返回空矩形
 */
cv::Rect draw_uv_overlay(
  cv::Mat & img, const auto_aim::Target & target, const std::list<auto_aim::Armor> & armors,
  bool uv_mode, int fused, double nis, const UvOverlayConfig & config = {});

/**
 * @brief 把装甲板附近区域放大贴到右上角，方便肉眼看灯条端点级别的拟合
 * @param img 已绘制完标注的图像 @param focus 需要放大的区域 @param size 放大窗口边长（原图像素）
 * @param scale 坐标缩放（画面被缩小过时应传缩放比） @param margin_px 贴图边距
 */
void draw_zoom_inset(
  cv::Mat & img, const cv::Rect & focus, int size = 520, double scale = 1.0, int margin_px = 12);

}  // namespace tools

#endif  // TOOLS__UV_OVERLAY_HPP
