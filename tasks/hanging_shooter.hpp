#ifndef HANGING_SHOOTER_HPP
#define HANGING_SHOOTER_HPP

#include <opencv2/opencv.hpp>
#include <vector>

#include "tools/img_tools.hpp"

namespace hanging_shooting
{

//  相机内参
static const cv::Mat camera_matrix =
  (cv::Mat_<double>(3, 3) << 4360.7139087362011, 0, 683.92579312415967, 0, 4390.4667853095989,
   401.85596366353803, 0, 0,
   1);  // 12mm镜头参数为：3615.7810408143187, 0, 779.87874382319069, 0, 3618.9880935370311,482.04218460953206, 0, 0,1
// 畸变系数
static const cv::Mat distort_coeffs =
  (cv::Mat_<double>(1, 5) << -0.046190212755841294, 1.6515418900072727, -5.1693400741013766e-05,
   -0.0079130702364351452,
   0);  // 12mm镜头参数为：-0.035346531904039129, 1.2817037157755276, -0.0012000404732901543,0.005985237112084223,0
constexpr double PI = 3.1415926;
constexpr double FAMILIAR = 0.80;           // 认定的判断值
constexpr double MIN_AREA = 80;             // 能接受的最小面积
constexpr double LIGHTSPOT_RADIUS = 55e-3;  // 发光板直径     单位：m

// 弹道解算参数(参考沈阳航空航天开源资料)
constexpr double g = 9.794;                    // 重力加速度   单位：m/s^2
constexpr double m = 0.041;                    // 大弹丸重量   单位：kg
constexpr double C = 0.275;                    // 大弹丸空气阻力系数
constexpr double rho = 1.169;                  // 空气密度     单位：kg/m^3
constexpr double BIG_BULLET_RADIUS = 0.02165;  // 大弹丸半径   单位：m

static const std::vector<cv::Point3f> lightspot_points{
  {0, -LIGHTSPOT_RADIUS / 2, 0},   // 点1
  {LIGHTSPOT_RADIUS / 2, 0, 0},    // 点2
  {0, LIGHTSPOT_RADIUS / 2, 0},    // 点3
  {-LIGHTSPOT_RADIUS / 2, 0, 0}};  // 点4
// 以圆心为xy的(0,0)，12点钟开始为点1,顺时钟方向为点2、点3、点4

struct LightSpot  //基地绿灯
{
  cv::Point2f center, top, bottom, left, right;  // 圆上下左右四点坐标
  float radius;                                  // 半径         单位：
  std::vector<cv::Point> contour;                // 轮廓点
  LightSpot(std::vector<cv::Point> contour, float radius, cv::Point2f center)
  {
    this->contour = contour;
    this->radius = radius;
    this->center = center;
    this->left = cv::Point(center.x - std::trunc(radius * 100) / 100.0f, center.y);
    this->top = cv::Point(center.x, center.y - std::trunc(radius * 100) / 100.0f);
    this->bottom = cv::Point(center.x, center.y + std::trunc(radius * 100) / 100.0f);
    this->right = cv::Point(center.x + std::trunc(radius * 100) / 100.0f, center.y);
    // this->left = cv::Point(center.x - int(radius), center.y);
    // this->top = cv::Point(center.x, center.y - int(radius));
    // this->bottom = cv::Point(center.x, center.y + int(radius));
    // this->right = cv::Point(center.x + int(radius), center.y);
    // 方案一：以最小外接圆得到的半径和圆心来得到上下左右点坐标
    // 问题：远距离识别时会造成很大误差
    // 解决：使用trunc函数来只取小数点后一位
    // std::sort(contour.begin(), contour.end(), [](const cv::Point2f & a, const cv::Point2f & b) {
    //   return a.y < b.y;
    // });  // 从小到大进行排序
    // this->top = contour[0];
    // this->bottom = contour.back();
    // std::sort(contour.begin(), contour.end(), [](const cv::Point2f & a, const cv::Point2f & b) {
    //   return a.x < b.x;
    // });  // 左到右进行排序
    // this->left = contour[0];
    // this->right = contour.back();
    //方案二：以上下左右极值作为识别到的绿灯上下左右四个点
    //问题：距离测量值会非常不稳定，近距离变化在一米以内，远距离会达到3米幅度。

    // for (std::vector<cv::Point>::iterator it = contour.begin(); it != contour.end(); it++) {
    //   if (fabs(it->x - center.x) > 0.01) {
    //     if (it->y > center.y)
    //       this->bottom = *it;
    //     else
    //       this->top = *it;
    //   }
    //   if (fabs(it->y - center.y) > 0.01) {
    //     if (it->x > center.x)
    //       this->right = *it;
    //     else
    //       this->left = *it;
    //   }
    // }
    //方案三：与圆心xy相同的四个点作为圆的上下左右四个点
    //问题：会出现tvec解不出值的情况，疑似因为四个点会有找不到的情况。
    //解决：两者在一定范围就判定是
    //问题2：解出来的值依旧不稳定
  };
};

class HangingShooter  //英雄吊射基地处理函数类
{
public:
  std::vector<LightSpot> detect(
    const cv::Mat & img);  // 检测图片当中的指示灯,返回检测到的指示灯数组
  void solvePnP(
    const LightSpot lightspot, cv::Mat & tvec,
    cv::Mat & rvec);  // 获得当前指示灯的方向向量

private:
  bool isCircle(
    const std::vector<cv::Point> & contour);  // 检测该轮廓是否为圆，并对过小的>圆进行过滤
};

class h_solver
{  // 弹道解算类
public:
  void calculate_pitch_noAirResistance(
    double bullet_speed, double distance, double height,
    double & pitch);  // 默认无空气阻力的弹道解算
  void calculate_pitch(
    double bulletSpeed, double distance, double height, double & pitch);  // 考虑空气阻力的弹道解算

private:
  void RungeKutta_4(
    std::vector<double> & x_vals, std::vector<double> & y_vals, double angle, double bullet_speed,
    double height, double dt = 0.1, double max_time = 10);  // 使用4阶龙格库塔推导弹道方程
  void culculate_pitch(
    double & pitch);  // 根据得到的弹道方程逆向求解得到打中y高度目标所需的pitch角度。
};
};  // namespace hanging_shooting
// namespace hanging_shooting
#endif
