/**
 * @file HandEyeCalib.h
 * @author philchen
 * @version 3.0
 * @date 2026-03-10
 */
#ifndef HANDEYECALIB_H
#define HANDEYECALIB_H

#include "base/customdialog.h"
#include <pcl/point_cloud.h>
#include <Eigen/Geometry>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_color_handlers.h>
#include <pcl/common/transforms.h>
#include <vtkGenericOpenGLRenderWindow.h>

namespace Ui
{
    class HandEyeCalib;
}

typedef std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> eigenVector;

typedef std::vector<Eigen::Affine3d, Eigen::aligned_allocator<Eigen::Affine3d>> EigenAffineVector;

enum Col { COL_INDEX = 0, COL_CHECK, COL_X, COL_Y, COL_Z, COL_COUNT };

struct Point3DConsistency
{
    Eigen::Vector3d meanAbsError = Eigen::Vector3d::Zero(); // 全局 X/Y/Z 平均绝对误差
    double reproject_error_pixel = 0.0;
    int    numPoses = 0;

    std::vector<Eigen::Vector3d> perPoseMeanAbsError; // [i] 该位姿 X/Y/Z 平均绝对误差
    std::vector<double>          perPoseError;        // [i] 该位姿标量误差（欧氏距离均值）

    std::vector<std::vector<Eigen::Vector3d>> perPointDev;

    std::vector<Eigen::Vector3d> pointMeanInBase;
};

class HandEyeCalib : public ct::CustomDialog
{
    Q_OBJECT

public:
    explicit HandEyeCalib(QWidget* parent = nullptr);
    ~HandEyeCalib();

    // 定义输入数据类型
    enum class RotateType
    {
        Euler,
        Quat,
        RotateVec
    };

    template <typename Input>
    Eigen::Vector3d eigenRotToEigenVector3dAngleAxis(Input eigenQuat) {
        Eigen::AngleAxisd ax3d(eigenQuat);
        return ax3d.angle() * ax3d.axis();
    }

    Eigen::Affine3d estimateHandEye(const EigenAffineVector& baseToTip,
        const EigenAffineVector& camToTag);

    void writeCalibration(const Eigen::Affine3f& result);

    double AssessImageQuality(const cv::Mat& image);

    cv::Mat Euler2CvRotationMatrix(const cv::Vec3d& rvec);

    Eigen::Matrix3d Euler2EigenRotationMatrix(const Eigen::Vector3d& rvec);

    void GenerateW3DPoints(bool symmetric, bool chess_board, int pattern_x, int pattern_y, double square_size, std::vector<cv::Point3f>& objp);

    void updateVtkWindow(const Eigen::Affine3f& transform, const Point3DConsistency& cons, bool handineye);

    void updateTable(QTableWidget* table, std::vector<bool>& validIndex, const Point3DConsistency& res);
    /**
     * @brief "眼在手外" (Eye-to-Hand) 手眼标定
     * @param pose_path 机器人位姿CSV文件路径
     * @param image_path 标定图像所在文件夹路径
     * @param square_size 棋盘格方格大小
     * @param pattern_x 棋盘格横向点数
     * @param pattern_y 棋盘格纵向点数
     * @param chess_board 是否为棋盘格
     * @param black_panel 是否为黑色面板
     * @param symmetric 是否为对称圆点网格
     * @param save_path 处理后图像的保存路径
     * @return std::tuple<cv::Mat, double, cv::Mat, cv::Mat> 返回相机到基座的变换矩阵、平均重投影误差、相机内参矩阵、畸变系数
     */
    Eigen::Affine3f eye_on_hand(
        const std::string& pose_path,
        const std::string& image_path,
        double square_size,
        int pattern_x,
        int pattern_y,
        bool chess_board,
        bool symmetric,
        const std::string& save_path,
        Point3DConsistency& cons);


    /**
     * @brief "眼在手上" (Eye-in-Hand) 手眼标定
     * @param pose_path 机器人位姿CSV文件路径
     * @param image_path 标定图像所在文件夹路径
     * @param square_size 棋盘格方格大小
     * @param pattern_x 棋盘格横向点数
     * @param pattern_y 棋盘格纵向点数
     * @param chess_board 是否为棋盘格
     * @param black_panel 是否为黑色面板
     * @param symmetric 是否为对称圆点网格
     * @param save_path 处理后图像的保存路径
     * @return std::tuple<cv::Mat, double, cv::Mat, cv::Mat> 返回相机到末端的变换矩阵、平均重投影误差、相机内参矩阵、畸变系数
     */
    Eigen::Affine3f eye_in_hand(
        const std::string& pose_path,
        const std::string& image_path,
        double square_size,
        int pattern_x,
        int pattern_y,
        bool chess_board,
        bool symmetric,
        const std::string& save_path,
        Point3DConsistency& cons);

    Point3DConsistency evaluate3DPointConsistencyEyeInHand(
        const std::vector<cv::Mat>& R_board2cams,
        const std::vector<cv::Mat>& t_board2cams,
        const std::vector<cv::Mat>& R_end2bases,
        const std::vector<cv::Mat>& t_end2bases,
        const cv::Mat& T_cam2end,
        const std::vector<cv::Point3f>& objp);

    /**
     * @brief 眼在手外 3D 点一致性评估
     * @param R_end2bases / t_end2bases 与 EyeInHand 相同：机器人原始位姿 end->base
     *        （函数内部取逆得到 base->end，再变到末端系评估）
     * @param T_cam2base calibrateHandEye 得到的相机到基座变换
     */
    Point3DConsistency evaluate3DPointConsistencyEyeOnHand(
        const std::vector<cv::Mat>& R_board2cams,
        const std::vector<cv::Mat>& t_board2cams,
        const std::vector<cv::Mat>& R_end2bases,
        const std::vector<cv::Mat>& t_end2bases,
        const cv::Mat& T_cam2base,
        const std::vector<cv::Point3f>& objp);

private slots:
    void on_loadImageBtn_clicked();

    void on_loadPoseBtn_clicked();

    void on_execCalibBtn_clicked();

    void on_saveCalibBtn_clicked();

private:

    EigenAffineVector base2end;

    EigenAffineVector cam2target;

    Eigen::Affine3f transform = Eigen::Affine3f::Identity();

    Ui::HandEyeCalib* ui;

    pcl::visualization::PCLVisualizer::Ptr m_viewer;
};

#endif // TCPCALIB_H
