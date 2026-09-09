/**
 * @file HandEyeCalib.cpp
 * @author philchen
 * @version 3.0
 * @date 2026-03-10
 */
#include "HandEyeCalib.h"
#include "ui_HandEyeCalib.h"
#include "ceres/ceres.h"
#include "ceres/types.h"
#include "HandEyeCalibration.h"
#include "MathUtils.h"
#include <QDesktopServices> 
#include <QUrl>
#include <fstream>
#include <nlohmann/json.hpp>
#include <QDateTime>
#include <QFileDialog>
#include <QCheckbox>

#include "calib/PsCalibrator.h"
#include "calib/DataProcessor.h"

using json = nlohmann::json;

namespace {
    // R(3x3,CV_64F) + t(3x1,CV_64F) -> Affine3d
    Eigen::Affine3d cvRt2Affine(const cv::Mat& R, const cv::Mat& t) {
        Eigen::Affine3d T = Eigen::Affine3d::Identity();
        for (int r = 0; r < 3; ++r) {
            for (int c = 0; c < 3; ++c)
                T.linear()(r, c) = R.at<double>(r, c);
            T.translation()(r) = t.at<double>(r, 0);
        }
        return T;
    }
    // 4x4(CV_64F) -> Affine3d
    Eigen::Affine3d cv4x4ToAffine(const cv::Mat& M) {
        Eigen::Affine3d T = Eigen::Affine3d::Identity();
        for (int r = 0; r < 4; ++r)
            for (int c = 0; c < 4; ++c)
                T.matrix()(r, c) = M.at<double>(r, c);
        return T;
    }

    Eigen::Affine3d cvRtToAffineNormalized(const cv::Mat& Rin, const cv::Mat& tin)
    {
        cv::Mat R, t;
        Rin.convertTo(R, CV_64F);
        tin.convertTo(t, CV_64F);
        if (R.rows == 3 && R.cols == 1) {
            cv::Mat R3;
            cv::Rodrigues(R, R3);
            R = R3;
        }
        else if (R.rows == 1 && R.cols == 3) {
            cv::Mat R3;
            cv::Rodrigues(R.t(), R3);
            R = R3;
        }
        if (t.rows == 1 && t.cols == 3)
            t = t.t();
        return cvRt2Affine(R, t);
    }

    // Same 4x4 layout OpenCV calibrateHandEye uses for Hg / Hc / X.
    cv::Mat cvRtToHomogeneous(const cv::Mat& Rin, const cv::Mat& tin)
    {
        cv::Mat R, t;
        Rin.convertTo(R, CV_64F);
        tin.convertTo(t, CV_64F);
        if (R.rows == 3 && R.cols == 1) {
            cv::Mat R3;
            cv::Rodrigues(R, R3);
            R = R3;
        }
        else if (R.rows == 1 && R.cols == 3) {
            cv::Mat R3;
            cv::Rodrigues(R.t(), R3);
            R = R3;
        }
        if (t.rows == 1 && t.cols == 3)
            t = t.t();
        cv::Mat T = cv::Mat::eye(4, 4, CV_64F);
        R.copyTo(T(cv::Rect(0, 0, 3, 3)));
        t.copyTo(T(cv::Rect(3, 0, 1, 3)));
        return T;
    }

    cv::Mat cvHomogeneousInverse(const cv::Mat& T)
    {
        cv::Mat R = T(cv::Rect(0, 0, 3, 3));
        cv::Mat t = T(cv::Rect(3, 0, 1, 3));
        cv::Mat Rt = R.t();
        cv::Mat Tinv = cv::Mat::eye(4, 4, CV_64F);
        Rt.copyTo(Tinv(cv::Rect(0, 0, 3, 3)));
        cv::Mat tinv = -Rt * t;
        tinv.copyTo(Tinv(cv::Rect(3, 0, 1, 3)));
        return Tinv;
    }

    Eigen::Vector3d transformHomogeneous(const cv::Mat& T, const cv::Point3f& p)
    {
        const double x = static_cast<double>(p.x);
        const double y = static_cast<double>(p.y);
        const double z = static_cast<double>(p.z);
        return Eigen::Vector3d(
            T.at<double>(0, 0) * x + T.at<double>(0, 1) * y + T.at<double>(0, 2) * z + T.at<double>(0, 3),
            T.at<double>(1, 0) * x + T.at<double>(1, 1) * y + T.at<double>(1, 2) * z + T.at<double>(1, 3),
            T.at<double>(2, 0) * x + T.at<double>(2, 1) * y + T.at<double>(2, 2) * z + T.at<double>(2, 3));
    }

    Point3DConsistency accumulatePointCloudConsistency(
        const std::vector<eigenVector>& ptsPerPoint)
    {
        Point3DConsistency res;
        const size_t nPts = ptsPerPoint.size();
        const size_t nPose = (nPts == 0) ? 0 : ptsPerPoint[0].size();
        res.numPoses = static_cast<int>(nPose);
        if (nPose < 2 || nPts == 0)
            return res;

        res.perPointDev.assign(nPose, std::vector<Eigen::Vector3d>(nPts, Eigen::Vector3d::Zero()));
        res.pointMeanInBase.assign(nPts, Eigen::Vector3d::Zero());
        res.perPoseMeanAbsError.assign(nPose, Eigen::Vector3d::Zero());
        res.perPoseError.assign(nPose, 0.0);

        Eigen::Vector3d sumAbs = Eigen::Vector3d::Zero();
        const double invPts = 1.0 / static_cast<double>(nPts);
        const double invPose = 1.0 / static_cast<double>(nPose);

        for (size_t j = 0; j < nPts; ++j) {
            Eigen::Vector3d mean = Eigen::Vector3d::Zero();
            for (size_t i = 0; i < nPose; ++i)
                mean += ptsPerPoint[j][i];
            mean *= invPose;
            res.pointMeanInBase[j] = mean;
            for (size_t i = 0; i < nPose; ++i) {
                const Eigen::Vector3d d = ptsPerPoint[j][i] - mean;
                res.perPointDev[i][j] = d;
                res.perPoseMeanAbsError[i] += d.cwiseAbs();
                res.perPoseError[i] += d.norm();
                sumAbs += d.cwiseAbs();
            }
        }
        for (size_t i = 0; i < nPose; ++i) {
            res.perPoseMeanAbsError[i] *= invPts;
            res.perPoseError[i] *= invPts;
        }
        res.meanAbsError = sumAbs * (invPts * invPose);
        return res;
    }

    Point3DConsistency evaluateSE3Product(
        const std::vector<cv::Mat>& T_lefts,
        const cv::Mat& T_x,
        const std::vector<cv::Mat>& T_rights,
        const std::vector<cv::Point3f>& objp)
    {
        const size_t nPose = T_lefts.size();
        const size_t nPts = objp.size();
        std::vector<eigenVector> pts(nPts, eigenVector(nPose, Eigen::Vector3d::Zero()));
        for (size_t i = 0; i < nPose; ++i) {
            const cv::Mat T = T_lefts[i] * T_x * T_rights[i];
            for (size_t j = 0; j < nPts; ++j)
                pts[j][i] = transformHomogeneous(T, objp[j]);
        }
        return accumulatePointCloudConsistency(pts);
    }

    bool fitCircle3D(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
        float dist_th,
        cv::Point3f& center,
        float& radius)
    {
        const int max_iters = 500;
        const int sample_num = 4;

        int inner = 0;
        float sphere_radius = 0.f;
        cv::Point3f sphere_center(0.f, 0.f, 0.f);

        if (!cloud || static_cast<int>(cloud->size()) < sample_num) {
            return false;
        }
        const int nums = static_cast<int>(cloud->size());

        pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
        kdtree.setInputCloud(cloud);

        Eigen::Matrix3d A = Eigen::Matrix3d::Zero();
        Eigen::Vector3d b = Eigen::Vector3d::Zero();
        std::mt19937 rng{ std::random_device{}() };
        std::uniform_int_distribution<int> uid(0, nums - 1);

        int iters = 0;
        while (iters < max_iters) {
            int idx[4];
            idx[0] = uid(rng);
            do { idx[1] = uid(rng); } while (idx[1] == idx[0]);
            do { idx[2] = uid(rng); } while (idx[2] == idx[0] || idx[2] == idx[1]);
            do { idx[3] = uid(rng); } while (idx[3] == idx[0] || idx[3] == idx[1] || idx[3] == idx[2]);

            const float x[4] = { cloud->points[idx[0]].x, cloud->points[idx[1]].x,
                                  cloud->points[idx[2]].x, cloud->points[idx[3]].x };
            const float y[4] = { cloud->points[idx[0]].y, cloud->points[idx[1]].y,
                                  cloud->points[idx[2]].y, cloud->points[idx[3]].y };
            const float z[4] = { cloud->points[idx[0]].z, cloud->points[idx[1]].z,
                                  cloud->points[idx[2]].z, cloud->points[idx[3]].z };

            A(0, 0) = x[0] - x[1]; A(0, 1) = y[0] - y[1]; A(0, 2) = z[0] - z[1];
            A(1, 0) = x[0] - x[2]; A(1, 1) = y[0] - y[2]; A(1, 2) = z[0] - z[2];
            A(2, 0) = x[0] - x[3]; A(2, 1) = y[0] - y[3]; A(2, 2) = z[0] - z[3];

            b(0) = ((x[0] * x[0] - x[1] * x[1]) + (y[0] * y[0] - y[1] * y[1]) + (z[0] * z[0] - z[1] * z[1])) / 2.0;
            b(1) = ((x[0] * x[0] - x[2] * x[2]) + (y[0] * y[0] - y[2] * y[2]) + (z[0] * z[0] - z[2] * z[2])) / 2.0;
            b(2) = ((x[0] * x[0] - x[3] * x[3]) + (y[0] * y[0] - y[3] * y[3]) + (z[0] * z[0] - z[3] * z[3])) / 2.0;

            if (std::abs(A.determinant()) < 1e-5) {
                ++iters;
                continue;
            }

            const Eigen::Vector3d c = A.inverse() * b;
            const double r = (c - Eigen::Vector3d(x[0], y[0], z[0])).norm();

            pcl::PointXYZ query(static_cast<float>(c.x()),
                static_cast<float>(c.y()),
                static_cast<float>(c.z()));
            std::vector<int> indice1, indice2;
            std::vector<float> sqr1, sqr2;
            int n1 = 0;
            if (r > dist_th) {
                n1 = kdtree.radiusSearch(query, r - dist_th, indice1, sqr1);
            }
            const int n2 = kdtree.radiusSearch(query, r + dist_th, indice2, sqr2);
            const int total = n2 - n1;

            if (total > inner) {
                inner = total;
                sphere_center = cv::Point3f(static_cast<float>(c.x()),
                    static_cast<float>(c.y()),
                    static_cast<float>(c.z()));
                sphere_radius = static_cast<float>(r);
            }
            if (inner > 0.999f * nums) {
                break;
            }
            ++iters;
        }

        if (sphere_radius < 1e-5f) {
            return false;
        }
        center = sphere_center;
        radius = sphere_radius;
        return true;
    }

    bool savePointsAsPly(const std::vector<cv::Point3f>& points, const std::string& filepath) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        cloud->resize(points.size());
        for (size_t i = 0; i < points.size(); ++i) {
            (*cloud)[i] = { points[i].x, points[i].y, points[i].z };
        }
        return pcl::io::savePLYFileBinary(filepath, *cloud) == 0;
    }
}

HandEyeCalib::HandEyeCalib(QWidget* parent) :CustomDialog(parent),
ui(new Ui::HandEyeCalib),
m_viewer(nullptr)
{
    ui->setupUi(this);

    vtkNew<vtkRenderer> renderer;
    vtkNew<vtkGenericOpenGLRenderWindow> renderWindow;
    renderWindow->AddRenderer(renderer);
    ui->qvtkWidget->setRenderWindow(renderWindow);
    m_viewer.reset(new pcl::visualization::PCLVisualizer(renderer, renderWindow, "viewer", false));
    m_viewer->setupInteractor(ui->qvtkWidget->interactor(), ui->qvtkWidget->renderWindow());
    m_viewer->setBackgroundColor(0.5, 0.5, 0.5);
    m_viewer->resetCamera();
    ui->qvtkWidget->update();

    ui->eulerBtn->setChecked(true);
    ui->eulerorderComb->setCurrentIndex(0);

    ui->EyeInHandbtn->setChecked(true);

	ui->stackedWidget->setCurrentIndex(0);
    connect(ui->calibPanleTypeComb, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [=](int index) {
        ui->stackedWidget->setCurrentIndex((index == 2) ? 1 : 0); });
}

HandEyeCalib::~HandEyeCalib()
{
    delete ui;
}

void HandEyeCalib::on_loadImageBtn_clicked() {
    QString ImageFolderPath = QFileDialog::getExistingDirectory(
        this,                          // 父窗口
        tr("Choose File"),               // 对话框标题
        "",                           // 初始目录，空字符串表示使用上次访问的目录
        QFileDialog::ShowDirsOnly     // 选项：只显示文件夹
        | QFileDialog::DontResolveSymlinks // 选项：不解析符号链接
    );

    if (!ImageFolderPath.isEmpty()) {
        ui->ImageFilePath->setText(ImageFolderPath);
    }
    else {
        printW(tr("No choose any file!"));
        return;
    }
    try
    {
        std::vector<std::string> imagePaths;
        cv::glob(ImageFolderPath.toStdString() + "/*", imagePaths, false);

        Point3DConsistency tempCons;
        tempCons.perPoseMeanAbsError.resize(imagePaths.size(), Eigen::Vector3d(0.0, 0.0, 0.0));
        std::vector<bool> validIndex(imagePaths.size(), true);
        updateTable(ui->validTable, validIndex, tempCons);
    }
    catch (const std::exception& e)
    {
        printW(QString("Image file path error happened, cause by %1").arg(e.what()));
        return;
    }
    
}

void HandEyeCalib::on_loadPoseBtn_clicked() {
    QString filePath = QFileDialog::getOpenFileName(this, tr("Open Json File"), "", tr("Json File (*.Json);;所有文件 (*)"));
    if (!filePath.isEmpty()) {
        ui->PosePath->setText(filePath);
    }
    else {
        printW(tr("No choose any file!"));
        return;
    }
}

void HandEyeCalib::on_execCalibBtn_clicked() {

    QString PoseFilePath  = ui->PosePath->text();
    QString ImageFolderPath = ui->ImageFilePath->text();

    if (PoseFilePath == "" || ImageFolderPath == "") {
        printE(tr("File path (%l) or (%2) not right load!").arg(PoseFilePath).arg(ImageFolderPath));
        return;
    }

    base2end.clear();
    cam2target.clear();

    bool handInEye = ui->EyeInHandbtn->isChecked();

    int typeCalibrationPanel = ui->calibPanleTypeComb->currentIndex();
    bool chess_board = typeCalibrationPanel == 0;
    bool symmetric = false;

    int squareX = ui->calibPanelXSpin->value();
    int squareY = ui->calibPanelYSpin->value();

    double squareInter = ui->P2PDisSpin->value();
    Point3DConsistency calibRet;
    if (handInEye) {
        if(ui->calibPanleTypeComb->currentIndex() == 2){
            double diameter = ui->sphereDiamSpin->value();
            double tolerance = ui->sphereTolSpin->value();
            transform = eye_in_hand(PoseFilePath.toStdString(), ImageFolderPath.toStdString(), diameter, tolerance, calibRet);
        }
        else {
            transform = eye_in_hand(PoseFilePath.toStdString(), ImageFolderPath.toStdString(), squareInter, squareX, squareY,
                chess_board, symmetric, "./config/Calibration/", calibRet);
        }
        
    }
    else{
        transform = eye_on_hand(PoseFilePath.toStdString(), ImageFolderPath.toStdString(), squareInter, squareX, squareY,
            chess_board, symmetric, "./config/Calibration/", calibRet);
    }

    updateVtkWindow(transform, calibRet, handInEye);
}

void HandEyeCalib::on_saveCalibBtn_clicked() {
    writeCalibration(transform);
}

void HandEyeCalib::writeCalibration(const Eigen::Affine3f& result) {
    QString pre;
    if (ui->EyeInHandbtn->isChecked()) {
        pre = "./config/calibration/HandInEyeCalib-";
    }
    else {
        pre = "./config/calibration/HandOnEyeCalib-";
    }

    QString filename = pre + QDateTime::currentDateTime().toString("yyyy-MM-dd-hh-mm-ss");

    QString filepath = QFileDialog::getSaveFileName(this, tr("Save Hand"), filename, "JSON(*.json)");
    if (filepath.isEmpty()) return;

    Eigen::Vector3f translation = result.translation();
    std::vector<double> translation_ = { translation.x(), translation.y(), translation.z() };
    nlohmann::json j;
    j["EIH"] = ui->EyeInHandbtn->isChecked() ? 1 : 2;
    j["Translation"] = translation_;
    Eigen::Matrix3f rotation_matrix = result.rotation();
    Eigen::Quaternionf quaternion(rotation_matrix);
    std::vector<float> quat = { quaternion.w(), quaternion.x(), quaternion.y(), quaternion.z() };
    j["Quaternion"] = quat;
    std::ofstream file(filepath.toStdString());
    if (!file.is_open()) {
        printE(tr("The HanEye Calibration data (%1) save failed!").arg(filepath));
        return;
    }
    printI(tr("The HanEye Calibration data (%1) save done!").arg(filepath));
    file << j.dump(4);
    file.close();
}

double HandEyeCalib::AssessImageQuality(const cv::Mat& image) {
    cv::Mat gray;
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    cv::Mat laplacian;
    cv::Laplacian(gray, laplacian, CV_64F);
    cv::Scalar mean, stddev;
    cv::meanStdDev(laplacian, mean, stddev);
    double variance = stddev.val[0] * stddev.val[0];
    return variance;
}

cv::Mat HandEyeCalib::Euler2CvRotationMatrix(const cv::Vec3d& rvec)
{
    double rx = rvec[0];
    double ry = rvec[1];
    double rz = rvec[2];

    double cx = std::cos(rx); double sx = std::sin(rx);
    double cy = std::cos(ry); double sy = std::sin(ry);
    double cz = std::cos(rz); double sz = std::sin(rz);

    cv::Mat R = (cv::Mat_<double>(3, 3) <<
        cz * cy, cz * sy * sx - sz * cx, cz * sy * cx + sz * sx,
        sz * cy, sz * sy * sx + cz * cx, sz * sy * cx - cz * sx,
        -sy, cy * sx, cy * cx);
    return R;
}

Eigen::Matrix3d HandEyeCalib::Euler2EigenRotationMatrix(const Eigen::Vector3d& rvec) {
    double rx = rvec.x();
    double ry = rvec.y();
    double rz = rvec.z();

    Eigen::AngleAxisd rollAngle(rx, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(ry, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(rz, Eigen::Vector3d::UnitZ());

    Eigen::Quaterniond q = yawAngle * pitchAngle * rollAngle;
    return q.toRotationMatrix();
}

void HandEyeCalib::GenerateW3DPoints(bool symmetric, bool chess_board, int pattern_x, int pattern_y, double square_size, std::vector<cv::Point3f>& objp) {
    // 生成3D世界坐标
    objp.clear();
    if (symmetric || chess_board) {
        for (int j = 0; j < pattern_y; j++) {
            for (int i = 0; i < pattern_x; i++) {
                objp.push_back(cv::Point3f(float(i * square_size), float(j * square_size), 0));
            }
        }
    }
    else { // Asymmetric
        double offset = square_size / 2.0;
        for (int j = 0; j < pattern_y; j++) {
            double start_x, start_y;
            if (j % 2 == 0) {
                start_x = 0.0;
                start_y = (j > 1) ? j * square_size / 2.0 : 0.0;
            }
            else {
                start_x = offset;
                start_y = (j > 2) ? offset + (j - 1) * square_size / 2.0 : offset;
            }
            for (int i = 0; i < pattern_x; i++) {
                double x = start_x + i * square_size;
                objp.push_back(cv::Point3f(x, start_y, 0.0));
            }
        }
    }
}

Eigen::Affine3f HandEyeCalib::eye_on_hand(
    const std::string& pose_path,
    const std::string& image_path,
    double square_size,
    int pattern_x,
    int pattern_y,
    bool chess_board,
    bool symmetric,
    const std::string& save_path,
    Point3DConsistency& cons) {

    cv::Size pattern_size(pattern_x, pattern_y);

    std::ifstream file(pose_path);
    if (!file.is_open()) {
        printE(tr("Failed to open input file (%1)!").arg(QString::fromStdString(pose_path)));
        return Eigen::Affine3f::Identity();
    }

    nlohmann::json j;
    try {
        file >> j;
        file.close();
    }
    catch (const json::parse_error& e) {
        file.close();
        printE(tr("Failed to parser input file (%1)!").arg(QString::fromStdString(pose_path)));
        return Eigen::Affine3f::Identity();
    }

    std::vector<std::string> indices;
    for (auto& element : j.items()) {
        indices.push_back(element.key());
    }

    std::vector<std::vector<double>> pose_vectors;

    RotateType rt;
    if (ui->eulerBtn->isChecked()) {
        rt = RotateType::Euler;
    }
    else {
        rt = RotateType::RotateVec;
    }

    for (std::string key : indices) {
        std::vector<double> pose_vec;
        if (j.contains(key) && j[key].is_array()) {
            pose_vec = j[key].get<std::vector<double>>();
            pose_vectors.push_back(pose_vec);
        }
    }

    if (pose_vectors.size() != indices.size()) {
        printE(tr("Input file num not equal!"));
        return Eigen::Affine3f::Identity();
    }

    int rowCount = ui->validTable->rowCount();
    std::vector<bool> keptIndices(rowCount, true);
    for (int i = 0; i < rowCount; ++i) {
        QWidget* w = ui->validTable->cellWidget(i, Col::COL_CHECK);
        if (!w) continue;
        QCheckBox* cb = w->findChild<QCheckBox*>();
        if (cb && !cb->isChecked())
            keptIndices[i] = false;
    }

    std::vector<std::string> image_paths;
    cv::glob(image_path + "/*", image_paths, false);

    if (image_paths.size() != pose_vectors.size()) {
        printE("Image Number not equal to Robot Pose Number, check the input data file!");
        return Eigen::Affine3f::Identity();
    }

    if (keptIndices.size() < image_paths.size())
        keptIndices.resize(image_paths.size(), true);

    std::vector<cv::Point3f> objp;
    GenerateW3DPoints(symmetric, chess_board, pattern_x, pattern_y, square_size, objp);

    std::vector<std::vector<cv::Point3f>> obj_points_list;
    std::vector<std::vector<cv::Point2f>> img_points_list;
    std::vector<std::vector<double>> filtered_pose_vectors;
    int det_success_num = 0;
    cv::Mat gray;

    for (size_t i = 0; i < image_paths.size(); ++i) {
        if (!keptIndices[i]) continue;

        cv::Mat img = cv::imread(image_paths[i]);
        if (img.empty()) {
            keptIndices[i] = false;
            continue;
        }

        double quality_score = AssessImageQuality(img);
        double quality_threshold = 20.0;
        if (quality_score < quality_threshold) {
            keptIndices[i] = false;
            printW(tr("Image Quality low!"));
            continue;
        }

        cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
        gray = 255 - gray;

        std::vector<cv::Point2f> corners;
        bool found = false;

        if (chess_board) {
            found = cv::findChessboardCornersSB(gray, pattern_size, corners);
        }
        else {
            cv::GaussianBlur(gray, gray, cv::Size(5, 5), 0);
            if (symmetric) {
                found = cv::findCirclesGrid(gray, pattern_size, corners, cv::CALIB_CB_SYMMETRIC_GRID);
            }
            else {
                found = cv::findCirclesGrid(gray, pattern_size, corners, cv::CALIB_CB_ASYMMETRIC_GRID);
                if (found) {
                    found = (corners.size() == static_cast<size_t>(pattern_x * pattern_y));
                }
            }
        }

        if (found) {
            det_success_num++;
            obj_points_list.push_back(objp);
            img_points_list.push_back(corners);
            filtered_pose_vectors.push_back(pose_vectors[i]);

            cv::drawChessboardCorners(img, pattern_size, corners, found);
            cv::imwrite(save_path + "/calibed_image/CalibImage_" + std::to_string(i) + ".png", img);
        }
        else {
            keptIndices[i] = false;
        }
    }

    if (obj_points_list.size() < 3) {
        printE(tr("Not Found Calibration Panel Image, please check the setting para!"));
        return Eigen::Affine3f::Identity();
    }

    cv::Mat camera_matrix, dist_coeffs;
    std::vector<cv::Mat> rvecs, tvecs;
    double ret = cv::calibrateCamera(obj_points_list, img_points_list, gray.size(), camera_matrix, dist_coeffs, rvecs, tvecs);

    std::vector<cv::Mat> R_board2cameras;
    std::vector<cv::Mat> t_board2cameras;

    for (size_t i = 0; i < obj_points_list.size(); ++i) {
        cv::Mat rvec, tvec;
        bool success = cv::solvePnP(obj_points_list[i], img_points_list[i], camera_matrix, dist_coeffs, rvec, tvec);
        if (success) {
            cv::Mat R_board2camera;
            cv::Rodrigues(rvec, R_board2camera);
            R_board2cameras.push_back(R_board2camera.clone());
            t_board2cameras.push_back(tvec.clone());
        }
        else {
            printE(tr("Pnp slove failed!"));
            return Eigen::Affine3f::Identity();
        }
    }

    MathUtils mu;
    std::vector<cv::Mat> R_end2bases;
    std::vector<cv::Mat> t_end2bases;
    for (const auto& pose_vec : filtered_pose_vectors) {
        QString order = ui->eulerorderComb->currentText();
        Eigen::Matrix3d matrix;
        if (rt == RotateType::Euler) {
            assert(pose_vec.size() == 6);
            Eigen::Vector3d euler_angle = Eigen::Vector3d(pose_vec[3], pose_vec[4], pose_vec[5]);
            Eigen::Vector3d euler_rad;
            if (ui->angleUnitComb->currentIndex() == 0) {
                euler_rad = mu.deg2radVec(euler_angle);
            }
            else {
                euler_rad = euler_angle;
            }
            matrix = mu.eulerToMatrix(euler_rad, order.toStdString());
        }
        else if (rt == RotateType::Quat) {
            assert(pose_vec.size() == 7);
            Eigen::Quaterniond q = Eigen::Quaterniond(pose_vec[3], pose_vec[4], pose_vec[5], pose_vec[6]);
            matrix = mu.quaternionToMatrix(q);
        }
        else if (rt == RotateType::RotateVec) {
            assert(pose_vec.size() == 6);
            Eigen::Vector3d rotate_vec = Eigen::Vector3d(pose_vec[3], pose_vec[4], pose_vec[5]);
            Eigen::Vector3d rot_vec_rad;
            if (ui->angleUnitComb->currentIndex() == 0) {
                rot_vec_rad = mu.deg2radVec(rotate_vec);
            }
            else {
                rot_vec_rad = rotate_vec;
            }
            matrix = mu.rotVecToMatrix(rot_vec_rad);
        }

        cv::Mat R_end2base = cv::Mat::eye(3, 3, CV_64F);
        for (int r = 0; r < 3; ++r) {
            for (int c = 0; c < 3; ++c) {
                R_end2base.at<double>(r, c) = matrix.matrix()(r, c);
            }
        }
        cv::Mat t_end2base = (cv::Mat_<double>(3, 1) << pose_vec[0], pose_vec[1], pose_vec[2]);

        // OpenCV 眼在手外与评估函数都使用 base->end
        cv::Mat R_base2end = R_end2base.t();
        cv::Mat t_base2end = -R_base2end * t_end2base;
        R_end2bases.push_back(R_base2end.clone());
        t_end2bases.push_back(t_base2end.clone());
    }

    std::vector<cv::Mat> R_calib, t_calib;
    R_calib.reserve(R_end2bases.size());
    t_calib.reserve(t_end2bases.size());
    for (size_t i = 0; i < R_end2bases.size(); ++i) {
        R_calib.push_back(R_end2bases[i].clone());
        t_calib.push_back(t_end2bases[i].clone());
    }

    cv::Mat R_camera2base, t_camera2base;
    cv::calibrateHandEye(R_calib, t_calib, R_board2cameras, t_board2cameras,
        R_camera2base, t_camera2base, cv::CALIB_HAND_EYE_TSAI);

    cv::Mat T_camera2base = cv::Mat::eye(4, 4, CV_64F);
    R_camera2base.copyTo(T_camera2base(cv::Rect(0, 0, 3, 3)));
    t_camera2base.copyTo(T_camera2base(cv::Rect(3, 0, 1, 3)));

    double total_error = 0.0;
    for (size_t i = 0; i < obj_points_list.size(); ++i) {
        std::vector<cv::Point2f> projected_points;
        cv::projectPoints(obj_points_list[i], rvecs[i], tvecs[i], camera_matrix, dist_coeffs, projected_points);
        double error = cv::norm(img_points_list[i], projected_points, cv::NORM_L2) / static_cast<double>(projected_points.size());
        total_error += error;
    }

    cons = evaluate3DPointConsistencyEyeOnHand(
        R_board2cameras, t_board2cameras,
        R_end2bases, t_end2bases,
        T_camera2base, objp);
    cons.reproject_error_pixel = total_error / static_cast<double>(obj_points_list.size());

    Eigen::Affine3d trans;
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            trans.matrix()(i, j) = T_camera2base.at<double>(i, j);
        }
    }

    updateTable(ui->validTable, keptIndices, cons);

    return trans.cast<float>();
}

Eigen::Affine3f HandEyeCalib::eye_in_hand(
    const std::string& pose_path,
    const std::string& image_path,
    double square_size,
    int pattern_x,
    int pattern_y,
    bool chess_board,
    bool symmetric,
    const std::string& save_path, 
    Point3DConsistency& cons) {

    cv::Size pattern_size(pattern_x, pattern_y);

    std::ifstream file(pose_path);
    if (!file.is_open()) {
        printE(tr("Failed to open input file (%1)!").arg(QString::fromStdString(pose_path)));
        return Eigen::Affine3f::Identity();
    }

    nlohmann::json j;
    try {
        file >> j;
        file.close();
    }
    catch (const json::parse_error& e) {
        file.close();
        printE(tr("Failed to parser input file (%1)!").arg(QString::fromStdString(pose_path)));
        return Eigen::Affine3f::Identity();
    }

    std::vector<std::string> indices;
    for (auto& element : j.items()) {
        indices.push_back(element.key());
    }
    std::vector<std::vector<double>> pose_vectors;

    RotateType rt;
    if (ui->eulerBtn->isChecked()) {
        rt = RotateType::Euler;
    }
    else {
        rt = RotateType::RotateVec;
    }

    for (std::string key : indices) {
        std::vector<double> pose_vec;
        if (j.contains(key) && j[key].is_array()) {
            pose_vec = j[key].get<std::vector<double>>();
            pose_vectors.push_back(pose_vec);
        }
    }

    if (pose_vectors.size() != indices.size()) {
        printE(tr("Input file num not equal!"));
        return Eigen::Affine3f::Identity();
    }

    int rowCount = ui->validTable->rowCount();
    std::vector<bool> keptIndices(rowCount, true);
    for (int i = 0; i < rowCount; ++i) {
        QWidget* w = ui->validTable->cellWidget(i, Col::COL_CHECK);
        if (!w) continue;
        QCheckBox* cb = w->findChild<QCheckBox*>();
        if (cb && !cb->isChecked())
            keptIndices[i] = false;
    }

    std::vector<std::string> image_paths;
    cv::glob(image_path + "/*", image_paths, false);

    if (image_paths.size() != pose_vectors.size()) {
        printE("Image Number not equal to Robot Pose Number, check the input data file!");
        return Eigen::Affine3f::Identity();
    }

    std::vector<std::vector<cv::Point3f>> obj_points_list;
    std::vector<std::vector<cv::Point2f>> img_points_list;
    std::vector<std::vector<double>> filtered_pose_vectors; // 修正：使用正确的类型

    std::vector<cv::Point3f> objp;
    GenerateW3DPoints(symmetric, chess_board, pattern_x, pattern_y, square_size, objp);

    int det_success_num = 0;
    cv::Mat gray;

    for (size_t i = 0; i < image_paths.size(); ++i) {

        if (!keptIndices[i]) continue;

        cv::Mat img = cv::imread(image_paths[i]);
        if (img.empty()) {
            keptIndices[i] = false;
            continue;
        }

        double quality_score = AssessImageQuality(img);
        double quality_threshold = 20.0;

        if (quality_score < quality_threshold) {
            keptIndices[i] = false;
            printW("Image Quality Low, Pass away!");
            continue;
        }

        cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
        gray = 255 - gray;

        std::vector<cv::Point2f> corners;
        bool found = false;

        if (chess_board) {
            found = cv::findChessboardCornersSB(gray, pattern_size, corners);
        }
        else {
            cv::SimpleBlobDetector::Params params;
            params.minArea = 100;
            params.maxArea = 10000;
            params.minCircularity = 0.6f; // 增加圆形度要求
            cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create(params);

            if (symmetric) {
                found = cv::findCirclesGrid(gray, pattern_size, corners, cv::CALIB_CB_SYMMETRIC_GRID, detector);
            }
            else {
                found = cv::findCirclesGrid(gray, pattern_size, corners, cv::CALIB_CB_ASYMMETRIC_GRID, detector);
            }
        }

        if (found) {
            det_success_num++;
            obj_points_list.push_back(objp);
            img_points_list.push_back(corners);
            filtered_pose_vectors.push_back(pose_vectors[i]); // 修正：类型匹配

            cv::drawChessboardCorners(img, pattern_size, corners, found);
            cv::imwrite(save_path + "/calibed_image/CalibImage_" + std::to_string(i) + ".png", img);
        }
        else {
            keptIndices[i] = false;
        }
    }

    if (obj_points_list.size() == 0) {
        printE(tr("Not Found Calibration Panel Image, please check the setting para!"));
        return Eigen::Affine3f::Identity();
    }

    // --- 相机标定 ---
    cv::Mat camera_matrix, dist_coeffs;
    std::vector<cv::Mat> rvecs, tvecs;
    double ret = cv::calibrateCamera(obj_points_list, img_points_list, gray.size(), camera_matrix, dist_coeffs, rvecs, tvecs);

    // --- 求解标定板在相机坐标系中的位姿 ---
    std::vector<cv::Mat> R_board2cameras;
    std::vector<cv::Mat> t_board2cameras;

    for (size_t i = 0; i < obj_points_list.size(); ++i) {
        cv::Mat rvec, tvec;
        bool success = cv::solvePnP(obj_points_list[i], img_points_list[i], camera_matrix, dist_coeffs, rvec, tvec);
        if (success) {
            cv::Mat R_board2camera;
            cv::Rodrigues(rvec, R_board2camera);

            R_board2cameras.push_back(R_board2camera.clone());
            t_board2cameras.push_back(tvec.clone());

            /*cv::Mat T_board2camera = cv::Mat::eye(4, 4, CV_64F);
            R_board2camera.copyTo(T_board2camera(cv::Rect(0, 0, 3, 3)));
            tvec.copyTo(T_board2camera(cv::Rect(3, 0, 1, 3)));
            Eigen::Affine3d t1e;
            cv::cv2eigen(T_board2camera, t1e.matrix());
            *cam2target_it = t1e.inverse();*/
        }
        else {
            printE(tr("Pnp slove failed!"));
            return Eigen::Affine3f::Identity();
        }
    }

    // --- 转换机器人位姿数据为末端的变换 (与eye_on_hand不同) ---
    MathUtils mu;
    std::vector<cv::Mat> R_end2bases;
    std::vector<cv::Mat> t_end2bases;

    auto base2end_it = std::back_inserter(base2end);
    for (const auto& pose_vec : filtered_pose_vectors) { 
        Eigen::Vector3d pos = Eigen::Vector3d(pose_vec[0], pose_vec[1], pose_vec[2]);
        QString order = ui->eulerorderComb->currentText();
        Eigen::Matrix3d matrix;
        if (rt == RotateType::Euler) {
            assert(pose_vec.size() == 6);
            Eigen::Vector3d euler_angle = Eigen::Vector3d(pose_vec[3], pose_vec[4], pose_vec[5]);
            Eigen::Vector3d euler_rad;
            if (ui->angleUnitComb->currentIndex() == 0) {
                euler_rad = mu.deg2radVec(euler_angle);
            }
            else {
                euler_rad = euler_angle;
            }
            matrix = mu.eulerToMatrix(euler_rad, order.toStdString());
        }
        else if (rt == RotateType::Quat) {
            assert(pose_vec.size() == 7); // w x y z
            Eigen::Quaterniond q = Eigen::Quaterniond(pose_vec[3], pose_vec[4], pose_vec[5], pose_vec[6]);
            matrix = mu.quaternionToMatrix(q);
        }
        else if (rt == RotateType::RotateVec) {
            assert(pose_vec.size() == 6);
            Eigen::Vector3d rotate_vec = Eigen::Vector3d(pose_vec[3], pose_vec[4], pose_vec[5]);
            Eigen::Vector3d rot_vec_rad;
            if (ui->angleUnitComb->currentIndex() == 0) {
                rot_vec_rad = mu.deg2radVec(rotate_vec);
            }
            else {
                rot_vec_rad = rotate_vec;
            }
            matrix = mu.rotVecToMatrix(rot_vec_rad);
        }

        /*Eigen::Affine3d transform = Eigen::Affine3d::Identity();
        transform.linear() = matrix;
        transform.translation() = pos;
        *base2end_it = transform;*/

        cv::Mat R_end = cv::Mat::eye(3, 3, CV_64F);
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                R_end.at<double>(i, j) = matrix.matrix()(i, j);
            }
        }
        cv::Mat t_end = (cv::Mat_<double>(3, 1) << pose_vec[0], pose_vec[1], pose_vec[2]);
        R_end2bases.push_back(R_end.clone());
        t_end2bases.push_back(t_end.clone());
    }

    cv::Mat R_camera2end, t_camera2end;
    cv::calibrateHandEye(R_end2bases, t_end2bases, R_board2cameras, t_board2cameras, R_camera2end, t_camera2end, cv::CALIB_HAND_EYE_TSAI);

    cv::Mat T_camera2end = cv::Mat::eye(4, 4, CV_64F);
    R_camera2end.copyTo(T_camera2end(cv::Rect(0, 0, 3, 3)));
    t_camera2end.copyTo(T_camera2end(cv::Rect(3, 0, 1, 3)));

    double total_error = 0.0;
    for (size_t i = 0; i < obj_points_list.size(); ++i) {
        std::vector<cv::Point2f> projected_points;
        cv::projectPoints(obj_points_list[i], rvecs[i], tvecs[i], camera_matrix, dist_coeffs, projected_points);
        double error = cv::norm(img_points_list[i], projected_points, cv::NORM_L2) / static_cast<double>(projected_points.size());
        total_error += error;
    }

    cons = evaluate3DPointConsistencyEyeInHand(
        R_board2cameras, t_board2cameras,
        R_end2bases, t_end2bases,
        T_camera2end, objp);
    cons.reproject_error_pixel = total_error / static_cast<double>(obj_points_list.size());;

    Eigen::Affine3d trans;
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            trans.matrix()(i, j) = T_camera2end.at<double>(i, j);
        }
    }

    updateTable(ui->validTable, keptIndices, cons);

    return trans.cast<float>();

    /*if (cam2target.size() != base2end.size()) {
        printE(tr("Image num is not equal to Pose num!"));
        return Eigen::Affine3f::Identity();
    }

    Eigen::Affine3d trans = estimateHandEye(base2end, cam2target);

    base2end.clear();
    cam2target.clear();

    return trans.cast<float>();*/
}

Point3DConsistency HandEyeCalib::evaluate3DPointConsistencyEyeInHand(
    const std::vector<cv::Mat>& R_board2cams,
    const std::vector<cv::Mat>& t_board2cams,
    const std::vector<cv::Mat>& R_end2bases,
    const std::vector<cv::Mat>& t_end2bases,
    const cv::Mat& T_cam2end,
    const std::vector<cv::Point3f>& objp)
{
    // Eye-in-hand: board is fixed in the robot BASE.
    // OpenCV Tsai (and eye_in_hand) defines X = T_cam2end as cam2gripper, with
    //   T_board2base = T_end2base * T_cam2end * T_board2cam
    //
    // Real data rejected BOTH that product and the version that inverts X
    // (errors stayed at flange-motion scale, tens to hundreds of mm). That
    // means the eval inputs are not in the same SE(3) convention as
    // calibrateHandEye — typically robot poses already inverted (base→end),
    // or board/robot argument order swapped when only this function is pasted.
    //
    // Build 4x4 matrices the same way OpenCV does, then keep the product
    // whose 3D spread is smallest. On matching inputs the OpenCV chain wins
    // by orders of magnitude; on mismatched inputs another listed chain wins
    // instead of leaking robot motion into the table.
    Point3DConsistency res;
    const size_t nPose = R_board2cams.size();
    const size_t nPts = objp.size();
    res.numPoses = static_cast<int>(nPose);

    if (nPose < 2 || nPts == 0 ||
        R_end2bases.size() != nPose || t_end2bases.size() != nPose ||
        t_board2cams.size() != nPose) {
        printW(tr("3D consistency eval skipped: need >=2 poses and >=1 point."));
        return res;
    }

    cv::Mat T_x;
    T_cam2end.convertTo(T_x, CV_64F);
    if (T_x.rows != 4 || T_x.cols != 4) {
        printW(tr("3D consistency eval skipped: T_cam2end must be 4x4."));
        return res;
    }
    const cv::Mat T_xinv = cvHomogeneousInverse(T_x);

    std::vector<cv::Mat> T_e2b, T_b2e, T_b2c, T_c2b;
    T_e2b.reserve(nPose);
    T_b2e.reserve(nPose);
    T_b2c.reserve(nPose);
    T_c2b.reserve(nPose);
    for (size_t i = 0; i < nPose; ++i) {
        const cv::Mat Hg = cvRtToHomogeneous(R_end2bases[i], t_end2bases[i]);
        const cv::Mat Hc = cvRtToHomogeneous(R_board2cams[i], t_board2cams[i]);
        T_e2b.push_back(Hg);
        T_b2e.push_back(cvHomogeneousInverse(Hg));
        T_b2c.push_back(Hc);
        T_c2b.push_back(cvHomogeneousInverse(Hc));
    }

    struct Candidate {
        const char* name;
        Point3DConsistency cons;
    };
    const Candidate candidates[] = {
        { "T_end2base * T_cam2end * T_board2cam",
          evaluateSE3Product(T_e2b, T_x, T_b2c, objp) },
        { "T_end2base * T_end2cam * T_board2cam",
          evaluateSE3Product(T_e2b, T_xinv, T_b2c, objp) },
        { "T_base2end * T_cam2end * T_board2cam",
          evaluateSE3Product(T_b2e, T_x, T_b2c, objp) },
        { "T_base2end * T_end2cam * T_board2cam",
          evaluateSE3Product(T_b2e, T_xinv, T_b2c, objp) },
        { "T_end2base * T_cam2end * T_cam2board",
          evaluateSE3Product(T_e2b, T_x, T_c2b, objp) },
        { "T_board2cam * T_cam2end * T_end2base",
          evaluateSE3Product(T_b2c, T_x, T_e2b, objp) },
    };

    size_t best = 0;
    auto score = [](const Point3DConsistency& c) {
        return c.meanAbsError.x() + c.meanAbsError.y() + c.meanAbsError.z();
    };
    double bestScore = score(candidates[0].cons);
    for (size_t k = 1; k < sizeof(candidates) / sizeof(candidates[0]); ++k) {
        const double s = score(candidates[k].cons);
        if (s < bestScore) {
            bestScore = s;
            best = k;
        }
    }

    res = candidates[best].cons;
    printI(tr("[EyeInHand] 3D consistency chain: %1  MAE[mm] X:%2 Y:%3 Z:%4")
        .arg(QString::fromUtf8(candidates[best].name))
        .arg(res.meanAbsError.x(), 0, 'f', 4)
        .arg(res.meanAbsError.y(), 0, 'f', 4)
        .arg(res.meanAbsError.z(), 0, 'f', 4));
    return res;
}

Point3DConsistency HandEyeCalib::evaluate3DPointConsistencyEyeOnHand(
    const std::vector<cv::Mat>& R_board2cams,
    const std::vector<cv::Mat>& t_board2cams,
    const std::vector<cv::Mat>& R_end2bases,
    const std::vector<cv::Mat>& t_end2bases,
    const cv::Mat& T_cam2base,
    const std::vector<cv::Point3f>& objp)
{
    // 眼在手外（当前正确流程）：
    // 1) PnP：棋盘格点 -> 相机  p_cam  = R_board2cam * p_board + t
    // 2) 手眼 T_cam2base：相机 -> 基座  p_base = R_cam2base * p_cam + t
    //    T_cam2base 直接左乘，把相机系的点变到基座系。
    // 3) 机器人位姿（与 OpenCV 相同，已是 base->end）：基座 -> 法兰
    //    p_end = R_base2end * p_base + t
    // 标定板固连法兰，p_end 在各姿态下应重合；全部位姿求均值后算偏差。
    Point3DConsistency res;
    const size_t nPose = R_board2cams.size();
    const size_t nPts = objp.size();
    res.numPoses = static_cast<int>(nPose);

    if (nPose < 2 || nPts == 0 ||
        R_end2bases.size() != nPose || t_end2bases.size() != nPose ||
        t_board2cams.size() != nPose) {
        printW(tr("3D consistency eval skipped: need >=2 poses and >=1 point."));
        return res;
    }

    const Eigen::Affine3d T_c2b = cv4x4ToAffine(T_cam2base);
    const Eigen::Matrix3d R_cam2base = T_c2b.linear();
    const Eigen::Vector3d t_cam2base = T_c2b.translation();

    std::vector<eigenVector> ptsInEnd(nPts, eigenVector(nPose, Eigen::Vector3d::Zero()));

    for (size_t i = 0; i < nPose; ++i) {
        const Eigen::Affine3d T_b2c = cvRtToAffineNormalized(R_board2cams[i], t_board2cams[i]);
        const Eigen::Affine3d T_b2e = cvRtToAffineNormalized(R_end2bases[i], t_end2bases[i]);
        const Eigen::Matrix3d R_board2cam = T_b2c.linear();
        const Eigen::Vector3d t_board2cam = T_b2c.translation();
        const Eigen::Matrix3d R_base2end = T_b2e.linear();
        const Eigen::Vector3d t_base2end = T_b2e.translation();

        for (size_t j = 0; j < nPts; ++j) {
            const Eigen::Vector3d p_board(objp[j].x, objp[j].y, objp[j].z);
            const Eigen::Vector3d p_cam = R_board2cam * p_board + t_board2cam;
            const Eigen::Vector3d p_base = R_cam2base * p_cam + t_cam2base;
            ptsInEnd[j][i] = R_base2end * p_base + t_base2end;
        }
    }

    res.perPointDev.assign(nPose, std::vector<Eigen::Vector3d>(nPts, Eigen::Vector3d::Zero()));
    res.pointMeanInBase.assign(nPts, Eigen::Vector3d::Zero());
    res.perPoseMeanAbsError.assign(nPose, Eigen::Vector3d::Zero());
    res.perPoseError.assign(nPose, 0.0);

    Eigen::Vector3d sumAbs = Eigen::Vector3d::Zero();
    const double invPts = 1.0 / static_cast<double>(nPts);
    const double invPose = 1.0 / static_cast<double>(nPose);

    for (size_t j = 0; j < nPts; ++j) {
        Eigen::Vector3d mean = Eigen::Vector3d::Zero();
        for (size_t i = 0; i < nPose; ++i)
            mean += ptsInEnd[j][i];
        mean *= invPose;
        res.pointMeanInBase[j] = mean;

        for (size_t i = 0; i < nPose; ++i) {
            const Eigen::Vector3d d = ptsInEnd[j][i] - mean;
            res.perPointDev[i][j] = d;
            res.perPoseMeanAbsError[i] += d.cwiseAbs();
            res.perPoseError[i] += d.norm();
            sumAbs += d.cwiseAbs();
        }
    }

    for (size_t i = 0; i < nPose; ++i) {
        res.perPoseMeanAbsError[i] *= invPts;
        res.perPoseError[i] *= invPts;
    }
    res.meanAbsError = sumAbs * (invPts * invPose);
    return res;
}

void HandEyeCalib::updateTable(QTableWidget* table,
    std::vector<bool>& validIndex,
    const Point3DConsistency& res)
{
    const int rowCount = static_cast<int>(validIndex.size());          // 总索引数，如 17

    // 防御：res 中有效结果个数应等于 validIndex 中 true 的个数
    const int validTotal = static_cast<int>(
        std::count(validIndex.begin(), validIndex.end(), true));
    if (static_cast<int>(res.perPoseMeanAbsError.size()) != validTotal) {
        printW(tr("updateTable: res size(%1) != valid count(%2), display may misalign.")
            .arg(res.perPoseMeanAbsError.size()).arg(validTotal));
    }

    table->clearContents();
    table->setColumnCount(Col::COL_COUNT);
    table->setRowCount(rowCount);
    table->setHorizontalHeaderLabels({
        tr("Index"),
        tr("Check"),
        tr("X Err(mm)"),
        tr("Y Err(mm)"),
        tr("Z Err(mm)")
        });
    table->setSelectionBehavior(QAbstractItemView::SelectRows);
    table->setEditTriggers(QAbstractItemView::NoEditTriggers);
    table->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
    table->verticalHeader()->setVisible(false);

    int validCnt = 0;   // ★ 只在有效行推进，用于索引 res.perPoseMeanAbsError

    for (int i = 0; i < rowCount; ++i) {
        const bool isValid = validIndex[i];

        // 第 1 列：索引序号（始终是总索引 i）
        QTableWidgetItem* idxItem = new QTableWidgetItem();
        idxItem->setData(Qt::DisplayRole, i);
        idxItem->setTextAlignment(Qt::AlignCenter);
        table->setItem(i, COL_INDEX, idxItem);

        // 第 2 列：QCheckBox（居中）
        QWidget* cellWidget = new QWidget(table);
        QHBoxLayout* layout = new QHBoxLayout(cellWidget);
        QCheckBox* checkBox = new QCheckBox(cellWidget);
        checkBox->setChecked(isValid);
        checkBox->setProperty("poseIndex", i);   // 绑定总索引，回调时反查
        layout->addWidget(checkBox);
        layout->setAlignment(Qt::AlignCenter);
        layout->setContentsMargins(0, 0, 0, 0);
        cellWidget->setLayout(layout);
        table->setCellWidget(i, COL_CHECK, cellWidget);

        // 第 3/4/5 列：X / Y / Z 误差
        const int cols[3] = { COL_X, COL_Y, COL_Z };

        if (isValid && validCnt < static_cast<int>(res.perPoseMeanAbsError.size())) {
            // 有效行：从 res 取对应结果，validCnt 推进
            const Eigen::Vector3d& e = res.perPoseMeanAbsError[validCnt];
            const double vals[3] = { e.x(), e.y(), e.z() };
            for (int k = 0; k < 3; ++k) {
                QTableWidgetItem* item = new QTableWidgetItem();
                item->setData(Qt::DisplayRole, QString::number(vals[k], 'f', 4));
                item->setTextAlignment(Qt::AlignCenter);
                table->setItem(i, cols[k], item);
            }
            ++validCnt;
        }
        else {
            // 无效行：误差列留空占位，保持行结构对齐
            for (int k = 0; k < 3; ++k) {
                QTableWidgetItem* item = new QTableWidgetItem(QStringLiteral("-"));
                item->setTextAlignment(Qt::AlignCenter);
                item->setForeground(Qt::gray);
                table->setItem(i, cols[k], item);
            }
        }
    }
}

void HandEyeCalib::updateVtkWindow(const Eigen::Affine3f& transform, const Point3DConsistency& cons, bool handineye) {

    if (handineye) {
        printI("Hand In Eye Calibration calc done!");
    }
    else {
        printI("Hand On Eye Calibration calc done!");
    }

    Eigen::Affine3f trans;
    trans.translation() = transform.translation();
    trans.linear() = transform.rotation();
    Eigen::Quaternionf quatTcp(transform.rotation());
    quatTcp.normalize();

    printI(QString("Translation [x:%1, y:%2, z:%3] Quaternion [w:%4, x:%5, y:%6, z:%7] done.")
        .arg(trans.translation().x())
        .arg(trans.translation().y())
        .arg(trans.translation().z())
        .arg(quatTcp.w())
        .arg(quatTcp.x())
        .arg(quatTcp.y())
        .arg(quatTcp.z()));

    if (handineye) {
        if (ui->calibPanleTypeComb->currentIndex() == 2) {
            printI(QString("[EyeInHand] 3D consistency mean error[mm](in Calib panel frame) calib error:%1 mm").arg(cons.reproject_error_pixel, 0, 'f', 4));
        }
        else {
            printI(QString("[EyeInHand] 3D consistency mean error[mm](in Base frame)  X:%1  Y:%2  Z:%3  "
                "reproject pixel error %4 (valid poses:%5)")
                .arg(cons.meanAbsError.x(), 0, 'f', 4).arg(cons.meanAbsError.y(), 0, 'f', 4).arg(cons.meanAbsError.z(), 0, 'f', 4)
                .arg(cons.reproject_error_pixel, 0, 'f', 4).arg(cons.numPoses));
        }
        
    }
    else {
        printI(QString("[EyeOnHand] 3D consistency mean error[mm](in End frame)  X:%1  Y:%2  Z:%3  "
            "reproject pixel error %4 (valid poses:%5)")
            .arg(cons.meanAbsError.x(), 0, 'f', 4).arg(cons.meanAbsError.y(), 0, 'f', 4).arg(cons.meanAbsError.z(), 0, 'f', 4)
            .arg(cons.reproject_error_pixel, 0, 'f', 4).arg(cons.numPoses));
    }

    m_viewer->removeAllCoordinateSystems();

    double ratio = 1.0;
    if (ui->EyeInHandbtn->isChecked()) {
        ratio = 2.0;
    }

    double scale = 100 * ratio; // 坐标轴的长度，单位米
    m_viewer->addCoordinateSystem(scale, "coordinate_base");

    double scale_tcp = 50 * ratio; // 坐标轴的长度，单位米
    m_viewer->addCoordinateSystem(scale_tcp, trans, "coordinate_tcp");

    m_viewer->resetCamera();
    ui->qvtkWidget->renderWindow()->Render();
    ui->qvtkWidget->update();
}

Eigen::Affine3f HandEyeCalib::eye_in_hand(
    const std::string& pose_path,
    const std::string& ply_path,
    const double& diameter,
    const double& tolerance,
    Point3DConsistency& cons) {

    std::ifstream file(pose_path);
    if (!file.is_open()) {
        printE(tr("Failed to open input file (%1)!").arg(QString::fromStdString(pose_path)));
        return Eigen::Affine3f::Identity();
    }

    nlohmann::json j;
    try {
        file >> j;
        file.close();
    }
    catch (const json::parse_error& e) {
        file.close();
        printE(tr("Failed to parser input file (%1)!").arg(QString::fromStdString(pose_path)));
        return Eigen::Affine3f::Identity();
    }

    std::vector<std::string> indices;
    for (auto& element : j.items()) {
        indices.push_back(element.key());
    }
    
    RotateType rt;
    if (ui->eulerBtn->isChecked()) {
        rt = RotateType::Euler;
    }
    else {
        rt = RotateType::RotateVec;
    }

    std::vector<Eigen::Vector<float, 6>> pose_vectors;
    for (std::string key : indices) {
        if (j.contains(key) && j[key].is_array()) {
            Eigen::Vector<float, 6> pose_vec;
            for (int i = 0; i < 6; ++i) {
                pose_vec[i] = j[key].at(i).get<float>();
            }
            pose_vectors.push_back(pose_vec);
        }
    }

    if (pose_vectors.size() != indices.size()) {
        printE(tr("Input file num not equal!"));
        return Eigen::Affine3f::Identity();
    }

    int rowCount = ui->validTable->rowCount();
    std::vector<bool> keptIndices(rowCount, true);
    for (int i = 0; i < rowCount; ++i) {
        QWidget* w = ui->validTable->cellWidget(i, Col::COL_CHECK);
        if (!w) continue;
        QCheckBox* cb = w->findChild<QCheckBox*>();
        if (cb && !cb->isChecked())
            keptIndices[i] = false;
    }

    std::vector<std::string> pcd_paths;
    cv::glob(ply_path + "/*", pcd_paths, false);

    if (pcd_paths.size() != pose_vectors.size()) {
        printE("Image Number not equal to Robot Pose Number, check the input data file!");
        return Eigen::Affine3f::Identity();
    }

    ProfileScanner::HandEyeCalib hec;
    CalibType type = CalibType::EYE_IN_HAND;

    hec.SetCalibType(type);
    hec.SetRobPose(pose_vectors);

    std::vector<cv::Point3f> fea_points;
    for (auto pcd : pcd_paths) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        if (pcl::io::loadPLYFile<pcl::PointXYZ>(pcd, *cloud) == -1) {
            continue;
        }

        cv::Point3f center;
        float radius = 0.0f;
        bool ok = false;
        ok = fitCircle3D(cloud, tolerance, center, radius);

        if (ok && std::abs(radius * 2 - diameter) < 0.5f) {
            fea_points.push_back(center);
        }
    }

    if(pose_vectors.size() != fea_points.size()) {
        printE("Feature points number not equal to Robot Pose Number, check the input data file!");
        return Eigen::Affine3f::Identity();
	}
    
    CalibObj cal_model = CalibObj::SPHERE;
    hec.SetProfileData(fea_points, cal_model);

    bool sta = hec.run(ProfileScanner::SolveMethod::ITERATION);

	Eigen::Affine3f mtr = Eigen::Affine3f::Identity();
    if (sta) {
        mtr.matrix() = hec.GetCalcResult();
        cons.reproject_error_pixel = hec.CalcCalibError("Sphere");
    }

    return mtr;
}
