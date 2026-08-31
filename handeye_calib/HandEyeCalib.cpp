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

    // 与 updateTable 一致：第 0 帧不选（该行 X/Y/Z 显示 "-"）。
    // 参考点与误差只使用 i = 1 .. n-1；perPose* 按该顺序打包，长度为 n-1，
    // 这样 validIndex[0]==false 时 validCnt 从第 1 行开始对应 res[0]。
    constexpr size_t kSkipFirstPose = 1;

    void accumulateSkippedFirstPoseConsistency(
        const std::vector<eigenVector>& ptsPerPoint,
        Point3DConsistency& res)
    {
        const size_t numPts = ptsPerPoint.size();
        const size_t numPoses = (numPts == 0) ? 0 : ptsPerPoint[0].size();
        if (numPoses <= kSkipFirstPose || numPts == 0)
            return;

        res.perPointDev.assign(numPoses, std::vector<Eigen::Vector3d>(numPts, Eigen::Vector3d::Zero()));
        res.pointMeanInBase.assign(numPts, Eigen::Vector3d::Zero());

        std::vector<Eigen::Vector3d> perPoseSumAbs(numPoses, Eigen::Vector3d::Zero());
        std::vector<double> perPoseSumDist(numPoses, 0.0);

        Eigen::Vector3d sumAbs = Eigen::Vector3d::Zero();
        size_t totalSamples = 0;
        const double nRef = static_cast<double>(numPoses - kSkipFirstPose);

        for (size_t jj = 0; jj < numPts; ++jj) {
            const eigenVector& obs = ptsPerPoint[jj];
            Eigen::Vector3d mean = Eigen::Vector3d::Zero();
            for (size_t i = kSkipFirstPose; i < numPoses; ++i)
                mean += obs[i];
            mean /= nRef;
            res.pointMeanInBase[jj] = mean;

            for (size_t i = 0; i < numPoses; ++i) {
                const Eigen::Vector3d d = obs[i] - mean;
                res.perPointDev[i][jj] = d;
                if (i < kSkipFirstPose)
                    continue;
                perPoseSumAbs[i] += d.cwiseAbs();
                perPoseSumDist[i] += d.norm();
                sumAbs += d.cwiseAbs();
                ++totalSamples;
            }
        }

        if (totalSamples > 0)
            res.meanAbsError = sumAbs / static_cast<double>(totalSamples);

        const size_t nOut = numPoses - kSkipFirstPose;
        res.numPoses = static_cast<int>(nOut);
        res.perPoseMeanAbsError.resize(nOut);
        res.perPoseError.resize(nOut);
        const double invNumPts = 1.0 / static_cast<double>(numPts);
        for (size_t i = kSkipFirstPose; i < numPoses; ++i) {
            res.perPoseMeanAbsError[i - kSkipFirstPose] = perPoseSumAbs[i] * invNumPts;
            res.perPoseError[i - kSkipFirstPose] = perPoseSumDist[i] * invNumPts;
        }
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

    std::vector<std::string> imagePaths;
    cv::glob(ImageFolderPath.toStdString() + "/*", imagePaths, false);

    Point3DConsistency tempCons;
    tempCons.perPoseMeanAbsError.resize(imagePaths.size(), Eigen::Vector3d(0.0, 0.0, 0.0));
    std::vector<bool> validIndex(imagePaths.size(), true);
    updateTable(ui->validTable, validIndex, tempCons);
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
    bool symmetric = typeCalibrationPanel == 1;

    int squareX = ui->calibPanelXSpin->value();
    int squareY = ui->calibPanelYSpin->value();

    double squareInter = ui->P2PDisSpin->value();
    Point3DConsistency calibRet;
    if (handInEye) {
        transform = eye_in_hand(PoseFilePath.toStdString(), ImageFolderPath.toStdString(), squareInter, squareX, squareY,
            chess_board, symmetric, "./config/Calibration/", calibRet);
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

Eigen::Affine3d HandEyeCalib::estimateHandEye(const EigenAffineVector& baseToTip,
    const EigenAffineVector& camToTag) {

    auto t1_it = baseToTip.begin();
    auto t2_it = camToTag.begin();

    Eigen::Affine3d firstEEInverse, firstCamInverse;
    eigenVector tvecsArm, rvecsArm, tvecsFiducial, rvecsFiducial;

    bool firstTransform = true;

    for (int i = 0; i < baseToTip.size(); ++i, ++t1_it, ++t2_it) {
        auto& eigenEE = *t1_it;
        auto& eigenCam = *t2_it;
        if (firstTransform) {
            firstEEInverse = eigenEE.inverse();
            firstCamInverse = eigenCam.inverse();
            firstTransform = false;
        }
        else {
            Eigen::Affine3d robotTipinFirstTipBase = firstEEInverse * eigenEE;
            Eigen::Affine3d fiducialInFirstFiducialBase = firstCamInverse * eigenCam;

            rvecsArm.push_back(eigenRotToEigenVector3dAngleAxis(robotTipinFirstTipBase.rotation()));
            tvecsArm.push_back(robotTipinFirstTipBase.translation());

            rvecsFiducial.push_back(eigenRotToEigenVector3dAngleAxis(fiducialInFirstFiducialBase.rotation()));
            tvecsFiducial.push_back(fiducialInFirstFiducialBase.translation());
        }
    }

    HandEye::HandEyeCalibration calib;
    Eigen::Matrix4d result;
    calib.estimateHandEyeScrew(rvecsArm, tvecsArm, rvecsFiducial, tvecsFiducial, result, false);

    Eigen::Transform<double, 3, Eigen::Affine> resultAffine(result);
    Eigen::Quaternion<double> quaternionResult(resultAffine.rotation());

    return resultAffine;
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
    else if (ui->quatBtn->isChecked()) {
        rt = RotateType::Quat;
    }
    else {
        rt = RotateType::RotateVec;
    }

    /*for (const auto& element : j) {
        std::vector<double> pose_vec;
        switch (rt)
        {
        case HandEyeCalib::RotateType::Euler:
        case HandEyeCalib::RotateType::RotateVec:
            pose_vec.push_back(element["x"].get<double>());
            pose_vec.push_back(element["y"].get<double>());
            pose_vec.push_back(element["z"].get<double>());
            pose_vec.push_back(element["rx"].get<double>());
            pose_vec.push_back(element["ry"].get<double>());
            pose_vec.push_back(element["rz"].get<double>());
            break;
        case HandEyeCalib::RotateType::Quat:
            pose_vec.push_back(element["x"].get<double>());
            pose_vec.push_back(element["y"].get<double>());
            pose_vec.push_back(element["z"].get<double>());
            pose_vec.push_back(element["qw"].get<double>());
            pose_vec.push_back(element["qx"].get<double>());
            pose_vec.push_back(element["qy"].get<double>());
            pose_vec.push_back(element["qz"].get<double>());
            break;
        default:
            break;
        }
        pose_vectors.push_back(pose_vec);
    }*/

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

    // 生成3D世界坐标
    std::vector<cv::Point3f> objp;
    GenerateW3DPoints(symmetric, chess_board, pattern_x, pattern_y, square_size, objp);

    std::vector<std::vector<cv::Point3f>> obj_points_list;
    std::vector<std::vector<cv::Point2f>> img_points_list;
    std::vector<std::vector<double>> filtered_pose_vectors; 
    int det_success_num = 0;
    cv::Mat gray; // 声明在循环外，用于获取图像尺寸

    for (size_t i = 0; i < image_paths.size(); ++i) {
        // 第 0 帧始终采集（评估函数内部再按 updateTable 不选），其余尊重勾选
        if (i != 0 && i < keptIndices.size() && !keptIndices[i])
            continue;

        cv::Mat img = cv::imread(image_paths[i]);
        if (img.empty()) {
            if (i < keptIndices.size()) keptIndices[i] = false;
            continue;
        }

        double quality_score = AssessImageQuality(img);
        double quality_threshold = 20.0;

        if (quality_score < quality_threshold) {
            printW(tr("Image Quality low!"));
            if (i < keptIndices.size()) keptIndices[i] = false;
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
            filtered_pose_vectors.push_back(pose_vectors[i]); // 修正：类型匹配

            cv::drawChessboardCorners(img, pattern_size, corners, found);
            cv::imwrite(save_path + "/calibed_image/CalibImage_" + std::to_string(i) + ".png", img);
        }
        else {
            keptIndices[i] = false;
        }
    }

    // --- 相机标定 ---
    cv::Mat camera_matrix, dist_coeffs;
    std::vector<cv::Mat> rvecs, tvecs;
    double ret = cv::calibrateCamera(obj_points_list, img_points_list, gray.size(), camera_matrix, dist_coeffs, rvecs, tvecs);

    // --- 求解标定板在相机坐标系中的位姿 ---
    std::vector<cv::Mat> R_board2cameras;
    std::vector<cv::Mat> t_board2cameras;

    auto cam2target_it = std::back_inserter(cam2target);

    for (size_t i = 0; i < obj_points_list.size(); ++i) {
        cv::Mat rvec, tvec;
        bool success = cv::solvePnP(obj_points_list[i], img_points_list[i], camera_matrix, dist_coeffs, rvec, tvec);
        if (success) {
            cv::Mat R_board2camera;
            cv::Rodrigues(rvec, R_board2camera);
            R_board2cameras.push_back(R_board2camera);
            t_board2cameras.push_back(tvec);
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

    /*auto base2end_it = std::back_inserter(base2end);
    MathUtils mu;
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
            matrix = mu.MatrixfromEuler(order.toStdString(), euler_rad[2], euler_rad[1], euler_rad[0]);
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
        Eigen::Affine3d transform = Eigen::Affine3d::Identity();
        transform.linear() = matrix;
        transform.translation() = pos;
        *base2end_it = transform.inverse();
    }

    if (cam2target.size() != base2end.size()) {
        printE(tr("Image num is not equal to Pose num!"));
        return Eigen::Affine3f::Identity();
    }

    Eigen::Affine3d trans = estimateHandEye(base2end, cam2target);

    base2end.clear();
    cam2target.clear();

    return trans.cast<float>();*/

    // 原始位姿与眼在手上相同：end->base（gripper2base）。
    // OpenCV 眼在手外需要 base->end，评估函数也接收 end->base 并在内部取逆。
    MathUtils mu;
    std::vector<cv::Mat> R_end2bases;
    std::vector<cv::Mat> t_end2bases;
    std::vector<cv::Mat> R_base2ends;
    std::vector<cv::Mat> t_base2ends;
    for (const auto& pose_vec : filtered_pose_vectors) { // 修正：类型匹配
        /*cv::Vec3d euler_rpy_deg(pose_vec[3], pose_vec[4], pose_vec[5]);
        cv::Vec3d euler_rpy_rad = euler_rpy_deg * M_PI / 180.0;
        cv::Mat R_base2end = Euler2CvRotationMatrix(euler_rpy_rad);*/

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

        cv::Mat R_end2base = cv::Mat::eye(3, 3, CV_64F);
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                R_end2base.at<double>(i, j) = matrix.matrix()(i, j);
            }
        }

        cv::Mat t_end2base = (cv::Mat_<double>(3, 1) << pose_vec[0], pose_vec[1], pose_vec[2]);
        R_end2bases.push_back(R_end2base);
        t_end2bases.push_back(t_end2base);

        cv::Mat R_base2end = R_end2base.t();
        cv::Mat t_base2end = -R_base2end * t_end2base;
        R_base2ends.push_back(R_base2end);
        t_base2ends.push_back(t_base2end);
    }

    cv::Mat R_camera2base, t_camera2base;
    cv::calibrateHandEye(R_base2ends, t_base2ends, R_board2cameras, t_board2cameras, R_camera2base, t_camera2base, cv::CALIB_HAND_EYE_TSAI);

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
        R_end2bases, t_end2bases,   // 原始 end->base，与 EyeInHand 相同
        T_camera2base, objp);
    cons.reproject_error_pixel = total_error / static_cast<double>(obj_points_list.size());

    Eigen::Affine3d trans;
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            trans.matrix()(i, j) = T_camera2base.at<double>(i, j);
        }
    }

    // 与评估函数一致：第 0 帧不选，updateTable 该行 X/Y/Z 显示 "-"
    if (!keptIndices.empty())
        keptIndices[0] = false;
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
    else if (ui->quatBtn->isChecked()) {
        rt = RotateType::Quat;
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

        // 第 0 帧始终采集（评估函数内部再按 updateTable 不选），其余尊重勾选
        if (i != 0 && i < keptIndices.size() && !keptIndices[i])
            continue;

        cv::Mat img = cv::imread(image_paths[i]);
        if (img.empty()) {
            if (i < keptIndices.size()) keptIndices[i] = false;
            continue;
        }

        double quality_score = AssessImageQuality(img);
        double quality_threshold = 20.0;

        if (quality_score < quality_threshold) {
            printW("Image Quality Low, Pass away!");
            if (i < keptIndices.size()) keptIndices[i] = false;
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

    //auto cam2target_it = std::back_inserter(cam2target);
    for (size_t i = 0; i < obj_points_list.size(); ++i) {
        cv::Mat rvec, tvec;
        bool success = cv::solvePnP(obj_points_list[i], img_points_list[i], camera_matrix, dist_coeffs, rvec, tvec);
        if (success) {
            cv::Mat R_board2camera;
            cv::Rodrigues(rvec, R_board2camera);

            R_board2cameras.push_back(R_board2camera);
            t_board2cameras.push_back(tvec);

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
        R_end2bases.push_back(R_end);
        t_end2bases.push_back(t_end);
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

    // 与评估函数一致：第 0 帧不选，updateTable 该行 X/Y/Z 显示 "-"
    if (!keptIndices.empty())
        keptIndices[0] = false;
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
    Point3DConsistency res;
    const size_t numPoses = R_board2cams.size();
    const size_t numPts = objp.size();
    res.numPoses = static_cast<int>(numPoses);

    // 第 0 帧按 updateTable 不选，至少还需要 2 帧才能评估
    if (numPoses < 2 + kSkipFirstPose || numPts == 0) {
        printW(tr("3D consistency eval skipped: need >=3 poses (first unused) and >=1 point."));
        return res;
    }

    const Eigen::Affine3d Tcam2end = cv4x4ToAffine(T_cam2end);

    // ptsInBase[j][i] = 第 j 个点在第 i 个位姿下反算到基座系的坐标
    std::vector<eigenVector> ptsInBase(numPts);
    for (size_t jj = 0; jj < numPts; ++jj)
        ptsInBase[jj].reserve(numPoses);

    for (size_t i = 0; i < numPoses; ++i) {
        const Eigen::Affine3d Tboard2cam = cvRt2Affine(R_board2cams[i], t_board2cams[i]);
        const Eigen::Affine3d Tend2base = cvRt2Affine(R_end2bases[i], t_end2bases[i]);
        const Eigen::Affine3d Tboard2base = Tend2base * Tcam2end * Tboard2cam;

        for (size_t jj = 0; jj < numPts; ++jj) {
            Eigen::Vector3d p_board(objp[jj].x, objp[jj].y, objp[jj].z);
            ptsInBase[jj].push_back(Tboard2base * p_board);
        }
    }

    accumulateSkippedFirstPoseConsistency(ptsInBase, res);
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
    // 眼在手外：标定板固连末端，必须变到末端系再比一致性。
    // p_end = T_end2base^{-1} * T_cam2base * T_board2cam * p_board
    // 第 0 帧按 updateTable 不选，与 EyeInHand 相同打包方式。
    Point3DConsistency res;
    const size_t numPoses = R_board2cams.size();
    const size_t numPts = objp.size();
    res.numPoses = static_cast<int>(numPoses);

    if (numPoses < 2 + kSkipFirstPose || numPts == 0) {
        printW(tr("3D consistency eval skipped: need >=3 poses (first unused) and >=1 point."));
        return res;
    }

    const Eigen::Affine3d Tcam2base = cv4x4ToAffine(T_cam2base);

    std::vector<eigenVector> ptsInEnd(numPts);
    for (size_t jj = 0; jj < numPts; ++jj)
        ptsInEnd[jj].reserve(numPoses);

    for (size_t i = 0; i < numPoses; ++i) {
        const Eigen::Affine3d Tboard2cam = cvRt2Affine(R_board2cams[i], t_board2cams[i]);
        const Eigen::Affine3d Tend2base = cvRt2Affine(R_end2bases[i], t_end2bases[i]);
        const Eigen::Affine3d Tboard2end = Tend2base.inverse() * Tcam2base * Tboard2cam;

        for (size_t jj = 0; jj < numPts; ++jj) {
            Eigen::Vector3d p_board(objp[jj].x, objp[jj].y, objp[jj].z);
            ptsInEnd[jj].push_back(Tboard2end * p_board);
        }
    }

    accumulateSkippedFirstPoseConsistency(ptsInEnd, res);
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
        printI(QString("[EyeInHand] 3D consistency mean error[mm](in Calib panel frame)  X:%1  Y:%2  Z:%3  "
            "reproject pixel error %4 (valid poses:%5)")
            .arg(cons.meanAbsError.x(), 0, 'f', 4).arg(cons.meanAbsError.y(), 0, 'f', 4).arg(cons.meanAbsError.z(), 0, 'f', 4)
            .arg(cons.reproject_error_pixel, 0, 'f', 4).arg(cons.numPoses));
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