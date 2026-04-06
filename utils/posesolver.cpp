#include "include/posesolver.hpp"
//初始化
PoseSolver::PoseSolver(const cv::Mat& camMat, const cv::Mat& disMat)
{
    this->K = camMat.clone();
    this->D = disMat.clone();
}
void PoseSolver::calibrate(const std::vector<cv::Point3f>& objectPoints, const std::vector<cv::Point2f>& imagePoints)
{
    cv::Mat rvec, tvec;
    cv::solvePnP(objectPoints, imagePoints, K, D, rvec, tvec);
    cv::Mat R_mat;
    cv::Rodrigues(rvec, R_mat);
    this->R = R_mat.t();
    this->T = -this->R * tvec;
    this->isPoseEstimated = true;
}

cv::Point2f PoseSolver::middletoworld(const cv::Rect& box)
{
    if (!isPoseEstimated) return cv::Point2f(0, 0);
    std::vector<cv::Point2f> middle = {cv::Point2f(box.x + box.width / 2.0f, box.y + box.height)};
    std::vector<cv::Point2f> normmiddle;
    cv::undistortPoints(middle, normmiddle, K, D);
    cv::Mat Vc = (cv::Mat_<double>(3,1) << normmiddle[0].x, normmiddle[0].y, 1.0);
    cv::Mat Vw = this->R * Vc;
    double S = -this->T.at<double>(2) / Vw.at<double>(2, 0);
    double worldX = this->T.at<double>(0) + S * Vw.at<double>(0, 0);
    double worldY = this->T.at<double>(1) + S * Vw.at<double>(1, 0);
    return cv::Point2f(worldX, worldY);
}
