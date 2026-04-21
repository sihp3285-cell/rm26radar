#include "include/posesolver.hpp"
#include "include/raycaster.hpp"
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

void PoseSolver::getExtrinsic(cv::Mat& R_out, cv::Mat& T_out) const
{
    R_out = this->R.clone();
    T_out = this->T.clone();
}

cv::Point2f PoseSolver::middletoworld(const cv::Rect& box)
{
    if (!isPoseEstimated) return cv::Point2f(0, 0);
    std::vector<cv::Point2f> middle = {cv::Point2f(box.x + box.width / 2.0f, box.y + box.height)};
    cv::Point3f worldPoint = raycaster_.pixelToWorld(middle[0], K, D, R, T);
    return {worldPoint.x, worldPoint.z};
}