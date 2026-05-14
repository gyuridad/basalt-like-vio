
 #pragma once
 #include <Eigen/Core>    


namespace basalt_like_vio {
namespace calibration {

struct CameraIntrinsics {
    double fx = 0.0;
    double fy = 0.0;
    double cx = 0.0;
    double cy = 0.0;

    int width = 0;
    int height = 0;

    CameraIntrinsics() = default;

    CameraIntrinsics(
        double fx_,
        double fy_,
        double cx_,
        double cy_,
        int width_,
        int height_
    ):
        fx(fx_),
        fy(fy_),
        cx(cx_),
        cy(cy_),
        width(width_),
        height(height_) {}

        
    // 카메라 좌표계의 3D 점 p_c = [X, Y, Z]를 이미지 픽셀 좌표 [u, v]로 변환하는 함수
    Eigen::Vector2d project(const Eigen::Vector3d& p_c) const {

        const double x = p_c.x() / p_c.z();
        const double y = p_c.y() / p_c.z();

        return Eigen::Vector2d(
            fx * x + cx,
            fy * y + cy);
    }

    // 픽셀 좌표 uv = [u, v]가 이미지 안쪽에 있는지 검사하는 함수
    Eigen::Vector3d unproject(const Eigen::Vector2d& uv) const {
        const double x = (uv.x() - cx) / fx;
        const double y = (uv.y() - cy) / fy;

        return Eigen::Vector3d(x, y, 1.0);
    }

    bool isInsideImage(const Eigen::Vector2d& uv, double border = 0.0) const {
        return uv.x() >= border &&
               uv.y() >= border &&
               uv.x() < static_cast<double>(width) - border &&
               uv.y() < static_cast<double>(height) - border;
    }
};

}  // namespace calibration
}  // namespace basalt_like_vio




