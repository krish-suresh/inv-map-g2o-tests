#include <g2o/types/slam3d/se3quat.h>

#include <Eigen/Core>
#include <chrono>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>


class PoseStamped : public g2o::SE3Quat {
   public:
    double _time;
    int _id;
    PoseStamped();
    PoseStamped(const Eigen::Quaterniond& q, const Eigen::Vector3d& t,
                const double& time, const int& id = 0)
        : _time(time), _id(id), SE3Quat(q,t){}
};

class RgbSlam {
   public:
    std::vector<PoseStamped> poses;
    RgbSlam(const std::string& file_path) {
        std::ifstream f(file_path);
        if (!f) throw std::runtime_error("couldn't open 'source' for reading");
        Eigen::Vector3d t;
        Eigen::Vector4d r;
        std::string time;
        int id;
        while (f >> time >> t[0] >> t[1] >> t[2] >> r[0] >> r[1] >> r[2] >> r[3] >>
               id) {
            poses.push_back(
                PoseStamped(Eigen::Quaterniond(r[3], r[0], r[1], r[2]), t,
                            std::stod(time), id));
        }
        f.close();
    }
};

int main(void) {
    RgbSlam rtabmap_data("poses.txt");
    for (auto& pose : rtabmap_data.poses) {
        std::cout << pose._id << " | " << pose.toMinimalVector() <<"\n";
        std::cout << pose._time << "\n";
        break;
    }
}