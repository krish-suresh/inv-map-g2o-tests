#include <g2o/config.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/solver.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/solvers/structure_only/structure_only_solver.h>
#include <g2o/types/sba/edge_project_p2mc.h>
#include <g2o/types/sba/edge_sba_cam.h>
#include <g2o/types/sba/vertex_cam.h>
#include <g2o/types/slam3d/vertex_pointxyz.h>
#include <g2o/types/sba/edge_sbacam_gravity.h>

#include <fstream>
#include <nlohmann/json.hpp>

using json = nlohmann::json;
using namespace g2o;
using namespace Eigen;


typedef struct tag_obs {
    int tag_id;
    std::array<double, 8> pixel_obs;
    std::array<double, 4> cam_int;
    SE3Quat tag_pose;
} TagObs;

int main(int argc, char const* argv[]) {
    // Setting up tag corners in Tag's Frame
    double half_tag_width = 0.173 / 2;
    Vector3d tl(-half_tag_width, half_tag_width, 0);
    Vector3d tr(half_tag_width, half_tag_width, 0);
    Vector3d bl(-half_tag_width, -half_tag_width, 0);
    Vector3d br(half_tag_width, -half_tag_width, 0);
    std::array<Vector3d, 4> corners = {bl, br, tr, tl};

    // Tag origin in tag's frame
    Eigen::Quaterniond q_no;
    q_no.setIdentity();
    Eigen::Isometry3d corner_pose;
    corner_pose = q_no;

    // Tag corner poses
    std::map<int, std::array<Eigen::Isometry3d, 4>> tag_poses;
    // Tag center poses
    std::map<int, SE3Quat> tag_c_poses;

    // Set up optimizer
    g2o::SparseOptimizer optimizer;
    optimizer.setVerbose(false);
    std::unique_ptr<g2o::BlockSolver_6_3::LinearSolverType> linearSolver;
    linearSolver = g2o::make_unique<
        g2o::LinearSolverDense<g2o::BlockSolver_6_3::PoseMatrixType>>();

    g2o::OptimizationAlgorithmLevenberg* solver =
        new g2o::OptimizationAlgorithmLevenberg(
            g2o::make_unique<g2o::BlockSolver_6_3>(std::move(linearSolver)));

    optimizer.setAlgorithm(solver);

    // Load Map
    std::ifstream f("map.json");
    json data = json::parse(f);
    auto tags = data["tag_vertices"];
    int v_i = 0; // Vertex index
    int e_i = 0; // Edge index
    for (auto tag : tags) {
        auto q_x = tag["rotation"]["x"].get<double>();
        auto q_y = tag["rotation"]["y"].get<double>();
        auto q_z = tag["rotation"]["z"].get<double>();
        auto q_w = tag["rotation"]["w"].get<double>();
        auto x = tag["translation"]["x"].get<double>() + 70;
        auto y = tag["translation"]["y"].get<double>();
        auto z = tag["translation"]["z"].get<double>();
        auto tag_id = tag["id"].get<int>();
        std::cout << "tag: " << tag_id << "\t" << x << "," << y << "," << z
                  << ","
                  << "\n";

        // Tag Center Pose
        Vector3d trans(x, y, z);
        Eigen::Quaterniond q(q_w, q_x, q_y, q_z);
        Eigen::Isometry3d tag_pose;
        tag_pose = q;
        tag_pose.translation() = trans;
        tag_c_poses[tag_id] = SE3Quat(q, trans);

        // Calculate and store tag corners
        std::array<Eigen::Isometry3d, 4> corner_poses;
        int i = 0;
        for (auto corner : corners) {
            corner_pose.translation() = corner;
            auto c_pose = tag_pose * corner_pose;
            corner_poses[i] = c_pose;
            i++;
        }
        tag_poses[tag_id] = corner_poses;
    }

    // Load Route
    std::ifstream f_nav("krishnaRaiyanPaul.json");
    json data_nav = json::parse(f_nav);
    auto pose_data = data_nav["pose_data"];
    auto tag_data = data_nav["tag_data"];

    std::map<int, std::vector<TagObs>> tag_obs_map; // Pose/odomid to tag observation
    std::map<int, std::array<VertexPointXYZ*, 4>> tag_vertices; // tag id to tag corner vertexxyz
    // Load tags into map between poseid and tag obs
    for (auto tags : tag_data) {
        for (auto tag_obs : tags) {
            auto tag_id = tag_obs["tag_id"].get<int>();
            if (tag_vertices.find(tag_id) == tag_vertices.end()) {
                std::array<VertexPointXYZ*, 4> corners;
                int i = 0;
                if (tag_poses.find(tag_id) != tag_poses.end()){
                    for (auto c_pose : tag_poses[tag_id]) {
                        std::cout << "tag vert added: "<< tag_id<<"\n";
                        VertexPointXYZ* tag_corner = new VertexPointXYZ();
                        tag_corner->setEstimate(c_pose.translation());
                        tag_corner->setFixed(true);
                        tag_corner->setId(v_i);
                        optimizer.addVertex(tag_corner);
                        v_i++;
                        corners[i] = tag_corner;
                        i++;
                    }
                    tag_vertices[tag_id] = corners;
                }
            }
            auto raw_pose = tag_obs["tag_pose"].get<std::array<double, 16>>();
            // 0  1  2  3
            // 4  5  6  7
            // 8  9  10 11
            // 12 13 14 15
            Matrix3d rot_mat;
            rot_mat(0, 0) = raw_pose[0];
            rot_mat(1, 0) = raw_pose[4];
            rot_mat(2, 0) = raw_pose[8];
            rot_mat(0, 1) = raw_pose[1];
            rot_mat(1, 1) = raw_pose[5];
            rot_mat(2, 1) = raw_pose[9];
            rot_mat(0, 2) = raw_pose[2];
            rot_mat(1, 2) = raw_pose[6];
            rot_mat(2, 2) = raw_pose[10];
            Vector3d translation_vec(raw_pose[3], raw_pose[7], raw_pose[11]);
            TagObs obs;
            obs.tag_id = tag_obs["tag_id"].get<int>();
            obs.cam_int =
                tag_obs["camera_intrinsics"].get<std::array<double, 4>>();
            obs.pixel_obs = tag_obs["tag_corners_pixel_coordinates"]
                                .get<std::array<double, 8>>();
            obs.tag_pose = SE3Quat(rot_mat, translation_vec);
            tag_obs_map[tag_obs["pose_id"].get<int>()].push_back(obs);
            // std::cout << "Added Tag Obs tid: " << tag_id
            //           << " poseid: " << tag_obs["pose_id"].get<int>() <<
            //           "\n";
        }
    }


    std::cout << "TAGS ADDED\n";
    // Loop through poses, check if tag obs exists, if true
    VertexCam* prev_pose = nullptr;
    SE3Quat odom_adjust; // base transformation to adjust all odom to first tag obs transformation

    // Calculate cam to phone transform
    Matrix3d ctoprot;
    ctoprot << 1, 0, 0, 0, -1, 0, 0, 0, -1;
    SE3Quat cam_to_phone(ctoprot, Vector3(0, 0, 0));


    bool det_first_tag = false; // Used to set when first tag is detected
    for (int i = 0; i < pose_data.size(); i++) {
        auto odom_data = pose_data[i];
        auto raw_pose = odom_data["pose"].get<std::array<double, 16>>();
        auto odom_id = odom_data["id"].get<int>();
        // 0  4  8  12
        // 1  5  9  13
        // 2  6  10 14
        // 3  7  11 15
        Matrix3d rot_mat;
        rot_mat(0, 0) = raw_pose[0];
        rot_mat(1, 0) = raw_pose[1];
        rot_mat(2, 0) = raw_pose[2];
        rot_mat(0, 1) = raw_pose[4];
        rot_mat(1, 1) = raw_pose[5];
        rot_mat(2, 1) = raw_pose[6];
        rot_mat(0, 2) = raw_pose[8];
        rot_mat(1, 2) = raw_pose[9];
        rot_mat(2, 2) = raw_pose[10];
        Vector3d translation_vec(raw_pose[12], raw_pose[13], raw_pose[14]);
        // phone_to_odom
        SE3Quat pose_se3 = SE3Quat(rot_mat, translation_vec) * cam_to_phone;

        if (!det_first_tag && tag_obs_map.find(odom_id) == tag_obs_map.end()) {
            continue;// Skip all first odom until first tag is seen
        } else if (!det_first_tag) {
            auto first_tag = tag_obs_map[odom_id][0];
            // Find the world adjust transformation
            auto tag_to_cam = first_tag.tag_pose;
            auto cam_to_odom = pose_se3;
            auto tag_to_odom = cam_to_odom * tag_to_cam;
            auto world_to_tag = tag_c_poses[first_tag.tag_id].inverse();
            odom_adjust = (tag_to_odom * world_to_tag).inverse();
            det_first_tag = true;
        }
        pose_se3 = odom_adjust * pose_se3; // Adjust raw camera pose to map localization
        SBACam odom_pose = SBACam(pose_se3);

        // if camera observation exists set camera intrinsics for SBA
        if (tag_obs_map.find(odom_id) != tag_obs_map.end()) {
            auto obs = tag_obs_map[odom_id][0];
            odom_pose.setKcam(obs.cam_int[0], obs.cam_int[1], obs.cam_int[2],
                              obs.cam_int[3], 1);
        } else {
            odom_pose.baseline = 0;
        }
        VertexCam* cur_pose = new VertexCam();
        cur_pose->setEstimate(odom_pose);
        cur_pose->setId(v_i);
        optimizer.addVertex(cur_pose);
        v_i++;

        // ***** START Gravity Edge
        Matrix<double, 6, 1> m;
        // Up vector in robot frame
        m.head<3>() = Vector3d::UnitY();
        // Observed Gravity vector in world frame
        // TODO shouldn't this be from the sensor measurement?
        auto eig_est = odom_pose.rotation().toRotationMatrix().transpose() * -Vector3d::UnitY();
		Eigen::Vector3d ea;

		// Transform pose from camera frame to world frame
		Eigen::Matrix3d t = odom_pose.rotation().toRotationMatrix();
		ea[0] = atan2(t (2, 1), t (2, 2));
		ea[1] = asin(-t (2, 0));
		ea[2] = atan2(t (1, 0), t (0, 0));

		Eigen::Matrix3d rot =
		   (Eigen::AngleAxisd(ea[1], Eigen::Vector3d::UnitY()) *
			Eigen::AngleAxisd(ea[0], Eigen::Vector3d::UnitX())).toRotationMatrix();

		Eigen::Vector3d estimate = rot * -Vector3d::UnitY();
        // std::cout <<"A: \n";
        // std::cout << eig_est << "\n";
        // std::cout <<"B: \n";
        // std::cout << estimate << "\n";
        m.tail<3>() = estimate;
        EdgeSBACamGravity* e_g = new EdgeSBACamGravity();
        e_g->setVertex(0, cur_pose);
        e_g->setInformation(MatrixXd::Identity(3, 3));
        e_g->setMeasurement(m);
        e_g->setId(e_i);
        optimizer.addEdge(e_g);
        e_i++;
        // END Gravity Edge

        // Add tag corner observations
        if (tag_obs_map.find(odom_id) != tag_obs_map.end()) {
            auto tags = tag_obs_map[odom_id];
            for (auto obs : tags) {
                if (tag_vertices.find(obs.tag_id) == tag_vertices.end()) {
                    continue;
                }
                int k = 0;
                for (auto corner_vert : tag_vertices[obs.tag_id]) {
                    EdgeProjectP2MC* e_p = new EdgeProjectP2MC();
                    e_p->setVertex(1, cur_pose);
                    e_p->setVertex(0, corner_vert);
                    e_p->setMeasurement(
                        Vector2(obs.pixel_obs[k], obs.pixel_obs[k + 1]));
                    e_p->setInformation(Matrix2d::Identity()*0.0001);
                    e_p->setId(e_i);
                    optimizer.addEdge(e_p);
                    e_i++;
                    // std::cout << obs.pixel_obs[k] << " " << obs.pixel_obs[k +
                    // 1] << "\n";
                    k += 2;
                    // g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                    // // rk->setDelta(5);
                    // e_p->setRobustKernel(rk);
                    e_p->computeError();
                    // std::cout << "Edge 1 " << e_p->error() << "\n";
                    // std::cout << e_p->chi2() << "\n";
                    // std::cout <<"est\n";
                    // std::cout << cur_pose->estimate() <<"\n";
                    // std::cout <<"w2n\n";
                    // std::cout << cur_pose->estimate().w2n <<"\n";
                    // std::cout <<"w2i\n";
                    // std::cout << cur_pose->estimate().w2i <<"\n";
                }
            }
        }

        // Add Odom edges
        if (prev_pose == nullptr) {
            prev_pose = cur_pose;
            continue;
        }
        EdgeSBACam* e = new EdgeSBACam();
        e->setVertex(0, prev_pose);
        e->setVertex(1, cur_pose);

        // cur pose and each of the corner poses
        e->setMeasurement(prev_pose->estimate().inverse() *
                          cur_pose->estimate());
        auto info = MatrixXd(6, 6);
        info(0, 0) = 1/10;
        info(1, 1) = 1/10;
        info(2, 2) = 1/10;
        info(3, 3) = 1;// / 100;
        info(4, 4) = 1;// / 100;
        info(5, 5) = 1;// / 100;
        e->setInformation(info);
        e->setId(e_i);
        optimizer.addEdge(e);
        // g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
        // e->setRobustKernel(rk);
        prev_pose = cur_pose;
        e_i++;
    }

    optimizer.save("test.g2o");
    std::cout << "SAVE"
              << "\n";
    optimizer.load("test.g2o");
    // optimizer.setVerbose(true);
    // optimizer.initializeOptimization();
    // std::cout << "Initialize" << "\n";
    // optimizer.optimize(100);
    return 0;
}
