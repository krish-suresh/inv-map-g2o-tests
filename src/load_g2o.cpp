
#include <g2o/config.h>
#include <g2o/core/optimizable_graph.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/types/sba/edge_project_p2mc.h>
#include <g2o/types/sba/edge_sba_cam.h>
#include <g2o/types/sba/vertex_cam.h>
#include <g2o/types/slam3d/vertex_pointxyz.h>
#include <g2o/types/sba/edge_sbacam_gravity.h>
#include <fstream>

using namespace g2o;
using namespace Eigen;

int main(int argc, char const *argv[])
{
    std::ifstream f("test.g2o");
    g2o::SparseOptimizer graph;
    graph.load(f);
    for (auto &v : graph.vertices()){ 
        std::cout << v.second->id() << "\n";
    }
    return 0;
}
