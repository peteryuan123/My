//
// Created by mpl on 22-9-2.
//

#include <g2o/types/sim3/sim3.h>
#include <g2o/types/sim3/types_seven_dof_expmap.h>

#include <g2o/types/slam3d/vertex_se3.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/core/solver.h>

#include <fstream>

typedef g2o::BlockSolver<g2o::BlockSolverTraits<7, 7>> BlockSolverType; //posedim, landmarkDim
typedef g2o::LinearSolverEigen<BlockSolverType::PoseMatrixType> LinearSolverType;

int main(int argc, char** argv)
{
    std::string node_file = argv[1];
    std::string edge_file = argv[2];

    auto solver = new g2o::OptimizationAlgorithmLevenberg(
            g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType>())
            );
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);
    optimizer.setVerbose(true);


    std::ifstream node_src, edge_src;
    node_src.open(node_file);
    edge_src.open(edge_file);

    int numNodes, numEdges;
    node_src >> numNodes;
    edge_src >> numEdges;

    int min_id = INT32_MAX, max_id = 0;

    std::cout <<"Nodes...\n";
    for (int i = 0; i < numNodes; i++)
    {
        double id, x, y, z, qx, qy, qz, qw, s;
        node_src >> id >> qw >> qx >> qy >> qz >> x >> y >> z >> s;
        std::cout <<  id << " " << qw << " " << qx << " " << qy << " " << qz << " " << x << " " << y << " " << z <<std::endl;

        g2o::Sim3 n_value(g2o::Quaternion(qw, qx, qy, qz), g2o::Vector3(x, y, z), s);
        std::cout << n_value.log().transpose() << std::endl;
        g2o::VertexSim3Expmap* v_sim3 = new g2o::VertexSim3Expmap();
        v_sim3->setId(id);
        v_sim3->setMarginalized(false);
        v_sim3->setEstimate(n_value);

        if (id == 0)
        {
            v_sim3->setFixed(true);
            v_sim3->_fix_scale = true;
        }

        if (id < min_id)
            min_id = id;
        if (id > max_id)
            max_id = id;

        optimizer.addVertex(v_sim3);
    }

    std::cout <<"Edges...\n";

    for (int i = 0; i < numEdges; i++)
    {
        double id0, id1, s, x, y, z, qx, qy, qz, qw;
        edge_src >> id0 >> id1  >> qw >> qx >> qy >> qz >> x >> y >> z >> s;
        std::cout << id0 << " " << id1 << " " << s << " " << qw << " " << qx << " " << qy << " " << qz <<
                  " " << x << " " << y << " " << z << std::endl;

        g2o::EdgeSim3* e_sim3 = new g2o::EdgeSim3();
        g2o::Sim3 e_value(g2o::Quaternion(qw, qx, qy, qz), g2o::Vector3(x, y, z), s);

        e_sim3->setId(i);
        e_sim3->setVertex(0, optimizer.vertex(id0));
        e_sim3->setVertex(1, optimizer.vertex(id1));
        e_sim3->information() = Eigen::Matrix<double, 7, 7>::Identity();
        e_sim3->setMeasurement(e_value);

        optimizer.addEdge(e_sim3);
    }

    node_src.close();
    edge_src.close();




    optimizer.initializeOptimization();
    optimizer.optimize(30);

//    for (auto it = optimizer.vertices().begin(); it != optimizer.vertices().end(); it++)
//    {
//        g2o::VertexSim3Expmap* v_sim3 =
//                static_cast<g2o::VertexSim3Expmap*>(it->second);
//        std::cout
//                  << v_sim3->estimate().rotation().w() << " "
//                  << v_sim3->estimate().rotation().x() << " "
//                  << v_sim3->estimate().rotation().y() << " "
//                  << v_sim3->estimate().rotation().z() << " "
//                  << v_sim3->estimate().translation().transpose() << " "
//                  << v_sim3->estimate().scale() << std::endl;
//    }

    for (int i = min_id; i <= max_id; i++)
    {
        if (optimizer.vertices().find(i) != optimizer.vertices().end())
        {
            g2o::VertexSim3Expmap* v_sim3 =
                    static_cast<g2o::VertexSim3Expmap*>(optimizer.vertices()[i]);
            std::cout
                    << v_sim3->estimate().rotation().w() << " "
                    << v_sim3->estimate().rotation().x() << " "
                    << v_sim3->estimate().rotation().y() << " "
                    << v_sim3->estimate().rotation().z() << " "
                    << v_sim3->estimate().translation().transpose() << " "
                    << v_sim3->estimate().scale() << std::endl;
//        std::cout << v_sim3->estimate().inverse().scale() << std::endl;
        }

    }

    std::cout << "-----------------\n";
//    g2o::Sim3 e_value(g2o::Quaternion(0.89894, 0.25292, 0.25292, 0.25292), g2o::Vector3(1, 2, 3), 2);
    g2o::Vector7 a;
    a << 0.001,0,0,0,0,0,0;
    g2o::Sim3 e_value(a);
    g2o::Sim3 test(Eigen::Quaterniond(0,1,0,0), Eigen::Vector3d(0,0,0),1);
//    std::cout << e_value.scale() * e_value.rotation().matrix() << std::endl;
    std::cout << test.log() << std::endl; //r,t,s

    return 0;
}