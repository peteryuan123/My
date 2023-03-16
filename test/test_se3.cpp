//
// Created by mpl on 22-9-13.
//
//
// Created by mpl on 22-9-2.
//

#include <g2o/types/slam3d/vertex_se3.h>
#include <g2o/types/slam3d/edge_se3.h>

#include <g2o/types/slam3d/vertex_se3.h>
#include <g2o/types/slam3d/types_slam3d.h>
#include <g2o/types/sba/types_six_dof_expmap.h>

#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/core/solver.h>

#include <fstream>

typedef g2o::BlockSolver<g2o::BlockSolverTraits<6, 6>> BlockSolverType; //posedim, landmarkDim
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

    std::cout <<"Nodes...\n";
    for (int i = 0; i < numNodes; i++)
    {
        double id, x, y, z, qx, qy, qz, qw, s;
        node_src >> id >> qw >> qx >> qy >> qz >> x >> y >> z >> s;
        std::cout <<  id << " " << qw << " " << qx << " " << qy << " " << qz << " " << x << " " << y << " " << z <<std::endl;

        Eigen::Quaterniond q(qw, qx, qy, qz);
        Eigen::Vector3d t(x,y,z);
        g2o::SE3Quat se3(q, t);

        g2o::VertexSE3Expmap* v = new g2o::VertexSE3Expmap();

        v->setId(id);
        v->setMarginalized(false);
        v->setEstimate(se3);

        if (id == 0)
            v->setFixed(true);

        optimizer.addVertex(v);
    }

    std::cout <<"Edges...\n";

    for (int i = 0; i < numEdges; i++)
    {
        double id0, id1, s, x, y, z, qx, qy, qz, qw;
        edge_src >> id0 >> id1 >> qw >> qx >> qy >> qz >> x >> y >> z >> s;
        std::cout << id0 << " " << id1 << " " << s << " " << qw << " " << qx << " " << qy << " " << qz <<
                  " " << x << " " << y << " " << z << std::endl;

        g2o::EdgeSE3Expmap* e = new g2o::EdgeSE3Expmap();

        g2o::SE3Quat e_value(g2o::Quaternion(qw, qx, qy, qz), g2o::Vector3(x, y, z));

        e->setId(i);
        e->setVertex(0, optimizer.vertex(id0));
        e->setVertex(1, optimizer.vertex(id1));
        e->information() = Eigen::Matrix<double, 6, 6>::Identity();
        e->setMeasurement(e_value);

        optimizer.addEdge(e);
    }


    node_src.close();
    edge_src.close();

    optimizer.initializeOptimization();
    optimizer.optimize(30);

    for (int i = 0; i < optimizer.vertices().size(); i++)
    {
        g2o::VertexSE3Expmap* v_sim3 =
                static_cast<g2o::VertexSE3Expmap*>(optimizer.vertices()[i]);
        std::cout << v_sim3->estimate().rotation().w() << " "
                  << v_sim3->estimate().rotation().x() << " "
                  << v_sim3->estimate().rotation().y() << " "
                  << v_sim3->estimate().rotation().z() << " "
                  << v_sim3->estimate().translation().transpose()
                  << std::endl;
//        std::cout << v_sim3->estimate().inverse().scale() << std::endl;
    }


    return 0;
}