//
// Created by mpl on 22-9-22.
//

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <ceres/autodiff_cost_function.h>

#include <vector>
#include <fstream>

typedef Eigen::Matrix<double, 7, 1> Vector7d;
typedef Eigen::Matrix<double, 8, 1> Vector8d;


template<typename T>
Eigen::Matrix<T, 3, 3> skewSymmetric(Eigen::Matrix<T, 3, 1> v)
{
    Eigen::Matrix<T, 3, 3> m;
    m << T(0), -v(2), v(1),
         v(2), T(0), -v(0),
         -v(1), v(0), T(0);
    return m;
}

// out: [qw, qx, qy, qz, x, y, z, s]
template <typename T>
Eigen::Matrix<T, 8, 1> Sim3Exp(const Eigen::Matrix<T, 7, 1> sim3)
{
    Eigen::Matrix<T, 3, 1> omega = sim3.segment(0, 3);
    Eigen::Matrix<T, 3, 1> upsilon = sim3.segment(3, 3);
    T sigma = sim3(6);

    // derive rotation (it's easy to derive so3 using quaternion)
    Eigen::Matrix<T, 4, 1> r;
    ceres::AngleAxisToQuaternion(omega.data(), r.data());
    T theta = omega.norm();

    // derive s
    T s = ceres::exp(sigma);

    // derive t
    Eigen::Matrix<T, 3, 3> omega0 = Eigen::Matrix<T, 3, 3>::Identity();
    Eigen::Matrix<T, 3, 3> omega1 = skewSymmetric(omega);
    Eigen::Matrix<T, 3, 3> omega2 = omega1 * omega1;
    T eps = T(0.00001);
    T block0 = T(0), block1 = T(0), block2 = T(0);
    if (ceres::abs(sigma) < eps)
    {
        block0 = T(1);
        if (ceres::abs(theta) < eps)
        {
            block1 = T(1.0 / 2.0);
            block2 = T(1.0 / 6.0);
        }
        else
        {
            block1 = (T(1) - ceres::cos(theta)) / (theta * theta);
            block2 = (theta - ceres::sin(theta)) / (theta * theta * theta);
        }
    }
    else
    {
        block0 = (s - T(1)) / sigma;
        if (ceres::abs(theta) < eps)
        {
            block1 = ((sigma - T(1)) * s + T(1)) / (sigma * sigma);
            block2 = (s * (sigma * sigma / T(2) - sigma + T(1)) - T(1)) / (sigma * sigma * sigma);
        }
        else
        {
            T A = s * ceres::sin(theta);
            T B = s * ceres::cos(theta);
            block1 = (A * sigma + (T(1) - B) * theta) / (theta*sigma*sigma + theta*theta*theta);
            block2 = ((s - T(1)) / sigma - ((B - T(1))*sigma + A*theta) / (sigma*sigma + theta*theta)) / (theta * theta);
        }
    }
    Eigen::Matrix<T, 3, 3> W = block0 * omega0 + block1 * omega1 + block2 * omega2;
    Eigen::Matrix<T, 3, 1> t = W * upsilon;

    Eigen::Matrix<T, 8, 1> SIM3 = Eigen::Matrix<T, 8, 1>::Zero();
    SIM3.segment(0, 4) = r;
    SIM3.segment(4, 3) = t;
    SIM3(7) = s;
    return SIM3;
}

// out: [omega, upsilon, sigma]
template <typename T>
Eigen::Matrix<T, 7, 1> Sim3Log(const Eigen::Matrix<T, 8, 1> SIM3)
{
    Eigen::Matrix<T, 4, 1> q = SIM3.segment(0, 4);
    Eigen::Matrix<T, 3, 1> t = SIM3.segment(4, 3);
    T s = SIM3(7);

    //derive omega
    Eigen::Matrix<T, 3, 1> omega;
    ceres::QuaternionToAngleAxis(q.data(), omega.data());


    // derive sigma
    T sigma = ceres::log(s);

    // derive upsilon
    Eigen::Matrix<T, 3, 3> omega0 = Eigen::Matrix<T, 3, 3>::Identity();
    Eigen::Matrix<T, 3, 3> omega1 = skewSymmetric(omega);
    Eigen::Matrix<T, 3, 3> omega2 = omega1 * omega1;
    T theta = omega.norm();
    T eps = T(0.00001);
    T block0 = T(0), block1 = T(0), block2 = T(0);
    if (ceres::abs(sigma) < eps)
    {
        block0 = T(1);
        if (ceres::abs(theta) < eps) // not sure about this comparison when things become template
        {
            block1 = T(1.0 / 2.0);
            block2 = T(1.0 / 6.0);
        }
        else
        {
            block1 = (T(1) - ceres::cos(theta)) / (theta * theta);
            block2 = (theta - ceres::sin(theta)) / (theta * theta * theta);
        }
    }
    else
    {
        block0 = (s - T(1)) / sigma;
        if (ceres::abs(theta) < eps)
        {
            block1 = ((sigma - T(1)) * s + T(1)) / (sigma * sigma);
            block2 = (s * (sigma * sigma / T(2) - sigma + T(1)) - T(1)) / (sigma * sigma * sigma);
        }
        else
        {
            T A = s * ceres::sin(theta);
            T B = s * ceres::cos(theta);
            block1 = (A * sigma + (T(1) - B) * theta) / (theta*sigma*sigma + theta*theta*theta);
            block2 = ((s - T(1)) / sigma - ((B - T(1))*sigma + A*theta) / (sigma*sigma + theta*theta)) / (theta * theta);
        }
    }
    Eigen::Matrix<T, 3, 3> W = block0 * omega0 + block1 * omega1 + block2 * omega2;
    Eigen::Matrix<T, 3, 1> upsilon = W.lu().solve(t);

    Eigen::Matrix<T, 7, 1> sim3 = Eigen::Matrix<T, 7, 1>::Zero();
    sim3.segment(0, 3) = omega;
    sim3.segment(3, 3) = upsilon;
    sim3(6) = sigma;
    return sim3;
}

template <typename T>
Eigen::Matrix<T, 8, 1> Sim3Inv(const Eigen::Matrix<T, 8, 1> SIM3)
{
    Eigen::Quaternion<T> q(SIM3(0), SIM3(1), SIM3(2), SIM3(3));
    Eigen::Matrix<T, 3, 1> t = SIM3.segment(4, 3);
    T s = SIM3(7);

    Eigen::Quaternion<T> q_inv = q.conjugate();
    T s_inv = T(1) / s;
    Eigen::Matrix<T, 3, 1> t_inv = -s_inv * q_inv.toRotationMatrix() * t;

    Eigen::Matrix<T, 8, 1> SIM3_inv;
    SIM3_inv << q_inv.w() , q_inv.x(), q_inv.y(), q_inv.z(), t_inv.x(), t_inv.y(), t_inv.z(), s_inv;

    return SIM3_inv;
}

template <typename T>
Eigen::Matrix<T, 8, 1> Tdot(const Eigen::Matrix<T, 8, 1> T0, const Eigen::Matrix<T, 8, 1> T1)
{
    Eigen::Quaternion<T> q0(T0(0), T0(1), T0(2), T0(3));
    Eigen::Matrix<T, 3, 1> t0 = T0.segment(4, 3);
    T s0 = T0(7);

    Eigen::Quaternion<T> q1(T1(0), T1(1), T1(2), T1(3));
    Eigen::Matrix<T, 3, 1> t1 = T1.segment(4, 3);
    T s1 = T1(7);

    Eigen::Quaternion<T> q_result = q0 * q1;
    Eigen::Matrix<T, 3, 1> t_result = s0 * q0.toRotationMatrix() * t1 + t0;
    T s_result = s0 * s1;

    Eigen::Matrix<T, 8, 1> T_result;
    T_result << q_result.w() , q_result.x(), q_result.y(), q_result.z(),
                t_result.x(), t_result.y(), t_result.z(), s_result;

    return T_result;
}

struct Node
{
    uint id;
    bool fixed;

    Vector8d SIM3_Tcw; // qw, qx, qy, qz, x, y, z, s
    Vector7d sim3_Tcw; // omega, upsilon, sigma (r, t, s)
};


struct Edge
{
    uint id0;
    uint id1;
    Vector8d SIM3_Tc1c0;
    bool unknown =  false;
};

struct Se3Functor
{
    Se3Functor(Edge edge): m_edge(edge){};

    template<class T>
    bool operator() (const T* const node0, const T* const node1, T* residual) const
    {
        Eigen::Matrix<T, 7, 1> se3_Tc0w(node0);
        Eigen::Matrix<T, 7, 1> se3_Tc1w(node1);
        se3_Tc0w(6) = T(0);
        se3_Tc1w(6) = T(0);

        Eigen::Matrix<T, 8, 1> SE3_Tc1c0;
        SE3_Tc1c0 << T(m_edge.SIM3_Tc1c0(0)), T(m_edge.SIM3_Tc1c0(1)), T(m_edge.SIM3_Tc1c0(2)),
                T(m_edge.SIM3_Tc1c0(3)), T(m_edge.SIM3_Tc1c0(4)), T(m_edge.SIM3_Tc1c0(5)),
                T(m_edge.SIM3_Tc1c0(6)), T(1);


        Eigen::Matrix<T, 8, 1> SE3_Tc0w = Sim3Exp(se3_Tc0w);
        Eigen::Matrix<T, 8, 1> SE3_Tc1w = Sim3Exp(se3_Tc0w);

        Eigen::Matrix<T, 8, 1> SE3_Tc0c1 = Tdot(SE3_Tc0w, Sim3Inv(SE3_Tc1w));

        Eigen::Matrix<T, 8, 1> SE3_error = Tdot(SE3_Tc0c1, SE3_Tc1c0);
//        Eigen::Matrix<T, 8, 1> SIM3_error = Tdot(SIM3_Tc1c0, SIM3_Tc0c1); // ? inverse ?
        Eigen::Matrix<T, 7, 1> se3_error = Sim3Log(SE3_error);
//        sim3_error(6) = T(0);
        for (int i = 0; i < 6; i++)
            residual[i] = se3_error(i);

        std::cout << "test:" << se3_error(6) << std::endl;
        return true;
    }

    Edge m_edge;

};

struct Sim3MagicFunctor
{
    Sim3MagicFunctor(Edge edge):m_edge(edge){};

    template<class T>
    bool operator() (const T* const node0, const T* const node1, T* residual) const
    {
        Eigen::Matrix<T, 7, 1> sim3_Tc0w(node0);
        Eigen::Matrix<T, 7, 1> sim3_Tc1w(node1);

        Eigen::Matrix<T, 8, 1> SIM3_Tc1c0;
        SIM3_Tc1c0 << T(m_edge.SIM3_Tc1c0(0)), T(m_edge.SIM3_Tc1c0(1)), T(m_edge.SIM3_Tc1c0(2)),
                T(m_edge.SIM3_Tc1c0(3)), T(m_edge.SIM3_Tc1c0(4)), T(m_edge.SIM3_Tc1c0(5)),
                T(m_edge.SIM3_Tc1c0(6)), T(m_edge.SIM3_Tc1c0(7));


        Eigen::Matrix<T, 8, 1> SIM3_Tc0w = Sim3Exp(sim3_Tc0w);
        Eigen::Matrix<T, 8, 1> SIM3_Tc1w = Sim3Exp(sim3_Tc1w);

        Eigen::Matrix<T, 8, 1> SIM3_Tc0c1_1 = Tdot(SIM3_Tc0w, Sim3Inv(SIM3_Tc1w));
        Eigen::Matrix<T, 8, 1> SIM3_error_1 = Tdot(SIM3_Tc0c1_1, SIM3_Tc1c0);
        Eigen::Matrix<T, 7, 1> sim3_error_1 = Sim3Log(SIM3_error_1);

//        Eigen::Matrix<T, 8, 1> SIM3_Tc1c0_2 = Tdot(SIM3_Tc1w, Sim3Inv(SIM3_Tc0w));
//        Eigen::Matrix<T, 8, 1> SIM3_error_2 = Tdot(SIM3_Tc1c0_2, Sim3Inv(SIM3_Tc1c0));


//        Eigen::Matrix<T, 8, 1> SIM3_error = Tdot(SIM3_Tc1c0, SIM3_Tc0c1); // ? inverse ?
//        Eigen::Matrix<T, 7, 1> sim3_error_2 = Sim3Log(SIM3_error_2);

//        sim3_error(6) = T(0);
//        std::cout << sim3_error_1 << std::endl;

        for (int i = 0; i < 6; i++)
            residual[i] = sim3_error_1(i);

        return true;
    }

    Edge m_edge;
};

struct Sim3Functor
{
    Sim3Functor(Edge edge):m_edge(edge){};

    template<class T>
    bool operator() (const T* const node0, const T* const node1, T* residual) const
    {
        Eigen::Matrix<T, 7, 1> sim3_Tc0w(node0);
        Eigen::Matrix<T, 7, 1> sim3_Tc1w(node1);

        Eigen::Matrix<T, 8, 1> SIM3_Tc1c0;
        SIM3_Tc1c0 << T(m_edge.SIM3_Tc1c0(0)), T(m_edge.SIM3_Tc1c0(1)), T(m_edge.SIM3_Tc1c0(2)),
                    T(m_edge.SIM3_Tc1c0(3)), T(m_edge.SIM3_Tc1c0(4)), T(m_edge.SIM3_Tc1c0(5)),
                    T(m_edge.SIM3_Tc1c0(6)), T(m_edge.SIM3_Tc1c0(7));


        Eigen::Matrix<T, 8, 1> SIM3_Tc0w = Sim3Exp(sim3_Tc0w);
        Eigen::Matrix<T, 8, 1> SIM3_Tc1w = Sim3Exp(sim3_Tc1w);

        Eigen::Matrix<T, 8, 1> SIM3_Tc0c1 = Tdot(SIM3_Tc0w, Sim3Inv(SIM3_Tc1w));

        Eigen::Matrix<T, 8, 1> SIM3_error = Tdot(SIM3_Tc0c1, SIM3_Tc1c0);
//        Eigen::Matrix<T, 8, 1> SIM3_error = Tdot(SIM3_Tc1c0, SIM3_Tc0c1); // ? inverse ?
        Eigen::Matrix<T, 7, 1> sim3_error = Sim3Log(SIM3_error);

        for (int i = 0; i < 7; i++)
            residual[i] = sim3_error(i);

        return true;
    }

    Edge m_edge;
};

struct Sim3FunctorFixNode0
{
    Sim3FunctorFixNode0(Node fixedNode, Edge edge):m_fixedNode(fixedNode), m_edge(edge){};

    template<class T>
    bool operator() (const T* const node1, T* residual) const
    {
        Eigen::Matrix<T, 7, 1> sim3_Tc1w(node1);

        Eigen::Matrix<T, 7, 1> sim3_Tc0w;
        sim3_Tc0w << T(m_fixedNode.sim3_Tcw(0)), T(m_fixedNode.sim3_Tcw(1)), T(m_fixedNode.sim3_Tcw(2)),
                T(m_fixedNode.sim3_Tcw(3)), T(m_fixedNode.sim3_Tcw(4)), T(m_fixedNode.sim3_Tcw(5)),
                T(m_fixedNode.sim3_Tcw(6));

        Eigen::Matrix<T, 8, 1> SIM3_Tc1c0;
        SIM3_Tc1c0 << T(m_edge.SIM3_Tc1c0(0)), T(m_edge.SIM3_Tc1c0(1)), T(m_edge.SIM3_Tc1c0(2)),
                T(m_edge.SIM3_Tc1c0(3)), T(m_edge.SIM3_Tc1c0(4)), T(m_edge.SIM3_Tc1c0(5)),
                T(m_edge.SIM3_Tc1c0(6)), T(m_edge.SIM3_Tc1c0(7));


        Eigen::Matrix<T, 8, 1> SIM3_Tc0w = Sim3Exp(sim3_Tc0w);
        Eigen::Matrix<T, 8, 1> SIM3_Tc1w = Sim3Exp(sim3_Tc1w);

        Eigen::Matrix<T, 8, 1> SIM3_Tc0c1 = Tdot(SIM3_Tc0w, Sim3Inv(SIM3_Tc1w));

        Eigen::Matrix<T, 8, 1> SIM3_error = Tdot(SIM3_Tc0c1, SIM3_Tc1c0);
//        Eigen::Matrix<T, 8, 1> SIM3_error = Tdot(SIM3_Tc1c0, SIM3_Tc0c1); // ? inverse ?
        Eigen::Matrix<T, 7, 1> sim3_error = Sim3Log(SIM3_error);

        for (int i = 0; i < 7; i++)
            residual[i] = sim3_error(i);

        return true;
    }

    Node m_fixedNode;
    Edge m_edge;
};

struct Sim3FunctorFixNode1
{
    Sim3FunctorFixNode1(Node fixedNode, Edge edge):m_fixedNode(fixedNode), m_edge(edge){};

    template<class T>
    bool operator() (const T* const node0, T* residual) const
    {
        Eigen::Matrix<T, 7, 1> sim3_Tc0w(node0);

        Eigen::Matrix<T, 7, 1> sim3_Tc1w;
        sim3_Tc1w << T(m_fixedNode.sim3_Tcw(0)), T(m_fixedNode.sim3_Tcw(1)), T(m_fixedNode.sim3_Tcw(2)),
                T(m_fixedNode.sim3_Tcw(3)), T(m_fixedNode.sim3_Tcw(4)), T(m_fixedNode.sim3_Tcw(5)),
                T(m_fixedNode.sim3_Tcw(6));

        Eigen::Matrix<T, 8, 1> SIM3_Tc1c0;
        SIM3_Tc1c0 << T(m_edge.SIM3_Tc1c0(0)), T(m_edge.SIM3_Tc1c0(1)), T(m_edge.SIM3_Tc1c0(2)),
                T(m_edge.SIM3_Tc1c0(3)), T(m_edge.SIM3_Tc1c0(4)), T(m_edge.SIM3_Tc1c0(5)),
                T(m_edge.SIM3_Tc1c0(6)), T(m_edge.SIM3_Tc1c0(7));


        Eigen::Matrix<T, 8, 1> SIM3_Tc0w = Sim3Exp(sim3_Tc0w);
        Eigen::Matrix<T, 8, 1> SIM3_Tc1w = Sim3Exp(sim3_Tc1w);

        Eigen::Matrix<T, 8, 1> SIM3_Tc0c1 = Tdot(SIM3_Tc0w, Sim3Inv(SIM3_Tc1w));

        Eigen::Matrix<T, 8, 1> SIM3_error = Tdot(SIM3_Tc0c1, SIM3_Tc1c0);
//        Eigen::Matrix<T, 8, 1> SIM3_error = Tdot(SIM3_Tc1c0, SIM3_Tc0c1); // ? inverse ?
        Eigen::Matrix<T, 7, 1> sim3_error = Sim3Log(SIM3_error);

        for (int i = 0; i < 7; i++)
            residual[i] = sim3_error(i);

        return true;
    }

    Node m_fixedNode;
    Edge m_edge;
};

int main(int argc, char** argv)
{
    std::map<uint, Node> nodes;
    std::vector<Edge> edges;

    std::string node_file = argv[1];
    std::string edge_file = argv[2];

    std::ifstream node_src, edge_src;
    node_src.open(node_file);
    edge_src.open(edge_file);


    int numNodes, numEdges;
    node_src >> numNodes;
    edge_src >> numEdges;

    int min_id = INT32_MAX, max_id = 0;
    // Read data
//    std::cout <<"Nodes...(qw, qx, qy, qz, x, y, z, s)\n";
    std::cout <<"Nodes...(id, x, y, z, qx, qy, qz, qw, s)\n";
    for (int i = 0; i < numNodes; i++)
    {
        double id, x, y, z, qx, qy, qz, qw, s;
        int f;
//        node_src >> id >> qw >> qx >> qy >> qz >> x >> y >> z >> s;
//        node_src >> id  >> qw >> qx >> qy >> qz >> x >> y >> z ; // test_new
        node_src >> id  >> x >> y >> z >> qx >> qy >> qz >> qw >> s; // test_new

        Node temp;
        temp.id = id;
//        temp.SIM3_Tcw << qw, qx, qy, qz, x, y, z, s;
        temp.SIM3_Tcw << qw, qx, qy, qz, x, y, z, 1; // test_new
        temp.sim3_Tcw = Sim3Log(temp.SIM3_Tcw);
        if (id == 0)
            temp.fixed = true;
        else
            temp.fixed = false;

//        if (id == 0)
//            temp.fixed = true;
//        else
//            temp.fixed = false;
        nodes[id] = temp;

        if (id < min_id)
            min_id = id;
        if (id > max_id)
            max_id = id;

        std::cout << id << " " << temp.SIM3_Tcw.transpose() << std::endl;
    }

    std::cout <<"Edges...(id0, id1, x, y, z, qx, qy, qz, qw, s)\n";
    for (int i = 0; i < numEdges; i++)
    {
        double id0, id1, s, x, y, z, qx, qy, qz, qw;
//        edge_src >> id0 >> id1  >> qw >> qx >> qy >> qz >> x >> y >> z >> s;
        edge_src >> id0 >> id1  >> x >> y >> z >> qx >> qy >> qz >> qw >> s;

        Edge temp;
        if (s == -1)
        {
            temp.id0 = id0;
            temp.id1 = id1;
            temp.SIM3_Tc1c0 << qw, qx, qy, qz, x, y, z, 1;
            temp.unknown = true;
        }else
        {
            temp.id0 = id0;
            temp.id1 = id1;
            temp.SIM3_Tc1c0 << qw, qx, qy, qz, x, y, z, s;
            temp.unknown = false;
        }


        // ORB
//        if (id0 == 115|| id0 == 116 || id0 == 114 || id0 == 113 || id0 == 117)
//        {
//            temp.unknown = true;
//        }
//        else
//            temp.unknown = false;
//        if (id1 == 115|| id1 == 116 || id1 == 114|| id1 == 113 || id1 == 117)
//        {
//            temp.unknown = true;
//        }
//        else
//            temp.unknown = false;

        // circle
//        if (id0 == 39 || id0 == 40 || id0 == 41 || id0 == 42 || id0 == 43|| id0 == 80 || id0 == 120 )
//        {
//            temp.unknown = true;
//        }
//        else
//            temp.unknown = false;

        // rectangular
//        if (id0 == 30 || id0 == 61 || id0 == 92 || id0 == 124)
//        {
//            temp.unknown = true;
//        }
//        else
//            temp.unknown = false;
        edges.emplace_back(temp);
        std::cout << id0 << " " << id1 << " " << temp.SIM3_Tc1c0.transpose() << std::endl;
    }

    node_src.close();
    edge_src.close();


    // Optimize
    google::InitGoogleLogging(argv[0]);
    ceres::Problem problem;
    ceres::Solver::Options options;

    for (Edge& edge: edges)
    {
        if (nodes[edge.id0].fixed)
        {
            ceres::CostFunction* costFunction =
                    new ceres::AutoDiffCostFunction<Sim3FunctorFixNode0, 7, 7>(
                            new Sim3FunctorFixNode0(nodes[edge.id0], edge));

            problem.AddResidualBlock(costFunction, nullptr,
                                     nodes[edge.id1].sim3_Tcw.data());
        }
        else if (nodes[edge.id1].fixed)
        {
            ceres::CostFunction* costFunction =
                    new ceres::AutoDiffCostFunction<Sim3FunctorFixNode1, 7, 7>(
                            new Sim3FunctorFixNode1(nodes[edge.id1], edge));

            problem.AddResidualBlock(costFunction, nullptr,
                                     nodes[edge.id0].sim3_Tcw.data());
        }
        else if(edge.unknown)
        {
//            std::cout << "hit\n";
            ceres::CostFunction* costFunction =
                    new ceres::AutoDiffCostFunction<Sim3MagicFunctor, 6, 7, 7>(new Sim3MagicFunctor(edge));

            // consider huber loss
            problem.AddResidualBlock(costFunction, nullptr,
                                     nodes[edge.id0].sim3_Tcw.data(),
                                     nodes[edge.id1].sim3_Tcw.data());
            std::cout << "hit" << std::endl;

            // --------------------------------
//            ceres::CostFunction* costFunction =
//                    new ceres::AutoDiffCostFunction<Se3Functor, 6, 7, 7>(new Se3Functor(edge));
//
//            // consider huber loss
//            problem.AddResidualBlock(costFunction, nullptr,
//                                     nodes[edge.id0].sim3_Tcw.data(),
//                                     nodes[edge.id1].sim3_Tcw.data());

        }
        else
        {
            ceres::CostFunction* costFunction =
                    new ceres::AutoDiffCostFunction<Sim3Functor, 7, 7, 7>(new Sim3Functor(edge));

            // consider huber loss
            problem.AddResidualBlock(costFunction, nullptr,
                                     nodes[edge.id0].sim3_Tcw.data(),
                                     nodes[edge.id1].sim3_Tcw.data());
        }
    }

    options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
    options.minimizer_progress_to_stdout = true;
    options.max_num_iterations = 200;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.FullReport() <<"\n";

    // Output
    std::ofstream dest;
    dest.open("result.txt");
    std::cout << "result............(qw, qx, qy, qz, x, y, z, s)\n";
    dest.precision(15);
    for (int i = min_id; i <= max_id; i++)
    {
        if (nodes.find(i) != nodes.end())
        {
            Vector8d Scw = Sim3Exp(nodes[i].sim3_Tcw);
            Eigen::Quaterniond Qcw(Scw[0], Scw[1], Scw[2], Scw[3]);
            Eigen::Vector3d tcw(Scw[4]/Scw[7], Scw[5]/Scw[7], Scw[6]/Scw[7]);
            Eigen::Quaterniond Qwc = Qcw.conjugate();
            Eigen::Vector3d twc = -Qwc.matrix() * tcw;

            std::cout << i << " " << Scw.transpose() << std::endl;
            if (Qwc.w() < 0)
                dest << i << " " << twc[0] << " " << twc[1] << " " << twc[2] << " "
                     << -Qwc.x() << " " << -Qwc.y() << " "
                     << -Qwc.z() << " " << -Qwc.w() << std::endl;
            else
                dest << i << " " << twc[0] << " " << twc[1] << " " << twc[2] << " "
                     << Qwc.x() << " " << Qwc.y() << " "
                     << Qwc.z() << " " << Qwc.w() << std::endl;

        }
    }
    dest.close();



}
