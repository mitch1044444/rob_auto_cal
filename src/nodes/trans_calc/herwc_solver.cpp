#include <ros/ros.h>

#include <iostream>
#include <stdexcept>

#include <Eigen/Dense>

#include "herwc_problem3.h"

#define SDPA_SOLVER

#include <trans_calc/gposolver_sdpa.h>
#include <rob_auto_cal/reqTransCalc.h>


#include "herwc_problem3.h"
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

using namespace std;
using namespace Eigen;
using namespace GpoSolver;

const char *status_str[] = {"SUCCESS_GLOPT", "SUCCESS", "UNBOUNDED_SOLUTION",
                            "INFEASIBLE_SDP", "INCOMPLETE_BASIS", "FAILURE"};

void q2rot(Vector4d &q, Matrix3d &R) {
R << q(0)*q(0) + q(1)*q(1) - q(2)*q(2) - q(3)*q(3),                     2*q(1)*q(2) - 2*q(0)*q(3),                     2*q(1)*q(3) + 2*q(0)*q(2),
                         2*q(1)*q(2) + 2*q(0)*q(3), q(0)*q(0) - q(1)*q(1) + q(2)*q(2) - q(3)*q(3),                     2*q(2)*q(3) - 2*q(0)*q(1), 
                         2*q(1)*q(3) - 2*q(0)*q(2),                     2*q(2)*q(3) + 2*q(0)*q(1), q(0)*q(0) - q(1)*q(1) - q(2)*q(2) + q(3)*q(3);
}

bool serviceCb(rob_auto_cal::reqTransCalc::Request &req, rob_auto_cal::reqTransCalc::Response &res)
{
  GpoSolverSdpa<HerwcProblem> gposolver;
    try
    {
      GpoSolverStatus status;
      GpoSolverSolutions sols;
      MatrixXd rvals;
      VectorXd pvals;



      // Load problem data using GpoSolver utility function 
      //gposolver.readProblemDataTxt(argv[1], rvals, pvals);

      pvals.resize(2);


      for (int i = 0; i < req.Avec.size(); ++i)
      {
        for (int i = 0; i < count; ++i)
        {
          /* code */
        }
      }



      // Scale translations
      double max_norm = 1;
      for (int i = 0; i < rvals.cols(); i++)
        {
          double ta_norm = rvals.block(9, i, 3, 1).norm();
          double tb_norm = rvals.block(21, i, 3, 1).norm();

          if (ta_norm > max_norm)
            max_norm = ta_norm;

          if (tb_norm > max_norm)
            max_norm = tb_norm;
        }

      for (int i = 0; i < rvals.cols(); i++)
        {
          rvals.block(9, i, 3, 1) /= max_norm;
          rvals.block(21, i, 3, 1) /= max_norm;
        }
        cout << "1" << endl;
      // Call GpoSolver
      gposolver.setParameter("verbose", 1);
      status = gposolver.solve(rvals.cols(), rvals.data(), pvals.data(), sols);
      cout << "2" << endl;
      // Print solution
      cout << endl << endl;
      cout << "GpoSolver status: " << status_str[status] << endl;
      cout << "GpoSolver: " << sols.size() << " solution(s) extracted" << endl;

      if (sols.size() == 0)
        return 0;

      if (status == SUCCESS_GLOPT)
        cout << "Global optimality certified numerically" << endl;
      else if (status == SUCCESS)
        cout << "Alas, global optimality could not be certified" << endl;

      if (sols.size() > 0)
        {
          int c = 0;
          for (list<vector<double> >::iterator it = sols.begin(); it != sols.end(); it++)
            {
              Matrix4d X, Z;
              Matrix3d Rx, Rz;
              Vector4d qx, qz;
              Vector3d tx, tz;

              for (int i = 0; i < 4; i++)
                qx(i) = (*it)[i];
              for (int i = 0; i < 3; i++)
                tx(i) = (*it)[i + 4];
              for (int i = 0; i < 4; i++)
                qz(i) = (*it)[i + 7];
              for (int i = 0; i < 3; i++)
                tz(i) = (*it)[i + 11];

              q2rot(qx, Rx);
              tx = tx * max_norm;
              X.setIdentity();
              X.block(0, 0, 3, 3) = Rx;
              X.block(0, 3, 3, 1) = tx;
    
              q2rot(qz, Rz);
              tz = tz * max_norm;
              Z.setIdentity();
              Z.block(0, 0, 3, 3) = Rz;
              Z.block(0, 3, 3, 1) = tz;

              cout << endl << endl;
              cout << "qx(" << ++c << ") = " << qx.transpose() << endl;
              cout << "tx(" <<   c << ") = " << tx.transpose() << endl;
              cout << "X("  <<   c << ") = " << endl << X << endl;
              cout << endl;
              cout << "qz(" <<   c << ") = " << qz.transpose() << endl;
              cout << "tz(" <<   c << ") = " << tz.transpose() << endl;
              cout << "Z("  <<   c << ") = " << endl << Z << endl;
            }
        }
    }
  catch (exception& e)
    {
      cerr << "Solver exception: " << e.what() << endl;
    }

}


int main(int argc, char** argv) {
cout << "Ready to do transformation calculation" << endl;

ros::init(argc, argv, "trans_calc");
ros::NodeHandle nh_;
ros::ServiceServer service_;
service_ = nh_.advertiseService("reqTransCalc", serviceCb);


ros::spin();

}
