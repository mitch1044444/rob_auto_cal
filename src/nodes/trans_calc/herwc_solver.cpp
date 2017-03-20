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
    ROS_INFO("Transformation calcualtion started, this may take several minuts.");
    ROS_INFO("When calcualtion has completed or failed a message will be printed.");
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
      pvals(0) = 2;
      pvals(1) = 100;

      cout << "There are " << req.Avec.size() << " A matrices and " << req.Bvec.size()  << " B matrices." << endl;
      rvals.resize(24, req.Avec.size());

      
      
      for (int i = 0; i < req.Avec.size(); ++i)
      {


        tf2::Matrix3x3 ArotMat = tf2::Matrix3x3(tf2::Quaternion(req.Avec.at(i).transform.rotation.x,
                                                                req.Avec.at(i).transform.rotation.y,
                                                                req.Avec.at(i).transform.rotation.z,
                                                                req.Avec.at(i).transform.rotation.w));

        tf2::Matrix3x3 BrotMat = tf2::Matrix3x3(tf2::Quaternion(req.Bvec.at(i).transform.rotation.x,
                                                                req.Bvec.at(i).transform.rotation.y,
                                                                req.Bvec.at(i).transform.rotation.z,
                                                                req.Bvec.at(i).transform.rotation.w));



        // Formatting requaried is:
        //A{i}(1,1), A{i}(2,1), A{i}(3,1), A{i}(1,2), A{i}(2,2), A{i}(3,2), A{i}(1,3), A{i}(2,3),...
        //A{i}(3,3), A{i}(1,4), A{i}(2,4), A{i}(3,4), B{i}(1,1), B{i}(2,1), B{i}(3,1), B{i}(1,2),...
        //B{i}(2,2), B{i}(3,2), B{i}(1,3), B{i}(2,3), B{i}(3,3), B{i}(1,4), B{i}(2,4), B{i}(3,4))
        
        for (int k = 0; k < 3; ++k)
        {   
            


            rvals(k+0, i) = ArotMat[k].getX();
            rvals(k+3, i) = ArotMat[k].getY();
            rvals(k+6, i) = ArotMat[k].getZ();
            rvals(k+0+12, i) = BrotMat[k].getX();
            rvals(k+3+12, i) = BrotMat[k].getY();
            rvals(k+6+12, i) = BrotMat[k].getZ();
        }
        

        rvals(9, i) = req.Avec.at(i).transform.translation.x;
        rvals(10, i) = req.Avec.at(i).transform.translation.y;
        rvals(11, i) = req.Avec.at(i).transform.translation.z;
        rvals(9+12, i) = req.Bvec.at(i).transform.translation.x;
        rvals(10+12, i) = req.Bvec.at(i).transform.translation.y;
        rvals(11+12, i) = req.Bvec.at(i).transform.translation.z;
        


      }

         //cout << "Rvals: " << endl << rvals << endl;
      
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

      // Call GpoSolver
      gposolver.setParameter("verbose", 1);
      status = gposolver.solve(rvals.cols(), rvals.data(), pvals.data(), sols);

      // Print solution
      // cout << endl << endl;
      // cout << "GpoSolver status: " << status_str[status] << endl;
      // cout << "GpoSolver: " << sols.size() << " solution(s) extracted" << endl;

      if (sols.size() == 0)
        return 0;

      if (status == SUCCESS_GLOPT)
        cout << "Global optimality certified numerically" << endl;
      else if (status == SUCCESS)
        cout << "Alas, global optimality could not be certified" << endl;
      cout << "Number of solutions: " << sols.size() << endl;
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
              res.X.transform.rotation.x = qx(0);
              res.X.transform.rotation.y = qx(1);
              res.X.transform.rotation.z = qx(2);
              res.X.transform.rotation.w = qx(3);
              res.X.transform.translation.x = tx(0);
              res.X.transform.translation.y = tx(1);
              res.X.transform.translation.z = tx(2);

              res.Z.transform.rotation.x = qz(0);
              res.Z.transform.rotation.y = qz(1);
              res.Z.transform.rotation.z = qz(2);
              res.Z.transform.rotation.w = qz(3);
              res.Z.transform.translation.x = tz(0);
              res.Z.transform.translation.y = tz(1);
              res.Z.transform.translation.z = tz(2);
            }
        }
    }
  catch (exception& e)
    {
      cerr << "Solver exception: " << e.what() << endl;
    }
  return true;
}


int main(int argc, char** argv) {
cout << "Ready to do transformation calculation" << endl;

ros::init(argc, argv, "trans_calc");
ros::NodeHandle nh_;
ros::ServiceServer service_;
service_ = nh_.advertiseService("reqTransCalc", serviceCb);


ros::spin();

}
