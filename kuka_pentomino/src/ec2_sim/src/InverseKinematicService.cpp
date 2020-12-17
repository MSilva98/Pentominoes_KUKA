
/********************************************************************************
Copyright (c) 2015, TRACLabs, Inc.
All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
 are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice, 
       this list of conditions and the following disclaimer.

    2. Redistributions in binary form must reproduce the above copyright notice,
       this list of conditions and the following disclaimer in the documentation
       and/or other materials provided with the distribution.

    3. Neither the name of the copyright holder nor the names of its contributors
       may be used to endorse or promote products derived from this software 
       without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, 
BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, 
DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE 
OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
OF THE POSSIBILITY OF SUCH DAMAGE.
********************************************************************************/


#ifndef TRAC_IK_HPP
#define TRAC_IK_HPP

#include <trac_ik/trac_ik.hpp>
#include <trac_ik/nlopt_ik.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <boost/thread.hpp>
#include <boost/asio.hpp>
#include <boost/scoped_ptr.hpp>


namespace TRAC_IK {

  enum SolveType { Speed, Distance, Manip1, Manip2 };

  class TRAC_IK 
  {
  public:
    TRAC_IK(const KDL::Chain& _chain, const KDL::JntArray& _q_min, const KDL::JntArray& _q_max, double _maxtime=0.005, double _eps=1e-5, SolveType _type=Speed);

    TRAC_IK(const std::string& base_link, const std::string& tip_link, const std::string& URDF_param="/robot_description", double _maxtime=0.005, double _eps=1e-5, SolveType _type=Speed);

    ~TRAC_IK();

    bool getKDLChain(KDL::Chain& chain_) {
      chain_=chain;
      return initialized;
    }

    bool getKDLLimits(KDL::JntArray& lb_, KDL::JntArray& ub_) {
      lb_=lb;
      ub_=ub;
      return initialized;
    }


    static double JointErr(const KDL::JntArray& arr1, const KDL::JntArray& arr2) {
      double err = 0;
      for (uint i=0; i<arr1.data.size(); i++) {
        err += pow(arr1(i) - arr2(i),2);
      }
      
      return err;
    }

    int CartToJnt(const KDL::JntArray &q_init, const KDL::Frame &p_in, KDL::JntArray &q_out, const KDL::Twist& bounds=KDL::Twist::Zero());

    inline void SetSolveType(SolveType _type) {
      solvetype = _type;
    }

  private:
    bool initialized;
    KDL::Chain chain;
    KDL::JntArray lb, ub;
    boost::scoped_ptr<KDL::ChainJntToJacSolver> jacsolver;
    double eps;
    double maxtime;
    SolveType solvetype;

    boost::scoped_ptr<NLOPT_IK::NLOPT_IK> nl_solver;
    boost::scoped_ptr<KDL::ChainIkSolverPos_TL> iksolver;

    boost::posix_time::ptime start_time;

    bool runKDL(const KDL::JntArray &q_init, const KDL::Frame &p_in);


    bool runNLOPT(const KDL::JntArray &q_init, const KDL::Frame &p_in);

    void normalize_seed(const KDL::JntArray& seed, KDL::JntArray& solution);
    void normalize_limits(const KDL::JntArray& seed, KDL::JntArray& solution);

  
    std::vector<KDL::BasicJointType> types;

    boost::mutex mtx_;
    std::vector<KDL::JntArray> solutions;
    std::vector<std::pair<double,uint> >  errors; 


    boost::asio::io_service io_service;
    boost::thread_group threads;
    boost::asio::io_service::work work;
    KDL::Twist bounds;

    bool unique_solution(const KDL::JntArray& sol);

    inline static double fRand(double min, double max)
    {
      double f = (double)rand() / RAND_MAX;
      return min + f * (max - min);
    }

    /* @brief Manipulation metrics and penalties taken from "Workspace
    Geometric Characterization and Manipulability of Industrial Robots",
    Ming-June, Tsia, PhD Thesis, Ohio State University, 1986. 
    https://etd.ohiolink.edu/!etd.send_file?accession=osu1260297835
    */
    double manipPenalty(const KDL::JntArray&); 
    double ManipValue1(const KDL::JntArray&); 
    double ManipValue2(const KDL::JntArray&);

    inline bool myEqual(const KDL::JntArray& a, const KDL::JntArray& b) {
      return (a.data-b.data).isZero(1e-4);
    }

    void initialize();

  };

}

#endif
#include <ros/ros.h>

#include <euroc_ros_msgs/GetInverseKinematic.h>

TRAC_IK::TRAC_IK *tracik_solver;
int jointNr;

bool get_inverse_kinematic( euroc_ros_msgs::GetInverseKinematic::Request  &req,
         euroc_ros_msgs::GetInverseKinematic::Response &res)
{
  	KDL::JntArray result;
  	
  	int rc;

	//ROS_INFO_STREAM("*** Testing TRAC-IK ***");
  
    KDL::JntArray joint_position(jointNr);
    double frontPosition[] = {0.0,45.0,0.0,-90.0,0.0,45.0,-45.0};
	for (int i = 0; i < jointNr; i++) {
		joint_position(i) = req.corresponding_joint_position.values[i];
		//if( i==2 || i == 4)joint_position(i) = 0.0;
		//joint_position(i)=frontPosition[i];
	}

    KDL::Frame reqPose(KDL::Rotation().EulerZYX(req.tcp_pose.a,req.tcp_pose.b,req.tcp_pose.c),
    									  KDL::Vector(req.tcp_pose.x,req.tcp_pose.y,req.tcp_pose.z));
    
    rc=tracik_solver->CartToJnt(joint_position,reqPose,result);
    for(int i =0;i<jointNr;i++)
    	res.joint_position.values.push_back(result(i));

  	if (rc < 0)
  	{
  		ROS_ERROR("Impossible to calculate a IK solution for the requested pose!!\n x:%f y:%f z:%f a:%f b:%f c:%f",req.tcp_pose.x,req.tcp_pose.y,req.tcp_pose.z,req.tcp_pose.a,req.tcp_pose.b,req.tcp_pose.c);
  		return false;
  	}

  	return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "inverse_kinematic_service");
  ros::NodeHandle nh("~");

  std::string chain_start, chain_end, urdf_param, serviceName;
  double timeout;

  nh.param("chain_start", chain_start, std::string("iiwa_base"));
  nh.param("chain_end", chain_end, std::string("schunk_adapter_link"));
  nh.param("serviceName1", serviceName, std::string("/miiwa/get_inverse_kinematic"));

  if (chain_start=="" || chain_end=="") {
    ROS_FATAL("Missing chain info in launch file");
    exit (-1);
  }

  nh.param("timeout", timeout, 0.005);
  nh.param("urdf_param", urdf_param, std::string("/robot_description"));

  double eps = 1e-5;

  // This constructor parses the URDF loaded in rosparm urdf_param into the
  // needed KDL structures.
  TRAC_IK::TRAC_IK tracik_solverTmp(chain_start, chain_end, urdf_param, timeout, eps, TRAC_IK::Distance );
  tracik_solver = &tracik_solverTmp;

  KDL::Chain chain;
  KDL::JntArray ll, ul; //lower joint limits, upper joint limits

  bool valid = tracik_solver->getKDLChain(chain);
  
  if (!valid) {
    ROS_ERROR("There was no valid KDL chain found");
    return -1;
  }

  valid = tracik_solver->getKDLLimits(ll,ul);

  /* hack to guide the ik module */
//  ll.data[1] = -0.523599; // -30 graus
//  ll.data[2] = -0.174533; // -10 graus
//  ul.data[2] = 0.174533; // 10 graus
//  ll.data[4] = -0.174533; // -10 graus
//  ul.data[4] = 0.174533; // 10 graus
//  ul.data[6] = (175.0+45.0)*M_PI/180.0; // 175+45 graus para aceitar soluções nesta gama (tcp base is 45)

  if (!valid) {
    ROS_ERROR("There were no valid KDL joint limits found");
    return -1;
  }

  assert(chain.getNrOfJoints() == ll.data.size());
  assert(chain.getNrOfJoints() == ul.data.size());

  ROS_INFO ("Using %d joints",chain.getNrOfJoints());

  jointNr = chain.getNrOfJoints();

  ros::ServiceServer service = nh.advertiseService(serviceName, get_inverse_kinematic);
  ROS_INFO("Ready to calculate inverse kinematic.");
  ros::spin();

  return 0;
}
