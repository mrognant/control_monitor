#ifndef CONTROL_MONITOR_H
#define CONTROL_MONITOR_H
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmg_msgs/Commande.h>
#include <cmg_msgs/Guidage.h>
#include <cmg_msgs/GimbalTarget.h>
#include <cmg_msgs/Commande_moteur.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <cmg_msgs/DynamixelStateList.h>
#include <sensor_msgs/Imu.h>
#include <cmg_msgs/AGConfig.h>

namespace CONTROL_MONITOR {

	typedef Eigen::Matrix<double, 6, 1> Vector6d;

	class Cluster_state
	{
		private:
			ros::Time time_stamp;
			double	m_htoupie;
			double 	m_beta;
			Vector6d m_alpha;
			Vector6d m_sigma;
			Vector6d m_dsigma;
			Eigen::Vector3d m_moment;
			Eigen::Vector3d m_torque;
			void m_compute_torque();
			void m_compute_moment();
		public:
			//! Constructor.
			Cluster_state();
			//! Destructor.
			~Cluster_state();

			//! Update state.
			void update_state(const cmg_msgs::DynamixelStateListConstPtr &state);
			void update_htoupie(const cmg_msgs::AGConfig::ConstPtr &state);
			//! Accessors
			void cc_torque(Eigen::Vector3d &torque);
			void cc_moment(Eigen::Vector3d &moment);
			void cc_dsigma(Vector6d &dsigma);
			void cc_sigma(Vector6d &sigma);
	};

	class Control_monitor {
		private:
			Eigen::Vector3d  m_desired_torque;
			Eigen::Vector3d  m_desired_omega;
			int	m_cmg_mode;
			Vector6d m_cons_motor;
			Cluster_state m_CMG;
			Eigen::Vector3d  m_omega_IMU;
		public:
			//! Constructor.
			Control_monitor();
			//! Destructor.
			~Control_monitor();

			ros::NodeHandle node;

			//! subscriber.
			ros::Subscriber sub_CMGstate, sub_torque, sub_config, sub_motor, sub_wd, sub_imu;

			//! Publisher.
			ros::Publisher pub_TorqueError, pub_GimbalError, pub_omegaError;

			void update_CMG_state(const cmg_msgs::DynamixelStateListConstPtr &state);
			void update_htoupie(const cmg_msgs::AGConfig::ConstPtr &state);
			void update_torque(const cmg_msgs::Commande::ConstPtr &state);
			void update_motor(const cmg_msgs::GimbalTarget::ConstPtr &state);
			void update_omega(const geometry_msgs::Vector3Stamped::ConstPtr &msg);
			void update_IMU(const sensor_msgs::Imu::ConstPtr &msg);
	};
}

#endif
