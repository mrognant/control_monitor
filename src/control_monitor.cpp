#include "control_monitor/control_monitor.hpp"
#include <iostream>
#include <sched.h>    // for scheduler choice (FIFO or RR for Real-Time Applications)

namespace CONTROL_MONITOR {
/*--------------------------------------------------------------------
/*--------------------------------------------------------------------
/* Class Cluster_state Definition
/*--------------------------------------------------------------------

/*--------------------------------------------------------------------
 * Cluster_state()
 * Constructor.
 *------------------------------------------------------------------*/
Cluster_state::Cluster_state():m_htoupie(4.5e-6*628),m_beta(65*M_PI/180)
{ m_alpha << 0, M_PI/3, 2*M_PI/3, M_PI, 4*M_PI/3, 5*M_PI/3;
	m_sigma.setZero();
	m_dsigma.setZero();
	m_moment.setZero();
	m_torque.setZero();
} // end Cluster_state()

/*--------------------------------------------------------------------
 * ~Cluster_state()
 * Destructor.
 *------------------------------------------------------------------*/
Cluster_state::~Cluster_state()
{
} // end ~Cluster_state()

/*--------------------------------------------------------------------
 * update_state()
 * Update the Cluster_state content.
 *------------------------------------------------------------------*/
void Cluster_state::update_state(const cmg_msgs::DynamixelStateListConstPtr &msg)
{
	for (int i=0; i<6; ++i)
	{
		m_sigma(i)=msg->states[i].present_position;
		m_dsigma(i)=msg->states[i].present_velocity;
	}
	m_compute_torque();
	m_compute_moment();
} // end update_state()

void Cluster_state::update_htoupie(const cmg_msgs::AGConfig::ConstPtr &msg)
{
		m_htoupie=msg->htoupie;
} // end update_htoupie()


/*--------------------------------------------------------------------
 * m_compute_torque()
 * Compute the CMG torque from the current CMG_state.
 *------------------------------------------------------------------*/
void Cluster_state::m_compute_torque()
{
	m_torque.setZero();
	for (int i=0; i<6; ++i)
	{
		m_torque(0)+= (sin(m_alpha(i))*sin(m_sigma(i))-cos(m_alpha(i))*cos(m_beta)*cos(m_sigma(i)))*m_dsigma(i);
    m_torque(1)+= (-cos(m_alpha(i))*sin(m_sigma(i))-sin(m_alpha(i))*cos(m_beta)*cos(m_sigma(i)))*m_dsigma(i);
    m_torque(2)+= (sin(m_beta)*cos(m_sigma(i)))*m_dsigma(i);
	}
	m_torque*=m_htoupie;
}// end m_compute_torque

/*--------------------------------------------------------------------
 * m_compute_moment()
 * Compute the CMG kinetic moment from the current CMG_state.
 *------------------------------------------------------------------*/
void Cluster_state::m_compute_moment()
{
	m_moment.setZero();
	for (int i=0; i<6; ++i)
	{
		m_moment(0)+= -sin(m_alpha(i))*cos(m_sigma(i))-cos(m_alpha(i))*cos(m_beta)*sin(m_sigma(i));
		m_moment(1)+= cos(m_alpha(i))*cos(m_sigma(i))-sin(m_alpha(i))*cos(m_beta)*sin(m_sigma(i));
		m_moment(2)+= sin(m_beta)*sin(m_sigma(i));
	}
	m_moment*=m_htoupie;

}// end m_compute_moment

/*--------------------------------------------------------------------
 * cc_torque(float*)
 * Copy the current torque value
 *------------------------------------------------------------------*/
void Cluster_state::cc_torque(Eigen::Vector3d &torque)
{for (int i=0; i<3; ++i) torque[i]=m_torque[i];}// end cc_torque

/*--------------------------------------------------------------------
 * cc_moment(float*)
 * Copy the current moment value
 *------------------------------------------------------------------*/
void Cluster_state::cc_moment(Eigen::Vector3d &moment)
{for (int i=0; i<3; ++i) moment(i)=m_moment(i);}// end cc_moment

/*--------------------------------------------------------------------
 * cc_dsigma(float*)
 * Copy the current dsigma value
 *------------------------------------------------------------------*/
void Cluster_state::cc_dsigma(Vector6d &dsigma)
{for (int i=0; i<6; ++i) dsigma(i)=m_dsigma(i);}// end cc_dsigma


/*--------------------------------------------------------------------
 * cc_sigma(float*)
 * Copy the current sigma value
 *------------------------------------------------------------------*/
void Cluster_state::cc_sigma(Vector6d &sigma)
{for (int i=0; i<6; ++i) sigma(i)=m_sigma(i);}// end cc_sigma

/*--------------------------------------------------------------------
/*--------------------------------------------------------------------
/* Class control_monitor
/*--------------------------------------------------------------------

/*--------------------------------------------------------------------
 * Control_monitor()
 * Constructor.
 *------------------------------------------------------------------*/
 Control_monitor::Control_monitor()
 {
	 m_desired_torque.setZero();
	 	 m_desired_omega.setZero();
	 m_cmg_mode=-1;
	 m_cons_motor.setZero();
	 m_CMG=Cluster_state();
	 m_omega_IMU.setZero();
	 pub_TorqueError = node.advertise<geometry_msgs::Vector3Stamped>("/Error/gamma", 1000);
	 pub_GimbalError = node.advertise<cmg_msgs::Commande_moteur>("/Error/motor", 1000);
	 pub_omegaError = node.advertise<geometry_msgs::Vector3Stamped>("/Error/omega", 1000);

 	 sub_CMGstate = node.subscribe("/gimbal/state", 10, &Control_monitor::update_CMG_state, this);
	 sub_config = node.subscribe("/parabola/agconfig", 10, &Control_monitor::update_htoupie, this);
	 sub_torque = node.subscribe("/boucle/commande",10, &Control_monitor::update_torque ,this);
	 sub_motor = node.subscribe("/gimbal/cmd",10, &Control_monitor::update_motor ,this);
	 sub_imu = node.subscribe("/imu/filtre", 10, &Control_monitor::update_IMU, this);
	 sub_wd = node.subscribe("/boucle/omega_d", 10, &Control_monitor::update_omega, this);

 }

 Control_monitor::~Control_monitor()
 {
 }


 void Control_monitor::update_CMG_state(const cmg_msgs::DynamixelStateListConstPtr &state)
 {
		geometry_msgs::Vector3Stamped gamma;
		cmg_msgs::Commande_moteur dCMG;
		Eigen::Vector3d torque_mes;
		Eigen::Vector3d torque_err;
		Vector6d motor_mes;
		Vector6d motor_err;

	  m_CMG.update_state(state);
		m_CMG.cc_torque(torque_mes);
		torque_err=m_desired_torque-torque_mes;
		gamma.header.stamp=state->header.stamp;
		//gamma.header.stamp=ros::Time::now();
		gamma.vector.x=torque_err(0) ;
		gamma.vector.y=torque_err(1) ;
		gamma.vector.z=torque_err(2) ;
		pub_TorqueError.publish(gamma);

		if ((m_cmg_mode==0)||(m_cmg_mode==1))
		{ if (m_cmg_mode==0) m_CMG.cc_sigma(motor_mes);
			if (m_cmg_mode==1) m_CMG.cc_dsigma(motor_mes);
			motor_err=m_cons_motor-motor_mes;
			dCMG.header=state->header;
			for (int i=0; i<6; i++) dCMG.com_moteur.push_back(motor_err(i));
			pub_GimbalError.publish(dCMG);
		}
	}

	void Control_monitor::update_htoupie(const cmg_msgs::AGConfig::ConstPtr &state)
	{
		m_CMG.update_htoupie(state);
	}

	void Control_monitor::update_torque(const cmg_msgs::Commande::ConstPtr &state)
	{
		for (int i=0; i<3; i++) m_desired_torque(i)=state->torque[i];
	}

	void Control_monitor::update_omega(const geometry_msgs::Vector3Stamped::ConstPtr &state)
	{
		m_desired_omega(0)=state->vector.x;
		m_desired_omega(1)=state->vector.y;
		m_desired_omega(2)=state->vector.z;
	}

	void Control_monitor::update_IMU(const sensor_msgs::Imu::ConstPtr &msg)
	{
		geometry_msgs::Vector3Stamped dw;

		dw.header.stamp=msg->header.stamp;
		dw.vector.x=m_desired_omega(0)-msg->angular_velocity.x;
		dw.vector.y=m_desired_omega(1)-msg->angular_velocity.y;
		dw.vector.z=m_desired_omega(2)-msg->angular_velocity.z;
		pub_omegaError.publish(dw);

	}

	void Control_monitor::update_motor(const cmg_msgs::GimbalTarget::ConstPtr &state)
	{
		m_cmg_mode=state->mode;
		for (int i=0; i<6; i++) m_cons_motor(i)=double(state->positions[i]);
	}
}

	int main(int argc, char **argv)
{

  ros::init(argc, argv, "control_monitor");
  CONTROL_MONITOR::Control_monitor control_monitor_node;


struct sched_param schedp;
  schedp.sched_priority = 99;
  sched_setscheduler(0, SCHED_FIFO, &schedp);

  while (ros::ok())
  {
    ros::spin();
  }

  return 0;
}
