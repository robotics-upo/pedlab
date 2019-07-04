#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <string>
#include <lightsfm/sfm.hpp>
#include <lightsfm/rosmap.hpp>
#include <lightsfm/astar.hpp>
#include <nav_msgs/Odometry.h>
#include <random>
#include <chrono>
#include <animated_marker_msgs/AnimatedMarkerArray.h>	
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/MarkerArray.h>

//#include <spencer_tracking_msgs/DetectedPersons.h>

#include <people_msgs/People.h>
#include <pedlab/PLabInfo.h>

namespace plab
{

const double PERSON_MESH_SCALE = (2.0 / 8.5 * 1.8)*0.9;
const int TYPE_ROBOT      = 0;
const int TYPE_PEDESTRIAN = 1;
const int TYPE_TARGET     = 2;

class Node
{
public:
	Node(ros::NodeHandle& n, ros::NodeHandle& pn);
	~Node() {}


private:

	struct AgentCfg 
	{
		utils::Vector2d position;
		double dx;
		double dy;
		
		int n;
		int type;
		std::vector<std::string> goals; 
	};
	
	void publishScan360(const ros::Time& current_time);
	bool publishOdom(const ros::Time& current_time);
	bool publishPeople(const ros::Time& current_time);
	bool publishDetections(const ros::Time& current_time);
	void robotCmdVelReceived(const geometry_msgs::Twist::ConstPtr& cmd_vel); 
	void targetCmdVelReceived(const geometry_msgs::Twist::ConstPtr& cmd_vel); 
	int getPersonCollisionIndex(const utils::Vector2d& x, double collisionThreshold) const;
	bool transformPose(double& x, double& y, double& theta, 
				const std::string& sourceFrameId, const std::string& targetFrameId) const;	
	bool transformPoint(double& x, double& y, const std::string& sourceFrameId, const std::string& targetFrameId) const;
        bool transformVelocity(double& vx, double& vy, const std::string& sourceFrameId, const std::string& targetFrameId) const;

	void init(TiXmlNode *pParent, std::vector<AgentCfg>& agents, int& peopleCount);
	void initAgents();	

	tf::TransformBroadcaster tf_broadcaster;
	std::vector<sfm::Agent> agents;
	
	double pose_initial_x,pose_initial_y,pose_initial_yaw,robot_radius,person_radius,
		robot_max_velocity,people_average_vel,people_sd_vel;
	bool teleoperated_target;
	std::string config_file;
	std::mt19937 gen;
	utils::AStar a_star;
	bool target_cyclic_goals;
	int target_index;
	tf::TransformListener tf_listener;
	ros::Publisher odom_pub;
	ros::Publisher people_markers_pub;
	std::vector<int> peopleDetected;
	sensor_msgs::LaserScan scan360;	
	double people_detection_range;
	ros::Publisher scan360_pub;
	bool perfect_tracking;
	double scan_range_max;
	ros::Publisher people_detected_markers_pub;
	ros::Publisher people_detected_pub;
	double sd_noise,sd_noise_theta,false_negative_prob;
	bool noisy_detections;
	ros::Publisher spencer_pub;

};



inline
void Node::init(TiXmlNode *pParent, std::vector<AgentCfg>& agents, int& peopleCount)
{
	
	if ( !pParent ) return;

	if ( pParent->Type() == TiXmlNode::TINYXML_ELEMENT) {
		TiXmlElement* pElement = pParent->ToElement();
		TiXmlAttribute* pAttrib=pElement->FirstAttribute();
		if (strcmp(pParent->Value(),"agent")==0) {
			AgentCfg agent;
			double x,y;
			pAttrib->QueryDoubleValue(&x);
			pAttrib=pAttrib->Next();
			pAttrib->QueryDoubleValue(&y);
			pAttrib=pAttrib->Next();
			pAttrib->QueryIntValue(&agent.n);
			peopleCount+=agent.n;
			pAttrib=pAttrib->Next();
			pAttrib->QueryDoubleValue(&agent.dx);
			pAttrib=pAttrib->Next();
			pAttrib->QueryDoubleValue(&agent.dy);
			pAttrib=pAttrib->Next();
			pAttrib->QueryIntValue(&agent.type);
			agent.position.set(x,y);
			agents.push_back(agent);
		} else if (strcmp(pParent->Value(),"addwaypoint")==0) { 
			std::string id;
			id.assign(pAttrib->Value());
			agents.back().goals.push_back(id);
		}
	}

	for (TiXmlNode* pChild = pParent->FirstChild(); pChild != 0; pChild = pChild->NextSibling()) {
		init( pChild, agents, peopleCount );
	}
}




inline
void Node::initAgents()
{
	target_index=-1;
	TiXmlDocument xml_doc(config_file);
	if (xml_doc.LoadFile()) {

		int peopleCount=0;
		std::vector<AgentCfg> agents_cfg;
		std::normal_distribution<double> randomVel(people_average_vel,people_sd_vel);	
		
		init(&xml_doc,agents_cfg,peopleCount);
		agents.resize(peopleCount+1);
		agents[0].position.set(pose_initial_x,pose_initial_y);
		agents[0].yaw = utils::Angle::fromRadian(pose_initial_yaw);
		agents[0].radius = robot_radius;
		agents[0].desiredVelocity = robot_max_velocity;
		agents[0].teleoperated = true;
		
		int counter=1;
		int groupCounter=0;
		for (unsigned i=0; i< agents_cfg.size(); i++) {
			std::uniform_real_distribution<double> randomX(-agents_cfg[i].dx / 2, agents_cfg[i].dx / 2);
    			std::uniform_real_distribution<double> randomY(-agents_cfg[i].dy / 2, agents_cfg[i].dy / 2);
			for (int j=0;j<agents_cfg[i].n;j++) {
				agents[counter].position = agents_cfg[i].position;
				agents[counter].desiredVelocity = randomVel(gen);
				agents[counter].radius = person_radius;
				if (agents_cfg[i].type == TYPE_TARGET) {
					target_index=counter;
					agents[counter].groupId = groupCounter;
					agents[0].groupId = groupCounter;
					agents[counter].teleoperated = teleoperated_target;
				} else if (agents_cfg[i].n>1) {
					if (agents_cfg[i].dx != 0) {
            					agents[counter].position.incX(randomX(gen));
					}
        				if (agents_cfg[i].dy != 0) {
            					agents[counter].position.incY(randomY(gen));
					}
					agents[counter].groupId = groupCounter;
				}
				if (!agents_cfg[i].goals.empty()) {
					if (agents_cfg[i].type == TYPE_PEDESTRIAN) {
						agents[counter].cyclicGoals=true;
					} else if (agents_cfg[i].type == TYPE_TARGET) {
						agents[counter].cyclicGoals=target_cyclic_goals;
					}
					for (unsigned k = 0; k< agents_cfg[i].goals.size(); k++) {
						sfm::Goal goal;
						double x,y,r;
						a_star.getPos(agents_cfg[i].goals[k],x,y,r);
						goal.center.set(x,y);
						goal.radius = r;
						agents[counter].goals.push_back(goal);
					}
					utils::Vector2d u = agents[counter].goals.front().center - agents[counter].position;
					agents[counter].yaw = u.angle();
				}
				counter++;
			}
			if (agents_cfg[i].n>1 || agents_cfg[i].type == TYPE_TARGET) {
				groupCounter++;
			}

		}
		if (target_index==-1) {
			teleoperated_target=false;
		} else if (target_index!=1) {
			std::swap(agents[target_index],agents[1]);
			target_index=1;
		}
	}
}



inline
bool Node::transformPose(double& x, double& y, double& theta, const std::string& sourceFrameId, const std::string& targetFrameId) const
{
	tf::Stamped<tf::Pose> pose,tfPose;
	pose.setData(tf::Pose(tf::createQuaternionFromRPY(0,0,theta), tf::Vector3(x,y,0)));
	pose.frame_id_ = sourceFrameId;
	pose.stamp_ = ros::Time(0);
	try
	{
		tf_listener.transformPose(targetFrameId, pose, tfPose);
	} catch(std::exception &e) {
		ROS_ERROR("%s",e.what());
		return false;
	}
	x = tfPose.getOrigin().getX();
	y = tfPose.getOrigin().getY();
	tf::Matrix3x3 m(tfPose.getRotation());
	double roll,pitch;
	m.getRPY(roll, pitch, theta);
	return true;
}


inline
bool Node::transformPoint(double& x, double& y, const std::string& sourceFrameId, const std::string& targetFrameId) const
{
	tf::Stamped<tf::Pose> pose,tfPose;
	pose.setData(tf::Pose(tf::createQuaternionFromRPY(0,0,0), tf::Vector3(x,y,0)));
	pose.frame_id_ = sourceFrameId;
	pose.stamp_ = ros::Time(0);
	try
	{
		tf_listener.transformPose(targetFrameId, pose, tfPose);
	} catch(std::exception &e) {
		ROS_ERROR("%s",e.what());
		return false;
	}
	x = tfPose.getOrigin().getX();
	y = tfPose.getOrigin().getY();
	return true;
}

inline
bool Node::transformVelocity(double& vx, double& vy, const std::string& sourceFrameId, const std::string& targetFrameId) const
{
	tf::Stamped<tf::Vector3> vec,tfVec;

	vec.setData(tf::Vector3(vx,vy,0));
	vec.frame_id_ = sourceFrameId;
	vec.stamp_ = ros::Time(0);
	try
	{
		tf_listener.transformVector(targetFrameId, vec, tfVec);
	} catch(std::exception &e) {
		ROS_ERROR("%s",e.what());
		return false;
	}
	vx = tfVec.getX(); 
	vy = tfVec.getY();
	return true;
}


inline
Node::Node(ros::NodeHandle& n, ros::NodeHandle& pn)
: gen(std::chrono::system_clock::now().time_since_epoch().count()),
  tf_listener(ros::Duration(10))
{
	double freq,dt;
	bool publish_map_trans;
	int scan360_readings;
	double target_force_factor_desired;
	ros::Time current_time,previous_time;	
	std::string cmd_vel_id,cmd_vel_target_id,odom_id,scan360_id;
	pn.param<double>("freq",freq,15);
	pn.param<std::string>("scan360_id",scan360_id,"/scan360");
	pn.param<std::string>("config_file",config_file,"");
	pn.param<std::string>("cmd_vel_id",cmd_vel_id,"/cmd_vel");
	pn.param<std::string>("odom_id",odom_id,"/odom");
	pn.param<std::string>("cmd_vel_target_id",cmd_vel_target_id,"/target/cmd_vel");
	pn.param<bool>("publish_map_trans",publish_map_trans,true);
	pn.param<double>("pose_initial_x", pose_initial_x, 0);
    	pn.param<double>("pose_initial_y", pose_initial_y, 0);
 	pn.param<double>("pose_initial_yaw", pose_initial_yaw, 0);
	pn.param<double>("robot_radius",robot_radius,0.3);
	pn.param<double>("person_radius",person_radius,0.35);
	pn.param<int>("scan360_readings",scan360_readings,1440);
	pn.param<double>("scan_range_max",scan_range_max,10.0);	
	pn.param<double>("robot_max_velocity",robot_max_velocity,0.6);
	pn.param<bool>("teleoperated_target",teleoperated_target,false);
	pn.param<double>("people_average_vel",people_average_vel,0.9);
	pn.param<double>("people_sd_vel",people_sd_vel,0.001);
	pn.param<bool>("target_cyclic_goals",target_cyclic_goals,false);
	pn.param<double>("people_detection_range",people_detection_range,8.0);
	pn.param<bool>("noisy_detections",noisy_detections,false);
	pn.param<double>("sd_noise",sd_noise,0.01); //0.25
	pn.param<double>("sd_noise_theta",sd_noise_theta,1);
	pn.param<double>("false_negative_prob",false_negative_prob,0.1);
	pn.param<double>("target_force_factor_desired",target_force_factor_desired,4.0);

	pn.param<bool>("perfect_tracking",perfect_tracking,false);
	ros::Subscriber cmd_vel_target_sub;
	ros::Subscriber cmd_vel_sub = n.subscribe<geometry_msgs::Twist>(cmd_vel_id,1,&Node::robotCmdVelReceived,this);
	scan360_pub = pn.advertise<sensor_msgs::LaserScan>(scan360_id, 1);
	odom_pub = pn.advertise<nav_msgs::Odometry>(odom_id, 1);
	tf::StampedTransform map_trans(tf::Transform(tf::createQuaternionFromRPY(0.0,0.0,pose_initial_yaw), 
					tf::Vector3(pose_initial_x,pose_initial_y,0.0)), ros::Time::now(), "map", "odom");

	people_detected_markers_pub = pn.advertise<visualization_msgs::MarkerArray>("/plab/markers/detected_people", 1);	
	people_markers_pub = pn.advertise<animated_marker_msgs::AnimatedMarkerArray>("/plab/markers/people", 1);
	people_detected_pub = pn.advertise<people_msgs::People>("/people_detections", 1);	

	ros::Rate r(freq);
	ROS_INFO("Initiating...");
	sfm::MAP;
	a_star.init(config_file);
	initAgents();
	ROS_INFO("Ok");
	sfm::MAP.computeDistances();	
	if (teleoperated_target)   {
		cmd_vel_target_sub = n.subscribe<geometry_msgs::Twist>(cmd_vel_target_id,1,&Node::targetCmdVelReceived,this);
	}
	peopleDetected.resize(agents.size());
	if (target_index!=-1) {
		agents[target_index].params.forceFactorDesired = target_force_factor_desired;
	}
	scan360.header.frame_id="base_link";
	scan360.angle_min = -M_PI;
	scan360.angle_max = M_PI;
	scan360.angle_increment = 2*M_PI/(double)scan360_readings;	
	scan360.time_increment = 0.0;
	scan360.scan_time = 1.0/freq;
	scan360.range_min = robot_radius;
	scan360.range_max = scan_range_max;
	scan360.ranges.resize(scan360_readings);

	previous_time = ros::Time::now();
	while (n.ok()) {
		current_time = ros::Time::now();
		map_trans.stamp_ = current_time;
		tf_broadcaster.sendTransform(map_trans);
		dt = (current_time-previous_time).toSec();
		previous_time=current_time;
		sfm::SFM.computeForces(agents,&sfm::MAP);
		sfm::SFM.updatePosition(agents,dt);
		publishOdom(current_time);
		publishScan360(current_time);
		publishPeople(current_time);
		publishDetections(current_time);
		sfm::MAP.computeDistances();	
		r.sleep();
		ros::spinOnce();	
	}

}



inline
int Node::getPersonCollisionIndex(const utils::Vector2d& x, double collisionThreshold) const
{
	double d;
	for (unsigned i = 1; i< agents.size(); i++) {
		d = (agents[i].position - x).squaredNorm();
		if (d<=collisionThreshold) {
			return i;
		}
	}
	return -1;
}


inline
void Node::publishScan360(const ros::Time& current_time)
{
	utils::Vector2d x,u;
	double angle=agents[0].yaw.toRadian();
	int collisionIndex;
	double collisionThreshold =person_radius * person_radius;
	angle-=M_PI;
	double d;		
	double maxX = sfm::MAP.getInfo().width * sfm::MAP.getInfo().resolution;
	double maxY = sfm::MAP.getInfo().height * sfm::MAP.getInfo().resolution; 
	for (unsigned i=0;i<peopleDetected.size();i++) {
		peopleDetected[i]=perfect_tracking?1:0;
	}
	for (unsigned i=0;i<scan360.ranges.size();i++) {
		u.set(std::cos(angle)*sfm::MAP.getInfo().resolution,std::sin(angle)*sfm::MAP.getInfo().resolution);		
		x=agents[0].position;
		d=0;
		collisionIndex=-1;
		while(x.getX()>0 && x.getY()>0 && x.getX()<maxX && x.getY()<maxY && 
			d<scan_range_max && !sfm::MAP.isObstacle(x) && 
			(collisionIndex = getPersonCollisionIndex(x,collisionThreshold))==-1) {
			x+=u;
			d +=sfm::MAP.getInfo().resolution;
		}
		if (collisionIndex!=-1 && d<=people_detection_range) {
			peopleDetected[collisionIndex] = 1;
		}
		scan360.ranges[i] = std::max(robot_radius,std::min(d,scan_range_max));
		angle+=scan360.angle_increment;
	}
	scan360.header.stamp = current_time;
	scan360_pub.publish(scan360);	

}

inline
bool Node::publishDetections(const ros::Time& current_time)
{
	static std::uniform_real_distribution<double> false_negative(0,1);
	static std::normal_distribution<double> noise_pos(0,sd_noise);	
	static std::normal_distribution<double> noise_yaw(0,sd_noise_theta);
	
	static int pose_seq=0;

	people_msgs::People persons;
	persons.header.seq = pose_seq++;
	persons.header.stamp = current_time;
	persons.header.frame_id = "odom";
	
	char buf[100];

	for(unsigned i=1;i<agents.size();i++) {
		if (peopleDetected[i]==0 || (noisy_detections && false_negative(gen)<false_negative_prob)) {
			continue;
		}
		double x1 = agents[i].position.getX();
		double y1 = agents[i].position.getY();
		double theta1 = agents[i].yaw.toRadian();
		if (!transformPose(x1,y1,theta1,"map","odom")) {
			return false;
		}
		if (noisy_detections) {
			x1+=noise_pos(gen);
			y1+=noise_pos(gen);
			theta1+=noise_yaw(gen);
		}

		double vx1 = agents[i].velocity.getX();
		double vy1 = agents[i].velocity.getY();
		
		if (!transformVelocity(vx1,vy1,"map","odom")) {
			return false;
		}
		//TODO: add noise to velocity

		people_msgs::Person p;
		//p.detection_id = i-1;
		sprintf(buf, "%d", i-1);
		p.tags.push_back(buf);
		p.name = "";
		p.reliability = 0.5;
		p.tagnames.push_back("LASER");
		//p.modality = spencer_tracking_msgs::DetectedPerson::MODALITY_GENERIC_LASER_2D;
		p.position.x = x1;
		p.position.y = y1;
		p.position.z = 0;
		p.velocity.x = vx1;
		p.velocity.y = vy1;
		p.velocity.z = 0;
		/*p.pose.pose.orientation.x = 0;
		p.pose.pose.orientation.y = 0;
		p.pose.pose.orientation.z = sin(theta1/2.0);
		p.pose.pose.orientation.w = cos(theta1/2.0);
		p.pose.covariance[0] = 0.1*0.1;
		p.pose.covariance[7] = 0.1*0.1;
		p.pose.covariance[14] = 0.1*0.1;
		p.pose.covariance[21] = 100000.0;
		p.pose.covariance[28] = 100000.0;
		p.pose.covariance[35] = 1.8*1.8;*/
		persons.people.push_back(p);				
	}

	if(persons.people.size() > 0)
		people_detected_pub.publish(persons);	
	

	return true;
}



inline
bool Node::publishPeople(const ros::Time& current_time)
{
	animated_marker_msgs::AnimatedMarkerArray marker_array;
	for(unsigned i=1;i<agents.size();i++) {
		animated_marker_msgs::AnimatedMarker marker;
       		marker.mesh_use_embedded_materials = true;
		marker.header.frame_id = "map";
		marker.header.stamp = current_time;
		marker.id = i-1;
		marker.type = animated_marker_msgs::AnimatedMarker::MESH_RESOURCE;
		marker.mesh_resource = "package://pedlab/images/animated_walking_man.mesh";
		marker.pose.position.x = agents[i].position.getX();
		marker.pose.position.y = agents[i].position.getY();
		marker.action = 0; 
		marker.scale.x = PERSON_MESH_SCALE;
		marker.scale.y = PERSON_MESH_SCALE;
		marker.scale.z = PERSON_MESH_SCALE;
		marker.color.a = 1.0;
		if ((int)i == target_index) {
			marker.color.r = 0.412;
			marker.color.g =0.882;
			marker.color.b =0.255;
		} else {
			marker.color.r =0.882;
			marker.color.g = 0.412;
			marker.color.b =0.255;
		}
		marker.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(M_PI*0.5, 0.0, agents[i].yaw.toRadian()+M_PI*0.5);
  		marker.animation_speed = agents[i].velocity.norm() * 0.7;
		marker_array.markers.push_back(marker);
	}
	people_markers_pub.publish(marker_array);
	return true;
}


inline
bool Node::publishOdom(const ros::Time& current_time)
{
	double x = agents[0].position.getX();
	double y = agents[0].position.getY();
	double yaw = agents[0].yaw.toRadian();
	if (!transformPose(x,y,yaw,"map","odom")) {
		return false;
	}
	nav_msgs::Odometry odom;
	odom.header.frame_id = "odom";
	odom.child_frame_id = "base_link";
	odom.header.stamp = current_time;
	odom.pose.pose.position.x = x;
	odom.pose.pose.position.y = y;
	odom.pose.pose.position.z = 0.0;
	odom.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, yaw);
	odom.twist.twist.linear.x = agents[0].linearVelocity;
	odom.twist.twist.linear.y = 0.0; 
	odom.twist.twist.linear.z = 0.0;
	odom.twist.twist.angular.x = 0.0;
	odom.twist.twist.angular.y = 0.0;
	odom.twist.twist.angular.z = agents[0].angularVelocity;
	odom_pub.publish(odom);

	geometry_msgs::TransformStamped odom_trans;
	odom_trans.header.frame_id = "odom";
	odom_trans.child_frame_id = "base_link";
	odom_trans.transform.translation.z = 0;
	odom_trans.header.stamp = current_time;
	odom_trans.transform.translation.x = odom.pose.pose.position.x;
	odom_trans.transform.translation.y = odom.pose.pose.position.y;
	odom_trans.transform.rotation = odom.pose.pose.orientation;
	tf_broadcaster.sendTransform(odom_trans);
	return true;
}

inline
void Node::robotCmdVelReceived(const geometry_msgs::Twist::ConstPtr& cmd_vel)
{
	agents[0].linearVelocity = cmd_vel->linear.x;
	agents[0].angularVelocity = cmd_vel->angular.z;
}

inline
void Node::targetCmdVelReceived(const geometry_msgs::Twist::ConstPtr& cmd_vel)
{
	agents[1].linearVelocity = cmd_vel->linear.x;
	agents[1].angularVelocity = cmd_vel->angular.z;
}



}




int main(int argc, char** argv)
{
	ros::init(argc, argv, "pedlab");
	ros::NodeHandle n;
	ros::NodeHandle pn("~");
	plab::Node node(n,pn);
	return 0;
}
