#include <ros/ros.h>

#include <geometry_msgs/PointStamped.h>
#include <lightsfm/vector2d.hpp>
#include <visualization_msgs/MarkerArray.h>
#include <list>
#include <map>
#include <iostream>
#include <fstream>

namespace plab
{

class EditionNode
{
public:
	
	struct Edge
	{
		unsigned src;
		unsigned dst;
	};


	EditionNode(ros::NodeHandle& n, ros::NodeHandle& pn);
	~EditionNode() {}
	


private:
	void saveFile();
	void pointReceived(const geometry_msgs::PointStamped::ConstPtr& point); 
	std::string file;
	std::map<unsigned,utils::Vector2d> nodes;
	std::list<Edge> edges;
	ros::NodeHandle& pn;
	const unsigned *clicked;
	bool delete_node;
	unsigned counter;

	double distance;
	
};


void EditionNode::saveFile()
{
	std::ofstream stream;
  	stream.open (file.c_str());
	stream<<"<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";
	stream<<"<scenario>\n";
	stream<<"\t<!-- goals -->\n";
	stream<<"\t<!-- waypoints -->\n";
	
	for (auto it = nodes.begin(); it!=nodes.end(); ++it) {
		stream<<"\t<waypoint id=\""<<(it->first)<<
			"\" x=\""<<(it->second.getX())<<"\" y=\""<<(it->second.getY())<<"\" r=\"0.5\"/>\n";
	}
	stream<<"\t<!-- edges -->\n";
	for (auto it= edges.begin(); it!= edges.end(); ++it) {
		stream<<"\t<edge from=\""<<(it->src)<<"\" to=\""<<(it->dst)<<"\"/>\n";
	}
	stream<<"</scenario>\n";
	stream.close();

}


inline
void EditionNode::pointReceived(const geometry_msgs::PointStamped::ConstPtr& point)
{
	
	utils::Vector2d v(point->point.x, point->point.y);
	// To delete nodes, click first on the first cell of the map and later on the node to delete
	if (v.getX()<=1.0 && v.getY()<=1.0) {
		delete_node=true;
		clicked=NULL;
		return;
	} 

	nodes[counter++] = v;

	//clicked = &(nodes[counter-1].first);

	auto it = nodes.begin();
	while (it != nodes.end()) {
		if ( ((it->second - v).norm()<distance) && (it->first != counter-1)) {
			if (delete_node) {
				ROS_INFO("Deleting nodes");
				auto it1=edges.begin();
				while(it1!=edges.end()) {
					if ( (it->second - nodes.at(it1->src)).norm()<distance || (it->second - nodes.at(it1->dst)).norm()<distance ) {
						it1 = edges.erase(it1);
					} else {
						++it1;
					}
				}
				clicked=NULL;
				nodes.erase(it);
			} else {
				ROS_INFO("Adding edge");
				Edge edge;
				edge.src = counter-1;
				edge.dst = it->first;
				edges.push_back(edge);
				clicked=NULL;
			//} else {
			//	ROS_INFO("Preparing clicked");
			//	clicked = &(it->first);
			}
			//delete_node=false;
			//return;
		} //else {
		
		++it;
		//}	
	}
	delete_node=false;
	clicked=NULL;
	
	
}


inline
EditionNode::EditionNode(ros::NodeHandle& n, ros::NodeHandle& pn)
: pn(pn),
  clicked(NULL),
  delete_node(false),
  counter(0)
{
	double freq;
	pn.param<std::string>("file",file,"graph.xml");
	pn.param<double>("freq",freq,15);
	pn.param<double>("distance",distance,3.0);

	ros::Subscriber point_sub = n.subscribe<geometry_msgs::PointStamped>("/clicked_point",1,&EditionNode::pointReceived,this);
	ros::Publisher nodes_pub = pn.advertise<visualization_msgs::MarkerArray>("/plab/markers/edited_nodes", 1);	
	ros::Publisher edges_pub = pn.advertise<visualization_msgs::Marker>("/plab/markers/edited_edges", 1);	
	ros::Rate r(freq);
	while (n.ok()) {
		unsigned index=0;
		visualization_msgs::MarkerArray node_markers; 
		for (auto it = nodes.begin(); it != nodes.end(); ++it) {
			visualization_msgs::Marker marker;
			marker.header.frame_id = "map";
          		marker.header.stamp = ros::Time::now();
			marker.ns = "node_markers";
			marker.type = visualization_msgs::Marker::SPHERE;
			marker.id = index++;
			marker.action = visualization_msgs::Marker::ADD;
			marker.color.a = 1.0;			
			marker.color.r = 0.0;
            		marker.color.g = 0.0;
            		marker.color.b = 1.0;
			marker.lifetime = ros::Duration(0.5);
			marker.scale.x = 0.5;
            		marker.scale.y = 0.5;
            		marker.scale.z = 0.5;
            		marker.pose.position.x = it->second.getX();
			marker.pose.position.y = it->second.getY();
			node_markers.markers.push_back(marker);	
		}
		nodes_pub.publish(node_markers);
		visualization_msgs::Marker marker; 
		marker.header.frame_id="map";
		marker.header.stamp =  ros::Time::now();
		marker.type =  visualization_msgs::Marker::LINE_LIST;
		marker.action = visualization_msgs::Marker::ADD;
		marker.color.a = 1.0;			
		marker.color.r = 0.0;
            	marker.color.g = 0.0;
            	marker.color.b = 1.0;
		marker.lifetime = ros::Duration(0.5);
		marker.scale.x = 0.1;
            	for (auto it = edges.begin(); it!= edges.end(); ++it) {
			geometry_msgs::Point p0,p1;
			p0.x = nodes.at(it->src).getX();
			p0.y = nodes.at(it->src).getY();
			p1.x = nodes.at(it->dst).getX();
			p1.y = nodes.at(it->dst).getY();
			marker.points.push_back(p0);
			marker.points.push_back(p1);
		}
		edges_pub.publish(marker);

		r.sleep();	
		ros::spinOnce();	
	}
	saveFile();

}

}
int main(int argc, char** argv)
{
	ros::init(argc, argv, "graph_edition");
	ros::NodeHandle n;
	ros::NodeHandle pn("~");
	plab::EditionNode node(n,pn);
	return 0;
}
