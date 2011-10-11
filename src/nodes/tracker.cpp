#include <stdexcept>
#include <string>

#include <boost/format.hpp>
#include <boost/make_shared.hpp>

#include <LinearMath/btQuaternion.h>

#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>

#include <evart-client.h>

#include "tracker.hh"

namespace evart
{
  static uint32_t
  getBodyIdFromName(const std::string& name)
  {
    const evas_body_list_t* bodyList = evas_body_list();
    if(!bodyList)
      throw std::runtime_error("failed to retrieve body list");

    for(uint32_t i = 0; i < bodyList->nbodies; ++i)
      if(name == bodyList->bodies[i])
	return i;

    boost::format fmt("failed to retrieve the body id associated to '%1%'");
    fmt % name;
    throw std::runtime_error(fmt.str());
  }

  static uint32_t
  getSegmentIdFromName(uint32_t bodyId, const std::string& name)
  {
    const evas_body_segments_list_t* segmentList =
      evas_body_segments_list(bodyId);
    if(!segmentList)
      throw std::runtime_error("failed to retrieve segment list");

    for(uint32_t i = 0; i < segmentList->nsegments; ++i)
      if (name == segmentList->hier[i].name)
	return i;

    boost::format fmt
      ("failed to retrieve the segment id associated to '%1%' of body id '%2%'");
    fmt % name % bodyId;
    throw std::runtime_error(fmt.str());
  }


  Tracker::Tracker(ros::NodeHandle& nh,
		   const std::string& objectName,
		   const std::string& segmentName,
		   const std::string& topicName,
		   const std::string& referenceFrameName,
		   const std::string& childFrameName)
    : publisher_(),
      seq_(0),
      objectName_(objectName),
      segmentName_(segmentName),
      referenceFrameName_(referenceFrameName),
      childFrameName_(childFrameName),
      bodyId_(getBodyIdFromName(objectName)),
      segmentId_(getSegmentIdFromName (bodyId_, segmentName))
  {
    publisher_ =
      nh.advertise<geometry_msgs::TransformStamped>(topicName, queueSize);

    evas_body_segments(bodyId_, EVAS_ON);
  }

  Tracker::~Tracker()
  {
    evas_body_segments(bodyId_, EVAS_OFF);
  }

  void
  Tracker::callback(const evas_body_segments_t& msg)
  {
    if (msg.segments[segmentId_].pos[0] == EVAS_EMPTY)
      return; // No data.
    if (msg.nsegments <= segmentId_)
      throw std::runtime_error("invalid segment id");

    boost::shared_ptr<geometry_msgs::TransformStamped> t
      (new geometry_msgs::TransformStamped());

    t->header.seq = seq_++;
    t->header.stamp.sec = msg.tv_sec;
    t->header.stamp.nsec = msg.tv_usec * 1000.;
    t->header.frame_id = referenceFrameName_;

    t->child_frame_id = childFrameName_;

    t->transform.translation.x = msg.segments[segmentId_].pos[0];
    t->transform.translation.y = msg.segments[segmentId_].pos[1];
    t->transform.translation.z = msg.segments[segmentId_].pos[2];

    btQuaternion q;
    q.setEuler(msg.segments[segmentId_].rot[2],
	       msg.segments[segmentId_].rot[1],
	       msg.segments[segmentId_].rot[0]);
    t->transform.rotation.x = q.x();
    t->transform.rotation.y = q.y();
    t->transform.rotation.z = q.z();
    t->transform.rotation.w = q.w();

    publisher_.publish (t);
  }

} // end of namespace evart.
