#include <stdexcept>
#include <boost/bind.hpp>
#include <boost/foreach.hpp>
#include <boost/format.hpp>
#include <boost/make_shared.hpp>
#include <ros/ros.h>

#include <evart-client.h>

#include "evart.hh"

namespace evart
{
  Evart::Evart()
    : nodeHandle_("evart"),
      evartHost_(),
      evartPort_(),
      tfReferenceFrameName_ (),
      trackSegmentsSrv_ (),
      listSegmentsSrv_ ()
  {
    ros::param::param<std::string>("~hostname", evartHost_, EVAS_STREAM_HOST);
    ros::param::param<int>("~port", evartPort_, EVAS_STREAM_PORT);

    ros::param::param<std::string>
      ("~tf_ref_frame_id", tfReferenceFrameName_, "/mocap_world");

    evas_sethost(evartHost_.c_str());
    evas_setport(evartPort_);

    typedef boost::function<bool
      (evart_ros::List::Request&, evart_ros::List::Response&)> listCallback_t;

    listCallback_t listCallback =
      boost::bind(&Evart::listSegments, this, _1, _2);
    listSegmentsSrv_ = nodeHandle_.advertiseService("list_segments",
						    listCallback);

    typedef boost::function<
    bool (evart_ros::TrackSegment::Request&,
	  evart_ros::TrackSegment::Response&)> trackSegmentCallback_t;

  trackSegmentCallback_t trackSegmentsCallback =
    boost::bind(&Evart::trackSegments, this, _1, _2);
  trackSegmentsSrv_ = nodeHandle_.advertiseService("track_segments",
						   trackSegmentsCallback);
}

  Evart::~Evart()
  {}

bool
Evart::trackSegments(evart_ros::TrackSegment::Request& req,
		     evart_ros::TrackSegment::Response& res)
{


  std::string topicName;
  {
    boost::format fmt("%1%/%2%");
    fmt % req.body_name % req.segment_name;
    topicName = fmt.str();
  }

  std::string childFrameName;
  {
    boost::format fmt("%1%/%2%/%3%");
    fmt % tfReferenceFrameName_ % req.body_name % req.segment_name;
    childFrameName = fmt.str();
  }

  TrackerShPtr ptr (new Tracker
		    (nodeHandle_,
		     req.body_name,
		     req.segment_name,
		     topicName,
		     tfReferenceFrameName_,
		     childFrameName));
  trackers_.push_back(ptr);
  return true;
}

bool
Evart::listSegments(evart_ros::List::Request& req,
		    evart_ros::List::Response& res)
{
  const evas_body_list_t* bodies = evas_body_list();
  if (!bodies)
    {
      ROS_ERROR("failed to retrieve bodies");
      return false;
    }

  evart_ros::Body bodyMsg;
  evart_ros::Segment segmentMsg;
  evart_ros::Dof dofMsg;

  for (uint32_t body = 0; body < bodies->nbodies; ++body)
    {
      const evas_body_segments_list_t* segments =
	evas_body_segments_list(body);
      const evas_body_markers_list_t* markers =
	evas_body_markers_list(body);
      const evas_body_dofs_list_t* dofs =
	evas_body_dofs_list(body);

      if (!segments)
	{
	  ROS_ERROR_STREAM("failed to retrieve segments for body "
			   << bodies->bodies[body]);
	  continue;
	}

      if(!markers)
	{
	  ROS_ERROR_STREAM("failed to retrieve markers for body "
			   << bodies->bodies[body]);
	  continue;
	}

      if(!dofs)
	{
	  ROS_ERROR_STREAM("failed to retrieve dofs for body "
			   << bodies->bodies[body]);
	  continue;
	}

      bodyMsg.name = bodies->bodies[body];

      bodyMsg.segments.clear();
      for(uint32_t segment = 0; segment < segments->nsegments; ++segment)
	{
	  segmentMsg.name = segments->hier[segment].name;
	  segmentMsg.parent = segments->hier[segment].parent;
	  bodyMsg.segments.push_back(segmentMsg);
	}

      bodyMsg.dofs.clear();
      for(uint32_t dof = 0; dof < dofs->ndofs; ++dof)
	{
	  dofMsg.name = dofs->dofs[dof];
	  bodyMsg.dofs.push_back(dofMsg);
	}
      res.bodies.push_back(bodyMsg);
    }
  return true;
}

void
Evart::spin()
{
  // Unpoll as many messages as possible to avoid receiving
  // obsolete message kept in the buffer.
  evas_msg_t msg;
  evas_sethandler (0, 0);
  while (evas_recv (&msg, 0.001))
    {}

  ros::Rate loopRateTracking(100);
  while(ros::ok())
    {
      evas_recv (&msg, 0.001);
      if (msg.type == EVAS_BODY_SEGMENTS)
	{
	  BOOST_FOREACH (TrackerShPtr tracker, trackers_)
	    for (unsigned i = 0; i < msg.body_segments.nsegments; ++i)
	      if (tracker->segmentId () == i)
		tracker->callback (msg.body_segments);
	}
      ros::spinOnce();
      loopRateTracking.sleep();
    }
}

} // end of namespace evart.


int main(int argc, char **argv)
{
  ros::init(argc, argv, "evart");
  try
    {
      evart::Evart evart;
      if (ros::ok())
	evart.spin();
    }
  catch(std::exception& e)
    {
      ROS_ERROR_STREAM("fatal error: " << e.what());
      return 1;
    }
  catch(...)
    {
      ROS_ERROR_STREAM("unexpected error");
      return 2;
    }
  return 0;
}
