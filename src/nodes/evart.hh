#ifndef EVART_ROS_EVART_HH
# define EVART_ROS_EVART_HH
# include <string>
# include <vector>

# include <boost/optional.hpp>

# include <ros/ros.h>
# include <tf/transform_broadcaster.h>

# include "tracker.hh"

# include "evart_ros/List.h"
# include "evart_ros/TrackSegment.h"

namespace evart
{
  class Evart
  {
  public:
    Evart();
    virtual ~Evart();

    void spin();

    bool trackSegments(evart_ros::TrackSegment::Request& req,
		       evart_ros::TrackSegment::Response& res);
    bool listSegments(evart_ros::List::Request& req,
		      evart_ros::List::Response& res);

  private:
    ros::NodeHandle nodeHandle_;

    std::vector<TrackerShPtr> trackers_;

    std::string evartHost_;
    int evartPort_;
    bool enableTfBroadcast_;
    std::string tfReferenceFrameName_;

    ros::ServiceServer trackSegmentsSrv_;
    ros::ServiceServer listSegmentsSrv_;

    tf::TransformBroadcaster transformBroadcaster_;

    double updateRate_;
  };

} // end of namespace evart.

#endif //! EVART_ROS_EVART_HH
