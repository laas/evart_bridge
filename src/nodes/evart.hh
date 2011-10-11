#ifndef EVART_ROS_EVART_HH
# define EVART_ROS_EVART_HH
# include <string>

# include <ros/ros.h>

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
    std::string tfReferenceFrameName_;

    ros::ServiceServer trackSegmentsSrv_;
    ros::ServiceServer listSegmentsSrv_;
  };

} // end of namespace evart.

#endif //! EVART_ROS_EVART_HH
