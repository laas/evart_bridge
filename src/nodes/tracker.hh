#ifndef EVART_ROS_TRACKER_HH
# define EVART_ROS_TRACKER_HH
# include <string>
# include <boost/optional.hpp>
# include <boost/shared_ptr.hpp>
# include <ros/ros.h>
# include <tf/transform_broadcaster.h>

# include <evart-client.h>

namespace evart
{
  class Tracker
  {
  public:
    static const int queueSize = 5;

    explicit Tracker(ros::NodeHandle& nh,
		     const std::string& objectName,
		     const std::string& segmentName,
		     const std::string& topicName,
		     const std::string& referenceFrameName,
		     const std::string& childFrameName,
		     boost::optional<tf::TransformBroadcaster&>
		     transformBroadcaster);
    virtual ~Tracker();

    void callback (const evas_body_segments_t& msg);

    unsigned segmentId () const
    {
      return segmentId_;
    }

    unsigned bodyId () const
    {
      return bodyId_;
    }

    const std::string& objectName () const
    {
      return objectName_;
    }

    const std::string& childFrameName () const
    {
      return childFrameName_;
    }

  private:
    ros::Publisher publisher_;
    uint32_t seq_;

    std::string objectName_;
    std::string segmentName_;
    const std::string& referenceFrameName_;
    std::string childFrameName_;

    uint32_t bodyId_;
    uint32_t segmentId_;
    boost::optional<tf::TransformBroadcaster&> transformBroadcaster_;
  };

  typedef boost::shared_ptr<Tracker> TrackerShPtr;

} // end of namespace evart.

#endif //! EVART_ROS_TRACKER_HH
