#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"

//TODO: define a template


class PoseFilter
{
public:
    PoseFilter(std::string topic_name,std::string ref_frame) : tf_(),  target_frame_(ref_frame), topic_name_(topic_name)
    {
        ROS_INFO("Topic is:  %s", topic_name_.c_str());
        pose_sub_.subscribe(n_,topic_name_, 10);
        tf_filter_ = new tf::MessageFilter<geometry_msgs::TransformStamped>(pose_sub_, tf_, target_frame_, 10);
        tf_filter_->registerCallback( boost::bind(&PoseFilter::msgCallback, this, _1) );
        publisher_ = n_.advertise<geometry_msgs::TransformStamped>(topic_name_ + "_filt", 10);
    }

private:
    message_filters::Subscriber<geometry_msgs::TransformStamped> pose_sub_;
    tf::TransformListener tf_;
    tf::MessageFilter<geometry_msgs::TransformStamped> * tf_filter_;
    ros::NodeHandle n_;
    std::string target_frame_;
    std::string topic_name_;
    ros::Publisher publisher_;

    //  Callback to register with tf::MessageFilter to be called when TransformStampeds are available
    void msgCallback(const boost::shared_ptr<const geometry_msgs::TransformStamped>& transf_ptr)
    {
        geometry_msgs::TransformStamped transf_out;
        geometry_msgs::PoseStamped pose_out;
        geometry_msgs::PoseStamped pose_in;

        try
        {

            pose_in.header = transf_ptr->header;
            pose_in.pose.position.x = transf_ptr->transform.translation.x;
            pose_in.pose.position.y = transf_ptr->transform.translation.y;
            pose_in.pose.position.z = transf_ptr->transform.translation.z;

            pose_in.pose.orientation.x = transf_ptr->transform.rotation.x;
            pose_in.pose.orientation.y = transf_ptr->transform.rotation.y;
            pose_in.pose.orientation.z = transf_ptr->transform.rotation.z;
            pose_in.pose.orientation.w = transf_ptr->transform.rotation.w;

            tf_.transformPose(target_frame_, pose_in, pose_out);

            transf_out.header = pose_out.header;
            transf_out.transform.translation.x = pose_out.pose.position.x;
            transf_out.transform.translation.y = pose_out.pose.position.y;
            transf_out.transform.translation.z = pose_out.pose.position.z;

            transf_out.transform.rotation.x = pose_out.pose.orientation.x;
            transf_out.transform.rotation.y = pose_out.pose.orientation.y;
            transf_out.transform.rotation.z = pose_out.pose.orientation.z;
            transf_out.transform.rotation.w = pose_out.pose.orientation.w;


            publisher_.publish(transf_out);

        }
        catch (tf::TransformException &ex)
        {
            printf ("Failure %s\n", ex.what()); //Print exception which was caught
        }
    };
};


/*
class PoseFilterHead
{
public:
    PoseFilterHead(std::string topic_name,std::string ref_frame) : tf_(),  target_frame_(ref_frame), topic_name_(topic_name)
    {
        ROS_INFO("Topic is:  %s", topic_name_.c_str());
        pose_sub_.subscribe(n_,topic_name_, 10);
        tf_filter_ = new tf::MessageFilter<geometry_msgs::Vector3Stamped>(pose_sub_, tf_, target_frame_, 10);
        tf_filter_->registerCallback( boost::bind(&PoseFilterHead::msgCallback, this, _1) );
        publisher_ = n_.advertise<geometry_msgs::Vector3>(topic_name_ + "_filt", 10);
    }

private:
    message_filters::Subscriber<geometry_msgs::Vector3Stamped> pose_sub_;
    tf::TransformListener tf_;
    tf::MessageFilter<geometry_msgs::Vector3Stamped> * tf_filter_;
    ros::NodeHandle n_;
    std::string target_frame_;
    std::string topic_name_;
    ros::Publisher publisher_;

    //  Callback to register with tf::MessageFilter to be called when Vector3Stamped are available
    void msgCallback(const boost::shared_ptr<const geometry_msgs::Vector3Stamped>& pose_ptr)
    {
        geometry_msgs::Vector3Stamped pose_out;
        geometry_msgs::Vector3 vect_out;
        try
        {
            tf_.transformVector(target_frame_, *pose_ptr, pose_out);

            vect_out = pose_out.vector;

            publisher_.publish(vect_out);
        }
        catch (tf::TransformException &ex)
        {
            printf ("Failure %s\n", ex.what()); //Print exception which was caught
        }
    };

};
*/

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "pose_filter"); //Init ROS

    PoseFilter pd(argv[1],argv[2]);

    ros::spin(); // Run until interupted
};
