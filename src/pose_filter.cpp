#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"

class PoseFilter
{
public:
    PoseFilter(std::string topic_name,std::string ref_frame,std::string param_name) :
        n_private_("~"),
        tf_(),
        target_frame_(ref_frame),
        topic_name_(topic_name),
        param_name_(param_name)
    {
        ROS_INFO("Filter subscribed to topic:  %s", topic_name_.c_str());
        pose_sub_.subscribe(n_,topic_name_, 10);
        tf_filter_ = new tf::MessageFilter<geometry_msgs::TransformStamped>(pose_sub_, tf_, target_frame_, 10);
        tf_filter_->registerCallback( boost::bind(&PoseFilter::msgCallback, this, _1) );
        publisher_ = n_.advertise<geometry_msgs::TransformStamped>(topic_name_ + "_filt", 10);

        quat_param_.resize(4);
        origin_param_.resize(3);

        // Retrain the transformation from the parameter server
        ROS_INFO("Loading transformation parameters from %s/quat ",param_name_.c_str());
        n_.param(param_name_+"/quat/x", quat_param_[0], 0.0);
        n_.param(param_name_+"/quat/y", quat_param_[1], 0.0);
        n_.param(param_name_+"/quat/z", quat_param_[2], 0.0);
        n_.param(param_name_+"/quat/w", quat_param_[3], 1.0);

        ROS_INFO("%s/quat/x %f",param_name_.c_str(),quat_param_[0]);
        ROS_INFO("%s/quat/y %f",param_name_.c_str(),quat_param_[1]);
        ROS_INFO("%s/quat/z %f",param_name_.c_str(),quat_param_[2]);
        ROS_INFO("%s/quat/w %f",param_name_.c_str(),quat_param_[3]);

        ROS_INFO("Loading transformation parameters from %s/origin ",param_name_.c_str());
        n_.param(param_name_+"/origin/x", origin_param_[0], 0.0);
        n_.param(param_name_+"/origin/y", origin_param_[1], 0.0);
        n_.param(param_name_+"/origin/z", origin_param_[2], 0.0);

        ROS_INFO("%s/origin/x %f",param_name_.c_str(),origin_param_[0]);
        ROS_INFO("%s/origin/y %f",param_name_.c_str(),origin_param_[1]);
        ROS_INFO("%s/origin/z %f",param_name_.c_str(),origin_param_[2]);
    }

private:
    message_filters::Subscriber<geometry_msgs::TransformStamped> pose_sub_;
    tf::TransformListener tf_;
    tf::MessageFilter<geometry_msgs::TransformStamped> * tf_filter_;
    ros::NodeHandle n_, n_private_;
    std::string target_frame_;
    std::string topic_name_;
    std::string param_name_;
    ros::Publisher publisher_;

    // Attributes used to adjust the transformation
    std::vector<double> quat_param_;
    std::vector<double> origin_param_;
    tf::StampedTransform adapt_transf, transf;
    tf::Transform out_transf_;
    tf::Quaternion quat;
    tf::Vector3 vect;

    //  Callback to register with tf::MessageFilter to be called when TransformStampeds are available
    void msgCallback(const boost::shared_ptr<const geometry_msgs::TransformStamped>& transf_ptr)
    {
        geometry_msgs::TransformStamped transf_out;
        geometry_msgs::PoseStamped pose_out;
        geometry_msgs::PoseStamped pose_in;

        try
        {
            quat.setValue(transf_ptr->transform.rotation.x,transf_ptr->transform.rotation.y,transf_ptr->transform.rotation.z,transf_ptr->transform.rotation.w);
            vect.setValue(transf_ptr->transform.translation.x,transf_ptr->transform.translation.y,transf_ptr->transform.translation.z);
            transf.setRotation(quat);
            transf.setOrigin(vect);

            // Adapt
            quat.setValue(quat_param_[0],quat_param_[1],quat_param_[2],quat_param_[3]);
            vect.setValue(origin_param_[0],origin_param_[1],origin_param_[2]);
            adapt_transf.setRotation(quat);
            adapt_transf.setOrigin(vect);
            out_transf_ = transf * adapt_transf;

            vect = out_transf_.getOrigin();
            quat = out_transf_.getRotation();

            pose_in.header = transf_ptr->header;
            pose_in.pose.position.x = vect.getX();
            pose_in.pose.position.y = vect.getY();
            pose_in.pose.position.z = vect.getZ();

            pose_in.pose.orientation.x = quat.getX();
            pose_in.pose.orientation.y = quat.getY();
            pose_in.pose.orientation.z = quat.getZ();
            pose_in.pose.orientation.w = quat.getW();

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
    if(argc != 3)
        ROS_ERROR_STREAM("Wrong number of arguments");

    ros::init(argc, argv, "pose_filter"); //Init ROS

    PoseFilter pd(argv[1],argv[2],argv[3]);

    ros::spin(); // Run until interupted
};
