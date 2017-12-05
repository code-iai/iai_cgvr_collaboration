#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseStamped.h>

std_msgs::Header create_header(const std::string& frame_id,
    const ros::Time& stamp)
{
    std_msgs::Header header;
    header.frame_id = frame_id;
    header.stamp = stamp;
    ROS_INFO_STREAM(header);
    return header;
}

geometry_msgs::Vector3 create_vector3(double x, double y, double z)
{
    geometry_msgs::Vector3 msg;
    msg.x = x;
    msg.y = y;
    msg.z = z;
    ROS_INFO_STREAM(msg);
    return msg;
}

geometry_msgs::Point create_point(double x, double y, double z)
{
    geometry_msgs::Point msg;
    msg.x = x;
    msg.y = y;
    msg.z = z;
    ROS_INFO_STREAM(msg);
    return msg;
}

geometry_msgs::Quaternion create_quaternion(double x, double y, double z, double w)
{
    geometry_msgs::Quaternion msg;
    msg.x = x;
    msg.y = y;
    msg.z = z;
    msg.w = w;
    ROS_INFO_STREAM(msg);
    return msg;
}

geometry_msgs::Pose create_pose(const geometry_msgs::Point& position, const geometry_msgs::Quaternion& orientation)
{
    geometry_msgs::Pose msg;
    msg.position = position;
    msg.orientation = orientation;
    ROS_INFO_STREAM(msg);
    return msg;
}

geometry_msgs::PoseStamped create_pose_stamped(const std_msgs::Header& header,
    const geometry_msgs::Pose& pose)
{
    geometry_msgs::PoseStamped msg;
    msg.pose = pose;
    msg.header = header;
    ROS_INFO_STREAM(msg);
    return msg;
}

std_msgs::ColorRGBA create_rgba(float r, float g, float b, float a)
{
    std_msgs::ColorRGBA msg;
    msg.r = r;
    msg.g = g;
    msg.b = b;
    msg.a = a;
    ROS_INFO_STREAM(msg);
    return msg;
}

class EpisodeLogger{
public:
    EpisodeLogger(const ros::NodeHandle& nh) :
            nh_(nh), marker_pub_(nh_.advertise<visualization_msgs::MarkerArray>("/visualization_marker_array", 1, true))
    {
        ROS_INFO("Reading params.");
        read_parameters();
        ROS_INFO("Publishing markers.");
        publish_markers();
        ROS_INFO("Done.");
    }

    ~EpisodeLogger() {}

protected:
    ros::NodeHandle nh_;
    ros::Publisher marker_pub_;
    std::map<std::string, visualization_msgs::Marker> objects_;

    void read_parameters()
    {
        read_objects();
        read_urdf();
    }

    void read_objects()
    {
        std::vector<std::string> object_names = readParamNoThrow< std::vector<std::string> >("objects/object_names");

        for (auto const & object_name: object_names)
            objects_.insert(std::make_pair(object_name, read_object(object_name)));
    }

    visualization_msgs::Marker read_object(const std::string& object_name)
    {
        visualization_msgs::Marker object;
        geometry_msgs::PoseStamped pose = read_object_pose(object_name);
        object.header.frame_id = pose.header.frame_id;
        object.header.stamp = ros::Time::now();
        object.ns = object_name;
        object.type = visualization_msgs::Marker::MESH_RESOURCE;
        object.action = visualization_msgs::Marker::ADD;
        object.pose = pose.pose;
        object.mesh_resource = readParam<std::string>("objects/" + object_name + "/mesh_resource");
        object.lifetime = ros::Duration(0.0);
        object.scale = create_vector3(1.0, 1.0, 1.0);
        object.color = create_rgba(1.0, 1.0, 1.0, 1.0);
        object.mesh_use_embedded_materials = true;

        return object;
    }

    geometry_msgs::Point read_position(const std::string& object_name)
    {
        std::string ns = "objects/" + object_name + "/pose/position/";
        return create_point(readParam<double>(ns + "x"), readParam<double>(ns + "y"), readParam<double>(ns + "z"));
    }

    geometry_msgs::Quaternion read_orientation(const std::string& object_name)
    {
        std::string ns = "objects/" + object_name + "/pose/orientation/";
        return create_quaternion(
                readParam<double>(ns + "x"), readParam<double>(ns + "y"), readParam<double>(ns + "z"), readParam<double>(ns + "w"));
    }

    geometry_msgs::PoseStamped read_object_pose(const std::string& object_name)
    {
        std::string ns = "objects/" + object_name + "/pose/";
        return create_pose_stamped(
            create_header(readParam<std::string>(ns + "frame_id"), ros::Time(0.0)),
            create_pose(read_position(object_name), read_orientation(object_name)));
    }

    void read_urdf()
    {
        // TODO: implement me
    }

    void publish_markers() const
    {
        visualization_msgs::MarkerArray msg;
        for (auto const & name_marker_pair : objects_)
            msg.markers.push_back(name_marker_pair.second);
        marker_pub_.publish(msg);
    }

    template<class T>
    inline T readParam(const std::string& name)
    {
      T param;
      if(!nh_.getParam(name, param))
        throw std::runtime_error("Could not find parameter '" + name + "' in namespace '" + nh_.getNamespace() + "'.");
      return param;
    }

    template<class T>
    inline T readParamNoThrow(const std::string& name)
    {
        try{
            return readParam<T>(name);
        }
        catch (const std::exception& e)
        {
            ROS_WARN("%s", e.what());
        }

        T default_result;
        return default_result;
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "episode_logger");
    ros::NodeHandle nh("~");

    try {
        EpisodeLogger my_logger(nh);
        ros::spin();
    }
    catch (const std::exception& e)
    {
        ROS_ERROR("%s", e.what());
    }

    return 0;
}
