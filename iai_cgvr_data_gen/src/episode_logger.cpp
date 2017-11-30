#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

geometry_msgs::Vector3 create_vector3(double x, double y, double z)
{
    geometry_msgs::Vector3 msg;
    msg.x = x;
    msg.y = y;
    msg.z = z;
    return msg;
}

std_msgs::ColorRGBA create_rgba(float r, float g, float b, float a)
{
    std_msgs::ColorRGBA msg;
    msg.r = r;
    msg.g = g;
    msg.b = b;
    msg.a = a;
    return msg;
}

class EpisodeLogger{
public:
    EpisodeLogger(const ros::NodeHandle& nh) :
            nh_(nh)
    {
        read_parameters();
    }

    ~EpisodeLogger() {}

protected:
    ros::NodeHandle nh_;
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
        object.header.frame_id = "world";
        object.header.stamp = ros::Time::now();
        object.type = visualization_msgs::Marker::MESH_RESOURCE;
        object.pose = read_object_pose(object_name);
        object.mesh_resource = readParam<std::string>("/objects/" + object_name + "/mesh_resource");
        object.lifetime = ros::Duration(0.0);
        object.scale = create_vector3(1.0, 1.0, 1.0);
        object.color = create_rgba(0.0, 0.0, 0.0, 1.0);
        object.mesh_use_embedded_materials = true;
    }

    geometry_msgs::Pose read_object_pose(const std::string& object_name)
    {
        // TODO: implement me
        return geometry_msgs::Pose();
    }

    void read_urdf()
    {
        // TODO: implement me
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
