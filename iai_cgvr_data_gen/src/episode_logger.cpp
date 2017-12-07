#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/JointState.h>
#include <std_srvs/Trigger.h>
#include <json.hpp>
#include <fstream>
#include <ros/package.h>
#include <urdf/model.h>
#include <tinyxml2.h>

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
            nh_(nh),
            marker_pub_(nh_.advertise<visualization_msgs::MarkerArray>("/visualization_marker_array", 1, true)),
            trigger_server_(nh_.advertiseService("emit_json", &EpisodeLogger::emit_callback, this)),
            js_sub_(nh_.subscribe("/joint_states", 1, &EpisodeLogger::js_callback, this))
    {
        read_parameters();
        publish_markers();
    }

    ~EpisodeLogger() {}

protected:
    ros::NodeHandle nh_;
    ros::Publisher marker_pub_;
    ros::ServiceServer trigger_server_;
    ros::Subscriber js_sub_;
    std::map<std::string, visualization_msgs::Marker> objects_;
    std::vector<sensor_msgs::JointState> joint_states_;
    urdf::Model robot_model_;
    tinyxml2::XMLDocument robot_srdf_;

    std::string strip_resource(const std::string& resource, const std::string& package)
    {
        return resource.substr(resource.find(package) + package.length());
    }

    bool emit_callback(std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response)
    {
        using json = nlohmann::json;
        json j;

        for (auto const & object_pair: objects_)
            j["objects"][object_pair.first] = object_pair.second.mesh_resource;

        // TODO: complete me with link info
        for (auto const & link_pair: robot_model_.links_)
            if (link_pair.second->visual.get() &&
                link_pair.second->visual->geometry.get() &&
                link_pair.second->visual->geometry->type == urdf::Geometry::MESH)
            {
                std::string mesh_resource =
                        boost::static_pointer_cast<urdf::Mesh>(link_pair.second->visual->geometry)->filename;
                std::string package_name ="pr2_description/";
                j["objects"][link_pair.first] = strip_resource(mesh_resource, package_name);
            }


        // TODO: complete me with transform info
        joint_states_.clear();

        // fill disabled collision checks
        j["disabled-collision-checks"] = disabled_collision_checks();

        std::string filename =
            ros::package::getPath("iai_cgvr_data_gen") + "/episode_" + std::to_string(ros::Time::now().toSec()) + ".json";
        std::ofstream o(filename);
        o << std::setw(4) << j << std::endl;

        return true;
    }

    void js_callback(const sensor_msgs::JointState::ConstPtr& msg)
    {
        joint_states_.push_back(*msg);
    }

    void read_parameters()
    {
        read_objects();
        read_urdf();
        read_srdf();
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
        robot_model_.initParam("/robot_description");
    }

    void read_srdf()
    {
        robot_srdf_.Parse(readParam<std::string>("/robot_srdf").c_str());
        if (robot_srdf_.Error())
            throw std::runtime_error("Error parsing SRDF into XML document.");
    }

    nlohmann::json disabled_collision_checks()
    {
        nlohmann::json j {};

        tinyxml2::XMLNode* robot = robot_srdf_.FirstChildElement("robot");
        if (robot == nullptr)
            throw std::runtime_error("Did not find element 'robot' in robot srdf.");

        tinyxml2::XMLNode* disable_collisions = robot->FirstChildElement("disable_collisions");
        while (disable_collisions != nullptr)
        {
            nlohmann::json disabled_collision;
            const tinyxml2::XMLElement* elem = disable_collisions->ToElement();
            if (elem == nullptr)
                throw std::runtime_error ("Could not cast disable_collisions node into element.");

            // TODO: refactor this into a method
            const tinyxml2::XMLAttribute* link1 = elem->FindAttribute("link1");
            if (link1 == nullptr)
                throw std::runtime_error("Could not find attribute link1.");

            disabled_collision["link1"] = link1->Value();

            const tinyxml2::XMLAttribute* link2 = elem->FindAttribute("link2");
            if (link2 == nullptr)
                throw std::runtime_error("Could not find attribute link2.");

            disabled_collision["link2"] = link2->Value();

            const tinyxml2::XMLAttribute* reason = elem->FindAttribute("reason");
            if (reason == nullptr)
                throw std::runtime_error("Could not find attribute reason.");

            disabled_collision["reason"] = reason->Value();

            j.push_back(disabled_collision);

            disable_collisions = disable_collisions->NextSiblingElement("disable_collisions");
        }

        return j;
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
