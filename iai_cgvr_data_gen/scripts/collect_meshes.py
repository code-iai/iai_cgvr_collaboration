#!/usr/bin/env python
import rospy
import rospkg
import shutil
from urdf_parser_py.urdf import URDF, Mesh

def remove_transmission(urdf_string):
    fixed_urdf_string = ''
    delete = False
    black_list = ['<transmission']
    white_list = ['</transmission']
    for line in list(iter(urdf_string.splitlines())):
        if len([x for x in black_list if x in line]) > 0:
            if not delete:
                delete = True
        if not delete:
            fixed_urdf_string += line
        if len([x for x in white_list if x in line]) > 0:
            if delete:
                delete = False
    return fixed_urdf_string

def get_mesh_filenames(robot_model):
    mesh_filenames = []
    for link in robot_model.links:
        if link.visual is not None:
            if isinstance(link.visual.geometry, Mesh):
                mesh_filenames.append(link.visual.geometry.filename)
    return mesh_filenames

def collector():
    rospy.init_node('collector', anonymous=True)
    urdf_string = ""
    if rospy.has_param("/robot_description"):
        urdf_string += rospy.get_param("/robot_description")
    else:
        raise rospy.ROSException("Could not find robot description on parameter server")
    destination = ""
    if rospy.has_param("/mesh_destination"):
        destination += rospy.get_param("/mesh_destination")
    else:
        raise rospy.ROSException("Could not find '/mesh_destination' on parameter server")

    rospack = rospkg.RosPack()
    robot_model = URDF.from_xml_string(remove_transmission(urdf_string))
    for filename in get_mesh_filenames(robot_model):
        absolute_path = filename.replace("package://pr2_description", rospack.get_path("pr2_description")) # get rid of this hard-coded package name
        print "copying: '" + absolute_path + "' to '" + destination + "'"
        shutil.copy(absolute_path, destination)


if __name__ == '__main__':
    try:
        collector()
    except rospy.ROSInterruptException:
        pass