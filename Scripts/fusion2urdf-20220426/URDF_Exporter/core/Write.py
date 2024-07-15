import adsk, os, re
from xml.etree.ElementTree import Element, SubElement, XML
from . import Link, Joint
from ..utils import utils

def write_link_urdf(joints_dict, repo, links_xyz_dict, components_properties_dict, file_name, inertial_dict):
    """
    Write links information into urdf "repo/file_name"

    Parameters
    ----------
    joints_dict: dict
        information of the each joint
    repo: str
        the name of the repository to save the xml file
    links_xyz_dict: vacant dict
        xyz information of the each link
    components_properties_dict: dict
        {component_name_1: {visibility: 0, color: 'red'}, component_name_2: {visibility: 1, color: 'black'}}
    file_name: str
        urdf full path
    inertial_dict:
        information of the each inertial

    Note
    ----------
    In this function, links_xyz_dict is set for write_joint_tran_urdf.
    The origin of the coordinate of center_of_mass is the coordinate of the link
    """
    with open(file_name, mode='a') as f:
        # for base_link
        center_of_mass = inertial_dict['base_link']['center_of_mass']
        link = Link.Link(name='base_link', xyz=[0,0,0],
            center_of_mass=center_of_mass, repo=repo,
            mass=inertial_dict['base_link']['mass'],
            inertia_tensor=inertial_dict['base_link']['inertia'])
        links_xyz_dict[link.name] = link.xyz
        link.make_link_xml(components_properties_dict)
        f.write(link.link_xml)
        f.write('\n')

        # others
        for joint in joints_dict:
            name = joints_dict[joint]['child']

            center_of_mass = \
                [ i-j for i, j in zip(inertial_dict[name]['center_of_mass'], joints_dict[joint]['xyz'])]
            link = Link.Link(name=name, xyz=joints_dict[joint]['xyz'],\
                center_of_mass=center_of_mass,\
                repo=repo, mass=inertial_dict[name]['mass'],\
                inertia_tensor=inertial_dict[name]['inertia'])
            links_xyz_dict[link.name] = link.xyz
            link.make_link_xml(components_properties_dict)
            if name in components_properties_dict and components_properties_dict[name]['visibility']:
                f.write(link.link_xml)
            else:
                print("INVISIBLE Component '%s' generates empty link" % name)
                f.write('<link name="%s" />\n' % name)
            f.write('\n')


def write_joint_urdf(joints_dict, links_xyz_dict, file_name):
    """
    Write joints and transmission information into urdf "repo/file_name"

    Parameters
    ----------
    joints_dict: dict
        information of the each joint
    links_xyz_dict: dict
        xyz information of the each link
    file_name: str
        urdf full path
    """

    with open(file_name, mode='a') as f:
        joint = Joint.Joint(name='base_footprint__base_link', joint_type = 'fixed',\
            parent = 'base_footprint', child = 'base_link',\
            xyz = None, rpy = None, axis = None, upper_limit = None, lower_limit = None)
        joint.make_joint_xml(hasOrigin = False)

        f.write(joint.joint_xml)
        f.write('\n')

        for j in joints_dict:
            parent = joints_dict[j]['parent']
            child = joints_dict[j]['child']
            joint_type = joints_dict[j]['type']
            upper_limit = joints_dict[j]['upper_limit']
            lower_limit = joints_dict[j]['lower_limit']
            rpy = joints_dict[j]['rpy']
            try:
                xyz = [round(p-c, 6) for p, c in \
                    zip(links_xyz_dict[parent], links_xyz_dict[child])]  # xyz = parent - child
            except KeyError as ke:
                app = adsk.core.Application.get()
                ui = app.userInterface
                ui.messageBox("There seems to be an error with the connection between\n\n%s\nand\n%s\n\nCheck \
whether the connections\nparent=component2=%s\nchild=component1=%s\nare correct or if you need \
to swap component1<=>component2"
                % (parent, child, parent, child), "Error!")
                quit()

            joint = Joint.Joint(name = j, joint_type = joint_type, xyz = xyz, rpy = rpy,\
            axis = joints_dict[j]['axis'], parent = parent, child = child, \
            upper_limit = upper_limit, lower_limit = lower_limit)
            joint.make_joint_xml()
            joint.make_transmission_xml()
            f.write(joint.joint_xml)
            f.write('\n')


def write_urdf_xacro(joints_dict, links_xyz_dict, inertial_dict, components_properties_dict, package_name, robot_name, save_dir):
    try: os.mkdir(save_dir + '/urdf')
    except: pass

    file_name = save_dir + '/urdf/' + robot_name + '.urdf.xacro'  # the name of urdf file
    repo = package_name + '/meshes/'  # the repository of binary stl files
    with open(file_name, mode='w') as f:
        f.write('<?xml version="1.0" ?>\n')
        f.write('<robot name="{}" xmlns:xacro="http://www.ros.org/wiki/xacro">\n'.format(robot_name))
        f.write('\n')
        f.write('<xacro:include filename="$(find {})/urdf/materials.urdf.xacro" />'.format(package_name))
        f.write('\n')
        f.write('<xacro:include filename="$(find {})/urdf/{}.urdf.trans" />'.format(package_name, robot_name))
        f.write('\n')
        f.write('\n')
        f.write('<xacro:include filename="$(find {})/urdf/materials.gazebo.xacro" />'.format(package_name))
        f.write('\n')
        f.write('<xacro:include filename="$(find {})/urdf/physics.gazebo.xacro" />'.format(package_name))
        f.write('\n')
        f.write('<xacro:include filename="$(find {})/urdf/plugins.gazebo.xacro" />'.format(package_name))
        f.write('\n')
        f.write('\n')
        f.write('<!-- Added this to get rid of the following warning in gazebo:')
        f.write('\n')
        f.write('The root link base_link has an inertia specified in the URDF, but KDL does not support a root link with an inertia. -->')
        f.write('\n')
        f.write('<link name="base_footprint" />')
        f.write('\n')
        f.write('\n')

    write_link_urdf(joints_dict, repo, links_xyz_dict, components_properties_dict, file_name, inertial_dict)
    write_joint_urdf(joints_dict, links_xyz_dict, file_name)

    with open(file_name, mode='a') as f:
        f.write('</robot>\n\n')


def write_materials_urdf_xacro(robot_name, save_dir):
    try: os.mkdir(save_dir + '/urdf')
    except: pass

    file_name = save_dir + '/urdf/materials.urdf.xacro'
    with open(file_name, mode='w') as f:
        f.write('<?xml version="1.0" ?>\n')
        f.write('<robot name="{}" xmlns:xacro="http://www.ros.org/wiki/xacro">\n'.format(robot_name))
        f.write('\n')
        f.write('<material name="grey">\n')
        f.write('    <color rgba="0.7 0.7 0.7 1.0" />\n')
        f.write('</material>\n')
        f.write('\n')
        f.write('<material name="black">\n')
        f.write('    <color rgba="0.0 0.0 0.0 1.0" />\n')
        f.write('</material>\n')
        f.write('\n')
        f.write('<material name="white">\n')
        f.write('    <color rgba="1.0 1.0 1.0 1.0" />\n')
        f.write('</material>\n')
        f.write('\n')
        f.write('<material name="red">\n')
        f.write('    <color rgba="0.8 0.0 0.0 1.0" />\n')
        f.write('</material>\n')
        f.write('\n')
        f.write('<material name="green">\n')
        f.write('    <color rgba="0.0 0.8 0.0 1.0" />\n')
        f.write('</material>\n')
        f.write('\n')
        f.write('<material name="blue">\n')
        f.write('    <color rgba="0.0 0.0 0.8 1.0" />\n')
        f.write('</material>\n')
        f.write('\n')
        f.write('<material name="yellow">\n')
        f.write('    <color rgba="0.8 0.8 0.0 1.0" />\n')
        f.write('</material>\n')
        f.write('\n')
        f.write('<material name="orange">\n')
        f.write('    <color rgba="${255/255} ${108/255} ${10/255} 1.0" />\n')
        f.write('</material>\n')
        f.write('\n')
        f.write('</robot>\n')


def write_materials_gazebo_xacro(joints_dict, components_properties_dict, robot_name, save_dir):
    try: os.mkdir(save_dir + '/urdf')
    except: pass

    file_name = save_dir + '/urdf/materials.gazebo.xacro'
    with open(file_name, mode='w') as f:
        f.write('<?xml version="1.0" ?>\n')
        f.write('<robot name="{}" xmlns:xacro="http://www.ros.org/wiki/xacro">\n'.format(robot_name))
        f.write('\n')
        f.write('<xacro:property name="color_grey" value="Gazebo/Grey" />\n')
        f.write('<xacro:property name="color_black" value="Gazebo/Black" />\n')
        f.write('<xacro:property name="color_white" value="Gazebo/White" />\n')
        f.write('<xacro:property name="color_red" value="Gazebo/Red" />\n')
        f.write('<xacro:property name="color_green" value="Gazebo/Green" />\n')
        f.write('<xacro:property name="color_blue" value="Gazebo/Blue" />\n')
        f.write('<xacro:property name="color_yellow" value="Gazebo/Yellow" />\n')
        f.write('<xacro:property name="color_orange" value="Gazebo/Orange" />\n')
        f.write('\n')

        # for base_link
        f.write('<gazebo reference="base_link">\n')
        f.write('    <material>${color_%s}</material>\n' % utils.get_color_by_component_name('base_link', components_properties_dict))
        f.write('</gazebo>\n')
        f.write('\n')

        # others
        for joint in joints_dict:
            name = joints_dict[joint]['child']
            f.write('<gazebo reference="{}">\n'.format(name))
            f.write('    <material>${color_%s}</material>\n' % utils.get_color_by_component_name(name, components_properties_dict))
            f.write('</gazebo>\n')
            f.write('\n')

        f.write('</robot>\n')


def write_physics_gazebo_xacro(joints_dict, robot_name, save_dir):
    try: os.mkdir(save_dir + '/urdf')
    except: pass

    file_name = save_dir + '/urdf/physics.gazebo.xacro'

    with open(file_name, mode='w') as f:
        f.write('<?xml version="1.0" ?>\n')
        f.write('<robot name="{}" xmlns:xacro="http://www.ros.org/wiki/xacro">\n'.format(robot_name))
        f.write('\n')
        f.write('<xacro:property name="standard_friction" value="1.0" />\n')
        f.write('<xacro:property name="caster_friction" value="0.01" />\n')
        f.write('<xacro:property name="wheel_friction" value="100.0" />\n')
        f.write('\n')

        # for base_link
        f.write('<gazebo reference="base_link">\n')
        f.write('    <mu1>${standard_friction}</mu1>\n')
        f.write('    <mu2>${standard_friction}</mu2>\n')
        f.write('    <selfCollide>true</selfCollide>\n')
        f.write('    <gravity>true</gravity>\n')
        f.write('</gazebo>\n')
        f.write('\n')

        # others
        for joint in joints_dict:
            name = joints_dict[joint]['child']
            friction = '${standard_friction}'
            if 'wheel' in name:
                friction = '${wheel_friction}'
            elif 'caster' in name:
                friction = '${caster_friction}'

            f.write('<gazebo reference="{}">\n'.format(name))
            f.write('    <mu1>%s</mu1>\n' % friction)
            f.write('    <mu2>%s</mu2>\n' % friction)
            f.write('    <selfCollide>true</selfCollide>\n')
            f.write('</gazebo>\n')
            f.write('\n')

        f.write('</robot>\n')


def write_plugins_gazebo_xacro(robot_name, save_dir):
    try: os.mkdir(save_dir + '/urdf')
    except: pass

    file_name = save_dir + '/urdf/plugins.gazebo.xacro'

    with open(file_name, mode='w') as f:
        f.write('<?xml version="1.0" ?>\n')
        f.write('<robot name="{}" xmlns:xacro="http://www.ros.org/wiki/xacro">\n'.format(robot_name))
        f.write('\n')

        gazebo = Element('gazebo')
        plugin = SubElement(gazebo, 'plugin')
        plugin.attrib = {'name':'control', 'filename':'libgazebo_ros_control.so'}
        gazebo_xml = "\n".join(utils.prettify(gazebo).split("\n")[1:])
        f.write(gazebo_xml)
        f.write('\n')

        gazebo = Element('gazebo')
        plugin = SubElement(gazebo, 'plugin')
        plugin.attrib = {'name': 'differential_drive_controller', 'filename': 'libgazebo_ros_diff_drive.so'}
        updateRate = SubElement(plugin, 'updateRate')
        updateRate.text = '50'
        leftJoint = SubElement(plugin, 'leftJoint')
        leftJoint.text = '[TOFILL_differential_drive_controller_leftJoint]'
        rightJoint = SubElement(plugin, 'rightJoint')
        rightJoint.text = '[TOFILL_differential_drive_controller_rightJoint]'
        wheelSeparation = SubElement(plugin, 'wheelSeparation')
        wheelSeparation.text = '[TOFILL_differential_drive_controller_wheelSeparation]'
        wheelDiameter = SubElement(plugin, 'wheelDiameter')
        wheelDiameter.text = '[TOFILL_differential_drive_controller_wheelDiameter]'
        wheelAcceleration = SubElement(plugin, 'wheelAcceleration')
        wheelAcceleration.text = '[TOFILL_differential_drive_controller_wheelAcceleration]'
        wheelTorque = SubElement(plugin, 'wheelTorque')
        wheelTorque.text = '[TOFILL_differential_drive_controller_wheelTorque]'
        commandTopic = SubElement(plugin, 'commandTopic')
        commandTopic.text = 'cmd_vel'
        odometryTopic = SubElement(plugin, 'odometryTopic')
        odometryTopic.text = 'odom'
        odometryFrame = SubElement(plugin, 'odometryFrame')
        odometryFrame.text = 'odom'
        robotBaseFrame = SubElement(plugin, 'robotBaseFrame')
        robotBaseFrame.text = 'base_footprint'
        odometrySource = SubElement(plugin, 'odometrySource')
        odometrySource.text = 'world'
        publishWheelTF = SubElement(plugin, 'publishWheelTF')
        publishWheelTF.text = 'false'
        publishOdom = SubElement(plugin, 'publishOdom')
        publishOdom.text = 'true'
        publishWheelJointState = SubElement(plugin, 'publishWheelJointState')
        publishWheelJointState.text = 'true'
        legacyMode = SubElement(plugin, 'legacyMode')
        legacyMode.text = 'false'
        rosDebugLevel = SubElement(plugin, 'rosDebugLevel')
        rosDebugLevel.text = 'na'
        publishOdomTF = SubElement(plugin, 'publishOdomTF')
        publishOdomTF.text = 'true'
        publishTf = SubElement(plugin, 'publishTf')
        publishTf.text = '1'
        gazebo_xml = "\n".join(utils.prettify(gazebo).split("\n")[1:])
        f.write(gazebo_xml)
        f.write('\n')

        gazebo = Element('gazebo')
        gazebo.attrib = {'reference': '[TOFILL_base_scan_reference]'}
        sensor = SubElement(gazebo, 'sensor')
        sensor.attrib = {'type': 'gpu_ray', 'name': 'rplidar_a2'}
        pose = SubElement(sensor, 'pose')
        pose.text = '0 0 0 0 0 0'
        visualize = SubElement(sensor, 'visualize')
        visualize.text = 'true'
        update_rate = SubElement(sensor, 'update_rate')
        update_rate.text = '10'
        ray = SubElement(sensor, 'ray')
        ray.extend(XML(re.sub('>\s*<', '><', '''
        <ray>
            <scan>
                <horizontal>
                    <samples>720</samples>
                    <resolution>1</resolution>
                    <min_angle>-3.14159</min_angle>
                    <max_angle>3.14159</max_angle>
                </horizontal>
            </scan>
            <range>
                <min>0.15</min>
                <max>8.0</max>
                <resolution>0.01</resolution>
            </range>
            <noise>
                <type>gaussian</type>
                <!-- Noise parameters based on published spec for Hokuyo laser
                    achieving "+-30mm" accuracy at range < 10m. A mean of 0.0m and
                    stddev of 0.01m will put 99.7 percent of samples within 0.03m of the true
                    reading. -->
                <mean>0.0</mean>
                <stddev>0.01</stddev>
            </noise>
        </ray>
        ''')))
        plugin = SubElement(sensor, 'plugin')
        plugin.attrib = {'name': 'gazebo_ros_base_scan_controller', 'filename': 'libgazebo_ros_gpu_laser.so'}
        topicName = SubElement(plugin, 'topicName')
        topicName.text = 'scan'
        frameName = SubElement(plugin, 'frameName')
        frameName.text = 'base_scan'
        gazebo_xml = "\n".join(utils.prettify(gazebo).split("\n")[1:])
        f.write(gazebo_xml)
        f.write('\n')

        gazebo = Element('gazebo')
        gazebo.attrib = {'reference': '[TOFILL_camera_front_reference]'}
        sensor = SubElement(gazebo, 'sensor')
        sensor.attrib = {'type': 'camera', 'name': 'camera_front'}
        always_on = SubElement(sensor, 'always_on')
        always_on.text = 'true'
        visualize = SubElement(sensor, 'visualize')
        visualize.text = 'true'
        camera = SubElement(sensor, 'camera')
        camera.extend(XML(re.sub('>\s*<', '><', '''
        <camera>
            <horizontal_fov>1.085595</horizontal_fov>
            <image>
                <width>640</width>
                <height>480</height>
                <format>R8G8B8</format>
            </image>
            <clip>
                <near>0.03</near>
                <far>100</far>
            </clip>
        </camera>
        ''')))
        plugin = SubElement(sensor, 'plugin')
        plugin.attrib = {'name': 'camera_controller', 'filename': 'libgazebo_ros_camera.so'}
        alwaysOn = SubElement(plugin, 'alwaysOn')
        alwaysOn.text = 'true'
        updateRate = SubElement(plugin, 'updateRate')
        updateRate.text = '30'
        cameraName = SubElement(plugin, 'cameraName')
        cameraName.text = 'camera_front'
        frameName = SubElement(plugin, 'frameName')
        frameName.text = 'camera_front_frame'
        imageTopicName = SubElement(plugin, 'imageTopicName')
        imageTopicName.text = 'image_raw'
        cameraInfoTopicName = SubElement(plugin, 'cameraInfoTopicName')
        cameraInfoTopicName.text = 'camera_info'
        hackBaseline = SubElement(plugin, 'hackBaseline')
        hackBaseline.text = '0.07'
        distortionK1 = SubElement(plugin, 'distortionK1')
        distortionK1.text = '0.0'
        distortionK2 = SubElement(plugin, 'distortionK2')
        distortionK2.text = '0.0'
        distortionK3 = SubElement(plugin, 'distortionK3')
        distortionK3.text = '0.0'
        distortionT1 = SubElement(plugin, 'distortionT1')
        distortionT1.text = '0.0'
        distortionT2 = SubElement(plugin, 'distortionT2')
        distortionT2.text = '0.0'
        gazebo_xml = "\n".join(utils.prettify(gazebo).split("\n")[1:])
        f.write(gazebo_xml)
        f.write('\n')

        gazebo = Element('gazebo')
        gazebo.attrib = {'reference': '[TOFILL_camera_rear_reference]'}
        sensor = SubElement(gazebo, 'sensor')
        sensor.attrib = {'type': 'camera', 'name': 'camera_rear'}
        always_on = SubElement(sensor, 'always_on')
        always_on.text = 'true'
        visualize = SubElement(sensor, 'visualize')
        visualize.text = 'true'
        camera = SubElement(sensor, 'camera')
        camera.extend(XML(re.sub('>\s*<', '><', '''
        <camera>
            <horizontal_fov>1.085595</horizontal_fov>
            <image>
                <width>640</width>
                <height>480</height>
                <format>R8G8B8</format>
            </image>
            <clip>
                <near>0.03</near>
                <far>100</far>
            </clip>
        </camera>
        ''')))
        plugin = SubElement(sensor, 'plugin')
        plugin.attrib = {'name': 'camera_controller', 'filename': 'libgazebo_ros_camera.so'}
        alwaysOn = SubElement(plugin, 'alwaysOn')
        alwaysOn.text = 'true'
        updateRate = SubElement(plugin, 'updateRate')
        updateRate.text = '30'
        cameraName = SubElement(plugin, 'cameraName')
        cameraName.text = 'camera_rear'
        frameName = SubElement(plugin, 'frameName')
        frameName.text = 'camera_rear_frame'
        imageTopicName = SubElement(plugin, 'imageTopicName')
        imageTopicName.text = 'image_raw'
        cameraInfoTopicName = SubElement(plugin, 'cameraInfoTopicName')
        cameraInfoTopicName.text = 'camera_info'
        hackBaseline = SubElement(plugin, 'hackBaseline')
        hackBaseline.text = '0.07'
        distortionK1 = SubElement(plugin, 'distortionK1')
        distortionK1.text = '0.0'
        distortionK2 = SubElement(plugin, 'distortionK2')
        distortionK2.text = '0.0'
        distortionK3 = SubElement(plugin, 'distortionK3')
        distortionK3.text = '0.0'
        distortionT1 = SubElement(plugin, 'distortionT1')
        distortionT1.text = '0.0'
        distortionT2 = SubElement(plugin, 'distortionT2')
        distortionT2.text = '0.0'
        gazebo_xml = "\n".join(utils.prettify(gazebo).split("\n")[1:])
        f.write(gazebo_xml)
        f.write('\n')

        gazebo = Element('gazebo')
        gazebo.attrib = {'reference': '[TOFILL_imu_link_reference]'}
        gravity = SubElement(gazebo, 'gravity')
        gravity.text = 'true'
        sensor = SubElement(gazebo, 'sensor')
        sensor.attrib = {'type': 'imu', 'name': 'imu_sensor'}
        always_on = SubElement(sensor, 'always_on')
        always_on.text = 'true'
        visualize = SubElement(sensor, 'visualize')
        visualize.text = 'false'
        update_rate = SubElement(sensor, 'update_rate')
        update_rate.text = '50'
        topic = SubElement(sensor, 'topic')
        topic.text = '__default_topic__'
        plugin = SubElement(sensor, 'plugin')
        plugin.attrib = {'name': 'imu_plugin', 'filename': 'libgazebo_ros_imu_sensor.so'}
        topicName = SubElement(plugin, 'topicName')
        topicName.text = '/imu/data'
        bodyName = SubElement(plugin, 'bodyName')
        bodyName.text = 'imu_link'
        updateRateHZ = SubElement(plugin, 'updateRateHZ')
        updateRateHZ.text = '50.0'
        gaussianNoise = SubElement(plugin, 'gaussianNoise')
        gaussianNoise.text = '0.0'
        xyzOffset = SubElement(plugin, 'xyzOffset')
        xyzOffset.text = '0 0 0'
        rpyOffset = SubElement(plugin, 'rpyOffset')
        rpyOffset.text = '0 0 0'
        frameName = SubElement(plugin, 'frameName')
        frameName.text = 'imu_link'
        initialOrientationAsReference = SubElement(plugin, 'initialOrientationAsReference')
        initialOrientationAsReference.text = 'false'
        pose = SubElement(sensor, 'pose')
        pose.text = '0 0 0 0 0 0'
        gazebo_xml = "\n".join(utils.prettify(gazebo).split("\n")[1:])
        f.write(gazebo_xml)
        f.write('\n')

        f.write('</robot>\n')


def write_transmissions_xacro(joints_dict, links_xyz_dict, robot_name, save_dir):
    """
    Write joints and transmission information into urdf "repo/file_name"

    Parameters
    ----------
    joints_dict: dict
        information of the each joint
    links_xyz_dict: dict
        xyz information of the each link
    robot_name: str
        name of the robot
    save_dir: str
        path of the repository to save
    """

    file_name = save_dir + '/urdf/{}.urdf.trans'.format(robot_name)  # the name of urdf file
    with open(file_name, mode='w') as f:
        f.write('<?xml version="1.0" ?>\n')
        f.write('<robot name="{}" xmlns:xacro="http://www.ros.org/wiki/xacro">\n'.format(robot_name))
        f.write('\n')

        for j in joints_dict:
            parent = joints_dict[j]['parent']
            child = joints_dict[j]['child']
            joint_type = joints_dict[j]['type']
            upper_limit = joints_dict[j]['upper_limit']
            lower_limit = joints_dict[j]['lower_limit']
            try:
                xyz = [round(p-c, 6) for p, c in \
                    zip(links_xyz_dict[parent], links_xyz_dict[child])]  # xyz = parent - child
            except KeyError as ke:
                app = adsk.core.Application.get()
                ui = app.userInterface
                ui.messageBox("There seems to be an error with the connection between\n\n%s\nand\n%s\n\nCheck \
whether the connections\nparent=component2=%s\nchild=component1=%s\nare correct or if you need \
to swap component1<=>component2"
                % (parent, child, parent, child), "Error!")
                quit()

            joint = Joint.Joint(name = j, joint_type = joint_type, xyz = xyz, rpy = None,\
            axis = joints_dict[j]['axis'], parent = parent, child = child, \
            upper_limit = upper_limit, lower_limit = lower_limit)
            if joint_type != 'fixed':
                joint.make_transmission_xml()
                f.write(joint.tran_xml)
                f.write('\n')

        f.write('</robot>\n')


def write_display_launch(package_name, robot_name, save_dir):
    """
    write display launch file "save_dir/launch/display.launch"

    Parameter
    ---------
    robot_name: str
        name of the robot
    save_dir: str
        path of the repository to save
    """
    try: os.mkdir(save_dir + '/launch')
    except: pass

    launch = Element('launch')

    arg1 = SubElement(launch, 'arg')
    arg1.attrib = {'name':'model', 'default':'$(find {})/urdf/{}.urdf.xacro'.format(package_name, robot_name)}

    arg3 = SubElement(launch, 'arg')
    arg3.attrib = {'name':'rvizconfig', 'default':'$(find {})/rviz/display.rviz'.format(package_name)}

    param1 = SubElement(launch, 'param')
    param1.attrib = {'name':'robot_description', 'command':'$(find xacro)/xacro $(arg model)'}

    node1 = SubElement(launch, 'node')
    node1.attrib = {'name':'joint_state_publisher_gui', 'pkg':'joint_state_publisher_gui', 'type':'joint_state_publisher_gui'}

    node2 = SubElement(launch, 'node')
    node2.attrib = {'name':'robot_state_publisher', 'pkg':'robot_state_publisher', 'type':'robot_state_publisher'}

    node3 = SubElement(launch, 'node')
    node3.attrib = {'name':'rviz', 'pkg':'rviz', 'args':'-d $(arg rvizconfig)', 'type':'rviz', 'required':'true'}

    launch_xml = "\n".join(utils.prettify(launch).split("\n")[1:])

    file_name = save_dir + '/launch/display.launch'
    with open(file_name, mode='w') as f:
        f.write('<?xml version="1.0" ?>\n')
        f.write(launch_xml)


def write_gazebo_launch(package_name, robot_name, save_dir):
    """
    write gazebo launch file "save_dir/launch/gazebo.launch"

    Parameter
    ---------
    robot_name: str
        name of the robot
    save_dir: str
        path of the repository to save
    """

    try: os.mkdir(save_dir + '/launch')
    except: pass

    launch = Element('launch')
    param = SubElement(launch, 'param')
    param.attrib = {'name':'robot_description', 'command':'$(find xacro)/xacro $(find {})/urdf/{}.urdf.xacro'.format(package_name, robot_name)}

    node = SubElement(launch, 'node')
    node.attrib = {'name':'spawn_urdf', 'pkg':'gazebo_ros', 'type':'spawn_model',\
                    'args':'-param robot_description -urdf -model {}'.format(robot_name)}

    include_ =  SubElement(launch, 'include')
    include_.attrib = {'file':'$(find gazebo_ros)/launch/empty_world.launch'}

    number_of_args = 5
    args = [None for i in range(number_of_args)]
    args_name_value_pairs = [['paused', 'false'], ['use_sim_time', 'true'],
                             ['gui', 'true'], ['headless', 'false'],
                             ['debug', 'false']]

    for i, arg in enumerate(args):
        arg = SubElement(include_, 'arg')
        arg.attrib = {'name' : args_name_value_pairs[i][0] ,
        'value' : args_name_value_pairs[i][1]}

    launch_xml = "\n".join(utils.prettify(launch).split("\n")[1:])

    file_name = save_dir + '/launch/' + 'gazebo.launch'
    with open(file_name, mode='w') as f:
        f.write('<?xml version="1.0" ?>\n')
        f.write(launch_xml)


def write_control_launch(package_name, robot_name, save_dir, joints_dict):
    """
    write control launch file "save_dir/launch/controller.launch"

    Parameter
    ---------
    robot_name: str
        name of the robot
    save_dir: str
        path of the repository to save
    joints_dict: dict
        information of the joints
    """

    try: os.mkdir(save_dir + '/launch')
    except: pass

    controller_args_str = ""
    for j in joints_dict:
        joint_type = joints_dict[j]['type']
        if joint_type != 'fixed':
            controller_args_str += j + '_position_controller '
    controller_args_str += 'joint_state_controller'

    node_controller = Element('node')
    node_controller.attrib = {'name':'controller_spawner', 'pkg':'controller_manager', 'type':'spawner',\
                    'respawn':'false', 'output':'screen', 'ns':robot_name,\
                    'args':'{}'.format(controller_args_str)}

    node_publisher = Element('node')
    node_publisher.attrib = {'name':'robot_state_publisher', 'pkg':'robot_state_publisher',\
                    'type':'robot_state_publisher', 'respawn':'false', 'output':'screen'}
    remap = SubElement(node_publisher, 'remap')
    remap.attrib = {'from':'/joint_states',\
                    'to':'/' + robot_name + '/joint_states'}

    #launch_xml  = "\n".join(utils.prettify(launch).split("\n")[1:])
    launch_xml  = "\n".join(utils.prettify(node_controller).split("\n")[1:])
    launch_xml += "\n".join(utils.prettify(node_publisher).split("\n")[1:])

    file_name = save_dir + '/launch/controller.launch'
    with open(file_name, mode='w') as f:
        f.write('<?xml version="1.0" ?>\n')
        f.write('<launch>\n')
        f.write('<!-- NOTE Not used yet -->\n')
        #for some reason ROS is very picky about the attribute ordering, so we'll bitbang this element
        f.write('<rosparam file="$(find {})/launch/controller.yaml" command="load" />'.format(package_name))
        f.write('\n')
        f.write(launch_xml)
        f.write('\n')
        f.write('</launch>\n')


def write_yaml(robot_name, save_dir, joints_dict):
    """
    write yaml file "save_dir/launch/controller.yaml"

    Parameter
    ---------
    robot_name: str
        name of the robot
    save_dir: str
        path of the repository to save
    joints_dict: dict
        information of the joints
    """
    try: os.mkdir(save_dir + '/launch')
    except: pass

    controller_name = robot_name + '_controller'
    file_name = save_dir + '/launch/controller.yaml'
    with open(file_name, 'w') as f:
        f.write(controller_name + ':\n')
        # joint_state_controller
        f.write('    # Publish all joint states -----------------------------------\n')
        f.write('    joint_state_controller:\n')
        f.write('        type: joint_state_controller/JointStateController\n')
        f.write('        publish_rate: 50\n\n')
        # position_controllers
        f.write('    # Position Controllers --------------------------------------\n')
        for joint in joints_dict:
            joint_type = joints_dict[joint]['type']
            if joint_type != 'fixed':
                f.write('    ' + joint + '_position_controller:\n')
                f.write('        type: effort_controllers/JointPositionController\n')
                f.write('        joint: '+ joint + '\n')
                f.write('        pid: {p: 100.0, i: 0.01, d: 10.0}\n')
