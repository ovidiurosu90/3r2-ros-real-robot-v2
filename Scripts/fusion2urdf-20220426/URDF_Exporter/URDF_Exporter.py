import adsk, adsk.core, adsk.fusion, traceback
import os
from .utils import utils
from .core import Link, Joint, Write

"""
# length unit is 'cm' and inertial unit is 'kg/cm^2'
# If there is no 'body' in the root component, maybe the corrdinates are wrong.
"""

# joint effort: 100
# joint velocity: 100
# supports "Revolute", "Rigid" and "Slider" joint types

# I'm not sure how prismatic joint acts if there is no limit in fusion model

def run(context):
    ui = None
    success_msg = "URDF exported successfully!\nPlease close the design without saving the changes!"
    msg = success_msg
    title = 'Fusion2URDF'

    try:
        # --------------------
        # initialize
        app = adsk.core.Application.get()
        ui = app.userInterface
        product = app.activeProduct
        design = adsk.fusion.Design.cast(product)

        if not design:
            ui.messageBox('No active Fusion design', title)
            return

        root = design.rootComponent  # root component

        # set the names
        robot_name = root.name.split()[0]
        package_name = robot_name + '_description'
        save_dir = utils.file_dialog(ui)
        if save_dir == False:
            ui.messageBox('Fusion2URDF was canceled', title)
            return 0

        save_dir = save_dir + '/' + package_name
        try: os.mkdir(save_dir)
        except: pass

        package_dir = os.path.abspath(os.path.dirname(__file__)) + '/package/'

        ui.messageBox("Generating the URDF. Please consider the following:\n\
            - the design should contain a component named base_link\n\
            - the main wheels should have good connection to the ground (casters should be a bit higher)\n\
            - the color of each component is taken from the first body's appearance\n\
            - the supported colors are: black, white, red, green, blue, yellow, orange\n\
            - the default color is grey\n\
            - invisible components generate empty links in the URDF files\n\
        \nPlease wait...", title)

        # --------------------
        # set dictionaries

        # Generate joints_dict. All joints are related to root.
        joints_dict, msg = Joint.make_joints_dict(root, msg)
        if msg != success_msg:
            ui.messageBox(msg, title)
            return 0

        components_properties_dict = utils.get_components_properties_dict(root)

        # Generate inertial_dict
        inertial_dict, msg = Link.make_inertial_dict(root, msg)
        if msg != success_msg:
            ui.messageBox(msg, title)
            return 0
        elif not 'base_link' in inertial_dict:
            msg = 'There is no base_link. Please set base_link and run again.'
            ui.messageBox(msg, title)
            return 0

        links_xyz_dict = {}

        # --------------------
        # Generate URDF
        Write.write_urdf_xacro(joints_dict, links_xyz_dict, inertial_dict, components_properties_dict, package_name, robot_name, save_dir)
        Write.write_materials_urdf_xacro(robot_name, save_dir)
        Write.write_materials_gazebo_xacro(joints_dict, components_properties_dict, robot_name, save_dir)
        Write.write_physics_gazebo_xacro(joints_dict, robot_name, save_dir)
        Write.write_plugins_gazebo_xacro(robot_name, save_dir)
        Write.write_transmissions_xacro(joints_dict, links_xyz_dict, robot_name, save_dir)
        Write.write_display_launch(package_name, robot_name, save_dir)
        Write.write_gazebo_launch(package_name, robot_name, save_dir)
        Write.write_control_launch(package_name, robot_name, save_dir, joints_dict)
        Write.write_yaml(robot_name, save_dir, joints_dict)

        # copy over package files
        utils.copy_package(save_dir, package_dir)
        utils.update_cmakelists(save_dir, package_name)
        utils.update_package_xml(save_dir, package_name)

        # Generate STL files
        ui.messageBox("Duplicating components under root that satisfy the following criteria:\n\
            - have bodies on the top level\n\
            - are visible\n\
        \nPlease wait...", title)
        utils.copy_occs(root)

        ui.messageBox("Exporting STLs for components under root that satisfy the following criteria:\n\
            - their name doesn't contain 'old_component'\n\
            - are visible\n\
        \nPlease wait...", title)
        utils.export_stl(design, save_dir, root)

        # Success Message
        ui.messageBox(msg, title)
        print('####### ALL DONE!')

    except:
        if ui:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))
