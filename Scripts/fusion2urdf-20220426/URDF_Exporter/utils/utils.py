import adsk, adsk.core, adsk.fusion
import os.path, re
from xml.etree import ElementTree
from xml.dom import minidom
from distutils.dir_util import copy_tree
import fileinput
import sys

def get_components_properties_dict(root):
    """
    Get a dictionary holding component_name and it's properties (visibility, color)

    Parameters
    ----------
    root: adsk.fusion.Design.cast(product)
        Root component

    Returns
    ----------
    dict:
        {component_name_1: {visibility: 0, color: 'red'}, component_name_2: {visibility: 1, color: 'black'}}
    """
    components_properties_dict = {}

    occurences = root.occurrences
    for occurence in occurences:
        if 'old_component' in occurence.component.name:
            print("# SKIPPING Component '%s' because its name contains 'old_component'" % occurence.component.name)
            continue

        component_name = re.sub('[ :()]', '_', occurence.name)
        if component_name not in components_properties_dict:
            components_properties_dict[component_name] = {}

        components_properties_dict[component_name]['visibility'] = occurence.isVisible

        appearance = None
        if occurence.bRepBodies.count <= 0:
            print("# Missing appearance for Component '%s' as it doesn't have a body on the top level" % component_name)
        else:
            body = occurence.bRepBodies.item(0)

            if body.appearance:
                appearance = body.appearance.name
                print("Appearance for Component '%s' is set to '%s'" %(component_name, appearance))
            else:
                print("# Missing appearance for Component '%s' as it doesn't have a body on the top level" % component_name)

        components_properties_dict[component_name]['appearance'] = appearance
        color = 'grey'
        if appearance:
            if 'black' in appearance.lower():
                color = 'black'
            elif 'white' in appearance.lower():
                color = 'white'
            elif 'red' in appearance.lower():
                color = 'red'
            elif 'green' in appearance.lower():
                color = 'green'
            elif 'blue' in appearance.lower():
                color = 'blue'
            elif 'yellow' in appearance.lower():
                color = 'yellow'
            elif 'orange' in appearance.lower():
                color = 'orange'
        components_properties_dict[component_name]['color'] = color

    return components_properties_dict


def get_color_by_component_name(component_name, components_properties_dict):
    if component_name in components_properties_dict and components_properties_dict[component_name]['color']:
        return components_properties_dict[component_name]['color']

    if re.search(r'\d+$', component_name) is None: # doesn't end with number (e.g. base_link)
        return get_color_by_component_name(component_name + '_1', components_properties_dict)

    return 'grey'


def copy_occs(root):
    """
    duplicate all the components
    """
    def copy_body(allOccs, occs):
        """
        copy the old occs to new component
        """

        bodies = occs.bRepBodies
        transform = adsk.core.Matrix3D.create()

        # Create new components from occs
        # This support even when a component has some occses.

        new_occs = allOccs.addNewComponent(transform)  # this create new occs
        if occs.component.name == 'base_link':
            occs.component.name = 'old_component'
            new_occs.component.name = 'base_link'
        else:
            new_occs.component.name = re.sub('[ :()]', '_', occs.name)
        new_occs = allOccs.item((allOccs.count-1))
        for i in range(bodies.count):
            body = bodies.item(i)
            body.copyToComponent(new_occs)

    allOccs = root.occurrences
    oldOccs = []
    coppy_list = [occs for occs in allOccs]
    for occs in coppy_list:
        if occs.bRepBodies.count <= 0:
            #NOTE '3R2 ROS Real Robot v2 v15:1' is skipped here as count is 0
            print("# SKIPPING Component '%s' because it doesn't have bodies on the top level" % occs.name)
            continue

        if not occs.isVisible:
            #NOTE 'camera_front_lens:1' is not visible so it should be skipped
            print("# SKIPPING Component '%s' because it is not visible" % occs.name)
            continue

        print("Duplicating Component '%s'..." % occs.name)
        copy_body(allOccs, occs)
        oldOccs.append(occs)

    for occs in oldOccs:
        occs.component.name = 'old_component'


def export_stl(design, save_dir, root):
    """
    export stl files into "sace_dir/"

    Parameters
    ----------
    design: adsk.fusion.Design.cast(product)
    save_dir: str
        directory path to save
    root: design.rootComponent
    """

    # create a single exportManager instance
    exportMgr = design.exportManager
    # get the script location
    try: os.mkdir(save_dir + '/meshes')
    except: pass

    scriptDir = save_dir + '/meshes'

    allOccs = root.occurrences
    for occ in allOccs:
        if 'old_component' in occ.component.name:
            print("# SKIPPING Component '%s' because its name contains 'old_component'" % occ.component.name)
            continue

        if not occ.isVisible:
            #NOTE 'camera_front_lens:1' is not visible so it should be skipped
            print("# SKIPPING Component '%s' because it is not visible" % occ.name)
            continue

        try:
            print("Exporting Component '%s'..." % occ.component.name)
            fileName = scriptDir + "/" + occ.component.name
            # create stl exportOptions
            stlExportOptions = exportMgr.createSTLExportOptions(occ, fileName)
            stlExportOptions.sendToPrintUtility = False
            stlExportOptions.isBinaryFormat = True
            # options are .MeshRefinementLow .MeshRefinementMedium .MeshRefinementHigh
            stlExportOptions.meshRefinement = adsk.fusion.MeshRefinementSettings.MeshRefinementLow
            exportMgr.execute(stlExportOptions)
        except Exception as e:
            print("# ERROR Component '%s': %s! " %(occ.component.name, e))


def file_dialog(ui):
    """
    display the dialog to save the file
    """
    # Set styles of folder dialog.
    folderDlg = ui.createFolderDialog()
    folderDlg.title = 'Fusion Folder Dialog'

    # Show folder dialog
    dlgResult = folderDlg.showDialog()
    if dlgResult == adsk.core.DialogResults.DialogOK:
        return folderDlg.folder
    return False


def origin2center_of_mass(inertia, center_of_mass, mass):
    """
    convert the moment of the inertia about the world coordinate into
    that about center of mass coordinate

    Parameters
    ----------
    moment of inertia about the world coordinate:  [xx, yy, zz, xy, yz, xz]
    center_of_mass: [x, y, z]

    Returns
    ----------
    moment of inertia about center of mass : [xx, yy, zz, xy, yz, xz]
    """
    x = center_of_mass[0]
    y = center_of_mass[1]
    z = center_of_mass[2]
    translation_matrix = [y**2+z**2, x**2+z**2, x**2+y**2,
                         -x*y, -y*z, -x*z]
    return [ round(i - mass*t, 6) for i, t in zip(inertia, translation_matrix)]


def prettify(elem):
    """
    Return a pretty-printed XML string for the Element.
    Parameters
    ----------
    elem : xml.etree.ElementTree.Element

    Returns
    ----------
    pretified xml : str
    """
    rough_string = ElementTree.tostring(elem, 'utf-8')
    reparsed = minidom.parseString(rough_string)
    xmlString = reparsed.toprettyxml(indent="    ")
    return xmlString.replace('"/>', '" />')


def copy_package(save_dir, package_dir):
    try: os.mkdir(save_dir + '/launch')
    except: pass
    try: os.mkdir(save_dir + '/urdf')
    except: pass
    try: os.mkdir(save_dir + '/rviz')
    except: pass
    copy_tree(package_dir, save_dir)


def update_cmakelists(save_dir, package_name):
    file_name = save_dir + '/CMakeLists.txt'

    for line in fileinput.input(file_name, inplace=True):
        if 'project(fusion2urdf)' in line:
            sys.stdout.write("project(" + package_name + ")\n")
        else:
            sys.stdout.write(line)

def update_package_xml(save_dir, package_name):
    file_name = save_dir + '/package.xml'

    for line in fileinput.input(file_name, inplace=True):
        if '<name>' in line:
            sys.stdout.write("    <name>" + package_name + "</name>\n")
        elif '<description>' in line:
            sys.stdout.write("    <description>The " + package_name + " package</description>\n")
        else:
            sys.stdout.write(line)
