import re
import rospkg
import pinocchio as pin, numpy as np

def compare_configurations(model, q_1, q_2, threshold = 0.001):
    """
    Check wether two configurations are similar enough.

    Compare the distance, between the two configuration, to a threshold.

    Parameters
    ----------
        model (pin.model): The model of the robot.
        q_1 (float[]): A configuration.
        q_2 (float[]): Another configuration.

    Optionnals parameters:
    ----------------------
        threshold (foat): Error tolerance.

    Returns
    -------
        isEqual (bool): True if the two configurations are close enough.
    """
    return pin.distance(model, q_1, q_2) < threshold

def compare_poses(pose_1, pose_2, threshold = 0.001):
    """
    Check wether two poses are close enough.

    Compare the SE3 distance between the two poses.

    Parameters
    ----------
        pose_1 ([float[6 or 7]]): Position and orientation (euler or quaternion) of a pose.
        pose_2 ([float[6 or 7]): Position and orientation (euler or quaternion) of the other pose.

    Optionnals parameters:
    ----------------------
        threshold (foat): Error tolerance.

    Returns
    -------
        isEqual (bool): True if the two poses are close enough.
    """
    if(len(pose_1) == 6):
        pose_1 = pose_1[:3] + euler_to_quaternion(pose_1[-3:])

    if(len(pose_2) == 6):
        pose_2 = pose_2[:3] + euler_to_quaternion(pose_2[-3:])

    pose_1 = pin.XYZQUATToSE3(np.array(pose_1))
    pose_2 = pin.XYZQUATToSE3(np.array(pose_2))

    vw = pin.log6(pose_1.inverse() * pose_2)

    return np.linalg.norm([np.linalg.norm(vw.angular), np.linalg.norm(vw.linear)]) < threshold

def euler_to_quaternion(euler):
    """
    Convert euler angles to quaternion.

    Parameters
    ----------
        euler (float[3]): Euler angles.

    Returns
    -------
        quat (float[4]): Quaternion.
    """
    return list(pin.SE3ToXYZQUAT(pin.SE3(pin.rpy.rpyToMatrix(np.array(euler)), np.zeros(3))))[-4:]

def quaternion_to_euler(quat):
    """
    Convert a quaternion to euler angles.

    Parameters
    ----------
        quat (float[4]): The quaternion to convert.

    Returns
    -------
        euler (float[3]): The corresponding euler angles.
    """
    return list(pin.rpy.matrixToRpy(pin.XYZQUATToSE3(np.array([0]*3 + quat)).rotation))

def replace_path_to_absolute(str):
    """
    Replace ros package names with their actual absolute paths.

    Look for sub string in the format of 'package://{package_name}/'

    Parameters
    ----------
        str (str): A string that contains on or multiple ros package names.

    Returns
    -------
        str (str): The same string with the package name replace with their absolute path.
    """
    packages = set(re.findall("package:\/\/(.*?)\/", str))
    rospack = rospkg.RosPack()
    for pkg in packages:
        str = str.replace("package://"+pkg+"/", ""+rospack.get_path(pkg)+"/")
    return str

def replace_placeholders(str, placeHolder, replacement):
    """
    Replace all the occurences of a substring in a string.

    Parameters
    ----------
        str (str): The string to be modified.
        placeHolder (str): The substring to replace.
        replacement (str): The replacement for the substring.

    Returns
    -------
        str (str): The new string with the substrings replaced.
    """
    return str.replace(placeHolder, replacement)

def wd(o):
    """
    Wrappped corba object to ensure they are destroyed on the server side when not used anymore.

    Parameters
    ----------
        o (corba object): Object to wrap.

    Optionnals parameters:
    ----------------------
        threshold (foat): Error tolerance for each joint.

    Returns
    -------
        wo : the wrapped object.
    """
    from hpp.corbaserver import wrap_delete
    from hpp.corbaserver.manipulation import CorbaClient
    return wrap_delete(o, CorbaClient().basic._tools)
