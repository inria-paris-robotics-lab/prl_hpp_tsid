import re
import rospkg

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
    return str.replace(placeHolder, replacement)

def compare_configurations(q_1, q_2, threshold = 0.01):
    """
    Check wether two configurations are similar enough.

    Compare the absolute difference of the position to certain tolerance, for every joints.

    Parameters
    ----------
        q_1 (float[]): A configuration.
        q_2 (float[]): Another configuration.

    Optionnals parameters:
    ----------------------
        threshold (foat): Error tolerance for each joint.

    Returns
    -------
        isEqual (bool): True if the two configurations are close enough.

    Raises
    ------
        AssertionError: If configurations sizes are different.
    """
    assert len(q_1) == len(q_2), "Different configuration size: " + str(len(q_1)) + " != " + str(len(q_2))
    for i in range(len(q_1)):
        if(abs(q_1[i] - q_2[i]) > threshold):
            return False
    return True


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
