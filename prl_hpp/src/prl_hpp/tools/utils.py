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