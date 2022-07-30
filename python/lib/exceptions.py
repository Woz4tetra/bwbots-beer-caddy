class SessionFinishedException(BaseException):
    """
    An exception to indicate the program has finished normally and to begin the shutdown sequence
    """

class ShutdownException(BaseException):
    """
    An exception to indicate the system should be shut down
    """
