"""Module containing several custom exceptions."""


class EePoseLookupError(Exception):
    """Custom exception that is raised when an error occurred while trying to retrieve
    the EE pose.

    Attributes:
        log_message (str): The full log message.
        details (dict): Dictionary containing extra Exception information.
    """

    def __init__(self, message="", log_message="", **details):
        """Initializes the EePoseLookupError exception object.

        Args:
            message (str, optional): Exception message specifying whether the exception
                occurred. Defaults to ``""``.
            log_message (str, optional): Full log message. Defaults to ``""``.
            details (dict): Additional dictionary that can be used to supply the user
                with more details about why the exception occurred.
        """
        super().__init__(message)

        self.log_message = log_message
        self.details = details


class EeRpyLookupError(Exception):
    """Custom exception that is raised when an error occurred while trying to retrieve
    the EE orientation (given in euler angles).

    Attributes:
        log_message (str): The full log message.
        details (dict): Dictionary containing extra Exception information.
    """

    def __init__(self, message="", log_message="", **details):
        """Initializes the EeRpyLookupError exception object.

        Args:
            message (str, optional): Exception message specifying whether the exception
                occurred. Defaults to ``""``.
            log_message (str, optional): Full log message. Defaults to ``""``.
            details (dict): Additional dictionary that can be used to supply the user
                with more details about why the exception occurred.
        """
        super().__init__(message)

        self.log_message = log_message
        self.details = details


class SpawnModelError(Exception):
    """Custom exception that is raised when an error occurred while trying to spawn a
    Gazebo model.

    Attributes:
        log_message (str): The full log message.
        details (dict): Dictionary containing extra Exception information.
    """

    def __init__(self, message="", log_message="", **details):
        """Initializes the SpawnModelError exception object.

        Args:
            message (str, optional): Exception message specifying whether the exception
                occurred. Defaults to ``""``.
            log_message (str, optional): Full log message. Defaults to ``""``.
            details (dict): Additional dictionary that can be used to supply the user
                with more details about why the exception occurred.
        """
        super().__init__(message)

        # Set attributes.
        self.log_message = log_message
        self.details = details


class GetModelStateError(Exception):
    """Custom exception that is raised when an error occurred while trying get the model
    state from gazebo.

    Attributes:
        log_message (str): The full log message.
        details (dict): Dictionary containing extra Exception information.
    """

    def __init__(self, message="", log_message="", **details):
        """Initializes the GetModelStateError exception object.

        Args:
            message (str, optional): Exception message specifying whether the exception
                occurred. Defaults to ``""``.
            log_message (str, optional): Full log message. Defaults to ``""``.
            details (dict): Additional dictionary that can be used to supply the user
                with more details about why the exception occurred.
        """
        super().__init__(message)

        # Set attributes.
        self.log_message = log_message
        self.details = details


class GetLinkStateError(Exception):
    """Custom exception that is raised when an error occurred while trying get the link
    state from gazebo.

    Attributes:
        log_message (str): The full log message.
        details (dict): Dictionary containing extra Exception information.
    """

    def __init__(self, message="", log_message="", **details):
        """Initializes the GetLinkStateError exception object.

        Args:
            message (str, optional): Exception message specifying whether the exception
                occurred. Defaults to ``""``.
            log_message (str, optional): Full log message. Defaults to ``""``.
            details (dict): Additional dictionary that can be used to supply the user
                with more details about why the exception occurred.
        """
        super().__init__(message)

        # Set attributes.
        self.log_message = log_message
        self.details = details


class SetModelStateError(Exception):
    """Custom exception that is raised when an error occurred while trying set the model
    state on gazebo.

    Attributes:
        log_message (str): The full log message.
        details (dict): Dictionary containing extra Exception information.
    """

    def __init__(self, message="", log_message="", **details):
        """Initializes the SetModelStateError exception object.

        Args:
            message (str, optional): Exception message specifying whether the exception
                occurred. Defaults to ``""``.
            log_message (str, optional): Full log message. Defaults to ``""``.
            details (dict): Additional dictionary that can be used to supply the user
                with more details about why the exception occurred.
        """
        super().__init__(message)

        # Set attributes.
        self.log_message = log_message
        self.details = details


class SetModelConfigurationError(Exception):
    """Custom exception that is raised when an error occurred while trying set the model
    configuration on gazebo.

    Attributes:
        log_message (str): The full log message.
        details (dict): Dictionary containing extra Exception information.
    """

    def __init__(self, message="", log_message="", **details):
        """Initializes the SetModelConfigurationError exception object.

        Args:
            message (str, optional): Exception message specifying whether the exception
                occurred. Defaults to ``""``.
            log_message (str, optional): Full log message. Defaults to ``""``.
            details (dict): Additional dictionary that can be used to supply the user
                with more details about why the exception occurred.
        """
        super().__init__(message)

        # Set attributes.
        self.log_message = log_message
        self.details = details


class GetPhysicsPropertiesError(Exception):
    """Custom exception that is raised when an error occurred while trying to retrieve
    the physics properties from gazebo.

    Attributes:
        log_message (str): The full log message.
        details (dict): Dictionary containing extra Exception information.
    """

    def __init__(self, message="", log_message="", **details):
        """Initializes the GetPhysicsPropertiesError exception object.

        Args:
            message (str, optional): Exception message specifying whether the exception
                occurred. Defaults to ``""``.
            log_message (str, optional): Full log message. Defaults to ``""``.
            details (dict): Additional dictionary that can be used to supply the user
                with more details about why the exception occurred.
        """
        super().__init__(message)

        # Set attributes.
        self.log_message = log_message
        self.details = details


class SetPhysicsPropertiesError(Exception):
    """Custom exception that is raised when an error occurred while trying to set
    physics properties on the gazebo physics engine.

    Attributes:
        log_message (str): The full log message.
        details (dict): Dictionary containing extra Exception information.
    """

    def __init__(self, message="", log_message="", **details):
        """Initializes the SetPhysicsPropertiesError exception object.

        Args:
            message (str, optional): Exception message specifying whether the exception
                occurred. Defaults to ``""``.
            log_message (str, optional): Full log message. Defaults to ``""``.
            details (dict): Additional dictionary that can be used to supply the user
                with more details about why the exception occurred.
        """
        super().__init__(message)

        # Set attributes.
        self.log_message = log_message
        self.details = details
