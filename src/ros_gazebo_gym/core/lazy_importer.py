"""Contains the LazyImporter class which can be used to import modules lazily."""

import importlib


class LazyImporter:
    """A class for importing modules lazily.

    This class can be used to import modules and module objects lazily. It caches
    imported modules and module objects to avoid repeated imports.

    Attributes:
        module_name (str): The name of the module to import.
    """

    def __init__(self, module_name):
        """Initialize a new LazyImporter object.

        Args:
            module_name (str): The name of the module to import.
        """
        self.module_name = module_name
        self._import_dict = {}

    def __getattr__(self, name):
        """Get an attribute of the LazyImporter object.

        This method is called when an attribute is accessed on the LazyImporter
        object. It tries to import the specified module or module object and
        caches the imported object to avoid repeated imports.

        Args:
            name (str): The name of the attribute to get.

        Returns:
            The imported module or module object.
        """
        # Return cached module or module object if it has already been imported.
        if name in self._import_dict:
            return self._import_dict[name]

        # Try to import module or module object.
        try:
            self._import_dict[name] = importlib.import_module(
                f"{self.module_name}.{name}"
            )
        except ModuleNotFoundError:
            # Try to see if the name refers to a module object.
            try:
                self._import_dict[name] = getattr(
                    importlib.import_module(self.module_name), name
                )
            except AttributeError:
                raise ModuleNotFoundError(
                    f"Module or object '{self.module_name}' could not be found. "
                    "Please make sure that the module is installed or the ROS "
                    "workspace that contains the module is sourced."
                )

        # If the imported object is a package, create a new LazyImporter object for it.
        if hasattr(self._import_dict[name], "__path__"):
            self._import_dict[name] = LazyImporter(f"{self.module_name}.{name}")

        return self._import_dict[name]
