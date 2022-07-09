import yaml
from lib.recursive_namespace import RecursiveNamespace


class Config(RecursiveNamespace):
    """
    Wraps RecursiveNamespace to add features for reading and writing to files.
    RecursiveNamespace acts as a dictionary that can be accessed by dot notation as well as array notation:
        config.something_interesting == config["something_interesting"]
    """
    def __init__(self) -> None:
        super().__init__()

    @classmethod
    def from_file(cls, path):
        """
        Load parameters from a YAML file and set them as properties of this config
        :param path: absolute or relative path to YAML file
        :return: A new Config object with loaded parameters
        """
        with open(path) as file:
            config_yaml = yaml.safe_load(file)
            if config_yaml is None:
                config_yaml = {}
        self = cls()
        self.update(config_yaml)
        return self

    def save(self, path):
        """
        Save all parameters contained in this config into a YAML file
        :param path: absolute or relative path to new YAML file
        :return: None
        """
        with open(path, 'w') as file:
            yaml.safe_dump(self.to_dict(), file)
