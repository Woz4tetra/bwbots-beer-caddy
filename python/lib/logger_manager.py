import os
import logging
import datetime
from logging import handlers
from lib.recursive_namespace import RecursiveNamespace


class MyFormatter(logging.Formatter):
    """Custom log line formatter so that microseconds can be included"""
    converter = datetime.datetime.fromtimestamp

    def formatTime(self, record, datefmt=None):
        ct = self.converter(record.created)
        if datefmt:
            s = ct.strftime(datefmt)
        else:
            s = ct.strftime("%Y-%m-%dT%H:%M:%S,%f")
            # s = "%s,%03d" % (t, record.msecs)
        return s


class LoggerManager:
    """
    A class that handles the global shared logger object. This object is shared across the entire project
    for a single log file. Utilize the logger by passing the output of get_logger to all parts of the project
    """
    logger = None
    name = "log"

    def __init__(self):
        raise Exception("{} is class only".format(self.__class__.__name__))

    @classmethod
    def get_logger(cls, config: RecursiveNamespace, log_only=False) -> logging.Logger:
        """
        Create a logging.Logger object
        :param config: RecursiveNamespace, contains the following properties:

        :param log_only:
        :return:
        """
        if cls.logger is not None:
            return cls.logger
        config.log_only = log_only
        config.path = cls.get_path(config)
        cls.logger = cls._create_logger(config)
        return cls.logger

    @classmethod
    def get_path(cls, config):
        if not os.path.isdir(config.directory):
            os.makedirs(config.directory)

        file_name = config.file_name.format(name=config.name, date=datetime.datetime.now())
        path = os.path.join(config.directory, file_name)
        return path

    @staticmethod
    def _create_logger(config: RecursiveNamespace):
        name = config.name
        level = config.level
        format = config.format
        suffix = config.suffix
        log_only = config.log_only
        path = config.path

        logger = logging.getLogger(name)
        logger.setLevel(level)

        formatter = MyFormatter(format)

        rotate_handle = handlers.TimedRotatingFileHandler(
            path,
            when="midnight", interval=1
        )
        rotate_handle.setLevel(level)
        rotate_handle.setFormatter(formatter)
        rotate_handle.suffix = suffix
        logger.addHandler(rotate_handle)

        if not log_only:  # add printing to stdout if this is not true
            print_handle = logging.StreamHandler()
            print_handle.setLevel(level)
            print_handle.setFormatter(formatter)
            logger.addHandler(print_handle)

        return logger
