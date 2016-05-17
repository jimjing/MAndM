#!/usr/bin/env python

import os
import xml.etree.ElementTree as ET
import yaml

import rospy
from highlevel_planner.configuration_object import ConfigurationObject

class FileInterface:
    def __init__(self):
        pass

    def _loadFile(self, path):
        if not os.path.isfile (path):
            raise IOError('Cannot find file: {}'.format(path))

        with open(path,'r') as f:
            output = f.read()
            return output

    def loadConfigurationFile(self, path):
        configuration_object = ConfigurationObject()

        data = self._loadFile(path)
        root = ET.fromstring(data)

        for module_node in root.iter('ModuleState'):
            configuration_object.addModuleFromXml(module_node)

        for connection_node in root.iter('Connection'):
            configuration_object.addConnectionFromXml(connection_node)

        return configuration_object

    def loadAutFile(self, path):
        data = self._loadFile(path)

    def loadMappingFile(self, path):
        data = self._loadFile(path)
        return yaml.load(data)


if __name__ == "__main__":
    CFL = FileInterface()
