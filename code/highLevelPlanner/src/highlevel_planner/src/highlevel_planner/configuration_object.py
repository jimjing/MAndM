class NodeNames(object):
    f = 'FrontWheel'
    b = 'BackPlate'
    l = 'LeftWheel'
    r = 'RightWheel'

class ModuleObject(object):
    def __init__(self):
        self.name = ''
        self.connected_to = {NodeNames.f:None,
                             NodeNames.b:None,
                             NodeNames.l:None,
                             NodeNames.r:None}
    def __repr__(self):
        return self.name

    def findConnectedModules(self, exclude=[]):
        connected_modules = []
        for node_name, co in self.connected_to.iteritems():
            if co != None and co not in exclude:
                if co.module2 == self:
                    connected_modules.append(co.module1)
                else:
                    connected_modules.append(co.module2)
        return connected_modules

class ConnectionObject(object):
    def __init__(self):
        self.module1 = None
        self.module2 = None
        self.node1 = ''
        self.node2 = ''

class ConfigurationObject(object):
    def __init__(self):
        self.list_of_modules = []
        self.list_of_connections = []

    def addModuleFromXml(self, xml_element):
        m = ModuleObject()
        m.name = xml_element.get('name')
        self.list_of_modules.append(m)

    def addConnectionFromXml(self, xml_element):
        c = ConnectionObject()
        c.module1 = self.findModuleWithName(xml_element.find('moduleName1').text)
        c.module2 = self.findModuleWithName(xml_element.find('moduleName2').text)
        c.node1 = xml_element.find('nodeName1').text
        c.node2 = xml_element.find('nodeName2').text
        c.module1.connected_to[c.node1] = c
        c.module2.connected_to[c.node2] = c
        self.list_of_connections.append(c)

    def findModuleWithName(self, name):
        for m in self.list_of_modules:
            if m.name == name:
                return m
        raise ValueError('Cannot find module with name: {}'.format(name))

    def findConnectionFromTwoModules(self, mo1, mo2):
        for co in self.list_of_connections:
            if co.module1 == mo1 and co.module2 == mo2:
                return co
            elif co.module1 == mo2 and co.module2 == mo1:
                return co
        raise ValueError('Cannot find connection with module {} and {}'.format(mo1, mo2))

