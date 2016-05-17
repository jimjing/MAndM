#!/usr/bin/env python

import os
import random
import pprint

import rospy
import highlevel_planner.file_interface as fi

class HighlevelPlanner:
    def __init__(self):
        self.root_path = ''
        self.data_path = ''
        self.FI = None
        self.configuration_object = None
        self.module_mapping = {}
        self.module_popularity_mapping = {}
        self.popularity_module_mapping = {}
        self.current_holding_module = None
        self.current_carrying_module = None
        self.current_popularity_level = 0
        self.finished_connections = []
        self.in_place_modules = []

        self._initialize()

    def _initialize(self):
        rospy.init_node('HighlevelPlanner', anonymous=True, log_level=rospy.DEBUG)

        self.root_path = self.getRootPath()
        self.data_path = os.path.join(self.root_path, 'data')

        self.FI = fi.FileInterface()
        self.configuration_object = self.FI.loadConfigurationFile(os.path.join(self.data_path,'ReconfSnake.xml'))
        self.module_mapping = self.FI.loadMappingFile(os.path.join(self.data_path,'ModuleMapping.yaml'))

    def getRootPath(self):
        import rospkg
        rospack = rospkg.RosPack()
        return rospack.get_path('highlevel_planner')

    def _prepareExecute(self):
        self._rankModulesWithConnections()
        self._findInitialModule()
        self.current_carrying_module = self._findBestConnectModule(self.current_holding_module)
        rospy.loginfo('Initial carrying module is {} ...'.format(self.current_carrying_module))

    def _rankModulesWithConnections(self):
        self.module_popularity_mapping = {}
        for mo in self.configuration_object.list_of_modules:
            self.module_popularity_mapping[mo] = 0

        for connection_object in self.configuration_object.list_of_connections:
            self.module_popularity_mapping[connection_object.module1] += 1
            self.module_popularity_mapping[connection_object.module2] += 1

        for mo, popularity in self.module_popularity_mapping.iteritems():
            if popularity not in self.popularity_module_mapping.keys():
                self.popularity_module_mapping[popularity] = []
            self.popularity_module_mapping[popularity].append(mo)

    def _findInitialModule(self):
        self._updatePopularityLevel()
        self.current_holding_module = self._findNextHoldModule()
        rospy.loginfo('Initial holding module is {} ...'.format(self.current_holding_module))

    def _findBestConnectModule(self, mo):
        connected_modules = mo.findConnectedModules(exclude = self.finished_connections)
        rospy.loginfo('Module {} is connected with modules {} ...'.format(mo, connected_modules))

        return random.choice(self._findModulesWithHighestPopularity(connected_modules))

    def _updatePopularityLevel(self):
        self.current_popularity_level = max(self.popularity_module_mapping.keys())
        while self.popularity_module_mapping[self.current_popularity_level] == []:
            self.current_popularity_level -= 1
            if self.current_popularity_level == 0:
                rospy.logwarn('Finished!')
                break
        rospy.loginfo('Current popularity level is {} ...'.format(self.current_popularity_level))

    def _findNextHoldModule(self):
        if self.in_place_modules == []:
            return random.choice(self.popularity_module_mapping[self.current_popularity_level])
        else:
            return random.choice(self._findModulesWithHighestPopularity(self.in_place_modules))

    def _findModulesWithHighestPopularity(self, list_of_modules):
        best_popularity = 0
        best_modules = []
        for mo in list_of_modules:
            rospy.logdebug('Module {} has {} popularity ...'.format(mo, self.module_popularity_mapping[mo]))
            if self.module_popularity_mapping[mo] > best_popularity:
                best_popularity = self.module_popularity_mapping[mo]
                best_modules = [mo]
            elif self.module_popularity_mapping[mo] == best_popularity:
                best_modules.append(mo)

        return best_modules


    def _finishConnection(self, co):
        self.finished_connections.append(co)
        if co.module1 not in self.in_place_modules:
            self.in_place_modules.append(co.module1)
        if co.module2 not in self.in_place_modules:
            self.in_place_modules.append(co.module2)

        self.popularity_module_mapping[self.module_popularity_mapping[co.module1]].remove(co.module1)
        self.popularity_module_mapping[self.module_popularity_mapping[co.module2]].remove(co.module2)
        self.module_popularity_mapping[co.module1] -= 1
        self.module_popularity_mapping[co.module2] -= 1
        if self.module_popularity_mapping[co.module1] > 0:
            self.popularity_module_mapping[self.module_popularity_mapping[co.module1]].append(co.module1)
        if self.module_popularity_mapping[co.module2] > 0:
            self.popularity_module_mapping[self.module_popularity_mapping[co.module2]].append(co.module2)
        self._printPopularity()

    def _printPopularity(self):
        print 'Popularity'
        pprint.pprint(self.module_popularity_mapping, width=1)

    def execute(self):

        while not rospy.is_shutdown():
            co = self.configuration_object.findConnectionFromTwoModules(self.current_carrying_module, self.current_holding_module)

            if co.module1 == self.current_carrying_module:
                carry_node = co.node1
                hold_node = co.node2
            else:
                hold_node = co.node1
                carry_node = co.node2

            rospy.loginfo('Picking up module {} ...'.format(self.current_carrying_module))
            rospy.loginfo('Carrying module {} ...'.format(self.current_carrying_module))
            rospy.loginfo('Grabing module {} ...'.format(self.current_holding_module))
            rospy.loginfo('Holding module {} ...'.format(self.current_holding_module))
            rospy.loginfo('Connecting node {} of module {} to node {} of module {} ...'.format(carry_node, self.current_carrying_module, hold_node, self.current_holding_module))

            self._finishConnection(co)
            self._updatePopularityLevel()

            if self.current_popularity_level == 0:
                return

            self.current_holding_module = self._findNextHoldModule()
            rospy.loginfo('Next holding module is {} ...'.format(self.current_holding_module))
            self.current_carrying_module = self._findBestConnectModule(self.current_holding_module)
            rospy.loginfo('Next carrying module is {} ...'.format(self.current_carrying_module))

if __name__ == "__main__":
    HP = HighlevelPlanner()
    HP._prepareExecute()
    HP.execute()
