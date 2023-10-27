#!/usr/bin/env python
from rcomponent.rcomponent import *


class MetadataWriter(RComponent):

    def __init__(self):
        # Init default values

        self.metadata_writers = []
        self.writers = {}
        self.folder_path = "/home/robot/"

        RComponent.__init__(self)

    def ros_read_params(self):
        """Gets params from param server"""
        RComponent.ros_read_params(self)

        self.folder_path = rospy.get_param('~folder_path', self.folder_path)
        self.metadata_writers = rospy.get_param('~metadata_writers', self.metadata_writers)


    def ros_setup(self):
        """Creates and inits ROS components"""

        RComponent.ros_setup(self)

        self.load_metadata_writers()

        return 0

    def ready_state(self):
        """Actions performed in ready state"""

        ## MetadataWriter stuff here
        return 

    def shutdown(self):
        """Shutdowns device

        Return:
            0 : if it's performed successfully
            -1: if there's any problem or the component is running
        """
        ## MetadataWriter stuff here

        return RComponent.shutdown(self)

    def switch_to_state(self, new_state):
        """Performs the change of state"""

        return RComponent.switch_to_state(self, new_state)

    def ros_publish(self):
        ## MetadataWriter stuff here
        return

    def load_metadata_writers(self):
        for metadata_writer in self.metadata_writers:
            if rospy.has_param('~'+metadata_writer) == False:
                msg = "%s::ros_setup: No parameters found for metadata_writers '%s'. Abort!"\
                    %(self._node_name, metadata_writer)
                rospy.logerr(msg)
                rospy.signal_shutdown(msg)
                return
            
            writer = self.load_metadata_writer(metadata_writer)
            self.writers[metadata_writer] = writer

    def load_metadata_writer(self, metadata_writer):
        metadata_writer_params = rospy.get_param('~' + metadata_writer)
        try:
            type_list = metadata_writer_params['type'].split('/')
            if len(type_list) == 2:
                package_name = ""
                module_name = type_list[0]
                class_name = type_list[1]
            elif len(type_list) == 3:
                package_name = type_list[0] + "."
                module_name = type_list[1]
                class_name = type_list[2]
            else:
                msg = "%s::load_metadata_writer: 'type' parameter must be defined by " \
                    + "package_name/module_name/class_name or just module_name/class_name"\
                    + " if the metadata_writer manager that you want to load is defined " \
                    + "in this package. Current 'type' parameter is '%s' for " \
                    + "the metadata_writer '%s'"
                msg = msg % (self._node_name, metadata_writer_params['type'], metadata_writer)
                rospy.logerr(msg)
                rospy.signal_shutdown(msg)
            
            module_import_name = package_name + "metadata_writers." + module_name
            module = __import__(module_import_name, fromlist=['object'])
            object_type = getattr(module, class_name)

            return object_type(self._node_name, metadata_writer, metadata_writer_params, self.folder_path)


        except KeyError as key_error:
            msg = "%s::load_metadata_writer: required %s parameter not found for '%s' metadata_writer manager."\
                 %(self._node_name, key_error, metadata_writer)
            rospy.logerr(msg)
            rospy.signal_shutdown(msg)
            return
        
        except ImportError as import_error:
            msg = "%s::load_metadata_writer: %s" %(self._node_name, import_error)
            rospy.logerr(msg)
            rospy.signal_shutdown(msg)
            return
        
        except AttributeError as attribute_error:
            msg = "%s::load_metadata_writer: %s" %(self._node_name, attribute_error)
            rospy.logerr(msg)
            rospy.signal_shutdown(msg)
            return