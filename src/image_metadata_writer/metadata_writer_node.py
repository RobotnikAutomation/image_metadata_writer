#!/usr/bin/env python

import rospy
from metadata_writer import MetadataWriter


def main():

    rospy.init_node("image_metadata_writer_node")

    rc_node = MetadataWriter()

    rospy.loginfo('%s: starting' % (rospy.get_name()))

    rc_node.start()


if __name__ == "__main__":
    main()
