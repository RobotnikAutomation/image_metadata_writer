#!/usr/bin/env python
import rospy

import piexif
from fractions import Fraction

from sensor_msgs.msg import NavSatFix
from robotnik_msgs.srv import SetString, SetStringResponse

from base_writer import BaseWriter

class NavSatFixWriter(BaseWriter):

    def __init__(self, ns, name, params, folder_path):
        BaseWriter.__init__(self, ns, name, params, folder_path)
        
        self.gps_msg = None
        self.gps_ns = params['namespace']
        self.execute_srv = rospy.Service("~" + name + "/write", SetString, self.write_cb)
        self.gps_cb = rospy.Subscriber(self.gps_ns, NavSatFix, self.gps_cb)

        
       
    def write_cb(self, request):
        response = SetStringResponse()

        if self.gps_msg == None:
            msg = "No GPS data received"
            response.ret.success = False
            response.ret.message = msg
            rospy.logerr("%s::write_cb: %s" % (self.ns, msg))
        else:
            image_url = ""
            if request.data.startswith("/"):
                image_url = request.data
            else:
                image_url = self.folder_path + request.data
            self.write_gps(image_url)

            msg = "GPS data recorded in " + image_url

            response.ret.success = True
            response.ret.message = msg

        
        return response

    def gps_cb(self, msg):
        self.gps_msg = msg
    
    def write_gps(self, file_name):
        lat_deg = self.to_deg(self.gps_msg.latitude, ["S", "N"])
        lng_deg = self.to_deg(self.gps_msg.longitude, ["W", "E"])

        exiv_lat = (self.change_to_rational(lat_deg[0]), self.change_to_rational(lat_deg[1]), self.change_to_rational(lat_deg[2]))
        exiv_lng = (self.change_to_rational(lng_deg[0]), self.change_to_rational(lng_deg[1]), self.change_to_rational(lng_deg[2]))

        gps_ifd = {
            piexif.GPSIFD.GPSVersionID: (2, 0, 0, 0),
            piexif.GPSIFD.GPSAltitudeRef: 0,
            piexif.GPSIFD.GPSAltitude: self.change_to_rational(round(self.gps_msg.altitude)),
            piexif.GPSIFD.GPSLatitudeRef: lat_deg[3],
            piexif.GPSIFD.GPSLatitude: exiv_lat,
            piexif.GPSIFD.GPSLongitudeRef: lng_deg[3],
            piexif.GPSIFD.GPSLongitude: exiv_lng,
        }

        gps_exif = {"GPS": gps_ifd}
        BaseWriter.write(self, file_name, gps_exif)

    def to_deg(self, value, loc):
        """convert decimal coordinates into degrees, munutes and seconds tuple

        Keyword arguments: value is float gps-value, loc is direction list ["S", "N"] or ["W", "E"]
        return: tuple like (25, 13, 48.343 ,'N')
        """
        if value < 0:
            loc_value = loc[0]
        elif value > 0:
            loc_value = loc[1]
        else:
            loc_value = ""
        abs_value = abs(value)
        deg =  int(abs_value)
        t1 = (abs_value-deg)*60
        min = int(t1)
        sec = round((t1 - min)* 60, 5)
        return (deg, min, sec, loc_value)


    def change_to_rational(self, number):
        """convert a number to rantional

        Keyword arguments: number
        return: tuple like (1, 2), (numerator, denominator)
        """
        f = Fraction(str(number))
        return (f.numerator, f.denominator)

    
