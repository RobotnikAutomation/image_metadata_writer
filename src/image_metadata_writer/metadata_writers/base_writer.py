#!/usr/bin/env python
import piexif

class BaseWriter():

    def __init__(self, ns, name, params, folder_path):
        self.ns = ns + "/" + name
        self.name = name
        self.params = params
        self.folder_path = folder_path
        
        self.gps_msg = None
        self.execute_srv = None         
       
    def write_cb(self, request):
        raise NotImplementedError("Function defined in base clase BaseWriter but not implemented.")

    def write(self, file_name, data):
        exif_data = piexif.load(file_name)

        # update original exif data to include GPS tag
        exif_data.update(data)
        exif_bytes = piexif.dump(exif_data)

        piexif.insert(exif_bytes, file_name)
        
        return