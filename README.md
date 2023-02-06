# Image Metadata Writer
Node to write metadata into JPG images


## GPS Writer

Reads sensor_msgs/NavSatFix message and writes it into image metadata. Config example:

```yaml
metadata_writers:
  - gps

gps:
  type: gps_writer/NavSatFixWriter
  namespace: gps/fix
```