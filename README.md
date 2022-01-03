# scan_to_point_cloud

ROS package for converting laser scans to point clouds.

## Nodelets

`scan_to_point_cloud/scan_to_point_cloud`
### Parameters
- `target_frame` Target frame to transform scans into (defaults to scan frame).
- `fixed_frame` Fixed frame to compensate in-scan robot movement (defaults to scan frame).
- `channel_options` Channels to produce (0x03).<br>
  - `0x00` - enable no channels.<br>
  - `0x01` - enable "intensities" channel.<br>
  - `0x02` - enable "index" channel.<br>
  - `0x04` - enable "distances" channel.<br>
  - `0x08` - enable "stamps" channel.<br>
  - `0x10` - enable "viewpoint" channel.<br>
- `scan_queue_size` Scan queue size (2).
- `cloud_queue_size` Point cloud queue size (2).
- `tf_cache` Size of transform cache (10 s).
- `tf_timeout` Wait for transform timeout (1 s).
### Published topics
- `cloud` Converted point clouds.
### Subsribed topics
- `scan` Input laser scans.
