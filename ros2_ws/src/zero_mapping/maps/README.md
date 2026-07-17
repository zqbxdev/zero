# Map Output Policy

This directory retains only this policy document. Generated occupancy-map files
are runtime results and must be written to an external prefix outside the source
repository; they are not source resources and are not installed with
`zero_mapping`.

## Future Runtime Procedure Only — Do Not Run During Build or Test

After a future mapping runtime has produced a map worth saving, an operator may
run the following command with an external destination prefix:

```bash
ros2 run nav2_map_server map_saver_cli -f <external-prefix>
```

The expected occupancy map output is YAML metadata plus an image file, normally
`.pgm` or `.png`. The prefix must remain external to
`ros2_ws/src/zero_mapping/maps/`; no generated `.yaml`, `.pgm`, or `.png` result
belongs in this package.

## Occupancy Map Versus Pose Graph

An occupancy YAML-and-image pair is input for a map server.
slam_toolbox pose-graph serialization is a separate operation with different
artifacts for continuing or refining a SLAM session.
`map_saver_cli` does not create a pose graph. A serialized pose graph must not
be described or packaged as an occupancy map result.
