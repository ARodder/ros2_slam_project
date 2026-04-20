Saved SLAM artifacts live here.

Recommended contents for each mapping run:

- `<name>.yaml` and `<name>.pgm`: occupancy map for Nav2 or other map-server consumers
- `<name>.posegraph`: serialized `slam_toolbox` pose graph for later localization or continued SLAM

Use the helper script in `server_ws/offboard_slam/scripts/save_slam_session.sh` to save both into this folder.
