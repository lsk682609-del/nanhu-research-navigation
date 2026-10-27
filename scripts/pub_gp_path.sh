SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "${SCRIPT_DIR}/install/setup.sh"
ros2 topic pub /navigate_track_gp_path nav_msgs/msg/Path "{
header: {frame_id: 'gps'},
poses: [
    {pose: {position: {x: 30.81357180, y: 120.83381690, z: 12.0}, orientation: {w: 1.0}}},
    {pose: {position: {x: 30.81360660, y: 120.83380662, z: 12.0}, orientation: {w: 1.0}}},
    {pose: {position: {x: 30.81359725, y: 120.83383081, z: 12.0}, orientation: {w: 1.0}}},
    {pose: {position: {x: 30.81361647, y: 120.83385138, z: 12.0}, orientation: {w: 1.0}}},
    {pose: {position: {x: 30.81366006, y: 120.83382396, z: 12.0}, orientation: {w: 1.0}}},
    {pose: {position: {x: 30.81369330, y: 120.83378787, z: 12.0}, orientation: {w: 1.0}}}
]
}" --once
