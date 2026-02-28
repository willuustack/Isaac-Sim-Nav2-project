#!/bin/bash
# replay_test.sh - Replay rosbag for regression testing
# Usage: ./replay_test.sh <path_to_bag> [--clock] [--rate=<rate>]

if [ $# -lt 1 ]; then
    echo "Usage: $0 <path_to_bag> [options]"
    echo ""
    echo "Options:"
    echo "  --clock        Publish clock time"
    echo "  --rate=<rate>  Playback rate (default: 1.0)"
    echo "  --start-paused Start paused, press space to play"
    echo ""
    echo "Example:"
    echo "  $0 /tmp/isaac_nav_bags/test_session_20250101_120000 --clock --rate=0.5"
    exit 1
fi

BAG_PATH="$1"
shift

if [ ! -d "$BAG_PATH" ]; then
    echo "Error: Bag directory not found: $BAG_PATH"
    exit 1
fi

echo "Replaying bag: $BAG_PATH"
echo ""
echo "Controls:"
echo "  Space - Pause/Play"
echo "  S     - Step (when paused)"
echo "  Q     - Quit"
echo ""

# Default options
OPTIONS="--clock --rate=1.0"

# Parse additional options
while [[ $# -gt 0 ]]; do
    case $1 in
        --clock)
            OPTIONS="$OPTIONS --clock"
            shift
            ;;
        --rate=*)
            RATE="${1#*=}"
            OPTIONS="$OPTIONS --rate=$RATE"
            shift
            ;;
        --start-paused)
            OPTIONS="$OPTIONS --start-paused"
            shift
            ;;
        *)
            echo "Unknown option: $1"
            shift
            ;;
    esac
done

echo "Running: ros2 bag play $BAG_PATH $OPTIONS"
ros2 bag play "$BAG_PATH" $OPTIONS
