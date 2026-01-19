#!/usr/bin/env python3
"""Basic ROS2 robot visualization example using ros2_viser."""

import logging
import time
from ros2_viser import ROS2ViserVisualizer, ROS2ViserConfig

# Set up logging to see debug messages
logging.basicConfig(
    level=logging.INFO,
    format='[%(levelname)s] [%(asctime)s] [%(name)s]: %(message)s',
    datefmt='%Y-%m-%d %H:%M:%S'
)


def main():
    """Main function for basic visualization."""
    # Create configuration
    config = ROS2ViserConfig(
        robot_description_topic="/robot_description",
        joint_states_topic="/joint_states",
        root_node_name="/robot",
        update_rate=30.0
    )
    
    # Create and start visualizer
    print("Starting ROS2 Viser visualizer...")
    print("Make sure ROS2 is running and /robot_description topic is available.")
    print("=" * 60)
    
    visualizer = ROS2ViserVisualizer(config)
    
    try:
        visualizer.start()
        print("\n✅ Visualizer started successfully!")
        print("Press Ctrl+C to stop.\n")
        
        # Keep running (ViserServer needs the main thread to stay alive)
        while True:
            time.sleep(1)
            
    except KeyboardInterrupt:
        print("\nStopping visualizer...")
    except Exception as e:
        print(f"\n❌ Error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        visualizer.stop()
        print("Visualizer stopped.")


if __name__ == "__main__":
    main()
