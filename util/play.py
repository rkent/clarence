#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from audio_common_msgs.msg import AudioData
import time

class AudioPlayer(Node):
    def __init__(self):
        super().__init__('audio_player')
        
        # Create publisher to the audio topic (default namespace is 'audio')
        self.publisher = self.create_publisher(AudioData, '/audio/audio', 10)
        
        # Wait for the publisher to be ready
        time.sleep(1.0)
        
    def play_mp3_file(self, file_path):
        """
        Play an MP3 file by reading it and publishing the data to the audio_play node.
        Note: This sends the raw MP3 data - the audio_play node handles decoding.
        """
        try:
            with open(file_path, 'rb') as f:
                mp3_data = f.read()
            
            # Create AudioData message
            msg = AudioData()
            msg.data = list(mp3_data)  # Convert bytes to list of uint8
            
            self.get_logger().info(f'Publishing MP3 file: {file_path} ({len(mp3_data)} bytes)')
            
            # Publish the entire file at once
            # Note: For larger files, you might want to chunk the data
            self.publisher.publish(msg)
            
            self.get_logger().info('MP3 data published successfully')
            
        except FileNotFoundError:
            self.get_logger().error(f'File not found: {file_path}')
        except Exception as e:
            self.get_logger().error(f'Error reading/publishing file: {str(e)}')

def main(args=None):
    print("Starting audio player node...")
    rclpy.init(args=args)
    
    audio_player = AudioPlayer()

    # Play the Front_Center.mp3 file
    audio_player.play_mp3_file('Front_Center.mp3')
    
    # Keep the node alive for a bit to ensure message is sent
    time.sleep(2.0)
    
    audio_player.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
