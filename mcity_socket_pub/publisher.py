import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix

import os
from dotenv import load_dotenv
import socketio

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('socket_publisher')

        #Load environment variables
        load_dotenv()
        api_key = os.environ.get('MCITY_OCTANE_KEY', 'reticulatingsplines') 
        server = os.environ.get('MCITY_OCTANE_SERVER', 'wss://octane.mvillage.um.city/')
        namespace = "/octane"

        #If no API Key provided, exit.
        if not api_key:
            print ("No API KEY SPECIFIED. EXITING")
            exit()

        #Create an SocketIO Python client.
        sio = socketio.Client()

        def send_auth():
            sio.emit('auth', {'x-api-key': api_key}, namespace=namespace)

        @sio.on('connect', namespace=namespace)
        def on_connect():
            send_auth()

        #Make connection.
        sio.connect(server, namespaces=[namespace])

        @sio.on('join', namespace=namespace)
        def on_join(data):
            print('Joining:', data)

        @sio.on('leave', namespace=namespace)
        def on_leave(data):
            print('Leaving:', data)
            
        @sio.on('channels', namespace=namespace)
        def on_channels(data):
            print('Channel information', data)

        sio.emit('join', {'channel': 'beacon'}, namespace='/octane')

        self.publisher_ = self.create_publisher(String, 'topic', 10)
        self.i = 0

        # callback function for when a beacon is updated
        @sio.on('beacon', namespace=namespace)
        def on_update(data):
            msg = NavSatFix()
            msg.header.stamp = rospy.Time.now() # get timestamp from socket?
            msg.header.frame_id = "gps"

            # not sure if this is how the data is actually stored
            # msg.status = data['status']
            # msg.latitude = data['latitude']
            # msg.longitude = data['longitude']
            # msg.altitude = data['altitude']
            # msg.position_covariance = data['position_covariance']
            # msg.position_covariance_type = data['position_covariance_type']

            self.i += 1
            self.publisher_.publish(msg)
            # self.get_logger().info('Publishing [%d]' % self.i)

    def __del__(self):
        sio.emit('disconnect_request', namespace='/octane')
        sio.disconnect()


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()