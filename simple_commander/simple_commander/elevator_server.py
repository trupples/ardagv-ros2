from example_interfaces.srv import SetBool
from rclpy.node import Node
import time

import rclpy


class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(SetBool, 'elevator_to_robot', self.start_elevator)

    def start_elevator(self, request, response):
        if request.data:
            print('Received request to start elevator')
            print("Waiting for elevator to pick up the box...")
            time.sleep(6.0) # Simulate elevator picking up the box
            response.success = True
            response.message = 'Elevator done'
            print('Elevator done')
        return response


def main(args=None):
    rclpy.init(args=args)

    minimal_service = MinimalService()
    print('Elevator server is running...')

    try:
        rclpy.spin(minimal_service)
    except KeyboardInterrupt:
        print('Shutting down elevator server...')
    finally:
        minimal_service.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()