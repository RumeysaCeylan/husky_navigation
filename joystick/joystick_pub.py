#!/usr/bin/env python3
import math
import pygame
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Joy, Image
import cv2

class MouseJoyPublisher(Node):
    def __init__(self):
        super().__init__('robot_controller')
        self.pub = self.create_publisher(Joy, '/joy', 10)
        self.sub = self.create_subscription(Image, "/a200_0000/sensors/camera_0/color/image",self.show_image, 10)
        self.timer = self.create_timer(0.02, self.publish_joy)  # 50 Hz

        self.bridge = CvBridge()
        self.latest_frame = None
        # joystick state
        self.ax = 0.0  # axes[0] (left/right)
        self.ay = 0.0  # axes[1] (up/down)
        self.btn = 0   # buttons[0] (mouse left as button)

        # pygame init
        pygame.init()
        self.size = (360, 360)
        self.screen = pygame.display.set_mode(self.size)
        pygame.display.set_caption("/joy Stick")
        self.screen = pygame.display.set_mode((600, 800))
        self.clock = pygame.time.Clock()

        self.center = (self.size[0] // 2, self.size[1] // 2)
        self.radius = 120  
        self.knob_radius = 18
        self.knob_pos = list(self.center)
        self.dragging = False

        self.get_logger().info("Publishing /joy (axes[0], axes[1])")

    def clamp_to_circle(self, x, y):
        dx = x - self.center[0]
        dy = y - self.center[1]
        dist = math.hypot(dx, dy)
        if dist <= self.radius or dist == 0:
            return x, y
        scale = self.radius / dist
        return self.center[0] + dx * scale, self.center[1] + dy * scale

    def handle_events(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                rclpy.shutdown()
                return

            elif event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
                self.dragging = True
                self.btn = 1

            elif event.type == pygame.MOUSEBUTTONUP and event.button == 1:
                self.dragging = False
                self.btn = 0
                # return to center when released
                self.knob_pos = list(self.center)
                self.ax = 0.0
                self.ay = 0.0

            elif event.type == pygame.MOUSEMOTION and self.dragging:
                mx, my = event.pos
                cx, cy = self.clamp_to_circle(mx, my)
                self.knob_pos = [cx, cy]

                
                dx = (cx - self.center[0]) / self.radius
                dy = (cy - self.center[1]) / self.radius

                # axes convention: right=+x, up=+y (often forward is -y in screen coords)
                self.ax = float(dx)
                self.ay = float(-dy) 

    def draw(self):
        self.screen.fill((18, 18, 18))

        # base circle
        pygame.draw.circle(self.screen, (70, 70, 70), self.center, self.radius, 2)

        # crosshair
        pygame.draw.line(self.screen, (40, 40, 40), (self.center[0]-self.radius, self.center[1]), (self.center[0]+self.radius, self.center[1]), 1)
        pygame.draw.line(self.screen, (40, 40, 40), (self.center[0], self.center[1]-self.radius), (self.center[0], self.center[1]+self.radius), 1)

        # knob
        pygame.draw.circle(self.screen, (220, 220, 220), (int(self.knob_pos[0]), int(self.knob_pos[1])), self.knob_radius)

        # text
        font = pygame.font.SysFont(None, 26)
        txt = font.render(f"axes[0]={self.ax:+.2f}  axes[1]={self.ay:+.2f}  btn0={self.btn}", True, (200, 200, 200))
        self.screen.blit(txt, (14, 14))

        frame = self.latest_frame
        if frame is not None:
            #frame = cv2.flip(frame, 0)
            frame = cv2.flip(frame, -1)
            frame_rgb = frame[:, :, ::-1]               # BGR -> RGB
            frame_rgb = frame_rgb.swapaxes(0, 1)        # (H,W,3) -> (W,H,3)
            surf = pygame.surfarray.make_surface(frame_rgb)
            surf = pygame.transform.smoothscale(surf, (600, 440))  # istediğin boyut
            self.screen.blit(surf, (0, 360))            # joystick altı
        else:
            font = pygame.font.SysFont(None, 28)
            txt = font.render("Waiting for camera image...", True, (200, 200, 200))
            self.screen.blit(txt, (14, 380))

        pygame.display.flip()

    def publish_joy(self):
        # pump UI
        self.handle_events()
        self.draw()
        self.clock.tick(60)

        msg = Joy()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.axes = [self.ax, self.ay]
        msg.buttons = [self.btn]
        self.pub.publish(msg)
    def show_image(self , msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        self.latest_frame = cv_image
        # cv2.imshow("cam", cv_image)
        # cv2.waitKey(1)
         
def main():
    rclpy.init()
    node = MouseJoyPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        pygame.quit()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
