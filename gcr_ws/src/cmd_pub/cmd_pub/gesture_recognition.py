import cv2
import mediapipe as mp
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

mp_draw = mp.solutions.drawing_utils
mp_hand = mp.solutions.hands

tipIds = [4, 8, 12, 16, 20]


class FingerCountPublisher(Node):
    def __init__(self):
        super().__init__('finger_count_publisher')
        self.publisher_ = self.create_publisher(Int32, 'finger_count', 10)

        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error("Failed to open the camera. Check the device ID and connection.")
            return

        self.publish_finger_count()

    def publish_finger_count(self):

        msg = Int32()

        with mp_hand.Hands(min_detection_confidence=0.5, min_tracking_confidence=0.5) as hands:
            while rclpy.ok():
                ret, image = self.cap.read()
                if not ret:
                    self.get_logger().warning("Failed to read frame from camera.")
                    break

                image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
                image.flags.writeable = False
                results = hands.process(image)
                image.flags.writeable = True
                image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
                lmList = []

                if results.multi_hand_landmarks:
                    for hand_landmark in results.multi_hand_landmarks:
                        myHands = results.multi_hand_landmarks[0]
                        for id, lm in enumerate(myHands.landmark):
                            h, w, c = image.shape
                            cx, cy = int(lm.x * w), int(lm.y * h)
                            lmList.append([id, cx, cy])
                        mp_draw.draw_landmarks(image, hand_landmark, mp_hand.HAND_CONNECTIONS)

                fingers = []
                if len(lmList) != 0:
                    if lmList[tipIds[0]][1] > lmList[tipIds[0] - 1][1]:
                        fingers.append(1)
                    else:
                        fingers.append(0)

                    for id in range(1, 5):
                        if lmList[tipIds[id]][2] < lmList[tipIds[id] - 2][2]:
                            fingers.append(1)
                        else:
                            fingers.append(0)

                    self.finger_counter = fingers.count(1)
                    msg.data = self.finger_counter
                    self.publisher_.publish(msg)
                    self.get_logger().info(f'Published finger count: {msg.data}')

                    # Display text on the video frame
                    if self.finger_counter == 5:
                        cv2.putText(image, 'Forward', (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2, cv2.LINE_4)
                    elif self.finger_counter == 4:
                        cv2.putText(image, 'Backward', (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2, cv2.LINE_4)
                    elif self.finger_counter == 0:
                        cv2.putText(image, 'Stop', (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2, cv2.LINE_4)
                    elif self.finger_counter == 2:
                        cv2.putText(image, 'Right', (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2, cv2.LINE_4)
                    elif self.finger_counter == 1:
                        cv2.putText(image, 'Left', (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2, cv2.LINE_4)

                cv2.imshow("result", image)

                k = cv2.waitKey(1)
                if k == ord('q'):
                    break

        self.cap.release()
        cv2.destroyAllWindows()
        

def main(args=None):
    rclpy.init(args=args)

    publisher = FingerCountPublisher()

    rclpy.spin(publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()