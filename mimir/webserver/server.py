from flask import Flask, render_template, Response, request, jsonify
import pyrealsense2.pyrealsense2 as rs
import cv2
import numpy as np
import rospy
from geometry_msgs.msg import Twist

app = Flask(__name__)

if not rospy.get_node_uri():
    rospy.init_node('hackrover_flask_{}'.format(rospy.get_time()), anonymous=True)

def generate_frames():
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    pipeline.start(config)

    try:
        while True:
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            if not color_frame:
                continue

            frame = cv2.cvtColor(np.asanyarray(color_frame.get_data()), cv2.COLOR_BGR2RGB)
            ret, buffer = cv2.imencode('.jpg', frame)
            frame = buffer.tobytes()
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
    finally:
        pipeline.stop()

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/command', methods=['POST'])
def command():
    data = request.get_json()
    command = data['command']

    # Create a ROS publisher for the twist message
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    # Create a Twist message based on the command
    if command == 'forward':
        twist = Twist(linear=Vector3(x=0.5))
    elif command == 'backward':
        twist = Twist(linear=Vector3(x=-0.5))
    elif command == 'left':
        twist = Twist(angular=Vector3(z=0.5))
    elif command == 'right':
        twist = Twist(angular=Vector3(z=-0.5))
    else:
        twist = Twist()

    # Publish the twist message to control the rover
    pub.publish(twist)

    return jsonify({'status': 'success'})

if __name__ == '__main__':
    app.run(debug=True, host='0.0.0.0')