from flask import Flask, render_template, Response, request, jsonify
import pyrealsense2.pyrealsense2 as rs
import cv2
import numpy as np
import rospy
from geometry_msgs.msg import Twist, Vector3

app = Flask(__name__)

rospy.init_node('hackroverFlask')

# Create a global variable to store the current command and status
current_command = None
current_status = 'Waiting for command'

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
    global current_command, current_status

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

    # Set the current command to the new command and update the status
    current_command = command
    current_status = command + ' in progress'

    # Create a JSON response indicating the status of the command
    response = {'status': current_status}
    return jsonify(response)

# Create a ROS timer that updates the status message every 0.1 seconds
def update_status(timer_event):
    global current_command, current_status

    if current_command is None:
        current_status = 'Waiting for command'

    # Update the status message in the HTML
    script = f"document.getElementById('status-message').innerHTML = '{current_status}';"
    script += "setTimeout(() => {document.getElementById('status-message').innerHTML = '';}, 2000);"
    script += "console.log('" + current_status + "');"
    script_tag = f"<script>{script}</script>"
    response_html = f"<html>{script_tag}</html>"

    # Send the updated HTML to the client
    response = Response(response_html, content_type='text/html')
    return response

rospy.Timer(rospy.Duration(0.1), update_status) # Call update_status every 0.1 seconds
