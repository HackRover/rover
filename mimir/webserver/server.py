from flask import Flask, render_template, Response, request, jsonify, stream_with_context
import pyrealsense2.pyrealsense2 as rs
import cv2
import matplotlib.pyplot as plt
import numpy as np
import rospy
from geometry_msgs.msg import Twist, Vector3

app = Flask(__name__)

rospy.init_node('hackroverFlask')

# Some global variables
current_command = None
current_status = 'Waiting for command'
pipeline = None
distance_value = 0
running = True

def start_pipeline():
    global pipeline
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    config.enable_stream(rs.stream.depth, 320, 240, rs.format.z16, 30)
    pipeline.start(config)


def generate_color_frames():
    global pipeline, running
    while running:
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            continue
        img = np.asanyarray(color_frame.get_data())
        ret, buffer = cv2.imencode('.jpg', img, params=[cv2.IMWRITE_JPEG_QUALITY, 90, cv2.IMWRITE_JPEG_OPTIMIZE, 1])
        frame = buffer.tobytes()

        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

def generate_depth_frames():
    global pipeline, running, distance_value
    align = rs.align(rs.stream.color)
    colorizer = rs.colorizer()
    while running:
        frames = pipeline.wait_for_frames()
        aligned_frames = align.process(frames)
        colorized = colorizer.process(aligned_frames).as_frame()

        depth_frame = aligned_frames.get_depth_frame()
        if not depth_frame:
            continue

        # Calculate distance value at the center of the frame
        width, height = depth_frame.get_width(), depth_frame.get_height()
        distance_value = depth_frame.get_distance(width // 2, height // 2)

        # Convert depth frame to heatmap
        heatmap = cv2.applyColorMap(cv2.convertScaleAbs(np.asanyarray(depth_frame.get_data()), alpha=0.03), cv2.COLORMAP_JET)

        # Convert heatmap to RGB format
        heatmap_rgb = cv2.cvtColor(heatmap, cv2.COLOR_BGR2RGB)

        # Encode image as JPEG with optimization
        ret, buffer = cv2.imencode('.jpg', heatmap_rgb, params=[cv2.IMWRITE_JPEG_QUALITY, 90, cv2.IMWRITE_JPEG_OPTIMIZE, 1])
        frame = buffer.tobytes()

        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/video_feed')
def video_feed():
    return Response(generate_color_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/depth_feed')
def depth_feed():
    return Response(generate_depth_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

def distance_data():
    global distance_value
    while True:
        yield f'data: {distance_value}\n\n'

@app.route('/distance_feed')
def distance_feed():
    return Response(stream_with_context(distance_data()), mimetype='text/event-stream')

# start that pipeline
start_pipeline()

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