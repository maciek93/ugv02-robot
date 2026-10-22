#!/usr/bin/env python3
"""Apply reviewed, reproducible stage-one USB/video-only adaptations."""
from pathlib import Path
import sys

root = Path(sys.argv[1])

def change(name, old, new):
    path = root / name
    text = path.read_text()
    if old not in text:
        raise RuntimeError(f'{name}: expected upstream text missing')
    path.write_text(text.replace(old, new, 1))

change('app.py', 'from base_ctrl import BaseController', 'from stationary_ctrl import BaseController')
change('app.py', 'from aiortc import RTCPeerConnection, RTCSessionDescription', '# WebRTC disabled; the installed UI uses MJPEG /video_feed.')
change('app.py', "    return offer()", "    return jsonify(error='WebRTC unavailable; use MJPEG video'), 501")
change('app.py', 'socketio = SocketIO(app)', "socketio = SocketIO(app, async_mode='threading')")
change('app.py', 'base.base_data[\'v\']', "None")
change('app.py', '    command = request.form[\'command\']', "    return jsonify(error='Robot commands disabled pending physical UART verification'), 423\n    command = request.form['command']")
change('app.py', '        base.base_json_ctrl(json)\n    except', "        return {'ok': False, 'error': 'Robot commands disabled pending physical UART verification'}\n    except")
change('app.py', '        cmd_actions[cmd_a]()', "        if cmd_a in [f['code']['mc_unlo'], f['code']['cv_auto'], f['code']['mp_hand'], f['code']['mp_face'], f['code']['mp_pose']]:\n            return {'ok': False, 'error': 'Feature disabled during stationary camera setup'}\n        cmd_actions[cmd_a]()")
change('app.py', '    # pt/arm looks forward', '    # No arm or pan/tilt is installed. All controller calls are currently no-ops.\n    # pt/arm looks forward')
change('app.py', '    cmd_on_boot()', '    print("STATIONARY MODE: no serial port opened; all robot commands disabled", flush=True)')
change('cv_ctrl.py', 'import mediapipe as mp', 'mp = None  # Optional MediaPipe is not installed for USB camera/manual setup.')
change('cv_ctrl.py', 'from picamera2 import Picamera2\nfrom picamera2.encoders import H264Encoder, Encoder\nfrom picamera2.outputs import FfmpegOutput', '# CSI camera support deferred; installed camera is USB/UVC.')
change('cv_ctrl.py', 'import depthai as dai', '# OAK camera support deferred; no OAK device installed.')
path = root / 'cv_ctrl.py'
text = path.read_text()
start = text.index('        # mediapipe\n')
end = text.index('        # base data\n', start)
# None of these optional detectors are needed in video-only mode.
text = text[:start] + '        # Optional MediaPipe and autonomous line driving are deferred.\n' + text[end:]
start = text.index('        # csi camera init\n')
end = text.index('\n\n    def frame_process', start)
text = text[:start] + '''        if not self.usb_camera_connected:
            raise RuntimeError('Configured USB camera is unavailable')
''' + text[end:]
text = text.replace('cv2.VideoCapture(0)', "cv2.VideoCapture(os.environ.get('UGV_CAMERA_DEVICE', '/dev/v4l/by-id/usb-USB_USB-video-index0'), cv2.CAP_V4L2)")
start = text.index('        lsusb_output =', text.index('    def usb_camera_detection'))
end = text.index('\n\n\n    def osd_render', start)
text = text[:start] + '''        device = os.environ.get('UGV_CAMERA_DEVICE', '/dev/v4l/by-id/usb-USB_USB-video-index0')
        return os.path.exists(device)
''' + text[end:]
# Require real camera frames: failures terminate this stream rather than posing
# a placeholder as successful observation.
text = text.replace("                    self.camera = cv2.VideoCapture(os.environ", "                    raise RuntimeError('USB camera read failed; reopen the stream after checking camera')\n                    self.camera = cv2.VideoCapture(os.environ")
start = text.index('        except Exception as e:\n', text.index('    def frame_process'))
end = text.index('\n        # opencv funcs', start)
text = text[:start] + '        except Exception:\n            raise\n' + text[end:]
text = text.replace("            self.camera.set(cv2.CAP_PROP_FRAME_WIDTH", "            self.camera.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))\n            self.camera.set(cv2.CAP_PROP_FPS, 30)\n            self.camera.set(cv2.CAP_PROP_BUFFERSIZE, 1)\n            self.camera.set(cv2.CAP_PROP_FRAME_WIDTH", 1)
path.write_text(text)
change('audio_ctrl.py', 'engine = pyttsx3.init()\nengine.setProperty(\'rate\', config[\'audio_config\'][\'speed_rate\'])', "engine = None\nif usb_connected:\n    try:\n        engine = pyttsx3.init()\n        engine.setProperty('rate', config['audio_config']['speed_rate'])\n    except Exception as exc:\n        print(f'Optional speech unavailable: {exc}')")
change('audio_ctrl.py', 'def play_speech(input_text):\n\tif not usb_connected:', 'def play_speech(input_text):\n\tif not usb_connected or engine is None:')
change('templates/index.html', '<body>', '<body><div style="background:#5b1c1c;color:white;padding:12px;text-align:center">Camera setup: driving disabled. UART, Stop and movement are not yet verified.</div>')
change('templates/index.html', '>WebRTC</button>', '>MJPEG</button>')
change('templates/index.html', '    <script src="./main.js"></script>', '    <!-- MJPEG is the supported video transport for this stage. -->')
change('templates/control.js', '(data[base_voltage]/100).toFixed(1)', "data[base_voltage] == null ? 'unknown' : (data[base_voltage]/100).toFixed(1)")
change('templates/index.html', '<span id="v_in">12</span>', '<span id="v_in">unknown</span>')
# A single camera reader prevents concurrent browser/snapshot clients racing
# over VideoCapture. Observations return real, recent frames or an error.
change('app.py', 'cvf = cv_ctrl.OpencvFuncs(thisPath, base)', 'cvf = cv_ctrl.OpencvFuncs(thisPath, base)\nfrom camera_stream import CameraStream\ncamera_stream = CameraStream(cvf.frame_process)')
path = root / 'app.py'
text = path.read_text()
start = text.index('def generate_frames():')
end = text.index('# Route to render', start)
text = text[:start] + '''def generate_frames():
    yield from camera_stream.frames()

@app.route('/snapshot.jpg')
def snapshot():
    frame = camera_stream.snapshot()
    if frame is None:
        return jsonify(error='Fresh camera observation unavailable'), 503
    return Response(frame, mimetype='image/jpeg', headers={'Cache-Control': 'no-store'})

@app.route('/health')
def health():
    ready = camera_stream.snapshot() is not None
    return jsonify(camera_ready=ready, controller_connected=False,
                   motion_enabled=False, mode='stationary'), 200 if ready else 503

''' + text[end:]
path.write_text(text)
print('Applied stationary USB camera adaptations')
