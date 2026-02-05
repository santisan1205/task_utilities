
#!/usr/bin/env python3
"""
CabinaSinfonIA v2 (annotated): photo booth state machine for Pepper/ROS.

This file marks modifications compared to CabinaSinfonIA_FIXED.py with
bold comments like:  # **CHANGED:** reason...
"""
import os
import sys
import time
import datetime
import threading
# **CHANGED:** new import for CLI parsing
import argparse  # **CHANGED**

# --- Robust Console formatter (noop if not present) ----------------------
try:
    import ConsoleFormatter
    _fmt = ConsoleFormatter.ConsoleFormatter()
    def fmt(tag, style="OK"):
        return _fmt.format(tag, style)
except Exception:
    def fmt(tag, style="OK"):
        return f"[{style}] {tag}"

# --- Task utilities import shims -----------------------------------------
TaskAPI = None
_import_error = None  # **CHANGED** ensure defined for error message

# Newer package (preferred)
try:
    from task_utilities2.task_module import TaskModule as _TaskModuleNew
    TaskAPI = _TaskModuleNew
except Exception:
    try:
        from task_module import Task_module as _TaskModuleOld
        TaskAPI = _TaskModuleOld
    except Exception as e:
        TaskAPI = None
        _import_error = e

# --- ROS imports (ROS2 preferred; fallback to ROS1 if available) ---------
try:
    import rclpy
    from rclpy.node import Node
    ROS2 = True
except Exception:
    ROS2 = False
    try:
        import rospy
    except Exception:
        rospy = None

# Image pipeline
try:
    from sensor_msgs.msg import Image
    from cv_bridge import CvBridge
except Exception as e:
    Image = None
    CvBridge = None
    _cv_import_error = e

import cv2

class CabinaSinfonIA:
    CAMERA_TOPIC = '/camera/rgb/image_raw'

    def __init__(self, save_dir=None, camera_topic=None, lang='spa', stt_seconds=3, headless=False):
        if TaskAPI is None:
            raise ImportError(
                "Could not import SinfonIA task utilities. "
                "Tried `task_utilities2.task_module.TaskModule` and legacy `task_module.Task_module`. "
                f"Root cause: {_import_error if _import_error else 'unknown'}"
            )

        self.tm = TaskAPI()
        self.lang = lang
        self.stt_seconds = stt_seconds  # **CHANGED** configurable listen window
        self.bridge = CvBridge() if CvBridge else None
        self.last_frame = None
        self.state = 'INIT'
        # **CHANGED:** allow env override for topic
        env_topic = os.getenv('SINFONIA_CAMERA_TOPIC')  # **CHANGED**
        self.camera_topic = camera_topic or env_topic or self.CAMERA_TOPIC  # **CHANGED**
        self.headless = headless  # **CHANGED**

        # Folder for pictures
        default_dir = os.path.expanduser('~/Pictures/SinfonIA')
        self.save_dir = save_dir or default_dir
        os.makedirs(self.save_dir, exist_ok=True)

        # **CHANGED:** defer ROS node creation until we actually need/aren't headless
        self._ros_ready = False  # **CHANGED**
        if not self.headless:  # **CHANGED**
            self._init_ros()  # **CHANGED**

    # **CHANGED:** factored ROS init to its own method
    def _init_ros(self):  # **CHANGED**
        if ROS2:
            if not rclpy.ok():
                rclpy.init()
            self.node = rclpy.create_node('cabina_sinfonia')
            self.subscription = self.node.create_subscription(
                Image, self.camera_topic, self._ros2_image_cb, 10
            )
            self._ros_ready = True
        else:
            if rospy is None:
                print(fmt("ROS not available; will try OpenCV fallback", "WARNING"))  # **CHANGED**
                self._ros_ready = False  # **CHANGED**
                return
            if not rospy.core.is_initialized():
                rospy.init_node('cabina_sinfonia', anonymous=True)
            self.subscriber = rospy.Subscriber(self.camera_topic, Image, self._ros1_image_cb, queue_size=1)
            self._ros_ready = True

    # ---------------------------- ROS Callbacks ----------------------------
    def _ros2_image_cb(self, msg: Image):
        if self.bridge:
            self.last_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def _ros1_image_cb(self, msg: Image):
        if self.bridge:
            self.last_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    # ----------------------------- State Hooks -----------------------------
    def on_enter_INIT(self):
        print(fmt("INIT", "HEADER"))
        try:
            if hasattr(self.tm, 'initialize_pepper'):
                self.tm.initialize_pepper()
        except Exception:
            pass

        # **CHANGED:** friendlier prompt and language autodetect
        greeting = "Hola, ¿quieres tomarte una foto? (sí/no)" if self.lang.startswith('spa') else "Hi! Want a photo? (yes/no)"  # **CHANGED**
        self._talk(greeting)  # **CHANGED**
        self._advance('ASK')

    def on_enter_ASK(self):
        print(fmt("ASK", "HEADER"))
        ans = self._listen(seconds=self.stt_seconds)  # **CHANGED**
        ans = (ans or '').strip().lower()

        negatives = ('no', 'nop', 'nope', 'nah', 'noup')  # **CHANGED** small expansion
        if any(k in ans for k in negatives):
            bye = "Todo bien. ¡Hasta luego!" if self.lang.startswith('spa') else "All good. See you!"  # **CHANGED**
            self._talk(bye)  # **CHANGED**
            self._advance('DONE')
            return

        prep = "Listo. Mira la cámara. Haré una cuenta regresiva." if self.lang.startswith('spa') else "Okay. Look at the camera. Countdown time."  # **CHANGED**
        self._talk(prep)  # **CHANGED**
        self._advance('SNAP')

    def on_enter_SNAP(self):
        print(fmt("SNAP", "HEADER"))
        for n in (3, 2, 1):
            self._talk(str(n))
            time.sleep(1.0)

        frame = self._wait_for_frame(timeout=5.0)
        # **CHANGED:** Fallback to OpenCV webcam if no ROS frame
        if frame is None:  # **CHANGED**
            frame = self._opencv_fallback_capture()  # **CHANGED**

        if frame is None:
            msg = "No pude ver la cámara. Intentemos de nuevo luego." if self.lang.startswith('spa') else "I couldn't see the camera. Try again later."  # **CHANGED**
            self._talk(msg)  # **CHANGED**
            self._advance('DONE')
            return

        timestamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        out_path = os.path.join(self.save_dir, f'SinfonIA_{timestamp}.jpg')
        cv2.imwrite(out_path, frame)

        done = "¡Listo! Ya guardé tu foto." if self.lang.startswith('spa') else "Done! Photo saved."  # **CHANGED**
        self._talk(done)  # **CHANGED**
        print(fmt(f"Saved photo -> {out_path}", "OKGREEN"))
        self._advance('DONE')

    def on_enter_DONE(self):
        print(fmt("DONE", "OKBLUE"))
        bye2 = "Fin de la cabina. Gracias." if self.lang.startswith('spa') else "Photo booth finished. Thanks!"  # **CHANGED**
        self._talk(bye2)  # **CHANGED**

    # ---------------------------- Helpers ----------------------------------
    def _advance(self, next_state):
        self.state = next_state
        getattr(self, f'on_enter_{next_state}')()

    def _wait_for_frame(self, timeout=5.0):
        start = time.time()
        while time.time() - start < timeout:
            if self.last_frame is not None:
                return self.last_frame.copy()
            time.sleep(0.05)
        return None

    def _opencv_fallback_capture(self):  # **CHANGED** new helper
        try:
            cap = cv2.VideoCapture(0)
            ok, frame = cap.read()
            cap.release()
            if ok:
                print(fmt("Captured frame via OpenCV fallback", "WARNING"))  # **CHANGED**
                return frame
        except Exception as e:
            print(fmt(f"OpenCV fallback failed: {e}", "WARNING"))  # **CHANGED**
        return None

    def _talk(self, text, language=None):
        language = language or ('Spanish' if self.lang.startswith('spa') else 'English')
        try:
            if hasattr(self.tm, 'talk'):
                self.tm.talk(text=text, language=language)
            elif hasattr(self.tm, 'say'):
                self.tm.say(text)
            else:
                print(fmt(f"TTS shim: {text}", "INFO"))  # **CHANGED**
        except Exception as e:
            print(fmt(f"TALK failed: {e}", "WARNING"))

    def _listen(self, seconds=3, lang='eng'):
        try:
            if hasattr(self.tm, 'speech2text_srv'):
                return self.tm.speech2text_srv(seconds=seconds, lang=lang) or ''
            if hasattr(self.tm, 'stt'):
                return self.tm.stt(timeout=seconds, lang=lang) or ''
        except Exception as e:
            print(fmt(f"STT failed: {e}", "WARNING"))
        return ''

    # Public runner (single pass)
    def run_once(self):
        # **CHANGED:** init ROS here if user started headless but later wants ROS (no-op if already done)
        if not self._ros_ready and not self.headless:  # **CHANGED**
            self._init_ros()  # **CHANGED**
        self.on_enter_INIT()

# ----------------------------- Entrypoint ---------------------------------
def _build_parser():  # **CHANGED** new CLI
    p = argparse.ArgumentParser(description="Cabina SinfonIA")
    p.add_argument('--save-dir', type=str, default=None, help='Directory to save photos')
    p.add_argument('--topic', type=str, default=None, help='Camera topic (overrides env)')
    p.add_argument('--lang', type=str, default='spa', help='Language code (spa|eng)')
    p.add_argument('--stt-seconds', type=int, default=3, help='Seconds to listen for yes/no')
    p.add_argument('--headless', action='store_true', help='Skip ROS init; try OpenCV fallback only')
    return p

def main():
    parser = _build_parser()  # **CHANGED**
    args = parser.parse_args()  # **CHANGED**

    bot = CabinaSinfonIA(
        save_dir=args.save_dir,
        camera_topic=args.topic,
        lang=args.lang,
        stt_seconds=args.stt_seconds,
        headless=args.headless
    )  # **CHANGED**
    bot.run_once()
    if bot.headless:  # **CHANGED**
        return
    if ROS2:
        try:
            rclpy.spin(bot.node)
        finally:
            bot.node.destroy_node()
            rclpy.shutdown()
    else:
        import rospy
        rospy.spin()

if __name__ == "__main__":
    main()
