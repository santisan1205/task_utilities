from transitions import Machine
from task_module import Task_module as tm
import ConsoleFormatter
import time
import threading
import rospy
import os
import datetime
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

##dos estados: INIT, uno que vuelva a ask y uno que permita tomar la foto desde la tablet 

class CabinaSinfonIA:
    CAMERA_TOPIC = "/camera/rgb/image_raw"
    SAVE_DIR = os.path.expanduser("~/mercadito_photos")

    def __init__(self):
        self.console = ConsoleFormatter.ConsoleFormatter()
        self.tm = tm(speech=True, pytoolkit=True)
        self.tm.initialize_node("CABINA_SINFONIA")
        self.bridge = CvBridge()
        self.last_image = None
        os.makedirs(self.SAVE_DIR, exist_ok=True)

        #Estados
        states = ['INIT', 'ASK', 'SNAP', 'DONE']
        #Transiciones
        transitions = [
            {'trigger': 'begin', 'source': 'INIT', 'dest': 'ASK'},
            {'trigger': 'snap', 'source': 'ASK', 'dest': 'SNAP'},
            {'trigger': 'finish', 'source': 'ASK', 'dest': 'DONE'},
            {'trigger': 'done', 'source': 'SNAP', 'dest': 'DONE'}
        ]

        self.machine = Machine(model=self, states=states, transitions=transitions, initial='INIT')

        threading.Thread(target=self._watch_ros, daemon=True).start()

        self.sub = None

    def on_enter_INIT(self):
        print(self.console.format("INIT", "HEADER"))
        self.tm.initialize_pepper()
        self.tm.talk(text = "Hola, quieres tomarte una foto?")
        self.begin()

    def on_enter_ASK(self):
        print(self.console.format("ASK", "HEADER"))
        ans = (self.tm.speech2text_srv(seconds=3, lang="eng") or "").lower()
        if "no" in ans:
            self.tm.talk(text="Todo bien. ¡Hasta luego!", language="Spanish")
            self.finish()
            return
        else:
            self.tm.talk(text="Listo. Mira la cámara. Haré una cuenta regresiva.", language="Spanish")
            self.snap()

    def on_enter_SNAP(self):
        print(self.console.format("SNAP", "HEADER"))
        self.sub = rospy.Subscriber(self.CAMERA_TOPIC, Image, self._image_callback)
        for i in range(3, 0, -1):
            self.tm.talk(text=str(i), language="Spanish")
            time.sleep(1)
        self.tm.talk(text="¡Sonríe!", language="Spanish")
        time.sleep(1)
        img =self._wait_image(timeout=3)
        if img is None:
            self.tm.talk(text="No pude capturar la imagen. Lo siento.", language="Spanish")
            self.done()
            return
        ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        path = os.path.join(self.SAVE_DIR, f"photo_{ts}.jpg")
        cv2.imwrite(path, img)
        self.tm.talk(text="Foto tomada. ¡Gracias por usar la cabina de fotos!", language="Spanish")
        print(self.console.format(f"Saved: {path}", "OKBLUE"))
        self.done()
        


    def on_enter_DONE(self):
        print(self.console.format("DONE", "HEADER"))
        self.tm.talk(text="Gracias por usar la cabina de fotos. ¡Hasta luego!", language="Spanish")
        os._exit(os.EX_OK)

    def run_once(self):
        if self.state == 'INIT':
            self.on_enter_INIT()
        elif self.state == 'ASK':
            self.on_enter_ASK()
        elif self.state == 'SNAP':
            self.on_enter_SNAP()
        elif self.state == 'DONE':
            self.on_enter_DONE()
    
    if __name__ == "__main__":
        bot = CabinaSinfonIA()
        bot.run_once()
        rospy.spin()



