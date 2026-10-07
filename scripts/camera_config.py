from dataclasses import dataclass

@dataclass
class OpenCVConfig():
    port: int = 0
    width: int = 640
    height: int = 480
    frame_type = "RGB"

    def build_camera(self) -> "OpenCVCamera":
        return OpenCVCamera(self)

class OpenCVCamera():
    width: int
    height: int

    def __init__(self, conf: OpenCVConfig):
        try:
            import cv2
        except ImportError:
            raise ImportError("OpenCV is not installed. In order to use the OpenCV Camera, please install it using 'pip install opencv-python'")

        self.cap = cv2.VideoCapture(conf.port)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, conf.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, conf.height)
        
        self.width = conf.width
        self.height = conf.height
        
    def next_frame(self):
        ret = False
        counter = 0
        while not ret and counter < 5:
            ret, frame = self.cap.read()
            counter += 1
        if counter == 5:
            frame = np.zeros([self.height, self.width, 3])
        frame[:,:,[0,2]] = frame[:,:,[2,0]] # BGR to RGB
        return frame
