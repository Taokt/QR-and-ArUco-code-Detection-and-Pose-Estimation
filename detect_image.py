import cv2 as cv
import numpy as np
import sys
import os

def read_camera_parameters(filepath = 'intrinsic.dat'):

    inf = open(filepath, 'r')

    cmtx = []
    dist = []

    #ignore first line by read it out
    line = inf.readline()
    for _ in range(3):
        line = inf.readline().split()
        line = [float(en) for en in line]
        cmtx.append(line)

    #ignore line that says "distortion" by read it out
    line = inf.readline()
    line = inf.readline().split()
    line = [float(en) for en in line]
    dist.append(line)

    #cmtx = camera matrix, dist = distortion parameters
    return np.array(cmtx), np.array(dist)

class qr_processor:
    def __init__(self, img):
        self.img = img
        #Define coordinate points for each corner of QR code.
        mark_length = 2 # meter
        self.edges = np.array([[-mark_length/2.0,mark_length/2.0,0],
                                [mark_length/2.0,mark_length/2.0,0],
                                [mark_length/2.0,-mark_length/2.0,0],
                                [-mark_length/2.0,-mark_length/2.0,0]], dtype = 'float32').reshape((4,1,3))
        
    def camera_settr(self, cmtx, dist):
        self.cmtx = np.array(cmtx)
        self.dist = np.array(dist)

    def use_detector(self):
        img = self.img
        detecter =  cv.QRCodeDetector()
        success, corners_multi = detecter.detectMulti(img)
        self.corners_multi = corners_multi
        return corners_multi, success
    
    def find_axe(self):
        img = self.img
        if not hasattr(self, 'corners_multi'):
            self.corners_multi, _ = self.use_detector(self, img)
        
        corners_multi = self.corners_multi
        rvec_multi, tvec_multi = [], []    
        
        for corners in corners_multi:
            #determine the orientation of QR code coordinate system with respect to camera coorindate system.
            success, rvec, tvec = cv.solvePnP(self.edges, corners, self.cmtx, self.dist)
            
            [rvec,_] = cv.Rodrigues(rvec)

            if success:
                rvec_multi.append(rvec)
                tvec_multi.append(tvec)
            else:
                rvec_multi.append([])
                tvec_multi.append([])
        self.rvec_multi, self.tvec_multi = rvec_multi, tvec_multi
        return rvec_multi, tvec_multi
        
    def visualize(self):
        img = self.img
        corners_multi = self.corners_multi
        points =  corners_multi.reshape((-1, 1, 2))
        isClosed = True
        thickness = 2
        # Blue color in BGR
        color = (0, 0, 255)
        img = cv.polylines(img, np.int32([points]), isClosed, color, thickness)
        
        if hasattr(self, 'rvec_multi') & hasattr(self, 'tvec_multi'):
            rvec_multi, tvec_multi = self.rvec_multi, self.tvec_multi    
            for rvec, tvec in zip( rvec_multi, tvec_multi):
                   cv.drawFrameAxes(img, self.cmtx, self.dist, rvec, tvec, 1)
            
        while True:
            cv.imshow('image', img)
            if cv.waitKey(20) & 0xFF == 27:
                break
                
        cv.destroyAllWindows()

  
class aruco_process(qr_processor):
    def __init__(self, img, type_name):
        super().__init__(img)
        self.ARUCO_DICT = {
                    "DICT_4X4_50": cv.aruco.DICT_4X4_50,
                    "DICT_4X4_100": cv.aruco.DICT_4X4_100,
                    "DICT_4X4_250": cv.aruco.DICT_4X4_250,
                    "DICT_4X4_1000": cv.aruco.DICT_4X4_1000,
                    "DICT_5X5_50": cv.aruco.DICT_5X5_50,
                    "DICT_5X5_100": cv.aruco.DICT_5X5_100,
                    "DICT_5X5_250": cv.aruco.DICT_5X5_250,
                    "DICT_5X5_1000": cv.aruco.DICT_5X5_1000,
                    "DICT_6X6_50": cv.aruco.DICT_6X6_50,
                    "DICT_6X6_100": cv.aruco.DICT_6X6_100,
                    "DICT_6X6_250": cv.aruco.DICT_6X6_250,
                    "DICT_6X6_1000": cv.aruco.DICT_6X6_1000,
                    "DICT_7X7_50": cv.aruco.DICT_7X7_50,
                    "DICT_7X7_100": cv.aruco.DICT_7X7_100,
                    "DICT_7X7_250": cv.aruco.DICT_7X7_250,
                    "DICT_7X7_1000": cv.aruco.DICT_7X7_1000,
                    "DICT_ARUCO_ORIGINAL": cv.aruco.DICT_ARUCO_ORIGINAL,
                    "DICT_APRILTAG_16h5": cv.aruco.DICT_APRILTAG_16h5,
                    "DICT_APRILTAG_25h9": cv.aruco.DICT_APRILTAG_25h9,
                    "DICT_APRILTAG_36h10": cv.aruco.DICT_APRILTAG_36h10,
                    "DICT_APRILTAG_36h11": cv.aruco.DICT_APRILTAG_36h11
                }
        self.type = self.ARUCO_DICT[type_name]
    
    def use_detector(self):
        img = self.img
        arucoDict = cv.aruco.getPredefinedDictionary(self.type)
        parameters =  cv.aruco.DetectorParameters()
        detector = cv.aruco.ArucoDetector(arucoDict, parameters)
        corners_multi, markerIds_multi, rejectedCandidates_multi = detector.detectMarkers(img)
        self.corners_multi, self.markerIds_multi = corners_multi, markerIds_multi
        return corners_multi, markerIds_multi
        
        
    def visualize(self):
        img = self.img
        corners_multi = self.corners_multi
        ids_multi = self.markerIds_multi
                
        cv.aruco.drawDetectedMarkers(img, corners_multi, ids_multi)
        
        if hasattr(self, 'rvec_multi') & hasattr(self, 'tvec_multi'):
            rvec_multi, tvec_multi = self.rvec_multi, self.tvec_multi    
            for rvec, tvec in zip( rvec_multi, tvec_multi):
                   cv.drawFrameAxes(img, self.cmtx, self.dist, rvec, tvec, 1)
            
        while True:
            cv.imshow('image', img)
            if cv.waitKey(20) & 0xFF == 27:
                break            
        cv.destroyAllWindows()
    

        
if __name__ == '__main__':

    code_name = "qr" # "qr" or "aruco"
    
    camera_filepath = 'intrinsic.dat'
    #read camera intrinsic parameters.
    cmtx, _ = read_camera_parameters()
    dixt = []
    
    if code_name == "qr":
        input_source = 'images/test7.jpg'
        img = cv.imread(input_source, 1)
        processor = qr_processor(img)
    else:
        input_source = 'images/test4.jpg'
        img = cv.imread(input_source, 1)
        type_name = "DICT_7X7_250"
        processor = aruco_process(img, type_name)
        
    processor.camera_settr(cmtx, dixt)
    processor.use_detector()
    processor.find_axe()
    processor.visualize()