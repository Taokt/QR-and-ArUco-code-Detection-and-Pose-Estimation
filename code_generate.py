import qrcode
import numpy as np
# import argparse
import cv2 as cv
import sys

ARUCO_DICT = {
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


def generate_aruco(type_name, id_num, size, savepath):

    # load the ArUCo dictionary
    arucoDict = cv.aruco.getPredefinedDictionary(ARUCO_DICT[type_name])
    # allocate memory for the output ArUCo tag and then draw the ArUCo
    # tag on the output image
     
    tag = np.zeros((size, size, 1), dtype="uint8")
    cv.aruco.generateImageMarker(arucoDict, id_num, size, tag, 1)

    # write the generated ArUCo tag to disk
    cv.imwrite(savepath, tag)
    
    print("Generating ArUco done")
    
def generate_qr(text_to_encode, savepath):
    
    qr = qrcode.QRCode(
        version=1,
        error_correction=qrcode.constants.ERROR_CORRECT_H,
        box_size=20,
        border=3
    )
    qr.add_data(text_to_encode)
    img = qr.make_image()
    img.save(savepath)
    
    print("Generating QRcode done")
    
def call_function(num):
    if num == 1:
        text = 'It is part of the plan.'
        path = 'codes/qr_5.png'
        generate_qr(text, path)
    else:
        type_name = "DICT_7X7_250"
        id_num = 3
        size = 600 
        savepath = 'codes/aruco_3.png'
        generate_aruco(type_name, id_num, size, savepath)
    
if __name__ == "__main__":
    # 1 for generate QR code, any other number of ArUco
    num = 2
    call_function(num)

 