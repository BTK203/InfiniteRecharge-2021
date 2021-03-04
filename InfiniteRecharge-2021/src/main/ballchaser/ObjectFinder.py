import libjevois as jevois
import cv2 as cv
import numpy as np

## Finds Objects Of Color
#
# Add some description of your module here.
#
# @author Marc Peczka
# 
# @videomapping YUYV 320 240 60 YUYV 320 240 60 Foximus ObjectFinder
# @email 
# @address 123 first street, Los Angeles CA 90012, USA
# @copyright Copyright (C) 2018 by Marc Peczka
# @mainurl 
# @supporturl 
# @otherurl 
# @license 
# @distribution Unrestricted
# @restrictions None
# @ingroup modules
class ObjectFinder:
    # ###################################################################################################
    ## Constructor
    def __init__(self):
        jevois.LINFO("ObjectFinder Constructor")
        self.frame = 0 # a simple frame counter used to demonstrate sendSerial()
        self.degrees = 320/90 # 320 Pixels, using a 90 Degree Camera lens
        self.widthOuterBuffer = 320*0.45

    # ###################################################################################################
    ## Process function with no USB output
    def processNoUSB(self, inframe):
        jevois.LINFO("process with usb")

        # Get the next camera image (may block until it is captured):
        inimg = inframe.get()
        jevois.LINFO("Input image is {} {}x{}".format(jevois.fccstr(inimg.fmt), inimg.width, inimg.height))

        # Get the next available USB output image:
        outimg = outframe.get()
        jevois.LINFO("Output image is {} {}x{}".format(jevois.fccstr(outimg.fmt), outimg.width, outimg.height))

        # Example of getting pixel data from the input and copying to the output:
        jevois.paste(inimg, outimg, 0, 0)

        # We are done with the input image:
        inframe.done()

        # Ranges For Color
        Yellow_Lower = [10, 80, 100]
        Yellow_Upper = [50, 255, 255]
        
        # Need to convert the dtype for openCV
        lower = np.array(Yellow_Lower, dtype = "uint8")
        upper = np.array(Yellow_Upper, dtype = "uint8")
        
        # Get the raw img into a workable format
        src = jevois.convertToCvBGR(inimg)
        hsv = cv.cvtColor(src, cv.COLOR_BGR2HSV)
        
        # Blur to help with edges
        blur = cv.GaussianBlur(hsv, (3, 3), 0)
        
        # Mask it
        mask = cv.inRange(hsv, lower, upper)
        
        #Erode image to reduce edges
        kernel = cv.getStructuringElement(cv.MORPH_RECT, (6,6))
        mask = cv.erode(mask, kernel)
        
        #Dilate image to make what we have more significant
        mask = cv.dilate(mask, kernel)
        
        # Find edges
        canny_edge = cv.Canny(mask, 100, 200)
        
        # Find circles
        rows = canny_edge.shape[0]
        circles = cv.HoughCircles(canny_edge, cv.HOUGH_GRADIENT, 1, 100,
                               param1=100, param2=15,
                               minRadius=5, maxRadius=50)
        
        # Circles Are reported as (X, Y, Radius)
        #jevois.LINFO("CIRCLES: {}".format(circles))
        if outimg is not None:
            if circles is not None:
                circles = np.uint16(np.around(circles))
                for i in circles[0, :]:
                    center = (i[0], i[1])
                    # circle center
                    #cv.circle(src, center, 1, (0, 100, 100), 3)
                    # circle outline
                    radius = i[2]
                    #cv.circle(src, center, radius, (255, 0, 255), 3)

        # We are done with the output, ready to send it to host over USB:

        #cv.line(src, (int(self.widthOuterBuffer), 0), (int(self.widthOuterBuffer), 240), (255, 0, 0), 5)
        #cv.line(src, (320 - int(self.widthOuterBuffer), 0), (320 - int(self.widthOuterBuffer), 240), (255, 0, 0), 5)
        
        jevois.sendSerial("{}".format(circles))
        
        self.frame += 1

    # ###################################################################################################
    ## Process function with USB output
    def process(self, inframe, outframe):
        jevois.LINFO("process with usb")

        # Get the next camera image (may block until it is captured):
        inimg = inframe.get()
        jevois.LINFO("Input image is {} {}x{}".format(jevois.fccstr(inimg.fmt), inimg.width, inimg.height))

        # Get the next available USB output image:
        outimg = outframe.get()
        jevois.LINFO("Output image is {} {}x{}".format(jevois.fccstr(outimg.fmt), outimg.width, outimg.height))

        # Example of getting pixel data from the input and copying to the output:
        jevois.paste(inimg, outimg, 0, 0)

        # We are done with the input image:
        inframe.done()

        # Ranges For Color
        Yellow_Lower = [10, 80, 100]
        Yellow_Upper = [50, 255, 255]
        
        # Need to convert the dtype for openCV
        lower = np.array(Yellow_Lower, dtype = "uint8")
        upper = np.array(Yellow_Upper, dtype = "uint8")
        
        # Get the raw img into a workable format
        src = jevois.convertToCvBGR(inimg)
        hsv = cv.cvtColor(src, cv.COLOR_BGR2HSV)
        
        # Blur to help with edges
        blur = cv.GaussianBlur(hsv, (3, 3), 0)
        
        # Mask it
        mask = cv.inRange(hsv, lower, upper)
        
        #Erode image to reduce edges
        kernel = cv.getStructuringElement(cv.MORPH_RECT, (6,6))
        mask = cv.erode(mask, kernel)
        
        #Dilate image to make what we have more significant
        mask = cv.dilate(mask, kernel)
        
        # Find edges
        canny_edge = cv.Canny(mask, 100, 200)
        
        # Find circles
        rows = canny_edge.shape[0]
        circles = cv.HoughCircles(canny_edge, cv.HOUGH_GRADIENT, 1, 100,
                               param1=100, param2=15,
                               minRadius=5, maxRadius=50)
        
        # Circles Are reported as (X, Y, Radius)
        #jevois.LINFO("CIRCLES: {}".format(circles))
        if outimg is not None:
            if circles is not None:
                circles = np.uint16(np.around(circles))
                for i in circles[0, :]:
                    center = (i[0], i[1])
                    # circle center
                    cv.circle(src, center, 1, (0, 100, 100), 3)
                    # circle outline
                    radius = i[2]
                    cv.circle(src, center, radius, (255, 0, 255), 3)

        # We are done with the output, ready to send it to host over USB:

        cv.line(src, (int(self.widthOuterBuffer), 0), (int(self.widthOuterBuffer), 240), (255, 0, 0), 5)
        cv.line(src, (320 - int(self.widthOuterBuffer), 0), (320 - int(self.widthOuterBuffer), 240), (255, 0, 0), 5)

        

        jevois.convertCvBGRtoRawImage(src, outimg, 1)
        outframe.send()
        
        jevois.sendSerial("[[[5, 8, 10]]]")
        
        #jevois.sendSerial("{}".format(circles))
        self.frame += 1

    # ###################################################################################################
    ## Parse a serial command forwarded to us by the JeVois Engine, return a string
    def parseSerial(self, str):
        jevois.LINFO("parseserial received command [{}]".format(str))
        if str == "hello":
            return self.hello()
        return "ERR: Unsupported command"
    
    # ###################################################################################################
    ## Return a string that describes the custom commands we support, for the JeVois help message
    def supportedCommands(self):
        # use \n seperator if your module supports several commands
        return "hello - print hello using python"

    # ###################################################################################################
    ## Internal method that gets invoked as a custom command
    def hello(self):
        return "Hello from python!"
