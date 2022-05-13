#!/usr/bin/env python3

from subprocess import check_output
from re import findall
import rospy
import PIL.Image as Image
from io import BytesIO
from datetime import datetime 

class Camera:
    """
    A class used to instantiate and use the raspicam on the LeoRover
    
    ...

    Attributes:
    -----------
    none

    Methods:
    --------
    take_photo():
        Creates the ros node to capture the image, captures the int[] from the 
        compressed imaged posted to the compressed image topic, and converts it 
        to a .jpg image for the user
    __get_data_str() -> str:
        Gets the output from the rostopic /camera/image_raw/compressed for a
        single frame, and strips the output down to just the actual image data,
        and returns that as a string
    __str_to_int_list(in_str: str) -> []:
        Takes a string and converts it to a list of integers that can be 
        interpreted as unsigned 8-bit integers for an image file, returns as an
        integer list
    __list_to_img(int_list: []) -> img: 
        Takes the list of unsigned 8-bit integers and converts it into an image
        object, then returns that image
    """

    def __init__(self):
        pass

    def take_photo(self):
        rospy.init_node("raspicam_still_photo")

        data_str = self.__get_data_str()
        int_list = self.__str_to_int_list(data_str)
        img = self.__list_to_img(int_list)
        
        time = datetime.now()
        time_str = "leo_cam_" + time.strftime("%d-%m-%Y_%H:%M:%S") + ".jpg"
        
        img.save(time_str)

    def __get_data_str(self) -> str:
        data = check_output("""rostopic echo -n 1 /camera/image_raw/compressed 
                                                  #  | grep data""", shell=True)
        data = data.decode("utf-8")
        data = data.lstrip("data: [")
        data = data.rstrip()
        data = data.rstrip("]")
        
        return data

    def __str_to_int_list(self, in_str: str) -> []:
        str_len = len(in_str)
        first_item = int(in_str[0:3])
        last_item = int(in_str[str_len - 3:str_len])

        found = findall("(?<=,)(.*?)(?=,)", in_str)
        
        int_map = map(int, found)
        
        int_list = list(int_map)
        int_list.insert(0, first_item)
        int_list.append(last_item)
        
        return int_list

    def __list_to_img(self, img_list: []) -> Image:
        bytestring = bytearray()
        
        for item in img_list:
            bytestring.append(item)
        
        img = Image.open(BytesIO(bytestring))
        
        return img
