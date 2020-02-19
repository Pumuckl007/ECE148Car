"""
move.py
Removes all annotations that do not have images associated with them.
"""

import os
import cv2

annots_dir = 'data/tennis_ball/annots'
no_image_dir = 'data/tennis_ball/no_image'
image_dir = 'data/tennis_ball/images'

# data/tennis_ball/annots/file.xml -> data/tennis_ball/images/file.JPEG
def to_image_path(annot_path):
    image_path = annot_path.replace('annots','images').replace('xml','JPEG')
    return image_path
    

def main():
    for fn in os.listdir(annots_dir):
        annot_path = os.path.join(annots_dir,fn)
        image_path = to_image_path(annot_path)
        img = cv2.imread(image_path)
        if(img is None):
            no_image_path = annot_path.replace('annots','no_image')
            os.rename(annot_path,no_image_path)
        print(image_path)
        
if(__name__ == '__main__'):
    main()
