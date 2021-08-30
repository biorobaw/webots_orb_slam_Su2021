from os import walk, listdir
from os.path import isfile, join, splitext

path = "/home/noah/Webots/sequence"
rgb_path = path + "/rgb"
depth_path = path + "/depth"

file_rgb = open("rgb.txt", "a")

for f in listdir(rgb_path):
    if isfile(join(rgb_path, f)):
    	n = splitext(f)[0]
    	file_rgb.write("{} rgb/{}\n".format(n,f))

file_rgb.close()

file_depth = open("depth.txt", "a")

for f in listdir(depth_path):
    if isfile(join(depth_path, f)):
    	n = splitext(f)[0]
    	file_depth.write("{} depth/{}\n".format(n, f))

file_depth.close()

