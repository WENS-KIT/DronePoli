import os
import time

def make_txt(now):
    file_path = now[0:len(now)-9]
    files = os.listdir(file_path)
    file_names = os.listdir(file_path)

    i = 1
    for name in file_names:
        if '.jpg' in name:
            src = os.path.join(file_path, name)
            dst = file_path[len(now)-13:len(now)-10] + '_' + str(i) + '.jpg'
            dst = os.path.join(file_path, dst)
            os.rename(src, dst)
            i += 1

make_txt(os.path.abspath(__file__))
