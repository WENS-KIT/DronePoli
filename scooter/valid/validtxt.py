import os
import time

def make_txt(now):
    path = now[0:len(now)-11]
    os.chdir(path)
    count = 0
    files = os.listdir(path)
    f1 = open(path+'valid.txt','w')
    #f2 = open(path+'valid.txt','w')
    for file in files:
        if '.jpg' in file:
            f1.write(path+file+'\n')
            time.sleep(0.001)
            count += 1
            #if count == 4:
                #f2.write(path+file+'\n')
                #count = 0

    #f2.close()
    f1.close()

make_txt(os.path.abspath(__file__))
