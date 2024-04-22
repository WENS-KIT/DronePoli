import time
import os
#content.txt와 filename.txt를 잘 쓰까쓰가 해주세요
def read_txt(labelfile, namefile) :
    if os.path.isdir('C:/Users/User/Desktop/gtsrb/txtman')==False:
        os.mkdir('C:/Users/User/Desktop/gtsrb/txtman')
    str1 = ''
    str2 = ''
    var = 'C:/Users/User/Desktop/gtsrb/'
    dirname = 'txtman/'
    f1 = open(labelfile,'r')
    f2 = open(namefile,'r')

    while True :
        str1 = f1.readline()
        str2 = f2.readline()
        if str1 == '' :
            break
        if str2 == '' :
            break
        f = open(var+dirname+str2[8:25]+'.txt', 'w')
        f.write(str1)
        f.close()

    time.sleep(2)
    f1.close()
    f2.close()

read_txt('C:/Users/User/Desktop/gtsrb/contents.txt',
         'C:/Users/User/Desktop/gtsrb/filename.txt')
