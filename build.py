from os import system

system('''
g++ -std=c++11 -Wall main.cpp -o main \
`pkg-config --libs --cflags opencv` -ldl -lm \
-I/home/dennis/Programs/C/lib/dennis/motor \
-L/home/dennis/Programs/C/lib/dennis/motor \
-I/home/dennis/Programs/C/lib/wmx/v/photo \
-L/home/dennis/Programs/C/lib/wmx/v/build/photo \
-I/usr/local/include \
-L/usr/local/lib \
-lwiringPi -lmotor -lopcv \
-lraspicam -lraspicam_cv -lopencv_core -lopencv_highgui -lopencv_imgcodecs
''')

