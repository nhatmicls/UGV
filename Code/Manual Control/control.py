import pygame
import time
import serial
import math

NameCOM="COM4"

button=[0,0,0,0,0]
axis=[0,0,0]
datasend=[]

def main():
    cache=[]
    transmit=serial.Serial(NameCOM,115200,timeout=2.5)

    pygame.display.init()
    pygame.joystick.init()
    print("Number device found: ",pygame.joystick.get_count())

    controljoy=pygame.joystick.Joystick(0)
    controljoy.init()
    print(controljoy.get_init())
    print("ID: ",controljoy.get_id())
    print("Name device: ",controljoy.get_name())
    print("Number of axis: ",controljoy.get_numaxes())
    print("Number of ball: ",controljoy.get_numballs())
    print("Number of button: ",controljoy.get_numbuttons())
    print("Number of hat button: ",controljoy.get_numhats())
    time.sleep(1)
    
    while(1):
        pygame.event.pump()

        #Get and calc data from joystick

        for i in range(2):
            axis[i]=round(controljoy.get_axis(i)*100,0)

            if(i==0):
                axis[i]=(3/4)*axis[i]+425
            if(i==1):
                axis[i]=-(7/2)*axis[i]
            axis[i]=int(axis[i])
            if(axis[i]==0):
                axis[i]=1
            if(axis[i]<0):
                cacheaxis=abs(axis[i])
                cachedic=1
            else:
                cacheaxis=axis[i]
                cachedic=0

            if(cacheaxis<10):
                cache+="00"
            elif (cacheaxis<100):
                cache+="0"

            if(i==0):
                cache+=str(axis[i])
            elif(i==1):
                cache+=str(cacheaxis)
                cache+=str(cachedic)

        for i in range(5):
            button[i]=controljoy.get_button(i)

        # cache+="."
        cache+=str(len(cache)-1)
        cache+="]"
        datasend=''.join(cache)
        #cache=datasend.encode()
        #print(type(datasend))
        print("{}".format(datasend))
        transmit.write(datasend.encode())
        time.sleep(0.02)
        cache=[]
        cache+="["
        datasend=[] 

if __name__ == "__main__":
    main()