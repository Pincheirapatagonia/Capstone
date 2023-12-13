from comunication import Communication
from control import PID
from tracker import NutsTracker
from threading import Thread


if __name__ == '__main__':
    print("Starting...")
    print("Video Started!")
    tracker = NutsTracker()
    tracker.initiateVideo()
    t1 = Thread(target = tracker.track)
    t1.start()

    coms = Communication()
    coms.begin()
    print("Coms Started!")

    RPMA = 0
    RPMB = 0

    try:
        while True:
            # hay que convertir a metros
            msg = f"{tracker.x},{tracker.y}"
            coms.comunicacion(msg)
            coms.read_and_print_messages()
            if(coms.data != ""):
                splitData = coms.data.split(',')
                timestamp = splitData[0]
                posX = splitData[1].split(':')[1]
    
    except KeyboardInterrupt:
        print("Data collection interrupted.")
        coms.close()
