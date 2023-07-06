import InterfaceSLAM as IS

ipEsp32 = '192.168.1.85'

if __name__ == "__main__":
    interface = IS.InterfaceModule(ipEsp32, ':81', '/robot', 100)
    interface.start()