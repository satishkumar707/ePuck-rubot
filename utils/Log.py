from settings import Setup
from collections import namedtuple
import os

# DO NOT EDIT HERE. CHANGE SETTINGS IN "settings.py"
options = namedtuple("options", "no error log all")
o = options("no", "error", "log", "all")

setup = Setup()
path = "/local/ePuck/log.txt"

# log
def l(name, text):
    if setup.other.filter_log == o.all or setup.other.filter_log == o.log:
        message = "[Log] _" + str(name) + "_ " + str(text)
        print message
        _file = openFile()
        if _file != None:
            _file.write(message + "\n")

# debug
def d(name, text):
    if setup.other.filter_log == o.all or setup.other.filter_log == o.error:
        message = "[Debug] _" + str(name) + "_ " + str(text)
        print message
        _file = openFile()
        if _file != None:
            _file.write(message + "\n")
      
# error  
def e(name, text):
    if setup.other.filter_log == o.all or setup.other.filter_log == o.error:
        message = "[Error] _" + str(name) + "_ " + str(text)
        print message
        _file = openFile()
        if _file != None:
            _file.write(message + "\n")

def openFile():
    _file = None
    
    try:
        _file = open(path, "a")
    except:
        pass
    
    return _file

def createLogFile(path, file_name):
    if not os.path.exists(path):
        # create directories
        os.makedirs(path)
    _file = open(path + file_name, "w")
