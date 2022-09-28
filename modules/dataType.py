from utils.Freezeable import Freezeable

class Odometry(Freezeable):
    """
    Stores odometry.
    """
    
    def __init__(self):
        
        self.location = [0, 0]
        self.angle = 0
        
        self.freeze()

    def getLocationString(self):
        """
        Returns a formatted String of location. Example: (1000,1000)
        """
        return "(" + str(self.location[0]) + "," + str(self.location[1]) + ")"
    
    def isValidLocation(self):
        """
        Returns true if coordinates are bigger than/equal 0
        """
        _bool = False
        if self.location[0] >= 0 and self.location[1] >= 0:
            _bool = True
        return _bool
