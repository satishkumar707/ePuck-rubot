from utils.Freezeable import Freezeable

class Modulo( Freezeable ):
    """
    Skeleton class to handle modulo computation.
    
    This class should especially handle limit cases like
    when checking a condition a < b and "a" is increased in steps of 50,
    and b is 49999, while the modulo divisor is 50000. When a steps to 50000,
    it gets resetted to 0 and therefore a < b is still fulfilled, even though "a"
    was at 50000 before.
    """

    def __init__(self, divisor):
        '''
        Constructor
        '''
        self.divisor = divisor
        
        self.freeze()
    
    def mod(self, dividend):
        """
        Simple modulo calculation
        """
        return dividend % self.dividend
    
    
