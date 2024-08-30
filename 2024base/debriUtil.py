from enum import Enum, auto
from py_trees.behaviour import Behaviour
from py_trees.common import Status
from py_etrobo_util import Video, TraceSide, Plotter

class Bottle(Enum):
    RED = auto()
    BLUE = auto()
    NONE = auto()
    END = auto()

class DebriStatus(Behaviour):
    def __init__(self, name: str):
        super(DebriStatus, self).__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))
        self.target_list=[[0,0]]
        self.now=[0,0]
        self.target=[0,0]
        self.i = 0
        self.bottle = Bottle.NONE

    def update(self) -> Status:
        if(self.target==[0,0]):
            self.target_list.append([0,1])
        elif(self.target==[0,1]):
            self.target_list.append([1,1])
        
        self.now = self.target
        if(self.i+1<len(self.target_list)):
            self.i += 1
            self.target = self.target_list[self.i]
        else:
            self.bottle = Bottle.END
        self.logger.info("now:"+str(self.now)+", target:"+str(self.target)+", targetList:"+str(self.target_list))
        return Status.SUCCESS
    
    def getTargetList(self) -> list:
        return self.target_list
    
    def getNow(self) -> list:
        return self.now
    
    def getTarget(self) -> list:
        return self.target
    
    def getBottle(self) -> Bottle:
        return self.bottle
    
    def setBottle(self, bottle) -> None:
        self.bottle = bottle
    
class IsEnd(Behaviour):
    def __init__(self, name: str, status: DebriStatus):
        super(IsEnd, self).__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))
        self.debri_status = status

    def update(self) -> Status:
        target_list = self.debri_status.getTargetList()
        now = self.debri_status.getNow()

        if(now == target_list[-1]):
            return Status.SUCCESS
        else:
            return Status.RUNNING

class IdentifyBottle(Behaviour):
    def __init__(self, name: str, video: Video, status: DebriStatus):
        super(IdentifyBottle, self).__init__(name)
        self.running = False
        self.prev_bottle = Bottle.NONE
        self.i = 0
        self.video = video
        self.status = status

    def update(self) -> Status:
        LOWER_RANGE = 8000
        if not self.running:
            self.running = True
            self.prev_bottle = Bottle.NONE
            self.i=0
        
        ror = self.video.get_range_of_red()
        rob = self.video.get_range_of_blue()

        if(ror>LOWER_RANGE):
            bottle = Bottle.RED
        elif(rob>LOWER_RANGE):
            bottle = Bottle.BLUE
        else:
            bottle = Bottle.NONE
        
        if(self.prev_bottle == bottle):
            self.i+=1
            if(self.i>3):
                self.status.setBottle(bottle)
                return Status.SUCCESS
        else:
            self.prev_bottle = bottle
            self.i=0

        return Status.RUNNING

class IsExecuteRemoveBottle(Behaviour):
    def __init__(self, name: str, debri_status: DebriStatus):
        super(IsExecuteRemoveBottle, self).__init__(name)
        self.status = debri_status

    def update(self) -> Status:
        if(self.status.getBottle()==Bottle.BLUE or self.status.getBottle()==Bottle.RED):
            return Status.SUCCESS
        else:
            return Status.FAILURE

class IsExecuteCrossCircle(Behaviour):
    def __init__(self, name: str, debri_status: DebriStatus):
        super(IsExecuteCrossCircle, self).__init__(name)
        self.status = debri_status

    def update(self) -> Status:
        if(self.status.getBottle()==Bottle.NONE):
            return Status.SUCCESS
        else:
            return Status.FAILURE
