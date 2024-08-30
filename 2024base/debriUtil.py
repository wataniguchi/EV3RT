from py_trees.behaviour import Behaviour
from py_trees.common import Status

class DebriStatus(Behaviour):
    def __init__(self, name: str):
        super(DebriStatus, self).__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))
        self.target_list=[[0,0]]
        self.now=[0,0]
        self.target=[0,0]
        self.i = 0

    def update(self) -> Status:
        if(self.target==[0,0]):
            self.target_list.append([0,1])
        elif(self.target==[0,1]):
            self.target_list.append([1,1])
        
        self.now = self.target
        if(self.i+1<len(self.target_list)):
            self.i += 1
            self.target = self.target_list[self.i]
        self.logger.info("now:"+str(self.now)+", target:"+str(self.target)+", targetList:"+str(self.target_list))
        return Status.SUCCESS
    
    def getTargetList(self) -> list:
        return self.target_list
    
    def getNow(self) -> list:
        return self.now
    
    def getTarget(self) -> list:
        return self.target
    
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
