
from PurePursuit import path_factory
from PurePursuit import pure_pursuit
from PurePursuit.path_manager import Point

class Behavior():
    
    def __init__(self,robot):
        raise NotImplementedError("This is an abstract class")
        
    def loop(self):
        raise NotImplementedError("Must be overriden")
    
    
    
class FSMMatch(Behavior):
    def __init__(self):
        self.colour = None
        self.state = None
        
    def loop(self):
        next_state = self.state.test()
        if next_state is not None:
            self.state.deinit()
            self.state = next_state(self)
    
    
    
class FSMState:
    def __init__(self, behavior):
        self.behavior = behavior
        self.robot = self.behavior.robot

    def test(self):
        raise NotImplementedError("Must be inherited")

    def deinit(self):
        raise NotImplementedError("Must be inherited")
    
    
    
class StateTakeDisc(FSMState):
    
    def __init__(self,behavior):
        super().__init__(behavior);
        
    def test(self):
        pass
        
    def deinit(self):
        pass
        
        
        
class DeadState(FSMState):
    
    def __init__(self,behavior):
        super().__init__(behavior)
        
    def test(self):
        pass
    
    def deinit(self):
        pass