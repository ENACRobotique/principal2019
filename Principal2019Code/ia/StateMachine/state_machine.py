
from PurePursuit import path_factory as pf
from PurePursuit import pure_pursuit as pp
from PurePursuit.path_manager import Point

from time import time
import params as p

class Behavior():
    
    def __init__(self,robot):
        raise NotImplementedError("This is an abstract class")
        
    def loop(self):
        raise NotImplementedError("Must be overriden")
    
    
    
class FSMMatch(Behavior):
    def __init__(self,robot):
        self.robot = robot
        self.colour = None
        self.state = None
        self.start_match = None
        self.time_now = None
        self.locomotion = pp.PurePursuit(robot)
        
    def loop(self):
        
        if self.start_match is not None and self.time_now - self.start_match >p.MATCH_DURATION:
            next_state = DeadState()
        else:
            next_state = self.state.test()
            
            if next_state is not None:
                self.state.deinit()
                self.state = next_state(self)
            
            
    def start_match(self):
        self.start_match = time.time()
    
    
    
class FSMState:
    def __init__(self, behavior):
        self.behavior = behavior
        self.robot = self.behavior.robot

    def test(self):
        raise NotImplementedError("Must be inherited")

    def deinit(self):
        raise NotImplementedError("Must be inherited")
    
    
    
class StateGoToPalet(FSMState):
    def __init__(self, behaviour):
        self.behavior = behaviour
        self.robot = self.behavior.robot
        
    def test(self):
        if(self.behavior.locomotion.move_finished()):
            return TestState2(self.behavior)
    
    def deinit(self):
        pass
    
    
    
class TestState(FSMState):
    
    def __init__(self,behaviour):
        super().__init__(behaviour)
        path = pf.polyline(2500, Point(0,0), Point(800,300), Point(1600,-200))
        self.behaviour.locomotion.add_path(path)
        
        
    def test(self):
        if self.behaviour.locomotion.move_finished():
            return TestState2(self.behaviour)
        
    def deinit(self):
        pass
    
    
    
class TestState2(FSMState):
    
    def __init__(self,behaviour):
        super().__init__(behaviour)
        self.behaviour.locomotion.add_turn(-90)
        
        
    def test(self):
        if self.behaviour.locomotion.move_finished():
            return TestState2(self.behaviour)
        
    def deinit(self):
        pass
    
    
    
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