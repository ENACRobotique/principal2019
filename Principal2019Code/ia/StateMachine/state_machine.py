
from PurePursuit import path_factory as pf
from PurePursuit import pure_pursuit as pp
from PurePursuit.path_manager import Point

from time import time
import params as p
from asn1crypto._ffi import null
from Communication.communication import path

class Behavior():
    
    def __init__(self,robot):
        raise NotImplementedError("This is an abstract class")
        
    def loop(self):
        raise NotImplementedError("Must be overriden")
    
    
    
class FSMMatch(Behavior):
    def __init__(self,robot, locomotion, comm):
        self.robot = robot
        self.colour = None
        self.start_match = None
        self.time_now = None
        self.locomotion = locomotion
        self.comm = comm
        self.state = StateTirette(self)
        
    def loop(self):
        
        if self.start_match is not None and self.time_now - self.start_match >p.MATCH_DURATION:
            next_state = DeadState()
        else:
            next_state = self.state.test()
            
            if next_state is not None:
                self.state.deinit()
                self.state = next_state(self)
            
    def start_match(self):
        print("Début du match")
        self.start_match = time.time()
    
    
    
class FSMState:
    def __init__(self, behavior):
        self.behavior = behavior
        self.robot = self.behavior.robot

    def test(self):
        raise NotImplementedError("Must be inherited")

    def deinit(self):
        raise NotImplementedError("Must be inherited")
    
    
class StateTirette(FSMState):
    def __init__(self, behavior):
        super().__init__(behavior)
        
        self.behavior.yellow_state_go_to_palet = pf.polyline(2500, Point(885,223), Point(1000,223),Point(1200,240), Point(1250,280), Point(1350,350), Point(1450,500), Point(1530,600),  Point(1530,900)) 
        self.behavior.purple_state_go_to_palet = pf.polyline(2500, Point(885,2777),Point(1000,2777),Point(1200,2760), Point(1250,2720), Point(1350,2650), Point(1450,2500), Point(1530,2400),  Point(1530,2100))
        
        self.behavior.yellow_state_go_to_accelerator = pf.polyline(2500,Point(1530,900), Point(1400,1100), Point(1100,1200),Point(600,1300),Point(250,1400),Point(100,1500),Point(25,1700),Point(20,2000))
        self.behaviour.purple_state_go_to_accelerator = pf.polyline(2500,Point(1530,2100), Point(1400,1900), Point(1100,1800),Point(600,1700),Point(250,1600),Point(100,1500),Point(25,1300),Point(20,1000))
        
        self.behavior.yellow_state_go_to_accelerator = pf.polyline(1000,Point(), Point())
        self.behavior.purple_state_go_to_accelerator = pf.polyline(1000,Point(), Point())
        
        self.behavior.comm.sendEarUpMessage()
        self.behavior.comm.sendLockerDownMessage()
        self.behavior.comm.sendHolderDownMessage()
        self.behavior.comm.sendDynTrompeNeutralMessage()
        self.behavior.comm.sendPumpCancelMessage()
        self.time = time()
        
    def test(self):
        #print("On est dans la tirette")
        if time() - self.time >0.5:
            self.behavior.comm.sendDynamicHolderDownMessage()
        self.behavior.comm.sendColorQuestion()
        self.behavior.comm.sendTiretteQuestion()
        print("TIRETTE A L'ENDROIT OU ON VEUT {}".format(self.behavior.robot.tiretteOn()))
        if self.behavior.robot.tiretteOn() == 0:
            print("C'est parti")
            self.behavior.start_match()
            self.behavior.colour = self.behavior.robot.getColor()
            return StateGoToPalet
        
    def deinit(self):
        pass
    

class StateGoToPalet(FSMState):
    
    def __init__(self, behavior):
        super().__init__(behavior)
        if self.behavior.color == p.JAUNE:
            path = self.behavior.yellow_state_go_to_palet
        else:
            path = self.behavior.purple_state_go_to_palet
        self.behavior.locomotion.add_path(path)
        
        
    def test(self):
        if self.behavior.locomotion.move_finished():
            return StateEarShake
    
    def deinit(self):
        pass
    
    
    
class StateEarShake(FSMState):
    def __init__(self, behavior):
        super().__init__(behavior)
        self.behavior.comm.sendEarUpMessage()
        self.time = time()
        print("EAR SHAKE")
        self.shaking =0
        
    def test(self):
        if time() - self.time > 0.5:
            self.time = time()
            self.shaking+=1
            if self.shaking >= 7:
                return StateHolderUp
            else:
                if self.shaking%2 == 1:
                    self.behavior.comm.sendEarDownMessage()
                else:
                    self.behavior.comm.sendEarUpMessage()
    
    def deinit(self):
        pass
    
    
class StateHolderUp(FSMState):
    def __init__(self, behavior):
        super().__init__(behavior)
        self.behavior.comm.sendDynamicHolderUpMessage()
        self.time = time()
        
        
    def test(self):
        if time() - self.time >1:
            return StateUnlock
        
        
    def deinit(self):
        pass
        
        
class StateUnlock(FSMState):
    
    def __init__(self, behavior):
        super().__init__(behavior)
        print("UNLOCK")
        self.behavior.comm.sendHolderUpMessage()
        self.time = time()
        
        
    def test(self):
        if time() - self.time >1:
            return StateHolderDown
        
        
    def deinit(self):
        pass
        
        
class StateHolderDown(FSMState):
    
    def __init__(self, behavior):
        super().__init__(behavior)
        print("HOLDER DOWN")
        self.behavior.comm.sendDynamicHolderDownMessage()
        self.time = time()
        
        
    def test(self):
        if time() - self.time >0.5:
            return StateRelock
        
        
    def deinit(self):
        pass
    

class StateRelock(FSMState):
    
    def __init__(self, behavior):
        super().__init__(behavior)
        print("RELOCK")
        self.behavior.comm.sendHolderDownMessage()
        self.time = time()
        
        
    def test(self):
        if time() - self.time >0.2:
            return MoveToOtherPalets
        
        
    def deinit(self):
        pass    


class MoveToOtherPalets(FSMState):
    
    def __init__(self, behavior):
        super().__init__(behavior)
        print("Other palets")
        path = null
        self.behavior.locomotion.add_path(path,False)
        
    def test(self):
        if self.behavior.locomotion.move_finished():
            return AnotherShake
    
    def deinit(self):
        pass
    
    
class AnotherShake(FSMState):
    def __init__(self, behavior):
        super().__init__(behavior)
        self.behavior.comm.sendEarUpMessage()
        self.time = time()
        print("ANTOHER SHAKE")
        self.shaking =0
        
    def test(self):
        if time() - self.time > 0.5:
            self.time = time()
            self.shaking+=1
            if self.shaking >= 7:
                return StateGoToAccelerator
            else:
                if self.shaking%2 == 1:
                    self.behavior.comm.sendEarDownMessage()
                else:
                    self.behavior.comm.sendEarUpMessage()
    
    def deinit(self):
        pass
    
    
class StateGoToAccelerator(FSMState):
    
    def __init__(self, behavior):
        super().__init__(behavior)
        print("Accelerator")
        path = null
        self.behavior.locomotion.add_path(path,False)
        
    def test(self):
        if self.behavior.locomotion.move_finished():
            return StateBlockPalets
    
    def deinit(self):
        pass
    

class StateBlockPalets(FSMState):
    def __init__(self, behavior):
        super().__init__(behavior)
        self.behavior.comm.sendEarClosedMessage()()
        self.time = time()
        
        
    def test(self):
        if time() - self.time >1:
            return StateUnlock
        
        
    def deinit(self):
        pass
    
    
class PumpPalets(FSMState):
    
    def __init__(self, behavior):
        super().__init__(behavior)
        self.behavior.comm.sendDynTrompeInMessage()
        self.time = time()
        self.steps = 0
        
        
    def test(self):
        if time() - self.time >0.5:
            self.steps+=1
            if self.steps >= 13:
                self.behavior.comm.sendDynTrompeNeutralMessage()
                return StateGoToGOLDENIUM
            if self.steps%4 == 1:
                self.behavior.comm.sendPumpActivateMessage()
            if self.steps%4 == 2:
                self.behavior.comm.sendDynTrompeOutMessage()
                self.behavior.comm.sendPumpCancelMessage()
            if self.steps%4 == 0:
                self.behavior.comm.sendDynTrompeInMessage()
        
        
    def deinit(self):
        pass


class StateGoToGOLDENIUM(FSMState):
    
    def __init__(self, behavior):
        super().__init__(behavior)
        print("go to GOLDENIUUUUUUUUUUUUUUUUUM")
        path = null
        self.behavior.locomotion.add_path(path,False)
        
    def test(self):
        if self.behavior.locomotion.move_finished():
            return StateGrabGOLDENIUM
    def deinit(self):
        pass


class StateGrabGOLDENIUM(FSMState):
    
    def __init__(self, behavior):
        super().__init__(behavior)
        print("grab GOLDENIUUUUUUUUUUUUUUUUUM")
        self.behavior.comm.sendDynTrompeOutMessage()
        self.time = time()
        
    def test(self):
        if time() - self.time() > 0.5:
            return StateSuckGOLDENIUM
        
    def deinit(self):
        pass


class StateSuckGOLDENIUM(FSMState):
    
    def __init__(self, behavior):
        super().__init__(behavior)
        print("suck GOLDENIUUUUUUUUUUUUUUUUUM")
        self.behavior.comm.sendPumpActivateMessage()
        self.time = time()
        
    def test(self):
        if time() - self.time() > 0.5:
            return StateBlockPalets
        
    def deinit(self):
        pass


class StateGoToScale(FSMState):
    
    def __init__(self, behavior):
        super().__init__(behavior)
        print("go to scale")
        path = null
        self.behavior.locomotion.add_path(path,False)
        
    def test(self):
        if self.behavior.locomotion.move_finished():
            return StateGrabGOLDENIUM
    def deinit(self):
        pass





class DeadState(FSMState):
    
    def __init__(self,behavior):
        super().__init__(behavior)
        
    def test(self):
        pass
    
    def deinit(self):
        pass
    
    

    
"""Etat d'homologation"""
class StateRotate(FSMState):
    
    def __init__(self, behavior):
        super().__init__(behavior)
        print("ROTATE")
        self.behavior.locomotion.add_turn(-120)
        
        
    def test(self):
        if self.behavior.locomotion.move_finished():
            return StateUnlockPalets
        
        
    def deinit(self):
        pass
    
"""Etat d'homologation"""
class StateMoveAgain(FSMState):
    
    def __init__(self, behavior):
        super().__init__(behavior)
        print("MOVE")
        path = pf.polyline(2500, Point(1520,900), Point(735,223)) 
        self.behavior.locomotion.add_path(path, False)
        
        
    def test(self):
        if self.behavior.locomotion.move_finished():
            return StateRotate
        
        
    def deinit(self):
        pass
    
"""Etat d'homologation"""
class StateDesespoir(FSMState):
    def __init__(self, behavior):
        super().__init__(behavior)
        path = pf.polyline(7000, Point(885,223), Point(1000,400),Point(1300,600),Point(1000,1000), Point(500,800), Point(450,200)) 
        self.behavior.locomotion.add_path(path)
        
        
    def test(self):
        if self.behavior.locomotion.move_finished():
            return StateDesespoir2
    
    def deinit(self):
        pass
    
"""Etat d'homologation"""
class StateDesespoir2(FSMState):
    def __init__(self, behavior):
        super().__init__(behavior)
        path = pf.polyline(1000, Point(450,200), Point(450,1500)) 
        self.behavior.locomotion.add_path(path, False)
        
        
    def test(self):
        if self.behavior.locomotion.move_finished():
            return DeadState
    
    def deinit(self):
        pass

    