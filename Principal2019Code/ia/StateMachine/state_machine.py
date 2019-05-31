
from PurePursuit import path_factory as pf
from PurePursuit import pure_pursuit as pp
from PurePursuit.path_manager import Point

from time import time
import params as p
from Communication.communication import path
from math import pi

class Behavior():
    
    def __init__(self,robot):
        raise NotImplementedError("This is an abstract class")
        
    def loop(self):
        raise NotImplementedError("Must be overriden")
    
    
    
class FSMMatch(Behavior):
    def __init__(self,robot, locomotion, comm):
        self.robot = robot
        self.color = None
        self.starting = None
        self.locomotion = locomotion
        self.comm = comm
        self.state = StateTirette(self)
        
    def loop(self):
        
        if self.starting is not None and time() - self.starting >p.MATCH_DURATION:
            next_state = DeadState(self)
        else:
            next_state = self.state.test()
            
            if next_state is not None:
                self.state.deinit()
                self.state = next_state(self)
            
    def start_match(self):
        print("Début du match")
        self.starting = time()
    
    
    
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
        """
        #self.behavior.yellow_state_go_to_palet = pf.polyline(2500, Point(885,223), Point(1000,223),Point(1200,240), Point(1250,280), Point(1380,350), Point(1430,500), Point(1510,600),  Point(1510,900))
        #self.behavior.purple_state_go_to_palet = pf.polyline(2500, Point(885,2777),Point(1000,2777),Point(1200,2760), Point(1250,2720), Point(1350,2650), Point(1450,2500), Point(1510,2400),  Point(1510,2100))
        
        self.behavior.yellow_state_go_to_palet = pf.polyline(2500, Point(885,223), Point(1000,223),Point(1050,2226),Point(1100,230),Point(1200,240), Point(1225,260), Point(1250,280), Point(1280,295),Point(1315,315),Point(1350,335),Point(1380,350), Point(1405,425),Point(1430,500), Point(1470,550),Point(1510,600),  Point(1510,900))
        self.behavior.purple_state_go_to_palet = pf.polyline(2500, Point(885,2777),Point(1000,2777),Point(1200,2760), Point(1250,2720), Point(1350,2650), Point(1450,2500), Point(1510,2400),  Point(1510,2100))
        
        self.behavior.yellow_state_go_to_accelerator = pf.polyline(2500,Point(1530,600), Point(1400,1100), Point(1100,1200),Point(600,1000),Point(450,1100),Point(420,1200),Point(370,1400),Point(370,1800))
        self.behavior.purple_state_go_to_accelerator = pf.polyline(2500,Point(1530,2400), Point(1400,1900), Point(1100,1800),Point(600,2000),Point(450,1900),Point(420,1800),Point(370,1600),Point(370,1200))
        
        self.behavior.yellow_state_go_to_accelerator_BACKUP = pf.polyline(2500,Point(1510,1400), Point(750,1300),Point(600,1100))
        self.behavior.purple_state_go_to_accelerator_BACKUP = pf.polyline(2500,Point(1510,1600), Point(750,1700), Point(600,1900))  
        
        self.behavior.yellow_state_to_blue = pf.polyline(2500, Point(600,1100),Point(500,1400),Point(400,1500), Point(400,1750))
        self.behavior.purple_state_to_blue = pf.polyline(2500, Point(600,1900),Point(500,1600),Point(400,1500), Point(400,1250))

        
        self.behavior.yellow_state_go_to_other_palets = pf.polyline(1000,Point(1510,900), Point(1510,600))
        self.behavior.purple_state_go_to_other_palets = pf.polyline(1000,Point(1510,2100), Point(1510,2400))
        
        self.behavior.yellow_go_to_goldenium = pf.polyline(1000,Point(300,1800), Point(300,2200))
        self.behavior.purple_go_to_goldenium = pf.polyline(1000,Point(20,1000), Point(20,800))
        
        self.behavior.yellow_do_to_scale = pf.polyline(4000,Point(300,2200), Point(300,1300),Point(800,700),Point(1200,800),Point(1330,1050),Point(1330,1350))
        self.behavior.purple_do_to_scale = pf.polyline(4000,Point(20,800), Point(500,1700),Point(1000,2300),Point(1400,2200),Point(1530,1950),Point(1530,1650))
        
        self.behavior.yellow_go_to_balance = pf.polyline(1000,Point(1510,600),Point(1510,1400))
        self.behavior.purple_go_to_balance = pf.polyline(1000,Point(1510,2400),Point(1510,1600))
        """
        
        self.behavior.path_pure_pursuit_yellow_accelerator = pf.polyline(4000, Point(885,223),Point(885,450),Point(830,500),Point(450,1000),Point(300,1300),Point(200,1400),Point(200,1600))
        self.behavior.path_pure_pursuit_purple_accelerator = pf.polyline(4000, Point(885,2777),Point(885,2550),Point(830,2500),Point(450,2000),Point(300,1700),Point(200,1600),Point(200,1400))
        
        
        self.behavior.comm.sendEarDownMessage()
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
        
        if self.behavior.robot.getColor() == p.VIOLET:
            self.robot.update(885,2777,0,0,0)
            self.behavior.comm.sendPositionMessage()
        else:
            self.robot.update(885,223,90*pi/180,0,0)
            self.behavior.comm.sendPositionMessage()
        
        if self.behavior.robot.tiretteOn() == 0:
            print("C'est parti")
            self.behavior.start_match()
            self.behavior.color = self.behavior.robot.getColor()
            if self.behavior.color == p.VIOLET:
                print("C'est violet !")
                self.robot.update(885,2777,0,0,0)
                #self.robot.update(885,2777,0,0,0)
                self.behavior.comm.sendPositionMessage()
            else:
                print("C'est jaune")
                self.robot.update(885,223,0,0,0)
                #self.robot.update(885,223,0,0,0)
                self.behavior.comm.sendPositionMessage()
                print("Message du robot dans la tirette")
                print(self.robot)
            #return StateRotate
            return StateSortieTirette
            #return StateGoToAccelerator_2
        
    def deinit(self):
        pass
   
    
class StateSortieTirette(FSMState):
    
    def __init__(self, behavior):
        super().__init__(behavior)
        self.time = time()
        print("Sortie de tirette")
        #self.behavior.comm.sendEarUpMessage()
        
    def test(self):
        if time() - self.time >10:
            print("Fin Sortie Tirette")
            return StateGoToPaletPart_1
            #return StateGoToAccelerator_1
            #return AccelerateurPurePursuit_1
            #return DeadState
    
    def deinit(self):
        pass



class AccelerateurPurePursuit_1(FSMState):
    
    def __init__(self,behavior):
        super().__init__(behavior)
        print("--------------PURE PURSUIT 1----------------")
        self.deroutement = None
        if self.behavior.color == p.JAUNE:
            self.robot._lidarZone.set_zones_forward_without_leeway_high()
            self.behavior.locomotion.add_path(self.behavior.path_pure_pursuit_yellow_accelerator)
        else:
            self.robot._lidarZone.set_zones_backward_without_leeway_high()
            self.behavior.locomotion.add_path(self.behavior.path_pure_pursuit_purple_accelerator,False)

    def test(self):
        if self.behavior.locomotion.move_finished():
            print("fin du forward")
            return AccelerateurPurePursuit_2
            #return DeadState
        if self.behavior.locomotion.is_stopped:
            if self.deroutement is None:
                self.deroutement = time()
            else:
                if time() - self.deroutement > 5:
                    return StateDeroutement
                
        else:
            self.deroutement = None
    
    def deinit(self):
        pass


class AccelerateurPurePursuit_2(FSMState):
    
    def __init__(self,behavior):
        super().__init__(behavior)
        print("--------------PURE PURSUIT 2----------------")
        self.behavior.comm.sendDynTrompeOutMessage()
        self.time = time()

    def test(self):
        if time() - self.time > 1:
            print("fin du forward")
            return AccelerateurPurePursuit_3
            #return DeadState
    
    def deinit(self):
        pass



class AccelerateurPurePursuit_3(FSMState):
    
    def __init__(self,behavior):
        super().__init__(behavior)
        print("--------------PURE PURSUIT 3----------------")
        if self.behavior.color == p.JAUNE:
            self.behavior.locomotion.add_precise_path(150)
        else:
            self.behavior.locomotion.add_precise_path(-150,False)

    def test(self):
        if self.behavior.locomotion.move_finished():
            print("fin du forward")
            return AccelerateurPurePursuit_3_bis
            #return DeadState
    
    def deinit(self):
        pass
    

class AccelerateurPurePursuit_3_bis(FSMState):
    
    def __init__(self,behavior):
        super().__init__(behavior)
        print("--------------PURE PURSUIT 3 BIS----------------")
        self.behavior.comm.sendDynTrompeNeutralMessage()
        self.time = time()

    def test(self):
        if time() - self.time > 1:
            print("fin du forward")
            return AccelerateurPurePursuit_4
            #return DeadState
    
    def deinit(self):
        pass
    
    
class AccelerateurPurePursuit_4(FSMState):
    
    def __init__(self,behavior):
        super().__init__(behavior)
        print("--------------PURE PURSUIT 4----------------")
        if self.behavior.color == p.JAUNE:
            path = pf.polyline(2000,Point(self.robot.x, self.robot.y), Point(400,180),Point(300,2000),Point(250,2200))
            self.behavior.locomotion.add_path(path)
        else:
            self.path = pf.polyline(2000,Point(self.robot.x, self.robot.y), Point(400,180),Point(300,2000),Point(250,2200))
            behavior.locomotion.add_path(path,False)

    def test(self):
        if self.behavior.locomotion.move_finished():
            print("fin du forward")
            return AccelerateurPurePursuit_5
            #return DeadState
    
    def deinit(self):
        pass


class AccelerateurPurePursuit_5(FSMState):
    
    def __init__(self,behavior):
        super().__init__(behavior)
        print("--------------PURE PURSUIT 5----------------")
        self.behavior.comm.sendDynTrompeOutMessage()
        self.time = time()

    def test(self):
        if time() - self.time > 1:
            print("fin du forward")
            return AccelerateurPurePursuit_6
            #return DeadState
    
    def deinit(self):
        pass
    
    
class AccelerateurPurePursuit_6(FSMState):
    
    def __init__(self,behavior):
        super().__init__(behavior)
        print("--------------PURE PURSUIT 6----------------")
        self.behavior.comm.sendPumpActivateMessage()
        self.time = time()

    def test(self):
        if time() - self.time > 1:
            print("fin du forward")
            return AccelerateurPurePursuit_7
            #return DeadState
    
    def deinit(self):
        pass
    
    
class AccelerateurPurePursuit_7(FSMState):
    
    def __init__(self,behavior):
        super().__init__(behavior)
        print("--------------PURE PURSUIT 7----------------")
        self.behavior.comm.sendDynTrompeNeutralMessage()
        self.time = time()

    def test(self):
        if time() - self.time > 1:
            print("fin du forward")
            return DeadState
            #return DeadState
    
    def deinit(self):
        pass



class StateDeroutement(FSMState):
    
    def __init__(self, behavior):
        super().__init__(behavior)
        print("------------DEROUTEMENT 1-----------------")
        if self.behavior.color == p.JAUNE:
            print("c'est toujours jaune")
            self.robot._lidarZone.set_zones_backward_with_leeway_high()
            path = pf.polyline(2500, Point(self.robot.x, self.robot.y), Point(200,400))
            self.behavior.locomotion.add_path(path,False)
            #self.behavior.locomotion.add_turn(1.07)
        else:
            print("C'etait un piege")
            path = pf.polyline(2500, Point(self.robot.x, self.robot.y), Point(200,2600))
            self.behavior.locomotion.add_path(path)
            self.robot._lidarZone.set_zones_forward_with_leeway_high()
            #self.behavior.locomotion.add_path(self.behavior.purple_state_go_to_palet, False)
            #self.behavior.locomotion.add_turn(-1.07)
        
        
    def test(self):
        if self.behavior.locomotion.move_finished():
            print("fin du forward")
            return Deroutement2
            #return DeadState
    
    def deinit(self):
        pass


class Deroutement2(FSMState):

    def __init__(self, behavior):
        super().__init__(behavior)
        print("------------DEROUTEMENT 2-----------------")
        self.time = time()
        
        
    def test(self):
        if time() - self.time > 25:
            print("fin du forward")
            return Deroutement3
            #return DeadState
    
    def deinit(self):
        pass
    
    
class Deroutement3(FSMState):
    
    def __init__(self, behavior):
        super().__init__(behavior)
        print("------------DEROUTEMENT 3-----------------")
        if self.behavior.color == p.JAUNE:
            print("c'est toujours jaune")
            self.robot._lidarZone.set_zones_forward_with_leeway_high()
            path = pf.polyline(2500, Point(self.robot.x, self.robot.y), Point(450,1000),Point(300,1200),Point(200,1400),Point(200,1600))
            self.behavior.locomotion.add_path(path)
            #self.behavior.locomotion.add_turn(1.07)
        else:
            print("C'etait un piege")
            path = pf.polyline(2500, Point(self.robot.x, self.robot.y), Point(450,2000),Point(300,1800),Point(200,1600),Point(200,1400))
            self.behavior.locomotion.add_path(path, False)
            self.robot._lidarZone.set_zones_backward_with_leeway_high()
            #self.behavior.locomotion.add_path(self.behavior.purple_state_go_to_palet, False)
            #self.behavior.locomotion.add_turn(-1.07)
        
        
    def test(self):
        if self.behavior.locomotion.move_finished():
            print("fin du forward")
            return AccelerateurPurePursuit_2
            #return DeadState
    
    def deinit(self):
        pass

"""Trajectoire sans pure pursuit"""
class StateGoToAccelerator_1(FSMState):

    def __init__(self, behavior):
        super().__init__(behavior)
        print("------------ACCELERATOR1-----------------")
        if self.behavior.color == p.JAUNE:
            print("c'est toujours jaune")
            self.robot._lidarZone.set_zones_forward_with_leeway_high()
            #self.behavior.locomotion.add_path(pf.polyline(2500, Point(885,223), Point(1000,223),Point(1200,240), Point(1250,280), Point(1350,350), Point(1450,500), Point(1530,600),  Point(1530,900)))
            self.behavior.locomotion.add_forwarding(600)
            #self.behavior.locomotion.add_turn(1.07)
        else:
            print("C'etait un piege")
            self.robot._lidarZone.set_zones_backward_with_leeway_high()
            #self.behavior.locomotion.add_path(self.behavior.purple_state_go_to_palet, False)
            self.behavior.locomotion.add_forwarding(-600,False)
            #self.behavior.locomotion.add_turn(-1.07)
        
        
    def test(self):
        if self.behavior.locomotion.move_finished():
            print("fin du forward")
            return StateGoToAccelerator_2
            #return DeadState
    
    def deinit(self):
        pass
    
    
class StateGoToAccelerator_2(FSMState):

    def __init__(self, behavior):
        super().__init__(behavior)
        print("------------ACCELERATOR2-----------------")
        if self.behavior.color == p.JAUNE:
            print("c'est toujours jaune")
            #self.robot._lidarZone.set_zones_forward()
            #self.behavior.locomotion.add_path(pf.polyline(2500, Point(885,223), Point(1000,223),Point(1200,240), Point(1250,280), Point(1350,350), Point(1450,500), Point(1530,600),  Point(1530,900)))
            self.behavior.locomotion.add_turn(40)
            #self.behavior.locomotion.add_turn(1.07)
        else:
            print("C'etait un piege")
            #self.robot._lidarZone.set_zones_backward()
            #self.behavior.locomotion.add_path(self.behavior.purple_state_go_to_palet, False)
            self.behavior.locomotion.add_turn(-40)
            #self.behavior.locomotion.add_turn(-1.07)
        
        
    def test(self):
        if self.behavior.locomotion.move_finished():
            print("fin du forward")
            #return StateGoToAccelerator_3
            return StateGoToAccelerator_3
    
    def deinit(self):
        pass
    
    
#Variables de navigation et de pure pursuit
class StateGoToAccelerator_3(FSMState):

    def __init__(self, behavior):
        super().__init__(behavior)
        print("-----------ACCEL3-------------")
        if self.behavior.color == p.JAUNE:
            print("c'est toujours jaune")
            self.robot._lidarZone.set_zones_forward_without_leeway_low()
            #self.behavior.locomotion.add_path(pf.polyline(2500, Point(885,223), Point(1000,223),Point(1200,240), Point(1250,280), Point(1350,350), Point(1450,500), Point(1530,600),  Point(1530,900)))
            self.behavior.locomotion.add_forwarding(790)
            #self.behavior.locomotion.add_turn(1.07)
        else:
            print("C'etait un piege")
            self.robot._lidarZone.set_zones_backward_without_leeway_low()
            #self.behavior.locomotion.add_path(self.behavior.purple_state_go_to_palet, False)
            self.behavior.locomotion.add_forwarding(-790,False)
            #self.behavior.locomotion.add_turn(-1.07)
        
        
    def test(self):
        if self.behavior.locomotion.move_finished():
            print("fin du forward")
            return StateGoToAccelerator_4
            
#Variables de navigation et de pure pursuiteturn StateGoToAccelerator_4
            #return DeadState
    
    def deinit(self):
        pass
    
    
class StateGoToAccelerator_4(FSMState):

    def __init__(self, behavior):
        super().__init__(behavior)
        print("------------ACCELERATOR4-----------------")
        if self.behavior.color == p.JAUNE:
            print("c'est toujours jaune")
            #self.robot._lidarZone.set_zones_forward()
            #self.behavior.locomotion.add_path(pf.polyline(2500, Point(885,223), Point(1000,223),Point(1200,240), Point(1250,280), Point(1350,350), Point(1450,500), Point(1530,600),  Point(1530,900)))
            self.behavior.locomotion.add_turn(30)
            #self.behavior.locomotion.add_turn(1.07)
        else:
            print("C'etait un piege")
            #self.robot._lidarZone.set_zones_backward()
            #self.behavior.locomotion.add_path(self.behavior.purple_state_go_to_palet, False)
            self.behavior.locomotion.add_turn(-30)
#Variables de navigation et de pure pursuitf.behavior.locomotion.add_turn(0)
            #self.behavior.locomotion.add_turn(-1.07)
        
        
    def test(self):
        if self.behavior.locomotion.move_finished():
            print("fin du forward")
            return StateGoToAccelerator_5
            #return DeadState
    
    def deinit(self):
        pass
    
    
class StateGoToAccelerator_5(FSMState):

    def __init__(self, behavior):
        super().__init__(behavior)
        print("------------ACCELERATOR5-----------------")
        self.behavior.comm.sendDynTrompeOutMessage()
        self.time = time()
        
        
    def test(self):
        if time() - self.time >1:
            print("fin du forward")
            return StateGoToAccelerator_6
            #return DeadState
    
    def deinit(self):
        pass
    
    
class StateGoToAccelerator_6(FSMState):

    def __init__(self, behavior):
        super().__init__(behavior)
        print("------------ACCELERATOR6-----------------")
        if self.behavior.color == p.JAUNE:
            print("c'est toujours jaune")
            #self.behavior.locomotion.add_path(pf.polyline(2500, Point(885,223), Point(1000,223),Point(1200,240), Point(1250,280), Point(1350,350), Point(1450,500), Point(1530,600),  Point(1530,900)))
            self.behavior.locomotion.add_forwarding(150)
            #self.behavior.locomotion.add_turn(1.07)
        else:
            print("C'etait un piege")
            #self.behavior.locomotion.add_path(self.behavior.purple_state_go_to_palet, False)
            self.behavior.locomotion.add_forwarding(-150,False)
            #self.behavior.locomotion.add_turn(-1.07)
        
    def test(self):
        if self.behavior.locomotion.move_finished():
            print("fin du forward")
            return StateGoToAccelerator_4_bis
            #return DeadState
    
    def deinit(self):
        pass
    
    
class StateGoToAccelerator_4_bis(FSMState):

    def __init__(self, behavior):
        super().__init__(behavior)
        print("------------ACCELERATOR4-----------------")
        if self.behavior.color == p.JAUNE:
            print("c'est toujours jaune")
            #self.robot._lidarZone.set_zones_forward()
            #self.behavior.locomotion.add_path(pf.polyline(2500, Point(885,223), Point(1000,223),Point(1200,240), Point(1250,280), Point(1350,350), Point(1450,500), Point(1530,600),  Point(1530,900)))
            self.behavior.locomotion.add_turn(20)
            #self.behavior.locomotion.add_turn(1.07)
        else:
            print("C'etait un piege")
            #self.robot._lidarZone.set_zones_backward()
            #self.behavior.locomotion.add_path(self.behavior.purple_state_go_to_palet, False)
            self.behavior.locomotion.add_turn(-20)
#Variables de navigation et de pure pursuitf.behavior.locomotion.add_turn(0)
            #self.behavior.locomotion.add_turn(-1.07)
        
        
    def test(self):
        if self.behavior.locomotion.move_finished():
            print("fin du forward")
            return StateGoToAccelerator_6_bis
            #return DeadState
    
    def deinit(self):
        pass
    
    
class StateGoToAccelerator_6_bis(FSMState):

    def __init__(self, behavior):
        super().__init__(behavior)
        print("------------ACCELERATOR6-----------------")
        if self.behavior.color == p.JAUNE:
            print("c'est toujours jaune")
            #self.behavior.locomotion.add_path(pf.polyline(2500, Point(885,223), Point(1000,223),Point(1200,240), Point(1250,280), Point(1350,350), Point(1450,500), Point(1530,600),  Point(1530,900)))
            self.behavior.locomotion.add_forwarding(100)
            #self.behavior.locomotion.add_turn(1.07)
        else:
            print("C'etait un piege")
            #self.behavior.locomotion.add_path(self.behavior.purple_state_go_to_palet, False)
            self.behavior.locomotion.add_forwarding(-100,False)
            #self.behavior.locomotion.add_turn(-1.07)
        
    def test(self):
        if self.behavior.locomotion.move_finished():
            print("fin du forward")
            return StateGoToAccelerator_4_bis_bis
            #return DeadState
    
    def deinit(self):
        pass
    
    
class StateGoToAccelerator_4_bis_bis(FSMState):

    def __init__(self, behavior):
        super().__init__(behavior)
        print("------------ACCELERATOR4-----------------")
        if self.behavior.color == p.JAUNE:
            print("c'est toujours jaune")
            #self.robot._lidarZone.set_zones_forward()
            #self.behavior.locomotion.add_path(pf.polyline(2500, Point(885,223), Point(1000,223),Point(1200,240), Point(1250,280), Point(1350,350), Point(1450,500), Point(1530,600),  Point(1530,900)))
            self.behavior.locomotion.add_turn(10)
            #self.behavior.locomotion.add_turn(1.07)
        else:
            print("C'etait un piege")
            #self.robot._lidarZone.set_zones_backward()
            #self.behavior.locomotion.add_path(self.behavior.purple_state_go_to_palet, False)
            self.behavior.locomotion.add_turn(10)
#Variables de navigation et de pure pursuitf.behavior.locomotion.add_turn(0)
            #self.behavior.locomotion.add_turn(-1.07)
        
        
    def test(self):
        if self.behavior.locomotion.move_finished():
            print("fin du forward")
            return StateGoToAccelerator_6_bis_bis
            #return DeadState
    
    def deinit(self):
        pass
    
    
class StateGoToAccelerator_6_bis_bis(FSMState):

    def __init__(self, behavior):
        super().__init__(behavior)
        print("------------ACCELERATOR6-----------------")
        if self.behavior.color == p.JAUNE:
            print("c'est toujours jaune")
            #self.behavior.locomotion.add_path(pf.polyline(2500, Point(885,223), Point(1000,223),Point(1200,240), Point(1250,280), Point(1350,350), Point(1450,500), Point(1530,600),  Point(1530,900)))
            self.behavior.locomotion.add_forwarding(220)
            #self.behavior.locomotion.add_turn(1.07)
        else:
            print("C'etait un piege")
            #self.behavior.locomotion.add_path(self.behavior.purple_state_go_to_palet, False)
            self.behavior.locomotion.add_forwarding(-270,False)
            #self.behavior.locomotion.add_turn(-1.07)
        
    def test(self):
        if self.behavior.locomotion.move_finished():
            print("fin du forward")
            return StateGoToAccelerator_2nd_try_1
            #return DeadState
    
    def deinit(self):
        pass
    
    

class StateGoToAccelerator_2nd_try_1(FSMState):

    def __init__(self, behavior):
        super().__init__(behavior)
        print("------------ACCELERATOR4-----------------")
        if self.behavior.color == p.JAUNE:
            print("c'est toujours jaune")
            #self.robot._lidarZone.set_zones_forward()
            #self.behavior.locomotion.add_path(pf.polyline(2500, Point(885,223), Point(1000,223),Point(1200,240), Point(1250,280), Point(1350,350), Point(1450,500), Point(1530,600),  Point(1530,900)))
            self.behavior.locomotion.add_turn(0)
            #self.behavior.locomotion.add_turn(1.07)
        else:
            print("C'etait un piege")
            #self.robot._lidarZone.set_zones_backward()
            #self.behavior.locomotion.add_path(self.behavior.purple_state_go_to_palet, False)
            self.behavior.locomotion.add_turn(0)
#Variables de navigation et de pure pursuitf.behavior.locomotion.add_turn(0)
            #self.behavior.locomotion.add_turn(-1.07)
        
        
    def test(self):
        if self.behavior.locomotion.move_finished():
            print("fin du forward")
            return StateGoToAccelerator_2nd_try_2
            #return DeadState
    
    def deinit(self):
        pass
    
    
class StateGoToAccelerator_2nd_try_2(FSMState):

    def __init__(self, behavior):
        super().__init__(behavior)
        print("------------ACCELERATOR6-----------------")
        if self.behavior.color == p.JAUNE:
            print("c'est toujours jaune")
            self.robot._lidarZone.set_zones_backward_without_leeway_low()
            #self.behavior.locomotion.add_path(pf.polyline(2500, Point(885,223), Point(1000,223),Point(1200,240), Point(1250,280), Point(1350,350), Point(1450,500), Point(1530,600),  Point(1530,900)))
            self.behavior.locomotion.add_forwarding(-200,False)
            #self.behavior.locomotion.add_turn(1.07)
        else:
            print("C'etait un piege")
            self.robot._lidarZone.set_zones_forward_without_leeway_low()
            #self.behavior.locomotion.add_path(self.behavior.purple_state_go_to_palet, False)
            self.behavior.locomotion.add_forwarding(200)
            #self.behavior.locomotion.add_turn(-1.07)
        
    def test(self):
        if self.behavior.locomotion.move_finished():
            print("fin du forward")
            return StateGoToAccelerator_2nd_try_3
            #return DeadState
    
    def deinit(self):
        pass    


class StateGoToAccelerator_2nd_try_3(FSMState):

    def __init__(self, behavior):
        super().__init__(behavior)
        print("------------ACCELERATOR4-----------------")
        if self.behavior.color == p.JAUNE:
            print("c'est toujours jaune")
            #self.robot._lidarZone.set_zones_forward()
            #self.behavior.locomotion.add_path(pf.polyline(2500, Point(885,223), Point(1000,223),Point(1200,240), Point(1250,280), Point(1350,350), Point(1450,500), Point(1530,600),  Point(1530,900)))
            self.behavior.locomotion.add_turn(10)
            #self.behavior.locomotion.add_turn(1.07)
        else:
            print("C'etait un piege")
            #self.robot._lidarZone.set_zones_backward()
            #self.behavior.locomotion.add_path(self.behavior.purple_state_go_to_palet, False)
            self.behavior.locomotion.add_turn(10)
#Variables de navigation et de pure pursuitf.behavior.locomotion.add_turn(0)
            #self.behavior.locomotion.add_turn(-1.07)
        
        
    def test(self):
        if self.behavior.locomotion.move_finished():
            print("fin du forward")
            return StateGoToAccelerator_2nd_try_4
            #return DeadState
    
    def deinit(self):
        pass
    
    
class StateGoToAccelerator_2nd_try_4(FSMState):

    def __init__(self, behavior):
        super().__init__(behavior)
        print("------------ACCELERATOR6-----------------")
        if self.behavior.color == p.JAUNE:
            print("c'est toujours jaune")
            #self.behavior.locomotion.add_path(pf.polyline(2500, Point(885,223), Point(1000,223),Point(1200,240), Point(1250,280), Point(1350,350), Point(1450,500), Point(1530,600),  Point(1530,900)))
            self.behavior.locomotion.add_forwarding(200)
            #self.behavior.locomotion.add_turn(1.07)
        else:
            print("C'etait un piege")
            #self.behavior.locomotion.add_path(self.behavior.purple_state_go_to_palet, False)
            self.behavior.locomotion.add_forwarding(-200,False)
            #self.behavior.locomotion.add_turn(-1.07)
        
    def test(self):
        if self.behavior.locomotion.move_finished():
            print("fin du forward")
            return StateGoToAccelerator_3nd_try_1
            #return DeadState
    
    def deinit(self):
        pass   
    
    
class StateGoToAccelerator_3nd_try_1(FSMState):

    def __init__(self, behavior):
        super().__init__(behavior)
        print("------------ACCELERATOR4-----------------")
        if self.behavior.color == p.JAUNE:
            print("c'est toujours jaune")
            #self.robot._lidarZone.set_zones_forward()
            #self.behavior.locomotion.add_path(pf.polyline(2500, Point(885,223), Point(1000,223),Point(1200,240), Point(1250,280), Point(1350,350), Point(1450,500), Point(1530,600),  Point(1530,900)))
            self.behavior.locomotion.add_turn(0)
            #self.behavior.locomotion.add_turn(1.07)
        else:
            print("C'etait un piege")
            #self.robot._lidarZone.set_zones_backward()
            #self.behavior.locomotion.add_path(self.behavior.purple_state_go_to_palet, False)
            self.behavior.locomotion.add_turn(0)
#Variables de navigation et de pure pursuitf.behavior.locomotion.add_turn(0)
            #self.behavior.locomotion.add_turn(-1.07)
        
        
    def test(self):
        if self.behavior.locomotion.move_finished():
            print("fin du forward")
            return StateGoToAccelerator_3nd_try_2
            #return DeadState
    
    def deinit(self):
        pass
    
    
class StateGoToAccelerator_3nd_try_2(FSMState):

    def __init__(self, behavior):
        super().__init__(behavior)
        print("------------ACCELERATOR6-----------------")
        if self.behavior.color == p.JAUNE:
            print("c'est toujours jaune")
            self.robot._lidarZone.set_zones_backward_without_leeway_low()
            #self.behavior.locomotion.add_path(pf.polyline(2500, Point(885,223), Point(1000,223),Point(1200,240), Point(1250,280), Point(1350,350), Point(1450,500), Point(1530,600),  Point(1530,900)))
            self.behavior.locomotion.add_forwarding(-200,False)
            #self.behavior.locomotion.add_turn(1.07)
        else:
            print("C'etait un piege")
            self.robot._lidarZone.set_zones_forward_without_leeway_low()
            #self.behavior.locomotion.add_path(self.behavior.purple_state_go_to_palet, False)
            self.behavior.locomotion.add_forwarding(200)
            #self.behavior.locomotion.add_turn(-1.07)
        
    def test(self):
        if self.behavior.locomotion.move_finished():
            print("fin du forward")
            return StateGoToAccelerator_3nd_try_3
            #return DeadState
    
    def deinit(self):
        pass    


class StateGoToAccelerator_3nd_try_3(FSMState):

    def __init__(self, behavior):
        super().__init__(behavior)
        print("------------ACCELERATOR4-----------------")
        if self.behavior.color == p.JAUNE:
            print("c'est toujours jaune")
            #self.robot._lidarZone.set_zones_forward()
            #self.behavior.locomotion.add_path(pf.polyline(2500, Point(885,223), Point(1000,223),Point(1200,240), Point(1250,280), Point(1350,350), Point(1450,500), Point(1530,600),  Point(1530,900)))
            self.behavior.locomotion.add_turn(10)
            #self.behavior.locomotion.add_turn(1.07)
        else:
            print("C'etait un piege")
            #self.robot._lidarZone.set_zones_backward()
            #self.behavior.locomotion.add_path(self.behavior.purple_state_go_to_palet, False)
            self.behavior.locomotion.add_turn(10)
#Variables de navigation et de pure pursuitf.behavior.locomotion.add_turn(0)
            #self.behavior.locomotion.add_turn(-1.07)
        
        
    def test(self):
        if self.behavior.locomotion.move_finished():
            print("fin du forward")
            return StateGoToAccelerator_3nd_try_4
            #return DeadState
    
    def deinit(self):
        pass
    
    
class StateGoToAccelerator_3nd_try_4(FSMState):

    def __init__(self, behavior):
        super().__init__(behavior)
        print("------------ACCELERATOR6-----------------")
        if self.behavior.color == p.JAUNE:
            print("c'est toujours jaune")
            self.robot._lidarZone.set_zones_forward_without_leeway_low()
            #self.behavior.locomotion.add_path(pf.polyline(2500, Point(885,223), Point(1000,223),Point(1200,240), Point(1250,280), Point(1350,350), Point(1450,500), Point(1530,600),  Point(1530,900)))
            self.behavior.locomotion.add_forwarding(200)
            #self.behavior.locomotion.add_turn(1.07)
        else:
            print("C'etait un piege")
            self.robot._lidarZone.set_zones_backward_without_leeway_low()
            #self.behavior.locomotion.add_path(self.behavior.purple_state_go_to_palet, False)
            self.behavior.locomotion.add_forwarding(-200,False)
            #self.behavior.locomotion.add_turn(-1.07)
        
    def test(self):
        if self.behavior.locomotion.move_finished():
            print("fin du forward")
            return StateGoToAccelerator_8_bis
            #return DeadState
    
    def deinit(self):
        pass   
    
    
class StateGoToAccelerator_8_bis(FSMState):

    def __init__(self, behavior):
        super().__init__(behavior)
        print("------------ACCELERATOR6-----------------")
        self.behavior.comm.sendDynTrompeNeutralMessage()
        self.time = time()
        
    def test(self):
        if time() - self.time >0.5:
            print("fin du forward")
            return StateGoToAccelerator_7
            #return DeadState
    
    def deinit(self):
        pass
    
    
class StateGoToAccelerator_7(FSMState):

    def __init__(self, behavior):
        super().__init__(behavior)
        print("------------ACCELERATOR7-----------------")
        if self.behavior.color == p.JAUNE:
            print("c'est toujours jaune")
            #self.behavior.locomotion.add_path(pf.polyline(2500, Point(885,223), Point(1000,223),Point(1200,240), Point(1250,280), Point(1350,350), Point(1450,500), Point(1530,600),  Point(1530,900)))
            self.behavior.locomotion.add_turn(10)
            #self.behavior.locomotion.add_turn(1.07)
        else:
            print("C'etait un piege")
            #self.behavior.locomotion.add_path(self.behavior.purple_state_go_to_palet, False)
            self.behavior.locomotion.add_turn(-10)
            #self.behavior.locomotion.add_turn(-1.07)
        
        
    def test(self):
        if self.behavior.locomotion.move_finished():
            print("fin du forward")
            return StateGoToAccelerator_8
            #return DeadState
    
    def deinit(self):
        pass
    
    
class StateGoToAccelerator_8(FSMState):

    def __init__(self, behavior):
        super().__init__(behavior)
        print("------------ACCELERATOR8-----------------")
        if self.behavior.color == p.JAUNE:
            print("c'est toujours jaune")
            #self.behavior.locomotion.add_path(pf.polyline(2500, Point(885,223), Point(1000,223),Point(1200,240), Point(1250,280), Point(1350,350), Point(1450,500), Point(1530,600),  Point(1530,900)))
            self.behavior.locomotion.add_forwarding(100)
            #self.behavior.locomotion.add_turn(1.07)
        else:
            print("C'etait un piege")
            #self.behavior.locomotion.add_path(self.behavior.purple_state_go_to_palet, False)
            self.behavior.locomotion.add_forwarding(-100, False)
            #self.behavior.locomotion.add_turn(-1.07)
        
        
    def test(self):
        if self.behavior.locomotion.move_finished():
            print("fin du forward")
            return StateGoToAccelerator_10
            #return DeadState
    
    def deinit(self):
        pass
    
    
class StateGoToAccelerator_9(FSMState):

    def __init__(self, behavior):
        super().__init__(behavior)
        print("------------ACCELERATOR9-----------------")
        if self.behavior.color == p.JAUNE:
            print("c'est toujours jaune")
            #self.behavior.locomotion.add_path(pf.polyline(2500, Point(885,223), Point(1000,223),Point(1200,240), Point(1250,280), Point(1350,350), Point(1450,500), Point(1530,600),  Point(1530,900)))
            self.behavior.locomotion.add_turn(0)
            #self.behavior.locomotion.add_turn(1.07)
        else:
            print("C'etait un piege")
            #self.behavior.locomotion.add_path(self.behavior.purple_state_go_to_palet, False)
            self.behavior.locomotion.add_turn(0)
            #self.behavior.locomotion.add_turn(-1.07)
        
        
    def test(self):
        if self.behavior.locomotion.move_finished():
            print("fin du forward")
            return StateGoToAccelerator_10
            #return DeadState
    
    def deinit(self):
        pass
    
    
class StateGoToAccelerator_10(FSMState):

    def __init__(self, behavior):
        super().__init__(behavior)
        print("------------ACCELERATOR10-----------------")
        if self.behavior.color == p.JAUNE:
            print("c'est toujours jaune")
            #self.behavior.locomotion.add_path(pf.polyline(2500, Point(885,223), Point(1000,223),Point(1200,240), Point(1250,280), Point(1350,350), Point(1450,500), Point(1530,600),  Point(1530,900)))
            self.behavior.locomotion.add_forwarding(390)
            #self.behavior.locomotion.add_turn(1.07)
        else:
            print("C'etait un piege")
            #self.behavior.locomotion.add_path(self.behavior.purple_state_go_to_palet, False)
            self.behavior.locomotion.add_forwarding(-390,False)
            #self.behavior.locomotion.add_turn(-1.07)
        
        
    def test(self):
        if self.behavior.locomotion.move_finished():
            print("fin du forward")
            return StateGoToAccelerator_11
            #return DeadState
    
    def deinit(self):
        pass
    
    
class StateGoToAccelerator_11(FSMState):

    def __init__(self, behavior):
        super().__init__(behavior)
        print("------------ACCELERATOR11-----------------")
        self.behavior.comm.sendPumpActivateMessage()
        self.behavior.comm.sendDynTrompeOutMessage()
        self.time = time()
        
        
    def test(self):
        if time() - self.time >1.5:
            print("fin du forward")
            return StateGoToAccelerator_11_bis
            #return DeadState
    
    def deinit(self):
        pass
    
    
    
class StateGoToAccelerator_11_bis(FSMState):

    def __init__(self, behavior):
        super().__init__(behavior)
        print("------------ACCELERATOR11bis-----------------")
        self.behavior.comm.sendDynTrompeNeutralMessage()
        self.time = time()
        
        
    def test(self):
        if time() - self.time >0.1:
            self.behavior.comm.sendDynTrompeOutMessage()
            print("fin du forward")
            return StateGoToAccelerator_12
            #return DeadState
    
    def deinit(self):
        pass
    
class StateGoToAccelerator_12(FSMState):

    def __init__(self, behavior):
        super().__init__(behavior)
        print("------------ACCELERATOR12-----------------")
        self.behavior.comm.sendDynTrompeNeutralMessage()
        self.time = time()
        
        
    def test(self):
        if time() - self.time > 1.5:
            self.behavior.comm.sendPumpCancelMessage()
            print("fin du forward")
            return StateGoToBalance_1
            #return DeadState
    
    def deinit(self):
        pass
    
    

class StateGoToBalance_1(FSMState):
    
    def __init__(self, behavior):
        super().__init__(behavior)
        print("------------ACCELERATOR2-----------------")
        if self.behavior.color == p.JAUNE:
            print("c'est toujours jaune")
            #self.robot._lidarZone.set_zones_forward()
            #self.behavior.locomotion.add_path(pf.polyline(2500, Point(885,223), Point(1000,223),Point(1200,240), Point(1250,280), Point(1350,350), Point(1450,500), Point(1530,600),  Point(1530,900)))
            self.behavior.locomotion.add_turn(60)
            #self.behavior.locomotion.add_turn(1.07)
        else:
            print("C'etait un piege")
            #self.robot._lidarZone.set_zones_backward()
            #self.behavior.locomotion.add_path(self.behavior.purple_state_go_to_palet, False)
            self.behavior.locomotion.add_turn(-60)
            #self.behavior.locomotion.add_turn(-1.07)
        
        
    def test(self):
        if self.behavior.locomotion.move_finished():
            print("fin du forward")
            #return StateGoToAccelerator_3
            return StateGoToBalance_2
    
    def deinit(self):
        pass
    


class StateGoToBalance_2(FSMState):
    
    def __init__(self, behavior):
        super().__init__(behavior)
        print("------------BALANCE1-----------------")
        if self.behavior.color == p.JAUNE:
            print("c'est toujours jaune")
            self.robot._lidarZone.set_zones_backward_with_leeway_high()
            #self.robot._lidarZone.set_zones_forward()
            #self.behavior.locomotion.add_path(pf.polyline(2500, Point(885,223), Point(1000,223),Point(1200,240), Point(1250,280), Point(1350,350), Point(1450,500), Point(1530,600),  Point(1530,900)))
            self.behavior.locomotion.add_forwarding(-600,False)
            #self.behavior.locomotion.add_turn(1.07)
        else:
            print("C'etait un piege")
            #self.robot._lidarZone.set_zones_backward()
            #self.behavior.locomotion.add_path(self.behavior.purple_state_go_to_palet, False)
            self.robot._lidarZone.set_zones_forward_with_leeway_high()
            #self.robot._lidarZone.set_zones_forward()
            #self.behavior.locomotion.add_path(pf.polyline(2500, Point(885,223), Point(1000,223),Point(1200,240), Point(1250,280), Point(1350,350), Point(1450,500), Point(1530,600),  Point(1530,900)))
            self.behavior.locomotion.add_forwarding(600)
            #self.behavior.locomotion.add_turn(-1.07)
        
        
    def test(self):
        if self.behavior.locomotion.move_finished():
            print("fin du forward")
            #return StateGoToAccelerator_3
            return DeadState
    
    def deinit(self):
        pass






"""Ancienneces trajectoires non réussies"""






class StateGoToPaletPart_1(FSMState):
    
    def __init__(self, behavior):
        super().__init__(behavior)
        print("Go to palets")
        if self.behavior.color == p.JAUNE:
            print("c'est toujours jaune")
            self.robot._lidarZone.set_zones_forward()
            #self.behavior.locomotion.add_path(pf.polyline(2500, Point(885,223), Point(1000,223),Point(1200,240), Point(1250,280), Point(1350,350), Point(1450,500), Point(1530,600),  Point(1530,900)))
            self.behavior.locomotion.add_forwarding(670)
            #self.behavior.locomotion.add_turn(1.07)
        else:
            print("C'etait un piege")
            self.robot._lidarZone.set_zones_backward()
            #self.behavior.locomotion.add_path(self.behavior.purple_state_go_to_palet, False)
            self.behavior.locomotion.add_forwarding(670,False)
            #self.behavior.locomotion.add_turn(-1.07)
        
        
    def test(self):
        if self.behavior.locomotion.move_finished():
            print("fin du forward")
            return StateGoToPaletPart_2
            #return DeadState
    
    def deinit(self):
        pass
    
class StateGoToPaletPart_2(FSMState):
    
    def __init__(self, behavior):
        super().__init__(behavior)
        print("Turn Go to palets")
        if self.behavior.color == p.JAUNE:
            print("on tourne jaune")
            self.robot._lidarZone.set_zones_forward()
            #self.behavior.locomotion.add_path(pf.polyline(2500, Point(885,223), Point(1000,223),Point(1200,240), Point(1250,280), Point(1350,350), Point(1450,500), Point(1530,600),  Point(1530,900)))
            #self.behavior.locomotion.add_forwarding(670)
            self.behavior.locomotion.add_turn(90)
        else:
            print("on tourne violet")
            self.robot._lidarZone.set_zones_backward()
            #self.behavior.locomotion.add_path(self.behavior.purple_state_go_to_palet, False)
            #self.behavior.locomotion.add_forwarding(670,False)
            self.behavior.locomotion.add_turn(-90)
        
        
    def test(self):
        if self.behavior.locomotion.move_finished():
            print("fin du turn")
            return StateGoToPaletPart_3
            #return DeadState
    
    def deinit(self):
        pass
    
    
class StateGoToPaletPart_3(FSMState):
    
    def __init__(self, behavior):
        super().__init__(behavior)
        print("Go to palets")
        if self.behavior.color == p.JAUNE:
            print("c'est toujours jaune")
            self.robot._lidarZone.set_zones_forward()
            #self.behavior.locomotion.add_path(pf.polyline(2500, Point(885,223), Point(1000,223),Point(1200,240), Point(1250,280), Point(1350,350), Point(1450,500), Point(1530,600),  Point(1530,900)))
            self.behavior.locomotion.add_forwarding(611)
            #self.behavior.locomotion.add_turn(1.07)
        else:
            print("C'etait un piege")
            self.robot._lidarZone.set_zones_backward()
            #self.behavior.locomotion.add_path(self.behavior.purple_state_go_to_palet, False)
            self.behavior.locomotion.add_forwarding(611,False)
            #self.behavior.locomotion.add_turn(-1.07)
        
        
    def test(self):
        if self.behavior.locomotion.move_finished():
            print("fin du forward")
            return StateEarShake
            #return DeadState
    
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
        if time() - self.time > 0.8:
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
        if time() - self.time >1.5:
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
        if time() - self.time >1.5:
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
        print("Go to other palets")
        if self.behavior.color == p.JAUNE:
            print("c'est toujours jaune")
            self.robot._lidarZone.set_zones_backward()
            #self.behavior.locomotion.add_path(pf.polyline(2500, Point(885,223), Point(1000,223),Point(1200,240), Point(1250,280), Point(1350,350), Point(1450,500), Point(1530,600),  Point(1530,900)))
            self.behavior.locomotion.add_forwarding(-300,False)
            #self.behavior.locomotion.add_turn(1.07)
        else:
            print("C'etait un piege")
            self.robot._lidarZone.set_zones_forward()
            #self.behavior.locomotion.add_path(self.behavior.purple_state_go_to_palet, False)
            self.behavior.locomotion.add_forwarding(300)
            #self.behavior.locomotion.add_turn(-1.07)
        
        
    def test(self):
        if self.behavior.locomotion.move_finished():
            print("fin du backward vers les autres palets")
            return AnotherShake
            #return DeadState
    
    def deinit(self):
        pass
    
class AnotherShake(FSMState):
    def __init__(self, behavior):
        super().__init__(behavior)
        self.behavior.comm.sendEarUpMessage()
        self.time = time()
        print("ANOTHER SHAKE")
        self.shaking =0
        
    def test(self):
        if time() - self.time > 0.5:
            self.time = time()
            self.shaking+=1
            if self.shaking >= 7:
                return StateDegagement
            else:
                if self.shaking%2 == 1:
                    self.behavior.comm.sendEarDownMessage()
                else:
                    self.behavior.comm.sendEarUpMessage()
    
    def deinit(self):
        pass
    
class StateDegagement(FSMState):
    
    def __init__(self, behavior):
        super().__init__(behavior)
        print("Go to other palets")
        if self.behavior.color == p.JAUNE:
            print("c'est toujours jaune")
            self.robot._lidarZone.set_zones_backward()
            #self.behavior.locomotion.add_path(pf.polyline(2500, Point(885,223), Point(1000,223),Point(1200,240), Point(1250,280), Point(1350,350), Point(1450,500), Point(1530,600),  Point(1530,900)))
            self.behavior.locomotion.add_forwarding(-300,False)
            #self.behavior.locomotion.add_turn(1.07)
        else:
            print("C'etait un piege")
            self.robot._lidarZone.set_zones_forward()
            #self.behavior.locomotion.add_path(self.behavior.purple_state_go_to_palet, False)
            self.behavior.locomotion.add_forwarding(300)
            #self.behavior.locomotion.add_turn(-1.07)
        
        
    def test(self):
        if self.behavior.locomotion.move_finished():
            print("fin du backward vers les autres palets")
            return StateGoToBalancePart_1
            #return DeadState
    
    def deinit(self):
        pass  


class StateGoToBalancePart_1(FSMState):
    
    def __init__(self, behavior):
        super().__init__(behavior)
        print("1er Turn Go to balance")
        if self.behavior.color == p.JAUNE:
            print("on tourne jaune")
            self.robot._lidarZone.set_zones_forward()
            #self.behavior.locomotion.add_path(pf.polyline(2500, Point(885,223), Point(1000,223),Point(1200,240), Point(1250,280), Point(1350,350), Point(1450,500), Point(1530,600),  Point(1530,900)))
            #self.behavior.locomotion.add_forwarding(670)
            self.behavior.locomotion.add_turn(100)
        else:
            print("on tourne violet")
            self.robot._lidarZone.set_zones_backward()
            #self.behavior.locomotion.add_path(self.behavior.purple_state_go_to_palet, False)
            #self.behavior.locomotion.add_forwarding(670,False)
            self.behavior.locomotion.add_turn(-100)
        
        
    def test(self):
        if self.behavior.locomotion.move_finished():
            print("fin du 1er turn balance")
            return StateGoToBalancePart_2_1
            #return DeadState
    
    def deinit(self):
        pass
    
class StateGoToBalancePart_2_1(FSMState):
    
    def __init__(self, behavior):
        super().__init__(behavior)
        print("1er forward Go to balance")
        if self.behavior.color == p.JAUNE:
            print("on tourne jaune")
            self.robot._lidarZone.set_zones_forward()
            #self.behavior.locomotion.add_path(pf.polyline(2500, Point(885,223), Point(1000,223),Point(1200,240), Point(1250,280), Point(1350,350), Point(1450,500), Point(1530,600),  Point(1530,900)))
            self.behavior.locomotion.add_forwarding(1015)
            #self.behavior.locomotion.add_turn(15)
        else:
            print("on tourne violet")
            self.robot._lidarZone.set_zones_backward()
            #self.behavior.locomotion.add_path(self.behavior.purple_state_go_to_palet, False)
            self.behavior.locomotion.add_forwarding(1015,False)
            #self.behavior.locomotion.add_turn(-15)
        
        
    def test(self):
        if self.behavior.locomotion.move_finished():
            print("fin du 1er forward balance")
            return StateGoToBalancePart_3
            #return DeadState
    
    def deinit(self):
        pass

class StateGoToBalancePart_2_2(FSMState):
    
    def __init__(self, behavior):
        super().__init__(behavior)
        print("1er forward Go to balance")
        if self.behavior.color == p.JAUNE:
            print("on tourne jaune")
            self.robot._lidarZone.set_zones_forward()
            #self.behavior.locomotion.add_path(pf.polyline(2500, Point(885,223), Point(1000,223),Point(1200,240), Point(1250,280), Point(1350,350), Point(1450,500), Point(1530,600),  Point(1530,900)))
            self.behavior.locomotion.add_forwarding(515)
            #self.behavior.locomotion.add_turn(15)
        else:
            print("on tourne violet")
            self.robot._lidarZone.set_zones_backward()
            #self.behavior.locomotion.add_path(self.behavior.purple_state_go_to_palet, False)
            self.behavior.locomotion.add_forwarding(515,False)
            #self.behavior.locomotion.add_turn(-15)
        
        
    def test(self):
        if self.behavior.locomotion.move_finished():
            print("fin du 1er forward balance")
            return StateGoToBalancePart_3
            #return DeadState
    
    def deinit(self):
        pass

class StateGoToBalancePart_3(FSMState):
    
    def __init__(self, behavior):
        super().__init__(behavior)
        print("2eme Turn Go to balance")
        if self.behavior.color == p.JAUNE:
            print("on tourne jaune")
            self.robot._lidarZone.set_zones_forward()
            #self.behavior.locomotion.add_path(pf.polyline(2500, Point(885,223), Point(1000,223),Point(1200,240), Point(1250,280), Point(1350,350), Point(1450,500), Point(1530,600),  Point(1530,900)))
            #self.behavior.locomotion.add_forwarding(670)
            self.behavior.locomotion.add_turn(0)
        else:
            print("on tourne violet")
            self.robot._lidarZone.set_zones_backward()
            #self.behavior.locomotion.add_path(self.behavior.purple_state_go_to_palet, False)
            #self.behavior.locomotion.add_forwarding(670,False)
            self.behavior.locomotion.add_turn(0)
        
        
    def test(self):
        if self.behavior.locomotion.move_finished():
            print("fin du 2eme turn balance")
            return StateGoToBalancePart_4
            #return DeadState
    
    def deinit(self):
        pass
    
class StateGoToBalancePart_4(FSMState):
    
    def __init__(self, behavior):
        super().__init__(behavior)
        print("1er forward Go to balance")
        if self.behavior.color == p.JAUNE:
            print("on tourne jaune")
            self.robot._lidarZone.set_zones_forward()
            #self.behavior.locomotion.add_path(pf.polyline(2500, Point(885,223), Point(1000,223),Point(1200,240), Point(1250,280), Point(1350,350), Point(1450,500), Point(1530,600),  Point(1530,900)))
            self.behavior.locomotion.add_forwarding(176)
            #self.behavior.locomotion.add_turn(15)
        else:
            print("on tourne violet")
            self.robot._lidarZone.set_zones_forward()
            #self.behavior.locomotion.add_path(self.behavior.purple_state_go_to_palet, False)
            self.behavior.locomotion.add_forwarding(176)
            #self.behavior.locomotion.add_turn(-15)
        
        
    def test(self):
        if self.behavior.locomotion.move_finished():
            print("fin du 1er forward balance")
            #return StateGoToPaletPart_3
            return StateThrowPalets
    
    def deinit(self):
        pass
    
class StateTurnToBalance(FSMState):
    
    
    def __init__(self, behavior):
        super().__init__(behavior)
        print("Accelerator")
        self.behavior.locomotion.add_turn(0)
        
    def test(self):
        if self.behavior.locomotion.move_finished():
            return StateAdvanceToBalance
    
    def deinit(self):
        pass
    
    
    
class StateAdvanceToBalance(FSMState):
    
    
    def __init__(self, behavior):
        super().__init__(behavior)
        print("Accelerator")
        self.robot._lidarZone.set_zones_null()
        self.behavior.locomotion.add_precise_path(30)
        
    def test(self):
        if self.behavior.locomotion.move_finished():
            return StateThrowPalets
    
    def deinit(self):
        pass
    

class StateThrowPalets(FSMState):
    
    def __init__(self, behavior):
        print("Block palets")
        super().__init__(behavior)
        self.behavior.comm.sendLockerUpMessage()
        self.time = time()
        
        
    def test(self):
        if time() - self.time >1:
            return StateBringOtherPalets
        
        
    def deinit(self):
        pass
    
    
class StateBringOtherPalets(FSMState):
    
    def __init__(self, behavior):
        print("Block palets")
        super().__init__(behavior)
        self.behavior.comm.sendDynamicHolderUpMessage()
        self.time = time()
        
        
    def test(self):
        if time() - self.time >1:
            return StateFinishBalance
        
        
    def deinit(self):
        pass
    
    
class StateFinishBalance(FSMState):
    
    def __init__(self, behavior):
        print("Block palets")
        super().__init__(behavior)
        self.behavior.comm.sendHolderDownMessage()
        self.time = time()
        
        
    def test(self):
        if time() - self.time >1:
            return DeadState
        
        
    def deinit(self):
        pass
    
    
class StateLetPaeltsGo(FSMState):
    
    def __init__(self, behavior):
        print("Block palets")
        super().__init__(behavior)
        self.behavior.comm.sendDynHolderDownMessage()
        self.time = time()
        
        
    def test(self):
        if time() - self.time >3:
            return StateAccelerator
        
        
    def deinit(self):
        pass
    
    
class StateAccelerator(FSMState):
    
    def __init__(self, behavior):
        super().__init__(behavior)
        print("1er forward Go to balance")
        if self.behavior.color == p.JAUNE:
            print("on tourne jaune")
            self.robot._lidarZone.set_zones_backward()
            #self.behavior.locomotion.add_path(pf.polyline(2500, Point(885,223), Point(1000,223),Point(1200,240), Point(1250,280), Point(1350,350), Point(1450,500), Point(1530,600),  Point(1530,900)))
            self.behavior.locomotion.add_forwarding(-1300)
            #self.behavior.locomotion.add_turn(15)
        else:
            print("on tourne violet")
            self.robot._lidarZone.set_zones_backward()
            #self.behavior.locomotion.add_path(self.behavior.purple_state_go_to_palet, False)
            self.behavior.locomotion.add_forwarding(-1300)
            #self.behavior.locomotion.add_turn(-15)
        
        
    def test(self):
        if self.behavior.locomotion.move_finished():
            print("fin du 1er forward balance")
            #return StateGoToPaletPart_3
            return TurnAccelerator
    
    def deinit(self):
        pass
    
    
    
def TurnAccelerator(FSMState):
    
    def __init__(self, behavior):
        super().__init__(behavior)
        print("1er Turn Go to balance")
        if self.behavior.color == p.JAUNE:
            print("on tourne jaune")
            #self.robot._lidarZone.set_zones_forward()
            #self.behavior.locomotion.add_path(pf.polyline(2500, Point(885,223), Point(1000,223),Point(1200,240), Point(1250,280), Point(1350,350), Point(1450,500), Point(1530,600),  Point(1530,900)))
            #self.behavior.locomotion.add_forwarding(670)
            self.behavior.locomotion.add_turn(90)
        else:
            print("on tourne violet")
            #self.robot._lidarZone.set_zones_backward()
            #self.behavior.locomotion.add_path(self.behavior.purple_state_go_to_palet, False)
            #self.behavior.locomotion.add_forwarding(670,False)
            self.behavior.locomotion.add_turn(-90)
        
        
    def test(self):
        if self.behavior.locomotion.move_finished():
            print("fin du 1er turn balance")
            return StateGoToBalancePart_2_1
            #return DeadState
    
    def deinit(self):
        pass
    
    
    
class StateGoToAccelerator(FSMState):
    
    def __init__(self, behavior):
        super().__init__(behavior)
        print("Accelerator")
        self.robot._lidarZone.set_zones_backward()
        if self.behavior.color == p.JAUNE:
            self.behavior.locomotion.add_path(self.behavior.yellow_state_go_to_accelerator_BACKUP, False)
        else:
            self.behavior.locomotion.add_path(self.behavior.purple_state_go_to_accelerator_BACKUP, False)
        
    def test(self):
        if self.behavior.locomotion.move_finished():
            #return StateBlockPalets
            return ArmTheDesperateFinalWeapon
    
    def deinit(self):
        pass
    
    
class ArmTheDesperateFinalWeapon(FSMState):
    
    def __init__(self, behavior):
        print("Block palets")
        super().__init__(behavior)
        self.behavior.comm.sendDynTrompeOutMessage()
        self.time = time()
        
        
    def test(self):
        if time() - self.time >1:
            return StateToBlue
        
        
    def deinit(self):
        pass
    
    
class StateToBlue(FSMState):
    
    def __init__(self, behavior):
        super().__init__(behavior)
        print("Accelerator")
        if self.behavior.color == p.JAUNE:
            self.robot._lidarZone.set_zones_forward()
            self.behavior.locomotion.add_forwarding(900)
            #self.behavior.locomotion.add_path(self.behavior.yellow_state_to_blue)
        else:
            self.robot._lidarZone.set_zones_backward()
            self.behavior.locomotion.add_forwarding(-900)
            #self.behavior.locomotion.add_path(self.behavior.purple_state_to_blue, False)
        
    def test(self):
        if self.behavior.locomotion.move_finished():
            #return StateBlockPalets
            return DeadState
    
    def deinit(self):
        pass
    
    

class StateBlockPalets(FSMState):
    def __init__(self, behavior):
        print("Block palets")
        super().__init__(behavior)
        self.behavior.comm.sendEarClosedMessage()
        self.time = time()
        
        
    def test(self):
        if time() - self.time >1:
            return PumpPalets
        
        
    def deinit(self):
        pass
    
    
class PumpPalets(FSMState):
    
    def __init__(self, behavior):
        super().__init__(behavior)
        self.behavior.comm.sendDynTrompeInMessage()
        self.time = time()
        self.steps = 0
        
        
    def test(self):
        if time() - self.time >1.5:
            self.time = time()
            self.steps+=1
            if self.steps >= 25:
                self.behavior.comm.sendDynTrompeNeutralMessage()
                return StateGoToGOLDENIUM
            if self.steps%8 == 1:
                print("Etat 1")
                self.behavior.comm.sendDynTrompeInTrickMessage()
            if self.steps%8== 2:
                print("Etat 2")
                self.behavior.comm.sendDynTrompeInMessage()
            if self.steps%8 == 3:
                print("Etat 3")
                self.behavior.comm.sendPumpActivateMessage()
            if self.steps%8 == 4:
                print("Etat 4")
                self.behavior.comm.sendDynTrompeOutMessage()
            if self.steps%8 == 5:
                print("Etat 5")
                self.behavior.comm.sendPumpCancelMessage()
            if self.steps%8 == 6:
                print("Etat 6")
                self.behavior.comm.sendEarAlmostClosedMessage()
            if self.steps%8 == 7:
                print("Etat 7")
                self.behavior.comm.sendEarClosedMessage()
            if self.steps%8 == 0:
                print("Etat 8")
                self.behavior.comm.sendDynTrompeInMessage()
        
        
    def deinit(self):
        pass


class StateGoToGOLDENIUM(FSMState):
    
    def __init__(self, behavior):
        super().__init__(behavior)
        print("go to GOLDENIUUUUUUUUUUUUUUUUUM")
        if self.behavior.color == p.JAUNE:
            self.behavior.locomotion.add_path(self.behavior.yellow_go_to_goldenium)
        else:
            self.behavior.locomotion.add_path(self.behavior.purple_go_to_goldenium, False)
        
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
        if time() - self.time > 0.5:
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
        if time() - self.time > 0.5:
            return StateGoToScale
        
    def deinit(self):
        pass


class StateGoToScale(FSMState):
    
    def __init__(self, behavior):
        super().__init__(behavior)
        print("go to scale")
        if self.behavior.color == p.JAUNE:
            self.behavior.locomotion.add_path(self.behavior.yellow_do_to_scale,False)
        else:
            self.behavior.locomotion.add_path(self.behavior.purple_do_to_scale,False)
        
    def test(self):
        if self.behavior.locomotion.move_finished():
            return StateUnsuckGOLDENIUM
    def deinit(self):
        pass


class StateUnsuckGOLDENIUM(FSMState):

    def __init__(self, behavior):
        super().__init__(behavior)
        print("unsuck GOLDENIUUUUUUUUUUUUUUUUUM")
        self.behavior.comm.sendPumpCancelMessage()
        self.time = time()
        
    def test(self):
        if time() - self.time > 0.5:
            return DeadState
        
    def deinit(self):
        pass


class DeadState(FSMState):
    
    def __init__(self,behavior):
        print("------------------DEAD STATE------------------")
        super().__init__(behavior)
        self.behavior.locomotion.stop()
        
    def test(self):
        self.behavior.locomotion.stop()
    
    def deinit(self):
        pass
    
    

    
"""Etat d'homologation"""
class StateRotate(FSMState):
    
    def __init__(self, behavior):
        deg = 45
        super().__init__(behavior)
        print("ROTATE")
        self.behavior.locomotion.add_turn(deg)
        
        
    def test(self):
        if self.behavior.locomotion.move_finished():
            return DeadState
        
        
    def deinit(self):
        pass
    
class StateForward(FSMState):

    def __init__(self, behavior):
        distance = 200
        super().__init__(behavior)
        print("FORWARD")
        self.behavior.locomotion.add_forwarding(distance)
        
        
    def test(self):
        if self.behavior.locomotion.move_finished():
            return DeadState
        
        
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

    