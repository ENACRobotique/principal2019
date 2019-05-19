import matplotlib.pyplot as plt

class Map():
    def __init__(self):
        self.map_ox,self.map_oy = self.create_map()
        self.obstacles = []
    
    def get(self):
        ox = self.map_ox.copy()
        oy = self.map_oy.copy()
        for obstacle  in self.obstacles :
            for point in obstacle :
                ox.append(point.x)
                oy.append(point.y)
        return ox,oy
        
        
    def add_obstacles(self,obstacle):
        """
        Ajoute l'obstacle désiré
        obstacle : Liste de Point
        """
        self.obstacles.append(obstacle)
        
    def clear_obstacles(self):
        self.obstacles = []
    
    def create_map(self):
        """
        Map Edition 2019
        """
        ox, oy =[], []
        
        "Bords de la table"
        for i in range(0,2000,10) :
            ox.append(i)
            oy.append(0.0)
            
        for i in range(0,2000,10) :
            ox.append(i)
            oy.append(3000.0)    
            
        for i in range(0,3000,10) :
            ox.append(0.0)
            oy.append(i)
            
        for i in range(0,3000,10) :
            ox.append(2000.0)
            oy.append(i)
      
        "Portes atome x6"
        for i in range(450,1050,10) :
            ox.append(1540)
            oy.append(i)
            
        for i in range(1950,2550,10) :
            ox.append(1540)
            oy.append(i)
            
        "Montes Balance"
        for i in range(450,2550,10) :
            ox.append(1600)
            oy.append(i)
    
        "Fin de montée balance"
        for i in range(1600,2000,10) :
            ox.append(i)
            oy.append(1220)
            
        for i in range(1600,2000,10) :
            ox.append(i)
            oy.append(1770)
        
        "Barre séparation balance"
        for i in range(1340,2000,10) :
            ox.append(i)
            oy.append(1500)
        
        "Accelerateur de particule"
        for i in range(500,2500,10) :
            ox.append(20)
            oy.append(i)
        
        return ox,oy
    
    def BordDeMap(self):
        """
        Polygone définissant une zone aux bords de la map pour le calcul des points de passage critique du A*
        (i.e. là où le robot est vraiment proche des bords)
        """
        Bords_15cm=[(0,0),(200,0),(200,122.8),(160,122.8),(160,45),(154,45),(154.3,148),(134.3,148),(134.3,152),(154.3,152),(154.3,255),
               (160,255),(160,177.2),(200,177.2),(200,300),(0,300),(0,1),
               (15,1),(15,285),(185,285),(185,192.2),(175,192.2),(175,270),(139.3,270),(139.3,167),(119.3,167),(119.3,133),
               (139.3,133),(139.3,30),(175,30),(175,107.8),(185,107.8),(185,15),(16,15),(16,0.5)
               ]
        return Bords_15cm
    
    def InterieurDeMap(self):
        """
        Polygone définissant une zone aux à l'interieur de la map pour le calcul des points de passage critique du A*
        (i.e. là où le robot n'est pas proche des bords)
        """
        Interieur_20cm = [(200,200),(200,2800),(1343,2800),(1343,1720),(1143,1720),(1143,1280),(1343,1280),(1343,200)]
        return Interieur_20cm
    
    def plot_map(self,sx,sy,gx,gy):
        ox,oy = self.get()
        plt.plot(ox, oy, ".k")
        plt.plot(sx, sy, "xr")
        plt.plot(gx, gy, "xb")
        #plt.grid(True)
        #plt.axis("equal")
        #plt.show()
