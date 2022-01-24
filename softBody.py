import sys
import pygame
import math

# Soft-body simulation!
#
# Here I am tryiong to implement a pressure based soft-body simulation by applyi9ng a hybrid custom approach with
# the approach given in "How to implement a pressure soft body model" by Maciej Matyka.
# The model works by having a spring connected outer shell of nodes that have a simulated pressure force acting on
# them.
#
# Future improvements:
# 1. Neaten up the code - Use vectors instead?
# 2. Make obstacles.
# 3. Multiple interacting blobs.

pygame.init()
pygame.display.set_caption("Soft-body")

# Frame rate
FPS = 144
clock = pygame.time.Clock()

# Window size
size = WIDTH, HEIGHT = 800, 800
BORDER = 10

# Initialize screen
screen = pygame.display.set_mode(size)

# Colours
black = 0, 0, 0
white = 255, 255, 255
red = 255, 0, 0


# Main code -----
# Blob variables
NUMN = 150
R = WIDTH/8
PRESSURE = 140000
MASS = 1

# Spring variables
KS = 500
KD = 15

# Gravity
GX = 0
GY = 3

# Timestep
DT = 0.02

# Friction and dampening
AIRRES = 0.9995
COREST = 0.9
FRICT = 0.995

# Mouse force
MFORCE = 3


class Node():
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.vx = 0
        self.vy = 0
        self.fx = 0
        self.fy = 0

class Spring():
    def __init__(self, i, j, length, nx, ny):
        self.i = i
        self.j = j
        self.length = length
        self.nx = nx
        self.ny = ny

class Blob():
    def __init__(self):
        self.pos = [WIDTH/2, HEIGHT/2]
        self.r = R
        self.nodes = self.spawnNodes()
        self.springs = self.spawnSprings()
        self.pressure = 0

    def spawnNodes(self): # Anti-clockwise from y-axis
        nodes = []
        for i in range(NUMN):
            ang = i*2*math.pi/NUMN - math.pi/2
            x = self.r*math.cos(ang) + self.pos[0]
            y = self.r*math.sin(ang) + self.pos[1]

            nodes.append(Node(x, y))
        return nodes

    def calcVolume(self):
        volume = 0
        for i in range(1, NUMN + 1):
            i = i % NUMN - 1

            x0 = self.nodes[0].x
            y0 = self.nodes[0].y
            x1 = self.nodes[i].x
            y1 = self.nodes[i].y
            x2 = self.nodes[i+1].x
            y2 = self.nodes[i+1].y

            volume += areaTri([x0, y0], [x1, y1], [x2, y2])
        return volume

    def applyGravity(self):
        for node in self.nodes:
            if (self.pressure - PRESSURE) >= 0: # Wait until inflated
                node.fx = GX * MASS
                node.fy = GY * MASS
            else:
                node.fx = 0
                node.fy = 0

    def spawnSprings(self):
        springs = []
        for i in range(NUMN):
            j = (i+1) % NUMN

            length = math.sqrt((self.nodes[i].x - self.nodes[j].x)*(self.nodes[i].x - self.nodes[j].x) +
                                (self.nodes[i].y - self.nodes[j].y)*(self.nodes[i].y - self.nodes[j].y))

            nx =  (self.nodes[i].y - self.nodes[j].y) / length
            ny = -(self.nodes[i].x - self.nodes[j].x) / length

            springs.append(Spring(i, j, length, nx, ny))
        return springs

    def applySprings(self):
        for spring in self.springs:
            i = spring.i
            j = spring.j
            x1 = self.nodes[i].x
            y1 = self.nodes[i].y
            x2 = self.nodes[j].x
            y2 = self.nodes[j].y

            r = math.sqrt((x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2))

            if r != 0:
                vx = self.nodes[i].vx - self.nodes[j].vx
                vy = self.nodes[i].vy - self.nodes[j].vy

                f = (r - spring.length)*KS + (vx * (x1 - x2) + vy * (y1 - y2)) * KD / r # Hookes law with dampening

                fx = ((x1 - x2) / r) * f
                fy = ((y1 - y2) / r) * f

                self.nodes[i].fx -= fx
                self.nodes[i].fy -= fy
                self.nodes[j].fx += fx
                self.nodes[j].fy += fy

                #spring.nx =  (y1 - y2) / r # Makes the blob actually rotate
                #spring.ny = -(x1 - x2) / r # - More accurate but less satisfying with current muse control

    def applyPressure(self):
        volume = self.calcVolume()
        for spring in self.springs:
            i = spring.i
            j = spring.j
            x1 = self.nodes[i].x
            y1 = self.nodes[i].y
            x2 = self.nodes[j].x
            y2 = self.nodes[j].y

            r = math.sqrt((x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2))

            pf = self.pressure * r / volume

            self.nodes[i].fx -= spring.nx*pf
            self.nodes[i].fy -= spring.ny*pf
            self.nodes[j].fx -= spring.nx*pf
            self.nodes[j].fy -= spring.ny*pf

    def applyMouseF(self):
        if pygame.mouse.get_pressed()[0]:
            mx = pygame.mouse.get_pos()[0]
            my = pygame.mouse.get_pos()[1]
            dx = mx - self.nodes[0].x
            dy = my - self.nodes[0].y
            self.nodes[0].fx += MFORCE*MASS*dx
            self.nodes[0].fy += MFORCE*MASS*dy

    def update(self):
        self.applyGravity()
        self.applySprings()
        self.applyPressure()
        self.applyMouseF() # Click and drag

        for node in self.nodes:
            node.vx = node.vx + (node.fx / MASS) * DT
            node.vy = node.vy + (node.fy / MASS) * DT
            node.x = node.x + node.vx * DT
            node.y = node.y + node.vy * DT

            if node.x < BORDER: # Boundary conditions
                node.x = BORDER
                node.vy = node.vy * COREST
                node.vx = -node.vx * FRICT
            if node.x > WIDTH - BORDER:
                node.x = WIDTH - BORDER
                node.vy = node.vy * COREST
                node.vx = -node.vx * FRICT
            if node.y < BORDER:
                node.y = BORDER
                node.vx = node.vx * COREST
                node.vy = -node.vy * FRICT
            if node.y > HEIGHT - BORDER:
                node.y = HEIGHT - BORDER
                node.vx = node.vx * COREST
                node.vy = -node.vy * FRICT
            
            node.vx = node.vx*AIRRES
            node.vy = node.vy*AIRRES

        if self.pressure < PRESSURE: # Inflation stage
            self.pressure += PRESSURE / 3000

    def draw(self):
        #for node in self.nodes:
        #    pygame.draw.circle(screen, black, (node.x, node.y), 3)
        for i in range(1, NUMN + 1):
            i = i % NUMN - 1
            x1 = self.nodes[i].x
            y1 = self.nodes[i].y
            x2 = self.nodes[i+1].x
            y2 = self.nodes[i+1].y
            pygame.draw.line(screen, black, (x1, y1), (x2, y2), 3)
        if pygame.mouse.get_pressed()[0]:
            mx = pygame.mouse.get_pos()[0]
            my = pygame.mouse.get_pos()[1]
            pygame.draw.line(screen, red, (self.nodes[0].x, self.nodes[0].y), (mx, my))


def areaTri(a, b, c):
    return abs(0.5*(a[0]*(b[1]-c[1]) + b[0]*(c[1]-a[1]) + c[0]*(a[1]-b[1])))


# Run -----
blob = Blob()
while True:
    clock.tick(FPS)
    for event in pygame.event.get():
        if event.type == pygame.QUIT: sys.exit()

    screen.fill(white)
    blob.draw()
    for i in range(10):
        blob.update()


    pygame.display.flip()