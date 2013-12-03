#!/bin/env python2.7

from visual import *
from visual.graph import *
from random import uniform
import time

BOX_SIZE = 5e2      # m
PARTICLES = 100
dt = .25            # s
SLEEP = 1           # amount of time to sleep each loop (if the scene is visible) in ms
NUM_SIMS = 1        # the number of times to run the simulation (starting over each time)
SIM_TIME = 100      # how long each simulation should run, in seconds of simulation-time

d2 = False          # set to true to simulate in 2 dimensions (all z-fields are 0)
d1 = False          # set to true to simulate in 1 dimension (all y- and z-fields are 0)

scene.visible = False
scene.fullscreen = True
scene.visible = False        # set to toggle the display

box_bottom = box(pos=(0, -BOX_SIZE, 0), length=2*BOX_SIZE, width=2*BOX_SIZE, height=0.01,       color=color.cyan, opacity=0.2)
box_top    = box(pos=(0,  BOX_SIZE, 0), length=2*BOX_SIZE, width=2*BOX_SIZE, height=0.01,       color=color.cyan, opacity=0.2)
box_left   = box(pos=(-BOX_SIZE, 0, 0), length=0.01,       width=2*BOX_SIZE, height=2*BOX_SIZE, color=color.cyan, opacity=0.2)
box_right  = box(pos=( BOX_SIZE, 0, 0), length=0.01,       width=2*BOX_SIZE, height=2*BOX_SIZE, color=color.cyan, opacity=0.2)
box_back   = box(pos=(0, 0, -BOX_SIZE), length=2*BOX_SIZE, width=0.01,       height=2*BOX_SIZE, color=color.cyan, opacity=0.2)

class Object(sphere):
    def __init__(self, pos=vector(0,0,0), radius=0, velocity=vector(0,0,0), color=color.white):
        sphere.__init__(self, color=color, radius=radius, pos=pos)
        self.velocity = velocity

    def __eq__(self, other):
        return self.pos == other.pos

    @property
    def velocity(self):
        return self._velocity
    @velocity.setter
    def velocity(self, value):
        self._velocity = value

    @property
    def volume(self):
        return 4/3*pi*self.radius**3
    
    @property
    def mass(self):
        return self.volume

    @property
    def momentum(self):
        return self.mass * self.velocity
    @momentum.setter
    def momentum(self, value):
        self.velocity = value/self.mass

    def collide_edge(self):
        if abs(self.pos.x) > BOX_SIZE:
            self.pos.x = -BOX_SIZE if self.pos.x > 0 else BOX_SIZE
        if abs(self.pos.y) > BOX_SIZE:
            self.pos.y = -BOX_SIZE if self.pos.y > 0 else BOX_SIZE
        if abs(self.pos.z) > BOX_SIZE:
            self.pos.z = -BOX_SIZE if self.pos.z > 0 else BOX_SIZE

    def tick(self, objects, start, dt):
        self.pos += self.velocity * dt
        self.collide_edge()

        for i in range(PARTICLES+1 - start):
            o = objects[PARTICLES - i]

            if self == o:
                continue

            tot_radius = self.radius + o.radius
            intersect_amount = tot_radius**2 - mag2(self.pos - o.pos)
            if intersect_amount > 1e-3:
                # move the objects so they are no longer intersecting
                # each object is adjusted by an amount proportional to its
                #  radius, a good approximation as the timestep gets small
                # note: the intersted_amount is a way to judge the accuracy of
                #  the simulation, if it is small, the simulation will be
                #  accurate, if it is large, the simulation could become
                #  inaccurate and the timestep should be decreased
                # from testing, the value (tot_radius - mag(self.pos - o.pos))
                #  is on the order of 1e-14, and so the bounds required for a
                #  collision should be adequate to consider each collision only
                #  once

                intersect_amount = tot_radius - mag(self.pos - o.pos)

                self_adjust = .5
                o_adjust = .5

                if self.radius > o.radius:
                    self_adjust = 0
                    o_adjust = 1
                elif self.radius < o.radius:
                    self_adjust = 1
                    o_adjust = 0

                self.pos -= norm(o.pos - self.pos) * self_adjust*intersect_amount
                o.pos -= norm(self.pos - o.pos) * o_adjust*intersect_amount

                frame = vector(o.velocity.x, o.velocity.y, o.velocity.z)

                self.velocity -= frame
                o.velocity -= frame

                v1 = vector(self.velocity.x, self.velocity.y, self.velocity.z)
                p = proj(v1, o.pos - self.pos)

                self.velocity = (v1 - p) + (p*(self.mass - o.mass)/(self.mass + o.mass))
                o.velocity = p*((2*self.mass)/(self.mass + o.mass))
                
                self.velocity += frame
                o.velocity += frame

                # self.pos += self.velocity * dt

class Particle(Object):
    RADIUS = 10
    MAX_SPEED = 100

    def __init__(self, objects):
        Object.__init__(self, color=color.yellow, radius=Particle.RADIUS, velocity=Particle.generate_velocity(), pos=Particle.generate_position(objects))

    @staticmethod
    def generate_velocity():
        return uniform(0, Particle.MAX_SPEED) * norm(vector(uniform(-1, 1),
            0 if d1 else uniform(-1, 1),
            0 if d2 or d1 else uniform(-1, 1)))

    @staticmethod
    def generate_position(objects):
        while True:
            candidate = vector(uniform(-BOX_SIZE, BOX_SIZE),
                               0 if d1 else uniform(-BOX_SIZE, BOX_SIZE),
                               0 if d1 or d2 else uniform(-BOX_SIZE, BOX_SIZE))

            for o in objects:
                if mag(candidate - o.pos) <= o.radius + Particle.RADIUS:
                    break
            else:
                return candidate

class Mass(Object):
    RADIUS = 100

    def __init__(self):
        Object.__init__(self, color=color.blue, radius=Mass.RADIUS)
        self.trace = curve(color=color.blue)
        self.velocity = vector(0, 0, 0)

    def collide_edge(self):
        if abs(self.pos.x) > BOX_SIZE:
            self.pos.x = BOX_SIZE if self.pos.x > 0 else -BOX_SIZE
            self.velocity.x *= -1
        if abs(self.pos.y) > BOX_SIZE:
            self.pos.y = BOX_SIZE if self.pos.y > 0 else -BOX_SIZE
            self.velocity.y *= -1
        if abs(self.pos.z) > BOX_SIZE:
            self.pos.z = BOX_SIZE if self.pos.z > 0 else -BOX_SIZE
            self.velocity.z *= -1

    def tick(self, objects, start, dt):
        Object.tick(self, objects, start, dt)
        self.trace.append(self.pos)

def run_sim(total_time=-1):
    objects = list()
    mass = Mass()
    objects.append(mass)
    for _ in range(PARTICLES):
        objects.append(Particle(objects))

    t = 0

    while total_time < 0 or t < total_time:
        for i in range(PARTICLES + 1):
            objects[i].tick(objects, i+1, dt)

        t += dt

        if scene.visible:
            time.sleep(.001*SLEEP)

        if scene.mouse.events and scene.mouse.getevent().click:
            break

    return mag(mass.pos)

distances = list()
for i in range(NUM_SIMS):
    print i
    distances.append(run_sim(SIM_TIME))

print distances
avg = 0

for d in distances:
    avg += d
avg /= len(distances)

sd = 0
for d in distances:
    sd += (d-avg)**2
sd /= len(distances)
sd = sqrt(sd)

print avg
print sd

datetime = time.strftime("%Y-%m-%d-%H:%M:%S", time.localtime())
filename = "data/data-" + datetime + ".txt"
f = open(filename, 'w')
f.write("Brownian Motion Simulation\n")
f.write(datetime + "\n")
f.write("Ran " + `NUM_SIMS` + " times, for " + `SIM_TIME` + " seconds of simulation-time\n")
f.write("Average displacement: " + `avg` + "\n")
f.write("Standard deviation: " + `sd` + "\n\n")
f.write("Displacement data:\n")

for d in distances:
    f.write(`d` + "\n")

print "Output written to " + filename