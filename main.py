#!/bin/env python2.7

from visual import *
from visual.graph import *
from random import uniform
import time

BOX_SIZE = 1e3      # m
PARTICLES = 50
dt = 1              # s
t = 0               # s
MAX_TIME = 1e5      # s, set to negative to run forever
SLEEP = 100           # amount of time to sleep each loop (if the scene is visible) in ms

d2 = False          # set to true to simulate in 2 dimensions (all z-fields are 0)
d1 = False          # set to true to simulate in 1 dimension (all y- and z-fields are 0)

scene.visible = False
scene.fullscreen = True
scene.visible = True        # set to toggle the display

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

    def intersects(self, other):
        return mag(self.pos - other.pos) <= self.radius + other.radius

    def tick(self, objects, start, dt):
        self.pos += self.velocity * dt

        if abs(self.pos.x) > BOX_SIZE:
            self.pos.x = -BOX_SIZE if self.pos.x > 0 else BOX_SIZE
        if abs(self.pos.y) > BOX_SIZE:
            self.pos.y = -BOX_SIZE if self.pos.y > 0 else BOX_SIZE
        if abs(self.pos.z) > BOX_SIZE:
            self.pos.z = -BOX_SIZE if self.pos.z > 0 else BOX_SIZE

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

                self.pos -= norm(o.pos - self.pos) * (self.radius/tot_radius)*intersect_amount
                o.pos -= norm(self.pos - o.pos) * (o.radius/tot_radius)*intersect_amount

                frame = copy(o.velocity)

                self.velocity -= frame
                o.velocity -= frame

                v1 = copy(self.velocity)
                p = proj(v1, o.pos - self.pos)

                self.velocity = (v1 - p) + (p*(self.mass - o.mass)/(self.mass + o.mass))
                o.velocity = p*((2*self.mass)/(self.mass + o.mass))
                
                self.velocity += frame
                o.velocity += frame

                # self.pos += self.velocity * dt

class Particle(Object):
    RADIUS = 15
    MAX_SPEED = 5

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

    def tick(self, objects, start, dt):
        Object.tick(self, objects, start, dt)
        self.trace.append(self.pos)

objects = list()
mass = Mass()
objects.append(mass)
for _ in range(PARTICLES):
    objects.append(Particle(objects))

times = list()

while MAX_TIME < 0 or t < MAX_TIME:
    start = time.time()

    for i in range(PARTICLES + 1):
        objects[i].tick(objects, i+1, dt)

    times.append(time.time() - start)

    t += dt

    if scene.visible:
        time.sleep(.001*SLEEP)

    if scene.mouse.events and scene.mouse.getevent().click:
        break

avg_time = 0
for t in times:
    avg_time += t

if len(times) > 0:
    avg_time /= len(times)

print "Average Tick:", avg_time, "(" + `len(times)` + ")"
print "Final Displacement:", mag(mass.pos)

scene.visible = False