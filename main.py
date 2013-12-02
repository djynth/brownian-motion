#!/bin/env python2.7

from visual import *
from visual.graph import *
from random import uniform

BOX_SIZE = 1e3    # m
PARTICLES = 30
time = 0
dt = 1            # s

scene.visible = True

box_bottom = box(pos=(0,-BOX_SIZE,0), length=2*BOX_SIZE, width=2*BOX_SIZE, height=0.01, color=color.cyan, opacity=0.2)
box_top = box(pos=(0,BOX_SIZE,0), length=2*BOX_SIZE, width=2*BOX_SIZE, height=0.01, color=color.cyan, opacity=0.2)
box_left = box(pos=(-BOX_SIZE,0,0), length=0.01, width=2*BOX_SIZE, height=2*BOX_SIZE, color=color.cyan, opacity=0.2)
box_right = box(pos=(BOX_SIZE,0,0), length=0.01, width=2*BOX_SIZE, height=2*BOX_SIZE, color=color.cyan, opacity=0.2)
box_back = box(pos=(0,0,-BOX_SIZE), length=2*BOX_SIZE, width=0.01, height=2*BOX_SIZE, color=color.cyan, opacity=0.2)

class Object(sphere):
    def __init__(self, pos=vector(0,0,0), radius=0, velocity=vector(0,0,0), color=color.white):
        super(sphere, self).__init__(color=color, radius=radius, pos=pos)
        self.velocity = velocity
        print self.velocity

    def __eq__(self, other):
        return self.pos == other.pos

    def get_momentum(self):
        return self.get_mass * self.velocity

    def get_volume(self):
        return 4/3*pi*radius**3

    def get_mass(self):
        return 1*self.get_volume()

    def tick(self, objects, dt):
        self.pos += self.velocity * dt
        if self.pos.x > BOX_SIZE or self.pos.x < -BOX_SIZE:
            self.velocity.x = -self.velocity.x
        if self.pos.y > BOX_SIZE or self.pos.y < -BOX_SIZE:
            self.velocity.y = -self.velocity.y
        if self.pos.z > BOX_SIZE or self.pos.z < -BOX_SIZE:
            self.velocity.z = -self.velocity.z

        for o in objects:
            if self != o and mag(self.pos - o.pos) <= self.radius + o.radius:
                tot_radius = self.radius + o.radius

                # move the objects so they are no longer intersecting
                # each object is adjusted by an amount proportional to its
                #  radius, a good approximation as the timestep gets small
                # note: the intersted_amount is a way to judge the accuracy of
                #  the simulation, if it is small, the simulation will be
                #  accurate, if it is large, the simulation could become
                #  inaccurate and the timestep should be decreased

                interesct_amount = tot_radius - mag(self.pos - o.pos)
                self.pos -= norm(o.pos - self.pos) * (self.radius/tot_radius)*interesct_amount
                o.pos -= norm(self.pos - o.pos) * (o.radius/tot_radius)*interesct_amount

                # TODO: find their new velocities after the collision

                print "collision (amount: ", interesct_amount, ")"

class Particle(Object):
    def __init__(self):
        super(Object, self).__init__(color=color.yellow, radius=25)
        self.velocity = self.generate_velocity()
        self.pos = self.generate_position()

    def generate_velocity(self):
        return vector(uniform(0, 10), uniform(0, 10), uniform(0, 10))

    def generate_position(self):
        return vector(uniform(-BOX_SIZE, BOX_SIZE),
                      uniform(-BOX_SIZE, BOX_SIZE),
                      uniform(-BOX_SIZE, BOX_SIZE))

class Mass(Object):
    def __init__(self):
        super(Object, self).__init__(color=color.blue, radius=1e2)
        self.velocity = vector(0, 0, 0)

objects = list()
objects.append(Mass())
i = 0
while i < PARTICLES:
    objects.append(Particle())
    i += 1

while True:
    for o in objects:
        o.tick(objects, dt)

    time += dt
    print time
