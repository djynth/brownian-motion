#!/bin/env python2.7

from visual import *
from visual.graph import *
from random import uniform
import time

BOX_SIZE = 5e2      # distance from the origin to the edges of the box, in meters
PARTICLES = 100     # the number of particles in the simulation
dt = .25            # the timestep between ticks in seconds - smaller for more accuracy, larger to run more quickly
SLEEP = .001        # amount of time to spend idle each tick (if running in demo mode), in seconds
NUM_SIMS = 250      # the number of times to run the simulation (starting over each time)
SIM_TIME = 100      # how long each simulation should run, in seconds of simulation-time
DEMO = False        # toggle running the demo (show the window, run only one simulation)

d2 = False          # set to true to simulate in 2 dimensions (all z-fields are 0)
d1 = False          # set to true to simulate in 1 dimension (all y- and z-fields are 0)

scene.visible = False
scene.fullscreen = True
scene.visible = DEMO

if DEMO:
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
        return self.volume          # density of 1 kg/m^3

    @property
    def momentum(self):
        return self.mass * self.velocity
    @momentum.setter
    def momentum(self, value):
        self.velocity = value/self.mass

    def collide_edge(self):
        if abs(self.pos.x) > BOX_SIZE:
            self.pos.x *= -1
        if abs(self.pos.y) > BOX_SIZE:
            self.pos.y *= -1
        if abs(self.pos.z) > BOX_SIZE:
            self.pos.z *= -1

    def tick(self, objects, start, dt):
        self.pos += self.velocity * dt
        self.collide_edge()

        for i in range(PARTICLES+1 - start):
            o = objects[PARTICLES - i]

            # determine whether the two objects are colliding, using mag2 for
            #  performance
            tot_radius = self.radius + o.radius
            intersect_amount = tot_radius**2 - mag2(self.pos - o.pos)
            if intersect_amount > 1e-3:
                # the amount of overlap between the two colliding objects
                intersect_amount = tot_radius - mag(self.pos - o.pos)
                # the vector from self to o
                r = o.pos - self.pos

                # adjust the objects so they are no longer intersecting
                # if the radii of the objects are the same (i.e. two Particles)
                #  collided, each is adjusted by the same amount
                # otherwise, the larger object (i.e. the Mass) is held not
                #  adjusted, and the smaller object (i.e. the Particle) is moved
                #  to the edge of the larger object
                # this is done to preserve the position of the Mass, which is
                #  what we are attempting to measure
                self_adjust = .5
                o_adjust = .5

                if self.radius > o.radius:
                    self_adjust = 0
                    o_adjust = 1
                elif self.radius < o.radius:
                    self_adjust = 1
                    o_adjust = 0

                self.pos -= norm(r) * self_adjust*intersect_amount
                o.pos += norm(r) * o_adjust*intersect_amount

                # switch into the frame of reference of the other object
                frame = vector(o.velocity)

                self.velocity -= frame
                o.velocity -= frame

                p = proj(self.velocity, r)

                # calculate the new velocities for the two objects
                self.velocity = (self.velocity - p) + (p*(self.mass - o.mass)/(self.mass + o.mass))
                o.velocity = p*((2*self.mass)/(self.mass + o.mass))
                
                # switch back into the original frame of reference
                self.velocity += frame
                o.velocity += frame

class Particle(Object):
    RADIUS = 10         # radius of each Particle, in meters
    MAX_SPEED = 100     # the maximum magnitude of the velocity of a Particle when it is created, in meters/second
    MIN_SPEED = 15      # the minimum magnitude of the velocity of a Particle when it is created, in meters/second

    def __init__(self, objects):
        Object.__init__(self, color=color.yellow, radius=Particle.RADIUS,
            velocity=Particle.generate_velocity(),
            pos=Particle.generate_position(objects))

    @staticmethod
    def generate_velocity():
        return uniform(Particle.MIN_SPEED, Particle.MAX_SPEED) * norm(
            vector(uniform(-1, 1),
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
    RADIUS = 100        # radius of the Mass, in meters

    def __init__(self):
        Object.__init__(self, color=color.blue, radius=Mass.RADIUS)
        self.trace = curve(color=color.blue)
        self.velocity = vector(0, 0, 0)     # FIXME: why is this necessary?

    def collide_edge(self):
        if self.pos.x + self.radius > BOX_SIZE or self.pos.x - self.radius < -BOX_SIZE:
            self.pos.x = BOX_SIZE - self.radius if self.pos.x > 0 else -BOX_SIZE + self.radius
            self.velocity.x *= -1
        if self.pos.y + self.radius > BOX_SIZE or self.pos.y - self.radius < -BOX_SIZE:
            self.pos.y = BOX_SIZE - self.radius if self.pos.y > 0 else -BOX_SIZE + self.radius
            self.velocity.y *= -1
        if self.pos.z + self.radius > BOX_SIZE or self.pos.z - self.radius < -BOX_SIZE:
            self.pos.z = BOX_SIZE - self.radius if self.pos.z > 0 else -BOX_SIZE + self.radius
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
            time.sleep(SLEEP)

    return mag(mass.pos)

if DEMO:
    run_sim()
else:
    distances = list()
    for i in range(NUM_SIMS):
        print "running simulation " + `i+1` + "/" + `NUM_SIMS`
        distances.append(run_sim(SIM_TIME))

    avg = 0

    for d in distances:
        avg += d
    avg /= len(distances)

    sd = 0
    for d in distances:
        sd += (d-avg)**2
    sd /= len(distances)
    sd = sqrt(sd)

    datetime = time.strftime("%Y-%m-%d-%H:%M:%S", time.localtime())
    filename = "data/data-" + datetime + ".txt"
    f = open(filename, 'w')
    f.write("Brownian Motion Simulation\n")
    f.write(datetime + "\n\n")
    f.write("Simulation Parameters:\n")
    f.write("   times run: " + `NUM_SIMS` + "\n")
    f.write("   number of particles: " + `PARTICLES` + "\n")
    f.write("   simulation time (s): " + `SIM_TIME` + "\n")
    f.write("   dt (s): " + `dt` + "\n")
    f.write("   box size (m): " + `BOX_SIZE` + "\n")
    f.write("   mass radius: " + `Mass.RADIUS` + "\n")
    f.write("   particle radius: " + `Particle.RADIUS` + "\n")
    f.write("   particle speed interval: [" + `Particle.MIN_SPEED` + "," + `Particle.MAX_SPEED` + "]\n")
    f.write("\n")
    f.write("Average displacement: " + `avg` + "\n")
    f.write("Standard deviation: " + `sd` + "\n\n")
    f.write("Displacement data:\n")

    for d in distances:
        f.write(`d` + "\n")

    f.close()

    print "Output written to " + filename
