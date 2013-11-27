from visual import *
from visual.graph import *
from random import uniform

BOX_SIZE = 1e3    # m
PARTICLES = 100
time = 0
dt = 0.01         # s

scene.visible = True

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
            if self != o:
                if mag(self.pos - o.pos) <= min(self.radius, o.radius):
                    # print "collision!"

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

mass = Mass()
objects = list()
i = 0
while i < PARTICLES:
    objects.append(Particle())
    i += 1

while True:
    mass.tick(objects, dt)
    for o in objects:
        o.tick(objects, dt)

    time += dt
    print time