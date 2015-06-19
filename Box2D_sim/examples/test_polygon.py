#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# C++ version Copyright (c) 2006-2007 Erin Catto http://www.box2d.org
# Python version by Ken Lauer / sirkne at gmail dot com
# 
# This software is provided 'as-is', without any express or implied
# warranty.  In no event will the authors be held liable for any damages
# arising from the use of this software.
# Permission is granted to anyone to use this software for any purpose,
# including commercial applications, and to alter it and redistribute it
# freely, subject to the following restrictions:
# 1. The origin of this software must not be misrepresented; you must not
# claim that you wrote the original software. If you use this software
# in a product, an acknowledgment in the product documentation would be
# appreciated but is not required.
# 2. Altered source versions must be plainly marked as such, and must not be
# misrepresented as being the original software.
# 3. This notice may not be removed or altered from any source distribution.

"""
test_polygon.py
Creates polygon objects and places them in a scene

Author: Jeff Mahler
"""

from framework import *
from test_Bridge import create_bridge
import math
import numpy as np

class Polygon:
    def __init__(self, vertices, transform=b2Transform(),
                 massDensity=0.1, friction=0.5, linearDamping=10, angularDamping=10, center=True):
        self.vertices_ = vertices
        self.transform_ = transform
        self.massDensity_ = massDensity
        self.friction_ = friction
        self.linearDamping_ = linearDamping
        self.angularDamping_ = angularDamping

        s = b2Separator()

        vert_array = np.array(self.vertices_)
        self.centroid_ = list(np.mean(vert_array, axis=0))
        if center:
            vert_array_cent = vert_array - self.centroid_
            self.vertices_ = vert_array_cent.tolist()

        self.polyShape_ = b2PolygonShape(vertices=self.vertices_)

    @property
    def shape(self):
        return self.polyShape_

    def centerVertices(self, world, dynamic):
        """ Center the polygon vertices at the center of mass for more intuitive pose control """
        localCenter = self.body_.localCenter
#        self.body_.position = self.body_.position - localCenter

        # center the vertex list
        newVertices = []
        for v in self.vertices_:
            newVertices.append((v[0] - localCenter[0], v[1] - localCenter[1]))
        self.vertices_ = newVertices
        self.polyShape_.vertices = self.vertices_
 
        # re-add to the world
        world.DestroyBody(self.body_)
        self.addToWorld(world, center=False, dynamic=dynamic)
       
    def addToWorld(self, world, center = False, dynamic=True):
        """ Add the polygon to the given world as a dynamic body """
        if dynamic:
            self.body_ = world.CreateDynamicBody(
                angle=self.transform_.angle,
                position=self.transform_.position,
                linearDamping=self.linearDamping_,
                angularDamping=self.angularDamping_,
                fixtures=b2FixtureDef(
                    shape=self.polyShape_,
                    density=self.massDensity_,
                    friction=self.friction_
                    )
                )
        else:
            self.body_ = world.CreateStaticBody(
                angle=self.transform_.angle,
                position=self.transform_.position,
                linearDamping=self.linearDamping_,
                angularDamping=self.angularDamping_,
                fixtures=b2FixtureDef(
                    shape=self.polyShape_,
                    density=self.massDensity_,
                    friction=self.friction_
                    )
                )

        if center:
            self.centerVertices(world, dynamic)
        
        return self.body_

class RobotArm:
    def __init__(self, transform, joint_angles):
        self.transform_ = transform
        self.joint_angles_ = joint_angles

        self.transform_ = b2Transform()
        self.transform_.angle = 0.0
        self.transform_.position = (0, 0)

        w = 2.0
        h = 5.0
        vertices = vertices=[(0,0),(w,0),(w,h),(0,h)]
        transform = b2Transform()
        transform.angle = 0.0
        transform.position = (0, 0)
        self.rootLink_ = Polygon(vertices, transform)

        vertices = vertices=[(0,0),(w,0),(w,h),(0,h)]
        transform = b2Transform()
        transform.angle = math.pi / 2.0
        transform.position = (3.6, 0)
        self.palmLink_ = Polygon(vertices, transform)

    def addToWorld(self, world):
        rootBody = self.rootLink_.addToWorld(world)
        palmBody = self.palmLink_.addToWorld(world)

        fixtures = []
        fixtures.extend(rootBody.fixtures)
        fixtures.extend(palmBody.fixtures)

        world.DestroyBody(rootBody)
        world.DestroyBody(palmBody)
        self.body_ = world.CreateDynamicBody(
                angle=self.transform_.angle,
                position=self.transform_.position,
                linearDamping=0.1,
                angularDamping=0.1,
                fixtures=fixtures
                )
        return self.body_

class PolygonDemo(Framework):
    name="PolygonDemo"
    description="Keys: none"
    hz=4
    zeta=0.7
    def __init__(self):
        super(PolygonDemo, self).__init__(gravity = (0,0))

        w = 4
        h = 4
        for i in range(0, 20, h):
            for j in range(-10, 10, w):
                vertices = vertices=[(0,0),(w,0),(w,h),(0,h)]
                transform = b2Transform()
                transform.angle = 0.0
                transform.position = (j, i)

                p = Polygon(vertices, transform)
                p.addToWorld(self.world)

        transform2 = b2Transform()
        #transform2.R = b2Mat22(0, 1, -1, 0)
        transform2.angle = 1.1
        transform2.position = (0, 8)

        p2 = Polygon(vertices, transform2) 
        #p2.addToWorld(self.world)

        #r = RobotArm(transform2, [])
        #r.addToWorld(self.world)

    def Keyboard(self, key):
        pass

    def Step(self, settings):
        super(PolygonDemo, self).Step(settings)
        self.Print("frequency = %g hz, damping ratio = %g" % (self.hz, self.zeta))

if __name__=="__main__":
     main(PolygonDemo)
