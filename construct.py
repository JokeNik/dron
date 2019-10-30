import numpy
from numpy import ndarray
import math
from collections import namedtuple
from math import cos, tan, sqrt
from app.model import Geometry

Pair = namedtuple("Pair", ["first", "second"])
EPS = 1e-9

class Construct(Geometry):

    base, zero = {"lat": 0, "lng": 0}, {"x": 0, "y": 0}
    dpx, dpy = 1, 1
    lx, ly, max_h = 100, 100, 0
    points, a, ans_points, all_points = [], [], [], []
    cur_ans = 0
    drone, result = {}, {}

    def get_dps(self, location):
        self.base = location
        phi = self.base["lat"] / 180 * 3.1415926
        self.dpx = 111.321*cos(phi) - 0.0094*cos(3*phi)
        self.dpy = 111.143

    def get_distance(self):
        self.a.clear()
        for i in range(0, len(self.points)):
            self.a.append({"x": (self.points[i]["lng"] - self.base["lng"])*self.dpx*1000, "y": (self.points[i]["lat"] - self.base["lat"])*self.dpy*1000, "id": i})

    def setDrone(self, drone):
        self.drone = drone

    def initialize(self):
        self.drone = self.drone
        self.dd = (2 * self.drone["height"] * tan(self.drone["angle"] / 360 * 3.1415926))
        self.dd *= (1 - self.drone["overlapping"])

    def make_ans(self):
        self.ans_points.clear()
        self.all_points.insert(0, self.zero)
        self.all_points.append(self.zero)
        for i in range(0, len(self.all_points)):
            self.ans_points.append({"lat": self.all_points[i]["y"] / 1000 / self.dpy + self.base["lat"], "lng": self.all_points[i]["x"] / 1000 / self.dpx + self.base["lng"]})

    def solve_good(self, points):
        a, n = self.Convex_hull(points, len(points))
        d = self.dd
        pt1, pt2 = self.findmax(a, n)
        aa, b, c = self.makeline(pt1, pt2)

        k = math.sqrt(aa * aa + b * b) * d;

        fl = 1
        j = 0
        ans = []
        ans.append(pt1)
        ans.append(pt2)
        sa = 2
        while (fl == 1):
            j = j - 1
            ass, sz1 = self.check(a, n, aa, b, c + j * k)
            if sz1 == 1:
                ans.append(ass[0])
                sa = sa + 1
                continue
            if sz1 == 0:
                break
            ax, ay = self.makevector(ans[sa - 1], ans[sa - 2])
            bx, by = self.makevector(ans[sa - 1], ass[0])
            cx, cy = self.makevector(ans[sa - 1], ass[1])
            cos1 = cs(ax, ay, bx, by);
            cos2 = cs(ax, ay, cx, cy);
            #print("kukarek")
            if (cos1 > cos2):
                ans.append(ass[1])
                ans.append(ass[0])
            else:
                ans.append(ass[0])
                ans.append(ass[1])
            sa = sa + 2


        fl = 1
        j = 0
        while (fl == 1):
            j = j + 1
            ass, sz1 = self.check(a, n, aa, b, c + j * k)
            #print(sz1)
            if sz1 == 1:
                ans.append(ass[0])
                sa = sa + 1
                continue
            if sz1 == 0:
                break
            ax, ay = self.makevector(ans[sa - 1], ans[sa - 2])
            bx, by = self.makevector(ans[sa - 1], ass[0])
            cx, cy = self.makevector(ans[sa - 1], ass[1])
            #print(ans[sa - 1]["x"], ans[sa - 1]["y"], sep = ' ')
            #print(ass[0]["x"], ass[0]["y"], sep = ' ')
            #print(ass[1]["x"], ass[1]["y"], sep = ' ')
            cos1 = self.cs(ax, ay, bx, by);
            cos2 = self.cs(ax, ay, cx, cy);
            t1 = cos1
            t2 = cos2
            if (cos1 > cos2):
                ans.append(ass[1])
                ans.append(ass[0])
            else:
                ans.append(ass[0])
                ans.append(ass[1])
            sa = sa + 2

        ans.append(ans[len(ans) - 1])
        print("Zdarova")
        print(ans)
        #for i in range(sa):
        #    print(ans[i]["x"], ans[i]["y"], sep = ' ')
        print("U menya vse!")
        return ans


    def rec(self, a):
        n = len(a)
        #print(n)
        if n == 3:
            a = self.solve_good(a)
            return a
        ind = -1
        for i in range(2, n):
            if self.cw(a[i - 2], a[i - 1], a[i]) == 0:
                ind  = i - 1
                break
        if self.cw(a[n - 2], a[n - 1], a[0]) == 0:
            ind = n - 1
        if self.cw(a[n - 1], a[0], a[1]) == 0:
            ind = 0
        #print(ind)
        if ind == -1:
            a = self.solve_good(a)
            return a
        for j in range(0, n):
            if j == ind or j == ind - 1 or j == ind + 1:
                continue
            if ind == n - 1 and j == 0:
                continue
            if ind == 0 and j == n - 1:
                continue
            print(a)
            if self.check1(a, ind, j):
                #print("Vse budet")
                #print(a)
                #print(j)
                b, c = self.make_ar(a, ind, j)
                #print(b)
                #print("wtf")
                #print(c)
                #print("flight")
                print(ind, j, sep = ' ')
                print('-----------------')
                print(len(b))
                print(len(c))
                print('-----------------')
                b = self.rec(b)
                c = self.rec(c)
                #print(b)
                #print("kukarek")
                #print(c)
                #print("end")
                if len(b) == 0:
                    a = c
                if len(c) == 0:
                    a = b
                if len(b) > 0 and len(c) > 0:
                    a = b + c
                return a
        return a

    def solve(self):
        self.get_distance()
        ans = []
        cur = []
        a = self.a
        pmin = 0
        for i in range(1, len(a)):
            if a[i]["x"] < a[pmin]["x"]:
                pmin = i
        b = []
        for i in range(pmin, len(a)):
            b.append(a[i])
        for i in range(0, pmin):
            b.append(a[i])
        a = b.copy()
        print(a)
        print("begin")
        ans = self.rec(a)
        print(ans)
        print(len(ans))
        self.all_points = ans
        self.make_ans()
        print(len(self.ans_points))
        self.result["path"] = self.ans_points
        self.result["ok"] = 1
        self.result["height"] = 1000
