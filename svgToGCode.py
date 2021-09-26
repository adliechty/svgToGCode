#This file imports an svg file generated from boxes.py online box generator.
#The tool is intended for laser cutters, and therefore only outputs 2d path information
#it does not include multiple passes or inserting of tabs
#
#With this generator you can directly go from boxes.py to g code without having to open
#and manually modify the files for cnc router use, which ends up being time consuming as
#importing into fusing, extruding, selecting paths, tool operation, etc takes time.
#
#This automates the process

from __future__ import division, print_function
from svgpathtools import Path, Line, QuadraticBezier, CubicBezier, Arc
from svgpathtools import svg2paths, wsvg, svg2paths2, polyline
import numpy as np
import math
import os
import re

#Imports to determine if a point is within a polygon
from shapely.geometry import Point, LineString, MultiPoint
from shapely.geometry.polygon import Polygon

from pygcode import GCodeRapidMove, GCodeFeedRate, GCodeLinearMove

import argparse
from copy import deepcopy



#####################################################################
#Arg Parser
#####################################################################
parser = argparse.ArgumentParser(description='Convert SVG path to G code for CNC Router')
parser.add_argument('--inputSvgFile', dest='inputSvgFile', type=str, nargs=1,
                    help='SVG Input file to convert to G Code')

parser.add_argument('--materialThickness', dest='materialThickness', type=float, nargs=1,
                    help='Depth to cut')
parser.add_argument('--depthBelowMaterial', dest='depthBelowMaterial', type=float, nargs=1,
                    help='Depth to cut below material to ensure cut all the way through')

parser.add_argument('--depthPerPass', dest='depthPerPass', type=float, nargs=1,
                    help='Depth to cut per pass')
parser.add_argument('--cutFeedRate', dest='cutFeedRate', type=float, nargs=1,
                    help='Feed rate of machine when cutting')
parser.add_argument('--safeHeight', dest='safeHeight', type=float, nargs=1,
                    help='Clearance Height for router for rapid traverse')
parser.add_argument('--tabHeight', dest='tabHeight', type=float, nargs=1,
                    help='Height for tabs')
parser.add_argument('--cutterDiameter', dest='cutterDiameter', type=float, nargs=1,
                    help='Diameter of the cutter')
parser.add_argument('--tabWidth', dest='tabWidth', type=float, nargs=1,
                    help='Width of the tabs at it\'s thinnest location')
args = parser.parse_args()

#####################################################################
#Helper functions
#####################################################################
def distanceXY(p1, p2):
  return ((p1.X - p2.X)**2 + (p1.Y - p2.Y)**2)**0.5

def isPointWithinPolygon(pointToTest, points):
  polygon = []
  #If path is just a line, then it is not a polygon, so the point can't be within it.
  if len(points) <= 2:
    return False
  for point in points:
    polygon.append((point.X, point.Y))
  polygon = Polygon(polygon)
  return polygon.contains(Point(pointToTest.X, pointToTest.Y))

def isPointWithinAnyPath(point, cncPaths):
  for cncPath in cncPaths:
    if isPointWithinPolygon(point, cncPath.points3D):
      return True
  return False

def lineOrCurveToPoints3D(lineOrCurve, pointsPerCurve):
  if isinstance(lineOrCurve,Line):
    #print(lineOrCurve)
    return [Point3D(lineOrCurve.bpoints()[0].real, lineOrCurve.bpoints()[0].imag), \
            Point3D(lineOrCurve.bpoints()[1].real, lineOrCurve.bpoints()[1].imag)]
  elif isinstance(lineOrCurve, CubicBezier):
    points3D = []
    for i in range(int(pointsPerCurve)):
      complexPoint = lineOrCurve.point(i / (pointsPerCurve - 1.0))
      points3D.append(Point3D(complexPoint.real, complexPoint.imag, None))
    return points3D

  else:
    print("unsuported type: " + str(lineOrCurve))
    quit()

def pathToPoints3D(path, pointsPerCurve):
  prevEnd = None
  points3D = []
  for lineOrCurve in path:
    curPoints3D = lineOrCurveToPoints3D(lineOrCurve, pointsPerCurve)
    #check that the last line ending starts the beginning of the next line.
    #print(curPoints3D)
    if prevEnd != None and distanceXY(curPoints3D[0], prevEnd) > 0.01:
      print(curPoints3D[0])
      print(prevEnd)
      print("A SVG path must be contiguous, one line ending and beginning on the next line.  Make a seperate path out of non contiguous lines")
      quit()
    elif prevEnd == None:
      #first line, store both beginning point and end point
      points3D.extend(curPoints3D)
    else:
      #add to point list except first one as it was verified to be same as ending of last
      points3D.extend(curPoints3D[1:])
    prevEnd = curPoints3D[-1]
  return points3D

def isPathClockWise(path):
  #Signed area is negative if clockwise, positive if counter-clockwise
  if signedArea(path) < 0:
    return True
  else:
    return False

def signedArea(path, pointsPerCurve):
  prevPoint = path[-1][0]
  signedArea = 0
  for line in path:
    if isinstance(line, Line):
      newPoint = line.bpoints()[0]
      signedArea += (prevPoint.real * newPoint.imag - newPoint.real * prevPoint.imag)
      #assume start of line is all needed and end of line is begginging of next start of line...May be bad assumption
      prevPoint = line.bpoints()[0]
    elif isinstance(line, CubicBezier):
      for i in range(int(pointsPerCurve)):
        newPoint = line.point(i / (pointsPerCurve - 1.0))
        signedArea += (prevPoint.real * newPoint.imag - newPoint.real * prevPoint.imag)
        prevPoint = newPoint
    return signedArea / 2.0

def findBoundingBox(path):
  bounds = [Point3D(math.inf, math.inf, math.inf), Point3D(-math.inf, -math.inf, -math.inf)]
  for line in path:
    points3D = lineOrCurveToPoints3D(line, 10)
    for point3D in points3D:
      bounds[0].X = min(bounds[0].X, point3D.X)
      bounds[0].Y = min(bounds[0].Y, point3D.Y)
      bounds[1].X = max(bounds[1].X, point3D.X)
      bounds[1].Y = max(bounds[1].Y, point3D.Y)
  return bounds

def distanceToLine(point, lineStart, lineEnd):
  #create shapley objects to perform calculation
  line = LineString([(lineStart.X, lineStart.Y), (lineEnd.X, lineEnd.Y)])
  p = Point(point.X, point.Y)
  return p.distance(line)

def lineIntersection(l1Start, l1End, l2Start, l2End):
  #create shapley objects to perform calculation
  l1 = LineString([(l1Start.X, l1Start.Y), (l1End.X, l1End.Y)])
  l2 = LineString([(l2Start.X, l2Start.Y), (l2End.X, l2End.Y)])
  intersection = l1.intersection(l2)
  if intersection:
    if isinstance(intersection, LineString):
      return Point3D(intersection.coords[0][0], intersection.coords[0][1])
    else:
      return Point3D(intersection.x, intersection.y)
  return None

def areBoundingBoxesAdjacentAlongX(bbox0, bbox1):
  #If any point of bounding box is within other it is adjacent
  #If test bounding box covers other bounding box, it is adjacent
  if (bbox0[0].Y  >= bbox1[0].Y and  \
     bbox0[0].Y   <= bbox1[1].Y) or  \
     (bbox0[1].Y  >= bbox1[0].Y and \
     bbox0[1].Y   <= bbox1[1].Y) or  \
     (bbox0[0].Y  <= bbox1[0].Y and \
      bbox0[1].Y  >= bbox1[1].Y):
    return True
  return False

def areBoundingBoxesAdjacentAlongY(bbox0, bbox1):
  #If any point of bounding box is within other it is adjacent
  #If test bounding box covers other bounding box, it is adjacent
  if (bbox0[0].X  >= bbox1[0].X and  \
     bbox0[0].X   <= bbox1[1].X) or  \
     (bbox0[1].X  >= bbox1[0].X and \
     bbox0[1].X   <= bbox1[1].X) or  \
     (bbox0[0].X  <= bbox1[0].X and \
      bbox0[1].X  >= bbox1[1].X):
    return True
  return False

def lineCircleIntersections(l1Start, l1End, cCenter, cDiameter):
  p = Point(cCenter.X, cCenter.Y)
  c = p.buffer(cDiameter / 2.0).boundary
  l = LineString([(l1Start.X,l1Start.Y), (l1End.X, l1End.Y)])
  i = c.intersection(l)
  coords = []
  if isinstance(i, Point):
    return [Point3D(i.coords[0][0], i.coords[0][1])]
  if isinstance(i, LineString) and len(i.coords) == 1:
    return [Point3D(i.coords[0][0], i.coords[0][1])]
  if isinstance(i, MultiPoint):
    return [Point3D(i.geoms[0].coords[0][0], i.geoms[0].coords[0][1]), \
            Point3D(i.geoms[1].coords[0][0], i.geoms[1].coords[0][1])]
  return coords

def hex_to_rgb(value):
  value = value.lstrip('#')
  lv = len(value)
  return tuple(int(value[i:i+lv//3], 16) for i in range(0, lv, lv//3))


class Point3D:
  def __init__(self, X, Y, Z = None):
    self.X = X
    self.Y = Y
    self.Z = Z
  def to2DComplex(self):
    return self.X + self.Y * 1j
  def distanceXY(self, point):
    return distanceXY(self, point)
  def __str__(self):
    return str("(" + str(self.X) + "," + str(self.Y) + "," + str(self.Z) + ")")
  def __repr__(self):
    return str("(" + str(self.X) + "," + str(self.Y) + "," + str(self.Z) + ")")
#####################################################################
#cncPath class
#####################################################################
class cncPathClass:
  #cncPath is either a hole or border for a contiguous cut
  #CNC class needs to know all the paths in file to know what is a hole and not.

  def __init__(self, path, color, distPerTab, tabWidth, pointsPerCurve, cutterDiameter):
    self.points3D = pathToPoints3D(path, pointsPerCurve)
    self.color = color
    self.pointIsTab = [False] * len(self.points3D)
    #A point is not a tab unless otherwise specified.  Tabs will consist of two adjacent points width of the tab
    self.isTab  = [False] * len(self.points3D)
    self.XtabLocations = []
    self.YtabLocations = []

    #Store information about the path for later use and quick access
    self.signedArea = signedArea(path, pointsPerCurve)
    self.isClockWise = self.signedArea > 0
    self.boundingBox = findBoundingBox(path)
    print("    BOUNDING BOX: " + str(self.boundingBox))
    self.height = self.boundingBox[1].Y - self.boundingBox[0].Y
    self.width = self.boundingBox[1].X - self.boundingBox[0].X
    self.idealXTabPositions = []
    self.idealYTabPositions = []
    self.distPerTab = distPerTab
    #This gets filled out later as we need knowledge of all paths to determine if something is a hole or border
    self.isBorder = None
    self.tabWidth = tabWidth
    self.cutterDiameter = cutterDiameter

  def isPointWithinAnyTab(self, point):
    for tab in self.XtabLocations:
      if distanceXY(tab, point) < self.tabWidth / 2.0 + self.cutterDiameter / 2.0:
        return True

    for tab in self.YtabLocations:
      if distanceXY(tab, point) < self.tabWidth / 2.0 + self.cutterDiameter / 2.0:
        return True

    return False

  def __str__(self):
    val = ""
    for point3D in self.points3D:
      val = val + "," + str(point3D)
    return val

  def lineTabIntersections(self, lineStart, lineEnd):
    #returns a list of lists containing intersections of a line within a single tab.
    #eg, a line could intersect 2 tabs, and therefore have a list of 2 elements, each containing a list of intersecting points within those two tabs
    #as a tab line can both enter and exit the tab
    allIntersections = []
    for tabs in [self.XtabLocations, self.YtabLocations]:
      for tab in tabs:
        intersections = lineCircleIntersections(lineStart, lineEnd, tab, self.tabWidth + self.cutterDiameter)
        if len(intersections) == 2:
          if distanceXY(lineStart, intersections[0]) < distanceXY(lineStart, intersections[1]):
            allIntersections.append(intersections)
          else:
            allIntersections.append([intersections[1], intersections[0]])
        if len(intersections) == 1:
          allIntersections.append(intersections)

    allIntersectionsSorted = []
    while len(allIntersections) > 0:
      smallestDistance = 10000000000000000
      j = 0
      for intersection in allIntersections:
        curDistance = distanceXY(lineStart, intersection[0])
        
        if curDistance < smallestDistance:
          smallestIndex = j
          smallestDistance = curDistance
        j = j + 1
      allIntersectionsSorted.append(allIntersections.pop(smallestIndex))
    return allIntersectionsSorted

  def cncPathToSvgPath(self):
    prevPoint = None
    path = Path()
    
    for point3D in self.points3D:
      if prevPoint == None:
        None
      else:
        path.append(Line(prevPoint.to2DComplex(), point3D.to2DComplex()))
      prevPoint = point3D
    
    #Convert list of lines to a svgPagh and return
    return path

  def isPointTabStart(self, index):
    if index == 0:
      #if first point is part of a tab, it is the start as long as it is not the end of a tab
      return self.pointIsTab[index] and not self.isPointTabEnd(index)
    #if tab is starting return True
    if self.pointIsTab[index] and not self.pointIsTab[index - 1]:
      return True

    #If point and previous point are too far away then consider this a new tab, else its the same tab
    return self.pointIsTab[index] and distanceXY(self.points3D[index], self.points3D[index - 1]) > (self.tabWidth + self.cutterDiameter) * 1.1

  def isPointTabEnd(self, index):
    #If last point in path, then consider it tab end as tabs don't get inserted across beginning and end of paths (check assumption)
    if index == len(self.points3D) - 1:
      return True

    if self.pointIsTab[index] and not self.pointIsTab[index + 1]:
      return True

    #if next point is farther away than tab distance then it must be a part of a new tab
    return self.pointIsTab[index] and distanceXY(self.points3D[index], self.points3D[index + 1]) > (self.tabWidth + self.cutterDiameter) * 1.1

  def cncTabsToPaths(self):
    prevPoint3D = self.points3D[0]
    path = Path()
    paths = []
    
    inTab = False
    prevIsTab = False
    for point3D, isTab in zip(self.points3D, self.pointIsTab):
      #Add a line if previous point and current is a tab and distance between two is closer than tab width and cutter diameter
      if prevIsTab and isTab and distanceXY(prevPoint3D, point3D) <= (self.tabWidth + self.cutterDiameter) * 1.1:
      #if prevIsTab and isTab:
        #print(distanceXY(prevPoint3D, point3D))
        #print(self.tabWidth + self.cutterDiameter)
        path.append(Line(prevPoint3D.to2DComplex(), point3D.to2DComplex()))
      #if ending current tab, then add that tab to current list, start a new path
      #Path ending if moving to a non tab point, or there are two consectutive tab points that are further than a tabWidth apart
      if (prevIsTab and not isTab) or (prevIsTab and isTab and distanceXY(prevPoint3D, point3D) > (self.tabWidth + self.cutterDiameter) * 1.1):
      #if (prevIsTab and not isTab):
        paths.append(path)
        path = Path()
      prevPoint3D = point3D
      prevIsTab = isTab

    #print("cncTabsPaths")
    #print(paths)
    
    return paths

  def DetermineIfBorder(self, allCnCPaths):
    #Determine if the CnCPath is a border by looking at if it is not within any other CnCPath

    #See if first point in cncPath is within any path, if so then it is a hole
    #Only checking one point assuming there are not overlapping paths
    #TODO:  could check this assumption possibly...But may slow program down too
    self.isBorder = not isPointWithinAnyPath(self.points3D[0], allCnCPaths)

  def SetConventionalMilling(self):
    #Must be run after a path is determined if it is a border or not
    #If path is a border then go counter clock wise for conventional milling
      #if path is a hole then go clock wise for conventional milling
      if (self.isBorder and self.isClockWise) or (not self.isBorder and not self.isClockWise):
        self.points3D = self.points3D[::-1]

  def SetClimbMilling(self):
    #Must be run after a path is determined if it is a border or not
    #If path is a border then go counter clock wise for conventional milling
    #if path is a hole then go clock wise for conventional milling
    if (self.isBorder and not self.isClockWise) or (not self.isBorder and self.isClockWise):
      self.points3D = self.points3D[::-1]
  
  def ModifyPointsFromTabLocations(self):
    #########################################################
    #Cycle through the points in the cncPath to add in points at tab border and indidate part of a tab
    #########################################################
    prevPoint3D = self.points3D[0]
    i = 0
    while i < len(self.points3D):
      point3D = self.points3D[i]
      allTabIntersections = self.lineTabIntersections(prevPoint3D, point3D)
      #itterate through list of intersections within a single tab
      for tabIntersections in allTabIntersections:
        if len(tabIntersections) == 2:
          #print("2 tabIntersections")
          #move to edge of tab
          self.pointIsTab.insert(i, True)
          self.points3D.insert(i, tabIntersections[0])
          i = i + 1
          #Move to outside of tab
          self.pointIsTab.insert(i, True)
          self.points3D.insert(i, tabIntersections[1])
          i = i + 1
        elif len(tabIntersections) == 1:
          #print("1 intersection")
          #move to edge of tab
          self.pointIsTab.insert(i, True)
          self.points3D.insert(i, tabIntersections[0])
          i = i + 1
        #If no tabIntersections with tab, just continue on like normal from point to point
      i = i + 1
      prevPoint3D = point3D
      #print(i)
    #print("END: length" + str(len(self.points3D)))

  def calculateIdealTabLocations(self):
    #Must be run after a path is determined if it is a border or not
    #Only add tabs on border paths, leave material in middle for holes, as they usually don't cause issues
    if not self.isBorder:
      return
 
    xSpan = (self.boundingBox[1].X - self.boundingBox[0].X)
    ySpan = (self.boundingBox[1].Y - self.boundingBox[0].Y)
    numXPositions = max(1, math.ceil(xSpan / self.distPerTab))
    numYPositions = max(1, math.ceil(ySpan / self.distPerTab))
    
    xDistancePerTab = xSpan / numXPositions
    yDistancePerTab = ySpan / numYPositions

    # 1 tab should go in middle (eg 6 in per tab and 6 inches long, should go at 3 inches)
    # 1 tab at 4 inches should still stay at middle
    # 2 tabs at 8 inches...4 in per tab.  Tabs go at 2 and 6
    XPositions = []
    YPositions = []
    for curPos in range(numXPositions):
        XPositions.append(self.boundingBox[0].X + (curPos + 0.5) * xDistancePerTab)
      
    for curPos in range(numYPositions):
      YPositions.append(self.boundingBox[0].Y + (curPos + 0.5) * yDistancePerTab)

    self.idealXTabPositions = XPositions
    self.idealYTabPositions = YPositions

  def addXTab(self, location):
    if self.color[1] != 0:
      return
    self.XtabLocations.append(location)
    #print("added Tab: " + str(location) + "NumTabs: " + str(len(self.XtabLocations)))

  def addYTab(self, location):
    if self.color[1] != 0:
      return
    self.YtabLocations.append(location)
    #print("************added Tab: " + str(location) + "NumTabs: " + str(len(self.YtabLocations)))
  
  def XTabOnMaxSide(self, l1):
    midY = (self.boundingBox[0].Y + self.boundingBox[1].Y) / 2.0
    #print("midY: " + str(midY) + " l1: " + str(l1) + " l2: " + str(l2))
    return l1.Y > midY

  def YTabOnMaxSide(self, l1):
    midX = (self.boundingBox[0].X + self.boundingBox[1].X) / 2.0
    return l1.X > midX

  def XTabsOnSameSide(self, l1, l2):
    midY = (self.boundingBox[0].Y + self.boundingBox[1].Y) / 2.0
    #print("midY: " + str(midY) + " l1: " + str(l1) + " l2: " + str(l2))
    #print("l1: " + str(l1) + " l2: " + str(l2))
    #print("  midY: " + str(midY) + " boundingBox: " + str(self.boundingBox))
    return ((l1.Y > midY and l2.Y > midY) or \
           (l1.Y < midY and l2.Y < midY))
  def YTabsOnSameSide(self, l1, l2):
    midX = (self.boundingBox[0].X + self.boundingBox[1].X) / 2.0
    return ((l1.X > midX and l2.X > midX) or \
           (l1.X < midX and l2.X < midX))

  def addXTabIfNoneAlready(self, location):
    alreadyTab = False
    for XtabLocation in self.XtabLocations:
      midY = (self.boundingBox[0].Y + self.boundingBox[1].Y) / 2.0
      #if proposed tab and current tab are on same side of object and the distance to the current tab is less than tab distance over 2
      #Then indicate the tab already exists.
      if self.XTabsOnSameSide(location, XtabLocation) and \
         abs(XtabLocation.X - location.X) < self.distPerTab / 2.0:
        #print("alreadyTab at  " + str(XtabLocation) + " proposed: " + str(location))
        alreadyTab = True
        break
    
    if not alreadyTab:
      self.addXTab(location)

  def addYTabIfNoneAlready(self, location):
    alreadyTab = False
    for YtabLocation in self.YtabLocations:
      midX = (self.boundingBox[0].X + self.boundingBox[1].X) / 2.0
      #if proposed tab on same side of part as curretn tab and close enough then flag already a tab
      if self.YTabsOnSameSide(location, YtabLocation) and\
         abs(YtabLocation.Y - location.Y) < self.distPerTab / 2.0:
        alreadyTab = True
        break
    
    if not alreadyTab:
      self.addYTab(location)


  def addYTabsIfAdjacentTab(self, cncPaths):
    #print()
    #print()
    #print()
    #print(str(self.boundingBox))
    for cncPath in cncPaths:
      #Only check for adjacent tabs for other paths than self
      if cncPath == self or cncPath.isBorder == False:
        continue
      #if the bounding boxes are not adjacent to each other or adjacent but farther than an inch away
      if not areBoundingBoxesAdjacentAlongX(cncPath.boundingBox, self.boundingBox) or \
         (abs(cncPath.boundingBox[0].X - self.boundingBox[0].X) > 25.4 and      \
         abs(cncPath.boundingBox[0].X - self.boundingBox[1].X) > 25.4 and       \
         abs(cncPath.boundingBox[1].X - self.boundingBox[0].X) > 25.4 and       \
         abs(cncPath.boundingBox[1].X - self.boundingBox[1].X) > 25.4):
        #print("   Not Adjacent " + str(cncPath.boundingBox))
        continue
      #print("   Adjacent " + str(cncPath.boundingBox))
      for YtabLocation in cncPath.YtabLocations:
        #print(YtabLocation)
        #####################################################
        #Find the closest point to this tab on the path
        #####################################################
        prevPoint = self.points3D[-1]
        smallestDist = self.distPerTab
        candidateTabLocation = 0
        #print((YtabLocation - self.distPerTab * 1j / 2))
        #print((YtabLocation))
        #print((YtabLocation + self.distPerTab * 1j / 2))
        for point in self.points3D:
          #See if a vertical line of possible adjacent shape intersects curernt path anywhere
          #If so add a tab to current shape
          intersection = lineIntersection(Point3D(YtabLocation.X - self.tabWidth * 4.0, YtabLocation.Y) ,\
                                          Point3D(YtabLocation.X + self.tabWidth * 4.0, YtabLocation.Y), \
                                          prevPoint, point)
          if intersection:
            #print("    Adjacent tab added" +str(intersection) )
            self.addYTabIfNoneAlready(intersection)
            break
          prevPoint = point

  def addXTabsIfAdjacentTab(self, cncPaths):
    for cncPath in cncPaths:
      #Only check for adjacent tabs for other paths than self
      if cncPath == self or cncPath.isBorder == False:
        continue
      #if the bounding boxes are adjacent to each other
      if not areBoundingBoxesAdjacentAlongY(cncPath.boundingBox, self.boundingBox) or \
         (abs(cncPath.boundingBox[0].Y - self.boundingBox[0].Y) > 25.4 and      \
         abs(cncPath.boundingBox[0].Y - self.boundingBox[1].Y) > 25.4 and       \
         abs(cncPath.boundingBox[1].Y - self.boundingBox[0].Y) > 25.4 and       \
         abs(cncPath.boundingBox[1].Y - self.boundingBox[1].Y) > 25.4):
        continue
      #print("   Adjacent " + str(areBoundingBoxesAdjacentAlongY(cncPath.boundingBox, self.boundingBox)))
      for XtabLocation in cncPath.XtabLocations:
        #####################################################
        #Find the closest point to this tab on the path
        #####################################################
        prevPoint = self.points3D[-1]
        smallestDist = self.distPerTab
        candidateTabLocation = 0
        #print((XtabLocation - self.distPerTab * 1j / 2))
        #print((XtabLocation))
        #print((XtabLocation + self.distPerTab * 1j / 2))
        for point in self.points3D:
          #See if a vertical line of possible adjacent shape intersects curernt path anywhere
          #If so add a tab to current shape
          intersection = lineIntersection(Point3D(XtabLocation.X, XtabLocation.Y - self.tabWidth * 4.0) , Point3D(XtabLocation.X, XtabLocation.Y + self.tabWidth * 4.0), prevPoint, point)
          #Don't add the tab if intersection not found or intersection is for opposite side of part
          if intersection and self.XTabsOnSameSide(intersection, XtabLocation):
            #print("      Attempting Adjacent Tab" )
            #print("      XtabLocation: " + str(XtabLocation) + " intersection: " + str(intersection))
            self.addXTabIfNoneAlready(intersection)
            break
          prevPoint = point

  def idealToActualXTabLocation(self, location, locationOnMaxSide):
    #print("Location: " + str(location))
    bestTabLocation = None
    prevPoint = self.points3D[-1]
    maxScore = -100000000000000000
    for point in self.points3D:
      #Sweep the tab from beginning spacing to end spacing to find ideal location
      pointsToTry = 20.0
      for XTabPosition in np.arange(location.X - self.distPerTab / 2.0, \
                                    location.X + self.distPerTab / 2.0 + self.distPerTab / pointsToTry, \
                                    self.distPerTab / pointsToTry):
        #If line along cncPath intersects candidate tab location and on same side as ideal tab
        #candPointOnMaxSide = self.XTabOnMaxSide(candPoint)
        #correctSide = (candPointOnMaxSide and locationOnMaxSide) or (not candPointOnMaxSide and not locationOnMaxSide)
        if ((point.X < XTabPosition and prevPoint.X >= XTabPosition) or (prevPoint.X < XTabPosition and point.X >= XTabPosition)):
          candPoint = Point3D(XTabPosition, point.Y)

          #hScore is highest at ideal location, starts falling off at outer locations
          hScore = 1 - abs(location.X - candPoint.X) / self.distPerTab
          #vScore is higest at outside of bounding box, smallest at center of bounding box
          bBoxMiddle = (self.boundingBox[1].Y + self.boundingBox[0].Y) / 2.0
          bBoxWidth  = max(0.1, (self.boundingBox[1].Y - self.boundingBox[0].Y))
          vScore = (candPoint.Y - bBoxMiddle) / bBoxWidth
          if not locationOnMaxSide:
            vScore = -vScore
          tabWidthAtMaterial = self.tabWidth + self.cutterDiameter
          tabNearCornerScore = min(distanceXY(candPoint, prevPoint) / (tabWidthAtMaterial / 2.0), distanceXY(candPoint, point) / (tabWidthAtMaterial / 2.0), 1.0)
          #tabNearCornerScore = 1
          score = hScore * vScore * tabNearCornerScore
          
          if bestTabLocation == None or score > maxScore:
            #print("****************Best found")
            #print(bBoxMiddle)
            #print(bBoxWidth)
            #print(candPoint.imag)
            #print(locationOnMaxSide)
            #print("score: " + str(score) + " location: " + str(candPoint))
            bestTabLocation = candPoint
            maxScore = score
      prevPoint = point


    #print("initial:" + str(location) + "bestTabLocation: " + str(bestTabLocation))
    return bestTabLocation

  def idealToActualYTabLocation(self, location, locationOnMaxSide):
    bestTabLocation = None
    prevPoint = self.points3D[-1]
    maxScore = -100000000000000000
    for point in self.points3D:
      #Sweep the tab from beginning spacing to end spacing to find ideal location
      pointsToTry = 20.0
      for YTabPosition in np.arange(location.Y - self.distPerTab / 2.0, \
                                    location.Y + self.distPerTab / 2.0 + self.distPerTab / pointsToTry, \
                                    self.distPerTab / pointsToTry):
        #If line along cncPath intersects cand tab location and on same side as ideal tab
        #candPointOnMaxSide = self.YTabOnMaxSide(candPoint)
        #correctSide = (candPointOnMaxSide and locationOnMaxSide) or (not candPointOnMaxSide and not locationOnMaxSide)
        if ((point.Y < YTabPosition and prevPoint.Y >= YTabPosition) or (prevPoint.Y < YTabPosition and point.Y >= YTabPosition)):
          candPoint = Point3D(point.X, YTabPosition)
          #vScore is highest at ideal location, starts falling off at outer locations
          vScore = 1 - abs(location.Y - candPoint.Y) / self.distPerTab
          #vScore is higest at outside of bounding box, smallest at center of bounding box
          bBoxMiddle = (self.boundingBox[1].X + self.boundingBox[0].X) / 2.0
          bBoxWidth = (self.boundingBox[1].X - self.boundingBox[0].X)
          if locationOnMaxSide:
            hScore = (candPoint.X - bBoxMiddle) / bBoxWidth * 2.0
          else:
            hScore = -(candPoint.X - bBoxMiddle) / bBoxWidth * 2.0

          #only need to be half distance of tab from corner as tab position is in middle of the tab
          tabWidthAtMaterial = self.tabWidth + self.cutterDiameter
          tabNearCornerScore = min(distanceXY(candPoint, prevPoint) / (tabWidthAtMaterial / 2.0), distanceXY(candPoint, point) / (tabWidthAtMaterial / 2.0), 1.0)
          score = hScore * vScore * tabNearCornerScore
          if bestTabLocation == None or score > maxScore:
            #print("score: " + str(score) + " location: " + str(candPoint))
            bestTabLocation = candPoint
            maxScore = score
      prevPoint = point
    #print()
    #print()
    return bestTabLocation


  def addXTabsWhereNoneAlread(self):
    #Add XTabs at the ideal location if a tab already doesn't exist in that location
    #Prior to calling this, addXTabsIfAdjacentTab should be called to give priority to already adjacent locations

    print("    addXTabsWhereNoneAlready")
    for idealXTabPosition in self.idealXTabPositions:
      #For each tab location only choose top most and bottom most location as those will be outer most portions of the part
      maxPoint = None
      minPoint = None
      prevPoint = self.points3D[-1]
      for point in self.points3D:
        #If line along cncPath intersects ideal tab location
        if (point.X < idealXTabPosition and prevPoint.X >= idealXTabPosition) or (prevPoint.X < idealXTabPosition and point.X >= idealXTabPosition):
          candPoint = Point3D(idealXTabPosition, point.Y)
          #Only add X tabs at topmost and bottom most locations, not in between
          if maxPoint == None or candPoint.Y > maxPoint.Y:
            maxPoint = candPoint
          if minPoint == None or candPoint.Y < minPoint.Y:
            minPoint = candPoint
        prevPoint = point

      
      if maxPoint != None:
        actualLocation = self.idealToActualXTabLocation(maxPoint, True)
        #print("idealX:" + str(maxPoint) + "actual: " + str(actualLocation))
        #print()
        self.addXTabIfNoneAlready(actualLocation)
      if minPoint != None and maxPoint != minPoint:
        actualLocation = self.idealToActualXTabLocation(minPoint, False)
        #print("idealX:" + str(minPoint) + "actual: " + str(actualLocation))
        #print()
        self.addXTabIfNoneAlready(actualLocation)

  def addYTabsWhereNoneAlread(self):
    print("    addYTabsWhereNoneAlready")
    #Add XTabs at the ideal location if a tab already doesn't exist in that location
    #Prior to calling this, addXTabsIfAdjacentTab should be called to give priority to already adjacent locations
    prevPoint = self.points3D[0]
    i=0
    for idealYTabPosition in self.idealYTabPositions:
      minPoint = None
      maxPoint = None
      prevPoint = self.points3D[-1]
      for point in self.points3D:
        if (point.Y < idealYTabPosition and prevPoint.Y >= idealYTabPosition) or (prevPoint.Y < idealYTabPosition and point.Y >= idealYTabPosition):
          candPoint = Point3D(point.X, idealYTabPosition)
          #self.addXTabIfNoneAlready(candPoint)
          if maxPoint == None or candPoint.X > maxPoint.X:
            maxPoint = candPoint
          if minPoint == None or candPoint.X < minPoint.X:
            minPoint = candPoint
      prevPoint = point

      if maxPoint != None:
        actualLocation = self.idealToActualYTabLocation(maxPoint, True)
        self.addYTabIfNoneAlready(actualLocation)
        #print("idealY:" + str(maxPoint) + "actual: " + str(actualLocation))
      if minPoint != None and maxPoint != minPoint:
        actualLocation = self.idealToActualYTabLocation(minPoint, False)
        self.addYTabIfNoneAlready(actualLocation)
        #print("idealY:" + str(minPoint) + "actual: " + str(actualLocation))





#####################################################################
#cncPaths class
#####################################################################
class cncPathsClass:
  def __init__(self, inputSvgFile, pointsPerCurve, distPerTab, tabWidth, cutterDiameter):
     #####################################################################
     #Open specified file
     #####################################################################
     paths, attributes, svg_attributes = svg2paths2(inputSvgFile)
     colors = []

     for path, attribute in zip(paths, attributes):
       if 'stroke' in attribute.keys():
         nextColor = hex_to_rgb(attribute['stroke'])
       elif 'style' in attribute.keys():
         match = re.search('stroke:#(\d+)', attribute['style'])
         if match:
           nextColor = hex_to_rgb("#" + match.group(1))
         else:
           nextColor = (0,0,0)
       colors.append(list(nextColor))
     #print()
     #print()
     #print(colors)
     #print(paths)
         
     self.attributes = attributes
     self.svg_attributes = svg_attributes
     print("CNC Paths: " + str(len(paths)))
     #####################################################################
     #Create a cncPath out of each svgPath to store the svg path and other information
     #####################################################################
     self.cncPaths = []
     for path, color in zip(paths, colors):
       self.cncPaths.append(cncPathClass(path, color, distPerTab, tabWidth, pointsPerCurve, cutterDiameter))

     #####################################################################
     #Now that all cncPaths are created, determine which ones are borders or holes, 
     #set milling direction, keep track of total bounding box
     #####################################################################
     self.boundingBox = self.cncPaths[0].boundingBox[:] #copy contents of list rather than reference
     for cncPath in self.cncPaths:
       cncPath.DetermineIfBorder(self.cncPaths)
       cncPath.SetConventionalMilling()
       cncPath.calculateIdealTabLocations()
       self.boundingBox[0].X = min(self.boundingBox[0].X, cncPath.boundingBox[0].X)
       self.boundingBox[0].Y = min(self.boundingBox[0].Y, cncPath.boundingBox[0].Y)
       self.boundingBox[1].X = max(self.boundingBox[1].X, cncPath.boundingBox[1].X)
       self.boundingBox[1].Y = max(self.boundingBox[1].Y, cncPath.boundingBox[1].Y)

     #####################################################################
     #Set class variables to those passed in
     #####################################################################
     self.pointsPerCurve = pointsPerCurve
     self.distPerTab = distPerTab
  def ToSvgFile(self, fileName):
    print("Saving cut paths to: " + fileName)
    paths = []
    attributes = []
    svg_attributes = []
    
    for cncPath in self.cncPaths:
      curPath = cncPath.cncPathToSvgPath()
      paths.append(curPath)
      attributes.append({'fill':'none', 'stroke': 'rgb(0,0,0)', 'stroke-width': '0.5'})
      #svg_attributes.append({})
    
    #print("*******************")
    #for cncPath in self.cncPaths:
    #  tabPaths = cncPath.cncTabsToPaths()
    #  for path in tabPaths:
    #    print(path)
    #    paths.append(path)
    #    attributes.append({'fill':'none', 'stroke': 'rgb(255,0,0)', 'stroke-width': '2.5'})
    #for cncPath in self.cncPaths:
    #  for x in cncPath.XtabLocations:
    #    path = Path(Line(x, x + 6 - 6j))
    #    paths.append(path)
    #    attributes.append({'fill':'none', 'stroke': 'rgb(0,255,0)', 'stroke-width': '0.5'})

    #for cncPath in self.cncPaths:
    #  for y in cncPath.YtabLocations:
    #    path = Path(Line(y, y + 6 - 6j))
    #    paths.append(path)
    #    attributes.append({'fill':'none', 'stroke': 'rgb(0,0,255)', 'stroke-width': '0.5'})

    wsvg(paths, attributes=attributes, svg_attributes=self.svg_attributes, filename=fileName)
  def orderCncHolePathsFirst(self):
    print("Order CNC Hole Paths First")
    #Order cuts from inside holes to outside paths
    orderedPaths = []
    for path in self.cncPaths:
      if not path.isBorder:
        orderedPaths.append(path)

    for path in self.cncPaths:
      if path.isBorder:
        orderedPaths.append(path)

    self.cncPaths = orderedPaths  

  def orderPartialCutsFirst(self):
    print("Order Partial cuts through material first")
    orderedPaths = []
    for path in self.cncPaths:
      if path.color[1] != 0:
        orderedPaths.append(path)

    for path in self.cncPaths:
      if path.color[1] == 0:
        orderedPaths.append(path)

  def ModifyPointsFromTabLocations(self):
    print("Adding points to paths for tabs")
    for path in self.cncPaths:
      path.ModifyPointsFromTabLocations()

  def addTabs(self):
    print("Adding Tabs")
    sortedByWidth = sorted(self.cncPaths, key=lambda x: x.width)
    sortedByHeight = sorted(self.cncPaths, key=lambda x: x.height)
    i = 0

    #Add tabs to X locations, considering other tabs already placed
    for path in sortedByWidth:
      #print("**********************************************")
      #print("Adding Tab for next Path, " + str(i) + "/" + str(len(sortedByWidth)))
      #print("**********************************************")
      i += 1
      #Starting with smallest add tabs where none already
      path.addXTabsWhereNoneAlread()
      #after adding tabs to each part, add adjacent tabs to a given part so the tab has something to hold on to
      #print("   Adding tabs for adacent paths")
      for path2 in sortedByWidth:
        #don't do it for current path
        if path != path2:
          path2.addXTabsIfAdjacentTab(self.cncPaths)

    #print()
    #print()
    #print()
    i = 0
    for path in sortedByHeight:
      i += 1
      #Starting with smallest add tabs where none already
      path.addYTabsWhereNoneAlread()
      #after adding tabs to each part, add adjacent tabs to a given part so the tab has something to hold on to
      #print("   Adding tabs for adacent paths")
      for path2 in sortedByHeight:
        #don't do it for current path
        #print(path.boundingBox)
        if path != path2:
          path2.addYTabsIfAdjacentTab(self.cncPaths)

#####################################################################
#cncGcodeGenerator class
#####################################################################
class cncGcodeGeneratorClass:
  def __init__(self, cncPaths, materialThickness, depthBelowMaterial, depthPerPass, cutFeedRate, safeHeight, tabHeight):

    self.cncPaths      = cncPaths
    #XYZ location undefined at the start
    self.location = Point3D(None, None, None)
    self.safeHeight = safeHeight

    self.materialThickness  = materialThickness
    self.depthBelowMaterial = depthBelowMaterial
    self.depthPerPass       = depthPerPass
    self.cutFeedRate        = cutFeedRate
    self.tabHeight          = tabHeight

    self.gCodes = []
    #Set the feed rate at the begining of the program
    self.gCodes.append(GCodeFeedRate(cutFeedRate))
    #Move to the safe Z height as the first move
    self.ZMove(self.safeHeight)

  def calculateDepthPerPass(self, idealDepthPerPass, cutDepth):
    passes = cutDepth / idealDepthPerPass
    #If depth per pass does not end at end of material thickness, then adjust depth per pass so all passes are even
    if passes - math.floor(passes) > 0.001:
      #print("    New Depth Per Pass: " + str( cutDepth / math.ceil(passes)))
      return cutDepth / math.ceil(passes)
    #print("New Depth Per Pass: " + str(idealDepthPerPass))
    return idealDepthPerPass

  def Generate(self):
    print("Generating G Code")
    for cncPath in self.cncPaths.cncPaths:  
      #print()
      #print()
      #print()
      #print("NEW PATH: " + str(cncPath))
      #########################################################
      #Cut each path, one depth of cut at a time
      #########################################################
      #If this is not a green line (green is >0) then this means cut all the way through the material
      if cncPath.color[1] == 0:
        cutThickness = self.materialThickness + self.depthBelowMaterial
      else:
        #Green line means cut partially through material
        cutThickness = self.materialThickness * (cncPath.color[1] / 255.0)

      depthPerPass = self.calculateDepthPerPass(self.depthPerPass, cutThickness)
      for height in np.arange(-depthPerPass, -cutThickness - depthPerPass, -depthPerPass):
        heightBelowTab = (self.materialThickness + height) < self.tabHeight
        #########################################################
        #If the first point in the cncPath does not start whith current router location
        #then pull tool up, move to that location, and back down
        #look at distance to get around rounding errors.  If distance is really closer
        #than 0.001mm we really don't care anyway.
        #else just move the tool down.
        #########################################################

        #start at tab height if this is a point specifying a tab but it is not the end of a tab
        if heightBelowTab and (cncPath.pointIsTab[0] and not cncPath.isPointTabEnd(0)):
          nextHeight = self.tabHeight
        else:
          nextHeight = height
        #if self.location.X != None:
          #print("first point: " + str(cncPath.points3D[0]))
          #print("curr loc:    " + str(self.location))
          #print("     distance: " + str(distanceXY(cncPath.points3D[0], self.location)))
        if self.location.X == None or self.location.Y == None or \
           distanceXY(cncPath.points3D[0], self.location) > 0.001:
          self.moveUpandDownToNextLocation(cncPath.points3D[0], nextHeight)
        else:
          self.ZMove(nextHeight)
        #print("NEXT HEIGHT:" + str(nextHeight))
        #########################################################
        #Cycle through the points in the cncPath
        #########################################################
        #prevPoint = cncPath.points3D[0]
        for i, point in enumerate(cncPath.points3D):
          #########################################################
          #Move tool to next point
          #########################################################
          #print("POINT: " + str(point))
          if distanceXY(point, self.location):
            self.XYMove(point)
          #if below tab height and point is start of a tab, then move to tab height
          if heightBelowTab and cncPath.isPointTabStart(i):
            self.ZMove(self.tabHeight - self.materialThickness)
            #print("TAB: " + str(self.tabHeight - self.materialThickness))
          if heightBelowTab and cncPath.isPointTabEnd(i):
            self.ZMove(height)

          #prevPoint = point
    #At end of cuts go back to a safe height
    self.gCodes.append(GCodeRapidMove(Z=self.safeHeight))

  def cutTabs(self):
    print("  Add G code to cut tabs")
    for cncPath in self.cncPaths.cncPaths:  
      totalDistance = 0
      cutSpacing = (cncPath.tabWidth + cncPath.cutterDiameter) / 5.0
      #print("******cutSpacing: " + str(cutSpacing))
      for i, point in enumerate(cncPath.points3D):
        #print(i)
        #If start of a tab
        if cncPath.isPointTabStart(i):
          totalDistance = 0
          nextCutDistance = cutSpacing
          #print("****************Tab start******************")
          #print(cncPath.points3D[i])

        #if middle of a tab
        elif cncPath.pointIsTab[i]:
          totalDistance = totalDistance + distanceXY(cncPath.points3D[i - 1], cncPath.points3D[i])
          startPoint = cncPath.points3D[i -1]
          #print("totalDistance: " + str(totalDistance))
          while totalDistance > nextCutDistance:
            cutLocation = lineCircleIntersections(startPoint, cncPath.points3D[i], startPoint, cutSpacing * 2.0)
            #print("cutLocation: " + str(cutLocation) + " startPoint: " + str(startPoint) + " endPoint: " + str(cncPath.points3D[i]))
            #print("nextCutDistance: " + str(nextCutDistance))
            #print()
            if len(cutLocation) == 1:
              cutLocation = cutLocation[0]
              self.moveUpandDownToNextLocation(cutLocation, -self.materialThickness - self.depthBelowMaterial)
              startPoint = cutLocation

            nextCutDistance = nextCutDistance + cutSpacing
    #At end of cutting tabs go back to a safe height
    self.gCodes.append(GCodeRapidMove(Z=self.safeHeight))

  def Save(self, fileName):
    print("Saving G code to: " + fileName)
    with open(fileName, "w") as outFile:
      outFile.write('\n'.join(str(g) for g in self.gCodes))

  def moveUpandDownToNextLocation(self, newLocation, depth):
    #print()
    #print("move up and down " + str(newLocation) + " depth " + str(depth))
    
    self.gCodes.append(GCodeRapidMove(Z=self.safeHeight))
    self.gCodes.append(GCodeRapidMove(X=newLocation.X, Y=newLocation.Y))
    self.gCodes.append(GCodeLinearMove(Z=depth))
    self.location.X = newLocation.X
    self.location.Y = newLocation.Y
    self.location.Z = depth

  def ZMove(self, depth):
    #print()
    #print("Move to Depth " + str(depth))
    self.gCodes.append(GCodeLinearMove(Z=depth))
    self.location.Z = depth
  def XYMove(self, newLocation):
    #print()
    #print("Move to next location " + str(newLocation))
    self.gCodes.append(GCodeLinearMove(X=newLocation.X, Y=newLocation.Y))
    self.location.X = newLocation.X
    self.location.Y = newLocation.Y

#####################################################################
#Main Code
#####################################################################

#Generate cncPaths object based on svgFile
cncPaths = cncPathsClass(inputSvgFile   = args.inputSvgFile[0],
                         pointsPerCurve = 30,
                         distPerTab      = 110,
                         tabWidth        = float(args.tabWidth[0]),
                         cutterDiameter  = float(args.cutterDiameter[0])
                        )

#Order cuts from inside holes to outside borders
cncPaths.orderCncHolePathsFirst()
cncPaths.orderPartialCutsFirst()

cncPaths.addTabs()
cncPaths.ModifyPointsFromTabLocations()

############################
# save cut paths to svg
############################
svgPath = os.path.join(os.path.dirname(args.inputSvgFile[0]), "output_cutPaths")
if not os.path.exists(svgPath):
    os.makedirs(svgPath)
cncPaths.ToSvgFile(os.path.join(svgPath, os.path.basename(os.path.splitext(args.inputSvgFile[0])[0] + "_cutPaths.svg")))

cncGcodeGenerator = cncGcodeGeneratorClass(cncPaths           = cncPaths,
                                           materialThickness  = float(args.materialThickness[0]),
                                           depthBelowMaterial = float(args.depthBelowMaterial[0]),
                                           depthPerPass       = float(args.depthPerPass[0]),
                                           cutFeedRate        = float(args.cutFeedRate[0]),
                                           safeHeight         = float(args.safeHeight[0]),
                                           tabHeight          = float(args.tabHeight[0])
                                          )
cncGcodeGenerator.Generate()
cncGcodeGenerator.cutTabs()

############################
# Save G code
############################
gcodePath = os.path.join(os.path.dirname(args.inputSvgFile[0]), "output_gcode")
if not os.path.exists(gcodePath):
    os.makedirs(gcodePath)
cncGcodeGenerator.Save(os.path.join(gcodePath, os.path.basename(os.path.splitext(args.inputSvgFile[0])[0] + ".gcode")))
