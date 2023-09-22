/**
 * Project Part 2
 * Student Name: Chen, Junhong
 * Student Number: 300140321
 */

package main

import (
	"bufio"
	"fmt"
	"math"
	"math/rand"
	"os"
	"strconv"
	"strings"
	"time"
)

type Point3D struct {
	X float64
	Y float64
	Z float64
}
type Plane3D struct {
	A float64
	B float64
	C float64
	D float64
	N Point3D
}
type Plane3DwSupport struct {
	Plane3D
	Size int
}

var eps float64
var iteration int
var points []Point3D

// reads an XYZ file and returns a slice of Point3D
func ReadXYZ(filename string) []Point3D {
	var points []Point3D
	file, _ := os.Open(filename)
	defer file.Close()
	scanner := bufio.NewScanner(file)
	num := 0
	for scanner.Scan() {
		num++
		if num != 1 {
			line := scanner.Text()
			coords := strings.Fields(line)
			x, _ := strconv.ParseFloat(coords[0], 64)
			y, _ := strconv.ParseFloat(coords[1], 64)
			z, _ := strconv.ParseFloat(coords[2], 64)
			p := Point3D{x, y, z}
			points = append(points, p)
		}
	}
	return points
}

// saves a slice of Point3D into an XYZ file
func SaveXYZ(filename string, points []Point3D) {
	flie, _ := os.Create(filename)
	defer flie.Close()
	for _, point := range points {
		s := fmt.Sprintf("%f %f %f\n", point.X, point.Y, point.Z)
		flie.WriteString(s)
	}
}

// computes the distance between points p1 and p2
func (p1 *Point3D) GetDistance(p2 *Point3D) float64 {
	result := math.Sqrt(math.Pow(p1.X-p2.X, 2) + math.Pow(p1.Y-p2.Y, 2) + math.Pow(p1.Z-p2.Z, 2))
	return result
}

// computes the plane defined by a set of 3 points
func GetPlane(points []Point3D) Plane3D {
	p1 := points[0]
	p2 := points[1]
	p3 := points[2]
	f := Point3D{p2.X - p1.X, p2.Y - p1.Y, p2.Z - p1.Z}
	g := Point3D{p3.X - p1.X, p3.Y - p1.Y, p3.Z - p1.Z}
	e := Point3D{f.Y*g.Z - f.Z*g.Y, f.Z*g.X - f.X*g.Z, f.X*g.Y - f.Y*g.X}
	a := e.X
	b := e.Y
	c := e.Z
	d := a*p1.X + b*p1.Y + c*p1.Z
	return Plane3D{a, b, c, d, e}
}

// computes the number of required RANSAC iterations
func GetNumberOfIterations(confidence float64, percentageOfPointsOnPlane float64) int {
	result := math.Ceil(math.Log(1-confidence) / math.Log(1-math.Pow(percentageOfPointsOnPlane, 3)))
	return int(result)
}

// computes the support of a plane in a set of points
func GetSupport(plane Plane3D, points []Point3D, eps float64) Plane3DwSupport {
	supportSize := 0
	for _, points := range points {
		if plane.getDistance(points) <= eps {
			supportSize++
		}
	}
	return Plane3DwSupport{plane, supportSize}
}

// extracts the points that supports the given plane
// and returns them as a slice of points
func GetSupportingPoints(plane Plane3D, points []Point3D, eps float64) []Point3D {
	var supporting_points []Point3D
	for _, points := range points {
		if plane.getDistance(points) <= eps {
			supporting_points = append(supporting_points, points)
		}
	}
	return supporting_points
}

// creates a new slice of points in which all points
// belonging to the plane have been removed
func RemovePlane(plane Plane3D, points []Point3D, eps float64) []Point3D {
	var result []Point3D
	for _, point := range points {
		if plane.getDistance(point) > eps {
			result = append(result, point)
		}
	}
	return result
}

// calculate distance between the point and the plane
func (plane *Plane3D) getDistance(pt Point3D) float64 {
	p := Point3D{0, 0, plane.D / plane.C}
	h := Point3D{pt.X - p.X, pt.Y - p.Y, pt.Z - p.Z}
	k := h.X*plane.N.X + h.Y*plane.N.Y + h.Z*plane.N.Z/
		math.Pow(math.Sqrt(math.Pow(plane.N.X, 2)+math.Pow(plane.N.Y, 2)+math.Pow(plane.N.Z, 2)), 2)
	project := Point3D{k * plane.N.X, k * plane.N.Y, k * plane.N.Z}
	length := math.Sqrt(math.Pow(project.X, 2) + math.Pow(project.Y, 2) + math.Pow(project.Z, 2))
	return length
}

// It randomly selects a point from the provided slice of Point3D (the input point cloud). Its output
// channel transmits instances of Point3D.
func randomPointGenerator(end chan bool) chan Point3D {
	result := make(chan Point3D)
	go func() {
		for {
			select {
			case <-end:
				close(result)
				return
			case result <- points[rand.Intn(len(points))]:
			}
		}
	}()
	return result
}

// It reads Point3D instances from its input channel and accumulate 3 points. Its output channel
// transmits arrays of Point3D
func tripletOfPointsGenerator(point chan Point3D, end chan bool) chan []Point3D {
	result := make(chan []Point3D)
	go func() {
		for {
			var point1 Point3D
			var point2 Point3D
			var point3 Point3D
			point1 = <-point
			for {
				point2 = <-point
				if point1 != point2 {
					break
				}
			}
			for {
				point3 = <-point
				if point1 != point3 && point2 != point3 {
					break
				}
			}
			select {
			case <-end:
				close(result)
				return
			case result <- []Point3D{point1, point2, point3}:
			}
		}
	}()
	return result
}

// It reads arrays of Point3D and resend them. It automatically stops the pipeline after having received N
// arrays.
func tankN(pointSet chan []Point3D, end chan bool) chan []Point3D {
	result := make(chan []Point3D)
	go func() {

		for i := 0; i < iteration; i++ {
			threePoints := <-pointSet
			result <- threePoints
		}
		for o := 0; o < 2; o++ {
			end <- true
		}
		close(result)
		return
	}()
	return result
}

// It reads arrays of three Point3D and compute the plane defined by these points. Its output channel
// transmits Plane3D instances describing the computed plane parameters.
func planeEstimator(pointSet chan []Point3D) chan Plane3D {
	result := make(chan Plane3D)
	go func() {
		for threePoints := range pointSet {
			plane := GetPlane(threePoints)
			result <- plane
		}
		close(result)
	}()
	return result
}

// It counts the number of points in the provided slice of Point3D (the input point cloud) that supports
// the received 3D plane. Its output channel transmits the plane parameters and the number of supporting
// points in a Point3DwSupport instance.
func supportingPointFinder(plane chan Plane3D) chan Plane3DwSupport {
	result := make(chan Plane3DwSupport)
	go func() {
		for plane := range plane {
			support := GetSupport(plane, points, eps)
			result <- support
		}
		close(result)
	}()
	return result
}

// It multiplexes the results received from multiple channels into one output channel.
func fanIn(pointSet []chan Plane3DwSupport) chan Plane3DwSupport {
	result := make(chan Plane3DwSupport)
	go func() {
		for i := 0; i < iteration; {
			for o := 0; o < len(pointSet); {
				select {
				case support := <-pointSet[o]:
					result <- support
					i++
					o++
				default:
					o++
				}
			}
		}
		close(result)
	}()
	return result
}

// It receives Plane3DwSupport instances and keepd in memory the plane with the best support
// received so far. This component does not output values, it simply maintains the provided
// *Plane3DwSupport variable.
func dominantPlaneIdentifier(pointSet chan Plane3DwSupport, best *Plane3DwSupport) {
	for support := range pointSet {
		if support.Size > best.Size {
			*best = support
		}
	}
}

// This is the main function
func main() {
	points = ReadXYZ("PointCloud3.xyz")
	//points = ReadXYZ("remain.xyz")
	start := time.Now()
	fmt.Println(len(points))
	point := Point3D{0, 0, 0}
	plane := Plane3D{0, 0, 0, 0, point}
	bestPlane := Plane3DwSupport{plane, 0}
	iteration = GetNumberOfIterations(0.99, 0.1)
	eps = 0.1
	stop := make(chan bool)
	size := iteration
	randmoPoint := randomPointGenerator(stop)
	threePoints := tripletOfPointsGenerator(randmoPoint, stop)
	point3D := tankN(threePoints, stop)
	planeEstimator := planeEstimator(point3D)
	var supportingPointFinders = make([]chan Plane3DwSupport, size)
	for i := 0; i < size; i++ {
		supportingPointFinders[i] = supportingPointFinder(planeEstimator)
	}
	dominantPlaneIdentifier(fanIn(supportingPointFinders), &bestPlane)
	elapsed := time.Since(start)
	fmt.Printf("Execution took %s", elapsed)
	finalResult := GetSupportingPoints(bestPlane.Plane3D, points, eps)
	//SaveXYZ("PointCloud3_p3.xyz", finalResult)
	SaveXYZ("PointCloud3_p1.xyz", finalResult)
	remainPoints := RemovePlane(bestPlane.Plane3D, points, eps)
	SaveXYZ("remain.xyz", remainPoints)
}
