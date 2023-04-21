package main

import (
	"bufio"
	"fmt"
	"log"
	"math"
	"math/rand"
	"os"
	"strconv"
	"strings"
	"sync"
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
}

type Plane3DwSupport struct {
	Plane3D
	SupportSize int
}

// reads an XYZ file and returns a slice of Point3D
// Source: https://zetcode.com/golang/readfile/
// https://www.geeksforgeeks.org/how-to-convert-string-to-float-type-in-golang/
func ReadXYZ(filename string) []Point3D{

	file, err := os.Open(filename)
    if err != nil {
        log.Fatal(err)
    }
    defer file.Close()

    scanner := bufio.NewScanner(file)

    var points []Point3D
    for scanner.Scan() {
        line := scanner.Text()
        fields := strings.Fields(line)
        if len(fields) != 3 {
            continue
        }
        x, err := strconv.ParseFloat(fields[0], 64)
        if err != nil {
            continue
        }
        y, err := strconv.ParseFloat(fields[1], 64)
        if err != nil {
            continue
        }
        z, err := strconv.ParseFloat(fields[2], 64)
        if err != nil {
            continue
        }
        points = append(points, Point3D{x, y, z})
    }

    if err := scanner.Err(); err != nil {
        log.Fatal(err)
    }

    return points
}

// saves a slice of Point3D into an XYZ file
// https://zetcode.com/golang/writefile/
func SaveXYZ(filename string, points []Point3D){

	file, err := os.Create(filename)
    if err != nil {
        log.Fatal(err)
    }
    defer file.Close()

    writer := bufio.NewWriter(file)

    for _, point := range points {
        fmt.Fprintf(writer, "%f %f %f\n", point.X, point.Y, point.Z)
    }

    if err := writer.Flush(); err != nil {
        log.Fatal(err)
    }
}

// computes the distance between points p1 and p2
func (p1 *Point3D) GetDistance(p2 *Point3D) float64{
	distanceX := p1.X - p2.X
    distanceY := p1.Y - p2.Y
    distanceZ := p1.Z - p2.Z
	
    return math.Sqrt(distanceX*distanceX + distanceY*distanceY + distanceZ*distanceZ)
}

// computes the plane defined by a set of 3 points
// Source: https://www.nagwa.com/en/explainers/373101390857/#:~:text=Definition%3A%20General%20Form%20of%20the,vector%20parallel%20to%20the%20plane.
// https://math.stackexchange.com/questions/2686606/equation-of-a-plane-passing-through-3-points
func GetPlane(points []Point3D) Plane3D{
	p1, p2, p3 := points[0], points[1], points[2]

	// Computes the normal vector of the plane
	u := Point3D{X: p2.X - p1.X, Y: p2.Y - p1.Y, Z: p2.Z - p1.Z}
	v := Point3D{X: p3.X - p1.X, Y: p3.Y - p1.Y, Z: p3.Z - p1.Z}
	n := Point3D{X: u.Y*v.Z - u.Z*v.Y, Y: u.Z*v.X - u.X*v.Z, Z: u.X*v.Y - u.Y*v.X}

	// Checks that the three points are not collinear
	if n.X == 0 && n.Y == 0 && n.Z == 0 {
		panic("Points are collinear")
	}

	// Normalizes the normal vector
	length := math.Sqrt(n.X*n.X + n.Y*n.Y + n.Z*n.Z)
	n.X /= length
	n.Y /= length
	n.Z /= length

	// Computes the plane parameters from the normal vector and a point on the plane
	A := n.X
	B := n.Y
	C := n.Z
	D := -(A*p1.X + B*p1.Y + C*p1.Z)

	return Plane3D{A: A, B: B, C: C, D: D}
}

// computes the number of required RANSAC iterations
func GetNumberOfIterations(confidence float64, percentageOfPointsOnPlane float64) int{
	k := int(math.Log(1-confidence) / math.Log(1-math.Pow(percentageOfPointsOnPlane/100, 3)))
	return k	
}

// computes the support of a plane in a set of points
func GetSupport(plane Plane3D, points []Point3D, eps float64) Plane3DwSupport{
	var support int
	for _, point := range points {
		if math.Abs(plane.A*point.X+plane.B*point.Y+plane.C*point.Z-plane.D) < eps {
			support++
		}
	}
	return Plane3DwSupport{plane, support}
}

// extracts the points that supports the given plane
// and returns them as a slice of points
func GetSupportingPoints(plane Plane3D, points []Point3D, eps float64) []Point3D{
	var supportingPoints []Point3D
	for _, point := range points {
		if math.Abs(plane.A*point.X+plane.B*point.Y+plane.C*point.Z-plane.D) < eps {
			supportingPoints = append(supportingPoints, point)
		}
	}
	return supportingPoints
}

// creates a new slice of points in which all points
// belonging to the plane have been removed
func RemovePlane(plane Plane3D, points []Point3D, eps float64) []Point3D{
	var remainingPoints []Point3D
	for _, point := range points {
		if math.Abs(plane.A*point.X+plane.B*point.Y+plane.C*point.Z-plane.D) >= eps {
			remainingPoints = append(remainingPoints, point)
		}
	}
	return remainingPoints
}

func randomPointGenerator(points []Point3D) <-chan Point3D {
    out := make(chan Point3D)
    go func() {
        defer close(out)
        for {
            out <- points[rand.Intn(len(points))]
        }
    }()
    return out
}

// Triplet of points generator
func tripletOfPointsGenerator(in <-chan Point3D) <-chan [3]Point3D {
    out := make(chan [3]Point3D)
    go func() {
        defer close(out)
        triplet := [3]Point3D{}
        for point := range in {
            triplet[0], triplet[1], triplet[2] = triplet[1], triplet[2], point
            if triplet[2] != (Point3D{}) {
                out <- triplet
            }
        }
    }()
    return out
}

// TakeN
func takeN(in <-chan [3]Point3D, n int) <-chan [3]Point3D {
    out := make(chan [3]Point3D)
    go func() {
        defer close(out)
        for i := 0; i < n; i++ {
            out <- <-in
        }
    }()
    return out
}

// Plane estimator
func planeEstimator(in <-chan [3]Point3D) <-chan Plane3D {
    out := make(chan Plane3D)
    go func() {
        defer close(out)
        for triplet := range in {
            plane := GetPlane(triplet[:])
            out <- plane
        }
    }()
    return out
}

// Supporting point finder
func supportingPointFinder(in <-chan Plane3D, points []Point3D, eps float64) <-chan Plane3DwSupport {
   	out := make(chan Plane3DwSupport)
    go func() {
        defer close(out)
        for plane := range in {
            support := GetSupport(plane, points, eps)
            out <- Plane3DwSupport{plane, support.SupportSize}
        }
    }()
    return out
}

// Fan In
func fanIn(ins []<-chan Plane3DwSupport) <-chan Plane3DwSupport {
    out := make(chan Plane3DwSupport)
    var wg sync.WaitGroup
    multiplex := func(in <-chan Plane3DwSupport) {
        defer wg.Done()
        for plane := range in {
            out <- plane
        }
    }
    wg.Add(len(ins))
    for _, in := range ins {
        go multiplex(in)
    }
    go func() {
        wg.Wait()
        close(out)
    }()
    return out
}

// Dominant plane identifier
func dominantPlaneIdentifier(in <-chan Plane3DwSupport, bestSupport *Plane3DwSupport) {
    for plane := range in {
        if plane.SupportSize > bestSupport.SupportSize {
            *bestSupport = plane
        }
    }
}

// RANSAC find dominant plane pipeline
func ransacPipeline(points []Point3D, confidence float64, percentageOfPointsOnPlane float64, eps float64, bestSupport *Plane3DwSupport) {
    numIterations := GetNumberOfIterations(confidence, percentageOfPointsOnPlane)
    randomPoints := randomPointGenerator(points)
    triplets := tripletOfPointsGenerator(randomPoints)
    planes := planeEstimator(triplets)
    supports := supportingPointFinder(planes, points, eps)
    ins := make([]<-chan Plane3DwSupport, numIterations)
    for i := range ins {
        ins[i] = supports
    }
}

func main() {
	// Parse command-line arguments
    if len(os.Args) < 5 {
        fmt.Println("Usage:", os.Args[0], "filename confidence percentage eps")
        return
    }
    filename := os.Args[1]
    confidence, err := strconv.ParseFloat(os.Args[2], 64)
    if err != nil {
        fmt.Println("Invalid confidence value:", os.Args[2])
        return
    }
    percentage, err := strconv.ParseFloat(os.Args[3], 64)
    if err != nil {
        fmt.Println("Invalid percentage value:", os.Args[3])
        return
    }
    eps, err := strconv.ParseFloat(os.Args[4], 64)
    if err != nil {
        fmt.Println("Invalid eps value:", os.Args[4])
        return
    }

    // Read the input point cloud from file
    points := ReadXYZ(filename)

    // Create the channels for the pipeline
    randPointChan := make(chan Point3D)
    tripletChan := make(chan [3]Point3D)
    planeChan := make(chan Plane3D)
    supportChan := make(chan Plane3DwSupport)
    bestSupportChan := make(chan Plane3DwSupport)

    // Create the pipeline components
    randPointGen := func() {
        for {
            randPointChan <- points[rand.Intn(len(points))]
        }
    }
    tripletGen := func() {
        triplet := [3]Point3D{}
        for {
            for i := 0; i < 3; i++ {
                triplet[i] = <-randPointChan
            }
            tripletChan <- triplet
        }
    }
    planeEstimator := func() {
        for {
            triplet := <-tripletChan
            plane := GetPlane(triplet[:])
            planeChan <- plane
        }
    }
    supportingPointFinder := func() {
        for {
            plane := <-planeChan
            support := GetSupport(plane, points, eps)
            supportChan <- support
        }
    }
    bestSupportIdentifier := func(bestSupport *Plane3DwSupport) {
        for {
            support := <-supportChan
            if support.SupportSize > bestSupport.SupportSize {
                *bestSupport = support
            }
        }
    }

    // Start the pipeline
    go randPointGen()
    go tripletGen()
    go fanIn(bestSupportChan)
    go bestSupportIdentifier(&bestSupport)

    // Iterate for the required number of iterations
    numIterations := GetNumberOfIterations(confidence, percentage)
    for i := 0; i < numIterations; i++ {
        <-bestSupportChan
    }

    // Save the supporting points and the remaining points
    supportingPoints := GetSupportingPoints(bestSupport.Plane3D, points, eps)
    SaveXYZ(filename+"_p", supportingPoints)
    remainingPoints := RemovePlane(bestSupport.Plane3D, points, eps)
    SaveXYZ(filename+"_p0", remainingPoints)
	
}

/*
if len(os.Args) < 5 {
		fmt.Println("Usage: go planeRANSAC filename confidence percentage eps")
		return
	}

	// Read the XYZ file specified as a first argument to your go program and create the corresponding slice of Point3D, composed of the set of points of the XYZ file.
	
	filename := os.Args[1]
	points := ReadXYZ(filename)
	
	
	// Create a bestSupport variable of type Plane3DwSupport initialized to all 0s.
	// alternative: var bestSupport Plane3DwSupport
	// bestSupport := Plane3DwSupport{Plane3D{}, 0}

	// find number of iterations required
	confidence, err := strconv.ParseFloat(os.Args[2], 64)
	if err != nil {
		fmt.Printf("Invalid confidence argument: %s\n", os.Args[2])
		return
	}
	percentageOfPointsOnPlane, err := strconv.ParseFloat(os.Args[3], 64)
	if err != nil {
		fmt.Printf("Invalid percentage argument: %s\n", os.Args[3])
		return
	}
	eps, err := strconv.ParseFloat(os.Args[4], 64)
	if err != nil {
		fmt.Printf("Invalid eps argument: %s\n", os.Args[4])
		return
	}
	
	// Find the number of iterations required based on the specified confidence and percentage provided as 1st and 2nd arguments for the GetNumberOfIterations function.
	numIterations := GetNumberOfIterations(confidence, percentageOfPointsOnPlane)
	
	// Create and start the RANSAC find dominant plane pipeline. This pipeline automatically stops after the required number of iterations.
    randPointChan := make(chan Point3D)
    tripletChan := make(chan [3]Point3D)
    planeChan := make(chan Plane3D)
    supportChan := make(chan Plane3DwSupport)
    bestSupportChan := make(chan Plane3DwSupport)

	randPointGen := func() {
        for {
            randPointChan <- points[rand.Intn(len(points))]
        }
    }
    tripletGen := func() {
        triplet := [3]Point3D{}
        for {
            for i := 0; i < 3; i++ {
                triplet[i] = <-randPointChan
            }
            tripletChan <- triplet
        }
    }
    planeEstimator := func() {
        for {
            triplet := <-tripletChan
            plane := GetPlane(triplet[:])
            planeChan <- plane
        }
    }
    supportingPointFinder := func() {
        for {
            plane := <-planeChan
            support := GetSupport(plane, points, eps)
            supportChan <- support
        }
    }
    bestSupportIdentifier := func(bestSupport *Plane3DwSupport) {
        for {
            support := <-supportChan
            if support.NumSupportingPoints > bestSupport.NumSupportingPoints {
                *bestSupport = support
            }
        }
    }

	go randPointGen()
    go tripletGen()
    go fanIn(bestSupportChan, supportingPointFinder)
    go bestSupportIdentifier(&bestSupport)
	
	// Once the pipeline has terminated, save the supporting points of the identified dominant plane in a file named by appending _p to the input filename.
	supportingPoints := GetSupportingPoints(ransacPipeline.BestSupport.Plane3D, points, eps)
	SaveXYZ(filename+"_p", supportingPoints)
	
	
	// Save the original point cloud without the supporting points of the dominant plane in a file named by appending _p0 to the input filename.
	remainingPoints := RemovePlane(ransacPipeline.BestSupport.Plane3D, points, eps)
	SaveXYZ(filename+"_p0", remainingPoints)
*/