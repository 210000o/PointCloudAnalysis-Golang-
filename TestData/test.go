package main

func sum(s []int, ch chan int) {
	sum := 0
	for _, v := range s {
		sum += v
	}
	ch <- sum
}

func main() {

	println(plustwo(plusone(0)))
}

func plusone(in int) int {
	return in + 1
}

func plustwo(in int) int {
	return in + 2
}