package main

import (
	"fmt"

	"github.com/setanarut/vec"
)

func main() {
	a := vec.Vec2{4344, 12}
	fmt.Println(a.Lerp(vec.Vec2{}, 3))
}
