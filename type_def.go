// Copyright 2022 Guan Jianchang. All rights reserved.
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file.

package nav

//========================
//          Grid
//========================
type Grid struct {
	Col int
	Row int
}

func NewGrid(col int, row int) *Grid {
	return &Grid{
		Col: col,
		Row: row,
	}
}

func (g *Grid) Update(col int, row int) {
	g.Col = col
	g.Row = row
}

func (g *Grid) IsSameGrid(g2 *Grid) bool {
	if g.Col != g2.Col {
		return false
	}

	if g.Row != g2.Row {
		return false
	}

	return true
}

func (g *Grid) IsSameGrid2(col int, row int) bool {
	if g.Col != col {
		return false
	}

	if g.Row != row {
		return false
	}

	return true
}

//========================
//         Vector
//========================
type Vector struct {
	X int
	Y int
}

func NewVector(x int, y int) *Vector {
	return &Vector{
		X: x,
		Y: y,
	}
}

func (v *Vector) IsEmptyVector() bool {
	if v.X != 0 {
		return false
	}

	if v.Y != 0 {
		return false
	}

	return true
}

func (v *Vector) IsOblique() bool {
	if v.X == 0 {
		return false
	}

	if v.Y == 0 {
		return false
	}

	return true
}
