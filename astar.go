// Copyright 2022 Guan Jianchang. All rights reserved.
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file.

package nav

//========================
//      AStarNode
//========================
type AStarNode struct {
	*BasePathNode
}

func NewAStarNode(parent PathNode, vecParent *Vector, minGValue uint32, col int, row int) *AStarNode {
	return &AStarNode{
		BasePathNode: NewBasePathNode(parent, vecParent, minGValue, col, row),
	}
}

//========================
//          AStar
//========================
type AStar struct {
	*BasePathFinder
}

func NewAStar() *AStar {
	a := &AStar{}

	a.BasePathFinder = NewBasePathFinder(a)
	return a
}

func (a *AStar) CreateFirstNode(col int, row int) PathNode {
	return NewAStarNode(nil, nil, 0, col, row)
}

func (a *AStar) UnfoldGrid(m NavigationMap, dstGrid *Grid, node PathNode) {
	grid := node.GetGrid()
	if a.handleGrid(m, grid.Col-1, grid.Row, node, dstGrid) {
		return
	}

	if a.handleGrid(m, grid.Col+1, grid.Row, node, dstGrid) {
		return
	}

	if a.handleGrid(m, grid.Col, grid.Row-1, node, dstGrid) {
		return
	}

	if a.handleGrid(m, grid.Col, grid.Row+1, node, dstGrid) {
		return
	}

	a.AddNodeToCloseList(node)
}

func (a *AStar) handleGrid(m NavigationMap, col int, row int, parent PathNode, dstGrid *Grid) bool {
	// find out dest grid
	if dstGrid.IsSameGrid2(col, row) {
		a.lastNode = NewAStarNode(parent, nil, 0, col, row)
		return true
	}

	// parent grid, skip
	grid := parent.GetGrid()
	if grid.IsSameGrid2(col, row) {
		return false
	}

	// can't cross, skip
	if !m.CanCross(col, row) {
		return false
	}

	addGValue := m.GetGValue(col, row)
	minGValue := parent.GetMinGValue() + addGValue

	// already in open list or close list, update min G value
	if a.UpdateExistList(m, col, row, parent, nil, minGValue) {
		return false
	}

	// new grid, add to open list
	node := NewAStarNode(parent, nil, minGValue, col, row)
	a.AddNodeToOpenList(node)

	return false
}
