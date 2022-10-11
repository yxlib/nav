// Copyright 2022 Guan Jianchang. All rights reserved.
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file.

package nav

import "math"

//========================
//      NavigationMap
//========================
type NavigationMap interface {
	GetColRow() (col uint32, row uint32)
	CanCross(col int, row int) bool
	GetGValue(col int, row int) uint32
	GetMinGValue() uint32
}

//========================
//      PathNode
//========================
type PathNode interface {
	SetParent(parent PathNode)
	GetParent() PathNode
	UpdateParent(parent PathNode, vecParent *Vector)
	GetParentVector() *Vector
	AddChild(node PathNode)
	RemoveChild(node PathNode)
	SetMinGValue(minGValue uint32, m NavigationMap)
	GetMinGValue() uint32
	GetGrid() *Grid
	// UpdateChildrenGValue(m NavigationMap)
}

type BasePathNode struct {
	parent    PathNode
	vecParent *Vector
	children  []PathNode
	minGValue uint32
	grid      *Grid
}

func NewBasePathNode(parent PathNode, vecParent *Vector, minGValue uint32, col int, row int) *BasePathNode {
	n := &BasePathNode{
		parent:    parent,
		vecParent: vecParent,
		children:  make([]PathNode, 0),
		minGValue: minGValue,
		grid:      NewGrid(col, row),
	}

	if n.parent != nil {
		n.parent.AddChild(n)
	}

	return n
}

func (n *BasePathNode) SetParent(parent PathNode) {
	n.parent = parent
}

func (n *BasePathNode) GetParent() PathNode {
	return n.parent
}

func (n *BasePathNode) UpdateParent(parent PathNode, vecParent *Vector) {
	if n.parent != nil {
		n.parent.RemoveChild(n)
	}

	n.parent = parent
	n.vecParent = vecParent
}

// func (n *BasePathNode) SetParentVector(vec *Vector) {
// 	n.vecParent = vec
// }

func (n *BasePathNode) GetParentVector() *Vector {
	return n.vecParent
}

func (n *BasePathNode) AddChild(node PathNode) {
	if node == nil {
		return
	}

	n.children = append(n.children, node)
}

func (n *BasePathNode) RemoveChild(node PathNode) {
	if node == nil {
		return
	}

	for i, exist := range n.children {
		if exist == node {
			n.children = append(n.children[:i], n.children[i+1:]...)
			return
		}
	}
}

func (n *BasePathNode) SetMinGValue(minGValue uint32, m NavigationMap) {
	gValueChange := int(minGValue) - int(n.minGValue)
	n.minGValue = minGValue
	n.UpdateChildrenGValue(m, gValueChange)
}

func (n *BasePathNode) GetMinGValue() uint32 {
	return n.minGValue
}

func (n *BasePathNode) GetGrid() *Grid {
	return n.grid
}

func (n *BasePathNode) UpdateChildrenGValue(m NavigationMap, gValueChange int) {
	for _, child := range n.children {
		oldMinGValue := child.GetMinGValue()
		minGValue := int(oldMinGValue) + gValueChange
		if minGValue < 0 {
			minGValue = 0
		}

		child.SetMinGValue(uint32(minGValue), m)
	}
}

//========================
//      PathFinder
//========================
type PathFinder interface {
	Reset()
	FindPath(m NavigationMap, startGrid *Grid, dstGrid *Grid) ([]PathNode, bool)
}

type PathFinderImpl interface {
	CreateFirstNode(col int, row int) PathNode
	UnfoldGrid(m NavigationMap, dstGrid *Grid, node PathNode)
	// GetFullPath() ([]*Grid, bool)
}

//========================
//     BasePathFinder
//========================
type BasePathFinder struct {
	openList  []PathNode
	closeList []PathNode
	lastNode  PathNode
	impl      PathFinderImpl
}

func NewBasePathFinder(impl PathFinderImpl) *BasePathFinder {
	return &BasePathFinder{
		openList:  make([]PathNode, 0),
		closeList: make([]PathNode, 0),
		lastNode:  nil,
		impl:      impl,
	}
}

func (f *BasePathFinder) Reset() {
	f.openList = make([]PathNode, 0)
	f.closeList = make([]PathNode, 0)
	f.lastNode = nil
}

func (f *BasePathFinder) FindPath(m NavigationMap, startGrid *Grid, dstGrid *Grid) ([]PathNode, bool) {
	// pre check
	fullPath, bSucc, bFinish := f.preCheck(m, startGrid, dstGrid)
	if bFinish {
		return fullPath, bSucc
	}

	// add start grid to open list first
	firstNode := f.impl.CreateFirstNode(startGrid.Col, startGrid.Row)
	f.AddNodeToOpenList(firstNode)

	for {
		// no grid to search again, can't not find a path
		if len(f.openList) == 0 {
			break
		}

		node, ok := f.popMinValueNode(m, dstGrid)
		if !ok {
			break
		}

		f.impl.UnfoldGrid(m, dstGrid, node)
		if f.lastNode != nil {
			break
		}
	}

	return f.getFullPath()
}

func (f *BasePathFinder) preCheck(m NavigationMap, startGrid *Grid, dstGrid *Grid) (fullPath []PathNode, bSucc bool, bFinish bool) {
	// start grid can't cross
	if !m.CanCross(startGrid.Col, startGrid.Row) {
		return nil, false, true
	}

	// dest grid can't cross
	if !m.CanCross(dstGrid.Col, dstGrid.Row) {
		return nil, false, true
	}

	// start grid and dest grid is the same grid
	if startGrid.IsSameGrid(dstGrid) {
		node := f.impl.CreateFirstNode(startGrid.Col, startGrid.Row)
		fullPath := []PathNode{
			node,
		}

		return fullPath, true, true
	}

	return nil, false, false
}

func (f *BasePathFinder) AddNodeToOpenList(node PathNode) {
	f.openList = append(f.openList, node)
}

func (f *BasePathFinder) GetOpenNode(col int, row int) (PathNode, bool) {
	for _, node := range f.openList {
		grid := node.GetGrid()
		if grid.IsSameGrid2(col, row) {
			return node, true
		}
	}

	return nil, false
}

func (f *BasePathFinder) AddNodeToCloseList(node PathNode) {
	f.closeList = append(f.closeList, node)
}

func (f *BasePathFinder) GetCloseNode(col int, row int) (PathNode, bool) {
	for _, node := range f.closeList {
		grid := node.GetGrid()
		if grid.IsSameGrid2(col, row) {
			return node, true
		}
	}

	return nil, false
}

func (f *BasePathFinder) UpdateExistList(m NavigationMap, col int, row int, parent PathNode, vecParent *Vector, minGValue uint32) bool {
	exist, ok := f.GetOpenNode(col, row)
	if !ok {
		exist, ok = f.GetCloseNode(col, row)
	}

	if ok {
		if exist.GetMinGValue() > minGValue {
			exist.UpdateParent(parent, vecParent)
			exist.SetMinGValue(minGValue, m)
		}
	}

	return ok
}

func (f *BasePathFinder) popMinValueNode(m NavigationMap, dstGrid *Grid) (PathNode, bool) {
	minF := uint32(math.MaxUint32)
	minIdx := -1

	baseGValue := m.GetMinGValue()
	// find index of the min F value node
	for i, node := range f.openList {
		h := f.calH(node, dstGrid, baseGValue)
		f := node.GetMinGValue() + h
		if minF > f {
			minF = f
			minIdx = i
		}
	}

	if minIdx == -1 {
		return nil, false
	}

	// pop min F value node
	minNode := f.openList[minIdx]
	f.openList = append(f.openList[:minIdx], f.openList[minIdx+1:]...)

	return minNode, true
}

func (f *BasePathFinder) calH(node PathNode, dstGrid *Grid, baseGValue uint32) uint32 {
	grid := node.GetGrid()
	xAbs := math.Abs(float64(dstGrid.Col-grid.Col) * float64(baseGValue))
	yAbs := math.Abs(float64(dstGrid.Row-grid.Row) * float64(baseGValue))
	return uint32(xAbs + yAbs)
}

func (f *BasePathFinder) getFullPath() ([]PathNode, bool) {
	if f.lastNode == nil {
		return nil, false
	}

	fullPath := make([]PathNode, 0)
	node := f.lastNode
	for {
		if node == nil {
			break
		}

		// grid := node.GetGrid()
		fullPath = append(fullPath, node)
		node = node.GetParent()
	}

	// reverse
	fullPathLen := len(fullPath)
	for i, j := 0, fullPathLen-1; i < j; i, j = i+1, j-1 {
		fullPath[i], fullPath[j] = fullPath[j], fullPath[i]
	}

	return fullPath, true
}
