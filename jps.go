// Copyright 2022 Guan Jianchang. All rights reserved.
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file.

package nav

import "math"

var (
	VecStart     = NewVector(0, 0)
	VecLeft      = NewVector(-1, 0)
	VecRight     = NewVector(1, 0)
	VecUp        = NewVector(0, -1)
	VecDown      = NewVector(0, 1)
	VecLeftUp    = NewVector(-1, -1)
	VecRightUp   = NewVector(1, -1)
	VecLeftDown  = NewVector(-1, 1)
	VecRightDown = NewVector(1, 1)
)

//========================
//      JpsNode
//========================
type JpsNode struct {
	*BasePathNode
	// vecParent         *Vector
	vecNeighbour      *Vector
	bOrthogonalUnfold bool
	bObliqueUnfold    bool
	bJumpPoint        bool
}

func NewJpsNode(parent PathNode, vecParent *Vector, minGValue uint32, col int, row int, bJumpPoint bool) *JpsNode {
	return &JpsNode{
		BasePathNode: NewBasePathNode(parent, vecParent, minGValue, col, row),
		// vecParent:         vecParent,
		vecNeighbour:      nil,
		bOrthogonalUnfold: false,
		bObliqueUnfold:    false,
		bJumpPoint:        bJumpPoint,
	}
}

func (n *JpsNode) SetNeighbourVector(vec *Vector) {
	n.vecNeighbour = vec
}

func (n *JpsNode) GetNeighbourVector() *Vector {
	return n.vecNeighbour
}

func (n *JpsNode) SetOrthogonalUnfold() {
	n.bOrthogonalUnfold = true
}

func (n *JpsNode) IsOrthogonalUnfold() bool {
	return n.bOrthogonalUnfold
}

func (n *JpsNode) SetObliqueUnfold() {
	n.bObliqueUnfold = true
}

func (n *JpsNode) IsObliqueUnfold() bool {
	return n.bObliqueUnfold
}

func (n *JpsNode) SetJumpPoint() {
	n.bJumpPoint = true
}

func (n *JpsNode) IsJumpPoint() bool {
	return n.bJumpPoint
}

//========================
//      Jps
//========================
type Jps struct {
	*BasePathFinder
	maxOrthogonalDeep uint32
	canObliqueMove    bool
}

func NewJps(maxOrthogonalDeep uint32, canObliqueMove bool) *Jps {
	j := &Jps{
		maxOrthogonalDeep: maxOrthogonalDeep,
		canObliqueMove:    canObliqueMove,
	}

	j.BasePathFinder = NewBasePathFinder(j)
	return j
}

func (j *Jps) CreateFirstNode(col int, row int) PathNode {
	return NewJpsNode(nil, VecStart, 0, col, row, true)
}

func (j *Jps) UnfoldGrid(m NavigationMap, dstGrid *Grid, node PathNode) {
	defer j.AddNodeToCloseList(node)

	startNode, ok := node.(*JpsNode)
	if !ok {
		return
	}

	j.findJumpPoint(m, dstGrid, startNode)
}

func (j *Jps) findJumpPoint(m NavigationMap, dstGrid *Grid, startNode *JpsNode) {
	// Orthogonal
	if !startNode.IsOrthogonalUnfold() {
		bFindOut := j.findJumpPointOrthogonal(m, dstGrid, startNode)
		startNode.SetOrthogonalUnfold()
		if bFindOut && j.tryAddToOpenList(startNode) {
			return
		}
	}

	// oblique unfold
	if !startNode.IsObliqueUnfold() {
		j.findJumpPointOblique(m, dstGrid, startNode)
		startNode.SetObliqueUnfold()
	}
}

func (j *Jps) findJumpPointOrthogonal(m NavigationMap, dstGrid *Grid, startNode *JpsNode) bool {
	bFindOut := false
	orthogonalVectors := j.getOrthogonalVectors(startNode)
	for _, vec := range orthogonalVectors {
		ok := j.findJumpPointLoop(m, dstGrid, startNode, vec)
		bFindOut = (bFindOut || ok)
		if j.lastNode != nil {
			break
		}
	}

	return bFindOut
}

func (j *Jps) getOrthogonalVectors(startNode *JpsNode) []*Vector {
	vectors := make([]*Vector, 0)
	vecParent := startNode.GetParentVector()
	if vecParent == VecStart || vecParent.X > 0 {
		vectors = append(vectors, VecRight)
	}

	if vecParent == VecStart || vecParent.X < 0 {
		vectors = append(vectors, VecLeft)
	}

	if vecParent == VecStart || vecParent.Y > 0 {
		vectors = append(vectors, VecDown)
	}

	if vecParent == VecStart || vecParent.Y < 0 {
		vectors = append(vectors, VecUp)
	}

	return vectors
}

func (j *Jps) findJumpPointLoop(m NavigationMap, dstGrid *Grid, startNode *JpsNode, vec *Vector) bool {
	maxCol, maxRow := m.GetColRow()
	endCol := int(maxCol - 1)
	if vec.X < 0 {
		endCol = 0
	}

	endRow := int(maxRow - 1)
	if vec.Y < 0 {
		endRow = 0
	}

	bFind := false
	// bFindDst := false
	gValue := startNode.GetMinGValue()
	grid := startNode.GetGrid()
	col := grid.Col
	row := grid.Row
	if startNode.IsJumpPoint() {
		col = grid.Col + vec.X
		row = grid.Row + vec.Y
		gValue += m.GetGValue(col, row)
	}

	for {
		vecParent := vec
		if grid.IsSameGrid2(col, row) {
			vecParent = startNode.GetParentVector()
		}

		// find dest
		if dstGrid.IsSameGrid2(col, row) {
			j.handleFindout(m, startNode, vecParent, nil, col, row, gValue, true)
			bFind = true
			// bFindDst = true
			break
		}

		// can't find any more
		if !m.CanCross(col, row) {
			break
		}

		// end
		if (vec.X != 0 && col == endCol) || (vec.Y != 0 && row == endRow) {
			break
		}

		// find jump point
		vecNeighbour, ok := j.getNeighbour(m, vecParent, col, row)
		if ok {
			// println("startNode(", grid.Col, ", ", grid.Row, ")")
			// println("grid(", col, ", ", row, ") vec: ", vecNeighbour.X, ", ", vecNeighbour.Y)
			j.handleFindout(m, startNode, vecParent, vecNeighbour, col, row, gValue, false)
			bFind = true

			if !grid.IsSameGrid2(col, row) {
				break
			}
		}

		col += vec.X
		row += vec.Y
		gValue += m.GetGValue(col, row)
	}

	return bFind
}

func (j *Jps) getNeighbour(m NavigationMap, vecParent *Vector, col int, row int) (*Vector, bool) {
	grid := NewGrid(col, row)

	// right up
	if vecParent.Y == -1 && vecParent.X <= 0 {
		if !m.CanCross(col+1, row) && j.canMoveOblique(m, grid, col+1, row-1) {
			return VecRightUp, true
		}
	}

	if vecParent.X == 1 && vecParent.Y >= 0 {
		if !m.CanCross(col, row-1) && j.canMoveOblique(m, grid, col+1, row-1) {
			return VecRightUp, true
		}
	}

	// right down
	if vecParent.Y == 1 && vecParent.X <= 0 {
		if !m.CanCross(col+1, row) && j.canMoveOblique(m, grid, col+1, row+1) {
			return VecRightDown, true
		}
	}

	if vecParent.X == 1 && vecParent.Y <= 0 {
		if !m.CanCross(col, row+1) && j.canMoveOblique(m, grid, col+1, row+1) {
			return VecRightDown, true
		}
	}

	// left up
	if vecParent.Y == -1 && vecParent.X >= 0 {
		if !m.CanCross(col-1, row) && j.canMoveOblique(m, grid, col-1, row-1) {
			return VecLeftUp, true
		}
	}

	if vecParent.X == -1 && vecParent.Y >= 0 {
		if !m.CanCross(col, row-1) && j.canMoveOblique(m, grid, col-1, row-1) {
			return VecLeftUp, true
		}
	}

	// left down
	if vecParent.Y == 1 && vecParent.X >= 0 {
		if !m.CanCross(col-1, row) && j.canMoveOblique(m, grid, col-1, row+1) {
			return VecLeftDown, true
		}
	}

	if vecParent.X == -1 && vecParent.Y <= 0 {
		if !m.CanCross(col, row+1) && j.canMoveOblique(m, grid, col-1, row+1) {
			return VecLeftDown, true
		}
	}

	return nil, false
}

func (j *Jps) handleFindout(m NavigationMap, startNode *JpsNode, vecParent *Vector, vecNeighbour *Vector, col int, row int, gValue uint32, bFindDst bool) {
	startNode.SetJumpPoint()
	startGrid := startNode.GetGrid()
	node := startNode
	if startGrid.IsSameGrid2(col, row) {
		if j.UpdateExistList(m, col, row, startNode.GetParent(), vecParent, gValue) {
			return
		}

	} else {
		if j.UpdateExistList(m, col, row, startNode, vecParent, gValue) {
			return
		}

		node = NewJpsNode(startNode, vecParent, gValue, col, row, true)
		j.AddNodeToOpenList(node)
	}

	node.SetNeighbourVector(vecNeighbour)

	if bFindDst {
		j.lastNode = node
	}
}

func (j *Jps) tryAddToOpenList(node *JpsNode) bool {
	grid := node.GetGrid()
	_, ok := j.GetOpenNode(grid.Col, grid.Row)
	if !ok {
		j.AddNodeToOpenList(node)
		return true
	}

	return false
}

func (j *Jps) findJumpPointOblique(m NavigationMap, dstGrid *Grid, startNode *JpsNode) {
	obliqueVectors := j.getNextObliqueVectors(startNode)
	for _, vec := range obliqueVectors {
		node, ok := j.findNextGridOblique(m, dstGrid, startNode, vec)
		if !ok {
			if j.lastNode != nil {
				break
			}

			continue
		}

		j.findJumpPoint(m, dstGrid, node)
		if j.lastNode != nil {
			break
		}
	}
}

func (j *Jps) getOVectorsForObliqueParent(startNode *JpsNode) []*Vector {
	vectors := make([]*Vector, 0)
	vec := startNode.GetParentVector()
	vectors = append(vectors, vec)

	if !startNode.IsJumpPoint() {
		return vectors
	}

	vecNeighbour := startNode.GetNeighbourVector()
	if vecNeighbour == nil {
		return vectors
	}

	if vecNeighbour.X != vec.X || vecNeighbour.Y != vec.Y {
		vectors = append(vectors, vecNeighbour)
	}

	return vectors
}

func (j *Jps) getNextObliqueVectors(startNode *JpsNode) []*Vector {
	vec := startNode.GetParentVector()
	if vec.IsOblique() {
		return j.getOVectorsForObliqueParent(startNode)
	}

	vectors := make([]*Vector, 0)
	if vec.IsEmptyVector() || vec.X == -1 || vec.Y == -1 {
		vectors = append(vectors, VecLeftUp)
	}

	if vec.IsEmptyVector() || vec.X == -1 || vec.Y == 1 {
		vectors = append(vectors, VecLeftDown)
	}

	if vec.IsEmptyVector() || vec.X == 1 || vec.Y == -1 {
		vectors = append(vectors, VecRightUp)
	}

	if vec.IsEmptyVector() || vec.X == 1 || vec.Y == 1 {
		vectors = append(vectors, VecRightDown)
	}

	return vectors
}

func (j *Jps) findNextGridOblique(m NavigationMap, dstGrid *Grid, startNode *JpsNode, vec *Vector) (*JpsNode, bool) {
	parent := j.getParentNode(startNode)
	grid := startNode.GetGrid()
	nextCol := grid.Col + vec.X
	nextRow := grid.Row + vec.Y

	// can't cross
	minGValue := j.getMinGValueOblique(m, grid, nextCol, nextRow)
	if minGValue == math.MaxUint32 {
		return nil, false
	}

	// find dest grid
	if dstGrid.IsSameGrid2(nextCol, nextRow) {
		j.lastNode = NewJpsNode(parent, vec, 0, nextCol, nextRow, true)
		return nil, false
	}

	// exist in open or closed list
	vecParent := parent.GetParentVector()
	if j.UpdateExistList(m, nextCol, nextRow, parent, vecParent, minGValue) {
		return nil, false
	}

	return NewJpsNode(parent, vec, minGValue, nextCol, nextRow, false), true
}

func (j *Jps) getParentNode(preNode *JpsNode) PathNode {
	var parent PathNode = preNode
	if !preNode.IsJumpPoint() {
		parent = preNode.GetParent()
	}

	return parent
}

func (j *Jps) canMoveOblique(m NavigationMap, parent *Grid, col int, row int) bool {
	return j.getMinGValueOblique(m, parent, col, row) != math.MaxUint32
}

func (j *Jps) getMinGValueOblique(m NavigationMap, parent *Grid, col int, row int) uint32 {
	if !m.CanCross(col, row) {
		return math.MaxUint32
	}

	if j.canObliqueMove {
		return m.GetGValue(col, row)
	}

	addGValue := uint32(math.MaxUint32)
	if m.CanCross(col, parent.Row) {
		addGValue = m.GetGValue(col, parent.Row)
	}

	if m.CanCross(parent.Col, row) {
		gValue := m.GetGValue(parent.Col, row)
		if addGValue > gValue {
			addGValue = gValue
		}
	}

	if addGValue == math.MaxUint32 {
		return math.MaxUint32
	}

	return addGValue + m.GetGValue(col, row)
}
