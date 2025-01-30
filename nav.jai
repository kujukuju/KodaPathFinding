
// features
// jumping
// have connectivity groupings to know if its even possible to go from one place to another
// visualize every cell its visiting... maybe with color gradient

// optimizations
// optimize integer abs???
// grid jump algorithm thing
// convex visibility cell groupings with more jumps
// maybe disallow moving diagonally once I add in the system to straighten out the lines
// unsorted stack for the current lowest value node (in grids)

// normal heuristic checking a*
// Nav path find took 109.1907 ms.

// notes
//
// diagonal jump points are just how far you have to move diagonally before you run into one of the straight
// jump points for at least one of the diagonal direction components
//
// so by extension I assume corner jump points are the same and only consider all possible straight jump points
// for at least one of the corner direction components
//
// this is a very important part that I kept confusing and it prevented me from making progress in 3d jps because
// I was trying to allow both diagonals and corners to be checked, but this goes against the premise of the algorithm
// and so inherently I'm pretty sure that you are implicitly not allowed to move diagonally around corners in real jps
// although I think in 2d you can hack it in various ways, so a lot of 2d examples online show diagonal
// forced neighbor movement
//
// in 2d when you're moving +x and you detect a forced neighbor causing you to move -y, you then have to move +x, +x-y, and -y
// in 3d the logical of diagonals is the same, however the logic of corners only requires you to move along that corner and
// not the diagonals. so for example if you're moving +x and you detect a forced neighbor causing you to move +y+z, you then
// have to move +x, +y+z, and +x+y+z. you don't consider any of the diagonal permutations as those rely on the diagonal
// forced neighbors
//
// I think I need to simplify the forced neighbor logic in 2d then try to apply it to 3d, because corners actually still don't seem solved
// so when moving in +x, you check if cell+(0,1) is empty and cell+(-1,1) is a wall. then this is an open node
// so it's +ortho direction must be empty, and +ortho direction-travel direction must be a wall.
//
// in 3d we can maybe say that a +x line has y and z ortho directions only
// while a +x+y diagonal has z ortho direction only
// and +x+y+z diagonal has... not sure
//
// so taking a step back the idea of why a forced neighbor is required is because you have to step around an obstracle IN YOUR
// DIRECTION OF TRAVEL to get the cell in question
// the implication of this is that for each corner you need to spawn 3 diagonals and 3 lines so that these can all
// do their own respective checks, and then each type of thing is responsible for only their own forced neighbor directions
// so in a line, that's checking either the 8 adjacent cells or the 4 adjacent cells, I don't yet know
//
// or you can break it down further and add more rules which might be smarter
// to say for a line moving in +x you check +/-y and +/-z and move in those directions
// also for a line moving in +x you check +y, +z, +y+z, and if all those are forced neighbors
// then you also move in that corner direction
// this allows you to still move in corner directions from the actual perspective of a line, which is the premise of the algorithm
// and this still respects the rule that seems to be emerging that you can't travel diagonally through cells
//
// you could say a problem with this is if you're travelling +x+y and there is a whole row of +x+y walls to your z or xz or yz, you wouldn't consider these
// forced neighbors because they're not adjacent. however I think this is fine because inherently the cells adjacent to this wall will get visited somehow
// and you will then detect them as forced neighbors
//
// the remaining problem though is that I expect there is some situation where it's actually more efficient to travel along parallel with this wall to
// your diagonal, and then move sqrt(2) to be in front of it, if for example you were treating it as a forced neighbor
// actually though this isn't true as long as you're creating your lines from the exact cell you're currently on, given that you can't move through filled diagonals
// because your diagonal will create lines, and those lines will end up being adjacent to your current parallel +x+y wall
// in fact you could even say this example is the same as 2d, and also it's pretty bad because you would have a forced neighbor along every single cell I think
//
// so I think from this example, assuming you can only move around objects in full steps of cells, not diagonally in any way, you're done
//
// lastly you're actually allowed to check if a cell is the end without waiting for the next pop because
// you're only allowed to move in straight lines, so whatever parent cell finds the end point is inherently the closest since the distance is exactly included
// in the hueristic
//
// lastly because we're not essentially baking the jump distances, either conceptually through a function or literally, we have to manually add potential jumps
// based on when something aligns in any axis with the end goal

// instructions
// check if the start is the end and return
// add the start cell to the open set
// pop the open set
//     check the line jump distances
//     maybe jump and add to the open set
//     check the diagonal jump distances
//     maybe jump an add to the open set
//     check the corner jump distances
//     maybe jump and add to the open set
// check if you're at the end
// add the start cell to the open set
// add the

NavPath :: struct {
    path: [..] SparseIndex;
    path_groups: [..] u16;
}

NavInfo :: struct (T: Type) {
    walkable_type: T;
    empty_type: T;
}

SearchPoint :: struct {
    cell: SparseIndex;
    distance_heuristic: float;
}

search_points: [..] SearchPoint;

performance_test :: true;

path_find_jps :: (sparse: *GroupedSparseGrid($T), source_point: Vector3, target_point: Vector3, wall_type: T) -> NavPath {
    source_cell := get_cell_index(*sparse.grid, source_point);
    target_cell := get_cell_index(*sparse.grid, target_point);

    return path_find_jps(sparse, source_cell, target_cell, wall_type);
}

path_find_jps :: (sparse: *GroupedSparseGrid($T), source_cell: SparseIndex, target_cell: SparseIndex, wall_type: T) -> NavPath {
    seconds_since_init :: () -> float64 {
        return to_float64_seconds(current_time_consensus());
    }

    StepType :: enum u8 { Line; Diagonal; Corner; }

    // this is implying that 1 maps to index 0
    ForcedOffsets :: enum_flags u32 {
        None :: 0;
        PX :: 1;
        NX;
        PY; NY;
        PZ; NZ;
        PXPZ; PXNZ; NXPZ; NXNZ;
        PXPY; PXNY; NXPY; NXNY;
        PYPZ; PYNZ; NYPZ; NYNZ;
        PXPYPZ; PXPYNZ; PXNYPZ; PXNYNZ;
        NXPYPZ; NXPYNZ; NXNYPZ; NXNYNZ;
    }

    offsets :: SparseDir.[
        .{1, 0, 0}/*0*/, .{-1, 0, 0}/*1*/,
        .{0, 1, 0}/*2*/, .{0, -1, 0}/*3*/,
        .{0, 0, 1}/*4*/, .{0, 0, -1}/*5*/,
        .{1, 0, 1}/*6*/, .{1, 0, -1}/*7*/, .{-1, 0, 1}/*8*/, .{-1, 0, -1}/*9*/,
        .{1, 1, 0}/*10*/, .{1, -1, 0}/*11*/, .{-1, 1, 0}/*12*/, .{-1, -1, 0}/*13*/,
        .{0, 1, 1}/*14*/, .{0, 1, -1}/*15*/, .{0, -1, 1}/*16*/, .{0, -1, -1}/*17*/,
        .{1, 1, 1}/*18*/, .{1, 1, -1}/*19*/, .{1, -1, 1}/*20*/, .{1, -1, -1}/*21*/,
        .{-1, 1, 1}/*22*/, .{-1, 1, -1}/*23*/, .{-1, -1, 1}/*24*/, .{-1, -1, -1}/*25*/,
    ];

    offset_lookup :: [3] [3] ForcedOffsets.[
        .[
            .[.NXNYNZ, .NXNY, .NXNYPZ],
            .[.NXNZ, .NX, .NXPZ],
            .[.NXPYNZ, .NXPY, .NXPYPZ],
        ], .[
            .[.NYNZ, .NY, .NYPZ],
            .[.NZ, .None, .PZ],
            .[.PYNZ, .PY, .PYPZ],
        ], .[
            .[.PXNYNZ, .PXNY, .PXNYPZ],
            .[.PXNZ, .PX, .PXPZ],
            .[.PXPYNZ, .PXPY, .PXPYPZ],
        ],
    ];

    get_offset :: (offset: SparseDir) -> ForcedOffsets {
        return offset_lookup[offset.x + 1][offset.y + 1][offset.z + 1];
    }

    step_types :: StepType.[
        .Line, .Line,
        .Line, .Line,
        .Line, .Line,
        .Diagonal, .Diagonal, .Diagonal, .Diagonal,
        .Diagonal, .Diagonal, .Diagonal, .Diagonal,
        .Diagonal, .Diagonal, .Diagonal, .Diagonal,
        .Corner, .Corner, .Corner, .Corner,
        .Corner, .Corner, .Corner, .Corner,
    ];

    // diagonal_line_directions :: [3] [3] [2] SparseDir.[
    //     .[
    //         .[.[.{}, .{}], .[.{}, .{}], .[.{}, .{}]],
    //         .[.[.{}, .{}], .[.{}, .{}], .[.{}, .{}]],
    //         .[.[.{}, .{}], .[.{}, .{}], .[.{}, .{}]],
    //     ], .[
    //         .[.[.{}, .{}], .[.{}, .{}], .[.{}, .{}]],
    //         .[.[.{}, .{}], .[.{}, .{}], .[.{}, .{}]],
    //         .[.[.{}, .{}], .[.{}, .{}], .[.{}, .{}]],
    //     ], .[
    //         .[.[.{}, .{}], .[.{}, .{}], .[.{}, .{}]],
    //         .[.[.{}, .{}], .[.{}, .{}], .[.{}, .{}]],
    //         .[.[.{}, .{}], .[.{}, .{}], .[.{}, .{}]],
    //     ],
    // ];

    diagonal_line_offset_indices :: [2] u8.[
        .[0, 0], .[0, 0],
        .[0, 0], .[0, 0],
        .[0, 0], .[0, 0],
        .[0, 4], .[0, 5], .[1, 4], .[1, 5],
        .[0, 2], .[0, 3], .[1, 2], .[1, 3],
        .[2, 4], .[2, 5], .[3, 4], .[3, 5],
        .[0, 0], .[0, 0], .[0, 0], .[0, 0],
        .[0, 0], .[0, 0], .[0, 0], .[0, 0],
    ];

    corner_diagonal_offset_indices :: [3] u8.[
        .[0, 0, 0], .[0, 0, 0],
        .[0, 0, 0], .[0, 0, 0],
        .[0, 0, 0], .[0, 0, 0],
        .[0, 0, 0], .[0, 0, 0], .[0, 0, 0], .[0, 0, 0],
        .[0, 0, 0], .[0, 0, 0], .[0, 0, 0], .[0, 0, 0],
        .[0, 0, 0], .[0, 0, 0], .[0, 0, 0], .[0, 0, 0],
        .[10, 6, 14], .[10, 7, 15], .[11, 6, 16], .[11, 7, 17],
        .[12, 8, 14], .[12, 9, 15], .[13, 8, 16], .[13, 9, 17],
    ];

    corner_line_offset_indices :: [3] u8.[
        .[0, 0, 0], .[0, 0, 0],
        .[0, 0, 0], .[0, 0, 0],
        .[0, 0, 0], .[0, 0, 0],
        .[0, 0, 0], .[0, 0, 0], .[0, 0, 0], .[0, 0, 0],
        .[0, 0, 0], .[0, 0, 0], .[0, 0, 0], .[0, 0, 0],
        .[0, 0, 0], .[0, 0, 0], .[0, 0, 0], .[0, 0, 0],
        .[0, 2, 4], .[0, 2, 5], .[0, 3, 4], .[0, 3, 5],
        .[1, 2, 4], .[1, 2, 5], .[1, 3, 4], .[1, 3, 5],
    ];

    is_wall :: (cell_type: T) -> bool #expand {
        return !!(cell_type & `wall_type);
    }

    is_empty :: (call_type: T) -> bool #expand {
        return !is_wall(cell_type);
    }

    calculate_end_distance :: (sparse: *GroupedSparseGrid(T), cell: SparseIndex, direction: SparseDir, wall_type: T) -> s8 #expand {
        // im going to do this such that positive numbers are forced neighbor distances
        // negative numbers are wall distances
        // and S8_MAX is a forced neighbor continuation indicator

        for i: cast(s8) 1..S8_MAX {
            check_cell := cell + direction * i;

            jump_point, wall_point := is_jump_point(sparse, cell, direction, wall_type);
            if jump_point {
                return i;
            } else if wall_point {
                return -i;
            }

            cell_type := get_cell(sparse, check_cell, *`cached_sparse_grid);
            if !!(cell_type & wall_type) {
                return -i;
            }
        }

        return S8_MAX;
    }

    calculate_end_distances :: (sparse: *GroupedSparseGrid(T), cell: SparseIndex, wall_type: T) -> [24] s8 #expand {
        distances: [24] s8;

        for offset: offsets {
            distances[it_index] = calculate_end_distance(sparse, cell, offset, wall_type);
        }

        return distances;
    }

    get_end_distance :: (sparse: *GroupedSparseGrid(T), cell: SparseIndex, direction: SparseDir, wall_type: T) -> s32 #expand {
        distance := calculate_end_distance(sparse, cell, direction, wall_type);
        acc_distance := cast(s32) distance;

        while distance == S8_MAX {
            current_cell := cell + direction * acc_distance;

            distance = calculate_end_distance(sparse, current_cell, direction, wall_type);
            acc_distance += abs(distance);

            if distance < 0 {
                acc_distance = -acc_distance;
            }
        }

        return acc_distance;
    }

    is_jump_point :: (sparse: *GroupedSparseGrid(T), cell: SparseIndex, direction: SparseDir, wall_type: T) -> jump: bool, wall: bool #expand {
        is_jump_point_line :: (sparse: *GroupedSparseGrid(T), cell: SparseIndex, direction: SparseDir, wall_type: T) -> jump: bool, wall: bool #expand {
            assert(abs(direction.x) + abs(direction.y) + abs(direction.z) == 1, "Line jump point check must have 1 component.\n");

            // the first cell you check CAN be a forced neighbor jump point
            // the first cell you check CANNOT be a wall

            current_cell := cell;

            if is_forced_neighbor(sparse, current_cell, direction, wall_type) {
                return true, false;
            }

            while true {
                current_cell += direction;

                if !!(get_cell(sparse, current_cell, *`cached_grid) & wall_type) {
                    return false, true;
                }

                if is_forced_neighbor(sparse, current_cell, direction, wall_type) {
                    return true, false;
                }
            }

            return false, false;
        }

        is_jump_point_diagonal :: (sparse: *GroupedSparseGrid(T), cell: SparseIndex, direction: SparseDir, wall_type: T) -> jump: bool, wall: bool #expand {
            assert(abs(direction.x) + abs(direction.y) + abs(direction.z) == 2, "Diagonal jump point check must have 2 components.\n");

            line_directions: [2] SparseDir = ---;

            if direction.x == 0 {
                line_directions = .[.{0, direction.y, 0}, .{0, 0, direction.z}];
            } else if direction.y == 0 {
                line_directions = .[.{direction.z, 0, 0}, .{0, 0, direction.z}];
            } else if direction.z == 0 {
                line_directions = .[.{direction.z, 0, 0}, .{0, direction.y, 0}];
            }

            jump_point, wall_point := is_jump_point_line(sparse, cell, line_directions[0], wall_type);
            if jump_point || wall_point {
                return jump_point, wall_point;
            }
            jump_point, wall_point = is_jump_point_line(sparse, cell, line_directions[1], wall_type);
            return jump_point, wall_point;
        }

        is_jump_point_corner :: (sparse: *GroupedSparseGrid(T), cell: SparseIndex, direction: SparseDir, wall_type: T) -> jump: bool, wall: bool #expand {
            assert(abs(direction.x) + abs(direction.y) + abs(direction.z) == 3, "Corner jump point check must have 3 components.\n");

            diagonal_directions := SparseDir.[
                .{direction.x, direction.y, 0},
                .{direction.x, 0, direction.z},
                .{0, direction.y, direction.z},
            ];

            jump_point, wall_point := is_jump_point_diagonal(sparse, cell, diagonal_directions[0], wall_type);
            if jump_point || wall_point {
                return jump_point, wall_point;
            }
            jump_point, wall_point = is_jump_point_diagonal(sparse, cell, diagonal_directions[1], wall_type);
            if jump_point || wall_point {
                return jump_point, wall_point;
            }
            jump_point, wall_point = is_jump_point_diagonal(sparse, cell, diagonal_directions[2], wall_type);
            return jump_point, wall_point;
        }

        component_count := abs(direction.x) + abs(direction.y) + abs(direction.z);
        if component_count == {
            case 1;
                jump_point, wall_point := is_jump_point_line(sparse, cell, direction, wall_type);
                return jump_point, wall_point;
            case 2;
                jump_point, wall_point := is_jump_point_diagonal(sparse, cell, direction, wall_type);
                return jump_point, wall_point;
            case 3;
                jump_point, wall_point := is_jump_point_corner(sparse, cell, direction, wall_type);
                return jump_point, wall_point;
        }

        return false, false;
    }

    is_forced_neighbor :: (sparse: *GroupedSparseGrid(T), cell: SparseIndex, direction: SparseDir, wall_type: T) -> ForcedOffsets, bool #expand {
        assert(abs(direction.x) + abs(direction.y) + abs(direction.z) == 1, "You can only check for forced neighbors of a line.\n");

        // we assume in this function the cell you're checking must be empty

        jump_point_empty_cells: [8] T = ---;
        jump_point_wall_cells: [8] T = ---;

        if direction.x != 0 {
            jump_point_empty_cells = .[
                get_cell(sparse, cell + .{0, -1, -1}, *`cached_grid),
                get_cell(sparse, cell + .{0, -1, 0}, *`cached_grid),
                get_cell(sparse, cell + .{0, -1, 1}, *`cached_grid),
                get_cell(sparse, cell + .{0, 0, -1}, *`cached_grid),
                get_cell(sparse, cell + .{0, 0, 1}, *`cached_grid),
                get_cell(sparse, cell + .{0, 1, -1}, *`cached_grid),
                get_cell(sparse, cell + .{0, 1, 0}, *`cached_grid),
                get_cell(sparse, cell + .{0, 1, 1}, *`cached_grid),
            ];
            jump_point_wall_cells = .[
                get_cell(sparse, cell + .{-direction.x, -1, -1}, *`cached_grid),
                get_cell(sparse, cell + .{-direction.x, -1, 0}, *`cached_grid),
                get_cell(sparse, cell + .{-direction.x, -1, 1}, *`cached_grid),
                get_cell(sparse, cell + .{-direction.x, 0, -1}, *`cached_grid),
                get_cell(sparse, cell + .{-direction.x, 0, 1}, *`cached_grid),
                get_cell(sparse, cell + .{-direction.x, 1, -1}, *`cached_grid),
                get_cell(sparse, cell + .{-direction.x, 1, 0}, *`cached_grid),
                get_cell(sparse, cell + .{-direction.x, 1, 1}, *`cached_grid),
            ];
        } else if direction.y != 0 {
            jump_point_empty_cells = .[
                get_cell(sparse, cell + .{-1, 0, -1}, *`cached_grid),
                get_cell(sparse, cell + .{-1, 0, 0}, *`cached_grid),
                get_cell(sparse, cell + .{-1, 0, 1}, *`cached_grid),
                get_cell(sparse, cell + .{0, 0, -1}, *`cached_grid),
                get_cell(sparse, cell + .{0, 0, 1}, *`cached_grid),
                get_cell(sparse, cell + .{1, 0, -1}, *`cached_grid),
                get_cell(sparse, cell + .{1, 0, 0}, *`cached_grid),
                get_cell(sparse, cell + .{1, 0, 1}, *`cached_grid),
            ];
            jump_point_wall_cells = .[
                get_cell(sparse, cell + .{-1, -direction.y, -1}, *`cached_grid),
                get_cell(sparse, cell + .{-1, -direction.y, 0}, *`cached_grid),
                get_cell(sparse, cell + .{-1, -direction.y, 1}, *`cached_grid),
                get_cell(sparse, cell + .{0, -direction.y, -1}, *`cached_grid),
                get_cell(sparse, cell + .{0, -direction.y, 1}, *`cached_grid),
                get_cell(sparse, cell + .{1, -direction.y, -1}, *`cached_grid),
                get_cell(sparse, cell + .{1, -direction.y, 0}, *`cached_grid),
                get_cell(sparse, cell + .{1, -direction.y, 1}, *`cached_grid),
            ];
        }  else if direction.z != 0 {
            jump_point_empty_cells = .[
                get_cell(sparse, cell + .{-1, -1, 0}, *`cached_grid),
                get_cell(sparse, cell + .{-1, 0, 0}, *`cached_grid),
                get_cell(sparse, cell + .{-1, 1, 0}, *`cached_grid),
                get_cell(sparse, cell + .{0, -1, 0}, *`cached_grid),
                get_cell(sparse, cell + .{0, 1, 0}, *`cached_grid),
                get_cell(sparse, cell + .{1, -1, 0}, *`cached_grid),
                get_cell(sparse, cell + .{1, 0, 0}, *`cached_grid),
                get_cell(sparse, cell + .{1, 1, 0}, *`cached_grid),
            ];
            jump_point_wall_cells = .[
                get_cell(sparse, cell + .{-1, -1, -direction.z}, *`cached_grid),
                get_cell(sparse, cell + .{-1, 0, -direction.z}, *`cached_grid),
                get_cell(sparse, cell + .{-1, 1, -direction.z}, *`cached_grid),
                get_cell(sparse, cell + .{0, -1, -direction.z}, *`cached_grid),
                get_cell(sparse, cell + .{0, 1, -direction.z}, *`cached_grid),
                get_cell(sparse, cell + .{1, -1, -direction.z}, *`cached_grid),
                get_cell(sparse, cell + .{1, 0, -direction.z}, *`cached_grid),
                get_cell(sparse, cell + .{1, 1, -direction.z}, *`cached_grid),
            ];
        }

        corner_indices_groups :: [3] u8.[
            // actual corner, adjacent cell, adjacent cell
            .[0, 1, 3],
            .[2, 1, 3],
            .[5, 6, 3],
            .[7, 6, 4],
        ];

        diagonal_indices :: u8.[1, 3, 4, 6];

        forced_offsets: ForcedOffsets;

        for corner_indices: corner_indices_groups {
            forced_neighbor := is_empty(jump_point_empty_cells[corner_indices[0]]);
            forced_neighbor = forced_neighbor && is_empty(jump_point_empty_cells[corner_indices[1]]);
            forced_neighbor = forced_neighbor && is_empty(jump_point_empty_cells[corner_indices[2]]);
            forced_neighbor = forced_neighbor && is_wall(jump_point_wall_cells[corner_indices[0]]);


        }

        if !!(jump_point_empty_cells

        get_forced_neighbor_pairs :: (direction: SparseDir, $N: int) -> [N] [2] SparseDir {
            result: [N] [2] SparseDir;

            if direction.x == 0 && direction.y == 0 && direction.z == 0 {
                return result;
            }

            index := 0;

            for x: -1..1 {
                for y: -1..1 {
                    for z: -1..1 {
                        matches_or_zero := x == direction.x || x == 0;
                        matches_or_zero = matches_or_zero && (y == direction.y || y == 0);
                        matches_or_zero = matches_or_zero && (z == direction.z || z == 0);
                        if matches_or_zero {
                            continue;
                        }

                        matches_one := x != 0 && x == direction.x;
                        matches_one = matches_one || (y != 0 && y == direction.y);
                        matches_one = matches_one || (z != 0 && z == direction[2]);

                        if !matches_one {
                            continue;
                        }

                        // at this point we've found a relevant empty cell
                        result[index][0] = .{cast(s32) x, cast(s32) y, cast(s32) z};
                        if direction.x != 0 then result[index][0].x = 0;
                        if direction.y != 0 then result[index][0].y = 0;
                        if direction[2] != 0 then result[index][0].z = 0;

                        result[index][1] = .{cast(s32) x, cast(s32) y, cast(s32) z};

                        index += 1;
                    }
                }
            }

            assert(index == N, "Did not fill up the forced neighbor pairs.\n");

            return result;
        }

        get_forced_neighbor_pairs :: (directions: [$S] SparseDir, $N: int) -> [S] [N] [2] SparseDir {
            result: [S] [N] [2] SparseDir;

            for direction: directions {
                result[it_index] = get_forced_neighbor_pairs(direction, N);
            }

            return result;
        }

        line_forced_neighbor_pairs :: #run get_forced_neighbor_pairs(.[
            offsets[0], offsets[1],
            offsets[2], offsets[3],
            offsets[4], offsets[5],
            .{}, .{}, .{}, .{},
            .{}, .{}, .{}, .{},
            .{}, .{}, .{}, .{},
            .{}, .{}, .{}, .{},
            .{}, .{}, .{}, .{},
        ], 8);

        diagonal_forced_neighbor_pairs :: #run get_forced_neighbor_pairs(.[
            .{}, .{},
            .{}, .{},
            .{}, .{},
            offsets[6], offsets[7], offsets[8], offsets[9],
            offsets[10], offsets[11], offsets[12], offsets[13],
            offsets[14], offsets[15], offsets[16], offsets[17],
            .{}, .{}, .{}, .{},
            .{}, .{}, .{}, .{},
        ], 12);

        corner_forced_neighbor_pairs :: #run get_forced_neighbor_pairs(.[
            .{}, .{},
            .{}, .{},
            .{}, .{},
            .{}, .{}, .{}, .{},
            .{}, .{}, .{}, .{},
            .{}, .{}, .{}, .{},
            offsets[18], offsets[19], offsets[20], offsets[21],
            offsets[22], offsets[23], offsets[24], offsets[25],
        ], 12);

        forced_offsets := ForcedOffsets.None;

        step_type := step_types[offset_index];
        if #complete step_type == {
            case .Line;
                forced_neighbor_pairs := line_forced_neighbor_pairs[offset_index];
                for forced_neighbor_pair: forced_neighbor_pairs {
                    trav_cell := cell + forced_neighbor_pair[0];
                    wall_cell := cell + forced_neighbor_pair[1];

                    if get_cell(sparse, trav_cell, cached_grid) != wall_type && get_cell(sparse, wall_cell, cached_grid) == wall_type {
                        forced_offset := forced_neighbor_pair[1];

                        forced_offsets |= offset_lookup[forced_offset.x + 1][forced_offset.y + 1][forced_offset.z + 1];
                    }
                }

            case .Diagonal;
                forced_neighbor_pairs := diagonal_forced_neighbor_pairs[offset_index];
                for forced_neighbor_pair: forced_neighbor_pairs {
                    trav_cell := cell + forced_neighbor_pair[0];
                    wall_cell := cell + forced_neighbor_pair[1];

                    if get_cell(sparse, trav_cell, cached_grid) != wall_type && get_cell(sparse, wall_cell, cached_grid) == wall_type {
                        forced_offset := forced_neighbor_pair[1];

                        forced_offsets |= offset_lookup[forced_offset.x + 1][forced_offset.y + 1][forced_offset.z + 1];
                    }
                }

            case .Corner;
                forced_neighbor_pairs := corner_forced_neighbor_pairs[offset_index];
                for forced_neighbor_pair: forced_neighbor_pairs {
                    trav_cell := cell + forced_neighbor_pair[0];
                    wall_cell := cell + forced_neighbor_pair[1];

                    if get_cell(sparse, trav_cell, cached_grid) != wall_type && get_cell(sparse, wall_cell, cached_grid) == wall_type {
                        forced_offset := forced_neighbor_pair[1];

                        forced_offsets |= offset_lookup[forced_offset.x + 1][forced_offset.y + 1][forced_offset.z + 1];
                    }
                }
        }

        return forced_offsets, !!forced_offsets;
    }

    ForcedEntry :: struct {
        cell: SparseIndex;
        travelled_distance: float;
        heuristic_distance: float;
        forced_offsets: ForcedOffsets;
    }

    sorted_insert :: (array: *LinkedArray(ForcedEntry), entry: ForcedEntry) {
        linked_insert_sorted(array, entry, (a: ForcedEntry, b: ForcedEntry) -> float {
            return b.heuristic_distance - a.heuristic_distance;
        });
    }

    start_cell := source_cell;
    maximum_fall_cells :: 40;
    for 0..maximum_fall_cells - 1 {
        if get_cell(sparse, start_cell) != .Empty {
            break;
        }

        start_cell.y -= 1;
    }
    if get_cell(sparse, start_cell) == .Empty {
        print("No source ground found.\n");
        return .{};
    }

    end_cell := target_cell;
    for 0..maximum_fall_cells - 1 {
        if get_cell(sparse, end_cell) != .Empty {
            break;
        }

        end_cell.y -= 1;
    }
    if get_cell(sparse, end_cell) == .Empty {
        print("No target ground found.\n");
        return .{};
    }

    // TODO in the jps algorithm because we only put entries into the grid once we've visited them
    // and we never use this grid to avoid iterations like in a*, the visited bool can just be removed
    SparseVisited :: struct {
        visited_heuristic: float;
        parent_cell: SparseIndex;
    }

    visited: SparseGrid(SparseVisited) = create_sparse_grid(SparseVisited, sparse.grid_size_x, sparse.grid_size_y, sparse.grid_size_z, sparse.cell_size);
    visited.allocator = temp;

    open_queue: LinkedArray(ForcedEntry);
    open_queue.allocator = temp;
    linked_reserve(*open_queue, 64);

    sorted_insert(*open_queue, ForcedEntry.{
        start_cell,
        0,
        grid_heuristic(start_cell, end_cell),
        .PX | .NX | .PY | .NY | .PZ | .NZ | .PXPZ | .PXNZ | .NXPZ | .NXNZ | .PXPY | .PXNY | .NXPY | .NXNY | .PYPZ | .PYNZ | .NYPZ | .NYNZ | .PXPYPZ | .PXPYNZ | .PXNYPZ | .PXNYNZ | .NXPYPZ | .NXPYNZ | .NXNYPZ | .NXNYNZ,
    });

    // set_cell(*visited, start_cell, SparseVisited.{
    //     grid_heuristic(start_cell, end_cell),
    //     start_cell,
    // });

    cached_sparse_grid: CachedGrid(SparseEntry(T));
    cached_visited_grid: CachedGrid(SparseVisited);

    StepInstruction :: struct {
        // TODO technically step_type isn't needed since that can be derived from offset_index
        step_type: StepType;
        cell: SparseIndex;
        offset_index: u8;
    }

    step_instructions: [..] StepInstruction;
    step_instructions.allocator = temp;

    get_path :: () -> NavPath #expand {
        path: NavPath;
        array_add(*path.path, end_cell);

        parent_cell := get_cell(*visited, end_cell, *cached_visited_grid);
        has_parent_cell := parent_cell.visited_heuristic != 0;

        while has_parent_cell {
            array_add(*path.path, parent_cell.parent_cell);

            if parent_cell.parent_cell == `start_cell {
                break;
            }

            next_parent_cell := get_cell(*visited, parent_cell.parent_cell, *cached_visited_grid);
            has_parent_cell = next_parent_cell.visited_heuristic != 0;
            parent_cell = next_parent_cell;
        }

        reverse(path.path);

        return path;
    }

    push_forced_cell :: () #expand {
        // this is always a line
        delta_travelled := `step_cell - `forced_cell;
        // TODO theres probably a faster way to do this math than abs abs abs
        new_travelled_distance := `travelled_distance + abs(delta_travelled.x) + abs(delta_travelled.y) + abs(delta_travelled.z);
        new_heuristic_distance := new_travelled_distance + grid_heuristic(`step_cell, `end_cell);

        visited_cell := get_cell(*`visited, `step_cell, *`cached_visited_grid);
        if visited_cell.visited_heuristic == 0 || new_heuristic_distance < visited_cell.visited_heuristic - 0.01 {
            set_cell(*`visited, `step_cell, .{new_heuristic_distance, `forced_cell}, *cached_visited_grid);

            print("Travelled from % to %.\n", `forced_cell, `step_cell);

            sorted_insert(*`open_queue, ForcedEntry.{
                `step_cell,
                new_travelled_distance,
                new_heuristic_distance,
                cast(ForcedOffsets) `forced_offsets,
            });
        }
    }

    if start_cell == end_cell {
        return NavPath.{};
    }

    check_count := 0;

    while open_queue.count > 0 {
        defer check_count += 1;

        forced_entry := linked_pop(*open_queue);
        forced_cell := forced_entry.cell;
        travelled_distance := forced_entry.travelled_distance;
        forced_offsets := forced_entry.forced_offsets;
        heuristic_distance := forced_entry.heuristic_distance;

        visited_cell := get_cell(*visited, forced_cell, *cached_visited_grid);

        if visited_cell.visited_heuristic != 0 && heuristic_distance >= visited_cell.visited_heuristic - 0.01 {
            continue;
        }

        step_instructions.count = 0;

        for < offset: offsets {
            offset_flag := 1 << it_index;

            if forced_offsets & cast(ForcedOffsets) offset_flag {
                array_add(*step_instructions, .{
                    step_types[it_index],
                    forced_cell + offset,
                    cast(u8) it_index,
                });
            }
        }

        while step_instructions.count > 0 {
            step_instruction := step_instructions[step_instructions.count - 1];
            step_instructions.count -= 1;

            step_cell := step_instruction.cell;
            offset_index := step_instruction.offset_index;

            if #complete step_instruction.step_type == {
                case .Line;
                    step_direction := offsets[offset_index];

                    while true {
                        if step_cell == end_cell {
                            print("Finished after checking % entries.\n", check_count);
                            // we've found the end, get the path back to here
                            return get_path();
                        }

                        ground_cell := get_cell(sparse, step_cell, *cached_sparse_grid) == .Ground;
                        if !ground_cell {
                            // can't continue on this line, so we're done jumping
                            break;
                        }

                        // put the cell that found the forced neighbor into the open set
                        additional_offsets, forced_neighbor := is_forced_neighbor(sparse, step_cell, offset_index, .Ground, *cached_sparse_grid);
                        if forced_neighbor {
                            forced_offsets := cast(u32) 1 << offset_index;
                            forced_offsets |= cast(u32) additional_offsets;

                            push_forced_cell();

                            break;
                        }

                        step_cell += step_direction;
                    }

                case .Diagonal;
                    step_direction := offsets[offset_index];

                    // TODO I'm doing these checks as part of the instruction stack
                    // but I think it would be more efficient to put the line jump code
                    // in a separate function and call it in a while true loop from here

                    if step_cell == end_cell {
                        print("Finished after checking % entries.\n", check_count);
                        return get_path();
                    }

                    ground_cell := get_cell(sparse, step_cell, *cached_sparse_grid) == .Ground;
                    if !ground_cell {
                        break;
                    }

                    additional_offsets, forced_neighbor := is_forced_neighbor(sparse, step_cell, offset_index, .Ground, *cached_sparse_grid);
                    if forced_neighbor {
                        forced_offsets := cast(u32) 1 << offset_index;
                        for line_offset_index: diagonal_line_offset_indices[offset_index] {
                            forced_offsets |= cast(u32) 1 << line_offset_index;
                        }
                        forced_offsets |= cast(u32) additional_offsets;

                        push_forced_cell();

                        break;
                    }

                    array_add(*step_instructions, .{
                        .Diagonal,
                        step_cell + step_direction,
                        cast(u8) offset_index,
                    });

                    for line_offset_index: diagonal_line_offset_indices[offset_index] {
                        array_add(*step_instructions, .{
                            .Line,
                            step_cell + offsets[line_offset_index],
                            cast(u8) line_offset_index,
                        });
                    }

                case .Corner;
                    step_direction := offsets[offset_index];

                    if step_cell == end_cell {
                        print("Finished after checking % entries.\n", check_count);
                        return get_path();
                    }

                    ground_cell := get_cell(sparse, step_cell, *cached_sparse_grid) == .Ground;
                    if !ground_cell {
                        break;
                    }

                    additional_offsets, forced_neighbor := is_forced_neighbor(sparse, step_cell, offset_index, .Ground, *cached_sparse_grid);
                    if forced_neighbor {
                        forced_offsets := cast(u32) 1 << offset_index;
                        for diagonal_offset_index: corner_diagonal_offset_indices[offset_index] {
                            forced_offsets |= cast(u32) 1 << diagonal_offset_index;
                        }
                        for line_offset_index: corner_line_offset_indices[offset_index] {
                            forced_offsets |= cast(u32) 1 << line_offset_index;
                        }
                        forced_offsets |= cast(u32) additional_offsets;

                        push_forced_cell();

                        break;
                    }

                    array_add(*step_instructions, .{
                        .Corner,
                        step_cell + step_direction,
                        cast(u8) offset_index,
                    });

                    for diagonal_offset_index: corner_diagonal_offset_indices[offset_index] {
                        array_add(*step_instructions, .{
                            .Diagonal,
                            step_cell + offsets[diagonal_offset_index],
                            cast(u8) diagonal_offset_index,
                        });
                    }

                    for line_offset_index: corner_line_offset_indices[offset_index] {
                        array_add(*step_instructions, .{
                            .Line,
                            step_cell + offsets[line_offset_index],
                            cast(u8) line_offset_index,
                        });
                    }
            }
        }
    }

    return .{};
}

find_path_a_star :: (sparse: *GroupedSparseGrid($T), source_point: Vector3, target_point: Vector3) -> NavPath {
    source_cell := get_cell_index(*sparse.grid, source_point);
    target_cell := get_cell_index(*sparse.grid, target_point);

    return find_path_a_star(sparse, source_cell, target_cell);
}

find_path_a_star :: (sparse: *GroupedSparseGrid($T), source_cell: SparseIndex, target_cell: SparseIndex) -> NavPath {
    seconds_since_init :: () -> float64 {
        // this is like twice as fast as current_time_monotonic
        return to_float64_seconds(current_time_consensus());
    }

    SearchEntry :: struct {
        cell: SparseIndex;
        cell_distance: float;
        heuristic_distance: float;
    }

    sorted_insert :: (array: *LinkedArray(SearchEntry), entry: SearchEntry) {
        linked_insert_sorted(array, entry, (a: SearchEntry, b: SearchEntry) -> float {
            return b.heuristic_distance - a.heuristic_distance;
        });
    }

    #if logging then start := seconds_since_init();

    search_points.count = 0;

    ground_source_cell := source_cell;
    maximum_fall_cells :: 40;
    for 0..maximum_fall_cells - 1 {
        if get_cell(sparse, ground_source_cell) != .Empty {
            break;
        }

        ground_source_cell.y -= 1;
    }
    if get_cell(sparse, ground_source_cell) == .Empty {
        print("No source ground found.\n");
        return .{};
    }

    ground_target_cell := target_cell;
    for 0..maximum_fall_cells - 1 {
        if get_cell(sparse, ground_target_cell) != .Empty {
            break;
        }

        ground_target_cell.y -= 1;
    }
    if get_cell(sparse, ground_target_cell) == .Empty {
        print("No target ground found.\n");
        return .{};
    }

    SparseVisited :: struct {
        visited_heuristic: float;
        parent_index: SparseIndex;
    }

    visited: SparseGrid(SparseVisited) = create_sparse_grid(SparseVisited, sparse.grid_size_x, sparse.grid_size_y, sparse.grid_size_z, sparse.cell_size);
    visited.allocator = temp;

    search_queue: LinkedArray(SearchEntry);
    search_queue.allocator = temp;
    linked_reserve(*search_queue, 1024);

    sorted_insert(*search_queue, SearchEntry.{
        ground_source_cell,
        0,
        grid_heuristic(ground_source_cell, ground_target_cell),
    });

    set_cell(*visited, ground_source_cell, SparseVisited.{
        grid_heuristic(ground_source_cell, ground_target_cell),
        ground_source_cell,
    });

    cached_sparse_grid: CachedGrid(SparseEntry(T));
    cached_visited_grid: CachedGrid(SparseVisited);

    check_count := 0;

    #if logging defer print("get_cell_time % ms\n", get_cell_time * 1000);
    #if logging defer print("sorted_insert_time % ms\n", sorted_insert_time * 1000);
    #if logging defer print("linked_pop_time % ms\n", linked_pop_time * 1000);
    #if logging defer print("find_or_add_time % ms\n", find_or_add_time * 1000);
    #if logging defer print("get_cell_position_time % ms\n", get_cell_position_time * 1000);
    #if logging defer print("startup_time % ms\n", startup_time * 1000);

    #if logging defer print("get_or_add_cell_pointer cache hit %\n", cache_hit);
    #if logging defer print("get_or_add_cell_pointer cache miss %\n", cache_miss);

    #if logging then startup_time += seconds_since_init() - start;
    #if logging then start = 0;

    offsets :: [3] s8.[
        .[1, 0, 0], .[-1, 0, 0],
        .[0, 1, 0], .[0, -1, 0],
        .[0, 0, 1], .[0, 0, -1],
        .[1, 0, 1], .[1, 0, -1], .[-1, 0, 1], .[-1, 0, -1],
        .[1, 1, 0], .[1, -1, 0], .[-1, 1, 0], .[-1, -1, 0],
        .[0, 1, 1], .[0, 1, -1], .[0, -1, 1], .[0, -1, -1],
        .[1, 1, 1], .[1, 1, -1], .[1, -1, 1], .[1, -1, -1],
        .[-1, 1, 1], .[-1, 1, -1], .[-1, -1, 1], .[-1, -1, -1],
    ];

    l1 :: 1.0;
    l2 :: 1.414214;
    l3 :: 1.732051;
    weights :: float.[
        l1, l1,
        l1, l1,
        l1, l1,
        l2, l2, l2, l2,
        l2, l2, l2, l2,
        l2, l2, l2, l2,
        l3, l3, l3, l3,
        l3, l3, l3, l3,
    ];

    while search_queue.count > 0 {
        defer check_count += 1;

        #if logging then start = seconds_since_init();
        search_entry := linked_pop(*search_queue);
        #if logging then linked_pop_time += seconds_since_init() - start;
        #if logging then start = 0;

        if search_entry.cell == ground_target_cell {
            #if logging then start = seconds_since_init();
            #if logging then defer print("Path lookup time % ms\n", (seconds_since_init() - start) * 1000);

            path: NavPath;
            array_add(*path.path, search_entry.cell);

            parent_cell := get_cell(*visited, search_entry.cell, *cached_visited_grid);
            has_parent_cell := parent_cell.visited_heuristic > 0;
            has_parent_cell = has_parent_cell && search_entry.cell != parent_cell.parent_index;

            while has_parent_cell {
                array_add(*path.path, parent_cell.parent_index);

                next_parent_cell := get_cell(*visited, parent_cell.parent_index, *cached_visited_grid);
                has_parent_cell = next_parent_cell.visited_heuristic > 0;
                has_parent_cell = has_parent_cell && next_parent_cell.parent_index != parent_cell.parent_index;
                parent_cell = next_parent_cell;
            }

            print("Finished with % left in queue after checking % entries.\n", search_queue.count, check_count);

            reverse(path.path);
            return path;
        }

        for offset: offsets {
            offset_x := offset[0];
            offset_y := offset[1];
            offset_z := offset[2];

            cell_index := SparseIndex.{search_entry.cell.x + offset_x, search_entry.cell.y + offset_y, search_entry.cell.z + offset_z};
            if get_cell(sparse, cell_index, *cached_sparse_grid) != .Ground {
                continue;
            }

            parent_cell_distance := weights[it_index];
            distance_heuristic := search_entry.cell_distance + parent_cell_distance + grid_heuristic(cell_index, ground_target_cell);

            #if logging then start = seconds_since_init();
            parent_cell := get_or_add_cell_pointer(*visited, cell_index, *cached_visited_grid);
            #if logging then find_or_add_time += seconds_since_init() - start;
            #if logging then start = 0;
            if parent_cell.visited_heuristic == 0 || distance_heuristic < parent_cell.visited_heuristic - 0.01 {
                #if !performance_test then array_add(*search_points, SearchPoint.{
                    cell_index,
                    distance_heuristic,
                });

                // set the parent lookup if you are adding a new entry
                parent_cell.* = SparseVisited.{
                    distance_heuristic,
                    search_entry.cell,
                };

                #if logging then start = seconds_since_init();
                // if get_cell(sparse, cell_index, *cached_sparse_grid) == .Ground {
                    #if logging then if start {
                        get_cell_time += seconds_since_init() - start;
                        start = 0;
                    }
                    #if logging then start = seconds_since_init();
                    #if logging then get_cell_position_time += seconds_since_init() - start;
                    #if logging then start = 0;

                    #if logging then start = seconds_since_init();
                    sorted_insert(*search_queue, SearchEntry.{
                        cell_index,
                        search_entry.cell_distance + parent_cell_distance,
                        distance_heuristic,
                    });
                    #if logging then sorted_insert_time += seconds_since_init() - start;
                    #if logging then start = 0;
                // }
                #if logging then if start {
                    get_cell_time += seconds_since_init() - start;
                    start = 0;
                }
            }
        }
    }

    return .{};
}

grid_heuristic :: (a: SparseIndex, b: SparseIndex) -> float {
    // move in the largest direction straight 1d line
    // move in the second largest direction 2d diagonal line
    // move in the third largest direction 3d diagonal line

    delta_x := abs(b.x - a.x);
    delta_y := abs(b.y - a.y);
    delta_z := abs(b.z - a.z);

    delta_max: s32 = ---;
    delta_mid: s32 = ---;
    delta_min: s32 = ---;

    if delta_x >= delta_y && delta_x >= delta_z {
        delta_max = delta_x;
        if delta_y >= delta_z {
            delta_mid = delta_y;
            delta_min = delta_z;
        } else {
            delta_mid = delta_z;
            delta_min = delta_y;
        }
    } else if delta_y >= delta_x && delta_y >= delta_z {
        delta_max = delta_y;
        if delta_x >= delta_z {
            delta_mid = delta_x;
            delta_min = delta_z;
        } else {
            delta_mid = delta_z;
            delta_min = delta_x;
        }
    } else if delta_z >= delta_x && delta_z >= delta_y {
        delta_max = delta_z;
        if delta_x >= delta_y {
            delta_mid = delta_x;
            delta_min = delta_y;
        } else {
            delta_mid = delta_y;
            delta_min = delta_x;
        }
    } else {
        assert(false, "Uh oh huh?\n");
    }

    one_dim_count := delta_max - delta_mid;
    two_dim_count := delta_mid - delta_min;
    three_dim_count := delta_min;

    one_dim_length :: 1.0;
    two_dim_length :: 1.414214;
    three_dim_length :: 1.732051;

    return one_dim_count * one_dim_length + two_dim_count * two_dim_length + three_dim_count * three_dim_length;
}

logging :: false;
get_cell_time: float64;
sorted_insert_time: float64;
linked_pop_time: float64;
find_or_add_time: float64;
get_cell_position_time: float64;
startup_time: float64;

// #run {
//     sorted_thing: [..] float;
//     sorted_insert(*sorted_thing, 0.5, (a: float, b: float, data: int) -> float {
//         return b - a;
//     }, 0);
//     sorted_insert(*sorted_thing, 1, (a: float, b: float, data: int) -> float {
//         return b - a;
//     }, 0);
//     sorted_insert(*sorted_thing, 2, (a: float, b: float, data: int) -> float {
//         return b - a;
//     }, 0);
//     sorted_insert(*sorted_thing, 1.5, (a: float, b: float, data: int) -> float {
//         return b - a;
//     }, 0);

//     print("Sorted smallest to largest? %\n", sorted_thing);
// };

#scope_file

sorted_insert :: (array: *[..] $T, entry: T, $compare: (T, T, data: $D) -> $R, data: D) -> *T {
    // TODO I bet this is wrong somehow
    lower_bound := 0;
    upper_bound := array.count - 1;
    while lower_bound <= upper_bound - 1 {
        check_index := (lower_bound + upper_bound) / 2;

        diff := compare(array.*[check_index], entry, data);
        if diff < 0 {
            lower_bound = check_index + 1;
        } else if diff > 0 {
            upper_bound = check_index;
        } else {
            lower_bound = check_index + 1;
            upper_bound = check_index;
        }
    }

    insert := max(lower_bound, upper_bound);
    if array.count > 0 {
        if compare(array.*[insert], entry, data) < 0 {
            insert += 1;
        }
    }

    array_insert_at(array, entry, insert);

    #if true {
        for i: 0..array.count - 2 {
            if compare(array.*[i], array.*[i + 1], data) > 0 {
                print("Sorted insert didn't work. %\n", entry);
                for entry: array.* {
                    print("\t%\n", entry);
                }
                assert(false);
            }
        }
    }

    return *array.*[insert];
}
