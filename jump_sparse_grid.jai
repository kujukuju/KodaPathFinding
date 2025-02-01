
JumpSparseGrid :: struct (T: Type) {
    using grid: SparseGrid(JumpEntry(T));
    // groups: Table(u16, [..] SparseIndex);
}

JumpEntry :: struct (T: Type) {
    data: T;
    distances: [offsets.count] s8;
    forced_neighbors: [3] [3] ForcedOffsets;
}

SparseConstructionInfo :: struct (T: Type, D: Type) {
    gravity_direction: Vector3;
    get_cell_empty: (cell: AABB3, data: D) -> bool;
    get_grid_empty: (grid: AABB3, data: D) -> bool;
    empty_type: T;
    ground_type: T;
    data: D;
}

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

offset_index_lookup :: [3] [3] u8.[
    .[
        .[25, 13, 24],
        .[9, 1, 8],
        .[23, 12, 22],
    ], .[
        .[17, 3, 16],
        .[5, 255, 4],
        .[15, 2, 14],
    ], .[
        .[21, 11, 20],
        .[7, 0, 6],
        .[19, 10, 18],
    ],
];

travel_offsets :: [3] [3] ForcedOffsets.[
    .[
        .[.NX | .NY | .NZ | .NXNY | .NXNZ | .NYNZ | .NXNYNZ, .NX | .NY | .NXNY, .NX | .NY | .PZ | .NXNY | .NXPZ | .NYPZ | .NXNYPZ],
        .[.NX | .NZ | .NXNZ, .NX, .NX | .PZ | .NXPZ],
        .[.NX | .PY | .NZ | .NXPY | .NXNZ | .PYNZ | .NXPYNZ, .NX | .PY | .NXPY, .NX | .PY | .PZ | .NXPY | .NXPZ | .PYPZ | .NXPYPZ],
    ], .[
        .[.NY | .NZ | .NYNZ, .NY, .NY | .PZ | .NYPZ],
        .[.NZ, .None, .PZ],
        .[.PY | .NZ | .PYNZ, .PY, .PY | .PZ | .PYPZ],
    ], .[
        .[.PX | .NY | .NZ | .PXNY | .PXNZ | .NYNZ | .PXNYNZ, .PX | .NY | .PXNY, .PX | .NY | .PZ | .PXNY | .PXPZ | .NYPZ | .PXNYPZ],
        .[.PX | .NZ | .PXNZ, .PX, .PX | .PZ | .PXPZ],
        .[.PX | .PY | .NZ | .PXPY | .PXNZ | .PYNZ | .PXPYNZ, .PX | .PY | .PXPY, .PX | .PY | .PZ | .PXPY | .PXPZ | .PYPZ | .PXPYPZ],
    ],
];

// a jump sparse grid is a sparse grid that contains connection information for visually continuous sections of mesh
// this, I think, will heavily optimize the path finding algorithm

// TODO
// write the actual path finding algorithm and make it visual in the game
// then go through and generate the sparse groups every time you add a new cell

create_grouped_sparse_grid :: ($T: Type, grid_size_x: int, grid_size_y: int, grid_size_z: int, cell_size: float) -> JumpSparseGrid(T) {
    jump: JumpSparseGrid(T);
    jump.grid = create_sparse_grid(JumpEntry(T), grid_size_x, grid_size_y, grid_size_z, cell_size);

    return jump;
}

for_expansion :: (sparse: JumpSparseGrid($T), body: Code, flags: For_Flags) #expand {
    #assert(!(flags & .REVERSE));

    #if flags & .POINTER {
        for *cell, cell_index: sparse.grid {
            `it := cell.data;
            `it_index := cell_index;
            #insert body;
        }
    } else {
        for cell, cell_index: sparse.grid {
            `it := cell.data;
            `it_index := cell_index;
            #insert body;
        }
    }
}

bake_distances :: (sparse: *JumpSparseGrid($T), wall_type: T) {
    bake_distances :: (sparse: *JumpSparseGrid(T), indices: [] u8, wall_type: T) {
        for grid, grid_index: sparse.grids {
            for *cell, cell_index: grid {
                if cell.data == wall_type {
                    continue;
                }

                cell_sparse_index := get_cell_index(sparse.grid, grid_index, cell_index);
                for index: indices {
                    direction := offsets[index];
                    cell.distances[index] = calculate_step_distance(sparse, cell_sparse_index, direction, wall_type);
                }
            }
        }
    }

    line_indices :: u8.[0, 1, 2, 3, 4, 5];
    diagonal_indices :: u8.[6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17];
    corner_indices :: u8.[18, 19, 20, 21, 22, 23, 24, 25];

    bake_distances(sparse, line_indices, wall_type);
    bake_distances(sparse, diagonal_indices, wall_type);
    bake_distances(sparse, corner_indices, wall_type);
}

bake_forced_neighbors :: (sparse: *JumpSparseGrid($T), wall_type: T) {
    for grid, grid_index: sparse.grids {
        for *cell, cell_index: grid {
            if cell.data == wall_type {
                continue;
            }

            cell_sparse_index := get_cell_index(sparse.grid, grid_index, cell_index);
            cell.forced_neighbors = calculate_forced_neighbors(sparse, cell_sparse_index, wall_type);
        }
    }
}

construct_grouped_sparse_grid :: (sparse: *JumpSparseGrid($T), aabb: AABB3, info: SparseConstructionInfo(T, $D)) {
    start_index := get_cell_index(*sparse.grid, aabb.lower);
    end_index := get_cell_index(*sparse.grid, aabb.upper);

    start_grid_index := get_grid_index(*sparse.grid, start_index);
    end_grid_index := get_grid_index(*sparse.grid, end_index);

    ground_add_x := cast(s32) sign(info.gravity_direction.x);
    ground_add_y := cast(s32) sign(info.gravity_direction.y);
    ground_add_z := cast(s32) sign(info.gravity_direction.z);

    // grid_count_x := end_grid_index.x - start_grid_index.x + 1;
    // grid_count_y := end_grid_index.y - start_grid_index.y + 1;
    // grid_count_z := end_grid_index.z - start_grid_index.z + 1;

    count_x := end_index.x - start_index.x + 1;
    count_y := end_index.y - start_index.y + 1;
    count_z := end_index.z - start_index.z + 1;

    cells_empty := NewArray(count_x * count_y * count_z, bool);
    defer array_free(cells_empty);

    ATTEMPT_OPTIMIZATION :: true;
    #if ATTEMPT_OPTIMIZATION {
        for grid_z: start_grid_index.z..end_grid_index.z {
            cell_start_z := max(grid_z * sparse.grid_size_z, start_index.z);
            cell_end_z := min(cell_start_z + sparse.grid_size_z - 1, end_index.z);

            for grid_y: start_grid_index.y..end_grid_index.y {
                cell_start_y := max(grid_y * sparse.grid_size_y, start_index.y);
                cell_end_y := min(cell_start_y + sparse.grid_size_y - 1, end_index.y);

                for grid_x: start_grid_index.x..end_grid_index.x {
                    cell_start_x := max(grid_x * sparse.grid_size_x, start_index.x);
                    cell_end_x := min(cell_start_x + sparse.grid_size_x - 1, end_index.x);

                    start_aabb := get_cell_aabb(*sparse.grid, cell_start_x, cell_start_y, cell_start_z);
                    end_aabb := get_cell_aabb(*sparse.grid, cell_end_x, cell_end_y, cell_end_z);
                    grid_aabb := merge(start_aabb, end_aabb);

                    // try to early out the entire call if possible
                    if info.get_grid_empty && info.get_grid_empty(grid_aabb, info.data) {
                        for cell_z: cell_start_z..cell_end_z {
                            offset_z := cell_z - start_index.z;

                            for cell_y: cell_start_y..cell_end_y {
                                offset_y := cell_y - start_index.y;

                                for cell_x: cell_start_x..cell_end_x {
                                    offset_x := cell_x - start_index.x;

                                    cell_index := offset_z * count_x * count_y + offset_y * count_x + offset_x;
                                    cells_empty[cell_index] = true;
                                }
                            }
                        }

                        continue;
                    }

                    for cell_z: cell_start_z..cell_end_z {
                        offset_z := cell_z - start_index.z;

                        for cell_y: cell_start_y..cell_end_y {
                            offset_y := cell_y - start_index.y;

                            for cell_x: cell_start_x..cell_end_x {
                                offset_x := cell_x - start_index.x;

                                cell_aabb := get_cell_aabb(*sparse.grid, cell_x, cell_y, cell_z);
                                cell_empty := info.get_cell_empty(cell_aabb, info.data);

                                cell_index := offset_z * count_x * count_y + offset_y * count_x + offset_x;
                                cells_empty[cell_index] = cell_empty;
                            }
                        }
                    }
                }
            }
        }
    } else {
        for cell_z: start_index.z..end_index.z {
            offset_z := cell_z - start_index.z;

            for cell_y: start_index.y..end_index.y {
                offset_y := cell_y - start_index.y;

                for cell_x: start_index.x..end_index.x {
                    offset_x := cell_x - start_index.x;

                    cell_aabb := get_cell_aabb(*sparse.grid, cell_x, cell_y, cell_z);
                    cell_empty := info.get_cell_empty(cell_aabb, info.data);

                    cell_index := offset_z * count_x * count_y + offset_y * count_x + offset_x;
                    cells_empty[cell_index] = cell_empty;
                }
            }
        }
    }

    for offset_z: cast(s32) 0..count_z - 1 {
        for offset_y: cast(s32) 0..count_y - 1 {
            for offset_x: cast(s32) 0..count_x - 1 {
                ground_offset_x := offset_x + ground_add_x;
                ground_offset_y := offset_y + ground_add_y;
                ground_offset_z := offset_z + ground_add_z;

                inside_range := ground_offset_x >= 0 && ground_offset_x < count_x;
                inside_range = inside_range && ground_offset_y >= 0 && ground_offset_y < count_y;
                inside_range = inside_range && ground_offset_z >= 0 && ground_offset_z < count_z;
                if !inside_range {
                    continue;
                }

                cell_index := offset_z * count_x * count_y + offset_y * count_x + offset_x;
                cell_empty := cells_empty[cell_index];

                if !cell_empty {
                    continue;
                }

                ground_index := ground_offset_z * count_x * count_y + ground_offset_y * count_x + ground_offset_x;
                ground_empty := cells_empty[ground_index];

                if ground_empty {
                    continue;
                }

                set_cell(sparse, SparseIndex.{start_index.x + offset_x, start_index.y + offset_y, start_index.z + offset_z}, info.empty_type);
                set_cell(sparse, SparseIndex.{start_index.x + ground_offset_x, start_index.y + ground_offset_y, start_index.z + ground_offset_z}, info.ground_type);
            }
        }
    }

    edges :: [3] [3] s8.[
        .[
            .[-1, -1, 0],
            .[-1, 0, 0],
            .[-1, 1, 0],
        ], .[
            .[1, -1, 0],
            .[1, 0, 0],
            .[1, 1, 0],
        ], .[
            .[0, -1, -1],
            .[0, 0, -1],
            .[0, 1, -1],
        ], .[
            .[0, -1, 1],
            .[0, 0, 1],
            .[0, 1, 1],
        ],
    ];

    // don't do this for performance test
    // for offset_z: cast(s32) 0..count_z - 1 {
    //     for offset_y: cast(s32) 0..count_y - 1 {
    //         for offset_x: cast(s32) 0..count_x - 1 {
    //             cell_index := offset_z * count_x * count_y + offset_y * count_x + offset_x;
    //             if cells_empty[cell_index] {
    //                 continue;
    //             }

    //             for edge_group: edges {
    //                 for edge: edge_group {
    //                     edge_offset_x := offset_x + edge[0];
    //                     edge_offset_y := offset_y + edge[1];
    //                     edge_offset_z := offset_z + edge[2];

    //                     is_empty_edge := edge_offset_x < 0 || edge_offset_x >= count_x;
    //                     is_empty_edge = is_empty_edge || edge_offset_y < 0 || edge_offset_y >= count_y;
    //                     is_empty_edge = is_empty_edge || edge_offset_z < 0 || edge_offset_z >= count_z;

    //                     edge_index := edge_offset_z * count_x * count_y + edge_offset_y * count_x + edge_offset_x;
    //                     is_empty_edge = is_empty_edge || cells_empty[edge_index];

    //                     if !is_empty_edge {
    //                         continue edge_group;
    //                     }
    //                 }

    //                 // if we've finished looking at all the edges in a given edge group
    //                 // and we haven't found a non-empty cell, then we remove this cell

    //                 set_cell(sparse, SparseIndex.{start_index.x + offset_x, start_index.y + offset_y, start_index.z + offset_z}, info.empty_type);
    //             }
    //         }
    //     }
    // }



    for #v2 < offset_y: cast(s32) 0..count_y - 1 {
        for offset_z: cast(s32) 0..count_z - 1 {
            for offset_x: cast(s32) 0..count_x - 1 {
                cell_index := SparseIndex.{start_index.x + offset_x, start_index.y + offset_y, start_index.z + offset_z};

                if get_cell(sparse, cell_index) == info.empty_type {
                    continue;
                }

                set_cell(sparse, cell_index + SparseDir.{0, 1, 0}, info.ground_type);

                // cell_index := offset_z * count_x * count_y + offset_y * count_x + offset_x;
                // if cells_empty[cell_index] {
                //     continue;
                // }

                // set_cell(sparse, SparseIndex.{start_index.x + offset_x, start_index.y + offset_y + 1, start_index.z + offset_z}, info.ground_type);
            }
        }
    }
}

get_cell :: (sparse: *JumpSparseGrid($T), point: Vector3) -> T, bool {
    entry, found := get_cell(*sparse.grid, point);
    return entry.data, found;
}

// *CachedGrid(JumpEntry(T)) doesn't work... try again in a future version
// get_cell :: (sparse: *JumpSparseGrid($T), cell_index: SparseIndex, cached_grid: *CachedGrid(JumpEntry(T))) -> T, bool {
// get_cell :: (sparse: *JumpSparseGrid($T), cell_index: SparseIndex, cached_grid: *$D) -> T, bool {
//     entry, found := get_cell(*sparse.grid, cell_index, cached_grid);
//     return entry.data, found;
// }

get_cell :: (sparse: *JumpSparseGrid($T), cell_index: SparseIndex) -> T, bool {
    entry, found := get_cell(*sparse.grid, cell_index);
    return entry.data, found;
}

set_cell :: (sparse: *JumpSparseGrid($T), point: Vector3, data: T) {
    entry: JumpEntry(T);
    entry.data = data;
    set_cell(*sparse.grid, point, entry);
}

set_cell :: (sparse: *JumpSparseGrid($T), cell_index: SparseIndex, data: T) {
    entry: JumpEntry(T);
    entry.data = data;
    set_cell(*sparse.grid, cell_index, entry);
}

get_nearest_cell :: (sparse: *JumpSparseGrid($T), line: Line3, valid: T) -> SparseIndex, bool {
    nearest_cell: SparseIndex;
    found_nearest_cell: bool;

    for cell, cell_index: get_cells_from_line(*sparse.grid, line) {
        if cell.data == valid || cell.data & valid {
            nearest_cell = cell_index;
            found_nearest_cell = true;
            break;
        }
    }

    if !found_nearest_cell {
        return .{}, false;
    }

    cell_boundaries := get_cell_boundaries(*sparse.grid);

    nearest_cell.x = clamp(nearest_cell.x, cell_boundaries[0].x, cell_boundaries[1].x);
    nearest_cell.y = clamp(nearest_cell.y, cell_boundaries[0].y, cell_boundaries[1].y);
    nearest_cell.z = clamp(nearest_cell.z, cell_boundaries[0].z, cell_boundaries[1].z);

    return nearest_cell, true;
}

get_memory_size :: (sparse: JumpSparseGrid($T)) -> grid_size: int, groups_size: int {
    grid_size := 0;
    groups_size := 0;

    for cell, cell_index: sparse.grid {
        grid_size += cell.count * size_of(type_of(cell));
        grid_size += size_of(type_of(cell_index));
    }
    for group, group_index: sparse.groups {
        groups_size += group.count * size_of(type_of(group));
        groups_size += size_of(type_of(group_index));
    }

    return grid_size, group_size;
}

#scope_file

// you have to calculate step distances line -> diagonal -> corner
// so we can assume when you're calling calculate on a diagonal every single cell has it's line value completed
calculate_step_distance :: (sparse: *JumpSparseGrid($T), cell: SparseIndex, direction: SparseDir, wall_type: T) -> s8 {
    // im going to do this such that positive numbers are forced neighbor distances
    // negative numbers are wall distances
    // and S8_MAX is a forced neighbor continuation indicator

    // TODO apparently this function doesn't need to return wall
    calculate_jump_point :: (sparse: *JumpSparseGrid(T), jump_cell: JumpEntry(T), cell: SparseIndex, direction: SparseDir, wall_type: T) -> jump: bool {
        calculate_jump_point_line :: (sparse: *JumpSparseGrid(T), cell: SparseIndex, direction: SparseDir, wall_type: T) -> jump: bool {
            assert(abs(direction.x) + abs(direction.y) + abs(direction.z) == 1, "Line jump point check must have 1 component.\n");

            // the first cell you check CAN be a forced neighbor jump point
            // the first cell you check CANNOT be a wall

            current_cell := cell;

            if calculate_forced_neighbor(sparse, current_cell, direction, wall_type) {
                return true;
            }

            while true {
                current_cell += direction;

                if is_wall(get_cell(sparse, current_cell), wall_type) {
                    return false;
                }

                if calculate_forced_neighbor(sparse, current_cell, direction, wall_type) {
                    return true;
                }
            }

            return false;
        }

        calculate_jump_point_diagonal :: (sparse: *JumpSparseGrid(T), jump_cell: JumpEntry(T), cell: SparseIndex, direction: SparseDir, wall_type: T) -> jump: bool {
            assert(abs(direction.x) + abs(direction.y) + abs(direction.z) == 2, "Diagonal jump point check must have 2 components.\n");

            line_directions: [2] SparseDir = ---;

            if direction.x == 0 {
                line_directions = .[.{0, direction.y, 0}, .{0, 0, direction.z}];
            } else if direction.y == 0 {
                line_directions = .[.{direction.x, 0, 0}, .{0, 0, direction.z}];
            } else if direction.z == 0 {
                line_directions = .[.{direction.x, 0, 0}, .{0, direction.y, 0}];
            }

            forced_neighbor := !!calculate_forced_neighbor(sparse, cell, line_directions[0], wall_type);
            forced_neighbor = forced_neighbor || calculate_forced_neighbor(sparse, cell, line_directions[1], wall_type);
            if forced_neighbor {
                return true;
            }

            line_indices := u8.[
                offset_index_lookup[line_directions[0].x + 1][line_directions[0].y + 1][line_directions[0].z + 1],
                offset_index_lookup[line_directions[1].x + 1][line_directions[1].y + 1][line_directions[1].z + 1],
            ];

            jump_point := jump_cell.distances[line_indices[0]] > 0;
            jump_point = jump_point || jump_cell.distances[line_indices[1]] > 0;
            return jump_point;
        }

        calculate_jump_point_corner :: (sparse: *JumpSparseGrid(T), jump_cell: JumpEntry(T), cell: SparseIndex, direction: SparseDir, wall_type: T) -> jump: bool {
            assert(abs(direction.x) + abs(direction.y) + abs(direction.z) == 3, "Corner jump point check must have 3 components.\n");

            diagonal_directions := SparseDir.[
                .{direction.x, direction.y, 0},
                .{direction.x, 0, direction.z},
                .{0, direction.y, direction.z},
            ];

            jump_point := calculate_jump_point_diagonal(sparse, jump_cell, cell, diagonal_directions[0], wall_type);
            jump_point = jump_point || calculate_jump_point_diagonal(sparse, jump_cell, cell, diagonal_directions[1], wall_type);
            jump_point = jump_point || calculate_jump_point_diagonal(sparse, jump_cell, cell, diagonal_directions[2], wall_type);
            return jump_point;

            // diagonal_indices := u8.[
            //     offset_index_lookup[diagonal_directions[0].x + 1][diagonal_directions[0].y + 1][diagonal_directions[0].z + 1],
            //     offset_index_lookup[diagonal_directions[1].x + 1][diagonal_directions[1].y + 1][diagonal_directions[1].z + 1],
            //     offset_index_lookup[diagonal_directions[2].x + 1][diagonal_directions[2].y + 1][diagonal_directions[2].z + 1],
            // ];

            // jump_point := jump_cell.distances[diagonal_indices[0]] > 0;
            // jump_point = jump_point || jump_cell.distances[diagonal_indices[1]] > 0;
            // jump_point = jump_point || jump_cell.distances[diagonal_indices[2]] > 0;
            // return jump_point;
        }

        component_count := abs(direction.x) + abs(direction.y) + abs(direction.z);
        if component_count == {
            case 1;
                return calculate_jump_point_line(sparse, cell, direction, wall_type);
            case 2;
                return calculate_jump_point_diagonal(sparse, jump_cell, cell, direction, wall_type);
            case 3;
                return calculate_jump_point_corner(sparse, jump_cell, cell, direction, wall_type);
        }

        return false;
    }

    direction_index := offset_index_lookup[direction.x + 1][direction.y + 1][direction.z + 1];

    for i: cast(s8) 1..S8_MAX - 1 {
        check_cell := cell + direction * i;

        jump_cell := get_cell(*sparse.grid, check_cell);
        if is_wall(jump_cell.data, wall_type) {
            return -i;
        }

        jump_point := calculate_jump_point(sparse, jump_cell, check_cell, direction, wall_type);
        if jump_point {
            return i;
        }

        if jump_cell.distances[direction_index] {
            jump_distance := cast(s32) jump_cell.distances[direction_index];
            if abs(jump_distance) + i >= S8_MAX {
                return S8_MAX;
            }

            return cast(s8) ((abs(jump_distance) + i) * sign(jump_distance));
        }
    }

    return S8_MAX;
}

calculate_forced_neighbors :: (sparse: *JumpSparseGrid($T), cell: SparseIndex, wall_type: T) -> [3] [3] ForcedOffsets {
    neighbors: [3] [3] ForcedOffsets;
    neighbors[0][0] = calculate_forced_neighbor(sparse, cell, .{-1, 0, 0}, wall_type);
    neighbors[0][1] = .None;
    neighbors[0][2] = calculate_forced_neighbor(sparse, cell, .{1, 0, 0}, wall_type);
    neighbors[1][0] = calculate_forced_neighbor(sparse, cell, .{0, -1, 0}, wall_type);
    neighbors[1][1] = .None;
    neighbors[1][2] = calculate_forced_neighbor(sparse, cell, .{0, 1, 0}, wall_type);
    neighbors[2][0] = calculate_forced_neighbor(sparse, cell, .{0, 0, -1}, wall_type);
    neighbors[2][1] = .None;
    neighbors[2][2] = calculate_forced_neighbor(sparse, cell, .{0, 0, 1}, wall_type);
    return neighbors;
}

calculate_forced_neighbor :: (sparse: *JumpSparseGrid($T), cell: SparseIndex, direction: SparseDir, wall_type: T) -> ForcedOffsets, bool {
    assert(abs(direction.x) + abs(direction.y) + abs(direction.z) == 1, "You can only check for forced neighbors of a line.\n");

    // we assume in this function the cell you're checking must be empty

    empty_cell_directions: [8] SparseDir = ---;
    forced_cell_offsets: [8] ForcedOffsets = ---;
    jump_point_empty_cells: [8] T = ---;
    jump_point_wall_cells: [8] T = ---;

    if direction.x != 0 {
        empty_cell_directions = .[
            .{0, -1, -1},
            .{0, -1, 0},
            .{0, -1, 1},
            .{0, 0, -1},
            .{0, 0, 1},
            .{0, 1, -1},
            .{0, 1, 0},
            .{0, 1, 1},
        ];
    } else if direction.y != 0 {
        empty_cell_directions = .[
            .{-1, 0, -1},
            .{-1, 0, 0},
            .{-1, 0, 1},
            .{0, 0, -1},
            .{0, 0, 1},
            .{1, 0, -1},
            .{1, 0, 0},
            .{1, 0, 1},
        ];
    } else if direction.z != 0 {
        empty_cell_directions = .[
            .{-1, -1, 0},
            .{-1, 0, 0},
            .{-1, 1, 0},
            .{0, -1, 0},
            .{0, 1, 0},
            .{1, -1, 0},
            .{1, 0, 0},
            .{1, 1, 0},
        ];
    }

    for empty_cell_direction: empty_cell_directions {
        forced_cell_offsets[it_index] = get_forced_offset(empty_cell_direction) | get_forced_offset(empty_cell_direction + direction);
        jump_point_empty_cells[it_index] = get_cell(sparse, cell + empty_cell_direction);
        jump_point_wall_cells[it_index] = get_cell(sparse, cell + empty_cell_direction - direction);
    }

    // actual corner, adjacent cell, adjacent cell
    // so we have to check that the corner empty cell is empty, the corner wall cell is wall
    // and that the 2 adjacent empty cells are empty so you're not cutting through any wall cell diagonals
    corner_indices_groups :: [3] u8.[
        .[0, 1, 3],
        .[2, 1, 3],
        .[5, 6, 3],
        .[7, 6, 4],
    ];

    diagonal_indices :: u8.[1, 3, 4, 6];

    forced_offsets: ForcedOffsets;

    for corner_indices: corner_indices_groups {
        forced_neighbor := is_empty(jump_point_empty_cells[corner_indices[0]], wall_type);
        forced_neighbor = forced_neighbor && is_empty(jump_point_empty_cells[corner_indices[1]], wall_type);
        forced_neighbor = forced_neighbor && is_empty(jump_point_empty_cells[corner_indices[2]], wall_type);
        forced_neighbor = forced_neighbor && is_wall(jump_point_wall_cells[corner_indices[0]], wall_type);

        if forced_neighbor {
            forced_offsets |= forced_cell_offsets[corner_indices[0]];
        }
    }

    for diagonal_index: diagonal_indices {
        forced_neighbor := is_empty(jump_point_empty_cells[diagonal_index], wall_type);
        forced_neighbor = forced_neighbor && is_wall(jump_point_wall_cells[diagonal_index], wall_type);

        if forced_neighbor {
            forced_offsets |= forced_cell_offsets[diagonal_index];
        }
    }

    return forced_offsets, !!forced_offsets;
}
