
GridGenerator :: struct (T: Type, D: Type) {
    character_step_size: float;
    get_character_position: #type (data: D) -> Vector3;
    get_character_velocity: #type (data: D) -> Vector3;
    set_character_position: #type (position: Vector3, data: D);
    set_character_velocity: #type (velocity: Vector3, data: D);
    update_physics: #type (delta_time: float, data: D);
    is_character_on_ground: #type (data: D) -> bool;
    grid_cell_added_callback: #type (grid_index: SparseIndex, cell_index: SparseIndex);
    gravity_direction: Vector3;
    // delta time of an update tick in seconds
    delta_time: float;
    empty_type: T;
    ground_type: T;
    aabb: AABB3;
    data: D;
    internals: struct {
        setting_position: bool;
        moving: bool;
        current_iteration_count: int;
        body_last_position: Vector3;
        move_queue: [..] GridMove;
        visited_ground: Table(SparseIndex, bool, given_hash_function = sparse_hash, given_compare_function = sparse_equals);
    };
}

GridMove :: struct {
    from: SparseIndex;
    to: SparseIndex;
}

// plans
// if you moved to a vertically adjacent cell in either direction, we will connect those cells
// if you fall down when going to the next cell we will connect the calls as fall cells
// if you managed to fall from one location to another there will be a second pass that will try to jump up all your falls
// if that succeeds then those nodes will be marked as jumpable
// all jumpable really means is that the jump distances pointing in those specific directions are positive
// and then possible I need to do a pass where I trim or at least mark cells as near edge cells
// so that if you're trying to walk to the end without falling you won't go through these cells
// so I think the implication is... maybe... these cells can't traverse into each other but only
// diagonally towards the edge or away from the edge
// I think this would inherently prevent the path finding from letting you even enter the edge
// cells unless you actually want to fall down the wall or jump up the wall

destroy :: (grid_generator: *GridGenerator) {
    array_free(grid_generator.internals.move_queue);
    deinit(*grid_generator.internals.visited_ground);
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

begin_nav_grid :: (using generator: *GridGenerator($T, $D), sparse: *JumpSparseGrid(T), start: Vector3) {
    using internals;

    internals = .{};

    cell_index := get_cell_index(sparse.grid, start);

    array_add(*move_queue, GridMove.{
        cell_index,
        cell_index,
    });
}

// TODO this is gonna have to change later
// I need to make connections between the cells that actually let you walk between them
// because the algorithm can only move in cardinal directions against walls
// so the connection corners that need to be added are dependent on a real path existing.....

iterate_nav_grid :: (using generator: *GridGenerator($T, $D), sparse: *JumpSparseGrid(T)) -> done: bool {
    using internals;

    if move_queue.count == 0 {
        return true;
    }
    grid_move := move_queue[move_queue.count - 1];

    if table_contains(*visited_ground, grid_move.to) {
        setting_position = false;
        moving = false;
        current_iteration_count = 0;

        move_queue.count -= 1;

        return false;
    }

    defer current_iteration_count += 1;

    if current_iteration_count > 40 {
        setting_position = false;
        moving = false;
        current_iteration_count = 0;

        move_queue.count -= 1;

        return false;
    }

    if !setting_position && !moving {
        setting_position = true;
        moving = false;
        current_iteration_count = 0;

        set_character_position(get_cell_position(sparse.grid, grid_move.from), data);
        update_physics(delta_time, data);

        body_last_position = get_character_position(data);

        return false;
    } else if setting_position {
        velocity := get_character_velocity(data);
        velocity.x = 0;
        velocity.z = 0;

        set_character_velocity(velocity, data);
        update_physics(delta_time, data);
        position := get_character_position(data);

        delta_position := position - body_last_position;
        body_last_position = position;

        stopped_moving := length(delta_position) < 0.0001;

        if stopped_moving {
            setting_position = false;
            moving = true;
            current_iteration_count = 0;
        }

        return false;
    } else if moving {
        velocity := get_character_velocity(data);

        position := get_character_position(data);
        remaining_delta := get_cell_position(sparse.grid, grid_move.to) - position;
        remaining_delta_xz := Vector3.{remaining_delta.x, 0, remaining_delta.z};
        if length(remaining_delta_xz) < character_step_size + 0.00001 {
            velocity.x = remaining_delta.x;
            velocity.z = remaining_delta.z;
        } else {
            remaining_delta_xz = normalize(remaining_delta_xz) * character_step_size;
            velocity.x = remaining_delta_xz.x;
            velocity.z = remaining_delta_xz.z;
        }

        velocity.x /= delta_time;
        velocity.z /= delta_time;

        set_character_velocity(velocity, data);
        update_physics(delta_time, data);
        position = get_character_position(data);

        delta_position := position - body_last_position;
        body_last_position = position;

        on_ground := is_character_on_ground(data);
        stopped_moving := length(delta_position) < 0.0001;

        if stopped_moving {
            setting_position = false;
            moving = false;
            current_iteration_count = 0;

            move_queue.count -= 1;

            position_cell := get_cell_index(sparse.grid, position);
            delta_first_cell := position_cell - grid_move.to;
            if on_ground && delta_first_cell.x == 0 && delta_first_cell.z == 0 /*&& abs(delta_first_cell.y) <= 1*/ {
                if table_contains(*visited_ground, position_cell) {
                    return false;
                }

                mark_ground(generator, sparse, position_cell);

                table_set(*visited_ground, position_cell, true);

                next_offsets :: SparseIndex.[.{1, 0, 0}, .{-1, 0, 0}, .{0, 0, 1}, .{0, 0, -1}];
                for next_offset: next_offsets {
                    next_cell := position_cell + next_offset;
                    if !table_contains(*visited_ground, next_cell) {
                        array_add(*move_queue, GridMove.{
                            position_cell,
                            next_cell,
                        });
                    }
                }
            }
        }

        return false;
    }

    assert(false, "Huh?\n");
    return true;
}

// TODO this method actually needs to look up its neighbors positions and try pretty hard to align
// itself, vertically with gravity, with its neighbors so that the actual jps algorithm can be fast
mark_ground :: (using generator: *GridGenerator($T, $D), sparse: *JumpSparseGrid(T), cell_index: SparseIndex) {
    set_cell(sparse, cell_index, ground_type);

    if grid_cell_added_callback then grid_cell_added_callback(get_grid_index(sparse.grid, cell_index), cell_index);
}

/*
construct_grouped_sparse_grid :: (generator: GridGenerator($T, $D), sparse: *JumpSparseGrid(T), aabb: AABB3) {
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
*/

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

