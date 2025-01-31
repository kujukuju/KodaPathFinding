
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
//
// you also need to consider how you actually get to the end point
// in 2d its simple because just either x or y aligns with the end point x or y
// in 3d I think you still only consider single axis directions because lines are the only things that truly have jump points associated with them
// and so for a given move direction you would check the end point planes that are aligned with all of the movement direction axis components

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
    print("first jps\n");
    source_cell := get_cell_index(*sparse.grid, source_point);
    print("got cell index1\n");
    target_cell := get_cell_index(*sparse.grid, target_point);
    print("got cell index2\n");

    return path_find_jps(sparse, source_cell, target_cell, wall_type);
}

path_find_jps :: (sparse: *GroupedSparseGrid($T), source_cell: SparseIndex, target_cell: SparseIndex, wall_type: T) -> NavPath {
    print("huh\n");
    seconds_since_init :: () -> float64 {
        return to_float64_seconds(current_time_consensus());
    }

    ForcedEntry :: struct {
        cell: SparseIndex;
        parent: SparseIndex;
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
        start_cell,
        0,
        grid_heuristic(start_cell, end_cell),
        .PX | .NX | .PY | .NY | .PZ | .NZ | .PXPZ | .PXNZ | .NXPZ | .NXNZ | .PXPY | .PXNY | .NXPY | .NXNY | .PYPZ | .PYNZ | .NYPZ | .NYNZ | .PXPYPZ | .PXPYNZ | .PXNYPZ | .PXNYNZ | .NXPYPZ | .NXPYNZ | .NXNYPZ | .NXNYNZ,
    });

    cached_grid: CachedGrid(SparseEntry(T));
    cached_visited_grid: CachedGrid(SparseVisited);

    get_path :: (before_end_cell: SparseIndex) -> NavPath #expand {
        path: NavPath;
        array_add(*path.path, `end_cell);
        array_add(*path.path, before_end_cell);

        parent_cell := get_cell(*visited, before_end_cell, *cached_visited_grid);
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

    print("before loop\n");

    check_count := 0;

    while open_queue.count > 0 {
        defer check_count += 1;

        print("loop\n");

        forced_entry := linked_pop(*open_queue);
        forced_cell := forced_entry.cell;
        parent_cell := forced_entry.parent;
        travelled_distance := forced_entry.travelled_distance;
        forced_offsets := forced_entry.forced_offsets;
        heuristic_distance := forced_entry.heuristic_distance;

        visited_cell := get_cell(*visited, forced_cell, *cached_visited_grid);

        if visited_cell.visited_heuristic != 0 && heuristic_distance >= visited_cell.visited_heuristic - 0.01 {
            continue;
        }

        set_cell(*visited, forced_cell, SparseVisited.{
            heuristic_distance,
            parent_cell,
        }, *cached_visited_grid);

        for < offset: offsets {
            offset_flag := 1 << it_index;

            if forced_offsets & cast(ForcedOffsets) offset_flag {
                print("get_jump_distance\n");
                jump_distance := get_jump_distance(sparse, forced_cell, offset);
                assert(jump_distance != 0, "Jump distance can't be 0 it's not possible.\n");

                if jump_distance > 0 {
                    jump_cell := forced_cell + offset * jump_distance;
                    if jump_cell == end_cell {
                        return get_path(forced_cell);
                    }

                    print("grid_heuristic\n");
                    new_travelled_distance := travelled_distance + grid_heuristic(forced_cell, jump_cell);

                    print("get_forced_neighbors\n");
                    new_forced_offsets := get_forced_neighbors(sparse, jump_cell, offset);

                    sorted_insert(*open_queue, ForcedEntry.{
                        jump_cell,
                        forced_cell,
                        new_travelled_distance,
                        new_travelled_distance + grid_heuristic(jump_cell, end_cell),
                        new_forced_offsets,
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

get_offset_index :: (direction: SparseDir) -> u8 {
    return offset_index_lookup[direction.x + 1][direction.y + 1][direction.z + 1];
}

forced_offset_lookup :: [3] [3] ForcedOffsets.[
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

get_forced_offset :: (offset: SparseDir) -> ForcedOffsets {
    return forced_offset_lookup[offset.x + 1][offset.y + 1][offset.z + 1];
}

is_wall :: (cell_type: $T) -> bool #expand {
    return !!(cell_type & `wall_type) || cell_type == `wall_type;
}

is_empty :: (cell_type: $T) -> bool #expand {
    return !is_wall(cell_type);
}

// TODO after getting the jump distance you need to check if this jump goes past the end point in any

// a negative value ist he distance to the next wall
// a positive value is the distance to the next jump
// a 0 value isn't possible outside of a wall
get_jump_distance :: (sparse: *GroupedSparseGrid($T), cell: SparseIndex, direction: SparseDir) -> s32 #expand {
    potential_jump_distance := get_jump_distance_without_end(sparse, cell, direction);
    end_jump_point, end_jump_point_distance := get_end_jump_point_distance(sparse, cell, direction, potential_jump_distance);

    return ifx end_jump_point then end_jump_point_distance else potential_jump_distance;
}

get_jump_distance_without_end :: (sparse: *GroupedSparseGrid($T), cell: SparseIndex, direction: SparseDir) -> s32 #expand {
    distance := get_step_distance(sparse, cell, direction);
    acc_distance := cast(s32) distance;

    while distance == S8_MAX {
        current_cell := cell + direction * acc_distance;

        distance = get_step_distance(sparse, current_cell, direction);
        acc_distance += abs(distance);

        if distance < 0 {
            acc_distance = -acc_distance;
        }
    }

    return acc_distance;
}

get_step_distance :: (sparse: *GroupedSparseGrid($T), cell: SparseIndex, direction: SparseDir) -> s8 #expand {
    step_distances := calculate_step_distances(sparse, cell, `wall_type);
    return step_distances[get_offset_index(direction)];
}

calculate_step_distances :: (sparse: *GroupedSparseGrid($T), cell: SparseIndex, wall_type: T) -> [24] s8 #expand {
    distances: [24] s8;

    for offset: offsets {
        distances[it_index] = calculate_step_distance(sparse, cell, offset, wall_type);
    }

    return distances;
}

calculate_step_distance :: (sparse: *GroupedSparseGrid($T), cell: SparseIndex, direction: SparseDir, wall_type: T) -> s8 #expand {
    // im going to do this such that positive numbers are forced neighbor distances
    // negative numbers are wall distances
    // and S8_MAX is a forced neighbor continuation indicator

    calculate_jump_point :: (sparse: *GroupedSparseGrid(T), cell: SparseIndex, direction: SparseDir, wall_type: T) -> jump: bool, wall: bool #expand {
        calculate_jump_point_line :: (sparse: *GroupedSparseGrid(T), cell: SparseIndex, direction: SparseDir, wall_type: T) -> jump: bool, wall: bool #expand {
            assert(abs(direction.x) + abs(direction.y) + abs(direction.z) == 1, "Line jump point check must have 1 component.\n");

            // the first cell you check CAN be a forced neighbor jump point
            // the first cell you check CANNOT be a wall

            current_cell := cell;

            if calculate_forced_neighbor(sparse, current_cell, direction, wall_type) {
                return true, false;
            }

            while true {
                current_cell += direction;

                if is_wall(get_cell(sparse, current_cell, *`cached_grid)) {
                    return false, true;
                }

                if calculate_forced_neighbor(sparse, current_cell, direction, wall_type) {
                    return true, false;
                }
            }

            return false, false;
        }

        calculate_jump_point_diagonal :: (sparse: *GroupedSparseGrid(T), cell: SparseIndex, direction: SparseDir, wall_type: T) -> jump: bool, wall: bool #expand {
            assert(abs(direction.x) + abs(direction.y) + abs(direction.z) == 2, "Diagonal jump point check must have 2 components.\n");

            line_directions: [2] SparseDir = ---;

            if direction.x == 0 {
                line_directions = .[.{0, direction.y, 0}, .{0, 0, direction.z}];
            } else if direction.y == 0 {
                line_directions = .[.{direction.z, 0, 0}, .{0, 0, direction.z}];
            } else if direction.z == 0 {
                line_directions = .[.{direction.z, 0, 0}, .{0, direction.y, 0}];
            }

            jump_point, wall_point := calculate_jump_point_line(sparse, cell, line_directions[0], wall_type);
            if jump_point || wall_point {
                return jump_point, wall_point;
            }
            jump_point, wall_point = calculate_jump_point_line(sparse, cell, line_directions[1], wall_type);
            return jump_point, wall_point;
        }

        calculate_jump_point_corner :: (sparse: *GroupedSparseGrid(T), cell: SparseIndex, direction: SparseDir, wall_type: T) -> jump: bool, wall: bool #expand {
            assert(abs(direction.x) + abs(direction.y) + abs(direction.z) == 3, "Corner jump point check must have 3 components.\n");

            diagonal_directions := SparseDir.[
                .{direction.x, direction.y, 0},
                .{direction.x, 0, direction.z},
                .{0, direction.y, direction.z},
            ];

            jump_point, wall_point := calculate_jump_point_diagonal(sparse, cell, diagonal_directions[0], wall_type);
            if jump_point || wall_point {
                return jump_point, wall_point;
            }
            jump_point, wall_point = calculate_jump_point_diagonal(sparse, cell, diagonal_directions[1], wall_type);
            if jump_point || wall_point {
                return jump_point, wall_point;
            }
            jump_point, wall_point = calculate_jump_point_diagonal(sparse, cell, diagonal_directions[2], wall_type);
            return jump_point, wall_point;
        }

        component_count := abs(direction.x) + abs(direction.y) + abs(direction.z);
        if component_count == {
            case 1;
                jump_point, wall_point := calculate_jump_point_line(sparse, cell, direction, wall_type);
                return jump_point, wall_point;
            case 2;
                jump_point, wall_point := calculate_jump_point_diagonal(sparse, cell, direction, wall_type);
                return jump_point, wall_point;
            case 3;
                jump_point, wall_point := calculate_jump_point_corner(sparse, cell, direction, wall_type);
                return jump_point, wall_point;
        }

        return false, false;
    }

    for i: cast(s8) 1..S8_MAX {
        check_cell := cell + direction * i;

        jump_point, wall_point := calculate_jump_point(sparse, check_cell, direction, wall_type);
        if jump_point {
            return i;
        } else if wall_point {
            return -i;
        }

        cell_type := get_cell(sparse, check_cell, *`cached_grid);
        if is_wall(cell_type) {
            return -i;
        }
    }

    return S8_MAX;
}

get_end_jump_point_distance :: (sparse: *GroupedSparseGrid($T), cell: SparseIndex, direction: SparseDir, jump_distance: s32) -> bool, s32 #expand {
    get_end_jump_point_line :: (sparse: *GroupedSparseGrid(T), cell: SparseIndex, direction: SparseDir, jump_distance: s32) -> bool, s32 #expand {
        assert(abs(direction.x) + abs(direction.y) + abs(direction.z) == 1);

        // if jump_distance is positive, a jump point, we can return false if it's equal because it's already a jump point and we will visit it
        // if jump_distance is negative, a wall, we can return false if it's equal because we can't go inside the wall
        // lastly when you multiply the single component direction by the travel delta in order to get the plane distance
        // the signs multiply into each other and so only positive numbers are considered valid

        end_point_delta := (`end_cell - cell) * direction;
        end_point_distance := end_point_delta.x + end_point_delta.y + end_point_delta.z;
        return end_point_distance >= 0 && abs(jump_distance) > end_point_distance, end_point_distance;
    }

    get_end_jump_point_diagonal :: (sparse: *GroupedSparseGrid(T), cell: SparseIndex, direction: SparseDir, jump_distance: s32) -> bool, s32 #expand {
        assert(abs(direction.x) + abs(direction.y) + abs(direction.z) == 2);

        // I could be wrong about this
        // I think if you're moving in the +x+z direction, and you hit the +x normal end point plane wall as the closest end point plane
        // then actually you're not allowed to readjust to go up because that's not one of your valid movement directions
        // so the diagonal rule is that your empty direction must be aligned with the end point

        valid_empty_axis := direction.x == 0 && cell.x == `end_cell.x;
        valid_empty_axis = valid_empty_axis || (direction.y == 0 && cell.y == `end_cell.y);
        valid_empty_axis = valid_empty_axis || (direction.z == 0 && cell.z == `end_cell.z);

        if !valid_empty_axis {
            return false, 0;
        }

        end_point_delta := (`end_cell - cell) * direction;

        // check the diagonal move distance intersection
        if direction.x != 0 {
            end_point_distance := end_point_delta.x;
            if end_point_distance >= 0 && abs(jump_distance) > end_point_distance {
                new_cell := cell + direction * end_point_distance;
                assert((`end_cell - new_cell).x == 0);

                new_direction := SparseDir.{0, direction.y, direction.z};
                new_jump_distance := get_jump_distance_without_end(sparse, new_cell, new_direction);

                jump_point := get_end_jump_point_line(sparse, new_cell, new_direction, new_jump_distance);
                if jump_point {
                    return true, end_point_distance;
                }
            }
        }

        if direction.y != 0 {
            end_point_distance := end_point_delta.y;
            if end_point_distance >= 0 && abs(jump_distance) > end_point_distance {
                new_cell := cell + direction * end_point_distance;
                assert((`end_cell - new_cell).y == 0);

                new_direction := SparseDir.{direction.x, 0, direction.z};
                new_jump_distance := get_jump_distance_without_end(sparse, new_cell, new_direction);

                jump_point := get_end_jump_point_line(sparse, new_cell, new_direction, new_jump_distance);
                if jump_point {
                    return true, end_point_distance;
                }
            }
        }

        if direction.z != 0 {
            end_point_distance := end_point_delta.z;
            if end_point_distance >= 0 && abs(jump_distance) > end_point_distance {
                new_cell := cell + direction * end_point_distance;
                assert((`end_cell - new_cell).z == 0);

                new_direction := SparseDir.{direction.x, direction.y, 0};
                new_jump_distance := get_jump_distance_without_end(sparse, new_cell, new_direction);

                jump_point := get_end_jump_point_line(sparse, new_cell, new_direction, new_jump_distance);
                if jump_point {
                    return jump_point, end_point_distance;
                }
            }
        }

        return false, 0;
    }

    get_end_jump_point_corner :: (sparse: *GroupedSparseGrid(T), cell: SparseIndex, direction: SparseDir, jump_distance: s32) -> bool, s32 #expand {
        assert(abs(direction.x) + abs(direction.y) + abs(direction.z) == 3);

        end_point_delta := (`end_cell - cell) * direction;

        end_point_distance := end_point_delta.x;
        if end_point_distance >= 0 && abs(jump_distance) > end_point_distance {
            new_cell := cell + direction * end_point_distance;
            new_delta := `end_cell - new_cell;

            if new_delta.x == 0 && new_delta.y == 0 && new_delta.z == 0 {
                return true, end_point_distance;
            } else if new_delta.x == 0 && new_delta.y == 0 {
                new_direction := SparseDir.{0, 0, direction.z};
                new_jump_distance := get_jump_distance_without_end(sparse, new_cell, new_direction);

                jump_point := get_end_jump_point_line(sparse, new_cell, new_direction, new_jump_distance);
                if jump_point {
                    return jump_point, end_point_distance;
                }
            } else if new_delta.x == 0 && new_delta.z == 0 {
                new_direction := SparseDir.{0, direction.y, 0};
                new_jump_distance := get_jump_distance_without_end(sparse, new_cell, new_direction);

                jump_point := get_end_jump_point_line(sparse, new_cell, new_direction, new_jump_distance);
                if jump_point {
                    return jump_point, end_point_distance;
                }
            } else if new_delta.y == 0 && new_delta.z == 0 {
                new_direction := SparseDir.{direction.x, 0, 0};
                new_jump_distance := get_jump_distance_without_end(sparse, new_cell, new_direction);

                jump_point := get_end_jump_point_line(sparse, new_cell, new_direction, new_jump_distance);
                if jump_point {
                    return jump_point, end_point_distance;
                }
            } else {
                new_direction := SparseDir.{0, direction.y, direction.z};
                new_jump_distance := get_jump_distance_without_end(sparse, new_cell, new_direction);

                jump_point := get_end_jump_point_diagonal(sparse, new_cell, new_direction, new_jump_distance);
                if jump_point {
                    return jump_point, end_point_distance;
                }
            }
        }

        end_point_distance = end_point_delta.y;
        if end_point_distance >= 0 && abs(jump_distance) > end_point_distance {
            new_cell := cell + direction * end_point_distance;
            new_delta := `end_cell - new_cell;

            if new_delta.x == 0 && new_delta.y == 0 && new_delta.z == 0 {
                return true, end_point_distance;
            } else if new_delta.x == 0 && new_delta.y == 0 {
                new_direction := SparseDir.{0, 0, direction.z};
                new_jump_distance := get_jump_distance_without_end(sparse, new_cell, new_direction);

                jump_point := get_end_jump_point_line(sparse, new_cell, new_direction, new_jump_distance);
                if jump_point {
                    return jump_point, end_point_distance;
                }
            } else if new_delta.x == 0 && new_delta.z == 0 {
                new_direction := SparseDir.{0, direction.y, 0};
                new_jump_distance := get_jump_distance_without_end(sparse, new_cell, new_direction);

                jump_point := get_end_jump_point_line(sparse, new_cell, new_direction, new_jump_distance);
                if jump_point {
                    return jump_point, end_point_distance;
                }
            } else if new_delta.y == 0 && new_delta.z == 0 {
                new_direction := SparseDir.{direction.x, 0, 0};
                new_jump_distance := get_jump_distance_without_end(sparse, new_cell, new_direction);

                jump_point := get_end_jump_point_line(sparse, new_cell, new_direction, new_jump_distance);
                if jump_point {
                    return jump_point, end_point_distance;
                }
            } else {
                new_direction := SparseDir.{direction.x, 0, direction.z};
                new_jump_distance := get_jump_distance_without_end(sparse, new_cell, new_direction);

                jump_point := get_end_jump_point_diagonal(sparse, new_cell, new_direction, new_jump_distance);
                if jump_point {
                    return jump_point, end_point_distance;
                }
            }
        }

        end_point_distance = end_point_delta.z;
        if end_point_distance >= 0 && abs(jump_distance) > end_point_distance {
            new_cell := cell + direction * end_point_distance;
            new_delta := `end_cell - new_cell;

            if new_delta.x == 0 && new_delta.y == 0 && new_delta.z == 0 {
                return true, end_point_distance;
            } else if new_delta.x == 0 && new_delta.y == 0 {
                new_direction := SparseDir.{0, 0, direction.z};
                new_jump_distance := get_jump_distance_without_end(sparse, new_cell, new_direction);

                jump_point := get_end_jump_point_line(sparse, new_cell, new_direction, new_jump_distance);
                if jump_point {
                    return jump_point, end_point_distance;
                }
            } else if new_delta.x == 0 && new_delta.z == 0 {
                new_direction := SparseDir.{0, direction.y, 0};
                new_jump_distance := get_jump_distance_without_end(sparse, new_cell, new_direction);

                jump_point := get_end_jump_point_line(sparse, new_cell, new_direction, new_jump_distance);
                if jump_point {
                    return jump_point, end_point_distance;
                }
            } else if new_delta.y == 0 && new_delta.z == 0 {
                new_direction := SparseDir.{direction.x, 0, 0};
                new_jump_distance := get_jump_distance_without_end(sparse, new_cell, new_direction);

                jump_point := get_end_jump_point_line(sparse, new_cell, new_direction, new_jump_distance);
                if jump_point {
                    return jump_point, end_point_distance;
                }
            } else {
                new_direction := SparseDir.{direction.x, direction.y, 0};
                new_jump_distance := get_jump_distance_without_end(sparse, new_cell, new_direction);

                jump_point := get_end_jump_point_diagonal(sparse, new_cell, new_direction, new_jump_distance);
                if jump_point {
                    return jump_point, end_point_distance;
                }
            }
        }

        return false, 0;
    }

    component_count := abs(direction.x) + abs(direction.y) + abs(direction.z);
    if component_count == {
        case 1;
            jump_point, jump_point_distance := get_end_jump_point_line(sparse, cell, direction, jump_distance);
            return jump_point && jump_point_distance > 0, jump_point_distance;
        case 2;
            jump_point, jump_point_distance := get_end_jump_point_diagonal(sparse, cell, direction, jump_distance);
            return jump_point && jump_point_distance > 0, jump_point_distance;
        case 3;
            jump_point, jump_point_distance := get_end_jump_point_corner(sparse, cell, direction, jump_distance);
            return jump_point && jump_point_distance > 0, jump_point_distance;
    }

    return false, 0;
}

ForcedNeighbors :: struct {
    // -1, 0, 1
    x: [3] ForcedOffsets;
    y: [3] ForcedOffsets;
    z: [3] ForcedOffsets;
}

get_forced_neighbors :: (sparse: *GroupedSparseGrid($T), cell: SparseIndex, direction: SparseDir) -> ForcedOffsets #expand {
    forced_neighbors := get_forced_neighbors(sparse, cell);

    return forced_neighbors.x[direction.x + 1] | forced_neighbors.y[direction.y + 1] | forced_neighbors.z[direction.z + 1];
}

get_forced_neighbors :: (sparse: *GroupedSparseGrid($T), cell: SparseIndex) -> ForcedNeighbors #expand {
    return calculate_forced_neighbors(sparse, cell, `wall_type);
}

calculate_forced_neighbor :: (sparse: *GroupedSparseGrid($T), cell: SparseIndex, direction: SparseDir, wall_type: T) -> ForcedOffsets, bool #expand {
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
        jump_point_empty_cells[it_index] = get_cell(sparse, cell + empty_cell_direction, *`cached_grid);
        jump_point_wall_cells[it_index] = get_cell(sparse, cell + empty_cell_direction - direction, *`cached_grid);
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
        forced_neighbor := is_empty(jump_point_empty_cells[corner_indices[0]]);
        forced_neighbor = forced_neighbor && is_empty(jump_point_empty_cells[corner_indices[1]]);
        forced_neighbor = forced_neighbor && is_empty(jump_point_empty_cells[corner_indices[2]]);
        forced_neighbor = forced_neighbor && is_wall(jump_point_wall_cells[corner_indices[0]]);

        if forced_neighbor {
            forced_offsets |= forced_cell_offsets[corner_indices[0]];
        }
    }

    for diagonal_index: diagonal_indices {
        forced_neighbor := is_empty(jump_point_empty_cells[diagonal_index]);
        forced_neighbor = forced_neighbor && is_wall(jump_point_wall_cells[diagonal_index]);

        if forced_neighbor {
            forced_offsets |= forced_cell_offsets[diagonal_index];
        }
    }

    return forced_offsets, !!forced_offsets;
}

calculate_forced_neighbors :: (sparse: *GroupedSparseGrid($T), cell: SparseIndex, wall_type: T) -> ForcedNeighbors #expand {
    neighbors: ForcedNeighbors;
    neighbors.x[0] = calculate_forced_neighbor(sparse, cell, .{-1, 0, 0}, wall_type);
    neighbors.x[1] = .None;
    neighbors.x[2] = calculate_forced_neighbor(sparse, cell, .{1, 0, 0}, wall_type);
    neighbors.y[0] = calculate_forced_neighbor(sparse, cell, .{0, -1, 0}, wall_type);
    neighbors.y[1] = .None;
    neighbors.y[2] = calculate_forced_neighbor(sparse, cell, .{0, 1, 0}, wall_type);
    neighbors.z[0] = calculate_forced_neighbor(sparse, cell, .{0, 0, -1}, wall_type);
    neighbors.z[1] = .None;
    neighbors.z[2] = calculate_forced_neighbor(sparse, cell, .{0, 0, 1}, wall_type);
    return neighbors;
}

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
