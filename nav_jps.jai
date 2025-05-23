
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

// future optimizations
// need groups for cells, with a table of group counts that auto merges when groups join and auto split when groups split I guess
// need to immediate exit paths that dont exist
// aabb direction nearest bounds
// maybe need directional goal hints but thats kind of what the aabb thing is
// need to optimize the actual grid so there's less vertical stuff going on

// optimizing
// initial jps takes 2231 ms
// after baking jump distances takes 2.3 ms
// after fixing issues with jump distances it takes 9.242 ms......
// implementing a probably not correct fast stack it takes 9.07 ms.....

// new path with sparse grid hash table takes 15.4646 ms
// same thing with hash table but without cached grid takes 16.5038 ms
// changed some sparse grid code but still should be all the same, took 18.9117 ms

// without goal bounds checking it took 5.8 ms unoptimized
// with goal bounds checking it took 7.35 ms unoptimized

NavPath :: struct {
    path: [..] SparseIndex;
    path_groups: [..] u16;
}

NavInfo :: struct (T: Type) {
    walkable_type: T;
    empty_type: T;
}

path_find_jps :: (sparse: *SparseGrid($A), source_point: Vector3, target_point: Vector3, wall_type: $T, air_type: T, jump_height: float) -> NavPath {
    source_cell := get_cell_index(sparse.*, source_point);
    target_cell := get_cell_index(sparse.*, target_point);

    return path_find_jps(sparse, source_cell, target_cell, wall_type, air_type, jump_height);
}

path_find_jps :: (sparse: *SparseGrid($A), source_cell: SparseIndex, target_cell: SparseIndex, wall_type: $T, air_type: T, jump_height: float) -> NavPath {
    log_reset();
    defer log_print();

    log("path_find_jps");

    sparse.last_visited_search_index = (sparse.last_visited_search_index + 1) % U8_MAX;
    visited_search_index := sparse.last_visited_search_index;

    ForcedEntry :: struct {
        cell: SparseIndex;
        parent: SparseIndex;
        sparse_entry: *A;
        travelled_distance: float;
        heuristic_distance: float;
        air_distance: float;
        forced_offsets: ForcedOffsets;
    }

    sorted_insert :: insert;

    sorted_insert :: (array: *LinkedArray(ForcedEntry), entry: ForcedEntry) {
        linked_insert_sorted(array, entry, (a: ForcedEntry, b: ForcedEntry) -> float {
            return b.heuristic_distance - a.heuristic_distance;
        });
    }

    sorted_insert :: (array: *[..] ForcedEntry, entry: ForcedEntry) {
        for #v2 < i: 0..array.count - 1 {
            if entry.heuristic_distance <= array.*[i].heuristic_distance {
                array_insert_at(array, entry, i + 1);
                return;
            }
        }

        array_insert_at(array, entry, 0);
    }

    linked_pop :: (array: *[..] ForcedEntry) -> ForcedEntry {
        array.count -= 1;
        return array.data + array.count;
    }

    start_cell := source_cell;
    maximum_fall_cells :: 10;
    for 0..maximum_fall_cells - 1 {
        if is_empty(get_cell(sparse, start_cell).type, wall_type) {
            break;
        }

        start_cell.y -= 1;
    }
    if is_wall(get_cell(sparse, start_cell).type, wall_type) {
        // print("No source ground found.\n");
        return .{};
    }

    end_cell := target_cell;
    for 0..maximum_fall_cells - 1 {
        if is_empty(get_cell(sparse, end_cell).type, wall_type) {
            break;
        }

        end_cell.y -= 1;
    }
    if is_wall(get_cell(sparse, end_cell).type, wall_type) {
        // print("No target ground found.\n");
        return .{};
    }

    heuristic :: (a: ForcedEntry) -> float {
        return a.heuristic_distance;
    }

    open_queue: BinaryHeap(ForcedEntry, heuristic);
    open_queue.allocator = temp;
    reserve(*open_queue, 1024);

    start_entry := ForcedEntry.{
        start_cell,
        start_cell,
        get_cell_pointer(sparse, start_cell),
        0,
        grid_heuristic(start_cell, end_cell),
        0.0,
        .PX | .NX | .PY | .NY | .PZ | .NZ | .PXPZ | .PXNZ | .NXPZ | .NXNZ | .PXPY | .PXNY | .NXPY | .NXNY | .PYPZ | .PYNZ | .NYPZ | .NYNZ | .PXPYPZ | .PXPYNZ | .PXNYPZ | .PXNYNZ | .NXPYPZ | .NXPYNZ | .NXNYPZ | .NXNYNZ,
    };

    fast_stack: [..] ForcedEntry;
    fast_stack.allocator = temp;
    array_reserve(*fast_stack, 64);

    fast_stack_heuristic := start_entry.heuristic_distance;

    get_path :: (sparse: *SparseGrid(A), before_end_cell: SparseIndex, start_cell: SparseIndex, end_cell: SparseIndex, visited_search_index: u8) -> NavPath {
        log("get_path");

        path: NavPath;
        array_add(*path.path, end_cell);
        array_add(*path.path, before_end_cell);

        parent_cell := get_cell(sparse, before_end_cell);
        has_parent_cell := parent_cell.visited_search_index == visited_search_index;

        while has_parent_cell {
            array_add(*path.path, parent_cell.visited_parent);

            if parent_cell.visited_parent == start_cell {
                break;
            }

            next_parent_cell := get_cell(sparse, parent_cell.visited_parent);
            has_parent_cell = next_parent_cell.visited_search_index == visited_search_index;
            parent_cell = next_parent_cell;
        }

        reverse(path.path);

        return path;
    }

    check_count := 0;
    open_queue_count := 0;
    fast_stack_count := 0;

    max_open_queue_count := 0;
    max_fast_stack_count := 0;

    // valid_bounds_count = 0;
    // invalid_bounds_count = 0;

    visited_heuristic_skipped := 0;
    visited_heuristic_visited := 0;

    // defer print("Skipped % cells from heuristic and visited %.\n", visited_heuristic_skipped, visited_heuristic_visited);

    // defer print("Valid bounds checks were % and invalid bounds checks were %.\n", valid_bounds_count, invalid_bounds_count);

    // defer print("Checked % entries with % open queue entries and % fast stack entries.\n", check_count, open_queue_count, fast_stack_count);
    // defer print("Max open queue size was % and max fast stack size was %.\n", max_open_queue_count, max_fast_stack_count);

    push_open_entry :: (open_queue: *BinaryHeap(ForcedEntry, heuristic), fast_stack: *[..] ForcedEntry, entry: ForcedEntry) #expand {
        if entry.heuristic_distance <= `fast_stack_heuristic + 0.01 {
            log("push_fast_stack");
            array_add(fast_stack, entry);
            `fast_stack_count += 1;
            `max_fast_stack_count = max(`max_fast_stack_count, fast_stack.count);
        } else {
            log("push_open_queue");
            insert(open_queue, entry);
            `open_queue_count += 1;
            `max_open_queue_count = max(`max_open_queue_count, open_queue.entries.count);
        }
    }

    push_open_entry(*open_queue, *fast_stack, start_entry);

    offset_distances :: float.[
        1.0, 1.0, 1.0,
        1.0, 1.0, 1.0,
        1.414214, 1.414214, 1.414214, 1.414214,
        1.414214, 1.414214, 1.414214, 1.414214,
        1.414214, 1.414214, 1.414214, 1.414214,
        1.732051, 1.732051, 1.732051, 1.732051,
        1.732051, 1.732051, 1.732051, 1.732051,
    ];

    while fast_stack.count > 0 || open_queue.entries.count > 0 {
        defer check_count += 1;
        log("loop");

        forced_entry: ForcedEntry = ---;
        if fast_stack.count > 0 {
            forced_entry = fast_stack[fast_stack.count - 1];
            fast_stack.count -= 1;
        } else {
            forced_entry = pop_min(*open_queue);
            fast_stack_heuristic = forced_entry.heuristic_distance;
        }

        forced_cell := forced_entry.cell;
        parent_cell := forced_entry.parent;
        sparse_entry := forced_entry.sparse_entry;

        if sparse_entry.visited_search_index != visited_search_index {
            visited_heuristic_visited += 1;
            sparse_entry.visited_search_index = visited_search_index;
            sparse_entry.visited_parent = parent_cell;
        } else {
            visited_heuristic_skipped += 1;
            continue;
        }

        travelled_distance := forced_entry.travelled_distance;
        forced_offsets := forced_entry.forced_offsets;
        air_distance := forced_entry.air_distance;

        // TODO try inverting this? after I add to visited earlier?
        for < offset: offsets {
            offset_flag := cast(ForcedOffsets) (1 << it_index);

            if forced_offsets & offset_flag {
                jump_distance := get_jump_distance(sparse, forced_cell, sparse_entry, offset, end_cell);

                if jump_distance > 0 {
                    // I think the way forced neighbors work for type boundaries is
                    // the jump starts just inside the new cell type
                    // and it jumps to when the next type would begin
                    // so if the current cell type is air we add the jump distance
                    new_air_distance := air_distance;
                    if is_type(sparse_entry.type, air_type) && offset.y >= 0 {
                        new_air_distance += jump_distance * offset_distances[it_index];
                    } else {
                        new_air_distance = 0;
                    }

                    if new_air_distance > jump_height {
                        continue;
                    }

                    jump_cell := forced_cell + offset * jump_distance;
                    if jump_cell == end_cell {
                        return get_path(sparse, forced_cell, start_cell, end_cell, visited_search_index);
                    }

                    jump_entry := get_cell_pointer(sparse, jump_cell);
                    assert(!!jump_entry, "Found invalid cell pointer. %\n", jump_cell);

                    new_forced_offsets := get_forced_neighbors(sparse, jump_entry, offset);
                    new_forced_offsets |= travel_offsets[offset.x + 1][offset.y + 1][offset.z + 1];

                    if new_forced_offsets {
                        #if #run is_using(A, "WeightData") {
                            new_travelled_distance := travelled_distance + jump_distance * offset_distances[it_index] * get_weight(sparse_entry.weights[it_index]);
                        } else {
                            new_travelled_distance := travelled_distance + jump_distance * offset_distances[it_index];
                        }
                        new_visited_heuristic := new_travelled_distance + grid_heuristic(jump_cell, end_cell);

                        push_open_entry(*open_queue, *fast_stack, ForcedEntry.{
                            jump_cell,
                            forced_cell,
                            jump_entry,
                            new_travelled_distance,
                            new_visited_heuristic,
                            new_air_distance,
                            new_forced_offsets,
                        });
                    }
                }
            }
        }
    }

    return .{};
}

grid_heuristic :: (a: SparseIndex, b: SparseIndex) -> float {
    log("grid_heuristic");

    // move in the largest direction straight 1d line
    // move in the second largest direction 2d diagonal line
    // move in the third largest direction 3d diagonal line

    delta_x := fastabs(b.x - a.x);
    delta_y := fastabs(b.y - a.y);
    delta_z := fastabs(b.z - a.z);

    delta_max := fastmax(delta_x, fastmax(delta_y, delta_z));
    delta_min := fastmin(delta_x, fastmin(delta_y, delta_z));
    delta_mid := delta_x + delta_y + delta_z - delta_min - delta_max;

    // one_dim_count := delta_max - delta_mid;
    // two_dim_count := delta_mid - delta_min;
    // three_dim_count := delta_min;

    one_dim_length :: 1.0;
    two_dim_length :: 1.414214;
    three_dim_length :: 1.732051;

    return (delta_max - delta_mid) * one_dim_length + (delta_mid - delta_min) * two_dim_length + delta_min * three_dim_length;
}

is_wall :: (cell_type: $T, wall_type: T) -> bool {
    return !!(cell_type & wall_type);
}

is_empty :: (cell_type: $T, wall_type: T) -> bool {
    return !is_wall(cell_type, wall_type);
}

get_bounded_offsets :: (sparse: *SparseGrid($A), cell: SparseIndex, end_cell: SparseIndex, available_offsets: ForcedOffsets) -> ForcedOffsets {
    bounded_offsets: ForcedOffsets;
    goal_bounds := get_cell(sparse, cell).goal_bounds;
    for offset, offset_index: offsets {
        if !(available_offsets & cast(ForcedOffsets) (1 << offset_index)) {
            continue;
        }

        if contains(goal_bounds.bounds[offset_index], end_cell) {
            bounded_offsets |= cast(ForcedOffsets) (1 << offset_index);
        }
    }
    return bounded_offsets;
}

// valid_bounds_count := 0;
// invalid_bounds_count := 0;

// is_bounded_offset :: (cell: $A, end_cell: SparseIndex, offset_index: int) -> bool {
//     valid_bounds := contains(cell.goal_bounds.bounds[offset_index], end_cell);
//     valid_bounds_count += ifx valid_bounds then 1 else 0;
//     invalid_bounds_count += ifx valid_bounds then 0 else 1;
//     return valid_bounds;
// }

// a negative value ist he distance to the next wall
// a positive value is the distance to the next jump
// a 0 value isn't possible outside of a wall
get_jump_distance :: (sparse: *SparseGrid($A), cell: SparseIndex, cell_entry: *A, direction: SparseDir, end_cell: SparseIndex) -> s32, end: bool {
    log("get_jump_distance");
    potential_jump_distance := get_jump_distance_without_end(sparse, cell, cell_entry, direction);
    end_jump_point, end_jump_point_distance := get_end_jump_point_distance(sparse, cell, direction, potential_jump_distance, end_cell);

    return ifx end_jump_point then end_jump_point_distance else potential_jump_distance, end_jump_point;
}

get_jump_distance_without_end :: (sparse: *SparseGrid($A), cell: SparseIndex, direction: SparseDir) -> s32 {
    log("get_jump_distance_without_end");
    distance := get_step_distance(sparse, cell, direction);
    acc_distance := min(cast(s32) distance, S8_MAX - 1);

    while distance == S8_MAX {
        current_cell := cell + direction * acc_distance;

        distance = get_step_distance(sparse, current_cell, direction);
        acc_distance += min(fastabs(distance), S8_MAX - 1);

        if distance < 0 {
            return -acc_distance;
        }
    }

    return acc_distance;
}

get_jump_distance_without_end :: (sparse: *SparseGrid($A), cell: SparseIndex, cell_entry: *A, direction: SparseDir) -> s32 {
    log("get_jump_distance_without_end");
    distance := get_step_distance(sparse, cell_entry, direction);
    acc_distance := min(cast(s32) distance, S8_MAX - 1);

    while distance == S8_MAX {
        current_cell := cell + direction * acc_distance;

        distance = get_step_distance(sparse, current_cell, direction);
        acc_distance += min(fastabs(distance), S8_MAX - 1);

        if distance < 0 {
            return -acc_distance;
        }
    }

    return acc_distance;
}

get_step_distance :: (sparse: *SparseGrid($A), cell_entry: *A, direction: SparseDir) -> s8 {
    step_distances := cell_entry.distances;

    offset_index := offset_index_lookup[direction.x + 1][direction.y + 1][direction.z + 1];
    return step_distances[offset_index];
} @thread

get_step_distance :: (sparse: *SparseGrid($A), cell: SparseIndex, direction: SparseDir) -> s8 {
    step_distances := get_cell(sparse, cell).distances;

    offset_index := offset_index_lookup[direction.x + 1][direction.y + 1][direction.z + 1];
    return step_distances[offset_index];
}

get_end_jump_point_distance :: (sparse: *SparseGrid($A), cell: SparseIndex, direction: SparseDir, jump_distance: s32, end_cell: SparseIndex) -> bool, s32 {
    log("get_end_jump_point_distance");
    get_end_jump_point_line :: (sparse: *SparseGrid(A), cell: SparseIndex, direction: SparseDir, jump_distance: s32, end_cell: SparseIndex) -> bool, s32 {
        assert(fastabs(direction.x) + fastabs(direction.y) + fastabs(direction.z) == 1);

        // if jump_distance is positive, a jump point, we can return false if it's equal because it's already a jump point and we will visit it
        // if jump_distance is negative, a wall, we can return false if it's equal because we can't go inside the wall
        // lastly when you multiply the single component direction by the travel delta in order to get the plane distance
        // the signs multiply into each other and so only positive numbers are considered valid

        end_point_delta := (end_cell - cell) * direction;
        end_point_distance := end_point_delta.x + end_point_delta.y + end_point_delta.z;
        return end_point_distance >= 0 && fastabs(jump_distance) > end_point_distance, end_point_distance;
    }

    get_end_jump_point_diagonal :: (sparse: *SparseGrid(A), cell: SparseIndex, direction: SparseDir, jump_distance: s32, end_cell: SparseIndex) -> bool, s32 {
        assert(fastabs(direction.x) + fastabs(direction.y) + fastabs(direction.z) == 2);

        // I could be wrong about this
        // I think if you're moving in the +x+z direction, and you hit the +x normal end point plane wall as the closest end point plane
        // then actually you're not allowed to readjust to go up because that's not one of your valid movement directions
        // so the diagonal rule is that your empty direction must be aligned with the end point

        valid_empty_axis := direction.x == 0 && cell.x == end_cell.x;
        valid_empty_axis = valid_empty_axis || (direction.y == 0 && cell.y == end_cell.y);
        valid_empty_axis = valid_empty_axis || (direction.z == 0 && cell.z == end_cell.z);

        if !valid_empty_axis {
            return false, 0;
        }

        end_point_delta := (end_cell - cell) * direction;

        // check the diagonal move distance intersection
        if direction.x != 0 {
            end_point_distance := end_point_delta.x;
            if end_point_distance >= 0 && fastabs(jump_distance) > end_point_distance {
                new_cell := cell + direction * end_point_distance;
                assert((end_cell - new_cell).x == 0);

                new_direction := SparseDir.{0, direction.y, direction.z};
                new_jump_distance := get_jump_distance_without_end(sparse, new_cell, new_direction);

                jump_point := get_end_jump_point_line(sparse, new_cell, new_direction, new_jump_distance, end_cell);
                if jump_point {
                    return true, end_point_distance;
                }
            }
        }

        if direction.y != 0 {
            end_point_distance := end_point_delta.y;
            if end_point_distance >= 0 && fastabs(jump_distance) > end_point_distance {
                new_cell := cell + direction * end_point_distance;
                assert((end_cell - new_cell).y == 0);

                new_direction := SparseDir.{direction.x, 0, direction.z};
                new_jump_distance := get_jump_distance_without_end(sparse, new_cell, new_direction);

                jump_point := get_end_jump_point_line(sparse, new_cell, new_direction, new_jump_distance, end_cell);
                if jump_point {
                    return true, end_point_distance;
                }
            }
        }

        if direction.z != 0 {
            end_point_distance := end_point_delta.z;
            if end_point_distance >= 0 && fastabs(jump_distance) > end_point_distance {
                new_cell := cell + direction * end_point_distance;
                assert((end_cell - new_cell).z == 0);

                new_direction := SparseDir.{direction.x, direction.y, 0};
                new_jump_distance := get_jump_distance_without_end(sparse, new_cell, new_direction);

                jump_point := get_end_jump_point_line(sparse, new_cell, new_direction, new_jump_distance, end_cell);
                if jump_point {
                    return true, end_point_distance;
                }
            }
        }

        return false, 0;
    }

    get_end_jump_point_corner :: (sparse: *SparseGrid(A), cell: SparseIndex, direction: SparseDir, jump_distance: s32, end_cell: SparseIndex) -> bool, s32 {
        assert(fastabs(direction.x) + fastabs(direction.y) + fastabs(direction.z) == 3);

        end_point_delta := (end_cell - cell) * direction;

        end_point_distance := end_point_delta.x;
        if end_point_distance >= 0 && fastabs(jump_distance) > end_point_distance {
            new_cell := cell + direction * end_point_distance;
            aligned_cell := bool.[end_cell.x == new_cell.x, end_cell.y == new_cell.y, end_cell.z == new_cell.z];
            // new_delta := end_cell - new_cell;

            if aligned_cell[0] && aligned_cell[1] && aligned_cell[2] {
                return true, end_point_distance;
            } else if aligned_cell[0] && aligned_cell[1] {
                new_direction := SparseDir.{0, 0, direction.z};
                new_jump_distance := get_jump_distance_without_end(sparse, new_cell, new_direction);

                jump_point := get_end_jump_point_line(sparse, new_cell, new_direction, new_jump_distance, end_cell);
                if jump_point {
                    return true, end_point_distance;
                }
            } else if aligned_cell[0] && aligned_cell[2] {
                new_direction := SparseDir.{0, direction.y, 0};
                new_jump_distance := get_jump_distance_without_end(sparse, new_cell, new_direction);

                jump_point := get_end_jump_point_line(sparse, new_cell, new_direction, new_jump_distance, end_cell);
                if jump_point {
                    return true, end_point_distance;
                }
            } else if aligned_cell[1] && aligned_cell[2] {
                new_direction := SparseDir.{direction.x, 0, 0};
                new_jump_distance := get_jump_distance_without_end(sparse, new_cell, new_direction);

                jump_point := get_end_jump_point_line(sparse, new_cell, new_direction, new_jump_distance, end_cell);
                if jump_point {
                    return true, end_point_distance;
                }
            } else {
                new_direction := SparseDir.{0, direction.y, direction.z};
                new_jump_distance := get_jump_distance_without_end(sparse, new_cell, new_direction);

                jump_point := get_end_jump_point_diagonal(sparse, new_cell, new_direction, new_jump_distance, end_cell);
                if jump_point {
                    return true, end_point_distance;
                }
            }
        }

        end_point_distance = end_point_delta.y;
        if end_point_distance >= 0 && fastabs(jump_distance) > end_point_distance {
            new_cell := cell + direction * end_point_distance;
            aligned_cell := bool.[end_cell.x == new_cell.x, end_cell.y == new_cell.y, end_cell.z == new_cell.z];
            // new_delta := end_cell - new_cell;

            if aligned_cell[0] && aligned_cell[1] && aligned_cell[2] {
                return true, end_point_distance;
            } else if aligned_cell[0] && aligned_cell[1] {
                new_direction := SparseDir.{0, 0, direction.z};
                new_jump_distance := get_jump_distance_without_end(sparse, new_cell, new_direction);

                jump_point := get_end_jump_point_line(sparse, new_cell, new_direction, new_jump_distance, end_cell);
                if jump_point {
                    return true, end_point_distance;
                }
            } else if aligned_cell[0] && aligned_cell[2] {
                new_direction := SparseDir.{0, direction.y, 0};
                new_jump_distance := get_jump_distance_without_end(sparse, new_cell, new_direction);

                jump_point := get_end_jump_point_line(sparse, new_cell, new_direction, new_jump_distance, end_cell);
                if jump_point {
                    return true, end_point_distance;
                }
            } else if aligned_cell[1] && aligned_cell[2] {
                new_direction := SparseDir.{direction.x, 0, 0};
                new_jump_distance := get_jump_distance_without_end(sparse, new_cell, new_direction);

                jump_point := get_end_jump_point_line(sparse, new_cell, new_direction, new_jump_distance, end_cell);
                if jump_point {
                    return true, end_point_distance;
                }
            } else {
                new_direction := SparseDir.{direction.x, 0, direction.z};
                new_jump_distance := get_jump_distance_without_end(sparse, new_cell, new_direction);

                jump_point := get_end_jump_point_diagonal(sparse, new_cell, new_direction, new_jump_distance, end_cell);
                if jump_point {
                    return true, end_point_distance;
                }
            }
        }

        end_point_distance = end_point_delta.z;
        if end_point_distance >= 0 && fastabs(jump_distance) > end_point_distance {
            new_cell := cell + direction * end_point_distance;
            aligned_cell := bool.[end_cell.x == new_cell.x, end_cell.y == new_cell.y, end_cell.z == new_cell.z];
            // new_delta := end_cell - new_cell;

            if aligned_cell[0] && aligned_cell[1] && aligned_cell[2] {
                return true, end_point_distance;
            } else if aligned_cell[0] && aligned_cell[1] {
                new_direction := SparseDir.{0, 0, direction.z};
                new_jump_distance := get_jump_distance_without_end(sparse, new_cell, new_direction);

                jump_point := get_end_jump_point_line(sparse, new_cell, new_direction, new_jump_distance, end_cell);
                if jump_point {
                    return true, end_point_distance;
                }
            } else if aligned_cell[0] && aligned_cell[2] {
                new_direction := SparseDir.{0, direction.y, 0};
                new_jump_distance := get_jump_distance_without_end(sparse, new_cell, new_direction);

                jump_point := get_end_jump_point_line(sparse, new_cell, new_direction, new_jump_distance, end_cell);
                if jump_point {
                    return true, end_point_distance;
                }
            } else if aligned_cell[1] && aligned_cell[2] {
                new_direction := SparseDir.{direction.x, 0, 0};
                new_jump_distance := get_jump_distance_without_end(sparse, new_cell, new_direction);

                jump_point := get_end_jump_point_line(sparse, new_cell, new_direction, new_jump_distance, end_cell);
                if jump_point {
                    return true, end_point_distance;
                }
            } else {
                new_direction := SparseDir.{direction.x, direction.y, 0};
                new_jump_distance := get_jump_distance_without_end(sparse, new_cell, new_direction);

                jump_point := get_end_jump_point_diagonal(sparse, new_cell, new_direction, new_jump_distance, end_cell);
                if jump_point {
                    return true, end_point_distance;
                }
            }
        }

        return false, 0;
    }

    component_count := fastabs(direction.x) + fastabs(direction.y) + fastabs(direction.z);
    if component_count == {
        case 1;
            jump_point, jump_point_distance := get_end_jump_point_line(sparse, cell, direction, jump_distance, end_cell);
            return jump_point && jump_point_distance > 0, jump_point_distance;
        case 2;
            jump_point, jump_point_distance := get_end_jump_point_diagonal(sparse, cell, direction, jump_distance, end_cell);
            return jump_point && jump_point_distance > 0, jump_point_distance;
        case 3;
            jump_point, jump_point_distance := get_end_jump_point_corner(sparse, cell, direction, jump_distance, end_cell);
            return jump_point && jump_point_distance > 0, jump_point_distance;
    }

    return false, 0;
}

get_forced_neighbors :: (sparse: *SparseGrid($A), cell_entry: *A, direction: SparseDir) -> ForcedOffsets {
    log("get_forced_neighbors");
    forced_neighbors := cell_entry.forced_neighbors;

    return forced_neighbors[0][direction.x + 1] | forced_neighbors[1][direction.y + 1] | forced_neighbors[2][direction.z + 1];
}

#scope_file

logging :: false;
get_cell_time: float64;
sorted_insert_time: float64;
linked_pop_time: float64;
find_or_add_time: float64;
get_cell_position_time: float64;
startup_time: float64;

#if logging {
    seconds_since_init :: () -> float64 {
        return to_float64_seconds(current_time_consensus());
    }

    log_reset :: () {
        array_free(log_times);
        log_times = NewArray(logging_index, float64);
        array_free(log_names);
        log_names = NewArray(logging_index, string);
    }

    log_print :: () {
        for i: 0..logging_index - 1 {
            print("% took % ms.\n", log_names[i], log_times[i]);
        }
    }

    log :: ($name: string) #expand {
        current_index :: #run () -> int {
            index := logging_index;
            logging_index += 1;

            return index;
        }();

        log_names[current_index] = name;

        current_time := seconds_since_init();
        `defer log_times[current_index] += (seconds_since_init() - current_time) * 1000;
    }

    log_times: [] float64;
    log_names: [] string;
    #no_reset logging_index: int;
} else {
    log_reset :: () #expand {}

    log_print :: () #expand {}

    log :: (#discard name: string) #expand {}
}

// sorted_insert :: (array: *[..] $T, entry: T, $compare: (T, T) -> $R) -> *T {
//     // TODO I bet this is wrong somehow
//     lower_bound := 0;
//     upper_bound := array.count - 1;
//     while lower_bound <= upper_bound - 1 {
//         check_index := (lower_bound + upper_bound) / 2;

//         diff := compare(array.*[check_index], entry);
//         if diff < 0 {
//             lower_bound = check_index + 1;
//         } else if diff > 0 {
//             upper_bound = check_index;
//         } else {
//             lower_bound = check_index + 1;
//             upper_bound = check_index;
//         }
//     }

//     insert := max(lower_bound, upper_bound);
//     if array.count > 0 {
//         if compare(array.*[insert], entry) < 0 {
//             insert += 1;
//         }
//     }

//     array_insert_at(array, entry, insert);

//     #if true {
//         for i: 0..array.count - 2 {
//             if compare(array.*[i], array.*[i + 1]) > 0 {
//                 print("Sorted insert didn't work. %\n", entry);
//                 for entry: array.* {
//                     print("\t%\n", entry);
//                 }
//                 assert(false);
//             }
//         }
//     }

//     return *array.*[insert];
// }
