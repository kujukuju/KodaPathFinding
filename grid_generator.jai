
GridGenerator :: struct (GridGeneratorType: Type, GridGeneratorData: Type, BodyType: Type) {
    character_count: int = 100;
    character_step_size: float;
    create_character_body: #type (data: GridGeneratorData) -> BodyType;
    create_air_test_body: #type (data: GridGeneratorData) -> BodyType;
    get_body_position: #type (body: BodyType, data: GridGeneratorData) -> Vector3;
    get_body_velocity: #type (body: BodyType, data: GridGeneratorData) -> Vector3;
    set_body_position: #type (body: BodyType, position: Vector3, data: GridGeneratorData);
    set_body_velocity: #type (body: BodyType, velocity: Vector3, data: GridGeneratorData);
    update_physics: #type (delta_time: float, data: GridGeneratorData);
    is_body_on_ground: #type (body: BodyType, data: GridGeneratorData) -> bool;
    destroy_body: #type (body: BodyType, data: GridGeneratorData);
    grid_cell_added_callback: #type (grid_index: SparseIndex, cell_index: SparseIndex);
    grid_cell_changed_callback: #type (grid_index: SparseIndex, cell_index: SparseIndex);
    // these are additional types you want to bake in that don't affect the pathfinding, so not ground air or empty
    get_cell_accent_type: #type (cell_index: SparseIndex, data: GridGeneratorData) -> GridGeneratorType;
    get_cell_weight: #type (type: GridGeneratorType) -> u8;
    gravity_direction: Vector3;
    // delta time of an update tick in seconds
    delta_time: float;
    empty_type: GridGeneratorType;
    ground_type: GridGeneratorType;
    air_type: GridGeneratorType;
    aabb: AABB3;
    data: GridGeneratorData;
    internals: struct {
        move_queue: [..] GridMove;
        visited_ground: Table(SparseIndex, bool, given_hash_function = sparse_hash, given_compare_function = sparse_equals);
        characters: [..] struct {
            body: BodyType;
            grid_move: GridMove;
            setting_position: bool;
            stabilizing_position: bool;
            moving: bool;
            move_stopping: bool;
            current_iteration_count: int;
            current_fall_count: int;
            start_position: Vector3;
            body_last_position: Vector3;
            last_ground: bool;
            moving_cells: [..] HistoryCell;
            line_cells_buffer: [..] HistoryCell;
        };
        air_queue_testing: bool;
        air_queue: [..] SparseIndex;
        air_tests: [..] struct {
            body: BodyType;
            test_cell: SparseIndex;
            setting_position: bool;
            sliding_down: bool;
            body_last_position: Vector3;
            current_iteration_count: int;
        };
    };
}

GridMove :: struct {
    from: SparseIndex;
    to: SparseIndex;
}

HistoryCell :: struct {
    cell: SparseIndex;
    // this is a cell you moved to
    // so if it's not ground, then it's air
    ground: bool;
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
    array_reset(*grid_generator.internals.move_queue);
    deinit(*grid_generator.internals.visited_ground);
    grid_generator.internals.visited_ground = .{};
    for *character: grid_generator.internals.characters {
        grid_generator.destroy_body(character.body, grid_generator.data);
        array_free(character.moving_cells);
        array_free(character.line_cells_buffer);
    }
    array_reset(*grid_generator.internals.characters);

    array_reset(*grid_generator.internals.air_queue);
    for *air: grid_generator.internals.air_tests {
        grid_generator.destroy_body(air.body, grid_generator.data);
    }
    array_reset(*grid_generator.internals.air_tests);
}

// TODO if I do *SparseGrid(JumpData(T)) it doesn't work
bake_distances :: (sparse: *SparseGrid($EXTREMELY_DUMB_BUG), wall_type: $T) {
    bake_distances :: (sparse: *SparseGrid(EXTREMELY_DUMB_BUG), indices: [] u8, wall_type: T) {
        for grid, grid_index: sparse.grids {
            for *cell, cell_index: grid {
                if cell.type == wall_type {
                    continue;
                }

                cell_sparse_index := get_cell_index(sparse, grid_index, cell_index);
                for index: indices {
                    direction := offsets[index];
                    cell.distances[index] = calculate_step_distance(sparse, cell_sparse_index, direction, wall_type);

                    #if #run is_using(EXTREMELY_DUMB_BUG, "WeightData") {
                        cell.weights[index] = get_weight_type(sparse, cell_sparse_index, direction, T);
                    }
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

bake_goal_bounds :: (sparse: *SparseGrid($EXTREMELY_DUMB_BUG), wall_type: $T) {
    // in jump point search (and what should happen in a* search) is that you can't cut diagonally through
    // cells that have filled neighbors. conceptually your path finding size is 1 block
    // for jps all that means is we will respect your jump distance values

    #import "System";
    #import "JaiParallel";

    core_count := max(get_number_of_processors() - 4, 1);
    print("Using % cores.\n", core_count);

    // we're going to allow 1 nearest direction per cell only
    // because it will greatly (I think) reduce the size of the goal bounds per direction
    // making the algorithm faster, and I dont't think the algorithm cares to choose between
    // two directions with the same heuristic... both are fine

    Visited :: struct {
        distances: [offsets.count] float;
    }

    SearchEntry :: struct {
        cell_index: SparseIndex;
        original_direction: ForcedOffsets;
        current_distance: float;
    }

    GoalBoundsRequest :: struct {
        // sparse: SparseGrid(EXTREMELY_DUMB_BUG);
        // best_offsets: SparseGrid(ForcedOffsets);
        visited_distances: SparseGrid([offsets.count] float);
        search_queue: BinaryHeap(SearchEntry, (a: SearchEntry) -> float {
            return a.current_distance;
        });
    }

    check_cells: [..] SparseIndex;
    cell_goal_bounds: [..] GoalBounds;
    defer array_free(check_cells);
    defer array_free(cell_goal_bounds);

    requests := NewArray(core_count, GoalBoundsRequest);

    for cell, cell_index: sparse {
        if cell.type == wall_type {
            continue;
        }
        array_add(*check_cells, cell_index);
    }

    array_resize(*cell_goal_bounds, check_cells.count);

    print("Total cell count is %.\n", check_cells.count);

    for *goal_bounds_request: requests {
        // goal_bounds_request.sparse = sparse_copy(sparse);
        // goal_bounds_request.best_offsets = create_sparse_grid(ForcedOffsets, sparse.grid_size.x, sparse.grid_size.y, sparse.grid_size.z, sparse.cell_size);
        goal_bounds_request.visited_distances = create_sparse_grid([offsets.count] float, sparse.grid_size.x, sparse.grid_size.y, sparse.grid_size.z, sparse.cell_size);
        reserve(*goal_bounds_request.search_queue, 1024);
    }

    defer {
        for *goal_bounds_request: requests {
            // sparse_free(*goal_bounds_request.sparse);
            // sparse_free(*goal_bounds_request.best_offsets);
            sparse_free(*goal_bounds_request.visited_distances);
            destroy(goal_bounds_request.search_queue);
        }
        array_free(requests);
    }

    // make cell request queue with an atomic increment index thing
    // and just pop from that

    // TODO you need to do each starting direction individually
    // because that way you can skip cells if the value is already the same
    // and then at the end you can combine all best offsets if the directional distances are the same
    // this would save you from having to do a million permutations per cell

    next_check_cell_index: s32;

    parallel_for(0, requests.count, (i: int, thread: int, using data: struct {
        sparse: *SparseGrid(EXTREMELY_DUMB_BUG);
        requests: [] GoalBoundsRequest;
        check_cells: [] SparseIndex;
        cell_goal_bounds: [] GoalBounds;
        wall_type: T;
        next_check_cell_index: *s32;
    }) {
        request := *requests[i];

        offset_distances :: float.[
            1.0, 1.0, 1.0,
            1.0, 1.0, 1.0,
            1.414214, 1.414214, 1.414214, 1.414214,
            1.414214, 1.414214, 1.414214, 1.414214,
            1.414214, 1.414214, 1.414214, 1.414214,
            1.732051, 1.732051, 1.732051, 1.732051,
            1.732051, 1.732051, 1.732051, 1.732051,
        ];

        atomic_increment :: (index: *s32) -> old: s32 {
            while true {
                current := index.*;
                success, old_value := compare_and_swap(index, current, current + 1);
                if success {
                    return old_value;
                }
            }
            return 0;
        }

        while true {
            check_cell_index := atomic_increment(next_check_cell_index);
            if check_cell_index >= check_cells.count {
                break;
            }
            cell_index := check_cells[check_cell_index];
            goal_bounds := *cell_goal_bounds[check_cell_index];

            // reset the nearest directions to every cell, go through and find all nearest directions
            // sparse_grid_reset(*request.best_offsets);
            sparse_grid_reset(*request.visited_distances);
            reset(*request.search_queue);

            starting_bounds := SparseAABB.{
                .{cast(s16) cell_index.x, cast(s16) cell_index.y, cast(s16) cell_index.z},
                .{cast(s16) cell_index.x, cast(s16) cell_index.y, cast(s16) cell_index.z},
            };

            for i: 0..offsets.count - 1 {
                goal_bounds.bounds[i] = starting_bounds;
            }

            // we go through every cell and build the bounds of all of the 26 or something directions
            for offset, offset_index: offsets {
                cell_entry := get_cell_pointer_uncached(sparse, cell_index);
                jump_distance := get_step_distance(sparse, cell_entry, offset);
                assert(jump_distance != 0, "I think jump distance can't ever be 0 unless you're in a wall.\n");

                // a jump distance of -1 means the very next cell is a wall so we don't check it
                if jump_distance == -1 {
                    continue;
                }

                next_cell_index := cell_index + offset;
                forced_offset := cast(ForcedOffsets) (1 << offset_index);

                visited_distances := get_or_add_cell_pointer(*request.visited_distances, next_cell_index);
                visited_distances.*[offset_index] = offset_distances[offset_index];

                next_entry := SearchEntry.{
                    next_cell_index,
                    forced_offset,
                    offset_distances[offset_index],
                };
                insert(*request.search_queue, next_entry);
            }

            while !is_empty(request.search_queue) {
                search_entry := pop_min(*request.search_queue);

                if search_entry.cell_index == cell_index {
                    continue;
                }

                // if this is at least currently the best cell for this guy, we will continue in this direction
                for offset, offset_index: offsets {
                    cell_entry := get_cell_pointer_uncached(sparse, search_entry.cell_index);
                    jump_distance := get_step_distance(sparse, cell_entry, offset);
                    assert(jump_distance != 0, "I think jump distance can't ever be 0 unless you're in a wall.\n");
                    if jump_distance == -1 {
                        continue;
                    }

                    next_cell_index := search_entry.cell_index + offset;
                    next_distance := search_entry.current_distance + offset_distances[offset_index];

                    visited_distances := get_or_add_cell_pointer(*request.visited_distances, next_cell_index);
                    if visited_distances.*[offset_index] == 0 || next_distance < visited_distances.*[offset_index] - 0.0001 {
                        visited_distances.*[offset_index] = next_distance;
                    } else {
                        continue;
                    }

                    next_entry := SearchEntry.{
                        next_cell_index,
                        search_entry.original_direction,
                        next_distance,
                    };

                    insert(*request.search_queue, next_entry);
                }
            }

            // at this point we've marked every reachable cell we can find as belonging to some nearest direction
            // so we can go and construct all of the goal bounds
            for visited_distances, visited_cell_index: request.visited_distances {
                best_distance := FLOAT32_MAX;
                best_component_count := 0;
                // iterate backwards to also capture the best component count where corner movements are best and line movements are worst
                for < distance: visited_distances {
                    if distance < best_distance - 0.0001 {
                        best_component_count = abs(offsets[it_index].x) + abs(offsets[it_index].y) + abs(offsets[it_index].z);
                        best_distance = distance;
                    }
                }

                if best_distance == FLOAT32_MAX {
                    continue;
                }

                // we're going to, if you can move diagonally and horizontally, only move diagonally since that's what jps expects?
                for distance: visited_distances {
                    component_count := abs(offsets[it_index].x) + abs(offsets[it_index].y) + abs(offsets[it_index].z);
                    if distance <= best_distance + 0.0001 && component_count >= best_component_count {
                        goal_bounds.bounds[it_index].lower.x = cast(s16) min(cast(s32) goal_bounds.bounds[it_index].lower.x, visited_cell_index.x);
                        goal_bounds.bounds[it_index].lower.y = cast(s16) min(cast(s32) goal_bounds.bounds[it_index].lower.y, visited_cell_index.y);
                        goal_bounds.bounds[it_index].lower.z = cast(s16) min(cast(s32) goal_bounds.bounds[it_index].lower.z, visited_cell_index.z);
                        goal_bounds.bounds[it_index].upper.x = cast(s16) max(cast(s32) goal_bounds.bounds[it_index].upper.x, visited_cell_index.x);
                        goal_bounds.bounds[it_index].upper.y = cast(s16) max(cast(s32) goal_bounds.bounds[it_index].upper.y, visited_cell_index.y);
                        goal_bounds.bounds[it_index].upper.z = cast(s16) max(cast(s32) goal_bounds.bounds[it_index].upper.z, visited_cell_index.z);
                    }
                }
            }

            if thread == 0 {
                print("Finished approximately %1%% of goal bounds generation. Done with %2/%3.\n", ((check_cell_index * 10000) / check_cells.count) / 100.0, check_cell_index, check_cells.count);
            }
        }
    }, .{
        sparse,
        requests,
        check_cells,
        cell_goal_bounds,
        wall_type,
        *next_check_cell_index,
    }, requests.count);

    for cell_index: check_cells {
        goal_bounds := cell_goal_bounds[it_index];
        cell_pointer := get_cell_pointer(sparse, cell_index);
        assert(!!cell_pointer, "Sparse cell for goal bounds doesn't exist.\n");

        cell_pointer.goal_bounds = goal_bounds;
    }
}

bake_forced_neighbors :: (sparse: *SparseGrid($EXTREMELY_DUMB_BUG), wall_type: $T) {
    for grid, grid_index: sparse.grids {
        for *cell, cell_index: grid {
            if cell.type == wall_type {
                continue;
            }

            cell_sparse_index := get_cell_index(sparse, grid_index, cell_index);
            cell.forced_neighbors = calculate_forced_neighbors(sparse, cell_sparse_index, wall_type);
        }
    }
}

iterate_air :: (using generator: *GridGenerator($T, $D, $B), sparse: *SparseGrid($EXTREMELY_DUMB_BUG)) -> bool {
    using internals;

    if !air_queue_testing {
        air_queue_testing = true;

        air_tests.count = 0;
        for i: 0..character_count - 1 {
            air_test := array_add(*air_tests);
            air_test.body = create_air_test_body(data);
        }

        for cell, cell_index: sparse {
            if cell.type == ground_type {
                array_add(*air_queue, cell_index);
            }
        }
    }

    should_continue := false;

    // pre physics update
    for *air_test: air_tests {
        using air_test;

        actively_moving := setting_position || sliding_down;
        if !actively_moving && air_queue.count == 0 {
            continue;
        } else if !actively_moving && air_queue.count > 0 {
            test_cell = air_queue[air_queue.count - 1];
            air_queue.count -= 1;

            setting_position = true;
            current_iteration_count = 0;
        }

        if current_iteration_count > 120 {
            setting_position = false;
            sliding_down = false;

            mark_air(generator, sparse, test_cell, changed = true);

            should_continue = true;
            continue;
        }

        current_iteration_count += 1;

        if setting_position {
            // here we set your body position to something like your cell center + 1 vertical cell
            start_position := get_cell_position(sparse.*, test_cell);
            start_position += .{0, sparse.cell_size, 0};

            set_body_position(body, start_position, data);

            continue;
        } else if sliding_down {
            body_velocity := get_body_velocity(body, data);
            body_velocity.x = 0;
            body_velocity.z = 0;

            set_body_velocity(body, body_velocity, data);

            continue;
        }

        assert(false, "Should never reach this point.\n");
    }

    update_physics(delta_time, data);

    for *air_test: air_tests {
        using air_test;

        actively_moving := setting_position || sliding_down;
        if !actively_moving {
            continue;
        }

        cell_position := get_cell_position(sparse.*, test_cell);

        // post physics update
        if setting_position {
            test_body_position := get_body_position(body, data);
            test_body_cell := get_cell_index(sparse, test_body_position);

            body_last_position = test_body_position;

            if test_body_cell.x != test_cell.x || test_body_cell.z != test_cell.z {
                // this is an invalid move we've been pushed outside of the cell
                mark_air(generator, sparse, test_cell, changed = true);

                setting_position = false;
                sliding_down = false;
            } else {
                setting_position = false;
                sliding_down = true;
            }

            should_continue = true;
            continue;
        } else if sliding_down {
            body_position := get_body_position(body, data);

            body_delta_position := body_position - body_last_position;
            body_last_position = body_position;

            on_ground := is_body_on_ground(body, data);
            stopped_moving := length(body_delta_position) < 0.01;

            went_past_cell := body_position.y < cell_position.y - sparse.cell_size;

            // print("on_ground %\n", on_ground);
            // print("stopped_moving %\n", stopped_moving);
            // print("went_past_cell %\n", went_past_cell);

            if on_ground {
                // this is in fact a valid ground cell so we skip it
                setting_position = false;
                sliding_down = false;
            } else if stopped_moving {
                // we stopped moving without being on the ground so its air
                setting_position = false;
                sliding_down = false;

                mark_air(generator, sparse, test_cell, changed = true);
            } else if went_past_cell {
                setting_position = false;
                sliding_down = false;

                mark_air(generator, sparse, test_cell, changed = true);
            }

            should_continue = true;
            continue;
        }

        assert(false, "Should never reach this point.\n");
    }

    return should_continue;
}

// TODO if I do *SparseGrid(JumpData(T)) it doesn't work
begin_nav_grid :: (using generator: *GridGenerator($T, $D, $B), sparse: *SparseGrid($EXTREMELY_DUMB_BUG), start: Vector3) {
    using internals;

    assert(ground_type != empty_type, "Invalid grid generator ground type.\n");
    assert(air_type != empty_type, "Invalid grid generator air type.\n");

    cell_index := get_cell_index(sparse, start);
    array_add(*move_queue, GridMove.{
        cell_index,
        cell_index,
    });

    characters.count = 0;
    for i: 0..character_count - 1 {
        character := array_add(*characters);

        character.body = create_character_body(data);
    }
}

// TODO this is gonna have to change later
// I need to make connections between the cells that actually let you walk between them
// because the algorithm can only move in cardinal directions against walls
// so the connection corners that need to be added are dependent on a real path existing.....

// TODO if I do *SparseGrid(JumpData(T)) it doesn't work
iterate_nav_grid :: (using generator: *GridGenerator($T, $D, $B), sparse: *SparseGrid($EXTREMELY_DUMB_BUG)) -> done: bool {
    using internals;

    is_already_attempting :: (using generator: *GridGenerator(T, D, B), move: GridMove) -> bool {
        using internals;

        for *character: characters {
            using character;

            actively_moving := setting_position || stabilizing_position || moving || move_stopping;
            if !actively_moving then continue;

            if grid_move.to == move.to/* || grid_move.from == move.from*/ {
                return true;
            }
        }

        return false;
    }

    should_continue := false;

    // pre move to set positions and velocities
    for *character: characters {
        using character;

        actively_moving := setting_position || stabilizing_position || moving || move_stopping;
        if !actively_moving && move_queue.count == 0 {
            continue;
        } else if !actively_moving && move_queue.count > 0 {
            valid_move_index := -1;
            for < move_entry: move_queue {
                if !is_already_attempting(generator, move_entry) {
                    valid_move_index = it_index;
                    break;
                }
            }

            if valid_move_index == -1 {
                continue;
            }

            grid_move = move_queue[valid_move_index];
            array_ordered_remove_by_index(*move_queue, valid_move_index);

            setting_position = true;

            if table_contains(*visited_ground, grid_move.to) {
                setting_position = false;
                stabilizing_position = false;
                moving = false;
                move_stopping = false;
                current_iteration_count = 0;
                current_fall_count = 0;

                should_continue = true;
                continue;
            }
        }

        start_position = get_body_position(body, data);

        if current_iteration_count > 120 || current_fall_count > 120 || !contains(aabb, get_cell_position(sparse.*, grid_move.from)) {
            setting_position = false;
            stabilizing_position = false;
            moving = false;
            move_stopping = false;
            current_iteration_count = 0;
            current_fall_count = 0;

            should_continue = true;
            continue;
        }

        if setting_position {
            // we're updating our position to the desired start cell
            set_body_position(body, get_cell_position(sparse.*, grid_move.from), data);

            continue;
        } else if stabilizing_position {
            // we're trying to stabilize into the desired start cell
            // so after we stop moving we can go on into finding the path to the next cell
            velocity := get_body_velocity(body, data);
            velocity.x = 0;
            velocity.z = 0;

            set_body_velocity(body, velocity, data);

            continue;
        } else if moving {
            // we will iteratively try to move into the actual desired start cell
            // in the xz plane only, and if we make it there we did it
            velocity := get_body_velocity(body, data);

            if move_stopping {
                velocity.x = 0;
                velocity.z = 0;
            } else {
                position := get_body_position(body, data);
                remaining_delta := get_cell_position(sparse.*, grid_move.to) - position;
                remaining_delta_xz := Vector3.{remaining_delta.x, 0, remaining_delta.z};
                if length(remaining_delta_xz) < character_step_size + 0.00001 {
                    velocity.x = remaining_delta.x;
                    velocity.z = remaining_delta.z;
                    move_stopping = true;
                } else {
                    remaining_delta_xz = normalize(remaining_delta_xz) * character_step_size;
                    velocity.x = remaining_delta_xz.x;
                    velocity.z = remaining_delta_xz.z;
                }
            }

            velocity.x /= delta_time;
            velocity.z /= delta_time;

            set_body_velocity(body, velocity, data);

            continue;
        }

        assert(false, "Should not reach end of character block.\n");
    }

    update_physics(delta_time, data);

    for *character: characters {
        using character;

        actively_moving := setting_position || stabilizing_position || moving || move_stopping;
        if !actively_moving {
            continue;
        }

        // TODO this isn't really right... it needs to be based on gravity speed * delta time or something
        defer {
            end_position := get_body_position(body, data);
            delta_position := end_position - start_position;

            if delta_position.y > -0.1 {
                current_iteration_count += 1;
            } else {
                current_fall_count += 1;
            }
        }

        if setting_position {
            setting_position = false;
            stabilizing_position = true;
            moving = false;
            move_stopping = false;
            current_iteration_count = 0;
            current_fall_count = 0;

            body_last_position = get_body_position(body, data);

            should_continue = true;
            continue;
        } else if stabilizing_position {
            position := get_body_position(body, data);

            delta_position := position - body_last_position;
            body_last_position = position;

            stopped_moving := length(delta_position) < 0.0001;

            if stopped_moving && is_body_on_ground(body, data) {
                setting_position = false;
                stabilizing_position = false;
                moving = true;
                current_iteration_count = 0;
                current_fall_count = 0;

                position_cell := get_cell_index(sparse.*, position);
                last_ground = true;

                moving_cells.count = 0;
                array_add(*moving_cells, HistoryCell.{
                    position_cell,
                    true,
                });
            }

            should_continue = true;
            continue;
        } else if moving {
            position := get_body_position(body, data);

            // TODO there should maybe be some logic of like you're only on the ground if you've been on the ground for 2 ticks
            on_ground := is_body_on_ground(body, data);
            next_last_ground := on_ground;
            defer last_ground = next_last_ground;
            on_ground = on_ground && last_ground;

            position_cell := get_cell_index(sparse.*, position);
            if moving_cells[moving_cells.count - 1].cell != position_cell {
                array_add(*moving_cells, HistoryCell.{
                    position_cell,
                    on_ground,
                });
            } else {
                moving_cells[moving_cells.count - 1].ground = moving_cells[moving_cells.count - 1].ground || on_ground;
            }

            delta_position := position - body_last_position;
            body_last_position = position;

            stopped_moving := length(delta_position) < 0.01;

            if stopped_moving {
                setting_position = false;
                stabilizing_position = false;
                moving = false;
                move_stopping = false;
                current_iteration_count = 0;
                current_fall_count = 0;

                // delta_first_cell := position_cell - grid_move.to;
                if on_ground { // && delta_first_cell.x == 0 && delta_first_cell.z == 0 /*&& abs(delta_first_cell.y) <= 1*/ {
                    line_cells_buffer.count = 0;
                    get_line(moving_cells, *line_cells_buffer);

                    for line_cell: line_cells_buffer {
                        // we don't count line cells as visited for the same logic of
                        // why we won't continue the iteration of these cells mid air
                        if line_cell.ground {
                            mark_ground(generator, sparse, line_cell.cell);
                        } else if get_cell(sparse, line_cell.cell).type == empty_type {
                            // we only want to set air on cells that are not ground, so empty
                            mark_air(generator, sparse, line_cell.cell);
                        }
                    }

                    if !table_contains(*visited_ground, position_cell) {
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

                    table_set(*visited_ground, position_cell, true);
                }
            }

            should_continue = true;
            continue;
        }

        assert(false, "Should never reach this point.\n");
    }

    return should_continue;
}

// TODO this method actually needs to look up its neighbors positions and try pretty hard to align
// itself, vertically with gravity, with its neighbors so that the actual jps algorithm can be fast
mark_ground :: (using generator: *GridGenerator($T, $D, $B), sparse: *SparseGrid($EXTREMELY_DUMB_BUG), cell_index: SparseIndex, changed: bool = false) {
    cell_type := ground_type;
    if get_cell_accent_type {
        cell_type |= get_cell_accent_type(cell_index, data);
    }
    set_cell(sparse, cell_index, cell_type);

    if changed && grid_cell_changed_callback then grid_cell_changed_callback(get_grid_index(sparse, cell_index), cell_index);
    if !changed && grid_cell_added_callback then grid_cell_added_callback(get_grid_index(sparse, cell_index), cell_index);
}

mark_air :: (using generator: *GridGenerator($T, $D, $B), sparse: *SparseGrid($EXTREMELY_DUMB_BUG), cell_index: SparseIndex, changed: bool = false) {
    cell_type := air_type;
    if get_cell_accent_type {
        cell_type |= get_cell_accent_type(cell_index, data);
    }
    set_cell(sparse, cell_index, cell_type);

    if changed && grid_cell_changed_callback then grid_cell_changed_callback(get_grid_index(sparse, cell_index), cell_index);
    if !changed && grid_cell_added_callback then grid_cell_added_callback(get_grid_index(sparse, cell_index), cell_index);
}

#scope_file

// you have to calculate step distances line -> diagonal -> corner
// so we can assume when you're calling calculate on a diagonal every single cell has it's line value completed
calculate_step_distance :: (sparse: *SparseGrid($EXTREMELY_DUMB_BUG), cell: SparseIndex, direction: SparseDir, wall_type: $T) -> s8 {
    // im going to do this such that positive numbers are forced neighbor distances
    // negative numbers are wall distances
    // and S8_MAX is a forced neighbor continuation indicator

    // the logic here is special in that if the current iteration direction hits a wall then it can't be a forced neighbor
    // this doesn't sound special but it makes it so that you can't just simply recurse forward you actually have to step

    calculate_jump_point :: (sparse: *SparseGrid(EXTREMELY_DUMB_BUG), jump_cell: EXTREMELY_DUMB_BUG, cell: SparseIndex, direction: SparseDir, wall_type: T) -> jump: bool {
        calculate_jump_point_line :: (sparse: *SparseGrid(EXTREMELY_DUMB_BUG), cell: SparseIndex, direction: SparseDir, wall_type: T) -> jump: bool {
            assert(abs(direction.x) + abs(direction.y) + abs(direction.z) == 1, "Line jump point check must have 1 component.\n");

            // the first cell you check CAN be a forced neighbor jump point
            // the first cell you check CANNOT be a wall

            current_cell := cell;

            if calculate_forced_neighbor(sparse, current_cell, direction, wall_type) {
                return true;
            }

            while true {
                current_cell += direction;

                if is_wall(get_cell(sparse, current_cell).type, wall_type) {
                    return false;
                }

                if calculate_forced_neighbor(sparse, current_cell, direction, wall_type) {
                    return true;
                }
            }

            return false;
        }

        calculate_jump_point_diagonal :: (sparse: *SparseGrid(EXTREMELY_DUMB_BUG), jump_cell: EXTREMELY_DUMB_BUG, cell: SparseIndex, direction: SparseDir, wall_type: T) -> jump: bool {
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

        calculate_jump_point_corner :: (sparse: *SparseGrid(EXTREMELY_DUMB_BUG), jump_cell: EXTREMELY_DUMB_BUG, cell: SparseIndex, direction: SparseDir, wall_type: T) -> jump: bool {
            assert(abs(direction.x) + abs(direction.y) + abs(direction.z) == 3, "Corner jump point check must have 3 components.\n");

            diagonal_directions := SparseDir.[
                .{direction.x, direction.y, 0},
                .{direction.x, 0, direction.z},
                .{0, direction.y, direction.z},
            ];

            // TODO this will check 3 diagonals which checks 6 lines
            // but really we just need to check 3 lines

            jump_point := calculate_jump_point_diagonal(sparse, jump_cell, cell, diagonal_directions[0], wall_type);
            jump_point = jump_point || calculate_jump_point_diagonal(sparse, jump_cell, cell, diagonal_directions[1], wall_type);
            jump_point = jump_point || calculate_jump_point_diagonal(sparse, jump_cell, cell, diagonal_directions[2], wall_type);
            return jump_point;
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

    // TODO since I always call this in order, line -> diagonal -> corner, can I not remove the recursion and just rely on jump values?

    direction_index := offset_index_lookup[direction.x + 1][direction.y + 1][direction.z + 1];

    is_wall_step :: (sparse: *SparseGrid($EXTREMELY_DUMB_BUG), step_cell: SparseIndex, direction: SparseDir, wall_type: $T) -> bool {
        origin_cell := step_cell - direction;

        component_count := fastabs(direction.x) + fastabs(direction.y) + fastabs(direction.z);
        if component_count == {
            case 1;
                return false;
            case 2;
                checks: [2] SparseDir = ---;
                if direction.x == 0 {
                    checks = .[.{0, direction.y, 0}, .{0, 0, direction.z}];
                } else if direction.y == 0 {
                    checks = .[.{direction.x, 0, 0}, .{0, 0, direction.z}];
                } else if direction.z == 0 {
                    checks = .[.{direction.x, 0, 0}, .{0, direction.y, 0}];
                }

                for check: checks {
                    if is_wall(get_cell(sparse, origin_cell + check).type, wall_type) {
                        return true;
                    }
                }
                return false;
            case 3;
                checks: [6] SparseDir = .[
                    .{direction.x, 0, 0}, .{0, direction.y, 0}, .{0, 0, direction.z},
                    .{direction.x, direction.y, 0}, .{direction.x, 0, direction.z}, .{0, direction.y, direction.z},
                ];

                for check: checks {
                    if is_wall(get_cell(sparse, origin_cell + check).type, wall_type) {
                        return true;
                    }
                }
                return false;
        }
        return false;
    }

    for i: cast(s8) 1..S8_MAX - 1 {
        check_cell := cell + direction * i;

        jump_cell := get_cell(sparse, check_cell);
        if is_wall(jump_cell.type, wall_type) {
            return -i;
        }

        if is_wall_step(sparse, check_cell, direction, wall_type) {
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

get_weight_type :: (sparse: *SparseGrid($EXTREMELY_DUMB_BUG), cell: SparseIndex, direction: SparseDir, $T: Type) -> T {
    cell_entry := get_cell_pointer(sparse, cell);
    offset_index := offset_index_lookup[direction.x + 1][direction.y + 1][direction.z + 1];
    jump_distance := cell_entry.distances[offset_index];
    if jump_distance == -1 {
        empty: T;
        return empty;
    }

    {
        // TODO test code
        invalid := false;
        start_weight := get_weight(get_cell(sparse, cell + direction).type);
        for i: cast(s32) 2..abs(jump_distance) - 1 {
            next_weight := get_weight(get_cell(sparse, cell + direction * i).type);
            invalid = start_weight != next_weight;
            if invalid break;
            assert(start_weight == next_weight, "Found a travel line with varying weights.\n");
        }

        if invalid {
            for i: cast(s32) 0..abs(jump_distance) - 1 {
                next_weight := get_weight(get_cell(sparse, cell + direction * i).type);
                print("%\n", next_weight);
            }
            assert(false, "Found a travel line with varying weights.\n");
        }
    }

    // next_cell_entry := get_cell(sparse, cell + direction);
    // return get_weight(next_cell_entry.type);
    return get_cell(sparse, cell + direction).type;
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

calculate_forced_neighbors :: (sparse: *SparseGrid($EXTREMELY_DUMB_BUG), cell: SparseIndex, wall_type: $T) -> [3] [3] ForcedOffsets {
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

calculate_forced_neighbor :: (sparse: *SparseGrid($EXTREMELY_DUMB_BUG), cell: SparseIndex, direction: SparseDir, wall_type: $T) -> ForcedOffsets, bool {
    assert(abs(direction.x) + abs(direction.y) + abs(direction.z) == 1, "You can only check for forced neighbors of a line.\n");

    // we assume in this function the cell you're checking must be valid

    empty_cell_directions: [8] SparseDir = ---;
    forced_cell_offsets: [8] ForcedOffsets = ---;
    forced_cell_corner_offsets: [8] ForcedOffsets = ---;
    jump_point_empty_cells: [8] T = ---;
    jump_point_empty_corner_cells: [8] T = ---;
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

    forward_cell := get_cell(sparse, cell + direction).type;

    for empty_cell_direction: empty_cell_directions {
        forced_cell_offsets[it_index] = get_forced_offset(empty_cell_direction);
        forced_cell_corner_offsets[it_index] = get_forced_offset(empty_cell_direction + direction);
        jump_point_empty_cells[it_index] = get_cell(sparse, cell + empty_cell_direction).type;
        jump_point_empty_corner_cells[it_index] = get_cell(sparse, cell + empty_cell_direction + direction).type;
        jump_point_wall_cells[it_index] = get_cell(sparse, cell + empty_cell_direction - direction).type;
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

        // now check true corner
        forced_neighbor = forced_neighbor && is_empty(forward_cell, wall_type);
        forced_neighbor = forced_neighbor && is_empty(jump_point_empty_corner_cells[corner_indices[0]], wall_type);
        forced_neighbor = forced_neighbor && is_empty(jump_point_empty_corner_cells[corner_indices[1]], wall_type);
        forced_neighbor = forced_neighbor && is_empty(jump_point_empty_corner_cells[corner_indices[2]], wall_type);

        if forced_neighbor {
            forced_offsets |= forced_cell_corner_offsets[corner_indices[0]];
        }
    }

    for diagonal_index: diagonal_indices {
        forced_neighbor := is_empty(jump_point_empty_cells[diagonal_index], wall_type);
        forced_neighbor = forced_neighbor && is_wall(jump_point_wall_cells[diagonal_index], wall_type);

        if forced_neighbor {
            forced_offsets |= forced_cell_offsets[diagonal_index];
        }

        // now check true corner
        forced_neighbor = forced_neighbor && is_empty(forward_cell, wall_type);
        forced_neighbor = forced_neighbor && is_empty(jump_point_empty_corner_cells[diagonal_index], wall_type);

        if forced_neighbor {
            forced_offsets |= forced_cell_corner_offsets[diagonal_index];
        }
    }

    if is_using(EXTREMELY_DUMB_BUG, "WeightData") {
        start_weight := get_weight(get_cell(sparse, cell - direction).type);

        // TODO I'm going to mark every cel that changes weight as as jump cell with forced neighbors
        // however in reality I think you mark cells that you enter that lower weight as jump cells
        // and you mark cells that are wall-configured forced neighbors that gain weight as jump cells

        // im going to return forward as a forced neighbor if the weight changes to trigger the jump distance result

        forward_weight := get_weight(get_cell(sparse, cell).type);

        if start_weight != forward_weight {
            forced_offsets |= get_forced_offset(direction);
        }

        for corner_indices: corner_indices_groups {
            // all the cells are empty, so it isn't a traditional forced neighbor
            // and also the cell we're trying to go to is a different weight
            forced_neighbor := is_empty(jump_point_empty_cells[corner_indices[0]], wall_type);
            forced_neighbor = forced_neighbor && is_empty(jump_point_empty_cells[corner_indices[1]], wall_type);
            forced_neighbor = forced_neighbor && is_empty(jump_point_empty_cells[corner_indices[2]], wall_type);
            // forced_neighbor = forced_neighbor && is_empty(jump_point_wall_cells[corner_indices[0]], wall_type);
            forced_neighbor = forced_neighbor && get_weight(jump_point_empty_cells[corner_indices[0]]) != start_weight;

            if forced_neighbor {
                forced_offsets |= forced_cell_offsets[corner_indices[0]];
            }

            // now check true corner
            forced_neighbor = forced_neighbor && is_empty(forward_cell, wall_type);
            forced_neighbor = forced_neighbor && is_empty(jump_point_empty_corner_cells[corner_indices[0]], wall_type);
            forced_neighbor = forced_neighbor && is_empty(jump_point_empty_corner_cells[corner_indices[1]], wall_type);
            forced_neighbor = forced_neighbor && is_empty(jump_point_empty_corner_cells[corner_indices[2]], wall_type);
            forced_neighbor = forced_neighbor && get_weight(jump_point_empty_corner_cells[corner_indices[0]]) != start_weight;

            if forced_neighbor {
                forced_offsets |= forced_cell_corner_offsets[corner_indices[0]];
            }
        }

        for diagonal_index: diagonal_indices {
            forced_neighbor := is_empty(jump_point_empty_cells[diagonal_index], wall_type);
            // forced_neighbor = forced_neighbor && is_empty(jump_point_wall_cells[diagonal_index], wall_type);
            forced_neighbor = forced_neighbor && get_weight(jump_point_empty_cells[diagonal_index]) != start_weight;

            if forced_neighbor {
                forced_offsets |= forced_cell_offsets[diagonal_index];
            }

            // now check true corner
            forced_neighbor = forced_neighbor && is_empty(forward_cell, wall_type);
            forced_neighbor = forced_neighbor && is_empty(jump_point_empty_corner_cells[diagonal_index], wall_type);
            forced_neighbor = forced_neighbor && get_weight(jump_point_empty_corner_cells[diagonal_index]) != start_weight;

            if forced_neighbor {
                forced_offsets |= forced_cell_corner_offsets[diagonal_index];
            }
        }
    }

    return forced_offsets, !!forced_offsets;
}

get_line :: (points: [] HistoryCell, result: *[..] HistoryCell) {
    for i: 0..points.count - 2 {
        start_result_index := result.count;
        get_line(points[i], points[i + 1], result);

        if start_result_index > 0 {
            if result.*[start_result_index].cell == result.*[start_result_index - 1].cell {
                result.*[start_result_index - 1].ground = result.*[start_result_index - 1].ground || result.*[start_result_index].ground;
                array_ordered_remove_by_index(result, start_result_index);
            }
        }
    }
}

// TODO I could add in a floating point delta value that I could use to decide which direction the line moves in for the next iteration
get_line :: (p1: HistoryCell, p2: HistoryCell, result: *[..] HistoryCell) {
    current := p1.cell;

    if p1.cell == p2.cell {
        array_add(result, HistoryCell.{
            p1.cell,
            p1.ground && p2.ground,
        });

        return;
    }

    array_add(result, HistoryCell.{
        current,
        p1.ground,
    });

    intermediary_ground := p1.ground && p2.ground;

    while current != p2.cell {
        pdelta := p2.cell - current;

        if abs(pdelta.x) >= abs(pdelta.y) && abs(pdelta.x) >= abs(pdelta.z) {
            current += SparseIndex.{pdelta.x, 0, 0};
        } else if abs(pdelta.y) >= abs(pdelta.x) && abs(pdelta.y) >= abs(pdelta.z) {
            current += SparseIndex.{0, pdelta.y, 0};
        } else if abs(pdelta.z) >= abs(pdelta.x) && abs(pdelta.z) >= abs(pdelta.y) {
            current += SparseIndex.{0, 0, pdelta.z};
        }

        array_add(result, HistoryCell.{
            current,
            intermediary_ground,
        });
    }

    result.*[result.count - 1].ground = p2.ground;
}

