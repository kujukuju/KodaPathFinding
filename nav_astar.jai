
// this behavior is different from jps because it allows cutting through 0 width diagonals and corners

// NavPath :: struct {
//     path: [..] SparseIndex;
//     path_groups: [..] u16;
// }

// NavInfo :: struct (T: Type) {
//     walkable_type: T;
//     empty_type: T;
// }

// SearchPoint :: struct {
//     cell: SparseIndex;
//     distance_heuristic: float;
// }

// search_points: [..] SearchPoint;

// jump_pairs: [..] [2] SparseIndex;
// jump_checks: [..] [2] SparseIndex;
// wall_checks: [..] [2] SparseIndex;

// visualize_checks: int = 10000000;

// performance_test :: false;
// visualize :: false;

find_path_a_star :: (sparse: *SparseGrid($T), source_point: Vector3, target_point: Vector3) -> NavPath {
    source_cell := get_cell_index(sparse, source_point);
    target_cell := get_cell_index(sparse, target_point);

    return find_path_a_star(sparse, source_cell, target_cell);
}

find_path_a_star :: (sparse: *SparseGrid($T), source_cell: SparseIndex, target_cell: SparseIndex) -> NavPath {
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

    visited: SparseGrid(SparseVisited) = create_sparse_grid(SparseVisited, sparse.grid_size.x, sparse.grid_size.y, sparse.grid_size.z, sparse.cell_size);
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

            parent_cell := get_cell(*visited, search_entry.cell);
            has_parent_cell := parent_cell.visited_heuristic > 0;
            has_parent_cell = has_parent_cell && search_entry.cell != parent_cell.parent_index;

            while has_parent_cell {
                array_add(*path.path, parent_cell.parent_index);

                next_parent_cell := get_cell(*visited, parent_cell.parent_index);
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
            if get_cell(sparse, cell_index) != .Ground {
                continue;
            }

            parent_cell_distance := weights[it_index];
            distance_heuristic := search_entry.cell_distance + parent_cell_distance + grid_heuristic(cell_index, ground_target_cell);

            #if logging then start = seconds_since_init();
            parent_cell := get_or_add_cell_pointer(*visited, cell_index);
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
                // if get_cell(sparse, cell_index) == .Ground {
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
