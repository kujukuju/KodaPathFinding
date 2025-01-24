
// features
// jumping
// have connectivity groupings to know if its even possible to go from one place to another
// visualize every cell its visiting... maybe with color gradient

// optimizations
// instead of a hash table for visited we're gonna reuse the sparse grid
// and then when we're doing our actual navigation iteration we're gonna somehow, really intelligently, hold on
// to the current grid until it's invalid before sampling to get a new grid
// and we'll also do that for the visited lookup

NavPath :: struct {
    path: [..] SparseIndex;
    path_groups: [..] u16;
}

NavInfo :: struct (T: Type) {
    walkable_type: T;
    empty_type: T;
}

SearchEntry :: struct {
    cell: SparseIndex;
    cell_point: Vector3;
    cell_distance: float;
    heuristic_distance: float;
    from_direction: [3] s8;
}

find_path :: (sparse: *GroupedSparseGrid($T), source_point: Vector3, target_point: Vector3) -> NavPath {
    source_cell := get_cell_index(sparse.grid, source_point);
    target_cell := get_cell_index(sparse.grid, target_point);

    return find_path(sparse, source_cell, target_cell);
}

find_path :: (sparse: *GroupedSparseGrid($T), source_cell: SparseIndex, target_cell: SparseIndex) -> NavPath {
    seconds_since_init :: () -> float64 {
        // this is like twice as fast as current_time_monotonic
        time := current_time_consensus();
        return  to_float64_seconds(time);
    }

    start := seconds_since_init();

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

    end_point := get_cell_position(sparse.grid, ground_target_cell);

    sorted_insert :: (array: *LinkedArray(SearchEntry), entry: SearchEntry) {
        linked_insert_sorted(array, entry, (a: SearchEntry, b: SearchEntry) -> float {
            return b.heuristic_distance - a.heuristic_distance;
        });
    }

    empty_cell :: SparseIndex.{S32_MAX, S32_MAX, S32_MAX};

    visited: SparseGrid(SparseIndex) = create_sparse_grid(SparseIndex, sparse.grid.grid_size_x, sparse.grid.grid_size_y, sparse.grid.grid_size_z, sparse.grid.cell_size);
    visited.allocator = temp;

    search_queue: LinkedArray(SearchEntry);
    linked_reserve(*search_queue, 1024);

    // linked_pop :: (arr: *[..] SearchEntry) -> SearchEntry {
    //     item := arr.*[arr.count - 1];
    //     arr.count -= 1;
    //     return item;
    // }

    // sorted_insert :: (array: *[..] SearchEntry, entry: SearchEntry) {
    //     insert_index := array.count;
    //     for < curr, index: array.* {
    //         if curr.heuristic_distance - entry.heuristic_distance >= 0 {
    //             break;
    //         }
    //         insert_index = index;
    //     }
    //     array_insert_at(array, entry, insert_index);
    // }

    ground_source_position := get_cell_position(sparse.grid, ground_source_cell);
    sorted_insert(*search_queue, SearchEntry.{
        ground_source_cell,
        ground_source_position,
        0,
        distance(ground_source_position, end_point),
        .[0, 0, 0],
    });

    set_cell(*visited, ground_source_cell, ground_source_cell);

    // table_set(visited, ground_source_cell, ground_source_cell);

    check_count := 0;

    defer print("get_cell_time % ms\n", get_cell_time * 1000);
    defer print("sorted_insert_time % ms\n", sorted_insert_time * 1000);
    defer print("linked_pop_time % ms\n", linked_pop_time * 1000);
    defer print("find_or_add_time % ms\n", find_or_add_time * 1000);
    defer print("get_cell_position_time % ms\n", get_cell_position_time * 1000);
    defer print("startup_time % ms\n", startup_time * 1000);

    startup_time += seconds_since_init() - start;
    start = 0;

    while search_queue.count > 0 {
        defer check_count += 1;
        print_items :: (linked: LinkedArray($T)) {
            for item: linked {
                print("% ", item.heuristic_distance);
            }
            print("\n");
        }

        // print_items(search_queue);

        start = seconds_since_init();
        search_entry := linked_pop(*search_queue);
        linked_pop_time += seconds_since_init() - start;
        start = 0;

        if search_entry.cell == ground_target_cell {
            start = seconds_since_init();
            defer print("Path lookup time % ms\n", (seconds_since_init() - start) * 1000);

            path: NavPath;
            array_add(*path.path, search_entry.cell);

            parent_cell := get_cell(*visited, search_entry.cell);
            has_parent_cell := parent_cell != empty_cell;
            has_parent_cell = has_parent_cell && search_entry.cell != parent_cell;

            while has_parent_cell {
                array_add(*path.path, parent_cell);

                next_parent_cell := get_cell(*visited, parent_cell);
                has_parent_cell = next_parent_cell != empty_cell;
                has_parent_cell = has_parent_cell && next_parent_cell != parent_cell;
                parent_cell = next_parent_cell;
            }

            print("Finished with % left in queue after checking % entries.\n", search_queue.count, check_count);

            reverse(path.path);
            return path;
        }

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

        for offset: offsets {
            offset_x := offset[0];
            offset_y := offset[1];
            offset_z := offset[2];

            // should_skip := offset_x != 0 && offset_x == -search_entry.from_direction[0];
            // should_skip = should_skip || (offset_y != 0 && offset_y == -search_entry.from_direction[1]);
            // should_skip = should_skip || (offset_z != 0 && offset_z == -search_entry.from_direction[2]);
            // if should_skip {
            //     continue;
            // }

            cell_index := SparseIndex.{search_entry.cell.x + offset_x, search_entry.cell.y + offset_y, search_entry.cell.z + offset_z};
            start = seconds_since_init();
            parent_cell := get_or_add_cell_pointer(*visited, cell_index);
            new_cell := parent_cell.* == empty_cell;
            find_or_add_time += seconds_since_init() - start;
            start = 0;
            if new_cell {
                // set the parent lookup if you are adding a new entry
                parent_cell.* = search_entry.cell;

                start = seconds_since_init();
                if get_cell(sparse, cell_index) == .Ground {
                    if start {
                        get_cell_time += seconds_since_init() - start;
                        start = 0;
                    }
                    start = seconds_since_init();
                    parent_cell_distance := length(Vector3.{cast(float) offset_x, cast(float) offset_y, cast(float) offset_z} * sparse.grid.cell_size);
                    cell_position := get_cell_position(sparse.grid, cell_index);
                    get_cell_position_time += seconds_since_init() - start;
                    start = 0;

                    start = seconds_since_init();
                    sorted_insert(*search_queue, SearchEntry.{
                        cell_index,
                        get_cell_position(sparse.grid, cell_index),
                        search_entry.cell_distance + parent_cell_distance,
                        search_entry.cell_distance + parent_cell_distance + distance(cell_position, end_point),
                        offset,
                    });
                    sorted_insert_time += seconds_since_init() - start;
                    start = 0;
                }
                if start {
                    get_cell_time += seconds_since_init() - start;
                    start = 0;
                }
            }
        }
    }

    return .{};
}

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

// sorted_insert :: (linked: *LinkedArray($T), entry: T, $compare: (T, T, data: $D) -> $R, data: D) -> *T {

// }

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
