
GroupedSparseGrid :: struct (T: Type) {
    grid: SparseGrid(SparseEntry(T));
    groups: Table(u16, [..] SparseIndex);
}

SparseEntry :: struct (T: Type) {
    data: T;
    group: u16;
}

SparseConstructionInfo :: struct (T: Type, D: Type) {
    gravity_direction: Vector3;
    get_cell_empty: (cell: AABB3, data: D) -> bool;
    get_grid_empty: (grid: AABB3, data: D) -> bool;
    empty_type: T;
    ground_type: T;
    data: D;
}

// a grouped sparse grid is a sparse grid that contains connection information for visually continuous sections of mesh
// this, I think, will heavily optimize the path finding algorithm

// TODO
// write the actual path finding algorithm and make it visual in the game
// then go through and generate the sparse groups every time you add a new cell

create_grouped_sparse_grid :: ($T: Type, grid_size_x: int, grid_size_y: int, grid_size_z: int, cell_size: float) -> GroupedSparseGrid(T) {
    grouped: GroupedSparseGrid(T);
    grouped.grid = create_sparse_grid(SparseEntry(T), grid_size_x, grid_size_y, grid_size_z, cell_size);

    return grouped;
}

for_expansion :: (sparse: GroupedSparseGrid($T), body: Code, flags: For_Flags) #expand {
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

construct_grouped_sparse_grid :: (sparse: *GroupedSparseGrid($T), aabb: AABB3, info: SparseConstructionInfo(T, $D)) {
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
            cell_start_z := max(grid_z * sparse.grid.grid_size_z, start_index.z);
            cell_end_z := min(cell_start_z + sparse.grid.grid_size_z - 1, end_index.z);

            for grid_y: start_grid_index.y..end_grid_index.y {
                cell_start_y := max(grid_y * sparse.grid.grid_size_y, start_index.y);
                cell_end_y := min(cell_start_y + sparse.grid.grid_size_y - 1, end_index.y);

                for grid_x: start_grid_index.x..end_grid_index.x {
                    cell_start_x := max(grid_x * sparse.grid.grid_size_x, start_index.x);
                    cell_end_x := min(cell_start_x + sparse.grid.grid_size_x - 1, end_index.x);

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
}

get_cell :: (sparse: *GroupedSparseGrid($T), point: Vector3) -> T, bool {
    entry, found := get_cell(*sparse.grid, point);
    return entry.data, found;
}

get_cell :: (sparse: *GroupedSparseGrid($T), cell_index: SparseIndex) -> T, bool {
    entry, found := get_cell(*sparse.grid, cell_index);
    return entry.data, found;
}

set_cell :: (sparse: *GroupedSparseGrid($T), point: Vector3, data: T) {
    entry: SparseEntry(T);
    entry.data = data;
    set_cell(*sparse.grid, point, entry);
}

set_cell :: (sparse: *GroupedSparseGrid($T), cell_index: SparseIndex, data: T) {
    entry: SparseEntry(T);
    entry.data = data;
    set_cell(*sparse.grid, cell_index, entry);
}

get_nearest_cell :: (sparse: *GroupedSparseGrid($T), line: Line3, valid: T) -> SparseIndex, bool {
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

get_memory_size :: (sparse: GroupedSparseGrid($T)) -> grid_size: int, groups_size: int {
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
