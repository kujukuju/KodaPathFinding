
sparse_table :: true;
cached_grid :: true;

SparseGrid :: struct (SparseGridType: Type) {
#if sparse_table {
    grids: Table(SparseIndex, [] SparseGridType, given_hash_function = sparse_hash, given_compare_function = sparse_equals);
} else {
    grids: [..] [] T;
}
    cached_grid: CachedGrid(SparseGridType);
#if !sparse_table {
    grid_min: SparseIndex;
    grid_max: SparseIndex;
}
    grid_size: SparseIndex;
    cell_size: float;
    allocator: Allocator;
}

SerializableGrid :: struct (T: Type) {
    index: SparseIndex;
    cells: [] T;
}

SerializableSparseGrid :: struct (T: Type) {
    grids: [] SerializableGrid(T);
    grid_size: SparseIndex;
    cell_size: float;
}

CachedGrid :: struct (T: Type) {
    grid_index: SparseIndex = .{S32_MAX, S32_MAX, S32_MAX};
    grid: [] T;
}

#if sparse_table {
    serialize_grid :: (sparse: SparseGrid($T)) -> SerializableSparseGrid(T) {
        serializable: SerializableSparseGrid(T);
        serializable.grids = NewArray(sparse.grids.count, SerializableGrid(T));
        index := 0;
        for grid, grid_index: sparse.grids {
            defer index += 1;

            serializable.grids[index].cells = grid;
            serializable.grids[index].index = grid_index;
        }
        serializable.grid_size = sparse.grid_size;
        serializable.cell_size = sparse.cell_size;
        return serializable;
    }
} else {
    serialize_grid :: (sparse: SparseGrid($T)) -> SerializableSparseGrid(T) {
        serializable: SerializableSparseGrid(T);
        serializable.grids = NewArray(sparse.grids.count, SerializableGrid(T));
        index := 0;
        for grid, grid_index: sparse.grids {
            defer index += 1;

            serializable.grids[index].cells = grid;
            serializable.grids[index].index = get_grid_index(sparse, grid_index);
        }
        serializable.grid_size = sparse.grid_size;
        serializable.cell_size = sparse.cell_size;
        return serializable;
    }
}

deserialize_grid :: (serialized: SerializableSparseGrid($T)) -> SparseGrid(T) {
    sparse: SparseGrid(T);
    sparse.allocator = context.allocator;
    sparse.grid_size = serialized.grid_size;
    sparse.cell_size = serialized.cell_size;
    for grid: serialized.grids {
        set_grid(*sparse, grid.index, grid.cells);
    }
    return sparse;
}

// this is a hashmap of grids
// inside each grid you have a normal 3d array of cells with the specified grid size
// so grid -> cell

create_sparse_grid :: ($T: Type, grid_size_x: int, grid_size_y: int, grid_size_z: int, cell_size: float) -> SparseGrid(T) {
    sparse: SparseGrid(T);
    initialize(*sparse, grid_size_x, grid_size_y, grid_size_z, cell_size);

    return sparse;
}

initialize :: (sparse: *SparseGrid($T), grid_size_x: int, grid_size_y: int, grid_size_z: int, cell_size: float) {
    sparse.grid_size.x = cast(s32) grid_size_x;
    sparse.grid_size.y = cast(s32) grid_size_y;
    sparse.grid_size.z = cast(s32) grid_size_z;
    sparse.cell_size = cell_size;
}

#if sparse_table {
    for_expansion :: (sparse: *SparseGrid($T), body: Code, flags: For_Flags) #expand {
        #assert(!(flags & .REVERSE));

        for grid, grid_index: sparse.grids {
            #if flags & .POINTER {
                for *cell, cell_index: grid {
                    cell_x := cell_index % sparse.grid_size.x + grid_index.x * sparse.grid_size.x;
                    cell_y := (cell_index % (sparse.grid_size.x * sparse.grid_size.y)) / sparse.grid_size.x + grid_index.y * sparse.grid_size.y;
                    cell_z := cell_index / (sparse.grid_size.x * sparse.grid_size.y) + grid_index.z * sparse.grid_size.z;

                    `it := cell;
                    `it_index := SparseIndex.{cast(s32) cell_x, cast(s32) cell_y, cast(s32) cell_z};
                    #insert body;
                }
            } else {
                for cell, cell_index: grid {
                    cell_x := cell_index % sparse.grid_size.x + grid_index.x * sparse.grid_size.x;
                    cell_y := (cell_index % (sparse.grid_size.x * sparse.grid_size.y)) / sparse.grid_size.x + grid_index.y * sparse.grid_size.y;
                    cell_z := cell_index / (sparse.grid_size.x * sparse.grid_size.y) + grid_index.z * sparse.grid_size.z;

                    `it := cell;
                    `it_index := SparseIndex.{cast(s32) cell_x, cast(s32) cell_y, cast(s32) cell_z};
                    #insert body;
                }
            }
        }
    }
} else {
    for_expansion :: (sparse: *SparseGrid($T), body: Code, flags: For_Flags) #expand {
        #assert(!(flags & .REVERSE));

        for grid, grid_index: sparse.grids {
            #if flags & .POINTER {
                for *cell, cell_index: grid {
                    grid_sparse_index := get_grid_index(sparse, grid_index);
                    cell_x := cell_index % sparse.grid_size.x + grid_sparse_index.x * sparse.grid_size.x;
                    cell_y := (cell_index % (sparse.grid_size.x * sparse.grid_size.y)) / sparse.grid_size.x + grid_sparse_index.y * sparse.grid_size.y;
                    cell_z := cell_index / (sparse.grid_size.x * sparse.grid_size.y) + grid_sparse_index.z * sparse.grid_size.z;

                    `it := cell;
                    `it_index := SparseIndex.{cast(s32) cell_x, cast(s32) cell_y, cast(s32) cell_z};
                    #insert body;
                }
            } else {
                for cell, cell_index: grid {
                    grid_sparse_index := get_grid_index(sparse, grid_index);
                    cell_x := cell_index % sparse.grid_size.x + grid_sparse_index.x * sparse.grid_size.x;
                    cell_y := (cell_index % (sparse.grid_size.x * sparse.grid_size.y)) / sparse.grid_size.x + grid_sparse_index.y * sparse.grid_size.y;
                    cell_z := cell_index / (sparse.grid_size.x * sparse.grid_size.y) + grid_sparse_index.z * sparse.grid_size.z;

                    `it := cell;
                    `it_index := SparseIndex.{cast(s32) cell_x, cast(s32) cell_y, cast(s32) cell_z};
                    #insert body;
                }
            }
        }
    }
}

#if sparse_table {
    set_grid :: (sparse: *SparseGrid($T), index: SparseIndex, grid: [] T) {
        table_set(*sparse.grids, index, grid);
    }
} else {
    set_grid :: (sparse: *SparseGrid($T), index: SparseIndex, grid: [] T) {
        if sparse.grids.count == 0 {
            array_add(*sparse.grids, grid);
            sparse.grid_min = index;
            sparse.grid_max = index;
        } else {
            inside := index.x >= sparse.grid_min.x && index.x <= sparse.grid_min.x;
            inside = inside && index.y >= sparse.grid_min.y && index.y <= sparse.grid_min.y;
            inside = inside && index.z >= sparse.grid_min.z && index.z <= sparse.grid_min.z;
            if inside {
                grid_index := get_grid_array_index(sparse, index);
                assert(!sparse.grids[grid_index].data, "Cannot set grid where one already exists.\n");
                sparse.grids[grid_index] = grid;
            } else {
                grid_min := sparse.grid_min;
                grid_max := sparse.grid_max;
                grid_min.x = min(grid_min.x, index.x);
                grid_min.y = min(grid_min.y, index.y);
                grid_min.z = min(grid_min.z, index.z);
                grid_max.x = max(grid_max.x, index.x);
                grid_max.y = max(grid_max.y, index.y);
                grid_max.z = max(grid_max.z, index.z);

                resize_grid(sparse, grid_min, grid_max);

                grid_index := get_grid_array_index(sparse, index);
                assert(!sparse.grids[grid_index].data, "Cannot set grid where one already exists.\n");
                sparse.grids[grid_index] = grid;
            }
        }
    }

    resize_grid :: (sparse: *SparseGrid, grid_min: SparseIndex, grid_max: SparseIndex) {
        assert(grid_min.x <= sparse.grid_min.x && grid_min.y <= sparse.grid_min.y && grid_min.z <= sparse.grid_min.z, "Cannot decrease grid size yet.\n");
        assert(grid_max.x >= sparse.grid_max.x && grid_max.y >= sparse.grid_max.y && grid_max.z >= sparse.grid_max.z, "Cannot decrease grid size yet.\n");

        if sparse.grid_min == grid_min && sparse.grid_max == grid_max {
            return;
        }

        old_min := sparse.grid_min;
        old_max := sparse.grid_max;
        old_size := old_max - old_min + SparseIndex.{1, 1, 1};
        delta := old_min - grid_min;

        sparse.grid_min = grid_min;
        sparse.grid_max = grid_max;
        new_size := grid_max - grid_min + SparseIndex.{1, 1, 1};

        array_resize(*sparse.grids, new_size.x * new_size.y * new_size.z);

        // because we can only ever increase the size we just have to go in reverse order I guess

        for #v2 < z_offset: 0..old_size.z - 1 {
            z_offset_new := z_offset + delta.z;
            for #v2 < y_offset: 0..old_size.y - 1 {
                y_offset_new := y_offset + delta.y;
                for #v2 < x_offset: 0..old_size.x - 1 {
                    x_offset_new := x_offset + delta.x;

                    old_index := z_offset * old_size.x * old_size.y + y_offset * old_size.x + x_offset;
                    new_index := z_offset_new * new_size.x * new_size.y + y_offset_new * new_size.x + x_offset_new;

                    sparse.grids[old_index], sparse.grids[new_index] = sparse.grids[new_index], sparse.grids[old_index];
                }
            }
        }
    }
}

get_cell :: (sparse: *SparseGrid($T), point: Vector3) -> T, bool {
    cell_index := get_cell_index(sparse, point);
    data, found := get_cell(sparse, cell_index);
    return data, found;
}

#if cached_grid {
    get_cell :: (sparse: *SparseGrid($T), cell_index: SparseIndex) -> T, bool {
        grid_index, grid_cell_index := get_grid_and_cell_indices(sparse, cell_index);
        if sparse.cached_grid.grid_index != grid_index {
            grid, grid_found := get_grid(sparse, grid_index);
            if grid_found {
                sparse.cached_grid.grid_index = grid_index;
                sparse.cached_grid.grid = grid;
                return sparse.cached_grid.grid[grid_cell_index], true;
            } else {
                empty: T;
                return empty, false;
            }
        }
        return sparse.cached_grid.grid[grid_cell_index], true;
    }
} else {
    get_cell :: (sparse: *SparseGrid($T), cell_index: SparseIndex) -> T, bool {
        grid_index, grid_cell_index := get_grid_and_cell_indices(sparse, cell_index);
        grid, grid_found := get_grid(sparse, grid_index);
        if grid_found {
            return grid[grid_cell_index], true;
        }
        empty: T;
        return empty, false;
    }
}

get_cell_pointer :: (sparse: *SparseGrid($T), cell_index: SparseIndex) -> *T, bool {
    // TODO cache
    grid_index, grid_cell_index := get_grid_and_cell_indices(sparse, cell_index);
    grid, grid_found := table_find(*sparse.grids, grid_index);
    if grid_found {
        return *grid[grid_cell_index], true;
    }
    return null, false;
}

#if cached_grid {
    get_or_add_cell_pointer :: (sparse: *SparseGrid($T), cell_index: SparseIndex) -> *T {
        grid_index, grid_cell_index := get_grid_and_cell_indices(sparse, cell_index);
        if sparse.cached_grid.grid_index != grid_index {
            sparse.cached_grid.grid_index = grid_index;
            sparse.cached_grid.grid = get_or_add_grid(sparse, grid_index);
        }

        return *sparse.cached_grid.grid[grid_cell_index];
    }
} else {
    get_or_add_cell_pointer :: (sparse: *SparseGrid($T), cell_index: SparseIndex) -> *T {
        grid_index, grid_cell_index := get_grid_and_cell_indices(sparse, cell_index);
        grid := get_or_add_grid(sparse, grid_index);
        return *grid[grid_cell_index];
    }
}

set_cell :: (sparse: *SparseGrid($T), point: Vector3, data: T) {
    cell_index := get_cell_index(sparse, point);
    set_cell(sparse, cell_index, data);
}

#if cached_grid {
    set_cell :: (sparse: *SparseGrid($T), index: SparseIndex, data: T) {
        grid_index, grid_cell_index := get_grid_and_cell_indices(sparse, index);

        if sparse.cached_grid.grid_index != grid_index {
            sparse.cached_grid.grid_index = grid_index;
            sparse.cached_grid.grid = get_or_add_grid(sparse, grid_index);
        }

        sparse.cached_grid.grid[grid_cell_index] = data;
    }
} else {
    set_cell :: (sparse: *SparseGrid($T), index: SparseIndex, data: T) {
        grid_index, grid_cell_index := get_grid_and_cell_indices(sparse, index);
        grid := get_or_add_grid(sparse, grid_index);
        grid[grid_cell_index] = data;
    }
}

get_cell_boundaries :: (sparse: *SparseGrid) -> [2] SparseIndex {
    aabb: [2] SparseIndex;
    aabb_set := false;

    for _, grid_index: sparse {
        cell_start := grid_index;
        cell_start.x *= cast(s32) sparse.grid_size.x;
        cell_start.y *= cast(s32) sparse.grid_size.y;
        cell_start.z *= cast(s32) sparse.grid_size.z;

        cell_end := cell_start;
        cell_end.x += cast(s32) sparse.grid_size.x - 1;
        cell_end.y += cast(s32) sparse.grid_size.y - 1;
        cell_end.z += cast(s32) sparse.grid_size.z - 1;

        if !aabb_set {
            aabb_set = true;
            aabb = .[cell_start, cell_end];
        } else {
            aabb[0].x = fastmin(aabb[0].x, cell_start.x);
            aabb[0].y = fastmin(aabb[0].y, cell_start.y);
            aabb[0].z = fastmin(aabb[0].z, cell_start.z);
            aabb[1].x = fastmax(aabb[1].x, cell_start.x);
            aabb[1].y = fastmax(aabb[1].y, cell_start.y);
            aabb[1].z = fastmax(aabb[1].z, cell_start.z);
        }
    }

    return aabb;
}

get_aabb :: (sparse: SparseGrid) -> AABB3 {
    aabb: AABB3;
    aabb_set := false;

    for _, grid_index: sparse {
        grid_aabb := get_grid_aabb(sparse, grid_index);

        if !aabb_set {
            aabb_set = true;
            aabb = grid_aabb;
        } else {
            aabb.lower = fastmin(aabb.lower, grid_aabb.lower);
            aabb.upper = fastmax(aabb.upper, grid_aabb.upper);
        }
    }

    return aabb;
}

GridIterator :: struct (T: Type) {
    sparse: *SparseGrid(T);
    line: Line3;
}

get_grids_from_line :: (sparse: *SparseGrid($T), line: Line3) -> GridIterator(T) {
    return GridIterator(T).{sparse, line};
}

for_expansion :: (iterator: GridIterator($IT), body: Code, flags: For_Flags) #expand {
    #assert(!(flags & .REVERSE));
    #assert(!(flags & .POINTER));

    // TODO this algorithm is shitty... I need to optimize it to make it not shitty

    ray := Ray3.{iterator.line.p1, iterator.line.p2 - iterator.line.p1};

    grid_start_index := get_grid_index(iterator.sparse, ray.p);

    current_point := iterator.line.p1;
    current_grid := grid_start_index;
    while distance_squared(current_point, iterator.line.p1) < distance_squared(iterator.line.p1, iterator.line.p2) {
        grid, found := get_grid(iterator.sparse, current_grid);

        defer {
            grid_aabb := get_grid_aabb(iterator.sparse, current_grid);
            point, dir_x, dir_y, dir_z := ray_aabb_direction(ray, grid_aabb);
            if dir_x == 0 && dir_y == 0 && dir_z == 0 {
                break;
            }

            current_point = point;
            current_grid.x += dir_x;
            current_grid.y += dir_y;
            current_grid.z += dir_z;
        };

        `it := grid;
        `it_index := current_grid;

        #insert body;
    }
}

CellIterator :: struct (T: Type) {
    sparse: *SparseGrid(T);
    line: Line3;
}

get_cells_from_line :: (sparse: *SparseGrid($T), line: Line3) -> CellIterator(T) {
    return CellIterator(T).{sparse, line};
}

for_expansion :: (iterator: CellIterator($IT), body: Code, flags: For_Flags) #expand {
    #assert(!(flags & .REVERSE));
    #assert(!(flags & .POINTER));

    // TODO this algorithm is shitty... I need to optimize it to make it not shitty

    ray := Ray3.{iterator.line.p1, iterator.line.p2 - iterator.line.p1};

    for grid, grid_index: get_grids_from_line(iterator.sparse, iterator.line) {
        grid_aabb := get_grid_aabb(iterator.sparse, grid_index);

        grid_cell_start_x := cast(s32) (grid_index.x * iterator.sparse.grid_size.x);
        grid_cell_start_y := cast(s32) (grid_index.y * iterator.sparse.grid_size.y);
        grid_cell_start_z := cast(s32) (grid_index.z * iterator.sparse.grid_size.z);

        grid_cell_end_x := cast(s32) (grid_cell_start_x + iterator.sparse.grid_size.x - 1);
        grid_cell_end_y := cast(s32) (grid_cell_start_y + iterator.sparse.grid_size.y - 1);
        grid_cell_end_z := cast(s32) (grid_cell_start_z + iterator.sparse.grid_size.z - 1);

        start_position := ray.p;
        if !contains(grid_aabb, start_position) {
            start_position = ray_aabb_front_intersection(ray, grid_aabb);
        }
        cell_start_index := get_cell_index(iterator.sparse, start_position);
        cell_start_index.x = clamp(cell_start_index.x, grid_cell_start_x, grid_cell_end_x);
        cell_start_index.y = clamp(cell_start_index.y, grid_cell_start_y, grid_cell_end_y);
        cell_start_index.z = clamp(cell_start_index.z, grid_cell_start_z, grid_cell_end_z);

        did_break: bool;

        current_point := start_position;
        current_cell := cell_start_index;
        while distance_squared(current_point, iterator.line.p1) < distance_squared(iterator.line.p1, iterator.line.p2) {
            cell, found := get_cell(iterator.sparse, current_cell);

            defer {
                cell_aabb := get_cell_aabb(iterator.sparse, current_cell);
                point, dir_x, dir_y, dir_z := ray_aabb_direction(ray, cell_aabb);
                if dir_x == 0 && dir_y == 0 && dir_z == 0 {
                    break;
                }

                current_point = point;
                current_cell.x += dir_x;
                current_cell.y += dir_y;
                current_cell.z += dir_z;
            };

            `it := cell;
            `it_index := current_cell;

            did_break = true;
            #insert body;
            did_break = false;
        }

        if did_break {
            break;
        }
    }
}

sparse_grid_reset :: (sparse: *SparseGrid($T)) {
    ini :: initializer_of(T);

    for grid, index: sparse.grids {
        #if ini {
            for *entry: grid {
                ini(entry);
            }
        } else {
            memset(grid.data, 0, grid.count * size_of(T));
        }
    }
}

get_cell_index :: (sparse: SparseGrid, point: Vector3) -> SparseIndex {
    cell_x := cast(s32) floor(point.x / sparse.cell_size);
    cell_y := cast(s32) floor(point.y / sparse.cell_size);
    cell_z := cast(s32) floor(point.z / sparse.cell_size);

    return SparseIndex.{cell_x, cell_y, cell_z};
}

get_cell_index :: (sparse: SparseGrid, grid_index: SparseIndex, cell_index: int) -> SparseIndex {
    cell_x := cell_index % sparse.grid_size.x + grid_index.x * sparse.grid_size.x;
    cell_y := (cell_index % (sparse.grid_size.x * sparse.grid_size.y)) / sparse.grid_size.x + grid_index.y * sparse.grid_size.y;
    cell_z := cell_index / (sparse.grid_size.x * sparse.grid_size.y) + grid_index.z * sparse.grid_size.z;
    return .{cast(s32) cell_x, cast(s32) cell_y, cast(s32) cell_z};
}

get_grid_array_index :: (sparse: SparseGrid, grid_index: SparseIndex) -> int, bool {
    outside := grid_index.x < sparse.grid_min.x || grid_index.y < sparse.grid_min.y || grid_index.z < sparse.grid_min.z;
    outside = outside || grid_index.x > sparse.grid_max.x || grid_index.y > sparse.grid_max.y || grid_index.z > sparse.grid_max.z;
    if outside {
        return 0, false;
    }

    delta := grid_index - sparse.grid_min;
    size := sparse.grid_max - sparse.grid_min + SparseIndex.{1, 1, 1};

    return delta.z * size.x * size.y + delta.y * size.x + delta.x, true;
}

get_grid_index :: (sparse: SparseGrid, point: Vector3) -> SparseIndex {
    cell_index := get_cell_index(sparse, point);
    return get_grid_index(sparse, cell_index);
}

get_grid_index :: (sparse: SparseGrid, grid_index: int) -> SparseIndex {
    size := sparse.grid_max - sparse.grid_min + SparseIndex.{1, 1, 1};
    x := grid_index % size.x;
    y := (grid_index % (size.x * size.y)) / size.x;
    z := grid_index / (size.x * size.y);
    return .{cast(s32) x, cast(s32) y, cast(s32) z} + sparse.grid_min;
}

get_grid_index :: (sparse: SparseGrid, cell_index: SparseIndex) -> SparseIndex {
    sub_x := cast(s32) (((cast,no_check(u32) cell_index.x) & 0x80000000) >> 31);
    sub_y := cast(s32) (((cast,no_check(u32) cell_index.y) & 0x80000000) >> 31);
    sub_z := cast(s32) (((cast,no_check(u32) cell_index.z) & 0x80000000) >> 31);
    sub_x = sub_x * sparse.grid_size.x - sub_x;
    sub_y = sub_y * sparse.grid_size.y - sub_y;
    sub_z = sub_z * sparse.grid_size.z - sub_z;

    grid_x := (cell_index.x - sub_x) / sparse.grid_size.x;
    grid_y := (cell_index.y - sub_y) / sparse.grid_size.y;
    grid_z := (cell_index.z - sub_z) / sparse.grid_size.z;

    return SparseIndex.{grid_x, grid_y, grid_z};
}

get_grid_and_cell_indices :: (sparse: SparseGrid, cell_index: SparseIndex) -> SparseIndex, int {
    grid_index := get_grid_index(sparse, cell_index);

    cell_offset_x := cell_index.x - grid_index.x * sparse.grid_size.x;
    cell_offset_y := cell_index.y - grid_index.y * sparse.grid_size.y;
    cell_offset_z := cell_index.z - grid_index.z * sparse.grid_size.z;

    return grid_index, cell_offset_z * (sparse.grid_size.x * sparse.grid_size.y) + cell_offset_y * sparse.grid_size.x + cell_offset_x;
}

get_grid_aabb :: (sparse: SparseGrid, grid_index: SparseIndex) -> AABB3 {
    grid_dimensions := Vector3.{sparse.cell_size, sparse.cell_size, sparse.cell_size};
    grid_dimensions *= .{cast(float) sparse.grid_size.x, cast(float) sparse.grid_size.y, cast(float) sparse.grid_size.z};

    aabb: AABB3;
    aabb.lower = grid_dimensions * .{cast(float) grid_index.x, cast(float) grid_index.y, cast(float) grid_index.z};
    aabb.upper = aabb.lower + grid_dimensions;

    return aabb;
}

get_cell_aabb :: (sparse: SparseGrid($T), cell_index: SparseIndex) -> AABB3 {
    return get_cell_aabb(sparse, cell_index.x, cell_index.y, cell_index.z);
}

get_cell_aabb :: (sparse: SparseGrid($T), cell_x: int, cell_y: int, cell_z: int) -> AABB3 {
    lower := Vector3.{
        cell_x * sparse.cell_size,
        cell_y * sparse.cell_size,
        cell_z * sparse.cell_size,
    };
    upper := lower + .{sparse.cell_size, sparse.cell_size, sparse.cell_size};
    return .{lower, upper};
}

get_cell_position :: (sparse: SparseGrid($T), cell_index: SparseIndex) -> Vector3 {
    return .{
        cell_index.x * sparse.cell_size + sparse.cell_size / 2.0,
        cell_index.y * sparse.cell_size + sparse.cell_size / 2.0,
        cell_index.z * sparse.cell_size + sparse.cell_size / 2.0,
    };
}

#if sparse_table {
    get_grid :: (sparse: *SparseGrid($T), index: SparseIndex) -> [] T, bool {
        grid, found := table_find(*sparse.grids, index);
        return grid, found;
    }
} else {
    get_grid :: (sparse: *SparseGrid($T), index: SparseIndex) -> [] T, bool {
        grid_array_index, index_valid := get_grid_array_index(sparse, index);
        if !index_valid || sparse.grids[grid_array_index].count == 0 {
            return .[], false;
        }

        return sparse.grids[grid_array_index], true;
    }
}

#if sparse_table {
    get_or_add_grid :: (sparse: *SparseGrid($T), index: SparseIndex) -> [] T {
        grid, added := find_or_add(*sparse.grids, index,, sparse.allocator);
        if added {
            grid_count := sparse.grid_size.x * sparse.grid_size.y * sparse.grid_size.z;
            grid.* = NewArray(grid_count, T,, sparse.allocator);
        }
        return grid.*;
    }
} else {
    get_or_add_grid :: (sparse: *SparseGrid($T), index: SparseIndex) -> [] T {
        grid, exists := get_grid(sparse, index);
        if exists {
            return grid;
        }

        grid_count := sparse.grid_size.x * sparse.grid_size.y * sparse.grid_size.z;
        grid = NewArray(grid_count, T,, sparse.allocator);

        set_grid(sparse, index, grid);
        return grid;
    }
}

#scope_module

plane_intersection :: (ray: Ray3, plane: Plane3) -> float {
    return (plane.d - dot(plane.normal, ray.p)) / dot(plane.normal, ray.dir);
}

ray_aabb_front_intersection :: (ray: Ray3, aabb: AABB3) -> Vector3 {
    max_t := 0.0;

    planes: [6] Plane3 = ---;
    planes[0] = create_plane3(aabb.upper, .{1, 0, 0});
    planes[1] = create_plane3(aabb.upper, .{0, 1, 0});
    planes[2] = create_plane3(aabb.upper, .{0, 0, 1});
    planes[3] = create_plane3(aabb.lower, .{-1, 0, 0});
    planes[4] = create_plane3(aabb.lower, .{0, -1, 0});
    planes[5] = create_plane3(aabb.lower, .{0, 0, -1});

    for plane: planes {
        if dot(ray.dir, plane.normal) < 0 {
            ray_t := plane_intersection(ray, plane);
            if ray_t > max_t {
                max_t = ray_t;
            }
        }
    }

    max_t = min(max_t, 1);

    return ray.p + ray.dir * max_t;
}

ray_aabb_back_intersection :: (ray: Ray3, aabb: AABB3, max_t: float = FLOAT32_MAX) -> Vector3 {
    min_t := max_t;

    planes: [6] Plane3 = ---;
    planes[0] = create_plane3(aabb.upper, .{1, 0, 0});
    planes[1] = create_plane3(aabb.upper, .{0, 1, 0});
    planes[2] = create_plane3(aabb.upper, .{0, 0, 1});
    planes[3] = create_plane3(aabb.lower, .{-1, 0, 0});
    planes[4] = create_plane3(aabb.lower, .{0, -1, 0});
    planes[5] = create_plane3(aabb.lower, .{0, 0, -1});

    for plane: planes {
        if dot(ray.dir, plane.normal) > 0 {
            ray_t := plane_intersection(ray, plane);
            if ray_t > 0 && ray_t < min_t {
                min_t = ray_t;
            }
        }
    }

    return ray.p + ray.dir * min_t;
}

ray_aabb_direction :: (ray: Ray3, aabb: AABB3) -> Vector3, s8, s8, s8 {
    min_t := 1.0;
    min_normal: Vector3;

    planes: [6] Plane3 = ---;
    planes[0] = create_plane3(aabb.upper, .{1, 0, 0});
    planes[1] = create_plane3(aabb.upper, .{0, 1, 0});
    planes[2] = create_plane3(aabb.upper, .{0, 0, 1});
    planes[3] = create_plane3(aabb.lower, .{-1, 0, 0});
    planes[4] = create_plane3(aabb.lower, .{0, -1, 0});
    planes[5] = create_plane3(aabb.lower, .{0, 0, -1});

    for plane: planes {
        if dot(ray.dir, plane.normal) > 0 {
            ray_t := plane_intersection(ray, plane);
            if ray_t > 0 && ray_t < min_t {
                min_t = ray_t;
                min_normal = plane.normal;
            }
        }
    }

    dir_x := cast(s8) sign(min_normal.x);
    dir_y := cast(s8) sign(min_normal.y);
    dir_z := cast(s8) sign(min_normal.z);

    return ray.p + ray.dir * min_t, dir_x, dir_y, dir_z;
}

print_binary :: (v: $T) {
    bits :: size_of(T) * 8;
    for i: 0..bits - 1 {
        bit_shift := (bits - 1) - i;
        if v & (1 << bit_shift) {
            print("1");
        } else {
            print("0");
        }
    }
    print("\n");
}

#import "Basic";
#import "Math";
#import "Hash_Table";
#import "Hash";

#import "JaiMath";
#import "JaiBoundingTree";
