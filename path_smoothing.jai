
smooth_path :: (grid: *SparseGrid($T), path: [] SparseIndex, valid_types: $R) -> [..] SparseIndex {
    result: [..] SparseIndex;

    if path.count <= 2 {
        array_copy(*result, path);
        return result;
    }

    array_add(*result, path[path.count - 1]);

    current_anchor := path[path.count - 2];
    last_valid_anchor := path[path.count - 2];

    next_end_index := path.count - 3;

    while jump_loop := next_end_index >= 0 {
        next_end := path[next_end_index];

        // get the cell we're currently checking if it has visibility
        for path_cell: JumpLineIterator.{current_anchor, next_end} {
            if it_index == 0 {
                continue;
            }

            if is_valid_path(grid, result[result.count - 1], path_cell, valid_types) {
                last_valid_anchor = path_cell;
            } else {
                print("Invalid path: % %\n", result[result.count - 1], path_cell);
                array_add(*result, last_valid_anchor);

                current_anchor = last_valid_anchor;
                continue jump_loop;
            }
        }

        // if you manage to finish your current line iterator
        // we move current anchor to the end point and move on to the next line
        current_anchor = next_end;
        last_valid_anchor = next_end;
        next_end_index -= 1;
    }

    if current_anchor != path[0] {
        array_add(*result, path[0]);
    }

    reverse(result);

    return result;
}

is_valid_path :: (grid: *SparseGrid($T), p1: SparseIndex, p2: SparseIndex, valid_types: $R) -> bool {
    // TODO maybe inverting this optimizes it

    for path_cell: DDAIterator.{p1, p2} {
        path_cell_type := get_cell(grid, path_cell).type;

        if path_cell_type & valid_types {
            continue;
        }

        return false;
    }

    return true;
}

JumpLineIterator :: struct {
    p1: SparseIndex;
    p2: SparseIndex;
}

for_expansion :: (iterator: JumpLineIterator, body: Code, flags: For_Flags) #expand {
    #assert(!(flags & .REVERSE));

    current := iterator.p1;
    end := iterator.p2;
    index := 0;

    delta := iterator.p2 - iterator.p1;

    valid_jump_line := abs(delta.x) == abs(delta.y) && abs(delta.x) == abs(delta.z);
    valid_jump_line = valid_jump_line || (abs(delta.x) == abs(delta.y) && delta.z == 0);
    valid_jump_line = valid_jump_line || (abs(delta.x) == abs(delta.z) && delta.y == 0);
    valid_jump_line = valid_jump_line || (abs(delta.y) == abs(delta.z) && delta.x == 0);
    valid_jump_line = valid_jump_line || (delta.x != 0 && delta.y == 0 && delta.z == 0);
    valid_jump_line = valid_jump_line || (delta.x == 0 && delta.y != 0 && delta.z == 0);
    valid_jump_line = valid_jump_line || (delta.x == 0 && delta.y == 0 && delta.z != 0);
    assert(valid_jump_line, "Jump line is not valid. % % %\n", delta.x, delta.y, delta.z);

    direction := SparseIndex.{sign(cast(s32) delta.x), sign(cast(s32) delta.y), sign(cast(s32) delta.z)};

    while true {
        `it := current;
        `it_index := index;

        defer {
            if current == end {
                break;
            }

            current += direction;

            index += 1;
        };

        #insert body;
    }
}

LineIterator :: struct {
    p1: SparseIndex;
    p2: SparseIndex;
}

// im allowing this line iterator to move actually diagonally if that's the correct path to get there
for_expansion :: (iterator: LineIterator, body: Code, flags: For_Flags) #expand {
    #assert(!(flags & .REVERSE));

    index := 0;
    current := iterator.p1;
    end := iterator.p2;

    line := Line3.{
        Vector3.{cast(float) current.x, cast(float) current.y, cast(float) current.z},
        Vector3.{cast(float) end.x, cast(float) end.y, cast(float) end.z},
    };
    line_dir := normalize(line.p2 - line.p1);

    DIAGONAL_EPS :: 0.0001;

    intsign :: (value: float) -> s32 {
        return cast(s32) sign(value);
    }

    while true {
        `it := current;
        `it_index := index;

        defer {
            if current == end {
                break;
            }

            current_point := Vector3.{cast(float) current.x, cast(float) current.y, cast(float) current.z};
            line_point, line_t := nearest_point(line, current_point);

            if line_t >= 1 {
                current = end;
                continue;
            }

            line_point_distance := distance(current_point, line_point);
            // 1 probably isn't correct here it probably needs to be something to do with the 3d slope of the line
            remaining_distance := 1.0; // max(1 - line_point_distance, 0);

            line_point += remaining_distance * line_dir;
            delta := line_point - current_point;

            if abs(delta.x) > abs(delta.y) + DIAGONAL_EPS && abs(delta.x) > abs(delta.z) + DIAGONAL_EPS {
                current += SparseIndex.{intsign(delta.x), 0, 0};
            } else if abs(delta.y) > abs(delta.x) + DIAGONAL_EPS && abs(delta.y) > abs(delta.z) + DIAGONAL_EPS {
                current += SparseIndex.{0, intsign(delta.y), 0};
            } else if abs(delta.z) > abs(delta.x) + DIAGONAL_EPS && abs(delta.z) > abs(delta.y) + DIAGONAL_EPS {
                current += SparseIndex.{0, 0, intsign(delta.z)};
            } else if abs(delta.x) > abs(delta.y) + DIAGONAL_EPS {
                // deltax > deltay && deltax == deltaz
                current += SparseIndex.{intsign(delta.x), 0, intsign(delta.z)};
            } else if abs(delta.x) > abs(delta.z) + DIAGONAL_EPS {
                current += SparseIndex.{intsign(delta.x), intsign(delta.y), 0};
            } else if abs(delta.y) > abs(delta.x) + DIAGONAL_EPS {
                current += SparseIndex.{0, intsign(delta.y), intsign(delta.z)};
            } else {
                current += SparseIndex.{intsign(delta.x), intsign(delta.y), intsign(delta.z)};
            }

            index += 1;
        };

        #insert body;
    }
}
