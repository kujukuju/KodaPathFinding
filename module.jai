
#load "binary_heap.jai";
#load "grid_generator.jai";
#load "jump_sparse_grid.jai";
#load "nav_astar.jai";
#load "nav_jps.jai";
#load "nav_setup.jai";
#load "path_smoothing.jai";
#load "path_visualizer.jai";
#load "sparse_grid.jai";
#load "sparse_index.jai";

#scope_module

is_type :: (a: $T, b: T) -> bool {
    return (a & b) || a == b;
}

get_using_type :: (parent: Type, child_name: string) -> Type {
    parent_info := cast(*Type_Info_Struct) parent;
    if parent_info.type != .STRUCT {
        return void;
    }

    for member: parent_info.members {
        member_struct := cast(*Type_Info_Struct) member.type;

        is_child := !!(member.flags & .USING);
        is_child = is_child && member_struct.type == .STRUCT && member_struct.name == child_name;
        if is_child {
            return (cast(*Type) *member_struct.type).*;
        }
    }

    return void;
}

is_using_runtime :: (parent: Type, child_name: string) -> bool {
    // this should bake the result into every uniquely polymorphic procedure at compile time but allow it to be called at runtime as far as I can figure
    // which should be better for the compiler
    parent_info := cast(*Type_Info_Struct) parent;
    if parent_info.type != .STRUCT {
        return false;
    }

    for member: parent_info.members {
        member_struct := cast(*Type_Info_Struct) member.type;

        is_child := !!(member.flags & .USING);
        is_child = is_child && member_struct.type == .STRUCT && member_struct.name == child_name;
        if is_child {
            return true;
        }
    }

    return false;
}

is_using :: ($parent: Type, $child_name: string) -> bool {
    // this should bake the result into every uniquely polymorphic procedure at compile time but allow it to be called at runtime as far as I can figure
    // which should be better for the compiler
    result :: #run () -> bool {
        return is_using_runtime(parent, child_name);
        // parent_info := cast(*Type_Info_Struct) parent;
        // if parent_info.type != .STRUCT {
        //     return false;
        // }

        // for member: parent_info.members {
        //     member_struct := cast(*Type_Info_Struct) member.type;

        //     is_child := !!(member.flags & .USING);
        //     is_child = is_child && member_struct.type == .STRUCT && member_struct.name == child_name;
        //     if is_child {
        //         return true;
        //     }
        // }

        // return false;
    }();
    return result;
}

is_using :: ($parent: Type, $child: Type) -> bool {
    // this should bake the result into every uniquely polymorphic procedure at compile time but allow it to be called at runtime as far as I can figure
    // which should be better for the compiler
    result :: #run () -> bool {
        parent_info := cast(*Type_Info_Struct) parent;
        child_info := cast(*Type_Info) child;
        if parent_info.type != .STRUCT {
            return false;
        }

        for member: parent_info.members {
            is_child := !!(member.flags & .USING);
            is_child = is_child && member.type == child_info;
            if is_child {
                return true;
            }
        }

        return false;
    }();
    return result;
}

DDAIterator :: struct {
    p1: SparseIndex;
    p2: SparseIndex;
}

// DDAIterator :: struct {
//     position: SparseIndex;
//     distance: SparseIndex;
// }

for_expansion :: (dda: DDAIterator, body: Code, flags: For_Flags) #expand {
    vec3 :: (point: SparseIndex) -> Vector3 {
        return .{cast(float) point.x, cast(float) point.y, cast(float) point.z};
    }

    distance := dda.p2 - dda.p1;

    steps: s32;
    if abs(distance.x) >= abs(distance.y) && abs(distance.x) >= abs(distance.z) {
        steps = abs(distance.x);
    } else if abs(distance.y) >= abs(distance.x) && abs(distance.y) >= abs(distance.z) {
        steps = abs(distance.y);
    } else if abs(distance.z) >= abs(distance.x) && abs(distance.z) >= abs(distance.y) {
        steps = abs(distance.z);
    }
    step_direction := vec3(distance) / (cast(float) steps);
    position := vec3(dda.p1);
    `it_index: SparseIndex;
    it_index.x = sign(distance.x);
    it_index.y = sign(distance.y);
    it_index.z = sign(distance.z);
    `it: SparseIndex;

    for i: 0..steps {
        it = SparseIndex.{cast(s32) round(position.x), cast(s32) round(position.y), cast(s32) round(position.z)};

        defer position += step_direction;

        #insert body;
    }
}

fastabs :: inline (v: $T) -> T {
    mask_shift :: size_of(T) * 8 - 1;
    mask := v >> mask_shift;
    return (v ^ mask) - mask;
}

fastmin :: inline (a: $T, b: T) -> T {
    return b ^ ((a ^ b) & -(cast,no_check(T) (a < b)));
}

fastmax :: inline (a: $T, b: T) -> T {
    return a ^ ((a ^ b) & -(cast,no_check(T) (a < b)));
}
