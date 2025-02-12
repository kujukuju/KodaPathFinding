
#load "binary_heap.jai";
#load "grid_generator.jai";
#load "jump_sparse_grid.jai";
#load "nav_astar.jai";
#load "nav_jps.jai";
#load "nav_setup.jai";
#load "sparse_grid.jai";
#load "sparse_index.jai";

#scope_module

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
