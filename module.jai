
#load "binary_heap.jai";
#load "grid_generator.jai";
#load "jump_sparse_grid.jai";
#load "nav_astar.jai";
#load "nav_jps.jai";
#load "sparse_grid.jai";
#load "sparse_index.jai";

#scope_module

// #run {
//     print("%: %\n", -500, fastabs(-500));
//     print("fastmax(-500, -200) %\n", fastmax(-500, -200));
//     print("fastmax(500, -200) %\n", fastmax(500, -200));
//     print("fastmax(500, 200) %\n", fastmax(500, 200));
//     print("fastmax(500, 500) %\n", fastmax(500, 500));
// }

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
