
JumpData :: struct (JumpDataType: Type, DefaultValue: JumpDataType) {
    type: JumpDataType;
    distances: [offsets.count] s8;
    forced_neighbors: [3] [3] ForcedOffsets;
}

WeightData :: struct (WeightDataType: Type) {
    weights: [offsets.count] WeightDataType;
}

GoalBounds :: struct {
    bounds: [26] SparseAABB;
}

// JumpEntry :: struct (T: Type) {
//     data: T;
//     distances: [offsets.count] s8;
//     forced_neighbors: [3] [3] ForcedOffsets;
// }

// this is implying that 1 maps to index 0
ForcedOffsets :: enum_flags u32 {
    None :: 0;
    PX :: 1;
    NX;
    PY; NY;
    PZ; NZ;
    PXPZ; PXNZ; NXPZ; NXNZ;
    PXPY; PXNY; NXPY; NXNY;
    PYPZ; PYNZ; NYPZ; NYNZ;
    PXPYPZ; PXPYNZ; PXNYPZ; PXNYNZ;
    NXPYPZ; NXPYNZ; NXNYPZ; NXNYNZ;
}

offsets :: SparseDir.[
    .{1, 0, 0}/*0*/, .{-1, 0, 0}/*1*/,
    .{0, 1, 0}/*2*/, .{0, -1, 0}/*3*/,
    .{0, 0, 1}/*4*/, .{0, 0, -1}/*5*/,
    .{1, 0, 1}/*6*/, .{1, 0, -1}/*7*/, .{-1, 0, 1}/*8*/, .{-1, 0, -1}/*9*/,
    .{1, 1, 0}/*10*/, .{1, -1, 0}/*11*/, .{-1, 1, 0}/*12*/, .{-1, -1, 0}/*13*/,
    .{0, 1, 1}/*14*/, .{0, 1, -1}/*15*/, .{0, -1, 1}/*16*/, .{0, -1, -1}/*17*/,
    .{1, 1, 1}/*18*/, .{1, 1, -1}/*19*/, .{1, -1, 1}/*20*/, .{1, -1, -1}/*21*/,
    .{-1, 1, 1}/*22*/, .{-1, 1, -1}/*23*/, .{-1, -1, 1}/*24*/, .{-1, -1, -1}/*25*/,
];

offset_index_lookup :: [3] [3] u8.[
    .[
        .[25, 13, 24],
        .[9, 1, 8],
        .[23, 12, 22],
    ], .[
        .[17, 3, 16],
        .[5, 255, 4],
        .[15, 2, 14],
    ], .[
        .[21, 11, 20],
        .[7, 0, 6],
        .[19, 10, 18],
    ],
];

travel_offsets :: [3] [3] ForcedOffsets.[
    .[
        .[.NX | .NY | .NZ | .NXNY | .NXNZ | .NYNZ | .NXNYNZ, .NX | .NY | .NXNY, .NX | .NY | .PZ | .NXNY | .NXPZ | .NYPZ | .NXNYPZ],
        .[.NX | .NZ | .NXNZ, .NX, .NX | .PZ | .NXPZ],
        .[.NX | .PY | .NZ | .NXPY | .NXNZ | .PYNZ | .NXPYNZ, .NX | .PY | .NXPY, .NX | .PY | .PZ | .NXPY | .NXPZ | .PYPZ | .NXPYPZ],
    ], .[
        .[.NY | .NZ | .NYNZ, .NY, .NY | .PZ | .NYPZ],
        .[.NZ, .None, .PZ],
        .[.PY | .NZ | .PYNZ, .PY, .PY | .PZ | .PYPZ],
    ], .[
        .[.PX | .NY | .NZ | .PXNY | .PXNZ | .NYNZ | .PXNYNZ, .PX | .NY | .PXNY, .PX | .NY | .PZ | .PXNY | .PXPZ | .NYPZ | .PXNYPZ],
        .[.PX | .NZ | .PXNZ, .PX, .PX | .PZ | .PXPZ],
        .[.PX | .PY | .NZ | .PXPY | .PXNZ | .PYNZ | .PXPYNZ, .PX | .PY | .PXPY, .PX | .PY | .PZ | .PXPY | .PXPZ | .PYPZ | .PXPYPZ],
    ],
];

get_cell_type :: (jump_data: JumpData($T, $R)) -> Type {
    return T;
}

// get_cell_type :: (jump_data: *JumpData($T, $R)) -> T {
//     return T;
// }

// get_cell_type :: (sparse: SparseGrid($A)) -> Type {

//     return #run () -> Type {
//         empty: A;
//         return get_cell_type(empty);
//     }();
// }

get_default_value :: (sparse: SparseGrid($A), $T: Type) -> T {
    empty: A;
    return get_default_value(empty.jump_data);
}

get_default_value :: (jump_data: JumpData($T, $R)) -> T {
    return R;
}

// im not sure of a better way to arbitrarily get access to these struct types
// get_weight :: (type: $T, weights: WeightData(T, $F)) -> u8 {
//     return F(type);
// }

// a jump sparse grid is a sparse grid that contains connection information for visually continuous sections of mesh
// this, I think, will heavily optimize the path finding algorithm

// TODO
// write the actual path finding algorithm and make it visual in the game
// then go through and generate the sparse groups every time you add a new cell

// create_grouped_sparse_grid :: ($T: Type, grid_size_x: int, grid_size_y: int, grid_size_z: int, cell_size: float) -> JumpSparseGrid(T) {
//     jump: JumpSparseGrid(T);
//     jump.grid = create_sparse_grid(JumpEntry(T), grid_size_x, grid_size_y, grid_size_z, cell_size);

//     return jump;
// }

// for_expansion :: (sparse: SparseGrid($T), body: Code, flags: For_Flags) #expand {
//     #assert(!(flags & .REVERSE));

//     #if flags & .POINTER {
//         for *cell, cell_index: sparse.grid {
//             `it := cell.data;
//             `it_index := cell_index;
//             #insert body;
//         }
//     } else {
//         for cell, cell_index: sparse.grid {
//             `it := cell.data;
//             `it_index := cell_index;
//             #insert body;
//         }
//     }
// }

// get_cell :: (sparse: *SparseGrid($T), point: Vector3) -> T, bool {
//     entry, found := get_cell(sparse, point);
//     return entry.data, found;
// }

// *CachedGrid(JumpEntry(T)) doesn't work... try again in a future version
// get_cell :: (sparse: *SparseGrid($T), cell_index: SparseIndex, cached_grid: *CachedGrid(JumpEntry(T))) -> T, bool {
// get_cell :: (sparse: *SparseGrid($T), cell_index: SparseIndex, cached_grid: *$D) -> T, bool {
//     entry, found := get_cell(sparse, cell_index, cached_grid);
//     return entry.data, found;
// }

// get_cell :: (sparse: *SparseGrid($T), cell_index: SparseIndex) -> T, bool {
//     entry, found := get_cell(sparse, cell_index);
//     return entry.data, found;
// }

is_jump_data :: (type: Type, other_type: Type) -> bool {
    if (cast(*Type_Info) other_type).type != .ENUM return false;
    info := cast(*Type_Info) type;
    if info.type != .STRUCT return false;
    struct_info := cast(*Type_Info_Struct) info;
    for member: struct_info.members {
        if member.type.type != .STRUCT continue;
        member_struct_info := cast(*Type_Info_Struct) member.type;
        if member_struct_info.name == "JumpData" {
            return true;
        }
    }
    return false;
}

// set_cell :: (sparse: *SparseGrid($EXTREMELY_DUMB_BUG), point: Vector3, type: $T) #modify {return is_jump_data(EXTREMELY_DUMB_BUG, T);} {
//     entry: EXTREMELY_DUMB_BUG;
//     entry.type = type;
//     set_cell(sparse, point, entry);
// }

set_cell :: (sparse: *SparseGrid($EXTREMELY_DUMB_BUG), cell_index: SparseIndex, type: $T) #modify {return is_jump_data(EXTREMELY_DUMB_BUG, T);} {
    entry := get_or_add_cell_pointer(sparse, cell_index);
    entry.type = type;
}

#scope_file


