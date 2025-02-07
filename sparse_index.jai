
SparseIndex :: struct {
    x: s32;
    y: s32;
    z: s32;
}

SparseAABB :: struct {
    lower: SparseBounds;
    upper: SparseBounds;
}

SparseBounds :: struct {
    x: s16;
    y: s16;
    z: s16;
}

SparseDir :: struct {
    x: s8;
    y: s8;
    z: s8;
}

operator == :: (a: SparseIndex, b: SparseIndex) -> bool {
    return a.x == b.x && a.y == b.y && a.z == b.z;
}

operator + :: (a: SparseIndex, b: SparseDir) -> SparseIndex #symmetric {
    r: SparseIndex = ---;
    r.x = a.x + b.x;
    r.y = a.y + b.y;
    r.z = a.z + b.z;
    return r;
}

operator + :: (a: SparseIndex, b: SparseIndex) -> SparseIndex #symmetric {
    r: SparseIndex = ---;
    r.x = a.x + b.x;
    r.y = a.y + b.y;
    r.z = a.z + b.z;
    return r;
}

operator - :: (a: SparseIndex, b: SparseIndex) -> SparseIndex {
    r: SparseIndex = ---;
    r.x = a.x - b.x;
    r.y = a.y - b.y;
    r.z = a.z - b.z;
    return r;
}

operator - :: (a: SparseIndex, b: SparseDir) -> SparseIndex {
    r: SparseIndex = ---;
    r.x = a.x - b.x;
    r.y = a.y - b.y;
    r.z = a.z - b.z;
    return r;
}

operator * :: (a: SparseIndex, b: s32) -> SparseIndex #symmetric {
    r: SparseIndex = ---;
    r.x = a.x * b;
    r.y = a.y * b;
    r.z = a.z * b;
    return r;
}

operator * :: (a: SparseIndex, b: SparseDir) -> SparseIndex #symmetric {
    r: SparseIndex = ---;
    r.x = a.x * b.x;
    r.y = a.y * b.y;
    r.z = a.z * b.z;
    return r;
}

// operator * :: (a: SparseIndex, b: SparseDir) -> SparseIndex #symmetric {
//     r: SparseIndex = ---;
//     r.x = a.x * b.x;
//     r.y = a.y * b.y;
//     r.z = a.z * b.z;
//     return r;
// }

operator == :: (a: SparseDir, b: SparseDir) -> bool {
    return a.x == b.x && a.y == b.y && a.z == b.z;
}

operator + :: (a: SparseDir, b: SparseDir) -> SparseDir #symmetric {
    r: SparseDir = ---;
    r.x = a.x + b.x;
    r.y = a.y + b.y;
    r.z = a.z + b.z;
    return r;
}

// operator + :: (a: SparseDir, b: SparseDir) -> SparseDir #symmetric {
//     r: SparseDir = ---;
//     r.x = a.x + b.x;
//     r.y = a.y + b.y;
//     r.z = a.z + b.z;
//     return r;
// }

operator - :: (a: SparseDir, b: SparseDir) -> SparseDir {
    r: SparseDir = ---;
    r.x = a.x - b.x;
    r.y = a.y - b.y;
    r.z = a.z - b.z;
    return r;
}

operator * :: (a: SparseDir, b: s8) -> SparseDir #symmetric {
    r: SparseDir = ---;
    r.x = a.x * b;
    r.y = a.y * b;
    r.z = a.z * b;
    return r;
}

operator * :: (a: SparseDir, b: s32) -> SparseIndex #symmetric {
    r: SparseIndex = ---;
    r.x = a.x * b;
    r.y = a.y * b;
    r.z = a.z * b;
    return r;
}

operator / :: (a: SparseDir, b: float) -> Vector3 {
    r: Vector3 = ---;
    r.x = a.x / b;
    r.y = a.y / b;
    r.z = a.z / b;
    return r;
}

sparse_hash :: inline (a: SparseIndex) -> u32 {
    // find_or_add_time 29.105425 ms
    // str: string = ---;
    // str.data = cast(*u8) *a;
    // str.count = size_of(SparseIndex);
    // return djb2_hash(str);

    // find_or_add_time 30.454636 ms
    // return sdbm_hash(*a, size_of(SparseIndex));

    // find_or_add_time 27.31967 ms
    hash: u32 = 2166136261; // fnv1a hash value
    hash ^= cast,no_check(u32) a.x;
    hash *= 16777619; // fnv1a prime
    hash ^= cast,no_check(u32) a.y;
    hash *= 16777619;
    hash ^= cast,no_check(u32) a.z;
    hash *= 16777619;
    return hash;
}

sparse_equals :: inline (a: SparseIndex, b: SparseIndex) -> bool {
    return a == b;
}

contains :: (aabb: SparseAABB, point: SparseIndex) -> bool {
    inside := point.x >= aabb.lower.x;
    inside = inside && point.y >= aabb.lower.y;
    inside = inside && point.z >= aabb.lower.z;
    inside = inside && point.x <= aabb.lower.x;
    inside = inside && point.y <= aabb.lower.y;
    inside = inside && point.z <= aabb.lower.z;
    return inside;
}

// #run {
//     a1 := SparseIndex.{-2, 2, 3};
//     print("%: %\n", a1, unpack(pack(a1)));
// };

// pack :: (index: SparseIndex) -> u64 {
//     index_x := cast(u64) (cast,no_check(u16) (cast(s16) index.x));
//     index_y := cast(u64) (cast,no_check(u16) (cast(s16) index.y));
//     index_z := cast(u64) (cast,no_check(u16) (cast(s16) index.z));

//     result: u64;
//     result |= index_x << 32;
//     result |= index_y << 16;
//     result |= index_z;

//     return result;
// }

// unpack :: (index: u64) -> SparseIndex {
//     index_x := (index >> 32) & 0xffff;
//     index_y := (index >> 16) & 0xffff;
//     index_z := index & 0xffff;

//     result: SparseIndex;
//     result.x = cast,no_check(s16) index_x;
//     result.y = cast,no_check(s16) index_y;
//     result.z = cast,no_check(s16) index_z;
//     return result;
// }
