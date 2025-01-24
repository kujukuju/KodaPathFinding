
SparseIndex :: struct {
    x: s32;
    y: s32;
    z: s32;
}

operator == :: (a: SparseIndex, b: SparseIndex) -> bool {
    return a.x == b.x && a.y == b.y && a.z == b.z;
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

pack :: (index: SparseIndex) -> u64 {
    index_x := cast,no_check(u32) index.x;
    index_y := cast,no_check(u32) index.y;
    index_z := cast,no_check(u32) index.z;

    result: u64;
    result |= (cast(u64) (index_x & 0b10000000_00000000_00000000_00000000)) << 31;
    result |= (cast(u64) (index_y & 0b10000000_00000000_00000000_00000000)) << 31;
}

unpack :: (index: u64) -> SparseIndex {

}
