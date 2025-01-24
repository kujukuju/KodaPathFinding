
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

// #run {
//     a1 := SparseIndex.{-2, 2, 3};
//     print("%: %\n", a1, unpack(pack(a1)));
// };

pack :: (index: SparseIndex) -> u64 {
    index_x := cast(u64) (cast,no_check(u16) (cast(s16) index.x));
    index_y := cast(u64) (cast,no_check(u16) (cast(s16) index.y));
    index_z := cast(u64) (cast,no_check(u16) (cast(s16) index.z));

    result: u64;
    result |= index_x << 32;
    result |= index_y << 16;
    result |= index_z;

    return result;
}

unpack :: (index: u64) -> SparseIndex {
    index_x := (index >> 32) & 0xffff;
    index_y := (index >> 16) & 0xffff;
    index_z := index & 0xffff;

    result: SparseIndex;
    result.x = cast,no_check(s16) index_x;
    result.y = cast,no_check(s16) index_y;
    result.z = cast,no_check(s16) index_z;
    return result;
}
