
BinaryHeap :: struct (T: Type, heuristic: (T) -> $R) {
    entries: [..] T;
    allocator: Allocator;
}

reserve :: (heap: *BinaryHeap($T, $heuristic), count: s32) {
    array_reserve(*heap.entries, count,, heap.allocator);
}

sift_up :: (heap: *BinaryHeap($T, $heuristic), index: s32) {
    while index > 0 && heuristic(heap.entries[PARENT(index)]) > heuristic(heap.entries[index]) {
        heap.entries[index], heap.entries[PARENT(index)] = heap.entries[PARENT(index)], heap.entries[index];
        index = PARENT(index);
    }
}

sift_down :: (heap: *BinaryHeap($T, $heuristic), index: s32) {
    while true {
        smallest := index;
        left := LEFT_CHILD(index);
        right := RIGHT_CHILD(index);

        if left < heap.entries.count && heuristic(heap.entries[left]) < heuristic(heap.entries[smallest]) {
            smallest = left;
        }
        if right < heap.entries.count && heuristic(heap.entries[right]) < heuristic(heap.entries[smallest]) {
            smallest = right;
        }

        if smallest == index {
            break;
        }

        heap.entries[index], heap.entries[smallest] = heap.entries[smallest], heap.entries[index];
        index = smallest;
    }
}

insert :: (heap: *BinaryHeap($T, $heuristic), value: T) {
    maybe_grow(*heap.entries,, heap.allocator);

    heap.entries.count += 1;
    heap.entries[heap.entries.count - 1] = value;
    sift_up(heap, cast(s32) heap.entries.count - 1);
}

pop_min :: (heap: *BinaryHeap($T, $heuristic)) -> T, bool {
    if heap.entries.count <= 0 {
        return .{}, false;
    }

    min := heap.entries[0];

    // Move last element to root and restore heap property
    heap.entries[0] = heap.entries[heap.entries.count - 1];
    heap.entries.count -= 1;

    sift_down(heap, 0);

    return min, true;
}

peek_min :: (heap: *BinaryHeap($T, $heuristic)) -> T, bool {
    if heap.entries.count <= 0 {
        return .{}, false;
    }

    return heap.entries[0];
}

is_empty :: (heap: BinaryHeap) -> bool {
    return heap.entries.count == 0;
}

destroy :: (heap: BinaryHeap) {
    array_free(heap.entries,, heap.allocator);
}

#scope_file

PARENT :: inline (i: s32) -> s32 {
    return (i - 1) / 2;
}

LEFT_CHILD :: inline (i: s32) -> s32 {
    return 2 * i + 1;
}

RIGHT_CHILD :: inline (i: s32) -> s32 {
    return 2 * i + 2;
}
