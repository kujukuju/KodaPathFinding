
funnel_path :: (nav_mesh: NavMesh, start: Vector3, end: Vector3, path: [] s32) -> [] Vector3 {
    if path.count < 2 {
        return .[];
    }

    portals: [..] [2] Vector3;
    portals.allocator = temp;
    array_reserve(*portals, path.count - 2);

    for i: 0..path.count - 2 {
        node := path[i];
        next_node := path[i + 1];
        portal := get_portal(nav_mesh, node, next_node);
        array_add(*portals, portal);
    }

    anchors: [..] Vector3;
    anchors.allocator = temp;

    anchor := start;
    array_add(*anchors, anchor);

    left_dir := portals[0][1] - anchor;
    right_dir := portals[0][0] - anchor;
    left_dir_portal_index := 0;
    right_dir_portal_index := 0;

    portal_index := 1;
    while portal_index < portals.count {
        portal := portals[portal_index];

        if is_right(right_dir, portal[1] - anchor) {
            // this is a fold
            anchor = portals[right_dir_portal_index][0];
            array_add(*anchors, anchor);

            remove_first_count(*portals, right_dir_portal_index + 1);
            left_dir = portals[0][1] - anchor;
            right_dir = portals[0][0] - anchor;
            left_dir_portal_index = 0;
            right_dir_portal_index = 0;

            portal_index = 1;
        } else if is_left(left_dir, portal[0] - anchor) {
            // this is a fold
            anchor = portals[left_dir_portal_index][1];
            array_add(*anchors, anchor);

            remove_first_count(*portals, left_dir_portal_index + 1);
            left_dir = portals[0][1] - anchor;
            right_dir = portals[0][0] - anchor;
            left_dir_portal_index = 0;
            right_dir_portal_index = 0;

            portal_index = 1;
        } else {
            if is_right(left_dir, portal[1] - anchor) {
                left_dir = portal[1] - anchor;
                left_dir_portal_index = portal_index;
            }
            if is_left(right_dir, portal[0] - anchor) {
                right_dir = portal[0] - anchor;
                right_dir_portal_index = portal_index;
            }

            portal_index += 1;
        }
    }

    array_add(*anchors, end);

    return anchors;
}

funnel_render :: () {
    // add := Vector3.{0, 2, 0};
    // draw_sphere(portal_anchor + add, .{0.1, 0.1, 0.1}, color = .{0.5, 0, 0, 1});

    // // for portal: portal_portals {
    // //     draw_sphere(portal[0] + add, .{0.1, 0.1, 0.1}, color = .{0.5, 0.5, 0.5, 1});
    // //     draw_sphere(portal[1] + add, .{0.1, 0.1, 0.1}, color = .{0.5, 0.5, 0.5, 1});
    // //     draw_line(portal_anchor + add, portal[0] + add, width = 2, color = .{0, 0.5, 0.5, 1});
    // //     draw_line(portal_anchor + add, portal[1] + add, width = 2, color = .{0.5, 0.5, 0, 1});
    // // }
    // draw_line(portal_anchor + add, portal_anchor + add + portal_left_dir * 100.0, width = 4, color = .{0, 0.5, 0, 1});
    // draw_line(portal_anchor + add, portal_anchor + add + portal_right_dir * 100.0, width = 4, color = .{0, 0, 0.5, 1});

    // draw_sphere(portal_most_recent_portal[0] + add, .{0.2, 0.2, 0.2}, color = .{1, 1, 1, 1});
    // draw_sphere(portal_most_recent_portal[1] + add, .{0.2, 0.2, 0.2}, color = .{0.5, 0.5, 0.5, 1});
}

#scope_file

// portal_anchor: Vector3;
// portal_portals: [..] [2] Vector3;
// portal_left_dir: Vector3;
// portal_right_dir: Vector3;
// portal_most_recent_portal: [2] Vector3;

remove_first_count :: (arr: *[] $T, count: int) {
    // TODO bad and slow
    for i: 0..count - 1 {
        array_ordered_remove_by_index(arr, 0);
    }
}

get_portal :: (nav_mesh: NavMesh, node: s32, next_node: s32) -> [2] Vector3 {
    for connected_node: nav_mesh.nodes[node] {
        if connected_node == next_node {
            return nav_mesh.portals[node][it_index];
        }
    }

    assert(false, "Did not find matching portal.\n");

    empty: [2] Vector3;
    return empty;
}

is_left :: (dir: Vector3, other: Vector3) -> bool {
    return cross(Vector2.{dir.x, dir.z}, Vector2.{other.x, other.z}) > 0.0001;
    // return dot(get_left(.{dir.x, dir.z}), .{other.x - dir.x, other.z - dir.z}) > 0.0001;
}

is_right :: (dir: Vector3, other: Vector3) -> bool {
    return cross(Vector2.{dir.x, dir.z}, Vector2.{other.x, other.z}) < -0.0001;
    // return dot(get_right(.{dir.x, dir.z}), .{other.x - dir.x, other.z - dir.z}) > 0.0001;
}

get_right :: (dir: Vector2) -> Vector2 {
    return .{dir.y, -dir.x};
}

get_left :: (dir: Vector2) -> Vector2 {
    return .{-dir.y, dir.x};
}
