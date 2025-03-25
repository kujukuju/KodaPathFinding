
funnel_path :: (nav_mesh: NavMesh, start: Vector3, end: Vector3, path: [] s32, $add_every_node: bool = true) -> [] Vector3 {
    if path.count < 2 {
        return .[];
    }

    portals: [..] [2] Vector3;
    portals.allocator = temp;
    array_reserve(*portals, path.count - 1);

    for i: 0..path.count - 2 {
        node := path[i];
        next_node := path[i + 1];
        portal := get_portal(nav_mesh, node, next_node);
        array_add(*portals, portal);
    }

    array_add(*portals, .[end, end]);

    anchors: [..] Vector3;
    anchors.allocator = temp;
    array_reserve(*anchors, path.count + 1);

    anchor := start;
    // array_add(*anchors, anchor);

    start_portal_index := 0;

    left_dir := portals[0][1] - anchor;
    right_dir := portals[0][0] - anchor;
    left_portal_index := 0;
    right_portal_index := 0;

    portal_index := 1;
    while portal_index < portals.count {
        portal := portals[portal_index];

        if left_dir == .{} {
            left_dir = portal[1] - anchor;
            left_portal_index = portal_index;
        }
        if right_dir == .{} {
            right_dir = portal[0] - anchor;
            right_portal_index = portal_index;
        }

        if is_right(right_dir, portal[1] - anchor) {
            next_index := right_portal_index + 1;
            next_anchor := portals[right_portal_index][0];

            #if add_every_node {
                line := Line3.{anchor, next_anchor};
                for i: start_portal_index..right_portal_index - 1 {
                    array_add(*anchors, get_portal_intersection(portals[i], line));
                }
                start_portal_index = right_portal_index + 1;
            }

            anchor = next_anchor;
            array_add(*anchors, anchor);

            left_dir = portals[next_index][1] - anchor;
            right_dir = portals[next_index][0] - anchor;
            left_portal_index = next_index;
            right_portal_index = next_index;

            portal_index = next_index + 1;
        } else if is_left(left_dir, portal[0] - anchor) {
            next_index := left_portal_index + 1;
            next_anchor := portals[left_portal_index][1];

            #if add_every_node {
                line := Line3.{anchor, next_anchor};
                for i: start_portal_index..left_portal_index - 1 {
                    array_add(*anchors, get_portal_intersection(portals[i], line));
                }
                start_portal_index = left_portal_index + 1;
            }

            anchor = next_anchor;
            array_add(*anchors, anchor);

            left_dir = portals[next_index][1] - anchor;
            right_dir = portals[next_index][0] - anchor;
            left_portal_index = next_index;
            right_portal_index = next_index;

            portal_index = next_index + 1;
        } else {
            if is_right(left_dir, portal[1] - anchor) {
                left_dir = portal[1] - anchor;
                left_portal_index = portal_index;
            }
            if is_left(right_dir, portal[0] - anchor) {
                right_dir = portal[0] - anchor;
                right_portal_index = portal_index;
            }

            portal_index += 1;
        }
    }

    #if add_every_node {
        line := Line3.{anchor, end};
        for i: start_portal_index..portals.count - 1 {
            array_add(*anchors, get_portal_intersection(portals[i], line));
        }
        start_portal_index = left_portal_index + 1;
    }

    array_add(*anchors, end);

    return anchors;
}

#scope_file

get_portal_intersection :: (portal: [2] Vector3, line: Line3) -> Vector3 {
    l0 := portal[0];
    ldir := portal[1] - portal[0];
    m0 := Vector3.{line.p1.x, 0, line.p1.z};
    mdir := Vector3.{line.p1.x, 0, line.p1.z} - .{line.p2.x, 0, line.p2.z};

    denom := ldir.x * mdir.z - ldir.z * mdir.x;
    if denom == 0 {
        return portal[0] / 2.0 + portal[1] / 2.0;
    }
    t := ((m0.x - l0.x) * mdir.z - (m0.z - l0.z) * mdir.x) / denom;

    return l0 + t * ldir;
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
}

is_right :: (dir: Vector3, other: Vector3) -> bool {
    return cross(Vector2.{dir.x, dir.z}, Vector2.{other.x, other.z}) < -0.0001;
}

get_right :: (dir: Vector2) -> Vector2 {
    return .{dir.y, -dir.x};
}

get_left :: (dir: Vector2) -> Vector2 {
    return .{-dir.y, dir.x};
}
