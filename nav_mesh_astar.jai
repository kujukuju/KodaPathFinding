
get_node :: (nav_mesh: NavMesh, point: Vector3) -> s32 {
    best_node: s32 = -1;
    best_node_dist := FLOAT32_MAX;

    for points, node: nav_mesh.points {
        plane := get_plane(points);
        plane_point := nearest_point(plane, point);

        point_dist := FLOAT32_MAX;

        if intersects(points, plane_point) {
            point_dist = distance(point, plane_point);
        } else {
            for i: 0..points.count - 1 {
                curr := points[i];
                next := points[(i + 1) % points.count];

                line := Line3.{curr, next};
                point_dist = min(point_dist, distance(line, point));
            }
        }

        if point_dist < best_node_dist {
            best_node = cast(s32) node;
            best_node_dist = point_dist;
        }
    }

    return best_node;
}

find_path_a_star :: (nav_mesh: NavMesh, start: Vector3, end: Vector3, flags: u64, allowed_edge_radius: float, movement_preference: Vector3 = .{0, 0, 0}) -> [] s32 {
    start_node := get_floor_node(nav_mesh, start, allowed_edge_radius, flags);
    end_node := get_floor_node(nav_mesh, end, allowed_edge_radius, flags);

    return find_path_a_start(nav_mesh, start_node, end_node, flags, movement_preference);
}

find_path_a_star :: (nav_mesh: NavMesh, start_node: s32, end_node: s32, flags: u64, movement_preference: Vector3 = .{0, 0, 0}) -> [] s32 {
    visited: [..] bool;
    visited.allocator = temp;
    array_resize(*visited, nav_mesh.nodes.count);

    NodeEntry :: struct {
        node: s32;
        parent_node: s32;
        heuristic: float;
    }

    visited_history: [..] s32;
    visited_history.allocator = temp;
    array_resize(*visited_history, nav_mesh.nodes.count);

    open_queue: BinaryHeap(NodeEntry, (a: NodeEntry) -> float {
        return a.heuristic;
    });
    open_queue.allocator = temp;
    reserve(*open_queue, 64);

    insert(*open_queue, NodeEntry.{
        start_node,
        start_node,
        0.00001,
    });
    // visited[start_node] = 0.00001;

    while open_queue.entries.count > 0 {
        entry := pop_min(*open_queue);
        entry_node := entry.node;
        entry_parent_node := entry.parent_node;
        entry_heuristic := entry.heuristic;
        entry_center := nav_mesh.centers[entry_node];

        if visited[entry_node] {
            continue;
        }
        visited[entry_node] = true;

        visited_history[entry_node] = entry_parent_node;

        if entry_node == end_node {
            path: [..] s32;
            path.allocator = temp;

            path_node := end_node;
            array_add(*path, path_node);

            while path_node != start_node {
                path_node = visited_history[path_node];
                array_add(*path, path_node);
            }

            reverse(path);
            return path;
        }

        for adjacent_node: nav_mesh.nodes[entry.node] {
            if !valid_flags(nav_mesh.flags[adjacent_node], flags) {
                continue;
            }

            movement_preference_heuristic := dot(normalize(nav_mesh.centers[adjacent_node] - entry_center), movement_preference);
            // we move the 0 -> 1 heuristic away from 0
            movement_preference_heuristic = movement_preference_heuristic / sqrt(abs(movement_preference_heuristic));
            movement_preference_heuristic += 1.0;

            adjacent_distance := distance(nav_mesh.centers[adjacent_node], entry_center) * nav_mesh.weights[adjacent_node] * movement_preference_heuristic;
            adjacent_heuristic := entry_heuristic + adjacent_distance;

            insert(*open_queue, NodeEntry.{
                adjacent_node,
                entry_node,
                adjacent_heuristic,
            });
        }
    }

    return .[];
}

find_path_slide_direction :: (start: Vector3, direction: Vector3, flags: u64, allowed_edge_radius: float) -> Vector3 {
    using context.nav_mesh_manager;

    start_node, start_point := get_floor_node(*context.nav_mesh_manager.nav_mesh, start, allowed_edge_radius, flags);
    if start_node == -1 {
        return start;
    }

    path_nodes: [..] s32;
    path_nodes.allocator = temp;

    // path_points: [..] Vector3;
    // path_points.allocator = temp;

    slide_ray := Ray3.{start_point, direction};

    current_node := start_node;
    while !array_find(path_nodes, current_node) {
        array_add(*path_nodes, current_node);

        found_portal := false;

        for portal: nav_mesh.portals[current_node] {
            adjacent_node := nav_mesh.nodes[current_node][it_index];
            adjacent_flags := nav_mesh.flags[adjacent_node];

            if valid_flags(adjacent_flags, flags) && intersects_portal(slide_ray, portal) {
                found_portal = true;
                current_node = adjacent_node;
                break;
            }
        }

        if !found_portal {
            points := nav_mesh.points[current_node];
            for i: 0..points.count - 1 {
                curr := points[i];
                next := points[(i + 1) % points.count];

                if intersects_portal(slide_ray, .[curr, next]) {
                    return get_portal_intersection(.[curr, next], slide_ray);
                }
            }
        }
    }

    return start;
}

#scope_module

valid_flags :: (node_flags: u64, user_flags: u64) -> bool {
    return (node_flags & user_flags) == node_flags;
}

intersects_portal :: (ray: Ray3, portal: [2] Vector3) -> bool {
    dir_xz := Vector2.{ray.dir.x, ray.dir.z};
    portal_dirs_xz := Vector2.[.{portal[0].x - ray.p.x, portal[0].z - ray.p.z}, .{portal[1].x - ray.p.x, portal[1].z - ray.p.z}];

    directions: [2] float;
    directions[0] = cross(dir_xz, portal_dirs_xz[0]);
    directions[1] = cross(dir_xz, portal_dirs_xz[1]);

    return directions[0] <= 0 && directions[1] >= 0;
}
