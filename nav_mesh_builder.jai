
NavMesh :: struct {
    nodes: [] [] s32;
    portals: [] [] [2] Vector3;
    points: [] [] Vector3;

    centers: [] Vector3;
    normals: [] Vector3;
    flags: [] u64;
    weights: [] u32;

    nodes_buffer: [..] s32;
    portals_buffer: [..] [2] Vector3;
    points_buffer: [..] Vector3;

    node_tree: BoundingTree(s32);
}

SerializedView :: struct {
    offset: int;
    count: int;
}

SerializedNavMesh :: struct {
    nodes: [] SerializedView;
    portals: [] SerializedView;
    points: [] SerializedView;

    centers: [] Vector3;
    normals: [] Vector3;
    flags: [] u64;
    weights: [] u32;

    nodes_buffer: [..] s32;
    portals_buffer: [..] [2] Vector3;
    points_buffer: [..] Vector3;
}

serialize :: (nav_mesh: NavMesh) -> SerializedNavMesh {
    serialized: SerializedNavMesh;

    serialized.nodes = NewArray(nav_mesh.nodes.count, SerializedView);
    serialized.portals = NewArray(nav_mesh.portals.count, SerializedView);
    serialized.points = NewArray(nav_mesh.points.count, SerializedView);

    for nodes_view: nav_mesh.nodes {
        serialized.nodes[it_index].offset = cast(int) nodes_view.data - cast(int) nav_mesh.nodes_buffer.data;
        serialized.nodes[it_index].count = nodes_view.count;
    }
    for portals_view: nav_mesh.portals {
        serialized.portals[it_index].offset = cast(int) portals_view.data - cast(int) nav_mesh.portals_buffer.data;
        serialized.portals[it_index].count = portals_view.count;
    }
    for points_view: nav_mesh.points {
        serialized.points[it_index].offset = cast(int) points_view.data - cast(int) nav_mesh.points_buffer.data;
        serialized.points[it_index].count = points_view.count;
    }

    serialized.centers = array_copy(nav_mesh.centers);
    serialized.normals = array_copy(nav_mesh.normals);
    serialized.flags = array_copy(nav_mesh.flags);
    serialized.weights = array_copy(nav_mesh.weights);

    array_copy(*serialized.nodes_buffer, nav_mesh.nodes_buffer);
    array_copy(*serialized.portals_buffer, nav_mesh.portals_buffer);
    array_copy(*serialized.points_buffer, nav_mesh.points_buffer);

    return serialized;
}

deserialize :: (serialized: SerializedNavMesh) -> NavMesh {
    nav_mesh: NavMesh;

    nav_mesh.nodes = NewArray(serialized.nodes.count, [] s32);
    nav_mesh.portals = NewArray(serialized.portals.count, [] [2] Vector3);
    nav_mesh.points = NewArray(serialized.points.count, [] Vector3);

    nav_mesh.centers = array_copy(serialized.centers);
    nav_mesh.normals = array_copy(serialized.normals);
    nav_mesh.flags = array_copy(serialized.flags);
    nav_mesh.weights = array_copy(serialized.weights);

    array_copy(*nav_mesh.nodes_buffer, serialized.nodes_buffer);
    array_copy(*nav_mesh.portals_buffer, serialized.portals_buffer);
    array_copy(*nav_mesh.points_buffer, serialized.points_buffer);

    for nodes_view: serialized.nodes {
        nav_mesh.nodes[it_index].data = cast(*s32) (cast(int) nav_mesh.nodes_buffer.data + nodes_view.offset);
        nav_mesh.nodes[it_index].count = nodes_view.count;
    }
    for portals_view: serialized.portals {
        nav_mesh.portals[it_index].data = cast(*[2] Vector3) (cast(int) nav_mesh.portals_buffer.data + portals_view.offset);
        nav_mesh.portals[it_index].count = portals_view.count;
    }
    for points_view: serialized.points {
        nav_mesh.points[it_index].data = cast(*Vector3) (cast(int) nav_mesh.points_buffer.data + points_view.offset);
        nav_mesh.points[it_index].count = points_view.count;
    }

    build_nav_mesh_tree(*nav_mesh);

    return nav_mesh;
}

free :: (serialized: SerializedNavMesh) {
    array_free(serialized.nodes);
    array_free(serialized.portals);
    array_free(serialized.points);

    array_free(serialized.centers);
    array_free(serialized.normals);
    array_free(serialized.flags);
    array_free(serialized.weights);

    array_free(serialized.nodes_buffer);
    array_free(serialized.portals_buffer);
    array_free(serialized.points_buffer);
}

free :: (nav_mesh: NavMesh) {
    array_free(nav_mesh.centers);
    array_free(nav_mesh.normals);
    array_free(nav_mesh.flags);
    array_free(nav_mesh.weights);

    array_free(nav_mesh.nodes_buffer);
    array_free(nav_mesh.portals_buffer);
    array_free(nav_mesh.points_buffer);

    free_bounding_tree(nav_mesh.node_tree);
}

generate_nav_mesh :: (polygons: [] NavMeshPolygon) -> NavMesh {
    nav_mesh: NavMesh;
    nav_mesh.nodes = NewArray(polygons.count, [] s32);
    nav_mesh.portals = NewArray(polygons.count, [] [2] Vector3);
    nav_mesh.points = NewArray(polygons.count, [] Vector3);
    nav_mesh.centers = NewArray(polygons.count, Vector3);
    nav_mesh.normals = NewArray(polygons.count, Vector3);
    nav_mesh.flags = NewArray(polygons.count, u64);
    nav_mesh.weights = NewArray(polygons.count, u32);

    // centers
    for polygon, polygon_index: polygons {
        for i: 0..polygon.count - 1 {
            nav_mesh.centers[polygon_index] += polygon.points[i];
        }
        nav_mesh.centers[polygon_index] /= polygon.count;
    }

    // normals
    for polygon, polygon_index: polygons {
        points: [] Vector3;
        points.data = polygon.points.data;
        points.count = polygon.count;
        nav_mesh.normals[polygon_index] = get_normal(points);
    }

    // flags
    for polygon, polygon_index: polygons {
        nav_mesh.flags[polygon_index] = polygon.level_flags;
    }

    // weights
    for polygon, polygon_index: polygons {
        nav_mesh.weights[polygon_index] = max(polygon.weight, 1);
    }

    // points
    current_points: [..] Vector3;
    current_points.allocator = temp;

    for a: 0..polygons.count - 1 {
        current_points.count = 0;

        for ai: 0..polygons[a].count - 1 {
            array_add(*current_points, polygons[a].points[ai]);
        }

        nav_mesh.points[a] = add_points(*nav_mesh, current_points);
    }

    // connected nodes
    current_nodes: [..] s32;
    current_nodes.allocator = temp;

    for a: 0..polygons.count - 1 {
        current_nodes.count = 0;

        for b: 0..polygons.count - 1 {
            if a == b {
                continue;
            }

            // for one way polygons we only allow you to go down
            if polygons[a].editor_flags & cast(u64) NavMeshEditorFlags.OneWay {
                for point_b: nav_mesh.points[b] {
                    if point_b.y > nav_mesh.centers[a].y {
                        continue b;
                    }
                }
            }

            found_count := 0;

            for point_a: nav_mesh.points[a] {
                if array_find(nav_mesh.points[b], point_a) {
                    found_count += 1;
                }

                if found_count >= 2 {
                    array_add(*current_nodes, cast(s32) b);
                    continue b;
                }
            }
        }

        nav_mesh.nodes[a] = add_nodes(*nav_mesh, current_nodes);
    }

    // portals
    current_portals: [..] [2] Vector3;
    current_portals.allocator = temp;

    for a: 0..polygons.count - 1 {
        current_portals.count = 0;
        array_resize(*current_portals, nav_mesh.nodes[a].count);

        for b, node_index_a: nav_mesh.nodes[a] {
            found_disconnected_vertex := false;
            found_first_vertex := false;

            ai := 0;
            while true {
                defer ai += 1;

                point_a := nav_mesh.points[a][ai % nav_mesh.points[a].count];
                found_vertex := array_find(nav_mesh.points[b], point_a);

                if !found_disconnected_vertex {
                    found_disconnected_vertex = !found_vertex;
                } else if !found_first_vertex {
                    if found_vertex {
                        found_first_vertex = true;
                        current_portals[node_index_a][0] = point_a;
                        current_portals[node_index_a][1] = point_a;
                    }
                } else if found_vertex {
                    current_portals[node_index_a][1] = point_a;
                } else {
                    break;
                }
            }
        }

        nav_mesh.portals[a] = add_portals(*nav_mesh, current_portals);
    }

    build_nav_mesh_tree(*nav_mesh);

    return nav_mesh;
}

build_nav_mesh_tree :: (nav_mesh: *NavMesh) {
    nav_mesh.node_tree = create_bounding_tree(s32, cast(s32) nav_mesh.nodes.count);
    for node: 0..nav_mesh.nodes.count - 1 {
        aabb: AABB3;
        aabb.lower = nav_mesh.points[node][0];
        aabb.upper = nav_mesh.points[node][0];

        for point: nav_mesh.points[node] {
            aabb.lower = min(aabb.lower, point);
            aabb.upper = max(aabb.upper, point);
        }

        create_proxy(*nav_mesh.node_tree, aabb, cast(s32) node);
    }
}

get_floor_node :: (nav_mesh: *NavMesh, point: Vector3, allowed_edge_radius: float, user_flags: u64) -> s32, Vector3 {
    radius := 2.0;

    ray := Ray3.{point, .{0, -1, 0}};

    nearest_node: s32 = -1;
    nearest_p: Vector3;
    nearest_ray_dist := FLOAT32_MAX;
    nearest_edge_dist := FLOAT32_MAX;

    iter_count := 0;
    while nearest_ray_dist == FLOAT32_MAX && nearest_edge_dist == FLOAT32_MAX && iter_count < 4 {
        aabb: AABB3;
        aabb.lower = point - .{radius, radius * 8.0, radius};
        aabb.upper = point + .{radius, radius * 8.0, radius};

        for node: query(*nav_mesh.node_tree, aabb) {
            node_flags := nav_mesh.flags[node];
            if !valid_flags(node_flags, user_flags) {
                continue;
            }

            points := nav_mesh.points[node];
            // TODO could optimize
            node_plane := get_plane(points);
            node_plane_point, _, node_plane_hit := intersection_simple(node_plane, ray);
            node_plane_dist := distance(node_plane_point, point);
            if node_plane_hit && intersects(points, node_plane.normal, node_plane_point) {
                if node_plane_dist < nearest_ray_dist {
                    nearest_node = node;
                    nearest_p = node_plane_point;
                    nearest_ray_dist = node_plane_dist;
                }
            } else {
                // we only look for edge points if there hasn't yet been a ray point
                for i: 0..points.count - 1 {
                    curr := points[i];
                    next := points[(i + 1) % points.count];

                    line := Line3.{curr, next};
                    node_line_point := nearest_point(line, ray);
                    node_ray_point := nearest_point(ray, node_line_point);
                    node_line_ray_dist := distance(node_line_point, node_ray_point);
                    node_edge_dist := node_line_ray_dist + distance(node_ray_point, point);
                    if node_line_ray_dist < allowed_edge_radius {
                        // treat as ray
                        if node_edge_dist < nearest_ray_dist {
                            nearest_node = node;
                            nearest_p = node_line_point;
                            nearest_ray_dist = node_edge_dist;
                        }
                    } else {
                        // treat as edge
                        if nearest_ray_dist == FLOAT32_MAX && node_edge_dist < nearest_edge_dist {
                            nearest_node = node;
                            nearest_p = node_line_point;
                            nearest_edge_dist = node_edge_dist;
                        }
                    }
                }
            }

            // if node_plane_hit && intersects(points, node_plane.normal, node_plane_point) {
            //     node_plane_dist := distance(node_plane_point, point);
            //     if node_plane_dist < nearest_dist {
            //         nearest_node = node;
            //         nearest_p = node_plane_point;
            //         nearest_dist = node_plane_dist;
            //     }
            // } else {
            //     // if no direct plane intersection then we get the nearest point on the plane specifically to the ray
            //     for i: 0..points.count - 1 {
            //         curr := points[i];
            //         next := points[(i + 1) % points.count];

            //         line := Line3.{curr, next};
            //         node_line_point := nearest_point(line, ray);
            //         node_ray_point := nearest_point(ray, node_line_point);
            //         node_line_dist := distance(node_line_point, node_ray_point) + distance(node_ray_point, point) * nearness_penalty_heuristic;
            //         if node_line_dist < nearest_dist {
            //             nearest_node = node;
            //             nearest_p = node_line_point;
            //             nearest_dist = node_line_dist;
            //         }
            //     }
            // }
        }

        radius *= 2;
        iter_count += 1;
    }

    return nearest_node, nearest_p;
}

#scope_file

reserve_buffers :: (nav_mesh: *NavMesh, nodes: int, portals: int, points: int) {
    if nav_mesh.nodes_buffer.count + nodes > nav_mesh.nodes_buffer.allocated {
        reserve_and_fix_buffer(*nav_mesh.nodes_buffer, *nav_mesh.nodes, nodes);
    }

    if nav_mesh.portals_buffer.count + portals > nav_mesh.portals_buffer.allocated {
        reserve_and_fix_buffer(*nav_mesh.portals_buffer, *nav_mesh.portals, portals);
    }

    if nav_mesh.points_buffer.count + points > nav_mesh.points_buffer.allocated {
        reserve_and_fix_buffer(*nav_mesh.points_buffer, *nav_mesh.points, points);
    }
}

reserve_and_fix_buffer :: (buffer: *[..] $T, views: *[] [] T, additional: int) {
    old_pointer := buffer.data;
    array_reserve(buffer, buffer.count + additional);

    new_pointer := buffer.data;
    if old_pointer == new_pointer {
        return;
    }

    for *view: views.* {
        if !view.data {
            continue;
        }

        offset := view.data - old_pointer;
        view.data = new_pointer + offset;
    }
}

add_nodes :: (nav_mesh: *NavMesh, nodes: [] s32) -> [] s32 {
    reserve_buffers(nav_mesh, nodes.count, 0, 0);
    assert(nav_mesh.nodes_buffer.allocated >= nav_mesh.nodes_buffer.count + nodes.count, "Did not reserve enough space.\n");

    view: [] s32;
    view.data = nav_mesh.nodes_buffer.data + nav_mesh.nodes_buffer.count;
    view.count = nodes.count;

    old_pointer := nav_mesh.nodes_buffer.data;
    array_add(*nav_mesh.nodes_buffer, ..nodes);
    new_pointer := nav_mesh.nodes_buffer.data;
    assert(new_pointer == old_pointer);

    return view;
}

add_portals :: (nav_mesh: *NavMesh, portals: [] [2] Vector3) -> [] [2] Vector3 {
    reserve_buffers(nav_mesh, 0, portals.count, 0);
    assert(nav_mesh.portals_buffer.allocated >= nav_mesh.portals_buffer.count + portals.count, "Did not reserve enough space.\n");

    view: [] [2] Vector3;
    view.data = nav_mesh.portals_buffer.data + nav_mesh.portals_buffer.count;
    view.count = portals.count;

    old_pointer := nav_mesh.portals_buffer.data;
    array_add(*nav_mesh.portals_buffer, ..portals);
    new_pointer := nav_mesh.portals_buffer.data;
    assert(new_pointer == old_pointer);

    return view;
}

add_points :: (nav_mesh: *NavMesh, points: [] Vector3) -> [] Vector3 {
    reserve_buffers(nav_mesh, 0, 0, points.count);
    assert(nav_mesh.points_buffer.allocated >= nav_mesh.points_buffer.count + points.count, "Did not reserve enough space.\n");

    view: [] Vector3;
    view.data = nav_mesh.points_buffer.data + nav_mesh.points_buffer.count;
    view.count = points.count;

    old_pointer := nav_mesh.points_buffer.data;
    array_add(*nav_mesh.points_buffer, ..points);
    new_pointer := nav_mesh.points_buffer.data;
    assert(new_pointer == old_pointer);

    return view;
}

array_find :: (arr: [] $T, entry: T) -> bool {
    for a: arr {
        if a == entry {
            return true;
        }
    }
    return false;
}
