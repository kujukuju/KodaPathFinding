
// be right back short break

NavMesh :: struct {
    nodes: [] [] s32;
    portals: [] [] [2] Vector3;
    points: [] [] Vector3;
    centers: [] Vector3;

    nodes_buffer: [..] s32;
    portals_buffer: [..] [2] Vector3;
    points_buffer: [..] Vector3;
}

generate_nav_mesh :: (polygons: [] NavMeshPolygon) -> NavMesh {
    nav_mesh: NavMesh;
    nav_mesh.nodes = NewArray(polygons.count, [] s32);
    nav_mesh.portals = NewArray(polygons.count, [] [2] Vector3);
    nav_mesh.points = NewArray(polygons.count, [] Vector3);
    nav_mesh.centers = NewArray(polygons.count, Vector3);

    // centers
    for polygon, polygon_index: polygons {
        for i: 0..polygon.count - 1 {
            nav_mesh.centers[polygon_index] += polygon.points[i];
        }
        nav_mesh.centers[polygon_index] /= polygon.count;
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

    return nav_mesh;
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
