
MAX_VERTS :: 16;

nav_mesh_editor_enabled: bool;

EditorState :: enum {
    None :: 0;
    CreatingPolygon :: 1;
}

NavMeshEditor :: struct {
    state: EditorState;
    polygons: [..] NavMeshPolygon;
}

NavMeshPolygon :: struct {
    points: [MAX_VERTS] Vector3;
    count: s32;
}

operator == :: (a: NavMeshPolygon, b: NavMeshPolygon) -> bool {
    matches := a.count == b.count;
    for i: 0..a.count - 1 {
        matches = matches && a.points[i] == b.points[i];
    }
    return matches;
}

// needed features
// drag vertices into each other and remove them
// auto convexify if youre holding some key

nav_mesh_editor_toggle :: () {
    nav_mesh_editor_enabled = !nav_mesh_editor_enabled;

    if nav_mesh_editor_enabled {
        color_seed = random_get();
    }
}

nav_mesh_set_raycast_callback :: (callback: #type () -> Vector3) {
    raycast_callback = callback;
}

nav_mesh_set_console_print_callback :: (callback: #type (text: string)) {
    console_print_callback = callback;
}

nav_mesh_set_inputs :: (raw_mouse_left: bool, raw_mouse_right: bool, raw_middle_mouse: bool, raw_shift: bool) {
    mouse_down_left = raw_mouse_left;
    mouse_down_right = raw_mouse_right;
    middle_mouse = raw_middle_mouse;
    shift = raw_shift;
}

nav_mesh_set_camera :: (scene_camera: Camera3) {
    camera = scene_camera;
}

nav_mesh_editor_update :: () {
    delta_mouse_left := mouse_down_left && !last_mouse_left;
    last_mouse_left = mouse_down_left;

    delta_mouse_right := mouse_down_right && !last_mouse_right;
    last_mouse_right = mouse_down_right;

    delta_middle_mouse := middle_mouse && !last_middle_mouse;
    last_middle_mouse = middle_mouse;

    delta_shift := shift && !last_shift;
    last_shift = shift;

    if editor.state == .None {
        concave_polygon = shift;
        concave_polygon_points.count = 0;
    }

    concave_convex_triangles := nav_mesh_concave_to_triangles();
    concave_convex_polygons := nav_mesh_triangles_to_convex(concave_convex_triangles);
    array_add(*editor.polygons, ..concave_convex_polygons);
    defer editor.polygons.count -= concave_convex_polygons.count;

    raw_point = nav_mesh_get_point();
    last_point = current_point;
    current_point = raw_point;
    vertex_point, vertex_point_found := nav_mesh_get_snap_point();
    edge_point: Vector3;
    edge_point1: Vector3;
    edge_point2: Vector3;
    edge_point_found: bool;
    if vertex_point_found {
        current_point = vertex_point;
    } else if !concave_polygon {
        edge_point, edge_point1, edge_point2, edge_point_found = nav_mesh_get_snap_edge();
        if edge_point_found {
            current_point = edge_point;
        }
    }

    if editor.state == .CreatingPolygon && !concave_polygon {
        corrected_point := nav_mesh_correct_polygon_point(current_point);
        if distance(corrected_point, current_point) > 0.01 {
            current_point = corrected_point;

            // no snaping
            vertex_point_found = false;
            edge_point_found = false;
        }
    }

    if dragging && !mouse_down_right {
        dragging = false;
    }

    last_state := editor.state;
    defer if editor.state == .None && editor.state != last_state {
        nav_save(to_lower_copy(tprint("%.temp.nav", context.selected_level),, temp));
    }

    if #complete editor.state == {
        case .None;
            if delta_mouse_left {
                // cannot start inside another polygon
                polygon_found := false;
                if !concave_polygon {
                    _, polygon_found = nav_mesh_get_snap_polygon(current_point, 0.2);
                }

                if !polygon_found {
                    valid := true;
                    if edge_point_found {
                        // if you're placing on an existing edge point
                        valid = nav_mesh_split_edge(edge_point1, edge_point2, edge_point);
                    }

                    if valid {
                        editor.state = .CreatingPolygon;

                        if concave_polygon {
                            array_add(*concave_polygon_points, current_point);
                        } else {
                            polygon := array_add(*editor.polygons);
                            nav_mesh_add_point(current_point);
                        }
                    }
                }
            } else if delta_middle_mouse {
                print("middle mouse\n");
                polygon, polygon_found := nav_mesh_get_snap_polygon();
                if polygon_found {
                    print("found\n");
                    for editor_polygon: editor.polygons {
                        if editor_polygon == polygon {
                            print("removing %\n", it_index);
                            array_ordered_remove_by_index(*editor.polygons, it_index);
                            break;
                        }
                    }
                }
            } else if delta_mouse_right {
                if vertex_point_found {
                    dragging = true;
                    dragging_points = .[vertex_point, vertex_point];
                } else {
                    if edge_point_found {
                        dragging = true;
                        dragging_points = .[edge_point1, edge_point2];
                    }
                }
            } else if mouse_down_right {
                if dragging {
                    new_dragging_points := dragging_points;
                    new_dragging_points[0] += current_point - last_point;
                    new_dragging_points[1] += current_point - last_point;

                    invalidated_something := false;

                    for *polygon: editor.polygons {
                        for i: 0..polygon.count - 1 {
                            if polygon.points[i] == dragging_points[0] {
                                polygon.points[i] = new_dragging_points[0];
                            } else if polygon.points[i] == dragging_points[1] {
                                polygon.points[i] = new_dragging_points[1];
                            }
                        }

                        if !is_polygon_convex(polygon.*) {
                            invalidated_something = true;
                            break;
                        }
                    }

                    if invalidated_something {
                        for *polygon: editor.polygons {
                            for i: 0..polygon.count - 1 {
                                if polygon.points[i] == new_dragging_points[0] {
                                    polygon.points[i] = dragging_points[0];
                                } else if polygon.points[i] == new_dragging_points[1] {
                                    polygon.points[i] = dragging_points[1];
                                }
                            }
                        }

                        current_point = last_point;
                    } else {
                        dragging_points = new_dragging_points;
                    }

                    // remove now duplicate points bc we snapped
                    for *polygon: editor.polygons {
                        i: s32;
                        while i < polygon.count {
                            curr := polygon.points[i];
                            next := polygon.points[(i + 1) % polygon.count];

                            if curr == next {
                                for a: i..polygon.count - 2 {
                                    polygon.points[a] = polygon.points[a + 1];
                                }
                                polygon.count -= 1;
                            } else {
                                i += 1;
                            }
                        }
                    }
                }
            }

        case .CreatingPolygon;
            if delta_mouse_left {
                if concave_polygon {
                    if concave_polygon_points[0] == current_point {
                        if concave_polygon_points.count < 3 {
                            editor.state = .None;
                        } else {
                            if concave_convex_polygons.count > 0 {
                                editor.state = .None;
                                editor.polygons.count += concave_convex_polygons.count;
                            }
                        }
                    } else {
                        valid := true;
                        if edge_point_found {
                            valid = nav_mesh_split_edge(edge_point1, edge_point2, edge_point);
                        }

                        if valid {
                            array_add(*concave_polygon_points, current_point);
                        }
                    }
                } else {
                    polygon := editor.polygons[editor.polygons.count - 1];
                    if polygon.points[0] == current_point {
                        editor.state = .None;

                        if polygon.count < 3 {
                            editor.polygons.count -= 1;
                        }
                    } else {
                        _, polygon_found := nav_mesh_get_snap_polygon(current_point, 0.2, ignore_last_polygon = true);
                        if !polygon_found {
                            valid := true;
                            if edge_point_found {
                                valid = nav_mesh_split_edge(edge_point1, edge_point2, edge_point);
                            }

                            if valid {
                                nav_mesh_add_point(current_point);
                            }
                        }
                    }
                }
            } else if delta_mouse_right {
                if concave_polygon {
                    if concave_polygon_points.count < 3 {
                        editor.state = .None;
                    } else {
                        if concave_convex_polygons.count > 0 {
                            editor.state = .None;
                            editor.polygons.count += concave_convex_polygons.count;
                        }
                    }
                } else {
                    editor.state = .None;
                    if editor.polygons[editor.polygons.count - 1].count < 3 {
                        editor.polygons.count -= 1;
                    }
                }
            } else if delta_middle_mouse {
                editor.state = .None;

                if !concave_polygon {
                    editor.polygons.count -= 1;
                }
            }
    }
}

nav_mesh_editor_render :: () {
    #import "GL";

    if !nav_mesh_editor_enabled {
        return;
    }

    nav_mesh_editor_update();

    triangles: [..] Vector3;
    triangles.allocator = temp;

    colors: [..] Vector3;
    colors.allocator = temp;

    lines: [..] Vector3;
    lines.allocator = temp;

    thicknesses: [..] float;
    thicknesses.allocator = temp;

    points: [..] Vector3;
    points.allocator = temp;

    random_seed(color_seed);

    last_color: Vector3;

    concave_convex_polygons := nav_mesh_concave_to_triangles();
    array_add(*editor.polygons, ..concave_convex_polygons);
    defer editor.polygons.count -= concave_convex_polygons.count;

    editing_count := 0;
    if editor.state == .CreatingPolygon {
        editing_count = max(concave_convex_polygons.count, 1);
    }

    for polygon: editor.polygons {
        hue := random_get_zero_to_one_open();
        rgb: Vector3;
        rgb.x, rgb.y, rgb.z = hsv_to_rgb(hue * 360.0, 0.34, 0.74);
        thickness := 0.02;

        if editor.state == .CreatingPolygon && it_index >= editor.polygons.count - editing_count {
            rgb = .{1, 1, 1};
            thickness = 0.04;
        }

        last_color = rgb;

        for i: 1..polygon.count - 2 {
            a := polygon.points[0];
            b := polygon.points[i];
            c := polygon.points[i + 1];

            array_add(*triangles, a, b, c);
            array_add(*colors, rgb, rgb, rgb);
        }

        for i: 0..polygon.count - 1 {
            a := polygon.points[i];
            b := polygon.points[(i + 1) % polygon.count];

            array_add(*lines, a, b);
            array_add(*thicknesses, thickness);

            array_add(*points, a);
        }
    }

    if editor.state == .CreatingPolygon {
        if concave_polygon {
            if concave_polygon_points.count > 0 {
                for i: 0..concave_polygon_points.count - 2 {
                    array_add(*lines, concave_polygon_points[i], concave_polygon_points[i + 1]);
                    array_add(*thicknesses, 0.04);
                }

                array_add(*lines, concave_polygon_points[concave_polygon_points.count - 1], current_point);
                array_add(*thicknesses, 0.04);
            }
        } else if editor.polygons.count > 0 {
            lines.count -= 2;
            thicknesses.count -= 1;

            polygon := editor.polygons[editor.polygons.count - 1];
            a := polygon.points[0];
            b := polygon.points[polygon.count - 1];
            c := current_point;

            array_add(*triangles, a, b, c);
            array_add(*lines, b, c);
            array_add(*thicknesses, 0.04);
            array_add(*colors, last_color, last_color, last_color);
        }
    }

    glEnable(GL_POLYGON_OFFSET_FILL);
    defer glDisable(GL_POLYGON_OFFSET_FILL);
    glPolygonOffset(-0.5, -0.5);

    {
        set_depth_func_scope(.GreaterEqual);
        set_depth_write_scope(false);

        nav_mesh_editor_draw(0.25);
    }

    {
        set_depth_func_scope(.LessEqual);
        set_depth_write_scope(false);

        nav_mesh_editor_draw(1.0);
    }
}

nav_mesh_editor_draw :: (alpha: float) #expand {
    draw_triangles(`triangles, `colors, color = .{1, 1, 1, alpha});

    // TODO draw lines
    i := 0;
    while i < `lines.count {
        defer i += 2;

        draw_line_mesh(`lines[i], `lines[(i + 1) % `lines.count], `thicknesses[i / 2], color = .{0, 0, 0, alpha});
    }

    draw_spheres(`points, .{0.05, 0.05, 0.05}, color = .{1, 1, 1, alpha});

    draw_sphere(raw_point, .{0.1, 0.1, 0.1}, color = .{0.5, 0.8, 0.82, alpha});
    draw_sphere(current_point, .{0.1, 0.1, 0.1}, color = .{0.5, 0.52, 0.8, alpha});
}

nav_mesh_editor_render_interface :: () {
    if !nav_mesh_editor_enabled {
        return;
    }


}

nav_save :: (name: string) {
    bytes: [..] u8;
    bytes.allocator = temp;
    serializer_write(*bytes, editor);

    success := write_entire_file(name, bytes.data, bytes.count);
    if success {
        console_print_callback("Saved nav mesh.");
    } else {
        console_print_callback("Could not save nav mesh.");
    }
} @command

nav_load :: (name: string) {
    file_string, success := read_entire_file(name);
    if success {
        bytes: [] u8;
        bytes.data = file_string.data;
        bytes.count = file_string.count;

        editor = .{};
        serializer_read(bytes, *editor);

        console_print_callback("Loaded nav mesh.");
    } else {
        console_print_callback("Could not load nav mesh.");
    }
} @command

#scope_file

nav_mesh_concave_to_triangles :: () -> [] NavMeshPolygon {
    if !concave_polygon_points {
        return .[];
    }

    convex: [..] NavMeshPolygon;
    convex.allocator = temp;

    triangles: [..] Triangle3;
    triangles.allocator = temp;

    expected_normal := -normalize(get_forward_vector(camera));

    remaining := array_copy(concave_polygon_points,, temp);

    while remaining.count >= 3 {
        ear_found := false;

        i := 0;
        while vertex_loop := i < remaining.count {
            defer i += 1;

            i1 := (i + 1) % remaining.count;
            i2 := (i + 2) % remaining.count;

            triangle: Triangle3;
            triangle.a = remaining[i];
            triangle.b = remaining[i1];
            triangle.c = remaining[i2];

            // it must be convex
            if dot(get_normal(triangle), expected_normal) < 0.0001 {
                continue;
            }

            // it must not have any other points inside of it
            for vertex: concave_polygon_points {
                if vertex == triangle.a || vertex == triangle.b || vertex == triangle.c {
                    continue;
                }

                if intersects(triangle, vertex) {
                    continue vertex_loop;
                }
            }

            ear_found = true;

            array_add(*triangles, triangle);
            array_ordered_remove_by_index(*remaining, i1);

            break;
        }

        if !ear_found {
            return .[];
        }
    }

    for triangle: triangles {
        polygon: NavMeshPolygon;
        polygon.points[0] = triangle.a;
        polygon.points[1] = triangle.b;
        polygon.points[2] = triangle.c;
        polygon.count = 3;

        array_add(*convex, polygon);
    }

    return convex;
}

nav_mesh_triangles_to_convex :: (triangles: [] NavMeshPolygon) -> [] NavMeshPolygon {
    share_edge :: (a: NavMeshPolygon, b: NavMeshPolygon) -> bool, int, int, int, int {
        for ai: 0..a.count - 1 {
            curr_a := a.points[ai];
            next_a := a.points[(ai + 1) % a.count];

            for bi: 0..b.count - 1 {
                curr_b := b.points[bi];
                next_b := b.points[(bi + 1) % b.count];

                if curr_a == next_b && next_a == curr_b {
                    return true, ai, (ai + 1) % a.count, (bi + 1) % b.count, bi;
                }
            }
        }

        return false, 0, 0, 0, 0;
    }

    merge :: (a: NavMeshPolygon, b: NavMeshPolygon) -> NavMeshPolygon {
        _, a1, a2, b1, b2 := share_edge(a, b);

        a1_count := (a2 + a.count - a1) % a.count;
        a2_count := a.count - a1_count;

        merged: NavMeshPolygon;
        for i: 0..a1_count - 1 {
            ai := (a1 + i) % a.count;

            merged.points[merged.count] = a.points[ai];
            merged.count += 1;
        }

        for i: 0..cast(int) b.count - 3 {
            bi := (b1 + i + 1) % b.count;

            merged.points[merged.count] = b.points[bi];
            merged.count += 1;
        }

        for i: 0..a2_count - 1 {
            ai := (a2 + i) % a.count;

            merged.points[merged.count] = a.points[ai];
            merged.count += 1;
        }

        return merged;
    }

    convex := array_copy(triangles,, temp);

    did_merge := true;
    while did_merge {
        did_merge = false;

        a := 0;
        while a < convex.count {
            defer a += 1;

            convex_a := convex[a];

            b := a + 1;
            while b < convex.count {
                defer b += 1;

                convex_b := convex[b];

                if !share_edge(convex_a, convex_b) {
                    continue;
                }

                final_vertex_count := convex_a.count + convex_b.count - 2;
                if final_vertex_count > MAX_VERTS {
                    continue;
                }

                merged := merge(convex_a, convex_b);
                if !is_polygon_convex(merged) {
                    continue;
                }

                did_merge = true;

                convex[a] = merged;
                array_ordered_remove_by_index(*convex, b);

                break;
            }
        }
    }

    return convex;
}

nav_mesh_split_edge :: (v1: Vector3, v2: Vector3, split: Vector3) -> valid: bool {
    invalidated_something := false;

    for *polygon: editor.polygons {
        if it_index == editor.polygons.count - 1 && editor.state == .CreatingPolygon {
            continue;
        }

        for i: 0..polygon.count - 1 {
            curr := polygon.points[i];
            next := polygon.points[(i + 1) % polygon.count];

            should_insert := false;

            if curr == v1 && next == v2 {
                should_insert = true;
            } else if curr == v2 && next == v1 {
                should_insert = true;
            }

            if should_insert {
                if polygon.count == MAX_VERTS {
                    invalidated_something = true;
                    break polygon;
                }

                for #v2 < a: i + 1..polygon.count {
                    polygon.points[a + 1] = polygon.points[a];
                }
                polygon.count += 1;

                polygon.points[i + 1] = split;
            }

            if !is_polygon_convex(polygon.*) {
                invalidated_something = true;
                break polygon;
            }
        }
    }

    if invalidated_something {
        for *polygon: editor.polygons {
            for i: 0..polygon.count - 2 {
                curr := polygon.points[i];
                mid := polygon.points[i + 1];
                next := polygon.points[(i + 2) % polygon.count];

                should_fix := false;
                if curr == v1 && mid == split && next == v2 {
                    should_fix = true;
                } else if curr == v2 && mid == split && next == v1 {
                    should_fix = true;
                }

                if should_fix {
                    for a: i + 1..polygon.count - 2 {
                        polygon.points[a] = polygon.points[a + 1];
                    }
                    polygon.count -= 1;
                }
            }
        }
    }

    return !invalidated_something;
}

hsv_to_rgb :: (cx: float, cy: float, cz: float) -> (r: float, g: float, b: float) {
    cx /= 360.0;
    p: Vector3;
    p.x = abs(fract(cx + 1)     * 6 - 3) - 1;
    p.y = abs(fract(cx + 2/3.0) * 6 - 3) - 1;
    p.z = abs(fract(cx + 1/3.0) * 6 - 3) - 1;

    Clamp(*p.x, 0, 1);
    Clamp(*p.y, 0, 1);
    Clamp(*p.z, 0, 1);

    p.x = lerp(1, p.x, cy);
    p.y = lerp(1, p.y, cy);
    p.z = lerp(1, p.z, cy);

    return cz * p.x, cz * p.y, cz * p.z;
}

nav_mesh_correct_polygon_point :: (point: Vector3) -> Vector3 {
    assert(editor.state == .CreatingPolygon, "Not creating polygon.\n");

    corrected_point := point;

    polygon := editor.polygons[editor.polygons.count - 1];
    polygon_normal := get_polygon_normal(polygon);
    if length(polygon_normal) == 0 {
        return corrected_point;
    }

    i := 0;
    while i < polygon.count - 1 {
        defer i += 1;

        line1 := Line3.{polygon.points[i], polygon.points[i + 1]};
        line2 := Line3.{polygon.points[i + 1], corrected_point};

        line_dir1 := normalize(line1.p2 - line1.p1);
        line_dir2 := normalize(line2.p2 - line2.p1);

        point_normal := cross(line_dir1, line_dir2);

        if dot(polygon_normal, point_normal) < -0.00001 {
            corrected_point = nearest_point(Ray3.{line1.p1, line_dir1}, corrected_point);
        }
    }

    if polygon.count >= 3 {
        // we'll also project to the final edge but inverted to prevent concave

        line1 := Line3.{polygon.points[polygon.count - 1], polygon.points[0]};
        line2 := Line3.{polygon.points[0], corrected_point};

        line_dir1 := normalize(line1.p2 - line1.p1);
        line_dir2 := normalize(line2.p2 - line2.p1);

        point_normal := -cross(line_dir1, line_dir2);

        if dot(polygon_normal, point_normal) < 0.00001 {
            corrected_point = nearest_point(line1, corrected_point);
        }
    }

    return corrected_point;
}

nav_mesh_get_snap_point :: () -> Vector3, bool {
    // if dragging {
    //     return .{}, false;
    // }

    snap_dist := ifx dragging then 0.1 else 0.4;

    ray := Ray3.{camera.position, get_forward_vector(camera)};

    min_point: Vector3;
    min_dist := FLOAT32_MAX;

    // TODO better
    for polygon: editor.polygons {
        for i: 0..polygon.count - 1 {
            point := polygon.points[i];

            // don't snap to the currently dragging point
            if dragging && dragging_points[0] == dragging_points[1] && dragging_points[0] == point {
                continue;
            }

            nearest_ray_dist := distance(nearest_point(ray, point), point);
            if nearest_ray_dist < snap_dist {
                camera_dist := distance(camera.position, point);
                if camera_dist < min_dist {
                    min_point = point;
                    min_dist = camera_dist;
                }
            }
        }
    }

    if min_dist < FLOAT32_MAX {
        return min_point, true;
    }

    return .{}, false;
}

nav_mesh_get_snap_edge :: () -> Vector3, Vector3, Vector3, bool {
    if dragging {
        return .{}, .{}, .{}, false;
    }

    ray := Ray3.{camera.position, get_forward_vector(camera)};

    min_point: Vector3;
    min_point1: Vector3;
    min_point2: Vector3;
    min_dist := FLOAT32_MAX;

    // TODO better
    for polygon: editor.polygons {
        // don't snap to currently editing polygon
        if editor.state == .CreatingPolygon && it_index == editor.polygons.count - 1 {
            continue;
        }

        for i: 0..polygon.count - 1 {
            point1 := polygon.points[i];
            point2 := polygon.points[(i + 1) % polygon.count];

            polygon_line := Line3.{point1, point2};
            if distance(polygon_line.p1, polygon_line.p2) < 0.001 {
                continue;
            }

            polygon_point := nearest_point(polygon_line, ray);
            ray_point := nearest_point(ray, polygon_point);

            nearest_ray_dist := distance(polygon_point, ray_point);
            if nearest_ray_dist < 0.4 {
                camera_dist := distance(camera.position, polygon_point);
                if camera_dist < min_dist {
                    min_point = polygon_point;
                    min_point1 = point1;
                    min_point2 = point2;
                    min_dist = camera_dist;
                }
            }
        }
    }

    if min_dist < FLOAT32_MAX {
        return min_point, min_point1, min_point2, true;
    }

    return .{}, .{}, .{}, false;
}

nav_mesh_get_snap_polygon :: (point: Vector3, distance: float, ignore_last_polygon: bool = false) -> NavMeshPolygon, bool {
    min_polygon: NavMeshPolygon;
    min_dist := FLOAT32_MAX;

    for polygon: editor.polygons {
        if ignore_last_polygon && it_index == editor.polygons.count - 1 {
            continue;
        }

        hit, dist := point_polygon_distance(point, polygon);

        if hit && dist < min_dist && dist < distance {
            min_polygon = polygon;
            min_dist = dist;
        }
    }

    if min_dist < FLOAT32_MAX {
        return min_polygon, true;
    }

    return .{}, false;
}

nav_mesh_get_snap_polygon :: () -> NavMeshPolygon, bool {
    ray := Ray3.{camera.position, get_forward_vector(camera)};

    min_polygon: NavMeshPolygon;
    min_dist := FLOAT32_MAX;

    for polygon: editor.polygons {
        hit, dist := ray_polygon_distance(ray, polygon);

        if hit && dist < min_dist {
            min_polygon = polygon;
            min_dist = dist;
        }
    }

    if min_dist < FLOAT32_MAX {
        return min_polygon, true;
    }

    return .{}, false;
}

ray_polygon_distance :: (ray: Ray3, polygon: NavMeshPolygon) -> bool, float {
    polygon_plane := get_polygon_plane(polygon);
    polygon_plane_point := intersection_simple(polygon_plane, ray);

    if is_point_inside_polygon(polygon, polygon_plane_point) {
        return true, distance(polygon_plane_point, ray.p);
    }

    return false, 0;
}

point_polygon_distance :: (point: Vector3, polygon: NavMeshPolygon) -> bool, float {
    polygon_plane := get_polygon_plane(polygon);
    polygon_plane_point := nearest_point(polygon_plane, point);

    if is_point_inside_polygon(polygon, polygon_plane_point) {
        return true, distance(polygon_plane_point, point);
    }

    return false, 0;
}

is_point_inside_polygon :: (polygon: NavMeshPolygon, point: Vector3) -> bool {
    polygon_normal := get_polygon_normal(polygon);
    if length(polygon_normal) == 0 {
        return false;
    }

    for i: 0..polygon.count - 1 {
        line1 := Line3.{polygon.points[i], polygon.points[(i + 1) % polygon.count]};
        line2 := Line3.{polygon.points[(i + 1) % polygon.count], point};

        line_dir1 := normalize(line1.p2 - line1.p1);
        line_dir2 := normalize(line2.p2 - line2.p1);

        point_normal := cross(line_dir1, line_dir2);

        if dot(polygon_normal, point_normal) < 0.00001 {
            return false;
        }
    }

    return true;
}

is_polygon_convex :: (polygon: NavMeshPolygon) -> bool {
    if polygon.count < 3 {
        return false;
    }

    polygon_normal := get_polygon_normal(polygon);

    for i: 0..polygon.count - 1 {
        line1 := Line3.{polygon.points[i], polygon.points[(i + 1) % polygon.count]};
        line2 := Line3.{polygon.points[(i + 1) % polygon.count], polygon.points[(i + 2) % polygon.count]};

        line_dir1 := normalize(line1.p2 - line1.p1);
        line_dir2 := normalize(line2.p2 - line2.p1);

        line_normal := cross(line_dir1, line_dir2);
        if dot(line_normal, polygon_normal) < -0.00001 {
            return false;
        }
    }

    return true;
}

get_polygon_plane :: (polygon: NavMeshPolygon) -> Plane3 {
    normal := get_polygon_normal(polygon);
    if length(normal) == 0 {
        return .{};
    }

    return create_plane3(polygon.points[0], normal);
}

get_polygon_normal :: (polygon: NavMeshPolygon) -> Vector3 {
    normal: Vector3;

    if polygon.count < 2 {
        return .{};
    }

    for i: 0..polygon.count - 1 {
        line1 := Line3.{polygon.points[i], polygon.points[(i + 1) % polygon.count]};
        line2 := Line3.{polygon.points[(i + 1) % polygon.count], polygon.points[(i + 2) % polygon.count]};

        line_dir1 := normalize(line1.p2 - line1.p1);
        line_dir2 := normalize(line2.p2 - line2.p1);

        line_normal := cross(line_dir1, line_dir2);
        if length(line_normal) > 0.0001 {
            normal += normalize(line_normal);
        }
    }

    if length(normal) == 0 {
        return -get_forward_vector(camera);
    }

    return normalize(normal);
}

get_concave_normal :: () -> Vector3 {
    normal: Vector3;

    if concave_polygon_points.count < 3 {
        return .{};
    }

    for i: 0..concave_polygon_points.count - 1 {
        line1 := Line3.{concave_polygon_points[i], concave_polygon_points[(i + 1) % concave_polygon_points.count]};
        line2 := Line3.{concave_polygon_points[(i + 1) % concave_polygon_points.count], concave_polygon_points[(i + 2) % concave_polygon_points.count]};

        line_dir1 := normalize(line1.p2 - line1.p1);
        line_dir2 := normalize(line2.p2 - line2.p1);

        line_normal := cross(line_dir1, line_dir2);
        normal += normalize(line_normal);
    }

    return normalize(normal);
}

nav_mesh_add_point :: (point: Vector3) {
    if editor.state != .CreatingPolygon {
        return;
    }

    if editor.polygons.count == 0 {
        assert(false, "Invalid editing polygon.\n");
        return;
    }

    polygon := *editor.polygons[editor.polygons.count - 1];
    if polygon.count == MAX_VERTS {
        return;
    }

    polygon.points[polygon.count] = point;
    polygon.count += 1;
}

nav_mesh_get_point :: () -> Vector3 {
    point := raycast_callback();

    return point;
}

dragging: bool;
dragging_points: [2] Vector3;

color_seed: u64;

camera_forward: Vector3;
raw_point: Vector3;
last_point: Vector3;
current_point: Vector3;

editor: NavMeshEditor;
raycast_callback: #type () -> Vector3;
console_print_callback: #type (text: string);
last_mouse_left: bool;
last_mouse_right: bool;
last_middle_mouse: bool;
last_shift: bool;

camera: Camera3;
mouse_down_left: bool;
mouse_down_right: bool;
middle_mouse: bool;
shift: bool;

concave_polygon: bool;
concave_polygon_points: [..] Vector3;

#import "KodaJai";
#import "JaiSerializer";

#import "Random";
#import "File";
#import "String";
