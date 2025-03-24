
MAX_VERTS :: 16;

nav_mesh_editor_enabled: bool;

EditorMode :: enum {
    Polygons;
    Weights;
    Flags;
}

FlagMode :: enum {
    None;
    Set;
    AddRemove;
}

WeightMode :: enum {
    AllWeights;
    SameWeights;
}

EditorStatePolygons :: enum {
    None :: 0;
    CreatingPolygon :: 1;
}

NavMeshEditor :: struct {
    polygons: [..] NavMeshPolygon;
}

NavMeshPolygon :: struct {
    points: [MAX_VERTS] Vector3;
    count: s32;
    weight: u32 = 1;
    editor_flags: u64;
    level_flags: u64;
}

NavMeshEditorFlags :: enum_flags {
    None :: 0;
    OneWay :: 1;
}

operator == :: (a: NavMeshPolygon, b: NavMeshPolygon) -> bool {
    matches := a.count == b.count;
    for i: 0..a.count - 1 {
        matches = matches && a.points[i] == b.points[i];
    }
    return matches;
}

nav_mesh_editor_toggle :: () {
    nav_mesh_editor_enabled = !nav_mesh_editor_enabled;

    if nav_mesh_editor_enabled {
        editor_flag_values, editor_flag_names = get_enum_arrays(NavMeshEditorFlags);

        color_seed = random_get();
        nav_load();
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

nav_mesh_set_frustum :: (raw_frustum: Frustum) {
    frustum = raw_frustum;
}

nav_mesh_set_camera :: (scene_camera: Camera3) {
    camera = scene_camera;
}

nav_mesh_set_flags :: ($T: Type) {
    level_flag_values, level_flag_names = get_enum_arrays(T);
}

nav_mesh_editor_update :: () {
    raw_point = nav_mesh_get_point();
    last_point = current_point;
    current_point = raw_point;

    if #complete mode == {
        case .Polygons;
            nav_mesh_editor_update_polygons();

        case .Weights;
            nav_mesh_editor_update_weights();

        case .Flags;
            nav_mesh_editor_update_flags();
    }
}

nav_mesh_editor_update_polygons :: () {
    delta_mouse_left := mouse_down_left && !last_mouse_left;
    last_mouse_left = mouse_down_left;

    delta_mouse_right := mouse_down_right && !last_mouse_right;
    last_mouse_right = mouse_down_right;

    delta_middle_mouse := middle_mouse && !last_middle_mouse;
    last_middle_mouse = middle_mouse;

    delta_shift := shift && !last_shift;
    last_shift = shift;

    if polygon_state == .None {
        concave_polygon = shift;
        concave_polygon_points.count = 0;

        // remove degenerate polygons
        did_remove := true;
        while did_remove {
            did_remove = false;

            for polygon: editor.polygons {
                if get_polygon_area(polygon) < 0.1 {
                    did_remove = true;

                    array_ordered_remove_by_index(*editor.polygons, it_index);
                    break;
                }
            }
        }
    }

    concave_convex_triangles := nav_mesh_concave_to_triangles();
    concave_convex_polygons := nav_mesh_triangles_to_convex(concave_convex_triangles);
    array_add(*editor.polygons, ..concave_convex_polygons);
    defer editor.polygons.count -= concave_convex_polygons.count;

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

    if polygon_state == .CreatingPolygon && !concave_polygon {
        corrected_point := nav_mesh_correct_polygon_point(current_point);
        if distance(corrected_point, current_point) > 0.01 {
            current_point = nav_mesh_correct_polygon_point(raw_point);

            // no snaping
            vertex_point_found = false;
            edge_point_found = false;
        }
    }

    if dragging && !mouse_down_right {
        dragging = false;
    }

    last_polygon_state = polygon_state;
    defer if polygon_state == .None && polygon_state != last_polygon_state {
        nav_save(to_lower_copy(tprint("raw/nav/%.temp.nav", context.selected_level),, temp));
    }

    if #complete polygon_state == {
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
                        polygon_state = .CreatingPolygon;

                        if concave_polygon {
                            array_add(*concave_polygon_points, current_point);
                        } else {
                            polygon := array_add(*editor.polygons);
                            nav_mesh_add_point(current_point);
                        }
                    }
                }
            } else if delta_middle_mouse {
                polygon, polygon_found := nav_mesh_get_snap_polygon();
                if polygon_found {
                    for editor_polygon: editor.polygons {
                        if editor_polygon == polygon {
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
                            polygon_state = .None;
                        } else {
                            if concave_convex_polygons.count > 0 {
                                polygon_state = .None;
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
                        polygon_state = .None;

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

                            if valid && current_point != polygon.points[polygon.count - 1] {
                                nav_mesh_add_point(current_point);
                            }
                        }
                    }
                }
            } else if delta_mouse_right {
                if concave_polygon {
                    if concave_polygon_points.count < 3 {
                        polygon_state = .None;
                    } else {
                        if concave_convex_polygons.count > 0 {
                            polygon_state = .None;
                            editor.polygons.count += concave_convex_polygons.count;
                        }
                    }
                } else {
                    polygon_state = .None;
                    if editor.polygons[editor.polygons.count - 1].count < 3 {
                        editor.polygons.count -= 1;
                    }
                }
            } else if delta_middle_mouse {
                polygon_state = .None;

                if !concave_polygon {
                    editor.polygons.count -= 1;
                }
            }
    }
}

nav_mesh_editor_update_weights :: () {
    delta_mouse_left := mouse_down_left && !last_mouse_left;
    last_mouse_left = mouse_down_left;

    delta_mouse_right := mouse_down_right && !last_mouse_right;
    last_mouse_right = mouse_down_right;

    delta_middle_mouse := middle_mouse && !last_middle_mouse;
    last_middle_mouse = middle_mouse;

    delta_shift := shift && !last_shift;
    last_shift = shift;

    if mouse_down_left {
        polygon, polygon_found := nav_mesh_get_snap_polygon();
        if polygon_found {
            for *editor_polygon: editor.polygons {
                if editor_polygon.* == polygon {
                    editor_polygon.weight = current_weight;
                    break;
                }
            }
        }
    } else if delta_middle_mouse {
        polygon, polygon_found := nav_mesh_get_snap_polygon();
        if polygon_found {
            current_weight = polygon.weight;
        }
    }
}

nav_mesh_editor_update_flags :: () {
    delta_mouse_left := mouse_down_left && !last_mouse_left;
    last_mouse_left = mouse_down_left;

    delta_mouse_right := mouse_down_right && !last_mouse_right;
    last_mouse_right = mouse_down_right;

    delta_middle_mouse := middle_mouse && !last_middle_mouse;
    last_middle_mouse = middle_mouse;

    delta_shift := shift && !last_shift;
    last_shift = shift;

    required_flags: u64;
    if editor_flag_mode {
        required_flags = current_editor_flags;
    } else if level_flag_mode {
        required_flags = current_level_flags;
    }

    if mouse_down_left {
        polygon, polygon_found := nav_mesh_get_snap_polygon();
        if polygon_found {
            for *editor_polygon: editor.polygons {
                if editor_polygon.* == polygon {
                    if #complete editor_flag_mode == {
                        case .None;
                        case .Set;
                            editor_polygon.editor_flags = current_editor_flags;
                        case .AddRemove;
                            editor_polygon.editor_flags |= current_editor_flags;
                    }
                    if #complete level_flag_mode == {
                        case .None;
                        case .Set;
                            editor_polygon.level_flags = current_level_flags;
                        case .AddRemove;
                            editor_polygon.level_flags |= current_level_flags;
                    }
                    break;
                }
            }
        }
    } else if mouse_down_right {
        polygon, polygon_found := nav_mesh_get_snap_polygon();
        if polygon_found {
            for *editor_polygon: editor.polygons {
                if editor_polygon.* == polygon {
                    if #complete editor_flag_mode == {
                        case .None;
                        case .Set;
                        case .AddRemove;
                            editor_polygon.editor_flags &= ~current_editor_flags;
                    }
                    if #complete level_flag_mode == {
                        case .None;
                        case .Set;
                        case .AddRemove;
                            editor_polygon.level_flags &= ~current_level_flags;
                    }
                    break;
                }
            }
        }
    } else if delta_middle_mouse {
        polygon, polygon_found := nav_mesh_get_snap_polygon();
        if polygon_found {
            if editor_flag_mode {
                current_editor_flags = polygon.editor_flags;
            } else if level_flag_mode {
                current_level_flags = polygon.level_flags;
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

    random_seed(color_seed);

    glEnable(GL_POLYGON_OFFSET_FILL);
    defer glDisable(GL_POLYGON_OFFSET_FILL);
    glPolygonOffset(-0.5, -0.5);

    set_depth_write_scope(false);

    if #complete mode == {
        case .Polygons;
            nav_mesh_editor_render_polygons();

        case .Weights;
            nav_mesh_editor_render_weights();

        case .Flags;
            nav_mesh_editor_render_flags();
    }
}

nav_mesh_editor_render_polygons :: () {
    #import "GL";

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

    last_color: Vector3;

    concave_convex_polygons := nav_mesh_concave_to_triangles();
    array_add(*editor.polygons, ..concave_convex_polygons);
    defer editor.polygons.count -= concave_convex_polygons.count;

    editing_count := 0;
    if polygon_state == .CreatingPolygon {
        editing_count = max(concave_convex_polygons.count, 1);
    }

    for polygon: editor.polygons {
        hue := random_get_zero_to_one_open();
        rgb: Vector3;
        rgb.x, rgb.y, rgb.z = hsv_to_rgb(hue * 360.0, 0.34, 0.74);
        thickness := 0.02;

        if polygon_state == .CreatingPolygon && it_index >= editor.polygons.count - editing_count {
            rgb = .{1, 1, 1};
            thickness = 0.04;
        }

        if !is_polygon_convex(polygon) {
            rgb = .{1, 0, 0};
        }

        last_color = rgb;

        for i: 1..polygon.count - 2 {
            a := polygon.points[0];
            b := polygon.points[i];
            c := polygon.points[i + 1];

            center := a / 3.0 + b / 3.0 + c / 3.0;
            radius := max(distance(center, a), distance(center, b), distance(center, c));
            if !intersects(frustum, center, radius) {
                continue;
            }

            array_add(*triangles, a, b, c);
            array_add(*colors, rgb, rgb, rgb);
        }

        for i: 0..polygon.count - 1 {
            a := polygon.points[i];
            b := polygon.points[(i + 1) % polygon.count];

            center := a / 2.0 + b / 2.0;
            radius := distance(a, b);
            if !intersects(frustum, center, radius) {
                continue;
            }

            array_add(*lines, a, b);
            array_add(*thicknesses, thickness);

            array_add(*points, a);
        }
    }

    if polygon_state == .CreatingPolygon {
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
            if lines.count >= 2 {
                lines.count -= 2;
                thicknesses.count -= 1;
            }

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

    set_lines_mesh(lines, thicknesses);
    set_triangles_colors(triangles, colors);

    // if get_lines_mesh_count() != lines.count / 2 || last_polygon_state == .CreatingPolygon {
    //     set_lines_mesh(lines, thicknesses);
    // }
    // if get_triangles_colors_vertices_count() != triangles.count || last_polygon_state == .CreatingPolygon {
    //     set_triangles_colors(triangles, colors);
    // }

    glEnable(GL_POLYGON_OFFSET_FILL);
    defer glDisable(GL_POLYGON_OFFSET_FILL);
    glPolygonOffset(-0.5, -0.5);

    set_depth_write_scope(false);

    {
        set_depth_func_scope(.GreaterEqual);
        nav_mesh_editor_draw(0.25);
    }

    {
        set_depth_func_scope(.LessEqual);
        nav_mesh_editor_draw(1.0);
    }
}

nav_mesh_editor_render_weights :: () {
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

    thickness :: 0.02;

    max_weight := 0;
    if weight_mode == .AllWeights {
        for polygon: editor.polygons {
            max_weight = max(max_weight, polygon.weight);
        }
    }

    for polygon: editor.polygons {
        min_color :: Vector3.{185.0 / 255.0, 185.0 / 255.0, 185.0 / 255.0};
        max_color :: Vector3.{188.0 / 255.0, 12.0 / 255.0, 220.0 / 255.0};
        weight_color: Vector3;
        if #complete weight_mode == {
            case .AllWeights;
                weight_color = min_color + (max_color - min_color) * (cast(float) (polygon.weight - 1) / max_weight);

            case .SameWeights;
                if polygon.weight == current_weight {
                    weight_color = max_color;
                } else {
                    weight_color = min_color;
                }
        }

        for i: 1..polygon.count - 2 {
            a := polygon.points[0];
            b := polygon.points[i];
            c := polygon.points[i + 1];

            center := a / 3.0 + b / 3.0 + c / 3.0;
            radius := max(distance(center, a), distance(center, b), distance(center, c));
            if !intersects(frustum, center, radius) {
                continue;
            }

            array_add(*triangles, a, b, c);
            array_add(*colors, weight_color, weight_color, weight_color);
        }

        for i: 0..polygon.count - 1 {
            a := polygon.points[i];
            b := polygon.points[(i + 1) % polygon.count];

            center := a / 2.0 + b / 2.0;
            radius := distance(a, b);
            if !intersects(frustum, center, radius) {
                continue;
            }

            array_add(*lines, a, b);
            array_add(*thicknesses, thickness);

            array_add(*points, a);
        }
    }

    set_lines_mesh(lines, thicknesses);
    set_triangles_colors(triangles, colors);

    {
        set_depth_func_scope(.GreaterEqual);
        nav_mesh_editor_draw(0.25);
    }

    {
        set_depth_func_scope(.LessEqual);
        nav_mesh_editor_draw(1.0);
    }
}

nav_mesh_editor_render_flags :: () {
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

    thickness :: 0.02;

    flag_mode: FlagMode;
    required_flags: u64;
    if editor_flag_mode {
        flag_mode = editor_flag_mode;
        required_flags = current_editor_flags;
    } else if level_flag_mode {
        flag_mode = level_flag_mode;
        required_flags = current_level_flags;
    }

    for polygon: editor.polygons {
        none_color :: Vector3.{185.0 / 255.0, 185.0 / 255.0, 185.0 / 255.0};
        some_color :: Vector3.{214.0 / 255.0, 165.0 / 255.0, 107.0 / 255.0};
        match_color :: Vector3.{188.0 / 255.0, 12.0 / 255.0, 220.0 / 255.0};
        flag_color: Vector3;

        polygon_flags: u64;
        if editor_flag_mode {
            polygon_flags = polygon.editor_flags;
        } else if level_flag_mode {
            polygon_flags = polygon.level_flags;
        }

        if #complete flag_mode == {
            case .None;
                flag_color = none_color;
            case .Set;
                flag_color = ifx polygon_flags == required_flags then match_color else none_color;
            case .AddRemove;
                if (polygon_flags & required_flags) == required_flags {
                    flag_color = match_color;
                } else if polygon_flags & required_flags {
                    flag_color = some_color;
                } else {
                    flag_color = none_color;
                }
        }

        for i: 1..polygon.count - 2 {
            a := polygon.points[0];
            b := polygon.points[i];
            c := polygon.points[i + 1];

            center := a / 3.0 + b / 3.0 + c / 3.0;
            radius := max(distance(center, a), distance(center, b), distance(center, c));
            if !intersects(frustum, center, radius) {
                continue;
            }

            array_add(*triangles, a, b, c);
            array_add(*colors, flag_color, flag_color, flag_color);
        }

        for i: 0..polygon.count - 1 {
            a := polygon.points[i];
            b := polygon.points[(i + 1) % polygon.count];

            center := a / 2.0 + b / 2.0;
            radius := distance(a, b);
            if !intersects(frustum, center, radius) {
                continue;
            }

            array_add(*lines, a, b);
            array_add(*thicknesses, thickness);

            array_add(*points, a);
        }
    }

    set_lines_mesh(lines, thicknesses);
    set_triangles_colors(triangles, colors);

    {
        set_depth_func_scope(.GreaterEqual);
        nav_mesh_editor_draw(0.25);
    }

    {
        set_depth_func_scope(.LessEqual);
        nav_mesh_editor_draw(1.0);
    }
}

nav_mesh_editor_draw :: (alpha: float) #expand {
    draw_triangles_colors(color = .{1, 1, 1, alpha});

    draw_lines_mesh(.{0, 0, 0, alpha});
    // draw_lines_mesh(`lines, `thicknesses, color = .{0, 0, 0, alpha});

    draw_spheres(`points, .{0.05, 0.05, 0.05}, color = .{1, 1, 1, alpha});

    draw_sphere(raw_point, .{0.1, 0.1, 0.1}, color = .{0.5, 0.8, 0.82, alpha});
    draw_sphere(current_point, .{0.1, 0.1, 0.1}, color = .{0.5, 0.52, 0.8, alpha});
}

nav_mesh_editor_render_interface :: () {
    if !nav_mesh_editor_enabled {
        return;
    }

    ImGui.SetNextWindowPos(.{0, 0}, .Always);
    // ImGui.SetNextWindowSizeConstraints(.{120, 0}, .{9999, 9999});
    // ImGui.SetNextWindowSize(.{120, 0}, .Always);

    ImGui.Begin("Nav Mesh", flags = .AlwaysAutoResize);
    defer ImGui.End();

    {
        if ImGui.RadioButton("Polygons", mode == .Polygons) {
            mode = .Polygons;
        }
        if ImGui.RadioButton("Weights", mode == .Weights) {
            mode = .Weights;
        }
        if ImGui.RadioButton("Flags", mode == .Flags) {
            mode = .Flags;
        }
    }

    ImGui.Separator();

    if mode == .Weights {
        all_weights := weight_mode == .AllWeights;
        if ToggleButton("All Weights", *all_weights) {
            weight_mode = .AllWeights;
        }

        same_weights := weight_mode == .SameWeights;
        ImGui.SameLine();
        if ToggleButton("Same Weights", *same_weights) {
            weight_mode = .SameWeights;
        }

        ImGui.SetNextItemWidth(100);
        input_weight := cast(s32) current_weight;
        ImGui.InputInt("Weight", *input_weight);
        current_weight = clamp(cast(u32) input_weight, cast(u32) 1, cast(u32) U32_MAX);
    } else if mode == .Flags {
        ImGui.Text("Editor Flags");

        {
            ImGui.PushID("editor flags");
            defer ImGui.PopID();

            none_mode := editor_flag_mode == .None;
            if ToggleButton("None", *none_mode) {
                editor_flag_mode = .None;
            }
            set_mode := editor_flag_mode == .Set;
            ImGui.SameLine();
            if ToggleButton("Set", *set_mode) {
                editor_flag_mode = .Set;
                level_flag_mode = .None;
            }
            add_remove_mode := editor_flag_mode == .AddRemove;
            ImGui.SameLine();
            if ToggleButton("Add/Remove", *add_remove_mode) {
                editor_flag_mode = .AddRemove;
                level_flag_mode = .None;
            }
        }

        for i: 0..editor_flag_names.count - 1 {
            if editor_flag_values[i] == 0 {
                continue;
            }

            enabled := !!(current_editor_flags & editor_flag_values[i]);
            if ImGui.Checkbox(editor_flag_names[i].data, *enabled) {
                if enabled {
                    current_editor_flags |= editor_flag_values[i];
                } else {
                    current_editor_flags &= ~editor_flag_values[i];
                }
            }
        }

        ImGui.Separator();

        ImGui.Text("Level Flags");

        {
            ImGui.PushID("level flags");
            defer ImGui.PopID();

            none_mode := level_flag_mode == .None;
            if ToggleButton("None", *none_mode) {
                level_flag_mode = .None;
            }
            set_mode := level_flag_mode == .Set;
            ImGui.SameLine();
            if ToggleButton("Set", *set_mode) {
                level_flag_mode = .Set;
                editor_flag_mode = .None;
            }
            add_remove_mode := level_flag_mode == .AddRemove;
            ImGui.SameLine();
            if ToggleButton("Add/Remove", *add_remove_mode) {
                level_flag_mode = .AddRemove;
                editor_flag_mode = .None;
            }
        }

        for i: 0..level_flag_names.count - 1 {
            if level_flag_values[i] == 0 {
                continue;
            }

            enabled := !!(current_level_flags & level_flag_values[i]);
            if ImGui.Checkbox(level_flag_names[i].data, *enabled) {
                if enabled {
                    current_level_flags |= level_flag_values[i];
                } else {
                    current_level_flags &= ~level_flag_values[i];
                }
            }
        }
    }
}

nav_mesh_editor_get_nav_mesh :: () -> NavMesh {
    nav_mesh_remove_duplicates();
    return generate_nav_mesh(editor.polygons);
}

nav_save :: () {
    nav_save(to_lower_copy(tprint("raw/nav/%.nav", context.selected_level),, temp));
} @command

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

nav_load :: () {
    nav_load(to_lower_copy(tprint("raw/nav/%.nav", context.selected_level),, temp));
} @command

nav_load :: (name: string) {
    print("Reading %\n", name);
    file_string, success := read_entire_file(name, log_errors = false);
    if success {
        print("Done reading % %\n", name, file_string.count);
        bytes: [] u8;
        bytes.data = file_string.data;
        bytes.count = file_string.count;

        // {
        //     NavMeshEditorOld :: struct {
        //         polygons: [..] NavMeshPolygonOld;
        //     }
        //     NavMeshPolygonOld :: struct {
        //         points: [MAX_VERTS] Vector3;
        //         count: s32;
        //         weight: u32 = 1;
        //         flags: u64;
        //     }
        //     editor_old: NavMeshEditorOld;
        //     serializer_read(bytes, *editor_old);
        //     for polygon: editor_old.polygons {
        //         polygon_new: NavMeshPolygon;
        //         polygon_new.points = polygon.points;
        //         polygon_new.count = polygon.count;
        //         polygon_new.weight = polygon.weight;
        //         array_add(*editor.polygons, polygon_new);
        //     }
        // }

        editor = .{};
        serializer_read(bytes, *editor);

        nav_mesh_remove_duplicates();

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
        best_triangle: Triangle3;
        best_triangle_heuristic := 0.0;
        best_triangle_remove_index := -1;

        i := 0;
        while vertex_loop := i < remaining.count {
            defer i += 1;

            i1 := (i + 1) % remaining.count;
            i2 := (i + 2) % remaining.count;

            triangle: Triangle3;
            triangle.a = remaining[i];
            triangle.b = remaining[(i + 1) % remaining.count];
            triangle.c = remaining[(i + 2) % remaining.count];

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

            a := area(triangle);
            p := perimeter(triangle);

            heuristic := 0.0;
            if a > 0 && p > 0 {
                heuristic = sqrt(sqrt(a)) / p;
            }

            if heuristic >= best_triangle_heuristic {
                best_triangle = triangle;
                best_triangle_heuristic = heuristic;
                best_triangle_remove_index = (i + 1) % remaining.count;
            }
        }

        if best_triangle_remove_index >= 0 {
            array_add(*triangles, best_triangle);
            array_ordered_remove_by_index(*remaining, best_triangle_remove_index);
        } else {
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
        if it_index == editor.polygons.count - 1 && polygon_state == .CreatingPolygon {
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
    assert(polygon_state == .CreatingPolygon, "Not creating polygon.\n");

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

    // ray := Ray3.{camera.position, get_forward_vector(camera)};

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

            nearest_ray_dist := distance(raw_point, point);
            if nearest_ray_dist < nav_mesh_get_snap_distance(point) {
                camera_dist := distance(camera.position, point);
                if camera_dist < min_dist {
                    min_point = point;
                    min_dist = camera_dist;
                }
            }
        }
    }

    for point: concave_polygon_points {
        // don't snap to the currently dragging point
        if dragging && dragging_points[0] == dragging_points[1] && dragging_points[0] == point {
            continue;
        }

        nearest_ray_dist := distance(raw_point, point);
        if nearest_ray_dist < nav_mesh_get_snap_distance(point) {
            camera_dist := distance(camera.position, point);
            if camera_dist < min_dist {
                min_point = point;
                min_dist = camera_dist;
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

    // ray := Ray3.{camera.position, get_forward_vector(camera)};

    min_point: Vector3;
    min_point1: Vector3;
    min_point2: Vector3;
    min_dist := FLOAT32_MAX;

    // TODO better
    for polygon: editor.polygons {
        // don't snap to currently editing polygon
        if polygon_state == .CreatingPolygon && it_index == editor.polygons.count - 1 {
            continue;
        }

        for i: 0..polygon.count - 1 {
            point1 := polygon.points[i];
            point2 := polygon.points[(i + 1) % polygon.count];

            polygon_line := Line3.{point1, point2};
            if distance(polygon_line.p1, polygon_line.p2) < 0.001 {
                continue;
            }

            polygon_point := nearest_point(polygon_line, raw_point);
            // ray_point := nearest_point(ray, polygon_point);

            nearest_ray_dist := distance(polygon_point, raw_point);
            if nearest_ray_dist < nav_mesh_get_snap_distance(polygon_point) {
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

nav_mesh_get_snap_distance :: (point: Vector3) -> float {
    dist := distance(point, camera.position);

    return (1 - exp(-dist * 0.05)) * 1.0;
}

nav_mesh_remove_duplicates :: () {
    found_duplicate := true;
    while duplicate_loop := found_duplicate {
        found_duplicate = false;

        for a: 0..editor.polygons.count - 1 {
            for b: 0..editor.polygons.count - 1 {
                if a == b || editor.polygons[a].count < editor.polygons[b].count {
                    // we don't want to match with ourselves or remove a polygon with more vertices than us
                    continue;
                }

                is_duplicate := true;

                for bi: 0..editor.polygons[b].count - 1 {
                    point_b := editor.polygons[b].points[bi];

                    for ai: 0..editor.polygons[a].count - 1 {
                        point_a := editor.polygons[a].points[ai];

                        if point_a == point_b {
                            continue bi;
                        }
                    }

                    is_duplicate = false;
                    break;
                }

                if is_duplicate {
                    // if we get to this point then b is a duplicate of a
                    array_ordered_remove_by_index(*editor.polygons, b);
                    found_duplicate = true;
                    continue duplicate_loop;
                }
            }
        }
    }
}

ray_polygon_distance :: (ray: Ray3, polygon: NavMeshPolygon) -> bool, float {
    polygon_plane, polygon_plane_valid := get_polygon_plane(polygon);
    if !polygon_plane_valid return false, 0;
    polygon_plane_point := intersection_simple(polygon_plane, ray);

    if is_point_inside_polygon(polygon, polygon_plane_point) {
        return true, distance(polygon_plane_point, ray.p);
    }

    return false, 0;
}

point_polygon_distance :: (point: Vector3, polygon: NavMeshPolygon) -> bool, float {
    polygon_plane, polygon_plane_valid := get_polygon_plane(polygon);
    if !polygon_plane_valid return false, 0;
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

get_polygon_area :: (polygon: NavMeshPolygon) -> float {
    vertices: [] Vector3;
    vertices.data = polygon.points.data;
    vertices.count = polygon.count;

    return area(vertices);
}

get_polygon_plane :: (polygon: NavMeshPolygon) -> Plane3, bool {
    normal := get_polygon_normal(polygon);
    if length(normal) == 0 {
        return .{}, false;
    }

    return create_plane3(polygon.points[0], normal), true;
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
    if polygon_state != .CreatingPolygon {
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

get_enum_arrays :: ($T: Type) -> [] u64, [] string {
    enum_info :: type_info(T);
    assert(enum_info.type == .ENUM);

    baked_values, baked_names :: #run () -> [] u64, [] string {
        values := NewArray(enum_info.values.count, u64);
        names := NewArray(enum_info.names.count, string);
        for name: enum_info.names {
            names[it_index] = tprint("%\0", name);
            values[it_index] = cast(u64) enum_info.values[it_index];
        }

        return values, names;
    }();

    return baked_values, baked_names;
}

ToggleButton :: (label: *u8, selected: *bool) -> bool {
    using ImGui;

    clicked := false;

    pre_selected := selected.*;
    if pre_selected {
        active_color := GetStyleColorVec4(.ButtonActive).*;

        PushStyleColor(.Button, active_color);
        PushStyleColor(.ButtonHovered, active_color);
    }

    if Button(label) {
        selected.* = !pre_selected;
        clicked = true;
    }

    if pre_selected {
        PopStyleColor();
        PopStyleColor();
    }

    return clicked;
}

mode: EditorMode;

polygon_state: EditorStatePolygons;
last_polygon_state: EditorStatePolygons;

editor_flag_mode: FlagMode;
level_flag_mode: FlagMode;

editor_flag_values: [] u64;
editor_flag_names: [] string;

level_flag_values: [] u64;
level_flag_names: [] string;

current_weight: u32;
current_level_flags: u64;
current_editor_flags: u64;

weight_mode: WeightMode;

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

frustum: Frustum;

concave_polygon: bool;
concave_polygon_points: [..] Vector3;

#import "KodaJai";
#import "JaiSerializer";

#import "Random";
#import "File";
#import "String";
ImGui :: #import "ImGui";
