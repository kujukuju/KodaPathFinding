
nav_mesh_editor_enabled: bool;

EditorState :: enum {
    None :: 0;
    CreatingPolygon :: 1;
}

NavMeshEditor :: struct {
    state: EditorState;
    polygons: [..] NavMeshPolygon;
    next_id: s32 = 1;
}

NavMeshPolygon :: struct {
    id: s32;
    points: [16] Vector3;
    polygons: [16] s32;
    count: s32;
}

// needed features
// drag vertices with convex checking
// when you connect a polygon to an edge and it can close itself using those edge loops, it will
// while creating polygon need to be able to click on edge and create a vertex on that edge, so snap the edge in half
//

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

nav_mesh_set_inputs :: (raw_mouse_left: bool, raw_mouse_right: bool, raw_middle_mouse: bool) {
    mouse_down_left = raw_mouse_left;
    mouse_down_right = raw_mouse_right;
    middle_mouse = raw_middle_mouse;
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

    if #complete editor.state == {
        case .None;
            if delta_mouse_left {
                editor.state = .CreatingPolygon;
                polygon := array_add(*editor.polygons);
                polygon.id = editor.next_id;
                editor.next_id += 1;

                nav_mesh_add_point(current_point);
            } else if delta_middle_mouse {
                mouse_ray := Ray3.{camera.position, get_forward_vector(camera)};

                min_polygon_index := -1;
                min_polygon_dist := FLOAT32_MAX;
                for polygon: editor.polygons {
                    polygon_hit, polygon_dist := ray_polygon_distance(mouse_ray, polygon);
                    if polygon_hit && polygon_dist < min_polygon_dist {
                        min_polygon_index = it_index;
                        min_polygon_dist = polygon_dist;
                    }
                }

                if min_polygon_index >= 0 {
                    array_ordered_remove_by_index(*editor.polygons, min_polygon_index);
                }
            }

        case .CreatingPolygon;
            if delta_mouse_left {
                polygon := editor.polygons[editor.polygons.count - 1];
                if polygon.points[0] == current_point {
                    editor.state = .None;
                } else {
                    nav_mesh_add_point(current_point);
                }
            } else if delta_mouse_right {
                editor.state = .None;
            } else if delta_middle_mouse {
                editor.state = .None;
                editor.polygons.count -= 1;
            }
    }
}

nav_mesh_editor_render :: () {
    #import "GL";

    if !nav_mesh_editor_enabled {
        return;
    }

    raw_point = nav_mesh_get_point();
    current_point = raw_point;
    current_point = nav_mesh_snap_polygon_point(current_point);
    if editor.state == .CreatingPolygon {
        current_point = nav_mesh_correct_polygon_point(current_point);
    }

    nav_mesh_editor_update();

    triangles: [..] Vector3;
    triangles.allocator = temp;

    colors: [..] Vector3;
    colors.allocator = temp;

    lines: [..] Vector3;
    lines.allocator = temp;

    points: [..] Vector3;
    points.allocator = temp;

    random_seed(color_seed);

    for polygon: editor.polygons {
        hue := random_get_zero_to_one_open();
        rgb: Vector3;
        rgb.x, rgb.y, rgb.z = hsv_to_rgb(hue * 360.0, 0.34, 0.74);

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

            array_add(*points, a);
        }
    }

    if editor.state == .CreatingPolygon {
        hue := random_get_zero_to_one_open();
        rgb: Vector3;
        rgb.x, rgb.y, rgb.z = hsv_to_rgb(hue * 360.0, 0.34, 0.74);

        polygon := editor.polygons[editor.polygons.count - 1];
        a := polygon.points[0];
        b := polygon.points[polygon.count - 1];
        c := current_point;

        array_add(*triangles, a, b, c);
        array_add(*colors, rgb, rgb, rgb);
    }

    glEnable(GL_POLYGON_OFFSET_FILL);
    defer glDisable(GL_POLYGON_OFFSET_FILL);
    glPolygonOffset(-0.5, -0.5);

    {
        set_depth_func_scope(.GreaterEqual);
        set_depth_write_scope(false);

        nav_mesh_editor_draw(0.5);
    }

    {
        set_depth_func_scope(.LessEqual);

        nav_mesh_editor_draw(1.0);
    }
}

nav_mesh_editor_draw :: (alpha: float) #expand {
    draw_triangles(`triangles, `colors, color = .{1, 1, 1, alpha});

    // TODO draw lines
    i := 0;
    while i < `lines.count {
        defer i += 2;

        draw_line(`lines[i], `lines[(i + 1) % `lines.count], color = .{0, 0, 0, alpha}, width = 4.0);
    }

    {
        set_cull_face_mode_scope(.Front);
        draw_spheres(`points, .{0.07, 0.07, 0.07}, color = .{0, 0, 0, alpha});
    }
    draw_spheres(`points, .{0.05, 0.05, 0.05}, color = .{1, 1, 1, alpha});

    // if editor.state == .CreatingPolygon {
    //     polygon := editor.polygons[editor.polygons.count - 1];
    //     draw_line(polygon.points[polygon.count - 1], current_point, color = .{0, 0, 0, alpha}, width = 4.0);
    // }

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

nav_mesh_snap_polygon_point :: (point: Vector3) -> Vector3 {
    // TODO do this better
    SNAP_DISTANCE :: 0.4;
    polygon_point_dist := SNAP_DISTANCE;
    polygon_point: Vector3;
    for polygon: editor.polygons {
        for i: 0..polygon.count - 1 {
            current_point := polygon.points[i];
            current_point_dist := distance(current_point, point);

            if current_point_dist < polygon_point_dist {
                polygon_point_dist = current_point_dist;
                polygon_point = current_point;
            }
        }
    }

    if polygon_point_dist < SNAP_DISTANCE {
        return polygon_point;
    }

    return point;
}

ray_polygon_distance :: (ray: Ray3, polygon: NavMeshPolygon) -> bool, float {
    polygon_plane := get_polygon_plane(polygon);
    polygon_plane_point := intersection_simple(polygon_plane, ray);

    if is_point_inside_polygon(polygon, polygon_plane_point) {
        return true, distance(polygon_plane_point, ray.p);
    }

    return false, 0;
}

is_point_inside_polygon :: (polygon: NavMeshPolygon, point: Vector3) -> bool {
    polygon_normal := get_polygon_normal(polygon);
    if length(polygon_normal) == 0 {
        return false;
    }

    i := 0;
    while i < polygon.count {
        defer i += 1;

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

    i := 0;
    while i < polygon.count - 2 {
        defer i += 1;

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

nav_mesh_add_point :: (point: Vector3) {
    if editor.state != .CreatingPolygon {
        return;
    }

    if editor.polygons.count == 0 {
        assert(false, "Invalid editing polygon.\n");
        return;
    }

    polygon := *editor.polygons[editor.polygons.count - 1];
    if polygon.count == 16 {
        assert(false, "Invalid editing polygon.\n");
        return;
    }

    polygon.points[polygon.count] = point;
    polygon.count += 1;
}

nav_mesh_get_point :: () -> Vector3 {
    point := raycast_callback();

    return point;
}

color_seed: u64;

camera_forward: Vector3;
raw_point: Vector3;
current_point: Vector3;

editor: NavMeshEditor;
raycast_callback: #type () -> Vector3;
console_print_callback: #type (text: string);
last_mouse_left: bool;
last_mouse_right: bool;
last_middle_mouse: bool;

camera: Camera3;
mouse_down_left: bool;
mouse_down_right: bool;
middle_mouse: bool;

#import "KodaJai";
#import "JaiSerializer";

#import "Random";
#import "File";
