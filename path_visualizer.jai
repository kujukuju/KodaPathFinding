
InstanceData :: struct {
    position: Vector3;
    color: Vector4;
}

koda_path_visualizer_update :: () {

}

koda_path_visualizer_render :: () {
    if path_draw_grid_instances {
        path_draw_grid_instances();
    }

    if path_draw_instances {

    }
}

koda_path_set_set_grid_instances :: (callback: #type (instances: [] InstanceData)) {
    set_grid_instances = callback;
}

koda_path_set_draw_grid_instances :: (callback: #type ()) {
    draw_grid_instances = callback;
}

koda_path_set_draw_instances :: (callback: #type (instances: [] InstanceData)) {
    draw_instances = callback;
}

koda_path_set_get_mouse_down_left :: (callback: #type () -> bool) {
    get_mouse_down_left = callback;
}

koda_path_set_get_mouse_down_right :: (callback: #type () -> bool) {
    get_mouse_down_right = callback;
}

koda_path_set_get_camera_position :: (callback: #type () -> Vector3) {
    get_camera_position = callback;
}

koda_path_set_get_camera_forward :: (callback: #type () -> Vector3) {
    get_camera_forward = callback;
}

#scope_file

path_set_grid_instances: (instances: [] InstanceData);
path_draw_grid_instances: ();
path_draw_instances: (instances: [] InstanceData);
path_get_mouse_down_left: () -> bool;
path_get_mouse_down_right: () -> bool;
path_get_camera_position: () -> Vector3;
path_get_camera_forward: () -> Vector3;
