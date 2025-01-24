
OctreeGrid :: struct (T: Type) {
    Node :: struct (T) {
        data: T;
        parent: u16 = NULL_NODE;
        children: [8] u16 = .[NULL_NODE, NULL_NODE, NULL_NODE, NULL_NODE, NULL_NODE, NULL_NODE, NULL_NODE, NULL_NODE];
    #place children;
        xp_yp_zp: u16 = ---;
        xn_yp_zp: u16 = ---;
        xp_yn_zp: u16 = ---;
        xn_yn_zp: u16 = ---;
        xp_yp_zn: u16 = ---;
        xn_yp_zn: u16 = ---;
        xp_yn_zn: u16 = ---;
        xn_yn_zn: u16 = ---;
    }

    nodes: LinkedArray(T);
    root: u16 = NULL_NODE;
    aabb: AABB3;
    dimensions: Vector3;
    cell_size: Vector3;
    axis_count: int;
    divisions: int;
}

create_octree_grid :: ($T: Type, aabb: AABB3, divisions: int) -> OctreeGrid(T) {
    axis_count := pow(2, divisions);
    assert(axis_count * axis_count * axis_count <= NULL_NODE, "Octree grid uses u16 and your tree has more than that many cells.\n");

    tree: OctreeGrid(T);
    tree.aabb = aabb;
    tree.dimensions = aabb.upper - aabb.lower;
    tree.cell_size = aabb.upper - aabb.lower;
    tree.cell_size.x /= cast(float) axis_count;
    tree.cell_size.y /= cast(float) axis_count;
    tree.cell_size.z /= cast(float) axis_count;
    tree.axis_count = axis_count;
    tree.divisions = divisions;

    return tree;
}

create_proxy :: (tree: *OctreeGrid($T), point: Vector3, data: T) -> u16 {
    assert(tree.count_x > 0 && tree.count_y > 0 && tree.count_z > 0, "You must initialize the octree grid before using it.\n");

    // cell_x := clamp(cast(int) ((point.x - tree.aabb.lower.x) / tree.dimensions.x * tree.count_x), 0, tree.count_x - 1);
    // cell_y := clamp(cast(int) ((point.y - tree.aabb.lower.y) / tree.dimensions.y * tree.count_y), 0, tree.count_y - 1);
    // cell_z := clamp(cast(int) ((point.z - tree.aabb.lower.z) / tree.dimensions.z * tree.count_z), 0, tree.count_z - 1);

    node_index := get_or_create_node(tree, point);
    node := get_node(tree, node_index);
    node.data = data;

    return node_index;
}

get_node :: (tree: OctreeGrid($T), index: u16) -> OctreeGrid(T).Node {
    return linked_get(tree.nodes, index);
}

get_node_dimensions :: (tree: OctreeGrid) -> Vector3 {
    dimesnsions := tree.dimensions;
    dimensions.x /= cast(float) tree.axis_count;
    dimensions.y /= cast(float) tree.axis_count;
    dimensions.z /= cast(float) tree.axis_count;
    return dimensions;
}

get_node_center :: (tree:

get_or_create_node :: (tree: *OctreeGrid($T), point: Vector3) -> u16, Vector3 {
    if tree.root == NULL_NODE {
        tree.root = cast(u16) linked_add(*tree.nodes);
    }

    parent_node_index := tree.root;
    parent_aabb := tree.aabb;
    divisions := 0;
    while divisions <= tree.divisions {
        defer divisions += 1;

        if divisions == tree.divisions {
            return parent_node_index;
        }
        parent_node := linked_get(*tree.nodes, parent_node_index);

        center := parent_aabb.lower / 2.0 + parent_aabb.upper / 2.0;
        if point.z < center.z {
            if point.y < center.y {
                if point.x < center.x {
                    if parent_node.xn_yn_zn == NULL_NODE {
                        child_index, child := linked_add(*tree.nodes);
                        child.parent = parent_node_index;
                        parent_node.xn_yn_zn = cast(u16) child_index;
                    }
                    parent_node_index = parent_node.xn_yn_zn;
                    parent_aabb.upper.x = center.x;
                    parent_aabb.upper.y = center.y;
                    parent_aabb.upper.z = center.z;
                } else {
                    if parent_node.xp_yn_zn == NULL_NODE {
                        child_index, child := linked_add(*tree.nodes);
                        child.parent = parent_node_index;
                        parent_node.xp_yn_zn = cast(u16) child_index;
                    }
                    parent_node_index = parent_node.xp_yn_zn;
                    parent_aabb.lower.x = center.x;
                    parent_aabb.upper.y = center.y;
                    parent_aabb.upper.z = center.z;
                }
            } else {
                if point.x < center.x {
                    if parent_node.xn_yp_zn == NULL_NODE {
                        child_index, child := linked_add(*tree.nodes);
                        child.parent = parent_node_index;
                        parent_node.xn_yp_zn = cast(u16) child_index;
                    }
                    parent_node_index = parent_node.xn_yp_zn;
                    parent_aabb.upper.x = center.x;
                    parent_aabb.lower.y = center.y;
                    parent_aabb.upper.z = center.z;
                } else {
                    if parent_node.xp_yp_zn == NULL_NODE {
                        child_index, child := linked_add(*tree.nodes);
                        child.parent = parent_node_index;
                        parent_node.xp_yp_zn = cast(u16) child_index;
                    }
                    parent_node_index = parent_node.xp_yp_zn;
                    parent_aabb.lower.x = center.x;
                    parent_aabb.lower.y = center.y;
                    parent_aabb.upper.z = center.z;
                }
            }
        } else {
            if point.y < center.y {
                if point.x < center.x {
                    if parent_node.xn_yn_zp == NULL_NODE {
                        child_index, child := linked_add(*tree.nodes);
                        child.parent = parent_node_index;
                        parent_node.xn_yn_zp = cast(u16) child_index;
                    }
                    parent_node_index = parent_node.xn_yn_zp;
                    parent_aabb.upper.x = center.x;
                    parent_aabb.upper.y = center.y;
                    parent_aabb.lower.z = center.z;
                } else {
                    if parent_node.xp_yn_zp == NULL_NODE {
                        child_index, child := linked_add(*tree.nodes);
                        child.parent = parent_node_index;
                        parent_node.xp_yn_zp = cast(u16) child_index;
                    }
                    parent_node_index = parent_node.xp_yn_zp;
                    parent_aabb.lower.x = center.x;
                    parent_aabb.upper.y = center.y;
                    parent_aabb.lower.z = center.z;
                }
            } else {
                if point.x < center.x {
                    if parent_node.xn_yp_zp == NULL_NODE {
                        child_index, child := linked_add(*tree.nodes);
                        child.parent = parent_node_index;
                        parent_node.xn_yp_zp = cast(u16) child_index;
                    }
                    parent_node_index = parent_node.xn_yp_zp;
                    parent_aabb.upper.x = center.x;
                    parent_aabb.lower.y = center.y;
                    parent_aabb.lower.z = center.z;
                } else {
                    if parent_node.xp_yp_zp == NULL_NODE {
                        child_index, child := linked_add(*tree.nodes);
                        child.parent = parent_node_index;
                        parent_node.xp_yp_zp = cast(u16) child_index;
                    }
                    parent_node_index = parent_node.xp_yp_zp;
                    parent_aabb.lower.x = center.x;
                    parent_aabb.lower.y = center.y;
                    parent_aabb.lower.z = center.z;
                }
            }
        }
    }

    return NULL_NODE;
}

tree_free :: (tree: OctreeGrid) {
    linked_free(tree.nodes);
}

tree_reset :: (tree: *OctreeGrid) {
    linked_reset(*tree.nodes);
}

#scope_file

NULL_NODE :: U16_MAX;

#import "Basic";
#import "Math";

#import "JaiBoundingTree";
#import "JaiMath";
