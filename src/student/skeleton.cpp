
#include "../scene/skeleton.h"

Vec3 closest_on_line_segment(Vec3 start, Vec3 end, Vec3 point) {

    // TODO(Animation): Task 3

    // Return the closest point to 'point' on the line segment from start to end
    auto dir = (start - end).unit();
    float t = dot(point - end, dir) / (start - end).norm();
    auto d_s = (point - start).norm();
    auto d_e = (point - end).norm();
    if(t < 0 || t > 1) {
        if(d_s < d_e) return start;
        return end;
    } else {
        auto ans_p = end + dir * dot(point - end, dir);
        return ans_p;
    }
}

Vec4 point_to_homo(const Vec3& p) {
    return Vec4(p.x, p.y, p.z, 1.f);
}

Vec3 homo_to_point(const Vec4& p) {
    return Vec3(p.x / p.w, p.y / p.w, p.z / p.w);
}

Vec4 vec_to_homo(const Vec3& p) {
    return Vec4(p.x, p.y, p.z, 0.f);
}

Vec3 homo_to_vec(const Vec4& p) {
    return Vec3(p.x, p.y, p.z);
}

Mat4 get_translation_matrix(Vec3 extent) {
    // translation
    return Mat4::translate(extent);
}

Mat4 get_rotate_matrix(Vec3 pose) {
    // rotate
    return Mat4::euler(pose);
}

Mat4 Joint::joint_to_bind() const {

    // TODO(Animation): Task 2

    // Return a matrix transforming points in the space of this joint
    // to points in skeleton space in bind position.

    // Bind position implies that all joints have pose = Vec3{0.0f}

    // You will need to traverse the joint heirarchy. This should
    // not take into account Skeleton::base_pos
    //auto ans = get_translation_matrix(extent);
    auto ans = Mat4::I;
    auto p = this;
    while(!p->is_root()) {
        p = p->parent;
        ans = get_translation_matrix(p->extent) * ans;
    }
    return ans;
}

Mat4 Joint::joint_to_posed() const {

    // TODO(Animation): Task 2

    // Return a matrix transforming points in the space of this joint
    // to points in skeleton space, taking into account joint poses.

    // You will need to traverse the joint heirarchy. This should
    // not take into account Skeleton::base_pos
    //auto ans = get_translation_matrix(extent) * get_rotate_matrix(pose);
    auto ans = get_rotate_matrix(pose);
    auto p = this;
    while(!p->is_root()) {
        p = p->parent;
        ans = get_rotate_matrix(p->pose) * get_translation_matrix(p->extent) * ans;
    }
    return ans;
}

Vec3 Skeleton::end_of(Joint* j) {

    // TODO(Animation): Task 2

    // Return the bind position of the endpoint of joint j in object space.
    // This should take into account Skeleton::base_pos.
    return base_pos + homo_to_point(j->joint_to_bind() * point_to_homo(j->extent));
}

Vec3 Skeleton::posed_end_of(Joint* j) {

    // TODO(Animation): Task 2

    // Return the posed position of the endpoint of joint j in object space.
    // This should take into account Skeleton::base_pos.
    return base_pos + homo_to_point(j->joint_to_posed() * point_to_homo(j->extent));
}

Mat4 Skeleton::joint_to_bind(const Joint* j) const {

    // TODO(Animation): Task 2

    // Return a matrix transforming points in joint j's space to object space in
    // bind position. This should take into account Skeleton::base_pos.
    return get_translation_matrix(base_pos) * j->joint_to_bind();
}

Mat4 Skeleton::joint_to_posed(const Joint* j) const {

    // TODO(Animation): Task 2

    // Return a matrix transforming points in joint j's space to object space with
    // poses. This should take into account Skeleton::base_pos.
    return get_translation_matrix(base_pos) * j->joint_to_posed();
}

void Skeleton::find_joints(const GL::Mesh& mesh, std::vector<std::vector<Joint*>>& map) {

    // TODO(Animation): Task 3

    // Construct a mapping: vertex index -> list of joints that should effect the vertex.
    // A joint should effect a vertex if it is within Joint::radius distance of the
    // bone's line segment in bind position.

    const std::vector<GL::Mesh::Vert>& verts = mesh.verts();
    map.resize(verts.size());

    // For each i in [0, verts.size()), map[i] should contain the list of joints that
    // effect vertex i. Note that i is NOT Vert::id! i is the index in verts.

    for_joints([&](Joint* j) {
        // What vertices does joint j effect?

        auto start = homo_to_point(joint_to_bind(j) * point_to_homo(Vec3(0.f))); // world
        auto end = homo_to_point(joint_to_bind(j) * point_to_homo(j->extent));   // world

        for(size_t i = 0; i < verts.size();i ++) {    
            auto v = verts[i]; // v.pos: world pos
            auto p = closest_on_line_segment(start, end, v.pos);
            if((v.pos - p).norm() <= j->radius) {
                map[i].push_back(j);
            }
        }
    });
}

void Skeleton::skin(const GL::Mesh& input, GL::Mesh& output,
                    const std::vector<std::vector<Joint*>>& map) {

    // TODO(Animation): Task 3

    // Apply bone poses & weights to the vertices of the input (bind position) mesh
    // and store the result in the output mesh. See the task description for details.
    // map was computed by find_joints, hence gives a mapping from vertex index to
    // the list of bones the vertex should be effected by.

    // Currently, this just copies the input to the output without modification.

    std::vector<GL::Mesh::Vert> verts = input.verts();

    for(size_t i = 0; i < verts.size(); i++) {
        // Skin vertex i. Note that its position is given in object bind space.
        if(map[i].size() == 0) continue;

        auto v = verts[i];
        std::vector<float> ws;
        float sum_w = 0;
        for(auto* j : map[i]) {
            auto start = homo_to_point(joint_to_bind(j) * point_to_homo(Vec3(0.f))); // world
            auto end = homo_to_point(joint_to_bind(j) * point_to_homo(j->extent));   // world    
            auto p = closest_on_line_segment(start, end, v.pos);
            float w = 1.f / (v.pos - p).norm();
            ws.push_back(w);   
            sum_w += w;
        }
        Vec3 p = Vec3(0.f);
        Vec3 norm = Vec3(0.f);
        for(size_t l = 0; l < map[i].size(); l ++) {
            auto* j = map[i][l];
            auto trans = joint_to_posed(j) * joint_to_bind(j).inverse();
            auto p_j = homo_to_point(trans * point_to_homo(v.pos));
            auto norm_j = homo_to_point(trans * point_to_homo(v.norm)) -
                          homo_to_point(trans * point_to_homo(Vec3(0.f)));
            norm_j.normalize();
            p += ws[l] * p_j;
            norm += ws[l] * norm_j;
        }
        p /= sum_w;
        norm /= sum_w;
        verts[i].pos = p;
        verts[i].norm = norm;
    }

    std::vector<GL::Mesh::Index> idxs = input.indices();
    output.recreate(std::move(verts), std::move(idxs));
}

void Joint::compute_gradient(Vec3 target, Vec3 current) {

    // TODO(Animation): Task 2

    // Computes the gradient of IK energy for this joint and, should be called
    // recursively upward in the heirarchy. Each call should storing the result
    // in the angle_gradient for this joint.

    // Target is the position of the IK handle in skeleton space.
    // Current is the end position of the IK'd joint in skeleton space.
    auto tmp = Vec3(0.f);
    auto p = current - homo_to_point(joint_to_posed() * point_to_homo(Vec3(0.f)));
    for(int i = 0; i < 3; i++) {
        auto zero = Vec3(0.f);
        auto from = zero;
        auto to = zero;
        to[i] = 1;
        auto r =
            homo_to_point(joint_to_posed() * point_to_homo(to)) -
            homo_to_point(joint_to_posed() * point_to_homo(from));
        r.normalize();
        angle_gradient[i] += dot(cross(r, p), (current - target)); // skelection
    }
    if(!is_root()) 
        parent->compute_gradient(target, current);
}

void Skeleton::step_ik(std::vector<IK_Handle*> active_handles) {

    // TODO(Animation): Task 2

    // Do several iterations of Jacobian Transpose gradient descent for IK
    std::set<Joint*> joints;
    for(auto* handle : active_handles) {
        auto* p = handle->joint;
        while(1) {
            joints.insert(p);
            if(p->is_root()) break;
            p = p->parent;
        }
    }
    float alpha = 0.01;
    for(int i = 0; i < 100; i++) {
        for(auto* handle : active_handles) {
            handle->joint->compute_gradient(handle->target, posed_end_of(handle->joint) - base_pos);
        }
        for(auto* p : joints) {
            p->pose -= alpha * p->angle_gradient;
            p->angle_gradient = Vec3(0.0);
        }
    }
}
