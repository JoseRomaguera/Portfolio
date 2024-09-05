#include "level/player.h"

//////////////////////////////// ENTITY SYSTEM ////////////////////////////////

#define SV_ENTITY_SYSTEM_DEF() auto sys = &sim->entity_system

static void compute_sparse_transform(v2 p, v2 s, f32 r, v2* position_, v2* size_)
{
    *position_ = p;
    
    if (ABS(r) < f32_epsilon)
    {
        *size_ = s;
    }
    else
    {
        f32 s0 = v2_length(s);
        *size_ = {s0, s0};
    }
}

static b8 entity_sparse_map_filter(Entity* entity)
{
    if (!layer_is_active(entity->render_layer))
        return FALSE;
    
    return TRUE;
}

static void entity_remove_sparse_map(Simulation* sim, Entity* entity)
{
    SV_PROFILE();
    SV_ENTITY_SYSTEM_DEF();
    
    if (!(entity->flags & EntityFlag_InSparseMap))
        return;
    
    v2 p, s;
    compute_sparse_transform(entity->position, entity->size, entity->rotation, &p, &s);
    sparse_map_remove(&sys->sparse_map, (u64)entity, p, s);
    
    entity->flags &= ~EntityFlag_InSparseMap;
}

static void entity_add_sparse_map(Simulation* sim, Entity* entity)
{
    SV_PROFILE();
    SV_ENTITY_SYSTEM_DEF();
    
    if (!entity_sparse_map_filter(entity) || (entity->flags & EntityFlag_InSparseMap))
        return;
    
    v2 p, s;
    compute_sparse_transform(entity->position, entity->size, entity->rotation, &p, &s);
    sparse_map_add(&sys->sparse_map, (u64)entity, p, s);
    
    entity->flags |= EntityFlag_InSparseMap;
}

static void entity_update_sparse_map(Simulation* sim, Entity* entity, v2 position, v2 size, f32 rotation)
{
    SV_ENTITY_SYSTEM_DEF();
    
    if (!entity_sparse_map_filter(entity))
    {
        entity_remove_sparse_map(sim, entity);
        return;
    }
    else
    {
        entity_add_sparse_map(sim, entity);
    }
    
    v2 p0, s0;
    compute_sparse_transform(entity->position, entity->size, entity->rotation, &p0, &s0);
    
    v2 p1, s1;
    compute_sparse_transform(position, size, rotation, &p1, &s1);
    
    sparse_map_update(&sys->sparse_map, (u64)entity, p0, s0, p1, s1);
}

static void entity_system_initialize(Simulation* sim)
{
    SV_ENTITY_SYSTEM_DEF();
    sys->entities = table_make<Entity>(sim->arena, 1024);
    sys->sparse_map = sparse_map_make(sim->arena, 5.f);
    
    u32 type_count = entity_type_get_count();
    sys->entity_data = arena_push_struct<InstanceAllocator>(sim->arena, type_count);
    
    foreach(i, type_count)
    {
        auto type = entity_type_get_by_index(i);
        sys->entity_data[i] = instance_allocator_create(sim->arena, type->size + sizeof(Entity*), 50);
    }
}

static void entity_system_close(Simulation* sim)
{
    SV_ENTITY_SYSTEM_DEF();
    
    {
        Entity* destroy = NULL;
        foreach_entity(sim, it)
        {
            if (destroy != NULL) entity_destroy(sim, entity_get_id(destroy), FALSE, NULL);
            destroy = it.value;
        }
        
        if (destroy != NULL) entity_destroy(sim, entity_get_id(destroy), FALSE, NULL);
    }
}

static void entity_system_update(Simulation* sim)
{
    SV_ENTITY_SYSTEM_DEF();
    if (sys->destroy_request)
    {
        Entity* destroy = NULL;
        
        foreach_entity(sim, it)
        {
            if (it.value->flags & EntityFlag_DestroyRequest)
            {
                if (destroy) entity_destroy(sim, entity_get_id(destroy), FALSE, NULL);
                destroy = it.value;
            }
        }
        
        if (destroy) entity_destroy(sim, entity_get_id(destroy), FALSE, NULL);
        sys->destroy_request = FALSE;
    }
    
#if SV_DEV
    foreach_entity(sim, it)
    {
        Entity* e = it.value;
        assert(v2_distance(e->position, e->last_position) < f32_epsilon);
        assert(v2_distance(e->size, e->last_size) < f32_epsilon);
        assert(ABS(e->rotation - e->last_rotation) < f32_epsilon);
    }
#endif
}

static u64 generate_entity_id()
{
    static u32 count = 0;
    u64 id = timer_seed();
    id = fhash64_combine(id, 0x17DA45D596F47ULL);
    id = fhash64_combine(id, (u64)count);
    return id;
}

Entity* entity_create_ex(Simulation* sim, u64 type, u64 id, const void* desc)
{
    SV_ENTITY_SYSTEM_DEF();
    
    Entity* entity = table_add(&sys->entities, id);
    
    sys->entity_count++;
    
    entity->position = v2_zero();
    entity->velocity = v2_zero();
    entity->rotation = 0.f;
    entity->size = v2_set(1.f, 1.f);
    entity->type = type;
    
    {
        auto type_struct = entity_type_get(type);
        
        {
            u32 type_index = entity_type_get_index(type);
            if (type_index != u32_max) 
            {
                Entity** raw_data = (Entity**)instance_allocator_allocate(sys->entity_data + type_index);
                *raw_data = entity;
                entity->data = raw_data + 1;
            }
        }
        
        if (type_struct->component_flags & EntityComponentFlag_Grappling)
        {
            entity->grappling = (GrapplingData*)os_heap_allocate(sizeof(GrapplingData), TRUE);
        }
    }
    
    entity_callback__create(sim, entity, desc);
    
    entity_add_sparse_map(sim, entity);
    
#if SV_DEV
    entity->last_position = entity->position;
    entity->last_size = entity->size;
    entity->last_rotation = entity->rotation;
#endif
    
    return entity;
}

u64 entity_create(Simulation* sim, u64 type, const void* desc)
{
    u64 id = generate_entity_id();
    entity_create_ex(sim, type, id, desc);
    return id;
}

void entity_destroy(Simulation* sim, u64 id, b8 dead, const void* desc)
{
    SV_ENTITY_SYSTEM_DEF();
    
    Entity* entity = entity_get(sim, id);
    if (entity == NULL) return;
    
    entity_remove_sparse_map(sim, entity);
    
    entity_callback__destroy(sim, entity, dead, desc);
    
    {
        u32 type_index = entity_type_get_index(entity->type);
        if (type_index != u32_max)
        {
            Entity** raw_data = (Entity**)entity->data;
            raw_data = raw_data - 1;
            instance_allocator_free(sys->entity_data + type_index, raw_data);
        }
    }
    
    if (table_erase(&sys->entities, id)) {
        if (sys->entity_count) sys->entity_count--;
    }
}

u64 entity_duplicate(Simulation* sim, u64 id)
{
    Entity* entity = entity_get(sim, id);
    
    if (entity == NULL)
        return 0;
    
    u64 duplicated_id = entity_create(sim, entity->type, NULL);
    Entity* duplicated = entity_get(sim, duplicated_id);
    
    duplicated->velocity = entity->velocity;
    duplicated->movement = entity->movement;
    duplicated->physics = entity->physics;
    duplicated->flags = entity->flags;
    duplicated->render_layer = entity->render_layer;
    duplicated->sort_index = entity->sort_index;
    duplicated->power_id = entity->power_id;
    
    entity_set_transform(sim, duplicated, entity->position, entity->size, entity->rotation);
    
    entity_callback__copy(sim, duplicated, entity);
    
    return duplicated_id;
}

b8 serialize_entity(Serializer* s, Entity* entity)
{
    u64 id = entity_get_id(entity);
    
    serialize_u64(s, id);
    serialize_u64(s, entity->type);
    serialize_u32(s, 6); // Version
    
    serialize_v2(s, entity->position);
    serialize_v2(s, entity->velocity);
    serialize_f32(s, entity->rotation);
    
    serialize_b8(s, (entity->flags & EntityFlag_XFlip) != 0);
    serialize_b8(s, (entity->flags & EntityFlag_YFlip) != 0);
    
    serialize_u32(s, entity->render_layer);
    serialize_u32(s, entity->sort_index);
    serialize_u32(s, entity->power_id);
    
    u32 version = 0;
    {
        const EntityTypeData* data = entity_type_get(entity->type);
        if (data)
            version = data->version;
    }
    
    serialize_u32(s, version);
    entity_callback__serialize(s, entity);
    
    return TRUE;
}

u64 deserialize_entity_and_create(Deserializer* s, Simulation* sim)
{
    u64 id;
    u64 type;
    u32 version;
    deserialize_u64(s, &id);
    deserialize_u64(s, &type);
    deserialize_u32(s, &version);
    
    Entity* entity = entity_create_ex(sim, type, id, NULL);
    
    v2 position;
    f32 rotation;
    deserialize_v2(s, &position);
    deserialize_v2(s, &entity->velocity);
    deserialize_f32(s, &rotation);
    
    b8 xflip = FALSE;
    b8 yflip = FALSE;
    
    if (version <= 4)
    {
        b8 ignore_in_editor = FALSE;
        deserialize_b8(s, &ignore_in_editor);
    }
    
    if (version >= 3)
    {
        deserialize_b8(s, &xflip);
        deserialize_b8(s, &yflip);
    }
    
    if (version > 0) deserialize_i32(s, &entity->render_layer);
    if (version > 3) deserialize_u32(s, &entity->sort_index);
    if (version >= 2) deserialize_u32(s, &entity->power_id);
    
    if (xflip) entity->flags |= EntityFlag_XFlip;
    if (yflip) entity->flags |= EntityFlag_YFlip;
    
    // Parse old render layers
    if (version <= 5)
    {
        if (entity->render_layer == OldRenderLayer_Deco) entity->render_layer = Layer_Deco;
        if (entity->render_layer == OldRenderLayer_Fore) entity->render_layer = Layer_Fore;
        if (entity->render_layer == OldRenderLayer_Back) entity->render_layer = Layer_Back;
        if (entity->render_layer == OldRenderLayer_Interior) entity->render_layer = Layer_Back;
        if (entity->render_layer == OldRenderLayer_BG0) entity->render_layer = Layer_BG0;
        if (entity->render_layer == OldRenderLayer_BG1) entity->render_layer = Layer_BG1;
        if (entity->render_layer == OldRenderLayer_None) entity->render_layer = Layer_None;
    }
    
    u32 type_version = 0;
    deserialize_u32(s, &type_version);
    
    entity_callback__deserialize(s, sim, entity, type_version);
    
    entity_set_transform(sim, entity, position, entity->size, rotation);
    
    return id;
}

void entity_power_id(Simulation* sim, u32 id, Entity* source, b8 power)
{
    if (id == 0) return;
    
    foreach_entity(sim, it)
    {
        if (it.value == source) continue;
        
        if (it.value->power_id == id) {
            entity_callback__power(sim, it.value, source, power);
        }
    }
}

Entity* entity_get(Simulation* sim, u64 id)
{
    SV_ENTITY_SYSTEM_DEF();
    return sys->entities[id];
}

u64 entity_get_id(Entity* entity)
{
    if (entity == NULL) return 0;
    return table_get_entry_hash(entity);
}

void entity_set_transform(Simulation* sim, Entity* entity, v2 position, v2 size, f32 rotation)
{
    if (entity == NULL)
        return;
    
#if SV_DEV
    Entity* e = entity;
    assert(v2_distance(e->position, e->last_position) < f32_epsilon);
    assert(v2_distance(e->size, e->last_size) < f32_epsilon);
    assert(ABS(e->rotation - e->last_rotation) < f32_epsilon);
#endif
    
    entity_update_sparse_map(sim, entity, position, size, rotation);
    entity->position = position;
    entity->size = size;
    entity->rotation = rotation;
    
#if SV_DEV
    entity->last_position = position;
    entity->last_size = size;
    entity->last_rotation = rotation;
#endif
}

void entity_set_position(Simulation* sim, Entity* entity, v2 position)
{
    if (entity == NULL) return;
    entity_set_transform(sim, entity, position, entity->size, entity->rotation);
}

void entity_set_size(Simulation* sim, Entity* entity, v2 size)
{
    if (entity == NULL) return;
    entity_set_transform(sim, entity, entity->position, size, entity->rotation);
}

void entity_set_rotation(Simulation* sim, Entity* entity, f32 rotation)
{
    if (entity == NULL) return;
    entity_set_transform(sim, entity, entity->position, entity->size, rotation);
}

Array<Entity*> entity_get_area(Arena* arena, Simulation* sim, v2 position, v2 size, EntityFilterFn* filter, const void* data)
{
    SV_ENTITY_SYSTEM_DEF();
    Array<u64> res0 = sparse_map_get_area(arena, &sys->sparse_map, position, size, (SparseMapFilterFn*)filter, data);
    Array<Entity*> res = { (Entity**)res0.data, res0.count };
    static_assert(sizeof(Entity*) == sizeof(u64));
    return res;
}

void entity_add_flags(Simulation* sim, Entity* entity, u64 flags)
{
    if (entity == NULL) return;
    
    SV_ENTITY_SYSTEM_DEF();
    
    if (flags & EntityFlag_DestroyRequest) sys->destroy_request = TRUE;
    entity->flags |= flags;
}

void entity_compute_axis_vectors(Entity* entity, v2* right_, v2* up_)
{
    if (entity == NULL) return;
    
    v2 right = v2_direction(entity->rotation);
    v2 up = v2_set(-right.y, right.x);
    
    if (entity->flags & EntityFlag_XFlip)
    {
        right.x = -right.x;
        up.x = -up.x;
    }
    if (entity->flags & EntityFlag_YFlip)
    {
        right.y = -right.y;
        up.x = -up.x;
    }
    
    if (right_) *right_ = right;
    if (up_) *up_ = up;
}

b8 entity_is_killable(u64 entity_type)
{
    if (entity_type == ENTITY_TYPE_PLAYER) return TRUE;
    if (entity_type == ENTITY_TYPE_DRONE) return TRUE;
    return FALSE;
}

b8 entity_has_dynamic_size(Entity* entity)
{
    if (entity->type == ENTITY_TYPE_LEVEL_LIMIT) return TRUE;
    if (entity->type == ENTITY_TYPE_PARTICLE_SYSTEM) return TRUE;
    
    if (entity->type == ENTITY_TYPE_PLATFORM)
    {
        PlatformData* data = (PlatformData*)entity->data;
        if (data->type == 0)
            return TRUE;
    }
    
    return FALSE;
}

f32 entity_get_min_rest_length(Entity* entity)
{
    const f32 default_length = 2.f;
    
    if (entity == NULL) return default_length;
    
    u64 t = entity->type;
    if (t == ENTITY_TYPE_ROTARY_SWITCH || t == ENTITY_TYPE_GRAVITY_GEAR) return 3.f;
    return default_length;
}

b8 entity_can_swing(Entity* entity)
{
    if (entity == NULL) return TRUE;
    if (entity->type == ENTITY_TYPE_POWERUP) return FALSE;
    return TRUE;
}

void entity_on_collision(Simulation* sim, CollisionData data)
{
    Entity* e0 = entity_get(sim, data.entity0);
    Entity* e1 = entity_get(sim, data.entity1);
    
    if (e0 != NULL && e1 != NULL)
    {
        entity_callback__collision(sim, e0, e1, &data);
        
        if (entity_get(sim, data.entity0) == NULL || entity_get(sim, data.entity1) == NULL) return;
        
        CollisionData data0 = data;
        data0.entity0 = data.entity1;
        data0.entity0_velocity = data.entity1_velocity;
        data0.entity1 = data.entity0;
        data0.entity1_velocity = data.entity0_velocity;
        data0.solve = v2_inverse(data.solve);
        
        entity_callback__collision(sim, e1, e0, &data0);
    }
}

u32 get_entity_count(Simulation* sim) {
    SV_ENTITY_SYSTEM_DEF();
    return sys->entity_count;
}

HashTable<Entity>* entity_table_get(Simulation* sim) {
    SV_ENTITY_SYSTEM_DEF();
    return &sys->entities;
}

EntityTypeIterator entity_type_iterator_begin(Simulation* sim, u64 type)
{
    EntityTypeIterator it;
    u32 type_index = entity_type_get_index(type);
    it._it = instance_allocator_iterator_begin(sim->entity_system.entity_data + type_index);
    
    if (it._it.valid) it.entity = *(Entity**)it._it.value;
    else it.entity = NULL;
    
    return it;
}

//////////////////////////////// TIMING ////////////////////////////////

f32 get_simulation_speed(Simulation* sim)
{
    f32 speed = 1.f;
    
#if ENABLE_SLOW_MOTION
    if (sim->slow_motion_timer > f32_epsilon) {
        f32 n = 1.f - sim->slow_motion_timer;
        
        f32 duration = 0.25f;
        n = f32_clamp01(n / duration);
        
        f32 in_duration = 0.1f;
        
        if (n < in_duration) {
            n = 1.f - (n / in_duration);
            speed = n;
        }
        else
        {
            n = (n - in_duration) / (1.f - in_duration);
            speed = f32_pow(n, 10.f);
        }
        
        speed = f32_lerp(0.05f, 1.f, speed);
    }
#endif
    
#if SV_DEV
    f32 m = 1.f;
    if (input_key(NULL, Key_S, InputState_Any) || !os_window_has_focus()) m = 0.1f;
    else if (input_key(NULL, Key_Left, InputState_Any)) m = 2.f;
    else if (input_key(NULL, Key_Down, InputState_Any)) m = 5.f;
    else if (input_key(NULL, Key_Right, InputState_Any)) m = 20.f;
    speed *= m;
#endif
    return speed;
}

f64 simulation_now(Simulation* sim)
{
    return (f64)sim->tick / (f64)sim->parameters.tick_rate;
}

f32 time_since(Simulation* sim, u32 tick)
{
    if (tick >= sim->tick) return 0.0f;
    return (f32)(sim->tick - tick) / (f32)sim->parameters.tick_rate;
}

f32 time_until(Simulation* sim, u32 tick, b8 relative_to_start)
{
    u32 base = 0;
    if (relative_to_start) {
        assert(sim->state != SimulationState_Waiting);
        base = sim->start_tick;
    }
    
    if (base >= tick) return 0.f;
    
    u32 ticks_ellapsed = tick - base;
    return (f32)ticks_ellapsed / (f32)sim->parameters.tick_rate;
}

#define DISABLE_ACCURATE_TIMER 0

#if !SV_DEV && DISABLE_ACCURATE_TIMER
#error Accurate timer is disabled!!!
#endif

f64 simulation_accurate_now(Simulation* sim)
{
    f64 now = simulation_now(sim);
#if DISABLE_ACCURATE_TIMER
    return now;
#endif
    return now + (f64)sim->tick_timer;
}

f32 accurate_time_since(Simulation* sim, u32 tick)
{
    f32 time = time_since(sim, tick);
    return time + calculate_fractional_tick(sim) * sim->delta_time;
}

f32 calculate_fractional_tick(Simulation* sim) {
#if DISABLE_ACCURATE_TIMER
    return 0.f;
#endif
    return f32_clamp01(sim->tick_timer / sim->delta_time);
}

v2 simulate_entity_position(Simulation* sim, Entity* entity, v2* out_velocity)
{
    v2 velocity = entity->velocity;
    
    // Player movement
    if (entity->type == ENTITY_TYPE_PLAYER)
    {
        velocity = player_calculate_total_velocity(sim);
    }
    
    f32 tdt = 1.f / (f32)sim->parameters.tick_rate;
    f32 f = calculate_fractional_tick(sim);
    f32 time = f * tdt;
    
    f32 friction = sim->parameters.air_friction;
    if (!entity->physics.in_ground) velocity += get_gravity_direction(sim) * sim->parameters.gravity * time;
    velocity.x = physics_apply_friction(velocity.x, friction, time);
    velocity.y = physics_apply_friction(velocity.y, friction, time);
    
    if (out_velocity) *out_velocity = velocity;
    
    return entity->position + velocity * time;
}

//////////////////////////////// CAMERA ////////////////////////////////

static void set_camera_state(Simulation* sim, CameraState state, f32 duration)
{
    sim->camera_state = state;
    
    if (state == CameraState_FollowPlayer) {
        Entity* player = entity_get(sim, sim->player);
        if (player) {
            sim->camera_follow_position = simulate_entity_position(sim, player);
        }
    }
    
    sim->camera_transition_position = sim->camera.position;
    sim->camera_transition_size = sim->camera.size;
    sim->camera_transition_duration = duration;
    sim->camera_transition_started_tick = sim->tick;
}

static CameraZoom get_zoom_at(Simulation* sim, v2 pos) {
    return sim->camera_zoom_base;
}

void simulation_start_camera(Simulation* sim, b8 animation)
{
    Camera* camera = &sim->camera;
    
    Entity* player = entity_get(sim, sim->player);
    v2 player_position = player ? player->position : v2_zero();
    
    v2 pos = player_position;
    f32 size = camera_zoom_get_value(get_zoom_at(sim, pos));
    
    camera->position = player_position;
    camera->size = size * 0.5f;
    set_camera_state(sim, CameraState_FollowPlayer, animation ? 1.5f : 0.f);
    
    camera_set_rotation(camera, v2_angle(get_gravity_direction(sim)) + f32_pi * 0.5f);
}

void simulation_update_matrices(Simulation* sim)
{
    Camera* camera = &sim->camera;
    v2 size = camera_get_size(camera);
    
    v2 position = camera_calculate_position(camera, (f32)simulation_accurate_now(sim));
    
    camera->inverse_view_matrix = m4_mul(m4_rotate_z(camera->rotation), m4_translate(position.x, position.y, 0.f));
    camera->view_matrix = m4_inverse(camera->inverse_view_matrix);
    camera->projection_matrix = m4_projection_orthographic(size.x * 0.5f, -size.x * 0.5f, size.y * 0.5f, -size.y * 0.5f, -100.f, 100.f);
}

static b8 camera_focus_with_velocity_entity_filter(Entity* entity, const void* data)
{
    if (entity->type == ENTITY_TYPE_PLAYER) return FALSE;
    
    if (entity->type == ENTITY_TYPE_PLATFORM)
    {
        auto platform = (PlatformData*)entity->data;
        PlatformTypeData data = platform_type_get_safe(platform->type);
        if (data.material == PlatformMaterial_Glass) return FALSE;
        return TRUE;
    }
    
    return TRUE;
}

static Entity* get_current_gear(Simulation* sim, Entity* player)
{
    SV_PROFILE();
    
    if (player == NULL) return NULL;
    
    auto gr = player->grappling;
    if (!gr->active) return NULL;
    
    f32 time = time_since(sim, gr->start_tick);
    if (time < gr->delay) return NULL;
    
    Entity* entity = entity_get(sim, gr->points[0].entity);
    if (entity == NULL) return NULL;
    if (entity->type == ENTITY_TYPE_ROTARY_SWITCH) return entity;
    if (entity->type == ENTITY_TYPE_GRAVITY_GEAR) return entity;
    return NULL;
}

static void set_camera_state_on_player_death(Simulation* sim)
{
    if (sim->state == SimulationState_Success) {
        set_camera_state(sim, CameraState_TargetExplosion, 0.5f);
    }
    else {
        set_camera_state(sim, CameraState_Death, 0.f);
    }
}

void simulation_update_camera(Simulation* sim, f32 dt)
{
    SV_PROFILE();
    
    Camera* camera = &sim->camera;
    Entity* player = entity_get(sim, sim->player);
    Entity* gear = get_current_gear(sim, player);
    
    // TODO(Jose): Why on gravity change there is another rotation animation
    // Play rotation animation
    {
        f32 current = v2_angle(get_gravity_direction(sim)) + f32_pi * 0.5f;
        
        current = f32_rotate_value(current, f32_tau);
        
        if (ABS(current - f32_rotate_value(camera->rotation1, f32_tau)) > 0.00001f)
            camera_play_rotation_animation(camera, current, 1);
    }
    
    // Change camera state
    {
        if (sim->camera_state == CameraState_FollowPlayer)
        {
            if (player == NULL) {
                set_camera_state_on_player_death(sim);
            }
            else
            {
                if (gear) {
                    set_camera_state(sim, CameraState_Gear, 0.7f);
                }
            }
        }
        else if (sim->camera_state == CameraState_Gear)
        {
            if (gear == NULL) {
                set_camera_state(sim, CameraState_FollowPlayer, 0.3f);
                if (player == NULL) set_camera_state_on_player_death(sim);
            }
        }
    }
    
    // Calculate camera position & size
    
    f32 size = sim->camera_transition_size;
    v2 position = sim->camera_transition_position;
    
    if (sim->camera_state == CameraState_FollowPlayer)
    {
        b8 follow = FALSE;
        
        PlayerData* player_data = (PlayerData*)player->data;
        if (player_data->active) follow = TRUE;
        
        if (follow)
        {
            v2 target = simulate_entity_position(sim, player);
            
            v2 to = target - sim->camera_follow_position;
            f32 distance = v2_length(to);
            
            if (distance > f32_epsilon)
            {
                f32 camera_velocity = MAX(40.f, v2_length(player->velocity) + 5.f);
                
                f32 adv = MIN(camera_velocity * dt, distance);
                
                v2 direction = to / distance;
                sim->camera_follow_position += direction * adv;
            }
        }
        
        position = sim->camera_follow_position;
        size = camera_zoom_get_value(get_zoom_at(sim, position));
    }
    else if (sim->camera_state == CameraState_Gear)
    {
        v2 player_position = simulate_entity_position(sim, player);
        
        size = camera_zoom_get_value(get_zoom_at(sim, gear->position));
        
        f32 extension = player->grappling->points[0].rest_length;
        const f32 padding = extension * 0.2f;
        size = MAX(size, (extension * 0.5f + padding) * 2.f);
        
        position = v2_lerp(player_position, gear->position, 0.5f);
    }
    else if (sim->camera_state == CameraState_TargetExplosion)
    {
        Entity* target = entity_get(sim, sim->target_entity);
        
        if (target)
        {
            f32 start_size = sim->camera_transition_size;
            f32 explosion_start_size = sim->camera_transition_size * 0.7f;
            f32 end_size = 25.f;
            
            f32 zoom_duration = 1.5f;
            f32 explosion_duration = 0.3f;
            
            f32 time = time_since(sim, sim->target_explosion.start_tick);
            
            if (time < zoom_duration)
            {
                f32 n = f32_clamp01(time / zoom_duration);
                n = f32_pow(n, 0.5f);
                size = f32_lerp(start_size, explosion_start_size, n);
            }
            else if (time - zoom_duration < explosion_duration) {
                f32 n = f32_clamp01((time - zoom_duration) / explosion_duration);
                n = f32_pow(n, 0.5f);
                size = f32_lerp(explosion_start_size, end_size, n);
            }
            else {
                size = end_size;
            }
            
            position = target->position;
        }
    }
    else if (sim->camera_state == CameraState_Death) {}
    
    // Apply transition
    f32 transition_timer = time_since(sim, sim->camera_transition_started_tick);
    if (transition_timer < sim->camera_transition_duration) {
        f32 n = transition_timer / sim->camera_transition_duration;
        n = f32_clamp01(n);
        n = math_fade_inout(n, 3.f, 0.25f);
        
        size = f32_lerp(sim->camera_transition_size, size, n);
        position = v2_lerp(sim->camera_transition_position, position, n);
    }
    
    camera->position = position;
    camera->size = size;
}

//////////////////////////////// COUNTDOWN ////////////////////////////////

#define COUNTDOWN_ENDING_ID 0x6969831FFAA

void countdown_stop_sound()
{
    audio_stop(COUNTDOWN_ENDING_ID);
}

void countdown_start(Simulation* sim, u64 entity_id, u32 seconds)
{
    if (sim->countdown_active_id != 0) countdown_stop(sim, FALSE);
    
    sim->countdown_active_id = entity_id;
    sim->countdown_end_tick = sim->tick + seconds * sim->parameters.tick_rate;
    
    if (seconds >= 7) play_global_effect(sounds->countdown_start);
}

void countdown_stop(Simulation* sim, b8 normally_finished)
{
    if (sim->countdown_active_id == 0) return;
    
    Entity* entity = entity_get(sim, sim->countdown_active_id);
    if (entity && entity->type == ENTITY_TYPE_COUNTDOWN_SWITCH)
        countdown_switch_set_value(sim, entity, FALSE);
    
    sim->countdown_end_tick = 0;
    sim->countdown_active_id = 0;
    sim->countdown_additional_ticks = 0;
    sim->countdown_trigger_intersection = 0;
    
    if (!normally_finished) countdown_stop_sound();
}

b8 countdown_is_active(Simulation* sim) {
    return sim->countdown_active_id != 0;
}

b8 countdown_kills_on_failure(Simulation* sim)
{
    // NOTE(Jose): Disabled by now
    return FALSE;
    
    if (!countdown_is_active(sim)) return FALSE;
    Entity* countdown_entity = entity_get(sim, sim->countdown_active_id);
    if (countdown_entity == NULL) return FALSE;
    CountdownSwitchData* countdown_data = (CountdownSwitchData*)countdown_entity->data;
    return countdown_data->kill_on_failure;
}

void countdown_add_ticks(Simulation* sim, i32 value)
{
    if (value == 0) return;
    
    countdown_stop_sound();
    
    assert(sim->countdown_active_id != 0);
    
    sim->countdown_additional_ticks += value;
    
    if (sim->callbacks.on_countdown_ticks_added) sim->callbacks.on_countdown_ticks_added(sim, value);
}

u32 countdown_calculate_remaining_ticks(Simulation* sim)
{
    if (!countdown_is_active(sim)) return 0;
    
    i32 end = (i32)sim->countdown_end_tick + sim->countdown_additional_ticks;
    
    u32 ticks = MAX(end - (i32)sim->tick, 0);
    return ticks;
}

static b8 get_countdown_trigger(Simulation* sim, u32 power_id, u32 unpower_id, v4* out_bounds, f32* out_rotation, v2* out_right)
{
    foreach_entity(sim, it)
    {
        if (it.value->type == ENTITY_TYPE_DOOR)
        {
            DoorData* door = (DoorData*)it.value->data;
            
            if (!door->trigger_countdown || !door->open) continue;
            
            if (it.value->power_id == power_id)
            {
                *out_bounds = bounds_set(it.value->position, it.value->size);
                *out_rotation = it.value->rotation;
                *out_right = v2_direction(*out_rotation);
                return TRUE;
            }
        }
        else if (it.value->type == ENTITY_TYPE_LASER_TRAP)
        {
            LaserTrapData* trap = (LaserTrapData*)it.value->data;
            
            if (!trap->trigger_countdown) continue;
            
            b8 active = laser_trap_ray_is_active(sim, it.value);
            if (active) continue;
            
            if (it.value->power_id == unpower_id)
            {
                *out_bounds = laser_trap_compute_ray_bounds(it.value, 0.f, FALSE);
                out_bounds->w = 3.f;
                *out_rotation = it.value->rotation;
                *out_right = v2_direction(*out_rotation + f32_pi * 0.5f);
                return TRUE;
            }
        }
    }
    
    return FALSE;
}

void countdown_update(Simulation* sim)
{
    if (!countdown_is_active(sim)) return;
    
    Entity* countdown_entity = entity_get(sim, sim->countdown_active_id);
    Entity* player = entity_get(sim, sim->player);
    
    u32 remaining_ticks = countdown_calculate_remaining_ticks(sim);
    
    // End countdown
    if (player == NULL || countdown_entity == NULL || remaining_ticks == 0 || sim->state == SimulationState_Success)
    {
        if (player != NULL)
        {
            if (countdown_kills_on_failure(sim))
                simulation_kill_player(sim, player->position, v2_set(0.f, 1.f));
        }
        
        b8 normally_finished = remaining_ticks == 0;
        countdown_stop(sim, normally_finished);
        
        return;
    }
    
    {
        u64 id = COUNTDOWN_ENDING_ID;
        
        b8 playing = audio_is_playing(id);
        
        if (!playing)
        {
            if (remaining_ticks <= sim->parameters.tick_rate * 6)
            {
                f32 time = remaining_ticks / (f32)sim->parameters.tick_rate;
                f32 offset = 6.f - time;
                play_global_effect(sounds->countdown_ending, id, 1.f, 1.f, MAX(offset, 0.f));
            }
        }
    }
    
    CountdownSwitchData* countdown_data = (CountdownSwitchData*)countdown_entity->data;
    
    u32 power_id = countdown_entity->power_id;
    u32 unpower_id = countdown_data->unpower_id;
    
    b8 intersect = FALSE;
    i32 player_side = 0;
    
    v4 trigger_bounds;
    f32 trigger_rotation;
    v2 trigger_right;
    if (get_countdown_trigger(sim, power_id, unpower_id, &trigger_bounds, &trigger_rotation, &trigger_right))
    {
        v2 trigger_points[4];
        physics_polygon_points(bounds_coord(trigger_bounds), bounds_size(trigger_bounds), trigger_rotation, bounds_coord(trigger_bounds), trigger_points);
        
        v2 player_points[4];
        physics_polygon_points(player->position, player->size, player->rotation, player->position, player_points);
        
        intersect = polygon2D_intersect(trigger_points, 4, player_points, 4, NULL);
        
        v2 player_to_center = bounds_coord(trigger_bounds) - player->position;
        
        if (!v2_is_zero(player_to_center))
        {
            player_to_center = v2_normalize(player_to_center);
            
            if (v2_dot(player_to_center, trigger_right) > 0.f) player_side = -1;
            else player_side = 1;
        }
    }
    
    if (sim->countdown_trigger_intersection == 0)
    {
        if (intersect) sim->countdown_trigger_intersection = player_side;
    }
    else
    {
        if (!intersect) 
        {
            if (player_side != sim->countdown_trigger_intersection) countdown_stop(sim, FALSE);
            sim->countdown_trigger_intersection = 0;
        }
    }
}

//////////////////////////////// TARGET EXPLOSION ////////////////////////////////

void play_target_explosion(Simulation* sim)
{
    assert(!target_explosion_is_active(sim));
    if (target_explosion_is_active(sim)) return;
    
    sim->target_explosion.playing = TRUE;
    sim->target_explosion.start_tick = sim->tick;
}

b8 target_explosion_is_active(Simulation* sim) {
    return sim->target_explosion.playing;
}

void target_explosion_update(Simulation* sim)
{
    if (!target_explosion_is_active(sim)) return;
    
    const f32 lights_off_duration = 1.5f;
    const f32 shock_wave_duration = 0.65f;
    const f32 calm_duration = 0.3f;
    const f32 total_duration = lights_off_duration + shock_wave_duration + calm_duration;
    
    f32 last_t = time_since(sim, sim->target_explosion.start_tick + 1);
    f32 t = time_since(sim, sim->target_explosion.start_tick);
    
    Entity* target = entity_get(sim, sim->target_entity);
    
    if (target)
    {
        TargetData* target_data = (TargetData*)target->data;
        
        f32 light = 0.f;
        f32 wave = 0.f;
        
        if (t < lights_off_duration) 
        {
            light = 1.f - f32_clamp01(t / lights_off_duration);
            target_data->animation_state = 1;
            target_data->animation_freq_timer = t / lights_off_duration;
        }
        else if (t - lights_off_duration < shock_wave_duration)
        {
            f32 n = (t - lights_off_duration) / shock_wave_duration;
            wave = n;
            light = f32_clamp01((n - 0.8f) / 0.2f);
        }
        else
        {
            wave = 0.999f;
            light = 1.f;
        }
        
        sim->lighting.global_light_chance = f32_pow(light, 0.5f);
        sim->lighting.shock_wave_progress = wave;
        sim->lighting.shock_wave_origin = target->position;
        sim->lighting.shock_wave_range = 25.f;
        
        // Shake
        if (t < lights_off_duration)
        {
            f32 intensity = f32_pow(t / lights_off_duration, 1.f);
            sim->camera.constant_shake = intensity * 0.3f;
        }
        
        // Explosion
        if (last_t < lights_off_duration && t >= lights_off_duration)
        {
            target_explode(sim);
            sim->camera.constant_shake = 0.f;
            simulation_play_slow_motion(sim, 0.1f);
        }
    }
    
    if (t >= total_duration) {
        sim->target_explosion.playing = FALSE;
        if (sim->callbacks.on_target_explosion_finished) sim->callbacks.on_target_explosion_finished(sim);
    }
}

b8 sample_shock_wave(Simulation* sim, v2 point)
{
    if (!target_explosion_is_active(sim)) {
        return sim->state == SimulationState_Success;
    }
    
    Entity* target = entity_get(sim, sim->target_entity);
    if (target == NULL) return FALSE;
    
    f32 distance = v2_distance(point, target->position);
    f32 wave = sim->lighting.shock_wave_progress * sim->lighting.shock_wave_range;
    
    return distance < wave;
}

//////////////////////////////// MISC ////////////////////////////////

v2_u32 calculate_current_power_ids(Simulation* sim)
{
    if (!countdown_is_active(sim)) return {};
    Entity* entity = entity_get(sim, sim->countdown_active_id);
    if (entity == NULL || entity->type != ENTITY_TYPE_COUNTDOWN_SWITCH) return {};
    CountdownSwitchData* data = (CountdownSwitchData*)entity->data;
    return { entity->power_id, data->unpower_id };
}

b8 simulation_started(Simulation* sim) {
    return sim->state >= SimulationState_Waiting;
}

b8 simulation_is_finished(Simulation* sim) {
    return sim->state == SimulationState_Death || sim->state == SimulationState_Success;
}

SimulationParameters simulation_parameters_get(u32 version)
{
    SimulationParameters p{};
    
    p.tick_rate = 120;
    p.gravity = 25.5f;
    p.air_friction = 0.2f;
    
    p.player_rotate_falling_time = 0.4f;
    p.player_rotate_moving_time = 0.2f;
    p.player_rotate_grappling_time = 0.1f;
    
    p.player_walk_fade_in_time = 0.27f;
    p.player_walk_fade_out_time = 0.15f;
    p.player_walk_speed = 8.7f;
    p.player_jump = 12.f;
    p.player_swing_force = 15.f;
    p.player_impulse_force = 25.f;
    
    return p;
}

void get_sky_limits(Simulation* sim, f32* min_, f32* max_)
{
    f32 y = sim->y_offset;
    f32 height = 35.f;
    f32 min = y - height * 0.5f;
    f32 max = y + height * 0.5f;
    
    if (min_) *min_ = min;
    if (max_) *max_ = max;
}

u64 get_current_hook_preview(Simulation* sim)
{
    if (!sim->hook_preview.valid) return 0;
    if (!sim->settings.allow_user_events) return 0;
    Entity* player = entity_get(sim, sim->player);
    if (player == NULL) return 0;
    if (player->grappling->active) return 0;
    
    return sim->hook_preview.entity;
}

void append_event(Simulation* sim, PlayerEvent event, b8 use_current_tick)
{
    if (use_current_tick) event.tick = sim->tick;
    //print_user_event_debug(event);
    array_add(&sim->events, event);
}

b8 calculate_light_noise(Simulation* sim, u64 seed, f32 threshold)
{
    f32 t = (f32)simulation_now(sim);
    f32 noise = math_perlin_noise(seed, t * 17.0f);
    return noise < threshold;
}

v4 apply_parallax_effect(Simulation* sim, v4 bounds, u32 bg_layer)
{
    Camera* camera = &sim->camera;
    
    v2 pos = bounds_coord(bounds);
    v2 size = bounds_size(bounds);
    
    f32 offset_mult0 = 0.65f;
    f32 scale_mult0 = 0.85f;
    
    f32 offset_mult1 = 1.f;
    f32 scale_mult1 = 0.75f;
    
    f32 offset_mult = (bg_layer == 1) ? offset_mult1 : offset_mult0;
    f32 scale_mult = (bg_layer == 1) ? scale_mult1 : scale_mult0;
    
    v2 offset = camera->position * offset_mult;
    pos = pos + offset;
    
    v2 min = pos - (size * 0.5f);
    v2 max = pos + (size * 0.5f);
    
    min *= scale_mult;
    max *= scale_mult;
    
    size = max - min;
    pos = min + (size * 0.5f);
    
    return bounds_set(pos, size);
}

internal_fn void print_user_event_debug(PlayerEvent e)
{
    const char* name = "Unknown";
    
    switch (e.type)
    {
        case PlayerEventType_Jump:
        name = "Jump";
        break;
        case PlayerEventType_ChangeDirection:
        name = "ChangeDirection";
        break;
        case PlayerEventType_ThrowHook:
        name = "ThrowHook";
        break;
        case PlayerEventType_ReleaseHook:
        name = "ReleaseHook";
        break;
        case PlayerEventType_ImpulseHook:
        name = "ImpulseHook";
        break;
        case PlayerEventType_ChangeSwingDirection:
        name = "ChangeSwingDirection";
        break;
        case PlayerEventType_PlayerState:
        name = "PlayerState";
        break;
        case PlayerEventType_Success:
        name = "Success";
        break;
        default:
        assert_title(FALSE, "Unknown event");
    }
    
    log_info("PlayerEvent: %s, %u\n", name, e.tick);
#if 0
    if (e.type == PlayerEventType_ChangeSwingDirection || e.type == PlayerEventType_ChangeDirection) {
        log_info("\tDir: %i\n", e.change_direction.direction);
    }
    else if (e.type == PlayerEventType_ThrowHook) {
        log_info("\tThrow Hook: %.5f, %.5f\n", e.throw_hook.position.x, e.throw_hook.position.y);
    }
#endif
}

void simulation_play_slow_motion(Simulation* sim, f32 delay)
{
    sim->slow_motion_timer = 1.f + delay;
}

////////////////////////////////////////////////////////////////

void simulation_create(Simulation* sim, u32 version)
{
    memory_zero(sim, sizeof(Simulation));
    
    sim->arena = arena_alloc(MB(1), MB(100));
    sim->version = version;
    
    entity_system_initialize(sim);
    
    sim->parameters = simulation_parameters_get(u32_max);
    
    sim->tick = 0;
    sim->tick_timer = 0.f;
    sim->delta_time = 1.f / sim->parameters.tick_rate;
    
    sim->attempt = 1;
    sim->attempts_text_position = v2_set(0.5f, 4.0f);
    
    sim->env = environment_get(EnvironmentType_None);
    
    sim->initial_gravity_direction = AxisDirection2D_Down;
    sim->gravity_direction = AxisDirection2D_Down;
    
    sim->state = SimulationState_None;
    sim->camera_zoom_base = CameraZoom_Normal;
    sim->camera = camera_initialize(sim->camera_zoom_base);
    
    sim->events = heap_array_make<PlayerEvent>(1000);
    
    sim->lighting.global_light_chance = 0.f;
    sim->lighting.shock_wave_progress = 0.f;
    
    physics_initialize(sim);
}

void simulation_destroy(Simulation* sim)
{
    entity_system_close(sim);
    
    array_free(&sim->events);
    
    arena_free(sim->arena);
}

static u64 find_throw_hook_entity_and_adjust_position(Simulation* sim, PlayerEvent* event)
{
    SCRATCH();
    assert(event->type == PlayerEventType_ThrowHook);
    
    v2 match_point = event->throw_hook.position;
    
    f32 min = f32_max;
    v2 min_point = match_point;
    u64 min_entity = 0;
    
    foreach_entity(sim, it)
    {
        Entity* entity = it.value;
        
        Array<HookPoint> points = entity_get_hook_points(scratch.arena, sim, entity);
        
        foreach(i, points.count)
        {
            v2 point = points[i].position;
            f32 distance = v2_distance(point, match_point);
            
            if (distance < min)
            {
                min = distance;
                min_point = point;
                min_entity = it.hash;
            }
        }
    }
    
    if (v2_distance(match_point, min_point) > 0.1f) {
        log_error("Throw hook event haven't found any matching point\n");
    }
    
    event->throw_hook.position = min_point;
    return min_entity;
}

void simulation_advance(Simulation* sim, b8 first, b8 last)
{
    SV_PROFILE();
    
    b8 has_player_state_event = FALSE;
    PlayerEvent player_state_event;
    
    if (sim->callbacks.on_tick_started) sim->callbacks.on_tick_started(sim, first, last);
    
    // Append events
    {
        player_controller__on_tick_started(sim, &sim->player_controller);
        
        if (sim->settings.allow_success_event)
        {
            Entity* player = entity_get(sim, sim->player);
            if (player)
            {
                PlayerData* player_data = (PlayerData*)player->data;
                if (player_data->active)
                {
                    u32 count = physics_collision_get_count(sim);
                    foreach(i, count)
                    {
                        CollisionData c = physics_collision_get(sim, i);
                        if (c.entity0 != sim->player && c.entity1 != sim->player) continue;
                        u64 target_id = (c.entity0 == sim->player) ? c.entity1 : c.entity0;
                        
                        Entity* target = entity_get(sim, target_id);
                        if (target == NULL || target->type != ENTITY_TYPE_TARGET) continue;
                        
                        PlayerEvent e{};
                        e.type = PlayerEventType_Success;
                        
                        append_event(sim, e);
                    }
                }
            }
        }
    }
    
    entity_system_update(sim);
    
    // Update input
    {
        i32 index = (i32)sim->events.count - 1;
        
        while (index >= 0)
        {
            u32 tick = sim->events[index].tick;
            if (tick < sim->tick) break;
            --index;
        }
        
        index = MAX(index, 0);
        
        while (index < sim->events.count)
        {
            PlayerEvent e = sim->events[index];
            
            if (e.tick > sim->tick) break;
            
            if (e.tick < sim->tick) {
                index++;
                continue;
            }
            
            if (sim->settings.enable_logs) print_user_event_debug(e);
            
            if (e.type != PlayerEventType_PlayerState && sim->state == SimulationState_Waiting)
            {
                if (sim->settings.enable_logs) log_info("Playing state started\n");
                simulation_set_state(sim, SimulationState_Playing);
                sim->start_tick = sim->tick;
            }
            
            if (e.type == PlayerEventType_Jump)
            {
                player_event__jump(sim);
            }
            else if (e.type == PlayerEventType_ChangeDirection)
            {
                player_event__change_move_direction(sim, e.change_direction.direction);
            }
            else if (e.type == PlayerEventType_ThrowHook)
            {
                u64 entity = find_throw_hook_entity_and_adjust_position(sim, &e);
                player_event__throw_hook(sim, e.throw_hook.position, entity);
            }
            else if (e.type == PlayerEventType_ImpulseHook)
            {
                player_event__impulse_hook(sim);
                sim->tick_last_hook_impulse = sim->tick;
            }
            else if (e.type == PlayerEventType_ReleaseHook)
            {
                player_event__release_hook(sim, FALSE);
            }
            else if (e.type == PlayerEventType_ChangeSwingDirection)
            {
                player_event__change_swing_direction(sim, e.change_swing_direction.direction);
            }
            else if (e.type == PlayerEventType_PlayerState && sim->settings.process_player_state_events)
            {
                if (sim->version == 0) {
                    player_event__player_state(sim, e.player_state.position, e.player_state.rotation, e.player_state.velocity, e.player_state.compressed);
                }
                else {
                    has_player_state_event = TRUE;
                    player_state_event = e;
                }
            }
            else if (e.type == PlayerEventType_Success)
            {
                Entity* player = entity_get(sim, sim->player);
                if (player != NULL) simulation_kill_player(sim, player->position, v2_set(0.f, 1.f), TRUE);
            }
            
            index++;
        }
    }
    
    // Update player
    {
        Entity* player = entity_get(sim, sim->player);
        if (player) 
        {
            PlayerData* player_data = (PlayerData*)player->data;
            if (player->physics.in_ground) player_data->last_tick_in_ground = sim->tick;
            
            if (sim->state == SimulationState_Playing)
            {
                player_update_movement(sim, player);
                player = entity_get(sim, sim->player);
                
                if (player)
                {
                    player_update_rotation(sim, entity_get_id(player));
                    if (last) player_update_engine_sound(sim, player);
                }
            }
        }
    }
    
    target_explosion_update(sim);
    
    countdown_update(sim);
    
    platform_move(sim);
    fan_move_objects(sim);
    
    target_spark_update_all(sim);
    laser_trap_update_all(sim);
    if (last) laser_trap_update_sounds(sim);
    rotary_switch_update_all(sim);
    gravity_gear_update_all(sim);
    platform_update_all(sim);
    
    bounce_update_previews(sim);
    drone_update_all(sim);
    
    physics_update(sim);
    
    if (has_player_state_event) {
        auto e = player_state_event;
        player_event__player_state(sim, e.player_state.position, e.player_state.rotation, e.player_state.velocity, e.player_state.compressed);
    }
    
    player_controller__on_tick_finished(sim, &sim->player_controller);
    
    if (sim->callbacks.on_tick_finished) sim->callbacks.on_tick_finished(sim);
    
    sim->tick++;
}

void simulation_update(Simulation* sim, OS_Input* input, f32 dt)
{
    sim->slow_motion_timer -= dt / 1.f;
    sim->slow_motion_timer = MAX(sim->slow_motion_timer, 0.f);
    
    dt *= get_simulation_speed(sim);
    
    player_controller_update(sim, &sim->player_controller, input);
    
    if (simulation_started(sim) && !sim->paused)
    {
        if (sim->state == SimulationState_Waiting || sim->state == SimulationState_Playing || sim->state == SimulationState_Death)
            game->capture_cursor = TRUE;
        
        sim->tick_timer += dt;
        
        b8 first = TRUE;
        
        while (sim->tick_timer >= sim->delta_time)
        {
            sim->tick_timer -= sim->delta_time;
            
            b8 last = sim->tick_timer < sim->delta_time;
            simulation_advance(sim, first, last);
            
            first = FALSE;
        }
        
        simulation_update_camera(sim, dt);
        camera_update(&sim->camera, dt);
    }
    
    player_controller_late_update(sim, &sim->player_controller);
}

void serialize_simulation(Serializer* s, Simulation* sim)
{
    SV_PROFILE();
    
    serialize_version(s, APP_VERSION);
    
    serialize_u32(s, 9); // VERSION
    serialize_f32(s, sim->y_offset);
    
    serialize_v2(s, sim->attempts_text_position);
    
    serialize_u32(s, (u32)sim->initial_gravity_direction);
    serialize_u32(s, (u32)sim->camera_zoom_base);
    
    // Entities
    {
        u64 start_cursor = s->cursor;
        serialize_u32(s, 0);
        
        u32 count = 0;
        
        foreach_entity(sim, it)
        {
            Entity* entity = it.value;
            if (serialize_entity(s, entity))
                count++;
        }
        
        u64 cursor = s->cursor;
        s->cursor = start_cursor;
        serialize_u32(s, count);
        s->cursor = cursor;
    }
    
    serialize_u64(s, sim->player);
}

void deserialize_simulation(Deserializer* s, Simulation* sim)
{
    SV_PROFILE();
    
    Version app_version;
    deserialize_version(s, &app_version);
    
    u32 version;
    
    if (version_less(app_version, version_make(0, 2, 1)))
    {
        version = 0;
    }
    else
    {
        deserialize_u32(s, &version);
    }
    
    if (version > 9) {
        log_error("Can't deserialize simulation, v%u not supported\n", version);
        s->corrupted = TRUE;
        return;
    }
    
    if (version > 0)
    {
        deserialize_f32(s, &sim->y_offset);
    }
    
    if (version > 1 && version <= 8) deserializer_ignore(s, sizeof(v2));
    if (version >= 5 && version <= 8) deserializer_ignore(s, sizeof(v2));
    
    if (version >= 7) deserialize_v2(s, &sim->attempts_text_position);
    
    if (version >= 6) deserialize_u32(s, (u32*)&sim->initial_gravity_direction);
    else sim->initial_gravity_direction = AxisDirection2D_Down;
    
    // Assign initial gravity direction
    sim->gravity_direction = sim->initial_gravity_direction;
    
    sim->camera_zoom_base = CameraZoom_Normal;
    if (version >= 8) {
        deserialize_u32(s, (u32*)&sim->camera_zoom_base);
    }
    
    if (version > 2 && version < 4)
    {
        f32 n;
        deserialize_f32(s, &n);
        deserialize_f32(s, &n);
        deserialize_f32(s, &n);
    }
    
    // Entities
    {
        u32 count;
        deserialize_u32(s, &count);
        
        foreach(i, count) {
            deserialize_entity_and_create(s, sim);
        }
    }
    
    deserialize_u64(s, &sim->player);
    
    sim->parameters = simulation_parameters_get(version);
}

void simulation_load_empty_level(Simulation* sim, b8 camera_animation)
{
    assert(sim->state == SimulationState_None);
    sim->state = SimulationState_Initialized;
}

b8 simulation_load_level(Simulation* sim, u64 level_id, u32 level_version)
{
    SCRATCH();
    String path = level_data_get_path(level_id, level_version);
    
    // Deserialize file
    {
        RawBuffer data;
        if (!os_read_asset_or_file(scratch.arena, path, &data)) {
            return FALSE;
        }
        
        Deserializer s;
        deserializer_begin_buffer(&s, data.data, data.size);
        b8 res = simulation_load_level_with_external_data(sim, level_id, level_version, &s);
        deserializer_end_buffer(&s);
        
        return res;
    }
}

b8 simulation_load_level_with_external_data(Simulation* sim, u64 level_id, u32 level_version, Deserializer* s)
{
    assert(sim->state == SimulationState_None);
    
    sim->level_id = level_id;
    sim->level_version = level_version;
    LevelData data = level_data_get(level_id);
    
    deserialize_simulation(s, sim);
    
    // Find target
    {
        foreach_entity_type(sim, it, ENTITY_TYPE_TARGET)
        {
            sim->target_entity = entity_get_id(it.entity);
            break;
        }
    }
    
    Entity* player = entity_get(sim, sim->player);
    if (player != NULL) sim->camera.position = player->position;
    
    sim->env = environment_get(data.environment_type);
    
    sim->state = SimulationState_Initialized;
    
    return TRUE;
}

b8 simulation_save_level(Simulation* sim, String path)
{
    SV_PROFILE();
    SCRATCH();
    
    Serializer s;
    serializer_begin_file(&s, 5000);
    
    serialize_simulation(&s, sim);
    
    String full_path = string_append(scratch.arena, os.assets_path, path);
    b8 res = serializer_end_file(&s, full_path);
    
    if (res) log_info("Level '%S' v%u saved\n", path, sim->level_version);
    else log_error("Can't save level '%S' v%u\n", path, sim->level_version);
    
    return res;
}

void simulation_append_events(Simulation* sim, Array<PlayerEvent> events)
{
    assert(sim->state == SimulationState_Initialized);
    foreach(i, events.count)
    {
        PlayerEvent event = events[i];
        append_event(sim, event, FALSE);
    }
}

void simulation_start(Simulation* sim, b8 camera_animation)
{
    assert(sim->state == SimulationState_Initialized);
    sim->state = SimulationState_Waiting;
    
    simulation_start_camera(sim, camera_animation);
}

void simulation_set_state(Simulation* sim, SimulationState state)
{
    if (sim->state == state) return;
    SimulationState last_state = sim->state;
    sim->state = state;
    
    if (state == SimulationState_Success)
    {
        sim->success_tick = sim->tick;
    }
    
    if (sim->callbacks.on_state_changed) sim->callbacks.on_state_changed(sim, last_state);
}

void simulation_kill_player(Simulation* sim, v2 position, v2 normal, b8 success, b8 falling)
{
    if (sim->state == SimulationState_Success || sim->state == SimulationState_Death) return;
    
    if (!success && !sim->settings.player_can_die) return;
    
#if SV_DEV
    if (!success && game->debug.invincible) return;
#endif
    
    // Target animation
    if (success)
    {
        Entity* player = entity_get(sim, sim->player);
        Entity* target = NULL;
        
        foreach_entity_type(sim, it, ENTITY_TYPE_TARGET)
        {
            Entity* entity = it.entity;
            assert_title(target == NULL, "Multiple targets are not allowed yet");
            target = entity;
        }
        
        if (target && player) entity_callback__power(sim, target, player, TRUE);
        else if (target == NULL)
        {
            log_warning("Can't play target animation, target entity not found\n");
        }
    }
    
    u64 seed = sim->player + 0x853ULL;
    entity_destroy(sim, sim->player, TRUE, NULL);
    sim->player = 0;
    
    sim->death_tick = sim->tick;
    
    {
        play_global_effect(sounds->player_death);
        camera_play_shake(&sim->camera, 0.4f, 0.5f);
        
        {
            Entity* particle = entity_get(sim, entity_create(sim, ENTITY_TYPE_PARTICLE_SYSTEM, NULL));
            entity_set_position(sim, particle, position);
            
            normal += v2_random_circle(seed, 0.5f);
            normal = v2_normalize(normal);
            particle_system_set_player_explosion(particle, normal, falling);
        }
    }
    
    if (success && !sim->settings.ignore_victory) simulation_set_state(sim, SimulationState_Success);
    else simulation_set_state(sim, SimulationState_Death);
}

void simulation_try_start_falling(Simulation* sim)
{
    if (!sim->settings.player_can_die) return;
    if (sim->state != SimulationState_Playing) return;
    
    Entity* player = entity_get(sim, sim->player);
    if (player == NULL) return;
    
    player_deactivate(sim, player);
}

void change_gravity_matching_axis(Simulation* sim, v2 direction, b8 animate_camera, b8 clockwise)
{
    v2 valid_directions[] = {
        v2_set(0.f, -1.f),
        v2_set(0.f, 1.f),
        v2_set(1.f, 0.f),
        v2_set(-1.f, 0.f),
    };
    
    u32 index = 0;
    f32 max_dot = f32_min;
    
    foreach(i, array_count(valid_directions))
    {
        f32 dot = v2_dot(valid_directions[i], direction);
        if (dot > max_dot)
        {
            max_dot = dot;
            index = i;
        }
    }
    
    AxisDirection2D dir = AxisDirection2D_Down;
    
    if (index == 0) dir = AxisDirection2D_Down;
    else if (index == 1) dir = AxisDirection2D_Up;
    else if (index == 2) dir = AxisDirection2D_Right;
    else if (index == 3) dir = AxisDirection2D_Left;
    
    change_gravity(sim, dir, animate_camera, clockwise);
}

void change_gravity(Simulation* sim, AxisDirection2D direction, b8 animate_camera, b8 clockwise)
{
    if (sim->gravity_direction == direction) return;
    
    sim->gravity_direction = direction;
    sim->gravity_animation_start_tick = sim->tick;
    
    if (animate_camera)
    {
        Camera* camera = &sim->camera;
        
        // Play rotation animation
        {
            f32 current = v2_angle(get_gravity_direction(sim)) + f32_pi * 0.5f;
            
            current = f32_rotate_value(current, f32_tau);
            
            if (ABS(current - f32_rotate_value(camera->rotation1, f32_tau)) > 0.00001f)
                camera_play_rotation_animation(camera, current, clockwise);
        }
    }
    
    door_update_all_on_gravity_change(sim);
    
    play_global_effect(sounds->gravity_powerup);
}

b8 in_gravity_rotation(Simulation* sim)
{
    Entity* player = entity_get(sim, sim->player);
    if (player == NULL) return FALSE;
    if (!player->grappling->active) return FALSE;
    auto data = (PlayerData*)player->data;
    return data->rotate_gravity.enabled;
}

v2 get_gravity_direction(Simulation* sim)
{
    return v2_from_axis_direction2D(sim->gravity_direction);
}

b8 is_gravity_sided(Simulation* sim)
{
    if (sim->gravity_direction == AxisDirection2D_Left) return TRUE;
    if (sim->gravity_direction == AxisDirection2D_Right) return TRUE;
    return FALSE;
}

static void draw_district3_clouds(Simulation* sim, v4 bounds)
{
    v2 pos = bounds_coord(bounds);
    v2 size = bounds_size(bounds);
    
    struct Cloud
    {
        Sprite sprite;
        v2 offset;
        f32 relative_height;
        f32 velocity;
        f32 space;
    };
    
    Cloud clouds[] =
    {
        { sprites->sky_district3_cloud0, v2_set(0.f, 0.f), 0.7f, 1.1f, 1.f },
        { sprites->sky_district3_cloud1, v2_set(-0.9f, 0.f), 0.6f, 1.1f, 1.f },
        { sprites->sky_district3_cloud2, v2_set(-1.f, 0.5f), 0.15f, 5.f, 1.f },
        { sprites->sky_district3_cloud2, v2_set(0.f, 0.2f), 0.13f, 4.f, 1.5f },
        
        // { sprites->sky_district3_cloud0, v2_set(-0.3f, 0.f), 0.8f, 1.1f, 0.3f },
        // { sprites->sky_district3_cloud1, v2_set(-0.6f, 0.f), 0.7f, 1.1f, 0.3f },
        // { sprites->sky_district3_cloud2, v2_set(-1.f, 0.5f), 0.24f, 5.f, 1.f },
        // { sprites->sky_district3_cloud2, v2_set(0.f, 0.2f), 0.2f, 4.f, 1.5f },
    };
    
    f32 alpha = 0.8f;
    
    f32 global_velocity = 0.003f;
    //global_velocity *= 20.f;
    
    f32 now = (f32)simulation_now(sim) + 5 * 3600.f;
    
    f32 start_x = pos.x - size.x * 0.5f;
    f32 end_x = pos.x + size.x * 0.5f;
    f32 total_width = size.x;
    f32 total_height = size.y;
    f32 bottom = pos.y - size.y * 0.5f;
    
    foreach(i, array_count(clouds))
    {
        Cloud cloud = clouds[i];
        
        f32 time_offset = 0.f;//f32_random(0x6534 + i * 0x392);
        
        Sprite sprite = cloud.sprite;
        GPUImage* image = sprite_get_image(sprite);
        f32 aspect = sprite_compute_aspect(sprite, image);
        
        f32 height = cloud.relative_height * total_height;
        f32 width = height * aspect;
        
        f32 p = (now * (cloud.velocity * global_velocity)) + time_offset;
        
        f32 x = f32_lerp(start_x, end_x, p);
        
        v2 size = v2_set(width, height);
        v2 pos = v2_set(x + size.x * 0.5f, bottom + size.y * 0.5f);
        
        // Apply offset
        pos += cloud.offset * v2_set(total_width, total_height);
        
        // Draw
        f32 space = cloud.space * size.x;
        while (pos.x - size.x * 0.5f > start_x) pos.x -= size.x + space;
        
        while (pos.x - size.x * 0.5f <= end_x)
        {
            renderer_batch_set_image(image);
            renderer_batch_append(pos, size, 0.f, sprite.texcoord, 0.f, 1.f, color_white(alpha));
            
            pos.x += size.x + space;
        }
    }
}

static v4 draw_background(Simulation* sim)
{
    SV_PROFILE();
    
    renderer_batch_start();
    renderer_batch_set_shader(RendererLayer_Sky);
    
#if SV_DEV
    renderer_batch_set_image(NULL);
    renderer_batch_append(v2_set(0.f, 0.f), v2_set(999999.f, 999999.f), 0.f, v4_zero(), 0.f, 1.f, color_red());
#endif
    
    Camera* c = &sim->camera;
    EnvironmentData env = sim->env;
    
    Sprite background{};
    if (env.type == EnvironmentType_Kantan) background = sprites->sky_district1;
    if (env.type == EnvironmentType_Moonfall) background = sprites->sky_district2;
    if (env.type == EnvironmentType_Staubland) background = sprites->sky_district3;
    if (env.type == EnvironmentType_Astun) background = sprites->sky_district4;
    
    GPUImage* background_image = sprite_get_image(background);
    if (background_image == NULL) {
        
        if (env.type == EnvironmentType_None) {
            renderer_batch_set_image(NULL);
            renderer_batch_append(v2_zero(), v2_set(100000, 100000), 0.f, {}, 0.f, 1.f, color_gray(0.7f));
            renderer_batch_draw();
        }
        
        return v4_zero();
    }
    
    f32 background_aspect = sprite_compute_aspect(background, background_image);
    
    v2 cam_size = camera_get_size(c);
    f32 cam_aspect = cam_size.x / cam_size.y;
    v2 size;
    
    f32 bg_def_size = sim->env.default_max_size;
    size = v2_set(bg_def_size * background_aspect, bg_def_size);
    
    if (size.x < cam_size.x || size.y < cam_size.y)
    {
        if (cam_aspect < background_aspect) size = v2_set(cam_size.y * background_aspect, cam_size.y);
        else size = v2_set(cam_size.x, cam_size.x / background_aspect);
    }
    
    // TEST
    //c->size = f32_lerp(10, 40, f32_sin01((f32)timer_now()));
    //size *= 0.7f;
    
    v2 pos = c->position;
    
    v4 bounds = v4_set(pos.x, pos.y, size.x, size.y);
    
    renderer_batch_set_image(background_image);
    renderer_batch_append(pos, size, 0.f, v4_set(0.0f, 0.0f, 1.f, 1.f), 0.f, 1.f, color_white());
    {
        f32 pixel_height = 1.f / (f32)background_image->height;
        pixel_height *= 0.5f;
        renderer_batch_append(pos + v2_set(0.f, size.y), size, 0.f, v4_set(0.0f, 0.0f, 1.f, pixel_height), 0.f, 1.f, color_white());
        renderer_batch_append(pos + v2_set(0.f, -size.y), size, 0.f, v4_set(0.0f, 1.f, 1.f, 1.0f - pixel_height), 0.f, 1.f, color_white());
        
        pixel_height = 1.f / (f32)background_image->width;
        pixel_height *= 0.5f;
        renderer_batch_append(pos + v2_set(size.x, 0.f), size, 0.f, v4_set(0.0f, 0.0f, pixel_height, 1.f), 0.f, 1.f, color_white());
        renderer_batch_append(pos + v2_set(-size.x, 0.f), size, 0.f, v4_set(1.0f, 0.0f, 1.f - pixel_height, 1.f), 0.f, 1.f, color_white());
    }
    
    if (env.type == EnvironmentType_Kantan)
    {
        Sprite clouds = sprites->sky_district1_clouds;
        GPUImage* image = sprite_get_image(clouds);
        
        f32 t = (f32)simulation_now(sim) / 70.f;
        t = f32_rotate_value(t, 1.f);
        
        pos.x += t * size.x;
        size.x *= 3.f;
        
        renderer_batch_set_image(image);
        renderer_batch_append(pos, size, 0.f, v4_set(-1.f, 0.f, 2.f, 1.f), 0.f, 1.f, color_white());
    }
    if (env.type == EnvironmentType_Staubland) draw_district3_clouds(sim, bounds);
    
    renderer_batch_draw();
    
    return bounds;
}

static void draw_editor_grid(Simulation* sim)
{
    SV_PROFILE();
    CommandList cmd = game->cmd;
    Camera* c = &sim->camera;
    
    imrend_begin(sim, cmd);
    
    v2 size = camera_get_size(c);
    b8 have_pixel_grid = FALSE;
    
    // Pixel grid
    {
        f32 appear_size = 9.f;
        
        f32 alpha = 1.f - f32_clamp01((c->size - 1.f) / appear_size);
        
        const u32 pixels = 64;
        
        if (alpha > f32_epsilon)
        {
            have_pixel_grid = TRUE;
            alpha *= 0.5f; // The pixels are rasterized twice
            
            alpha *= 0.3f;
            
            Color colors[] = 
            {
                color_red(alpha),
                color_green(alpha),
            };
            
            u32 color_index = 0;
            
            v2 center = c->position;
            
            f32 grid_size = 1.f / pixels;
            
            v2 begin = center - size * 0.5f;
            v2 end = begin + size;
            
            color_index = (i32)(begin.y / grid_size);
            for (f32 y = (i32)(begin.y / grid_size) * grid_size; y < end.y; y += grid_size) {
                Color color = colors[color_index++ % array_count(colors)];
                v2 c0 = { begin.x, y };
                v2 c1 = { end.x, y + grid_size };
                v2 s = c1 - c0;
                imrend_draw_quad(c0 + s * 0.5f, s, 0.f, color, cmd);
            }
            color_index = (i32)(begin.x / grid_size);
            for (f32 x = (i32)(begin.x / grid_size) * grid_size; x < end.x; x += grid_size) {
                Color color = colors[color_index++ % array_count(colors)];
                v2 c0 = { x, begin.y };
                v2 c1 = { x + grid_size, end.y };
                v2 s = c1 - c0;
                imrend_draw_quad(c0 + s * 0.5f, s, 0.f, color, cmd);
            }
        }
    }
    
    // Unit grid
    {
        f32 size0 = 100.f;
        f32 size1 = 150.f;
        
        f32 alpha = math_fade_inout((c->size - size0) / (size1 - size0), 3.f, 0.5f);
        alpha = 1.f - f32_clamp01(alpha);
        
        if (have_pixel_grid) imrend_line_width(5.f, cmd);
        else imrend_line_width(1.f, cmd);
        
        Color color = color_gray(100 / 255.f, alpha * (100.f / 255.f));
        imrend_draw_orthographic_grip(c->position, v2_zero(), size, v2_set(1.f, 1.f), color, cmd);
    }
    
    // Center lines
    {
        Color color = color_gray(100 / 255.f, 100 / 255.f);
        imrend_line_width(2.0f, cmd);
        imrend_draw_line(v3_set(c->position.x - size.x, 0.f, 0.f), v3_set(c->position.x + size.x, 0.f, 0.f), color, cmd);
        imrend_draw_line(v3_set(0.f, c->position.y - size.y, 0.f), v3_set(0.f, c->position.y + size.y, 0.f), color, cmd);
    }
    
    imrend_end(cmd);
}

static void draw_chain(Simulation* sim)
{
    SV_PROFILE();
    
    CommandList cmd = game->cmd;
    Camera* c = &sim->camera;
    
    renderer_batch_start();
    renderer_batch_set_shader(RendererLayer_Foreground);
    
    Entity* player = entity_get(sim, sim->player);
    
    if (player == NULL) return;
    
    PhysicsData* ph = &player->physics;
    GrapplingData* gr = player->grappling;
    
    v2 hook_position = v2_zero();
    f32 hook_rotation = 0.f;
    f32 hook_size = 1.1f;
    u32 hook_state = 0;
    
    if (gr->active)
    {
        f32 absolute_time = accurate_time_since(sim, gr->start_tick);
        f32 time = MIN(absolute_time / gr->delay, 1.f);
        
        // Compute final hook position
        {
            GrapplingPoint* point = gr->points + 0;
            Entity* entity = entity_get(sim, point->entity);
            
            if (entity != NULL)
            {
                hook_state = 2;
                hook_position = grappling_position(sim, point);
                hook_rotation = gr->launch_angle;
                
                v2 dir = v2_direction(hook_rotation);
                hook_position = v2_add(hook_position, v2_mul_scalar(dir, hook_size * 0.2f));
            }
        }
        
        v2 origin = player_calculate_chain_origin(sim, player);
        f32 total_length = 0.0f;
        
        // Compute total length
        for (i32 i = gr->point_count - 1; i >= 0; --i)
        {
            GrapplingPoint* point = gr->points + i;
            v2 target = grappling_position(sim, point);
            
            total_length += v2_distance(target, origin);
            
            origin = target;
        }
        
        origin = player_calculate_chain_origin(sim, player);
        
        f32 total_progress = total_length * time;
        f32 progress_count = 0.0f;
        
        for (i32 i = gr->point_count - 1; i >= 0; --i)
        {
            b8 is_first = i == gr->point_count - 1;
            b8 is_last  = i == 0;
            
            GrapplingPoint* point = gr->points + i;
            
            v2 target = grappling_position(sim, point);
            
            f32 amplitude = 0.f;
            
            v2 direction = v2_sub(target, origin);
            f32 relative_length = v2_length(direction);
            direction = v2_normalize(direction);
            v2 perp = v2_perpendicular(direction);
            
            f32 progress0 = progress_count;
            f32 progress1 = progress0 + relative_length;
            f32 progress = f32_clamp01((total_progress - progress0) / (progress1 - progress0));
            
            f32 progress_length = progress * relative_length;
            
            // Compute amplitude
            if (is_first)
            {
                // By delay
                {
                    f32 a = MIN(absolute_time / (gr->delay - 0.05f), 1.f);
                    a = (1.f - f32_pow(a, 2.f));
                    amplitude = MAX(a, amplitude);
                }
                
                // TODO
                const f32 GRAPPLING_MIN_EXTENSION = 3.f;
                
                // By extension
                {
                    f32 extension = -MIN(relative_length - point->rest_length, 0.f);
                    f32 a = f32_clamp01(extension / GRAPPLING_MIN_EXTENSION) * 0.5f;
                    amplitude = MAX(a, amplitude);
                }
                
                const f32 min_distance = 3.f;
                const f32 max_distance = 15.f;
                
                f32 mul = f32_clamp01((point->rest_length - min_distance) / (max_distance - min_distance));
                amplitude *= f32_lerp(GRAPPLING_MIN_AMPLITUDE, GRAPPLING_MAX_AMPLITUDE, mul);
                
                amplitude *= 0.5f;
            }
            
            if (is_last && time < 1.f - f32_epsilon)
            {
                hook_state = 1;
                
                f32 l0 = progress_length - 0.01f;
                f32 l1 = progress_length;
                
                f32 n0 = chain_noise(l0, relative_length, time) * amplitude;
                f32 n1 = chain_noise(l1, relative_length, time) * amplitude;
                
                v2 p0 = v2_add(origin, v2_mul_scalar(direction, l0));
                p0 = v2_add(p0, v2_mul_scalar(perp, n0));
                
                v2 p1 = v2_add(origin, v2_mul_scalar(direction, l1));
                p1 = v2_add(p1, v2_mul_scalar(perp, n1));
                
                v2 dir = v2_normalize(v2_sub(p1, p0));
                
                v2 p = v2_add(p1, v2_mul_scalar(dir, hook_size * 0.5f));
                
                hook_position = p;
                hook_rotation = v2_angle(dir);
            }
            
            renderer_draw_chain(origin, target, time, progress_length, amplitude);
            
            origin = target;
            progress_count += relative_length;
        }
    }
    else if (gr->last_release_position_count)
    {
        f32 total_time = accurate_time_since(sim, gr->last_release_tick);
        
        v2* points = gr->last_release_positions;
        u32 point_count = gr->last_release_position_count;
        
        v2 origin = player_calculate_chain_origin(sim, player);
        
        f32 total_length = 0.0f;
        
        // Compute total length
        for (i32 i = point_count - 1; i >= 0; --i)
        {
            v2 target = points[i];
            total_length += v2_distance(target, origin);
            origin = target;
        }
        
        origin = player_calculate_chain_origin(sim, player);
        
        f32 max_time = 0.06f + f32_clamp01(total_length / 20.f) * 0.5f;
        f32 time = 1.f - MIN(total_time / max_time, 1.f);
        
        f32 total_progress = total_length * time;
        f32 length_count = 0.f;
        
        for (i32 i = point_count - 1; i >= 0; --i)
        {
            v2 target = points[i];
            f32 relative_length = v2_distance(target, origin);
            
            f32 progress0 = length_count;
            f32 progress1 = progress0 + relative_length;
            f32 progress = f32_clamp01((total_progress - progress0) / (progress1 - progress0));
            
            f32 relative_time = progress0 + total_time;
            f32 progress_length = progress * relative_length;
            
            const f32 min_distance = 3.f;
            const f32 max_distance = 15.f;
            
            f32 amplitude = 0.111111111111111111111111111f;
            
            if (progress < 1.f - f32_epsilon)
            {
                f32 mul = f32_clamp01((relative_length - min_distance) / (max_distance - min_distance));
                amplitude = f32_lerp(GRAPPLING_MIN_AMPLITUDE, GRAPPLING_MAX_AMPLITUDE, mul);
                amplitude *= 0.3f;
            }
            
            if (progress > f32_epsilon && progress < 1.f + f32_epsilon)
            {
                hook_state = 1;
                
                v2 direction = v2_normalize(v2_sub(target, origin));
                v2 perp = v2_perpendicular(direction);
                
                f32 l0 = progress_length - 0.01f;
                f32 l1 = progress_length;
                
                f32 n0 = chain_noise(l0, relative_length, relative_time) * amplitude;
                f32 n1 = chain_noise(l1, relative_length, relative_time) * amplitude;
                
                v2 p0 = v2_add(origin, v2_mul_scalar(direction, l0));
                p0 = v2_add(p0, v2_mul_scalar(perp, n0));
                
                v2 p1 = v2_add(origin, v2_mul_scalar(direction, l1));
                p1 = v2_add(p1, v2_mul_scalar(perp, n1));
                
                v2 dir = v2_normalize(v2_sub(p1, p0));
                
                v2 p = v2_add(p1, v2_mul_scalar(dir, hook_size * 0.5f));
                
                hook_position = p;
                hook_rotation = v2_angle(dir);
            }
            
            if (progress > f32_epsilon)
                renderer_draw_chain(origin, target, relative_time, progress_length, amplitude);
            
            length_count += relative_length;
            origin = target;
        }
    }
    
    // Hook
    if (hook_state)
    {
        Sprite sprite = sprites->hook;
        v4 tc = sprite_compute_frame_texcoord(sprite, (hook_state == 1) ? 1 : 0);
        
        renderer_batch_set_image(sprite_get_image(sprite));
        renderer_batch_append(hook_position, v2_set(hook_size, hook_size), hook_rotation, tc, 0.8f, 0.5f, color_white());
    }
    
    renderer_batch_draw();
}

static void draw_hook_preview(Simulation* sim)
{
    SV_PROFILE();
    
    CommandList cmd = game->cmd;
    Camera* camera = &sim->camera;
    
    if (!sim->settings.allow_user_events) return;
    
#if SV_DEV
    // if (game->debug.screenshot_mode) return;
#endif
    
    Entity* player = entity_get(sim, sim->player);
    
    if (player == NULL) return;
    
    
    PhysicsData* ph = &player->physics;
    GrapplingData* gr = player->grappling;
    
    b8 cursor_show = FALSE;
    v2 cursor_position = v2_zero();
    b8 cursor_special = FALSE;
    
    if (!gr->active && sim->hook_preview.valid)
    {
        cursor_show = TRUE;
        cursor_position = sim->hook_preview.position;
        
        Entity* preview_entity = entity_get(sim, sim->hook_preview.entity);
        if (preview_entity && preview_entity->physics.grappling_priority == GrapplingPriority_Special)
        {
            cursor_special = TRUE;
        }
    }
    
    if (sim->draw_settings.hide_player) cursor_show = FALSE;
    if (sim->draw_settings.hide_preview) cursor_show = FALSE;
    
    if (!cursor_show) return;
    
    renderer_batch_start();
    renderer_batch_set_shader(RendererLayer_SpecialUI);
    
    f32 now = (f32)simulation_now(sim);
    
    renderer_batch_set_image(NULL);
    f32 size0 = sim->camera.size * 0.045f;
    f32 size1 = size0 * 1.3f;
    
    f32 size = f32_lerp(size0, size1, f32_sin01(now * 3.f));
    
    f32 rotation = camera->rotation;
    
    Sprite cursor_sprite = cursor_special ? sprites->hook_preview_special : sprites->hook_preview;
    
    renderer_batch_set_image(sprite_get_image(cursor_sprite));
    renderer_batch_append(cursor_position, v2_set(size, size), rotation, cursor_sprite.texcoord, 0.f, 1.f, color_white());
    
    // Particles
    if (cursor_special)
    {
        f32 size = size1 * 1.3f;
        
        v2 up = v2_direction(rotation + f32_pi * 0.5f);
        
        Sprite spr[] = {
            sprites->hook_preview_sparks0,
            sprites->hook_preview_sparks1,
        };
        
        u32 count = 4;
        foreach(i, count)
        {
            f32 offset = ((now * 0.5f) + f32_random(0x83651535ULL + i * 0x744ULL));
            offset = f32_rotate_value(offset, 1.0f);
            
            f32 alpha = 1.0f;
            if (offset < 0.2f) alpha = offset / 0.4f;
            else if (offset > 0.9f) alpha = 1.f - ((offset - 0.9f) / 0.1f);
            
            Color color = color_white();
            color.a = (u8)(alpha * 255.f);
            
            Sprite sprite = spr[i % array_count(spr)];
            
            v2 p = cursor_position;
            p = p + (up * (offset * 0.4f));
            
            renderer_batch_set_image(sprite_get_image(sprite));
            renderer_batch_append(p, v2_set(size, size), rotation, sprite.texcoord, 0.f, 1.f, color);
        }
    }
    
    renderer_batch_draw();
}

static void draw_gravity_animation(Simulation* sim)
{
    SV_PROFILE();
    
    Camera* camera = &sim->camera;
    
    if (sim->gravity_animation_start_tick == 0) return;
    
    const f32 duration = 0.7f;
    
    const f32 min_height = 5.f;
    const f32 max_height = 10.f;
    
    const f32 max_offset = 0.15f;
    
    const f32 max_alpha = 0.5f;
    const f32 alpha_offset = 0.2f;
    const f32 fade_in_duration_alpha = 0.2f;
    const f32 fade_out_start_alpha = 0.2f;
    
    const f32 thickness_mult = 0.005f;
    
    const f32 x_min_advance = 0.0025f;
    const f32 x_max_advance = 0.04f;
    
    f32 t = time_since(sim, sim->gravity_animation_start_tick);
    t /= duration;
    
    if (t >= 1.0f) return;
    
    renderer_batch_start();
    
    Sprite sprite = sprites->line_particle;
    
    renderer_batch_set_shader(RendererLayer_SpecialUI);
    renderer_batch_set_image(sprite_get_image(sprite));
    
    v2 camera_size = camera_get_size(camera);
    
    f32 thickness = camera_size.y * thickness_mult;
    
    v2 gravity_direction = v2_from_axis_direction2D(sim->gravity_direction);
    f32 rotation = v2_angle(gravity_direction) - f32_pi * 0.5f;
    v2 up = -gravity_direction;
    v2 right = v2_set(up.y, -up.x);
    
    v2 center = camera_calculate_position(camera, (f32)simulation_accurate_now(sim));
    
    v2 min = center;
    min -= right * camera_size.x * 0.5f;
    min -= up * camera_size.y * 0.5f;
    
    u64 seed = 0x3ffAA5ULL;
    
    f32 x = -0.25f;
    u32 i = 0;
    
    while (1)
    {
        u64 seed0 = seed + i * 0x29FFF548ULL;
        x += f32_random_range(seed0 + 0x29158645ULL, x_min_advance, x_max_advance);
        
        if (x > 1.25f) break;
        
        v2 origin = center;
        origin += (x - 0.5f) * camera_size.x * right;
        origin += camera_size.y * 0.5f * up;
        
        origin += up * (f32_random_max(seed0 + 0x665666ULL, max_offset) - max_offset * 0.5f) * camera_size.y;
        
        f32 height = f32_random_range(seed0 + 0x84251555ULL, min_height, max_height);
        
        f32 desplacement = camera_size.y;
        
        f32 n = f32_pow(t, 0.75f);
        
        v2 pos = v2_lerp(origin, origin + desplacement * -up, n);
        
        f32 alpha = 1.f;
        {
            if (t < fade_in_duration_alpha) alpha = t / fade_in_duration_alpha;
            else if (t >= fade_out_start_alpha) alpha = 1.f - (t - fade_out_start_alpha) / (1.f - fade_out_start_alpha);
            alpha *= f32_random_range(seed0 + 0x8FFULL, max_alpha - alpha_offset, max_alpha);;
        }
        
        Color color = COLOR_GRAVITY;
        color.a = alpha;
        
        renderer_batch_append(pos, v2_set(thickness, height), rotation, sprite.texcoord, 0.f, 1.f, color);
        i++;
    }
    
    // BG
    {
        renderer_batch_set_image(NULL);
        
        const f32 max_alpha = 0.05f;
        const f32 fade_in_duration_alpha = 0.2f;
        const f32 fade_out_start_alpha = 0.5f;
        
        f32 alpha = 1.f;
        {
            if (t < fade_in_duration_alpha) alpha = t / fade_in_duration_alpha;
            else if (t >= fade_out_start_alpha) alpha = 1.f - (t - fade_out_start_alpha) / (1.f - fade_out_start_alpha);
            alpha *= max_alpha;
        }
        
        
        Color color = COLOR_GRAVITY;
        color.a = alpha;
        
        f32 size = MAX(camera_size.x, camera_size.y) * 1.5f;
        renderer_batch_append(center, v2_set(size, size), camera->rotation, v4_zero(), 0.f, 1.f, color);
    }
    
    renderer_batch_draw();
}

b8 entity_filter__gamepad_hook_preview(Entity* entity, const void*_)
{
    if (entity->type == ENTITY_TYPE_PLAYER) return FALSE;
    if (entity->type == ENTITY_TYPE_LASER_TRAP) return FALSE;
    if (entity->type == ENTITY_TYPE_LEVEL_LIMIT) return FALSE;
    return TRUE;
}

void draw_gamepad_hook_preview(Simulation* sim)
{
    SV_PROFILE();
    
    Camera* camera = &sim->camera;
    Entity* player = entity_get(sim, sim->player);
    
    if (player == NULL || player->grappling->active) return;
    
    v2 direction = sim->player_controller.look_direction;
    v2 cursor = sim->player_controller.look_cursor;
    
    if (v2_is_zero(direction)) return;
    
    renderer_batch_start();
    renderer_batch_set_shader(RendererLayer_SpecialUI);
    
    renderer_batch_set_image(NULL);
    
    // Calculate distance & adjust direction & cursor
    f32 distance = 0.f;
    {
        //distance = v2_distance(player->position, cursor);
        distance = sim->camera.size * 1.5f;
        
        Ray2D ray;
        ray.origin = player->position;
        ray.direction = direction;
        
        f32 dist;
        if (ray_cast_collisions(sim, ray, distance, entity_filter__gamepad_hook_preview, NULL, TRUE, &dist, NULL, NULL))
        {
            distance = dist;
            cursor = ray.origin + ray.direction * distance;
        }
    }
    
    Color color = COLOR_CYAN;
    color.a = 100;
    
    v2 p = player->position;
    f32 step_length = 0.5f;
    f32 rotation = v2_angle(direction);
    
    u32 steps = (u32)f32_truncate_high(distance / step_length);
    foreach(i, steps)
    {
        v2 p1 = p + direction * step_length * 0.5f;
        v2 size = p1 - p;
        v2 center = p + size * 0.5f;
        size = v2_set(step_length * 0.5f, 0.05f);
        
        f32 dist = v2_distance(p1, player->position);
        if (dist > distance) break;
        
        renderer_batch_append(center, size, rotation, v4_zero(), 1.f, 0.f, color);
        
        p += direction * step_length;
    }
    
    // renderer_batch_append(cursor, v2_set(0.2f, 0.2f), 0.f, v4_zero(), 1.f, 0.f, color_blue());
    
    renderer_batch_draw();
}

void simulation_draw(Simulation* sim)
{
    SV_PROFILE();
    SCRATCH();
    
    CommandList cmd = game->cmd;
    Camera* camera = &sim->camera;
    
    simulation_update_matrices(sim);
    
    renderer_draw_light_buffer(sim);
    renderer_bind_globals(sim);
    
    v4 bg_bounds = draw_background(sim);
    
    renderer_enable_parallax_effect(!sim->draw_settings.hide_parallax_effect);
    
    if (!sim->draw_settings.hide_layer_bg1) renderer_draw_entities(sim, Layer_BG1);
    if (!sim->draw_settings.hide_layer_bg0) renderer_draw_entities(sim, Layer_BG0);
    
    if (!sim->draw_settings.hide_layer_active)
    {
        renderer_draw_entities(sim, Layer_Fog);
        renderer_draw_entities(sim, Layer_Back);
        renderer_draw_entities(sim, Layer_Player);
        renderer_draw_entities(sim, Layer_Fore);
    }
    
    draw_chain(sim);
    
    if (!sim->draw_settings.hide_layer_active)
    {
        renderer_draw_entities(sim, Layer_Deco);
    }
    
    {
        auto l = sim->lighting;
        postprocess_shock_wave(&sim->camera, l.shock_wave_origin, l.shock_wave_range, l.shock_wave_progress);
    }
    
    draw_hook_preview(sim);
    draw_gravity_animation(sim);
    
    // Draw editor grid
    if (sim->draw_settings.editor_grid) draw_editor_grid(sim);
    
    // Gamepad Hook Preview
    if (sim->state == SimulationState_Playing && os.input.controller == ControllerType_Gamepad && sim->settings.allow_user_events) 
        draw_gamepad_hook_preview(sim);
    
#if SV_DEV
    // Debug
    {
        imrend_begin(sim, cmd);
        
        // TEMP
        if (sim->hook_preview.valid)
        {
            Entity* player = entity_get(sim, sim->player);
            assert(player);
            
            const f32 alpha = 0.5f;
            
            HookRayCastConfig config = player_get_hook_ray_cast_config(os.input.controller);
            
            Camera* camera = &sim->camera;
            v2 camera_size = camera_get_size(camera);
            
            v2 origin = camera->position;
            v2 direction = sim->player_controller.look_direction;
            v2 point_position = sim->hook_preview.position;
            
            Array<HookPointTarget> points = hook_ray_cast(scratch.arena, sim, origin, direction, sim->player_controller.look_cursor, player->physics.in_ground, &config);
            
            // Draw available points
            {
                f32 max_weigth = 0.f;
                
                foreach(i, points.count) {
                    HookPointTarget point = points[i];
                    max_weigth = MAX(max_weigth, point.total_weight);
                }
                
                foreach(i, points.count) {
                    HookPointTarget point = points[i];
                    
                    f32 weight = point.total_weight / max_weigth;
                    Color color = color_lerp(color_red(), color_green(), weight);
                    
                    imrend_draw_quad(point.position, v2_set(0.2f, 0.2f), 0.f, color, cmd);
                }
            }
            
            // Angle range
            {
                v2 up = v2_set(-direction.y, direction.x);
                
                v2 desp = up * config.angle_range * camera_size.y * 0.5f;
                
                v2 p0 = origin + desp;
                v2 p1 = origin - desp;
                
                imrend_draw_line(p0, p0 + direction * camera_size.y, color_red(alpha), cmd);
                imrend_draw_line(p1, p1 + direction * camera_size.y, color_red(alpha), cmd);
                
                v2 proj0 = p0 + v2_dot(point_position - p0, direction) * direction;
                v2 proj1 = p1 + v2_dot(point_position - p1, direction) * direction;
                
                imrend_draw_quad(proj0, v2_set(0.2f, 0.2f), 0.f, color_blue(alpha), cmd);
                imrend_draw_quad(proj1, v2_set(0.2f, 0.2f), 0.f, color_blue(alpha), cmd);
                
                v2 dir0 = v2_normalize(point_position - proj0);
                v2 dir1 = v2_normalize(point_position - proj1);
                
                imrend_draw_line(proj0, proj0 + dir0 * 0.2f, color_green(alpha), cmd);
                imrend_draw_line(proj1, proj1 + dir1 * 0.2f, color_green(alpha), cmd);
            }
            
            // Cursor range
            if (config.cursor_mult > f32_epsilon)
            {
                v2 o = sim->player_controller.look_cursor;
                f32 r = config.cursor_range * camera_size.y;
                
                Color color = color_red(alpha);
                
                u32 res = 100;
                foreach(i, res)
                {
                    f32 a0 = ((f32)(i + 0) / (f32)res) * f32_tau;
                    f32 a1 = ((f32)(i + 1) / (f32)res) * f32_tau;
                    
                    v2 d0 = v2_direction(a0);
                    v2 d1 = v2_direction(a1);
                    
                    v2 p0 = o + d0 * r;
                    v2 p1 = o + d1 * r;
                    imrend_draw_line(p0, p1, color, cmd);
                }
            }
        }
        
        // Colliders
        if (game->debug.physics.draw_colliders)
        {
            imrend_line_width(2.f, cmd);
            
            foreach_entity(sim, it)
            {
                if (it.value->physics.body_type == BodyType_None) continue;
                if (!layer_is_active(it.value->render_layer)) continue;
                
                Collider colliders[MAX_COLLIDERS];
                u32 collider_count = physics_compute_colliders(sim, it.value, colliders);
                
                foreach(i, collider_count)
                {
                    v2 p = bounds_coord(colliders[i].bounds);
                    v2 s = bounds_size(colliders[i].bounds);
                    f32 r = colliders[i].rotation;
                    
                    v2 c[4];
                    physics_polygon_points(p, s, r, it.value->position, c);
                    
                    Color color = color_green();
                    
                    imrend_draw_line(v2_to_v3(c[0], 0.f), v2_to_v3(c[1], 0.f), color, cmd);
                    imrend_draw_line(v2_to_v3(c[1], 0.f), v2_to_v3(c[2], 0.f), color, cmd);
                    imrend_draw_line(v2_to_v3(c[2], 0.f), v2_to_v3(c[3], 0.f), color, cmd);
                    imrend_draw_line(v2_to_v3(c[3], 0.f), v2_to_v3(c[0], 0.f), color, cmd);
                }
            }
        }
        
        imrend_end(cmd);
    }
#endif
}