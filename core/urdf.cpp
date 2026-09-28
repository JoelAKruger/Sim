#include "core/urdf.h"

#include <expat.h>
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// Expat reads the document twice with the same handlers: the first pass only counts
// elements so the second can fill arrays allocated once. During counting, writes go to
// scratch records that are thrown away.

#define MAX_MATERIALS 256
#define MAX_CONTROL_JOINTS 256

enum Urdf_Scope {
    SCOPE_NONE,
    SCOPE_LINK,
    SCOPE_JOINT,
    SCOPE_MATERIAL,
    SCOPE_GAZEBO,
    SCOPE_CONTROL,
    SCOPE_TRANSMISSION,
};

struct Named_Color {
    char name[URDF_NAME_SIZE];
    f32 rgba[4];
};

struct Joint_Names {
    char parent[URDF_NAME_SIZE];
    char child[URDF_NAME_SIZE];
};

// A joint named inside <ros2_control> or <transmission>, resolved after parsing.
struct Control_Joint {
    char joint[URDF_NAME_SIZE];
    i32 control;
    bool position;
};

struct Urdf_Parser {
    XML_Parser xml;
    Urdf_Model *model;
    bool counting;
    bool failed;
    char *error;
    u32 error_size;

    u32 links;
    u32 joints;
    u32 visuals;
    u32 collisions;
    u32 controls;
    Joint_Names *joint_names;

    Urdf_Scope scope;
    u32 depth;
    u32 skip_depth; // ignore everything inside the element at this depth; 0 when not skipping
    bool in_inertial;
    Urdf_Shape *shape;
    char material_name[URDF_NAME_SIZE];
    bool in_closure_plugin;
    char closure_parent[URDF_NAME_SIZE];
    char closure_child[URDF_NAME_SIZE];
    char closure_model[URDF_NAME_SIZE];
    char param_name[URDF_NAME_SIZE];
    char text[512];
    u32 text_length;

    Named_Color materials[MAX_MATERIALS];
    u32 material_count;
    char closure_names[URDF_MAX_CLOSURES][2][URDF_NAME_SIZE];
    u32 closure_count;
    Control_Joint control_joints[MAX_CONTROL_JOINTS];
    u32 control_joint_count;

    Urdf_Link scratch_link;
    Urdf_Joint scratch_joint;
    Joint_Names scratch_names;
    Urdf_Shape scratch_shape;
    Urdf_Control scratch_control;
};

static void fail_parse(Urdf_Parser *p, const char *format, ...)
    __attribute__((format(printf, 2, 3)));

static void fail_parse(Urdf_Parser *p, const char *format, ...)
{
    if (p->failed) {
        return;
    }
    p->failed = true;
    int used = snprintf(p->error, p->error_size,
                        "line %lu: ", (unsigned long)XML_GetCurrentLineNumber(p->xml));
    va_list args;
    va_start(args, format);
    vsnprintf(p->error + used, p->error_size - (u32)used, format, args);
    va_end(args);
    XML_StopParser(p->xml, XML_FALSE);
}

static const char *get_attribute(const XML_Char **attributes, const char *name)
{
    for (u32 i = 0; attributes[i]; i += 2) {
        if (strcmp(attributes[i], name) == 0) {
            return attributes[i + 1];
        }
    }
    return NULL;
}

// Exactly count whitespace-separated numbers.
static bool parse_numbers(const char *text, f64 *values, u32 count)
{
    const char *cursor = text;
    for (u32 i = 0; i < count; i++) {
        char *end;
        values[i] = strtod(cursor, &end);
        if (end == cursor || !isfinite(values[i])) {
            return false;
        }
        cursor = end;
    }
    while (*cursor == ' ' || *cursor == '\t' || *cursor == '\n' || *cursor == '\r') {
        cursor++;
    }
    return *cursor == 0;
}

static f32 get_attribute_number(Urdf_Parser *p, const XML_Char **attributes, const char *name,
                                f32 fallback)
{
    const char *text = get_attribute(attributes, name);
    f64 value;
    if (!text) {
        return fallback;
    }
    if (!parse_numbers(text, &value, 1)) {
        fail_parse(p, "%s=\"%s\" is not a number", name, text);
        return fallback;
    }
    return (f32)value;
}

static v3 get_attribute_vector(Urdf_Parser *p, const XML_Char **attributes, const char *name,
                               v3 fallback)
{
    const char *text = get_attribute(attributes, name);
    f64 values[3];
    if (!text) {
        return fallback;
    }
    if (!parse_numbers(text, values, 3)) {
        fail_parse(p, "%s=\"%s\" is not three numbers", name, text);
        return fallback;
    }
    return v3{(f32)values[0], (f32)values[1], (f32)values[2]};
}

static void copy_name(char *destination, const char *source)
{
    snprintf(destination, URDF_NAME_SIZE, "%s", source ? source : "");
}

Quat make_quat_from_rpy(f64 roll, f64 pitch, f64 yaw)
{
    f64 cr = cos(roll * 0.5), sr = sin(roll * 0.5);
    f64 cp = cos(pitch * 0.5), sp = sin(pitch * 0.5);
    f64 cy = cos(yaw * 0.5), sy = sin(yaw * 0.5);
    Quat q;
    q.s = (f32)(cr * cp * cy + sr * sp * sy);
    q.v.x = (f32)(sr * cp * cy - cr * sp * sy);
    q.v.y = (f32)(cr * sp * cy + sr * cp * sy);
    q.v.z = (f32)(cr * cp * sy - sr * sp * cy);
    return q;
}

static Pose parse_origin(Urdf_Parser *p, const XML_Char **attributes)
{
    Pose origin = {};
    origin.p = get_attribute_vector(p, attributes, "xyz", v3{0.0f, 0.0f, 0.0f});
    const char *rpy_text = get_attribute(attributes, "rpy");
    f64 rpy[3] = {0.0, 0.0, 0.0};
    if (rpy_text && !parse_numbers(rpy_text, rpy, 3)) {
        fail_parse(p, "rpy=\"%s\" is not three numbers", rpy_text);
    }
    origin.q = make_quat_from_rpy(rpy[0], rpy[1], rpy[2]);
    return origin;
}

static void add_material(Urdf_Parser *p, const char *name, const f32 *rgba)
{
    if (!name || !name[0] || p->material_count == MAX_MATERIALS) {
        return;
    }
    Named_Color *material = &p->materials[p->material_count++];
    copy_name(material->name, name);
    memcpy(material->rgba, rgba, sizeof(material->rgba));
}

static Urdf_Link *get_current_link(Urdf_Parser *p)
{
    return p->counting ? &p->scratch_link : &p->model->links[p->links - 1];
}

static Urdf_Joint *get_current_joint(Urdf_Parser *p)
{
    return p->counting ? &p->scratch_joint : &p->model->joints[p->joints - 1];
}

static Joint_Names *get_current_joint_names(Urdf_Parser *p)
{
    return p->counting ? &p->scratch_names : &p->joint_names[p->joints - 1];
}

static Urdf_Control *get_current_control(Urdf_Parser *p)
{
    return p->counting ? &p->scratch_control : &p->model->controls[p->controls - 1];
}

static void start_shape(Urdf_Parser *p, bool visual)
{
    Urdf_Shape *shape;
    if (p->counting) {
        shape = &p->scratch_shape;
    } else if (visual) {
        shape = &p->model->visuals[p->visuals];
    } else {
        shape = &p->model->collisions[p->collisions];
    }
    *shape = {};
    shape->link = p->links - 1;
    shape->origin.q = Quat{{0.0f, 0.0f, 0.0f}, 1.0f};
    shape->color[0] = -1.0f; // unresolved until the end
    if (visual) {
        p->visuals++;
    } else {
        p->collisions++;
    }
    p->shape = shape;
}

static void start_geometry(Urdf_Parser *p, const char *name, const XML_Char **attributes)
{
    Urdf_Geometry *geometry = &p->shape->geometry;
    if (strcmp(name, "box") == 0) {
        geometry->type = URDF_GEOMETRY_BOX;
        geometry->size = get_attribute_vector(p, attributes, "size", v3{0.0f, 0.0f, 0.0f});
    } else if (strcmp(name, "cylinder") == 0) {
        geometry->type = URDF_GEOMETRY_CYLINDER;
        geometry->radius = get_attribute_number(p, attributes, "radius", 0.0f);
        geometry->length = get_attribute_number(p, attributes, "length", 0.0f);
    } else if (strcmp(name, "sphere") == 0) {
        geometry->type = URDF_GEOMETRY_SPHERE;
        geometry->radius = get_attribute_number(p, attributes, "radius", 0.0f);
    } else if (strcmp(name, "mesh") == 0) {
        geometry->type = URDF_GEOMETRY_MESH;
        geometry->scale = get_attribute_vector(p, attributes, "scale", v3{1.0f, 1.0f, 1.0f});
        const char *filename = get_attribute(attributes, "filename");
        if (!filename) {
            fail_parse(p, "<mesh> without a filename");
            return;
        }
        snprintf(geometry->mesh, sizeof(geometry->mesh), "%s", filename);
    }
}

static void start_joint(Urdf_Parser *p, const XML_Char **attributes)
{
    p->joints++;
    Urdf_Joint *joint = get_current_joint(p);
    *joint = {};
    *get_current_joint_names(p) = {};
    copy_name(joint->name, get_attribute(attributes, "name"));
    joint->origin.q = Quat{{0.0f, 0.0f, 0.0f}, 1.0f};
    joint->axis = v3{1.0f, 0.0f, 0.0f};
    joint->control = -1;
    const char *type = get_attribute(attributes, "type");
    static const struct {
        const char *name;
        Urdf_Joint_Type type;
    } types[] = {
        {"fixed", URDF_JOINT_FIXED},           {"revolute", URDF_JOINT_REVOLUTE},
        {"continuous", URDF_JOINT_CONTINUOUS}, {"prismatic", URDF_JOINT_PRISMATIC},
        {"floating", URDF_JOINT_FLOATING},     {"planar", URDF_JOINT_PLANAR},
    };
    for (u32 i = 0; i < ARRAY_COUNT(types); i++) {
        if (type && strcmp(type, types[i].name) == 0) {
            joint->type = types[i].type;
            return;
        }
    }
    fail_parse(p, "joint \"%s\" has unknown type \"%s\"", joint->name, type ? type : "");
}

static void start_joint_child(Urdf_Parser *p, const char *name, const XML_Char **attributes)
{
    Urdf_Joint *joint = get_current_joint(p);
    Joint_Names *names = get_current_joint_names(p);
    if (strcmp(name, "origin") == 0) {
        joint->origin = parse_origin(p, attributes);
    } else if (strcmp(name, "parent") == 0) {
        copy_name(names->parent, get_attribute(attributes, "link"));
    } else if (strcmp(name, "child") == 0) {
        copy_name(names->child, get_attribute(attributes, "link"));
    } else if (strcmp(name, "axis") == 0) {
        v3 axis = get_attribute_vector(p, attributes, "xyz", v3{1.0f, 0.0f, 0.0f});
        if (get_length(axis) < 1e-6f) {
            fail_parse(p, "joint \"%s\" has a zero axis", joint->name);
            return;
        }
        joint->axis = normalize(axis);
    } else if (strcmp(name, "limit") == 0) {
        joint->has_limits = true;
        joint->lower = get_attribute_number(p, attributes, "lower", 0.0f);
        joint->upper = get_attribute_number(p, attributes, "upper", 0.0f);
        joint->effort = get_attribute_number(p, attributes, "effort", 0.0f);
        joint->velocity = get_attribute_number(p, attributes, "velocity", 0.0f);
    } else if (strcmp(name, "dynamics") == 0) {
        joint->damping = get_attribute_number(p, attributes, "damping", 0.0f);
        joint->friction = get_attribute_number(p, attributes, "friction", 0.0f);
    }
}

static void start_link_child(Urdf_Parser *p, const char *name, const XML_Char **attributes)
{
    Urdf_Link *link = get_current_link(p);
    if (strcmp(name, "inertial") == 0) {
        p->in_inertial = true;
        link->inertial.present = true;
    } else if (strcmp(name, "visual") == 0) {
        start_shape(p, true);
    } else if (strcmp(name, "collision") == 0) {
        start_shape(p, false);
    } else if (strcmp(name, "origin") == 0) {
        if (p->in_inertial) {
            link->inertial.origin = parse_origin(p, attributes);
        } else if (p->shape) {
            p->shape->origin = parse_origin(p, attributes);
        }
    } else if (p->in_inertial && strcmp(name, "mass") == 0) {
        link->inertial.mass = get_attribute_number(p, attributes, "value", 0.0f);
    } else if (p->in_inertial && strcmp(name, "inertia") == 0) {
        f32 ixx = get_attribute_number(p, attributes, "ixx", 0.0f);
        f32 ixy = get_attribute_number(p, attributes, "ixy", 0.0f);
        f32 ixz = get_attribute_number(p, attributes, "ixz", 0.0f);
        f32 iyy = get_attribute_number(p, attributes, "iyy", 0.0f);
        f32 iyz = get_attribute_number(p, attributes, "iyz", 0.0f);
        f32 izz = get_attribute_number(p, attributes, "izz", 0.0f);
        link->inertial.inertia.cx = v3{ixx, ixy, ixz};
        link->inertial.inertia.cy = v3{ixy, iyy, iyz};
        link->inertial.inertia.cz = v3{ixz, iyz, izz};
    } else if (p->shape && strcmp(name, "material") == 0) {
        copy_name(p->shape->material, get_attribute(attributes, "name"));
    } else if (p->shape && strcmp(name, "color") == 0) {
        const char *text = get_attribute(attributes, "rgba");
        f64 rgba[4];
        if (!text || !parse_numbers(text, rgba, 4)) {
            fail_parse(p, "<color> needs rgba=\"r g b a\"");
            return;
        }
        for (u32 i = 0; i < 4; i++) {
            p->shape->color[i] = (f32)rgba[i];
        }
        add_material(p, p->shape->material, p->shape->color); // usable by name elsewhere
    } else if (p->shape) {
        start_geometry(p, name, attributes);
    }
}

static bool contains_text(const char *text, const char *part)
{
    return text && strstr(text, part) != NULL;
}

static void handle_start_element(void *context, const XML_Char *name, const XML_Char **attributes)
{
    Urdf_Parser *p = (Urdf_Parser *)context;
    p->depth++;
    p->text_length = 0;
    p->text[0] = 0;
    if (p->skip_depth) {
        return;
    }
    if (strncmp(name, "xacro:", 6) == 0) {
        fail_parse(p, "found <%s>: this is an unexpanded xacro; run xacro on it first", name);
        return;
    }
    if (p->depth == 1) {
        if (strcmp(name, "robot") != 0) {
            fail_parse(p, "expected <robot>, found <%s>", name);
            return;
        }
        copy_name(p->model->name, get_attribute(attributes, "name"));
        return;
    }
    if (p->depth == 2) {
        if (strcmp(name, "link") == 0) {
            p->scope = SCOPE_LINK;
            p->links++;
            Urdf_Link *link = get_current_link(p);
            *link = {};
            copy_name(link->name, get_attribute(attributes, "name"));
            link->inertial.origin.q = Quat{{0.0f, 0.0f, 0.0f}, 1.0f};
            link->parent_joint = -1;
            link->first_visual = p->visuals;
            link->first_collision = p->collisions;
        } else if (strcmp(name, "joint") == 0) {
            p->scope = SCOPE_JOINT;
            start_joint(p, attributes);
        } else if (strcmp(name, "material") == 0) {
            p->scope = SCOPE_MATERIAL;
            copy_name(p->material_name, get_attribute(attributes, "name"));
        } else if (strcmp(name, "gazebo") == 0 && !get_attribute(attributes, "reference")) {
            p->scope = SCOPE_GAZEBO;
        } else if (strcmp(name, "ros2_control") == 0) {
            p->scope = SCOPE_CONTROL;
            p->controls++;
            Urdf_Control *control = get_current_control(p);
            *control = {};
            copy_name(control->name, get_attribute(attributes, "name"));
        } else if (strcmp(name, "transmission") == 0) {
            p->scope = SCOPE_TRANSMISSION;
        } else {
            p->skip_depth = p->depth; // extensions we don't use
        }
        return;
    }

    switch (p->scope) {
    case SCOPE_LINK:
        start_link_child(p, name, attributes);
        break;
    case SCOPE_JOINT:
        start_joint_child(p, name, attributes);
        break;
    case SCOPE_MATERIAL:
        if (strcmp(name, "color") == 0) {
            f64 rgba[4];
            const char *text = get_attribute(attributes, "rgba");
            if (!text || !parse_numbers(text, rgba, 4)) {
                fail_parse(p, "<color> needs rgba=\"r g b a\"");
                return;
            }
            f32 color[4] = {(f32)rgba[0], (f32)rgba[1], (f32)rgba[2], (f32)rgba[3]};
            add_material(p, p->material_name, color);
        }
        break;
    case SCOPE_GAZEBO:
        if (p->depth == 3 && strcmp(name, "plugin") == 0) {
            if (contains_text(get_attribute(attributes, "filename"), "detachable-joint") ||
                contains_text(get_attribute(attributes, "name"), "DetachableJoint")) {
                p->in_closure_plugin = true;
                p->closure_parent[0] = p->closure_child[0] = p->closure_model[0] = 0;
            } else {
                p->skip_depth = p->depth;
            }
        }
        break;
    case SCOPE_CONTROL:
        if (strcmp(name, "param") == 0) {
            copy_name(p->param_name, get_attribute(attributes, "name"));
        } else if (strcmp(name, "joint") == 0 && p->control_joint_count < MAX_CONTROL_JOINTS) {
            Control_Joint *entry = &p->control_joints[p->control_joint_count++];
            copy_name(entry->joint, get_attribute(attributes, "name"));
            entry->control = (i32)p->controls - 1;
            entry->position = false;
        } else if (strcmp(name, "command_interface") == 0 && p->control_joint_count > 0 &&
                   strcmp(get_attribute(attributes, "name") ? get_attribute(attributes, "name")
                                                            : "",
                          "position") == 0) {
            p->control_joints[p->control_joint_count - 1].position = true;
        }
        break;
    case SCOPE_TRANSMISSION:
        if (strcmp(name, "joint") == 0 && p->control_joint_count < MAX_CONTROL_JOINTS) {
            Control_Joint *entry = &p->control_joints[p->control_joint_count++];
            copy_name(entry->joint, get_attribute(attributes, "name"));
            entry->control = -1;
            entry->position = false;
        }
        break;
    case SCOPE_NONE:
        break;
    }
}

static bool is_text_true(const char *text)
{
    return strcmp(text, "true") == 0 || strcmp(text, "1") == 0;
}

static void handle_end_element(void *context, const XML_Char *name)
{
    Urdf_Parser *p = (Urdf_Parser *)context;
    char *text = p->text;
    // Trim surrounding whitespace from character data.
    while (*text == ' ' || *text == '\n' || *text == '\t' || *text == '\r') {
        text++;
    }
    for (char *end = text + strlen(text);
         end > text && (end[-1] == ' ' || end[-1] == '\n' || end[-1] == '\t' || end[-1] == '\r');
         end--) {
        end[-1] = 0;
    }

    if (p->skip_depth) {
        if (p->depth == p->skip_depth) {
            p->skip_depth = 0;
        }
        p->depth--;
        return;
    }

    switch (p->scope) {
    case SCOPE_LINK:
        if (strcmp(name, "inertial") == 0) {
            p->in_inertial = false;
        } else if (strcmp(name, "visual") == 0 || strcmp(name, "collision") == 0) {
            p->shape = NULL;
        } else if (p->depth == 2) {
            Urdf_Link *link = get_current_link(p);
            link->visual_count = p->visuals - link->first_visual;
            link->collision_count = p->collisions - link->first_collision;
        }
        break;
    case SCOPE_GAZEBO:
        if (strcmp(name, "self_collide") == 0) {
            p->model->self_collide = is_text_true(text);
        } else if (p->in_closure_plugin && strcmp(name, "parent_link") == 0) {
            copy_name(p->closure_parent, text);
        } else if (p->in_closure_plugin && strcmp(name, "child_link") == 0) {
            copy_name(p->closure_child, text);
        } else if (p->in_closure_plugin && strcmp(name, "child_model") == 0) {
            copy_name(p->closure_model, text);
        } else if (p->in_closure_plugin && strcmp(name, "plugin") == 0) {
            p->in_closure_plugin = false;
            if (p->closure_model[0] && strcmp(p->closure_model, p->model->name) != 0) {
                log_warning("urdf: ignoring a DetachableJoint to another model (%s)",
                            p->closure_model);
            } else if (p->closure_count == URDF_MAX_CLOSURES) {
                fail_parse(p, "more than %u loop closures", URDF_MAX_CLOSURES);
            } else {
                copy_name(p->closure_names[p->closure_count][0], p->closure_parent);
                copy_name(p->closure_names[p->closure_count][1], p->closure_child);
                p->closure_count++;
            }
        }
        break;
    case SCOPE_CONTROL: {
        Urdf_Control *control = get_current_control(p);
        if (strcmp(name, "plugin") == 0) {
            copy_name(control->plugin, text);
        } else if (strcmp(name, "param") == 0 && p->depth == 4 &&
                   control->param_count < URDF_MAX_CONTROL_PARAMS) {
            Urdf_Control_Param *param = &control->params[control->param_count++];
            copy_name(param->name, p->param_name);
            copy_name(param->value, text);
        }
        break;
    }
    case SCOPE_TRANSMISSION:
        if (strcmp(name, "hardwareInterface") == 0 && p->control_joint_count > 0 &&
            contains_text(text, "Position")) {
            p->control_joints[p->control_joint_count - 1].position = true;
        }
        break;
    default:
        break;
    }

    if (p->depth == 2) {
        p->scope = SCOPE_NONE;
    }
    p->depth--;
}

static void handle_character_data(void *context, const XML_Char *data, int length)
{
    Urdf_Parser *p = (Urdf_Parser *)context;
    u32 room = (u32)sizeof(p->text) - 1 - p->text_length;
    u32 take = min((u32)length, room);
    memcpy(p->text + p->text_length, data, take);
    p->text_length += take;
    p->text[p->text_length] = 0;
}

static bool run_pass(Urdf_Parser *p, const char *xml, u64 xml_size)
{
    p->xml = XML_ParserCreate(NULL);
    XML_SetUserData(p->xml, p);
    XML_SetElementHandler(p->xml, handle_start_element, handle_end_element);
    XML_SetCharacterDataHandler(p->xml, handle_character_data);
    bool parsed = XML_Parse(p->xml, xml, (int)xml_size, XML_TRUE) == XML_STATUS_OK;
    if (!parsed && !p->failed) {
        p->failed = true;
        snprintf(p->error, p->error_size, "line %lu: %s",
                 (unsigned long)XML_GetCurrentLineNumber(p->xml),
                 XML_ErrorString(XML_GetErrorCode(p->xml)));
    }
    XML_ParserFree(p->xml);
    return !p->failed;
}

static bool resolve_model(Urdf_Parser *p)
{
    Urdf_Model *model = p->model;
    for (u32 i = 0; i < model->link_count; i++) {
        for (u32 j = 0; j < i; j++) {
            if (strcmp(model->links[i].name, model->links[j].name) == 0) {
                snprintf(p->error, p->error_size, "two links are named \"%s\"",
                         model->links[i].name);
                return false;
            }
        }
    }
    for (u32 i = 0; i < model->joint_count; i++) {
        Urdf_Joint *joint = &model->joints[i];
        const Joint_Names *names = &p->joint_names[i];
        for (u32 j = 0; j < i; j++) {
            if (strcmp(joint->name, model->joints[j].name) == 0) {
                snprintf(p->error, p->error_size, "two joints are named \"%s\"", joint->name);
                return false;
            }
        }
        i32 parent = find_urdf_link(model, names->parent);
        i32 child = find_urdf_link(model, names->child);
        if (parent < 0 || child < 0) {
            snprintf(p->error, p->error_size, "joint \"%s\" refers to a missing link \"%s\"",
                     joint->name, parent < 0 ? names->parent : names->child);
            return false;
        }
        if (model->links[child].parent_joint >= 0) {
            snprintf(p->error, p->error_size,
                     "link \"%s\" is the child of both \"%s\" and \"%s\"; URDF must be a tree",
                     names->child, model->joints[model->links[child].parent_joint].name,
                     joint->name);
            return false;
        }
        joint->parent = (u32)parent;
        joint->child = (u32)child;
        model->links[child].parent_joint = (i32)i;
    }

    u32 roots = 0;
    for (u32 i = 0; i < model->link_count; i++) {
        if (model->links[i].parent_joint < 0) {
            model->root = i;
            roots++;
        }
    }
    if (roots != 1) {
        snprintf(p->error, p->error_size, "expected one root link, found %u", roots);
        return false;
    }
    // With one parent per link and one root, a link that can't reach the root is on a cycle.
    for (u32 i = 0; i < model->link_count; i++) {
        u32 link = i;
        for (u32 steps = 0; link != model->root; steps++) {
            if (steps > model->link_count) {
                snprintf(p->error, p->error_size, "link \"%s\" is part of a joint cycle",
                         model->links[i].name);
                return false;
            }
            link = model->joints[model->links[link].parent_joint].parent;
        }
    }

    for (u32 i = 0; i < p->closure_count; i++) {
        i32 parent = find_urdf_link(model, p->closure_names[i][0]);
        i32 child = find_urdf_link(model, p->closure_names[i][1]);
        if (parent < 0 || child < 0) {
            snprintf(p->error, p->error_size, "DetachableJoint refers to a missing link \"%s\"",
                     parent < 0 ? p->closure_names[i][0] : p->closure_names[i][1]);
            return false;
        }
        model->closures[model->closure_count++] = Urdf_Closure{(u32)parent, (u32)child};
    }

    for (u32 i = 0; i < p->control_joint_count; i++) {
        const Control_Joint *entry = &p->control_joints[i];
        i32 index = find_urdf_joint(model, entry->joint);
        if (index < 0) {
            log_warning("urdf: <ros2_control> names a missing joint \"%s\"; ignored", entry->joint);
            continue;
        }
        Urdf_Joint *joint = &model->joints[index];
        joint->actuated = true;
        joint->position_command = joint->position_command || entry->position;
        if (entry->control >= 0) {
            joint->control = entry->control;
        }
    }

    // Visual colours: inline, else a named material defined anywhere, else light grey.
    for (u32 i = 0; i < model->visual_count; i++) {
        Urdf_Shape *visual = &model->visuals[i];
        if (visual->color[0] >= 0.0f) {
            continue;
        }
        f32 grey[4] = {0.7f, 0.7f, 0.7f, 1.0f};
        memcpy(visual->color, grey, sizeof(grey));
        for (u32 m = 0; m < p->material_count; m++) {
            if (visual->material[0] && strcmp(p->materials[m].name, visual->material) == 0) {
                memcpy(visual->color, p->materials[m].rgba, sizeof(visual->color));
                break;
            }
        }
    }
    return true;
}

bool parse_urdf(Urdf_Model *model, Linear_Allocator *allocator, const char *xml, u64 xml_size,
                char *error, u32 error_size)
{
    *model = {};
    static Urdf_Parser parser; // large; one parse at a time
    Urdf_Parser *p = &parser;
    *p = {};
    p->model = model;
    p->error = error;
    p->error_size = error_size;
    p->counting = true;
    if (!run_pass(p, xml, xml_size)) {
        return false;
    }

    model->link_count = p->links;
    model->joint_count = p->joints;
    model->visual_count = p->visuals;
    model->collision_count = p->collisions;
    model->control_count = p->controls;
    model->links = ALLOCATE_ARRAY(allocator, Urdf_Link, max(model->link_count, 1u));
    model->joints = ALLOCATE_ARRAY(allocator, Urdf_Joint, max(model->joint_count, 1u));
    model->visuals = ALLOCATE_ARRAY(allocator, Urdf_Shape, max(model->visual_count, 1u));
    model->collisions = ALLOCATE_ARRAY(allocator, Urdf_Shape, max(model->collision_count, 1u));
    model->controls = ALLOCATE_ARRAY(allocator, Urdf_Control, max(model->control_count, 1u));
    Joint_Names *joint_names = ALLOCATE_ARRAY(allocator, Joint_Names, max(model->joint_count, 1u));
    if (!model->links || !model->joints || !model->visuals || !model->collisions ||
        !model->controls || !joint_names) {
        snprintf(error, error_size, "out of memory for the robot description");
        return false;
    }
    if (model->link_count == 0) {
        snprintf(error, error_size, "the robot has no links");
        return false;
    }

    *p = {};
    p->model = model;
    p->error = error;
    p->error_size = error_size;
    p->joint_names = joint_names;
    if (!run_pass(p, xml, xml_size)) {
        return false;
    }
    return resolve_model(p);
}

i32 find_urdf_link(const Urdf_Model *model, const char *name)
{
    for (u32 i = 0; i < model->link_count; i++) {
        if (strcmp(model->links[i].name, name) == 0) {
            return (i32)i;
        }
    }
    return -1;
}

i32 find_urdf_joint(const Urdf_Model *model, const char *name)
{
    for (u32 i = 0; i < model->joint_count; i++) {
        if (strcmp(model->joints[i].name, name) == 0) {
            return (i32)i;
        }
    }
    return -1;
}

const char *get_control_param(const Urdf_Model *model, i32 control, const char *name)
{
    if (control < 0 || (u32)control >= model->control_count) {
        return NULL;
    }
    const Urdf_Control *block = &model->controls[control];
    for (u32 i = 0; i < block->param_count; i++) {
        if (strcmp(block->params[i].name, name) == 0) {
            return block->params[i].value;
        }
    }
    return NULL;
}
