#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "core/config.h"

// A strict reader for the small subset of YAML that settings files use: nested "key:"
// maps indented with spaces; "key: number", "key: [number, ...]" or "key: text" (plain,
// "double" or 'single' quoted) on one line; and # comments. Keys below any
// "ros__parameters" level are joined with dots, so a ROS 2 parameters file and a plain
// one both work.

#define CONFIG_MAX_DEPTH 8
#define CONFIG_MAX_FILE (1 * MEGABYTE)

struct Yaml_Level {
    u32 indent;
    char key[64];
};

static char *trim_text(char *text)
{
    while (*text == ' ') {
        text++;
    }
    char *end = text + strlen(text);
    while (end > text && (end[-1] == ' ' || end[-1] == '\r')) {
        end--;
    }
    *end = 0;
    return text;
}

// Cuts the line at a # that starts a comment: outside quotes, and at the start of the line
// or after a space.
static void strip_comment(char *line)
{
    char quote = 0;
    for (char *c = line; *c; c++) {
        if (quote) {
            if (*c == quote) {
                quote = 0;
            }
        } else if (*c == '"' || *c == '\'') {
            quote = *c;
        } else if (*c == '#' && (c == line || c[-1] == ' ')) {
            *c = 0;
            return;
        }
    }
}

// A plain scalar, a "double-quoted" string (with \" and \\ escapes) or a 'single-quoted'
// one (with '' for a quote). False if the quotes don't close or anything follows them.
static bool parse_text(const char *text, char *out, u32 out_size)
{
    u32 length = 0;
    char quote = (*text == '"' || *text == '\'') ? *text : 0;
    if (!quote) {
        if (*text == '[' || *text == '{') {
            return false;
        }
        snprintf(out, out_size, "%s", text);
        return strlen(text) < out_size;
    }
    const char *c = text + 1;
    for (;; c++) {
        if (*c == 0) {
            return false; // unterminated
        }
        char next = *c;
        if (quote == '"' && *c == '\\' && (c[1] == '"' || c[1] == '\\')) {
            next = *++c;
        } else if (quote == '\'' && *c == '\'' && c[1] == '\'') {
            next = *++c;
        } else if (*c == quote) {
            break;
        }
        if (length + 1 >= out_size) {
            return false;
        }
        out[length++] = next;
    }
    out[length] = 0;
    c++;
    while (*c == ' ') {
        c++;
    }
    return *c == 0;
}

// true or false, as ROS 2 (yaml-cpp) also spells them.
static bool parse_flag(const char *text, f64 *value)
{
    const char *yes[] = {"true", "True", "TRUE"};
    const char *no[] = {"false", "False", "FALSE"};
    for (u32 i = 0; i < ARRAY_COUNT(yes); i++) {
        if (strcmp(text, yes[i]) == 0 || strcmp(text, no[i]) == 0) {
            *value = strcmp(text, yes[i]) == 0 ? 1.0 : 0.0;
            return true;
        }
    }
    return false;
}

// "1.5" or "[1, 2.5, -3]", and nothing else.
static bool parse_numbers(const char *text, f64 *values, u32 capacity, u32 *count)
{
    *count = 0;
    bool list = *text == '[';
    const char *cursor = list ? text + 1 : text;
    while (*cursor == ' ') {
        cursor++;
    }
    if (list && *cursor == ']') {
        cursor++; // empty list
    } else {
        for (;;) {
            char *end;
            f64 value = strtod(cursor, &end);
            if (end == cursor || *count == capacity) {
                return false;
            }
            values[(*count)++] = value;
            cursor = end;
            while (*cursor == ' ') {
                cursor++;
            }
            if (!list) {
                break;
            }
            if (*cursor == ']') {
                cursor++;
                break;
            }
            if (*cursor != ',') {
                return false;
            }
            cursor++;
        }
    }
    while (*cursor == ' ') {
        cursor++;
    }
    return *cursor == 0;
}

bool load_config_text(Sim_Config *config, const char *text, const char *name, u32 *unknown_keys,
                      char *error, u32 error_size)
{
    Yaml_Level levels[CONFIG_MAX_DEPTH];
    u32 depth = 0;
    u32 unknown = 0;
    u32 line_number = 0;
    const char *cursor = text;
    while (*cursor) {
        line_number++;
        const char *line_end = strchr(cursor, '\n');
        u32 length = line_end ? (u32)(line_end - cursor) : (u32)strlen(cursor);
        char line[512];
        if (length >= sizeof(line)) {
            snprintf(error, error_size, "%s:%u: line too long", name, line_number);
            return false;
        }
        memcpy(line, cursor, length);
        line[length] = 0;
        cursor += length + (line_end ? 1 : 0);

        strip_comment(line);
        u32 indent = 0;
        while (line[indent] == ' ') {
            indent++;
        }
        if (line[indent] == '\t') {
            snprintf(error, error_size, "%s:%u: indent with spaces, not tabs", name, line_number);
            return false;
        }
        char *content = trim_text(line + indent);
        if (!*content) {
            continue;
        }
        if (*content == '-') {
            snprintf(error, error_size, "%s:%u: write lists inline, as [a, b, c]", name,
                     line_number);
            return false;
        }
        char *colon = strchr(content, ':');
        if (!colon || colon == content) {
            snprintf(error, error_size, "%s:%u: expected \"key: value\"", name, line_number);
            return false;
        }
        *colon = 0;
        char *key = trim_text(content);
        char *value = trim_text(colon + 1);
        if (strlen(key) >= sizeof(levels[0].key)) {
            snprintf(error, error_size, "%s:%u: key too long", name, line_number);
            return false;
        }

        while (depth > 0 && levels[depth - 1].indent >= indent) {
            depth--;
        }
        if (!*value) {
            if (depth == CONFIG_MAX_DEPTH) {
                snprintf(error, error_size, "%s:%u: nested too deeply", name, line_number);
                return false;
            }
            levels[depth].indent = indent;
            snprintf(levels[depth].key, sizeof(levels[depth].key), "%s", key);
            depth++;
            continue;
        }

        u32 first = 0;
        for (u32 i = 0; i < depth; i++) {
            if (strcmp(levels[i].key, "ros__parameters") == 0) {
                first = i + 1;
            }
        }
        // Every part is shorter than a level key, so this can hold the deepest path.
        char full_key[(CONFIG_MAX_DEPTH + 1) * sizeof(levels[0].key)];
        u32 used = 0;
        for (u32 i = first; i < depth; i++) {
            used += (u32)snprintf(full_key + used, sizeof(full_key) - used, "%s.", levels[i].key);
        }
        snprintf(full_key + used, sizeof(full_key) - used, "%s", key);

        const Config_Field *field = find_config_field(full_key);
        if (!field) {
            // use_sim_time is a standard ROS 2 parameter that may share the file.
            if (strcmp(full_key, "use_sim_time") != 0) {
                log_warning("%s:%u: %s is not a Regolith setting; ignored", name, line_number,
                            full_key);
                unknown++;
            }
            continue;
        }
        char message[256];
        if (field->type == CONFIG_STRING) {
            char value_text[CONFIG_STRING_SIZE];
            if (!parse_text(value, value_text, sizeof(value_text))) {
                snprintf(error, error_size, "%s:%u: %s: expected text on one line", name,
                         line_number, full_key);
                return false;
            }
            if (!set_config_text(config, full_key, value_text, message, sizeof(message))) {
                snprintf(error, error_size, "%s:%u: %s", name, line_number, message);
                return false;
            }
            continue;
        }
        if (field->type == CONFIG_BOOL) {
            f64 flag = 0.0;
            if (!parse_flag(value, &flag)) {
                snprintf(error, error_size, "%s:%u: %s: expected true or false", name, line_number,
                         full_key);
                return false;
            }
            if (!set_config(config, full_key, &flag, 1, message, sizeof(message))) {
                snprintf(error, error_size, "%s:%u: %s", name, line_number, message);
                return false;
            }
            continue;
        }
        f64 values[CONFIG_MAX_COUNT];
        u32 count;
        if (!parse_numbers(value, values, CONFIG_MAX_COUNT, &count)) {
            snprintf(error, error_size, "%s:%u: %s: expected a number or [numbers] on one line",
                     name, line_number, full_key);
            return false;
        }
        if (!set_config(config, full_key, values, count, message, sizeof(message))) {
            snprintf(error, error_size, "%s:%u: %s", name, line_number, message);
            return false;
        }
    }
    if (unknown_keys) {
        *unknown_keys = unknown;
    }
    return true;
}

bool load_config_file(Sim_Config *config, const char *path, u32 *unknown_keys, char *error,
                      u32 error_size)
{
    FILE *file = fopen(path, "rb");
    if (!file) {
        snprintf(error, error_size, "%s: %s", path, strerror(errno));
        return false;
    }
    char *text = (char *)malloc(CONFIG_MAX_FILE + 1);
    u64 size = fread(text, 1, CONFIG_MAX_FILE + 1, file);
    fclose(file);
    if (size > CONFIG_MAX_FILE) {
        free(text);
        snprintf(error, error_size, "%s: larger than %llu bytes", path,
                 (unsigned long long)CONFIG_MAX_FILE);
        return false;
    }
    text[size] = 0;
    bool loaded = load_config_text(config, text, path, unknown_keys, error, error_size);
    free(text);
    return loaded;
}
