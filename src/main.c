#include "linalg.h"
#include "glad/glad.h"
#include <SDL.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#define PI 3.14159265358979

typedef unsigned int uint;

static inline float clamp(float x, float low, float high) {
    if (x < low) {
        return low;
    } else if (x > high) {
        return high;
    }
    return x;
}

enum {
    INPUT_FORWARD, INPUT_BACKWARD, INPUT_LEFT, INPUT_RIGHT,
    INPUT_X, INPUT_Y, INPUT_Z, INPUT_SHIFT,
    N_INPUTS
};

static bool inputs[N_INPUTS];

static char* read_file(const char* name) {
    FILE* f = fopen(name, "r");
    if (!f) {
        return NULL;
    }
    fseek(f, 0, SEEK_END);
    long len = ftell(f);
    fseek(f, 0, SEEK_SET);
    char* str = malloc(len + 1);
    if (!str) {
        fclose(f);
        return NULL;
    }
    size_t l, read_len = 0;
    while ((l = fread(str + read_len, 1, len - read_len, f))) {
        read_len += l;
    }
    fclose(f);
    str[read_len] = '\0';
    return str;
}

static GLuint load_shader(GLenum type, const char* file_name) {
    char* content = read_file(file_name);
    if (content == NULL) {
        fprintf(stderr, "Could not read shader file %s\n", file_name);
        return 0;
    }
    GLuint shader = glCreateShader(type);
    glShaderSource(shader, 1, &content, NULL);
    free(content);
    content = NULL;
    glCompileShader(shader);
    GLint status = 0;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &status);
    if (status == GL_FALSE) {
        GLint log_size = 0;
        glGetShaderiv(shader, GL_INFO_LOG_LENGTH, &log_size);
        char* log = malloc(log_size);
        if (log) {
            glGetShaderInfoLog(shader, log_size, NULL, log);
            fprintf(stderr, "Error compiling %s:\n%s\n", file_name, log);
            free(log);
            return 0;
        }
    }
    return shader;
}

static GLuint load_shaders(const char* name) {
    char file_name[512] = {0};
    static const char base_path[] = "res/";
    size_t name_len = strlen(name);
    size_t base_len = sizeof base_path - 1;
    strncpy(file_name, base_path, 511);
    strncpy(file_name + base_len, name, 511 - base_len);
    strncpy(file_name + base_len + name_len, ".vert",
            511 - base_len - name_len);
    GLuint vert = load_shader(GL_VERTEX_SHADER, file_name);
    strncpy(file_name + base_len + name_len, ".frag",
            511 - base_len - name_len);
    GLuint frag = load_shader(GL_FRAGMENT_SHADER, file_name);

    GLuint program = 0;
    if (vert && frag) {
        program = glCreateProgram();
        glAttachShader(program, vert);
        glAttachShader(program, frag);
        glLinkProgram(program);
        int status = 0;
        glGetProgramiv(program, GL_LINK_STATUS, &status);
        if (status == GL_FALSE) {
            GLint log_size = 0;
            glGetProgramiv(program, GL_INFO_LOG_LENGTH, &log_size);
            char* log = malloc(log_size);
            if (log) {
                glGetProgramInfoLog(program, log_size, NULL, log);
                fprintf(stderr, "Error linking %s:\n%s\n", name, log);
                free(log);
                program = 0;
            }
        }
        glDetachShader(program, vert);
        glDetachShader(program, frag);
    }
    glDeleteShader(vert);
    glDeleteShader(frag);
    return program;
}

typedef struct {
    Vec3 pos;
    Quat rot;
    Vec3 scale;
} Transform;

typedef struct {
    Vec3 pos;
    float pitch, yaw;
} Camera;

static Transform default_transform() {
    return (Transform){
        .pos = vec3(0, 0, 0),
        .rot = quat(1, 0, 0, 0),
        .scale = vec3(1, 1, 1),
    };
}

static void translate_local(Transform* t, Vec3 v) {
    t->pos = vec_add(t->pos, mat_vec_mul(quat_to_mat(t->rot), v));
}

static void translate_local_camera(Camera* cam, Vec3 v) {
    float c = cosf(cam->yaw);
    float s = sinf(cam->yaw);
    cam->pos.x += v.x * c - v.y * s;
    cam->pos.y += v.y * c + v.x * s;
    cam->pos.z += v.z;
}

static char* after_char(char* str, char ch) {
    while (*str != ch) {
        if (*str == '\0') {
            return NULL;
        }
        str++;
    }
    str++;
    return str;
}
static void set_vert(float* f, char* str,
                     float* verts, size_t n_verts,
                     float* texs, size_t n_texs,
                     float* norms, size_t n_norms) {
    size_t i;
    i = atoi(str) - 1;
    if (i < n_verts) {
        f[0] = verts[i*3];
        f[1] = verts[i*3+1];
        f[2] = verts[i*3+2];
    } else {
        f[0] = f[1] = f[2] = 0;
    }
    str = after_char(str, '/');
    if (str != NULL) {
        i = atoi(str) - 1;
        if (i < n_texs) {
            f[3] = texs[i*2];
            f[4] = texs[i*2+1];
        } else {
            f[3] = f[4] = 0;
        }
        str = after_char(str, '/');
        if (str != NULL) {
            i = atoi(str) - 1;
            if (i < n_norms) {
                f[5] = norms[i*3];
                f[6] = norms[i*3+1];
                f[7] = norms[i*3+2];
            }
        } else {
            f[5] = f[6] = f[7] = 0;
        }
    } else {
        f[3] = f[4] = f[5] = f[6] = f[7] = 0;
    }
}
static bool read_obj_file(const char* name,
                          float** faces, size_t* n_faces) {
    char file_name[512] = {0};
    static const char base_path[] = "res/";
    size_t name_len = strlen(name);
    size_t base_len = sizeof base_path - 1;
    strncpy(file_name, base_path, 511);
    strncpy(file_name + base_len, name, 511 - base_len);
    strncpy(file_name + base_len + name_len, ".obj",
            511 - base_len - name_len);

    *faces = NULL;
    *n_faces = 0;
    char* content = read_file(file_name);
    if (!content) {
        fprintf(stderr, "Could not read obj file %s\n", file_name);
        return false;
    }
    char* c = content;
    float* verts = NULL;
    size_t n_verts = 0;
    float* texs = NULL;
    size_t n_texs = 0;
    float* norms = NULL;
    size_t n_norms = 0;
    while (c[0] != '\0') {
        if (c[0] != '#') {
            char cmd[8] = {0}, arg1[16] = {0}, arg2[16] = {0}, arg3[16] = {0};
            sscanf(c, "%7s %15s %15s %15s", cmd, arg1, arg2, arg3);
            if (strcmp(cmd, "v") == 0) {
                verts = realloc(verts, (n_verts+1)*3 * sizeof (float));
                verts[n_verts*3] = atof(arg1);
                verts[n_verts*3+1] = atof(arg2);
                verts[n_verts*3+2] = atof(arg3);
                n_verts++;
            } else if (strcmp(cmd, "vt") == 0) {
                texs = realloc(texs, (n_texs+1)*2 * sizeof (float));
                texs[n_texs*2] = atof(arg1);
                texs[n_texs*2+1] = atof(arg2);
                n_texs++;
            } else if (strcmp(cmd, "vn") == 0) {
                norms = realloc(norms, (n_norms+1)*3 * sizeof (float));
                norms[n_norms*3] = atof(arg1);
                norms[n_norms*3+1] = atof(arg2);
                norms[n_norms*3+2] = atof(arg3);
                n_norms++;
            } else if (strcmp(cmd, "f") == 0) {
                *faces = realloc(*faces, (*n_faces+1)*24 * sizeof (float));
                float* f = &(*faces)[*n_faces * 24];
                set_vert(f, arg1, verts, n_verts, texs, n_texs, norms, n_norms);
                set_vert(f+8, arg2, verts, n_verts, texs, n_texs, norms, n_norms);
                set_vert(f+16, arg3, verts, n_verts, texs, n_texs, norms, n_norms);
                if (f[5] == 0 && f[6] == 0 && f[7] == 0) {
                    Vec3 v1 = vec3(f[0], f[1], f[2]);
                    Vec3 v2 = vec3(f[8], f[9], f[10]);
                    Vec3 v3 = vec3(f[16], f[17], f[18]);
                    Vec3 norm = vec_cross(vec_to(v1, v2), vec_to(v1, v3));
                    norm = vec_norm(norm);
                    f[5] = f[13] = f[21] = norm.x;
                    f[6] = f[14] = f[22] = norm.y;
                    f[7] = f[15] = f[23] = norm.z;
                }
                (*n_faces)++;
            }
        }
        while (c[0] != '\0' && c[0] != '\n') {
            c++;
        }
        while (c[0] == '\n') {
            c++;
        }
    }
    free(content);
    free(norms);
    free(verts);
    free(texs);
    return true;
}

typedef struct {
    uint vao;
    GLuint shader;
    GLint loc_model;
    GLint loc_view;
    GLint loc_proj;
    GLint loc_color;
    GLuint n_verts;
    GLuint texture;
    GLenum mode;
} Obj;

bool obj_setup(Obj* obj, GLuint shader, const char* texname,
               float* data, size_t n_data, GLenum mode) {
    bool use_texture = texname != NULL;
    size_t stride = 6;
    if (use_texture) {
        stride += 2;
    }

    glGenVertexArrays(1, &obj->vao);
    glBindVertexArray(obj->vao);

    uint vbo;
    glGenBuffers(1, &vbo);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, n_data * stride * sizeof *data, data,
                 GL_STATIC_DRAW);

    if (use_texture) {
        glGenTextures(1, &obj->texture);
        glBindTexture(GL_TEXTURE_2D, obj->texture);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER,
                        GL_LINEAR_MIPMAP_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexStorage2D(GL_TEXTURE_2D, 1, GL_RGBA8, 256, 256);
        SDL_Surface* surf = SDL_LoadBMP(texname);
        if (surf == NULL) {
            return false;
        }
        glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, 256, 256, GL_BGR,
                        GL_UNSIGNED_BYTE, surf->pixels);
        SDL_FreeSurface(surf);
        glGenerateMipmap(GL_TEXTURE_2D);
    }

    float* offset = 0;
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, stride * sizeof *data,
                          offset);
    glEnableVertexAttribArray(0);
    offset += 3;
    if (use_texture) {
        glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, stride * sizeof *data,
                              offset);
        glEnableVertexAttribArray(1);
        offset += 2;
    }
    glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, stride * sizeof *data,
                          offset);
    glEnableVertexAttribArray(2);
    offset += 3;

    obj->shader = shader;
    obj->loc_model = glGetUniformLocation(shader, "model");
    obj->loc_view = glGetUniformLocation(shader, "view");
    obj->loc_proj = glGetUniformLocation(shader, "proj");
    obj->loc_color = glGetUniformLocation(shader, "color");
    obj->n_verts = n_data;
    obj->mode = mode;
    return true;
}

static Obj new_rect(GLuint shader) {
    float ts = 16;
    float verts[] = {
        -1, -1, 0, 0, 0, 0, 0, 1,
        1, -1, 0, 1*ts, 0, 0, 0, 1,
        -1, 1, 0, 0, 1*ts, 0, 0, 1,
        1, 1, 0, 1*ts, 1*ts, 0, 0, 1,
    };

    Obj obj = {0};
    obj_setup(&obj, shader, "res/grass.bmp", verts, 4, GL_TRIANGLE_STRIP);
    return obj;
}

static Obj new_obj(GLuint shader, const char* file_name) {
    float* faces;
    size_t n_faces;
    if (!read_obj_file(file_name, &faces, &n_faces)) {
        return (Obj){0};
    }

    Obj obj = {0};
    obj_setup(&obj, shader, "res/wood.bmp", faces, n_faces * 3, GL_TRIANGLES);

    free(faces);

    return obj;
}

static void render_obj(const Obj* o, float color[4],
                       float* model, float* view, float* proj) {
    glUseProgram(o->shader);
    glBindVertexArray(o->vao);
    glBindTexture(GL_TEXTURE_2D, o->texture);
    glUniform4fv(o->loc_color, 1, color);
    glUniformMatrix4fv(o->loc_model, 1, GL_TRUE, model);
    glUniformMatrix4fv(o->loc_view, 1, GL_TRUE, view);
    glUniformMatrix4fv(o->loc_proj, 1, GL_TRUE, proj);
    glDrawArrays(o->mode, 0, o->n_verts);
}

typedef struct {
    int w, h;
    SDL_Window* window;
    SDL_GLContext context;
} Window;

bool create_window(Window* window, int width, int height, const char* title) {
    int x = SDL_WINDOWPOS_UNDEFINED, y = SDL_WINDOWPOS_UNDEFINED;
    if (SDL_Init(SDL_INIT_VIDEO) < 0) {
        return false;
    }
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 3);
    int flags = SDL_WINDOW_RESIZABLE | SDL_WINDOW_OPENGL;
    window->window = SDL_CreateWindow(title, x, y, width, height, flags);
    if (!window->window) {
        return false;
    }
    window->w = width;
    window->h = height;
    window->context = SDL_GL_CreateContext(window->window);
    if (!window->context) {
        return false;
    }
    if (!gladLoadGLLoader((GLADloadproc)SDL_GL_GetProcAddress)) {
        return false;
    }
    return true;
}

void destroy_window(Window* window) {
    SDL_GL_DeleteContext(window->context);
    window->context = 0;
    SDL_DestroyWindow(window->window);
    window->window = NULL;
    window->w = 0;
    window->h = 0;
    SDL_Quit();
}

int main(void) {
    Window window;
    if (!create_window(&window, 852, 480, "Hello")) {
        return 1;
    }

    SDL_SetRelativeMouseMode(true);
    GLuint shader_tex = load_shaders("shader_tex");
    GLuint shader_plain = load_shaders("shader_plain");
    glViewport(0, 0, window.w, window.h);
    glClearColor(0.3, 0.5, 0.7, 1);
    Mat4 view;
    float ratio_hw = (float)window.h / window.w;
    float clip_near = 0.01, clip_far = 300;
    float fov = 60;
    Mat4 proj = mat_from_persp(fov*PI/180, ratio_hw, clip_near, clip_far);
    glEnable(GL_DEPTH_TEST);
    Obj house = new_obj(shader_tex, "house");
    Obj ball = new_obj(shader_plain, "ball");
    Obj rect = new_rect(shader_tex);
    Transform fly_camera = default_transform();
    fly_camera.pos.z = 1.6;
    fly_camera.rot = quat_from_rot(vec3(PI*0.2, 0, 0));
    bool flying = false;
    bool recording = false;
    FILE* moviefile = NULL;
    Camera camera = {0};
    camera.pos = vec3(7, 5, 1.7);
    camera.pitch = PI/2;
    camera.yaw = PI * 0.65f;
    float delta_time = 0;
    int prev_tick = 0;
    bool running = true;
    while (running) {
        SDL_Event event;
        while (SDL_PollEvent(&event)) {
            switch (event.type) {
            case SDL_QUIT:
                running = false;
                break;
            case SDL_WINDOWEVENT:
                switch (event.window.event) {
                case SDL_WINDOWEVENT_SIZE_CHANGED:
                    window.w = event.window.data1;
                    window.h = event.window.data2;
                    glViewport(0, 0, window.w, window.h);
                    ratio_hw = (float)window.h / window.w;
                    proj = mat_from_persp(fov*PI/180, ratio_hw,
                                          clip_near, clip_far);
                    break;
                }
                break;
            case SDL_MOUSEMOTION:
                if (flying) {
                    fly_camera.rot = quat_mul(fly_camera.rot,
                                              quat_from_rot(vec3(
                        event.motion.yrel * -0.003,
                        event.motion.xrel * -0.003,
                        0
                    )));
                } else {
                    camera.pitch += event.motion.yrel * -0.003;
                    camera.yaw += event.motion.xrel * -0.003;
                    camera.pitch = clamp(camera.pitch, 0, PI);
                }
                break;
            case SDL_KEYDOWN:
                switch (event.key.keysym.sym) {
                case SDLK_a:
                    inputs[INPUT_LEFT] = true;
                    break;
                case SDLK_s:
                    inputs[INPUT_RIGHT] = true;
                    break;
                case SDLK_w:
                    inputs[INPUT_FORWARD] = true;
                    break;
                case SDLK_r:
                    inputs[INPUT_BACKWARD] = true;
                    break;
                case SDLK_RSHIFT:
                case SDLK_LSHIFT:
                    inputs[INPUT_SHIFT] = true;
                    break;
                case SDLK_x:
                    inputs[INPUT_X] = true;
                    break;
                case SDLK_y:
                    inputs[INPUT_Y] = true;
                    break;
                case SDLK_z:
                    inputs[INPUT_Z] = true;
                    break;
                }
                break;
            case SDL_KEYUP:
                switch (event.key.keysym.sym) {
                case SDLK_a:
                    inputs[INPUT_LEFT] = false;
                    break;
                case SDLK_s:
                    inputs[INPUT_RIGHT] = false;
                    break;
                case SDLK_w:
                    inputs[INPUT_FORWARD] = false;
                    break;
                case SDLK_r:
                    inputs[INPUT_BACKWARD] = false;
                    break;
                case SDLK_RSHIFT:
                case SDLK_LSHIFT:
                    inputs[INPUT_SHIFT] = false;
                    break;
                case SDLK_x:
                    inputs[INPUT_X] = false;
                    break;
                case SDLK_y:
                    inputs[INPUT_Y] = false;
                    break;
                case SDLK_z:
                    inputs[INPUT_Z] = false;
                    break;
                case SDLK_ESCAPE:
                    SDL_SetRelativeMouseMode(!SDL_GetRelativeMouseMode());
                    break;
                case SDLK_SPACE:
                    flying = !flying;
                    break;
                case SDLK_F12:
                    if (inputs[INPUT_SHIFT]) {
                        if (recording) {
                            fclose(moviefile);
                        } else {
                            moviefile = fopen("movie.raw", "w");
                        }
                        recording = !recording;
                    } else {
                        size_t bypp = 3;
                        size_t size = window.w * window.h * bypp;
                        char* buffer = malloc(size);
                        glReadPixels(0, 0, window.w, window.h, GL_RGB,
                                     GL_UNSIGNED_BYTE, buffer);
                        FILE* f = fopen("scrot.ppm", "w");
                        char header[] = "P6\n                255\n";
                        sprintf(header+3, "%d %d", window.w, window.h);
                        fwrite(header, 1, sizeof header - 1, f);
                        for (int i = size - window.w*bypp; i >= 0;
                             i -= window.w * bypp) {
                            fwrite(buffer+i, bypp, window.w, f);
                        }
                        fclose(f);
                        free(buffer);
                        break;
                    }
                }
                break;
            }
        }

        float fly_speed = 10;
        float walk_speed = 4;
        Vec3 direction = {0};
        if (flying) {
            if (inputs[INPUT_LEFT]) {
                fly_camera.rot = quat_mul(fly_camera.rot, quat_from_rot(vec3(
                    0, 0, 0.03
                )));
            }
            if (inputs[INPUT_RIGHT]) {
                fly_camera.rot = quat_mul(fly_camera.rot, quat_from_rot(vec3(
                    0, 0, -0.03
                )));
            }
            if (inputs[INPUT_FORWARD]) {
                direction = vec_add(direction,
                                    vec3(0, 0, -fly_speed * delta_time));
            }
            if (inputs[INPUT_BACKWARD]) {
                direction = vec_add(direction,
                                    vec3(0, 0, fly_speed * delta_time));
            }
            translate_local(&fly_camera, vec_to_circular(direction));
        } else {
            if (inputs[INPUT_LEFT]) {
                direction = vec_add(direction,
                                    vec3(-walk_speed * delta_time, 0, 0));
            }
            if (inputs[INPUT_RIGHT]) {
                direction = vec_add(direction,
                                    vec3(walk_speed * delta_time, 0, 0));
            }
            if (inputs[INPUT_FORWARD]) {
                direction = vec_add(direction,
                                    vec3(0, walk_speed * delta_time, 0));
            }
            if (inputs[INPUT_BACKWARD]) {
                direction = vec_add(direction,
                                    vec3(0, -walk_speed * delta_time, 0));
            }
            translate_local_camera(&camera, vec_to_circular(direction));
        }

        if (inputs[INPUT_X]) {
            if (inputs[INPUT_SHIFT]) {
                camera.pos.x -= delta_time;
            } else {
                camera.pos.x += delta_time;
            }
        }
        if (inputs[INPUT_Y]) {
            if (inputs[INPUT_SHIFT]) {
                camera.pos.y -= delta_time;
            } else {
                camera.pos.y += delta_time;
            }
        }
        if (inputs[INPUT_Z]) {
            if (inputs[INPUT_SHIFT]) {
                camera.pos.z -= delta_time;
            } else {
                camera.pos.z += delta_time;
            }
        }

        Mat4 view_pos, view_rot;
        if (flying) {
            view_pos = mat_from_pos(vec_neg(fly_camera.pos));
            view_rot = quat_to_mat(quat_neg(fly_camera.rot));
        } else {
            view_pos = mat_from_pos(vec_neg(camera.pos));
            view_rot = mat_mul(quat_to_mat(quat_from_rot(vec3(
                    -camera.pitch, 0, 0
                ))),
                quat_to_mat(quat_from_rot(vec3(
                    0, 0, -camera.yaw
                )))
            );
        }
        view = mat_mul(view_rot, view_pos);

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        float white[] = {1, 1, 1, 1};
        render_obj(&rect, white,
                   mat_from_scale(vec3(80, 80, 1)).v, view.v, proj.v);
        render_obj(&house, white, mat_identity().v, view.v, proj.v);
        render_obj(&ball, white,
                   mat_mul(mat_from_pos(vec3(4, 0, 0)),
                           mat_from_scale(vec3(0.5, 0.5, 0.5))).v,
                   view.v, proj.v);

        SDL_GL_SwapWindow(window.window);

        if (recording && moviefile != NULL) {
            size_t bypp = 3;
            size_t size = window.w * window.h * bypp;
            char* buffer = malloc(size);
            glReadPixels(0, 0, window.w, window.h, GL_RGB,
                         GL_UNSIGNED_BYTE, buffer);
            for (int i = size - window.w*bypp; i >= 0; i -= window.w * bypp) {
                fwrite(buffer+i, bypp, window.w, moviefile);
            }
            free(buffer);
        }

        int ticks = SDL_GetTicks();
        delta_time = (ticks - prev_tick) * 0.001;
        prev_tick = ticks;
        int fps = 60;
        SDL_Delay(prev_tick+1000/fps-ticks);
    }
    destroy_window(&window);
    return 0;
}
